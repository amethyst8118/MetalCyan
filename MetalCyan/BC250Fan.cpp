// ASRock BC-250 NCT6686D fan monitoring + opt-in PWM control
//
// Copyright © 2026 NootedRed-BC250 contributors. Licensed under the Thou Shalt Not Profit License version 1.5.
// See LICENSE for details.
//
// Register facts only (ports, addresses, bits, ID values, byte order); no code copied from Fred78290/nct6687d
// or Linux nct6683 (both GPL-2.0). UNTESTED on the board.

#include <BC250Fan.hpp>
#include <BC250.hpp>
#include <Headers/kern_util.hpp>
#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <architecture/i386/pio.h>
#include <libkern/c++/OSArray.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSNumber.h>

namespace
{
    // Super I/O config ports, probed in order.
    constexpr UInt16 kSioPorts[] {0x2E, 0x4E};
    constexpr UInt8  kSioEnter = 0x87, kSioExit = 0xAA;
    constexpr UInt8  kSioIdHi = 0x20, kSioIdLo = 0x21, kSioLdn = 0x07, kSioBaseHi = 0x60, kSioBaseLo = 0x61,
                     kSioEnable = 0x30;
    constexpr UInt16 kIdMask = 0xFFF0, kIdNct6686 = 0xD440;
    constexpr UInt8  kLdnHwm = 0x0B;

    // EC window: one page/index/data sequence per register, always under the lock.
    constexpr UInt8 kPageSelect = 0xFF;
    constexpr UInt8 kOffPage = 4, kOffIndex = 5, kOffData = 6;

    // EC-space registers (default mapping; the MSI-alt mapping does not apply to this board).
    constexpr UInt16 kRegRpmBase = 0x140;      // + 2i, 16-bit big-endian direct RPM
    constexpr UInt16 kRegPwmRead = 0x160;      // + i, 0-255 duty readback
    constexpr UInt16 kRegMonCfg  = 0x1A0;      // + i, per-monitor source descriptor (32 channels)
    constexpr UInt16 kRegMonVal  = 0x100;      // + 2i, 16-bit value behind descriptor i
    constexpr UInt16 kRegMode    = 0xA00;      // bit i = channel i manual
    constexpr UInt16 kRegCommand = 0xA01;      // 0x80 = open config, 0x40 = commit
    constexpr UInt16 kRegDuty    = 0xA28;      // + i, 0-255 duty write
    constexpr UInt16 kRegEngine  = 0xCF8;      // fan engine status
    constexpr UInt8  kCmdOpen = 0x80, kCmdCommit = 0x40;
    constexpr UInt8  kEngPhase = 1U << 3, kEngInvalid = 1U << 4, kEngCheckDone = 1U << 5, kEngLock = 1U << 6;

    // Monitor-source descriptor values that mark temperature inputs: AMD TSI in 0x90-0x9D, board thermistors
    // 14/15 at 0x08/0x09. Anything else (voltages etc.) is skipped.
    constexpr UInt8 kSrcTsiLo = 0x90, kSrcTsiHi = 0x9D, kSrcTherm14 = 0x08, kSrcTherm15 = 0x09;
    constexpr UInt32 kMonChannels = 32;

    constexpr UInt32 kStartDelayPolls = 60;    // same 60 s delay as the SMU tuning
    constexpr UInt32 kReassertPolls   = 10;    // re-assert mode+duty against EC reclaim
    constexpr UInt32 kZeroStrikesMax  = 3;
    constexpr SInt32 kGuardDeciC      = 850;   // Tctl >= 85 C forces 100% on controlled channels
    constexpr SInt32 kCurveLoDeciC    = 500, kCurveHiDeciC = 800;

    UInt32 bootArg(const char* name)
    {
        UInt32 value = 0;
        return PE_parse_boot_argn(name, &value, sizeof(value)) ? value : 0;
    }
}    // namespace

UInt8 BC250Fan::sioRead(UInt16 port, UInt8 reg)
{
    outb(port, reg);
    return inb(port + 1);
}

void BC250Fan::sioWrite(UInt16 port, UInt8 reg, UInt8 value)
{
    outb(port, reg);
    outb(port + 1, value);
}

void BC250Fan::sioExit(UInt16 port) { outb(port, kSioExit); }

UInt8 BC250Fan::ecRead(UInt16 addr)
{
    const UInt8 page = static_cast<UInt8>(addr >> 8), index = static_cast<UInt8>(addr & 0xFF);
    IOLockLock(this->lock);
    outb(this->ecBase + kOffPage, kPageSelect);
    outb(this->ecBase + kOffPage, page);
    outb(this->ecBase + kOffIndex, index);
    const UInt8 value = inb(this->ecBase + kOffData);
    IOLockUnlock(this->lock);
    return value;
}

UInt16 BC250Fan::ecRead16(UInt16 addr)
{
    const UInt8 hi = this->ecRead(addr), lo = this->ecRead(addr + 1);
    return static_cast<UInt16>((static_cast<UInt16>(hi) << 8) | lo);
}

void BC250Fan::ecWrite(UInt16 addr, UInt8 value)
{
    const UInt8 page = static_cast<UInt8>(addr >> 8), index = static_cast<UInt8>(addr & 0xFF);
    IOLockLock(this->lock);
    outb(this->ecBase + kOffPage, kPageSelect);
    outb(this->ecBase + kOffPage, page);
    outb(this->ecBase + kOffIndex, index);
    outb(this->ecBase + kOffData, value);
    IOLockUnlock(this->lock);
}

void BC250Fan::detect()
{
    UInt32 off = 1;
    if (PE_parse_boot_argn("bc250fan", &off, sizeof(off)) && off == 0) {
        this->disabled = true;
        BCLOG("BC250Fan", "disabled (bc250fan=0)");
        return;
    }
    this->lock = IOLockAlloc();
    if (this->lock == nullptr) {
        BCLOG("BC250Fan", "no lock; fan monitoring off");
        return;
    }
    for (const auto port : kSioPorts) {
        outb(port, kSioEnter);
        outb(port, kSioEnter);
        const UInt16 id = static_cast<UInt16>((static_cast<UInt16>(this->sioRead(port, kSioIdHi)) << 8) |
                                              this->sioRead(port, kSioIdLo));
        if ((id & kIdMask) != kIdNct6686) {
            this->sioExit(port);
            continue;
        }
        this->sioWrite(port, kSioLdn, kLdnHwm);
        const UInt16 base = static_cast<UInt16>((static_cast<UInt16>(this->sioRead(port, kSioBaseHi)) << 8) |
                                                this->sioRead(port, kSioBaseLo));
        if (base == 0 || base == 0xFFFF || base < 0x100 || (base & 0xF007) != 0) {
            BCLOG("BC250Fan", "SIO id 0x%04X at 0x%X but EC base 0x%04X implausible; ignoring", id, port, base);
            this->sioExit(port);
            continue;
        }
        const UInt8 enable = this->sioRead(port, kSioEnable);
        if ((enable & 0x01) == 0) {
            BCLOG("BC250Fan", "SIO id 0x%04X at 0x%X: EC access disabled, enabling it", id, port);
            this->sioWrite(port, kSioEnable, enable | 0x01);
        }
        // A base ending in 5 sits one window step off; align it down like the Linux probe.
        this->ecBase = ((base & 0x07) == 5) ? (base & 0xFFF8) : base;
        this->sioExit(port);
        this->present = true;
        BCLOG("BC250Fan", "NCT6686D (id 0x%04X) at SIO 0x%X, EC base 0x%X", id, port, this->ecBase);
        break;
    }
    if (!this->present) {
        BCLOG("BC250Fan", "NCT6686D not found (SIO 0x2E/0x4E); no fan telemetry");
        return;
    }
    // Monitor-descriptor walk: the first AMD TSI channel is the CPU, then up to two board thermistors.
    UInt32 found = 0;
    for (UInt32 i = 0; i < kMonChannels && found < kMaxTemps; i++) {
        const UInt8 src = this->ecRead(kRegMonCfg + i);
        const bool tsi   = src >= kSrcTsiLo && src <= kSrcTsiHi;
        const bool therm = src == kSrcTherm14 || src == kSrcTherm15;
        if ((found == 0 && tsi) || (found > 0 && therm)) { this->tempCh[found++] = i; }
    }
    // No TSI channel: still show board zones if any (they just sit after the missing CPU slot).
    if (found == 0) {
        for (UInt32 i = 0; i < kMonChannels && found < kMaxTemps; i++) {
            const UInt8 src = this->ecRead(kRegMonCfg + i);
            if (src == kSrcTherm14 || src == kSrcTherm15) { this->tempCh[found++] = i; }
        }
    }
    this->tempCount = found;
    if (found > 0) {
        BCLOG("BC250Fan", "temperature monitors: %u channel(s) (CPU %s)", found, this->tempCh[0] >= 0 ? "found" : "missing");
    }
    else {
        BCLOG("BC250Fan", "no temperature monitors identified; reporting RPM/PWM only");
    }
    this->fixedPct = bootArg("bc250fanpct");
    if (this->fixedPct != 0 && (this->fixedPct < 30 || this->fixedPct > 100)) {
        BCLOG("BC250Fan", "bc250fanpct=%u ignored (30-100)", this->fixedPct);
        this->fixedPct = 0;
    }
    const UInt32 curveArg = bootArg("bc250fancurve");
    if (curveArg == 1) { this->curve = true; }
    else if (curveArg != 0) {
        BCLOG("BC250Fan", "bc250fancurve=%u ignored (only 1)", curveArg);
    }
    const UInt32 minArg = bootArg("bc250fanmin");
    if (minArg != 0) {
        if (minArg >= 20 && minArg <= 100) { this->floorPct = minArg; }
        else { BCLOG("BC250Fan", "bc250fanmin=%u ignored (20-100, default 30)", minArg); }
    }
    if (this->fixedPct != 0 && this->curve) {
        this->curve = false;
        BCLOG("BC250Fan", "bc250fanpct=%u wins over bc250fancurve", this->fixedPct);
    }
    if (this->fixedPct != 0) { BCLOG("BC250Fan", "fixed duty %u%% requested (starts after 60 s)", this->fixedPct); }
    else if (this->curve) { BCLOG("BC250Fan", "curve requested (floor %u%%, 50 C -> 100%% at 80 C)", this->floorPct); }
}

bool BC250Fan::configOpen()
{
    // Open the fan-config phase: idle (no phase, no request) first, then ask and wait for in-phase + unlocked.
    for (UInt32 i = 0; i < 1000; i++) {
        if ((this->ecRead(kRegEngine) & kEngPhase) == 0 && (this->ecRead(kRegCommand) & kCmdOpen) == 0) { break; }
        if (i == 999) { return false; }
        IOSleep(1);
    }
    this->ecWrite(kRegCommand, kCmdOpen);
    for (UInt32 i = 0; i < 1000; i++) {
        const UInt8 st = this->ecRead(kRegEngine);
        if ((st & kEngLock) == 0 && (st & kEngPhase) != 0) { return true; }
        IOSleep(1);
    }
    return false;
}

bool BC250Fan::configCommit()
{
    this->ecWrite(kRegCommand, kCmdCommit);
    for (UInt32 i = 0; i < 1000; i++) {
        if ((this->ecRead(kRegEngine) & kEngCheckDone) != 0) { break; }
        if (i == 999) { return false; }
        IOSleep(1);
    }
    const UInt8 st = this->ecRead(kRegEngine);
    return (st & kEngInvalid) == 0 && (st & kEngLock) != 0;
}

bool BC250Fan::setDuty(UInt8 mask, UInt8 duty)
{
    if (!this->configOpen()) { return false; }
    this->ecWrite(kRegMode, this->ecRead(kRegMode) | mask);
    for (UInt32 i = 0; i < kChannels; i++) {
        if ((mask & (1U << i)) == 0) { continue; }
        this->ecWrite(kRegDuty + i, duty);
        if (this->ecRead(kRegDuty + i) != duty) {
            this->configCommit();
            return false;
        }
    }
    if (!this->configCommit()) { return false; }
    return this->verifyDuty(mask, duty);
}

bool BC250Fan::verifyDuty(UInt8 mask, UInt8)
{
    return (this->ecRead(kRegMode) & mask) == mask;
}

void BC250Fan::restoreFirmware()
{
    // Back to firmware control: clear our mode bits. No handshake on this path.
    this->ecWrite(kRegMode, this->ecRead(kRegMode) & ~this->mask);
}

void BC250Fan::failSafe(const char* reason)
{
    BCLOG("BC250Fan", "%s; firmware control restored, control off for this boot", reason);
    this->restoreFirmware();
    this->latchedOff = true;
    this->engaged    = false;
    this->lastDuty   = -1;
}

UInt8 BC250Fan::liveMask() const
{
    UInt8 mask = 0;
    for (UInt32 i = 0; i < kChannels; i++) {
        if (this->rpm[i] > 0) { mask |= (1U << i); }
    }
    return mask;
}

UInt32 BC250Fan::wantedPct(SInt32 tctlDeciC) const
{
    if (this->fixedPct != 0) { return this->fixedPct; }
    if (tctlDeciC <= kCurveLoDeciC) { return this->floorPct; }
    if (tctlDeciC >= kCurveHiDeciC) { return 100; }
    return this->floorPct +
           (100 - this->floorPct) * static_cast<UInt32>(tctlDeciC - kCurveLoDeciC) /
               static_cast<UInt32>(kCurveHiDeciC - kCurveLoDeciC);
}

void BC250Fan::poll(SInt32 tctlDeciC, bool tctlValid)
{
    if (!this->present || this->disabled) { return; }
    this->polls++;
    for (UInt32 i = 0; i < kChannels; i++) {
        this->rpm[i]    = this->ecRead16(kRegRpmBase + 2 * i);
        this->pwmRaw[i] = this->ecRead(kRegPwmRead + i);
    }
    for (UInt32 s = 0; s < this->tempCount; s++) {
        // Board value: 16-bit raw, raw/128*500 millidegrees.
        const UInt16 raw = this->ecRead16(kRegMonVal + 2 * static_cast<UInt16>(this->tempCh[s]));
        this->tempDeciC[s] = static_cast<SInt32>(raw) * 5 / 128;
    }
    if (this->latchedOff || (this->fixedPct == 0 && !this->curve)) { return; }
    if (this->polls < kStartDelayPolls || !tctlValid) { return; }
    if (!this->engaged) {
        const UInt8 mask = this->liveMask();
        if (mask == 0) {
            static bool waited = false;
            if (!waited) {
                waited = true;
                BCLOG("BC250Fan", "control waiting: no channel reports RPM yet");
            }
            return;
        }
        this->mask    = mask;
        this->engaged = true;
        BCLOG("BC250Fan", "taking control of channel(s) 0x%02X (duty %u%%)", mask, this->wantedPct(tctlDeciC));
    }
    for (UInt32 i = 0; i < kChannels; i++) {
        if ((this->mask & (1U << i)) == 0) { continue; }
        this->zeroStrikes[i] = this->rpm[i] == 0 ? this->zeroStrikes[i] + 1 : 0;
        if (this->zeroStrikes[i] >= kZeroStrikesMax) {
            this->failSafe("controlled channel stopped (0 RPM 3x)");
            return;
        }
    }
    UInt32 want = this->wantedPct(tctlDeciC);
    if (tctlDeciC >= kGuardDeciC) {
        if (want != 100) { BCLOG("BC250Fan", "Tctl %d.%d C >= 85 C, forcing 100%%", tctlDeciC / 10, tctlDeciC % 10); }
        want = 100;
    }
    const SInt32 duty = pctToDuty(want);
    // Fresh duty each poll only on change; the mode bit + duty are re-asserted every 10 s anyway in case the
    // EC reclaimed them (reclaim behaviour is board-only, so assume it can happen).
    if (duty != this->lastDuty || this->polls % kReassertPolls == 0) {
        if (!this->setDuty(this->mask, static_cast<UInt8>(duty))) {
            this->failSafe("EC handshake timed out or duty rejected");
            return;
        }
        this->lastDuty = duty;
    }
}

size_t BC250Fan::describeAppend(char* out, size_t capacity) const
{
    if (!this->present || this->disabled || capacity == 0) { return 0; }
    int n = snprintf(out, capacity, " | fans %u/%u/%u/%u/%u rpm pwm %u/%u/%u/%u/%u%%", this->rpm[0], this->rpm[1],
        this->rpm[2], this->rpm[3], this->rpm[4], rawToPct(this->pwmRaw[0]), rawToPct(this->pwmRaw[1]),
        rawToPct(this->pwmRaw[2]), rawToPct(this->pwmRaw[3]), rawToPct(this->pwmRaw[4]));
    if (this->tempCount > 0 && n > 0 && static_cast<size_t>(n) < capacity) {
        n += snprintf(out + n, capacity - n, " nct ");
        for (UInt32 s = 0; s < this->tempCount && n > 0 && static_cast<size_t>(n) < capacity; s++) {
            const SInt32 t = this->tempDeciC[s];
            n += snprintf(out + n, capacity - n, "%s%d.%d", s == 0 ? "" : "/", t / 10,
                t % 10 < 0 ? -(t % 10) : t % 10);
        }
        if (n > 0 && static_cast<size_t>(n) < capacity) { n += snprintf(out + n, capacity - n, "C"); }
    }
    if (n > 0 && static_cast<size_t>(n) < capacity) {
        if (this->latchedOff) { n += snprintf(out + n, capacity - n, " ctl off"); }
        else if (this->engaged && this->lastDuty >= 0) {
            n += snprintf(out + n, capacity - n, " ctl %u%%",
                (static_cast<UInt32>(this->lastDuty) * 100 + 127) / 255);
        }
    }
    return n > 0 ? static_cast<size_t>(n) : 0;
}

void BC250Fan::publish(OSDictionary* dict) const
{
    if (!this->present || this->disabled || dict == nullptr) { return; }
    if (auto* rpms = OSArray::withCapacity(kChannels)) {
        for (const auto rpm : this->rpm) {
            if (auto* number = OSNumber::withNumber(rpm, 32)) {
                rpms->setObject(number);
                number->release();
            }
        }
        dict->setObject("Fan Speed (RPM)", rpms);
        rpms->release();
    }
    if (auto* pwms = OSArray::withCapacity(kChannels)) {
        for (const auto raw : this->pwmRaw) {
            if (auto* number = OSNumber::withNumber(rawToPct(raw), 32)) {
                pwms->setObject(number);
                number->release();
            }
        }
        dict->setObject("Fan PWM (%)", pwms);
        pwms->release();
    }
    if (this->tempCount > 0) {
        if (auto* temps = OSArray::withCapacity(this->tempCount)) {
            for (UInt32 s = 0; s < this->tempCount; s++) {
                if (auto* number = OSNumber::withNumber(static_cast<UInt32>(this->tempDeciC[s]), 32)) {
                    temps->setObject(number);
                    number->release();
                }
            }
            dict->setObject("NCT Temp (0.1 C)", temps);
            temps->release();
        }
    }
}
