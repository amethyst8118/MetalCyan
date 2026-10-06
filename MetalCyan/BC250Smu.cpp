// ASRock BC-250 (AMD Cyan Skillfish) SMU access
//
// Copyright © 2026 NootedRed-BC250 contributors. Licensed under the Thou Shalt Not Profit License version 1.5.
// See LICENSE for details.
//
// Mailbox addresses, message IDs and encodings follow bc250_smu in bc250-collective/bc250_smu_oc (MIT), which
// documents the BC-250's SMU queues from its firmware's descriptor table.

#include <BC250.hpp>
#include <BC250HWL.hpp>
#include <BC250Smu.hpp>
#include <Headers/kern_util.hpp>
#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOService.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <NRed.hpp>
#include <Regs/CyanSkillfish.hpp>
#include <kern/thread.h>
#include <libkern/c++/OSArray.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSNumber.h>
#include <sys/sysctl.h>

static BC250Smu moduleInstance;

BC250Smu& BC250Smu::singleton() { return moduleInstance; }

namespace
{
    // Host bridge SMN index/data pair (00:00.0 config space).
    constexpr UInt32 kSmnIndex = 0xB8, kSmnData = 0xBC;

    // Per queue: command, response, argument (argument + 4 = high argument word).
    struct Queue
    {
        UInt32 cmd, rsp, arg;
    };
    constexpr Queue kQueues[] = {
        {0x03B10A08, 0x03B10A68, 0x03B10A48},
        {0x03B10A00, 0x03B10A60, 0x03B10A40},
        {0x03B10528, 0x03B10564, 0x03B10998},
        {0x03B10A20, 0x03B10A80, 0x03B10A88},
        {0x03B10A24, 0x03B10A84, 0x03B10A8C},
    };

    // Telemetry sends only these (queue << 8 | message). All of them only report state.
    constexpr UInt32 kAllowed[] = {
        0x0002,    // Q0 GetSmuVersion
        0x000C,    // Q0 QueryCorePstate (arg: core)
        0x000F,    // Q0 QueryGfxclk (MHz)
        0x001E,    // Q0 QueryActiveWgp
        0x0037,    // Q0 GetGfxFrequency (MHz)
        0x0038,    // Q0 GetGfxVid
        0x003D,    // Q0 GetEnabledSmuFeatures
        0x0301,    // Q3 test message: returns arg + 1
        0x0336,    // Q3 current CPU voltage (mV)
        0x0337,    // Q3 current GPU voltage (mV)
        0x033B,    // Q3 clock of a P-state (arg: 0-7)
        0x0340,    // Q3 CPU temperature limit (°C)
        0x0343,    // Q3 core frequency (arg: core 0-7, MHz)
    };

    // Tuning messages (queue << 8 | message); each argument is checked in sendTune.
    constexpr UInt32 kTuneCoreUnlock = 0x0398, kTuneCpuTemp = 0x038B, kTuneGpuTemp = 0x038C, kTuneExtraVolt = 0x039A,
                     kTuneVidScale = 0x0350, kTuneCpuBoost = 0x038F, kTuneGfxFreq = 0x0039, kTuneGfxUnforceFreq = 0x003A,
                     kTuneGfxVid = 0x003B, kTuneGfxUnforceVid = 0x003C;
    constexpr UInt32 kCoreUnlockSmn = 0x0005A870;    // What bc250-core-unlock reads and passes to 0x98 (never 0)
    constexpr UInt32 kCoreMaskStock = 0x77, kCoreMaskAll = 0xFF;
    constexpr UInt32 kCpuMinMHz = 3500, kCpuMaxMHz = 4500, kCpuVmin = 950, kCpuVmaxLimit = 1325, kCpuVmaxDefault = 1275;
    constexpr SInt32 kScaleMin = -50, kScaleMax = 0;
    constexpr UInt32 kTempMin = 50, kTempMax = 100, kCpuTempDefault = 90, kGpuGuardDefault = 90;
    // Hard limits. 2000 MHz is amdgpu's CYAN_SKILLFISH_SCLK_MAX; on the stock SMU firmware Q0 0x39 with 2230 MHz
    // never answered and the SMU stopped responding on every queue (CPU clocks froze) until a power cycle.
    // 1100 mV stays under amdgpu's 1129 mV CYAN_SKILLFISH_VDDC_MAX.
    constexpr UInt32 kGfxMinMHz = 350, kGfxMaxMHz = 2000, kGfxMinMv = 700, kGfxMaxMv = 1100;
    constexpr UInt32 kGfxCurveMv = 1000, kGfxUndervoltMax = 50;

    // GFX VID code: (1.55 V - v) / 6.25 mV, rounded.
    UInt32 milliVoltsToVid(UInt32 mv) { return mv >= 1550 ? 0 : ((1550 - mv) * 100 + 312) / 625; }

    // Default voltage for a clock without bc250gfxmv: the governor's safe points, linear from 350 MHz/700 mV to
    // 2000 MHz/1000 mV.
    UInt32 gfxSafeMilliVolts(UInt32 mhz)
    {
        return kGfxMinMv + (mhz - kGfxMinMHz) * (kGfxCurveMv - kGfxMinMv) / (kGfxMaxMHz - kGfxMinMHz);
    }

    // bc250_smu_oc's CPU VID model (bc250_detect.py): vid = 0.0003 f^2 + (-1.519 + 0.004325 s) f + 2800 - 10 s, in mV
    // and MHz. The scale for a voltage at a clock, rounded down (more undervolt); INT32_MIN if the clock can't reach it.
    SInt32 predictScale(UInt32 mhz, UInt32 vmax)
    {
        const SInt64 f = mhz;
        // Everything in 1e6 units: base = 300 f^2 - 1519000 f + 2800e6, slope = 4325 f - 10e6 (per scale step).
        const SInt64 base  = 300 * f * f - 1519000 * f + 2800000000LL;
        const SInt64 slope = 4325 * f - 10000000;
        if (slope <= 0) { return INT32_MIN; }
        const SInt64 num = static_cast<SInt64>(vmax) * 1000000 - base;
        SInt64       s   = num / slope;
        if (num < 0 && num % slope != 0) { s--; }
        return static_cast<SInt32>(s);
    }

    UInt32 bootArg(const char* name)
    {
        UInt32 value = 0;
        return PE_parse_boot_argn(name, &value, sizeof(value)) ? value : 0;
    }

    constexpr UInt32 kCoreMaskSmn = 0x0115A870;    // Core presence mask (SMU-owned)
    constexpr UInt32 kTctlSmn     = 0x00059800;    // THM_TCON_CUR_TMP (Zen: Tctl, 0.125 °C units in [31:21])

    UInt32 vidToMilliVolts(UInt32 vid) { return vid <= 248 ? 1550 - (vid * 625 + 50) / 100 : 0; }

    char   smuText[1024];
    size_t smuTextLen = 0;

    int sysctlHandleSmu(struct sysctl_oid*, void*, int, struct sysctl_req* req)
    {
        return SYSCTL_OUT(req, smuText, smuTextLen + 1);
    }
}    // namespace

SYSCTL_DECL(_debug_bc250);
SYSCTL_PROC(_debug_bc250, OID_AUTO, smu, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, nullptr, 0, sysctlHandleSmu, "A",
    "BC-250 SMU telemetry");

void bc250SmuThread(void* self, int) { static_cast<BC250Smu*>(self)->run(); }

void BC250Smu::start()
{
    UInt32 enabled = 1;
    PE_parse_boot_argn("bc250smu", &enabled, sizeof(enabled));
    if (this->started || enabled == 0) { return; }
    this->lock = IOLockAlloc();
    if (this->lock == nullptr) { return; }
    thread_t thread = nullptr;
    if (kernel_thread_start(reinterpret_cast<thread_continue_t>(bc250SmuThread), this, &thread) != KERN_SUCCESS) {
        return;
    }
    thread_deallocate(thread);
    this->started = true;
    sysctl_register_oid(&sysctl__debug_bc250_smu);
    BCLOG("BC250SMU", "telemetry thread started (bc250smu=0 disables)");
}

bool BC250Smu::findHostBridge()
{
    auto* matching = IOService::serviceMatching("IOPCIDevice");
    if (matching == nullptr) { return false; }
    auto* iter = IOService::getMatchingServices(matching);
    matching->release();
    if (iter == nullptr) { return false; }
    while (auto* object = iter->getNextObject()) {
        auto* pci = OSDynamicCast(IOPCIDevice, object);
        if (pci == nullptr || pci->getBusNumber() != 0 || pci->getDeviceNumber() != 0 || pci->getFunctionNumber() != 0) {
            continue;
        }
        if (pci->configRead16(kIOPCIConfigVendorID) != 0x1022) { continue; }
        pci->retain();
        this->hostBridge = pci;
        break;
    }
    iter->release();
    return this->hostBridge != nullptr;
}

UInt32 BC250Smu::smnRead(UInt32 address)
{
    this->hostBridge->configWrite32(kSmnIndex, address);
    return this->hostBridge->configRead32(kSmnData);
}

void BC250Smu::smnWrite(UInt32 address, UInt32 value)
{
    this->hostBridge->configWrite32(kSmnIndex, address);
    this->hostBridge->configWrite32(kSmnData, value);
}

UInt32 BC250Smu::send(UInt32 queue, UInt32 msg, UInt32 arg, UInt32* out)
{
    bool allowed = false;
    for (const auto id : kAllowed) { allowed |= id == ((queue << 8) | msg); }
    if (!allowed || queue >= arrsize(kQueues) || this->hostBridge == nullptr) { return kFailed; }
    const auto& q = kQueues[queue];
    IOLockLock(this->lock);
    // A queue whose last message has no response yet is busy: skip this poll instead of clobbering it.
    if (this->smnRead(q.rsp) == 0) {
        static bool logged[arrsize(kQueues)] {};
        if (!logged[queue]) {
            logged[queue] = true;
            BCLOG("BC250SMU", "Q%u busy (no response to its last message), polls skipped until it answers", queue);
        }
        IOLockUnlock(this->lock);
        return kTimeout;
    }
    this->smnWrite(q.rsp, 0);
    this->smnWrite(q.arg, arg);
    this->smnWrite(q.arg + 4, 0);
    this->smnWrite(q.cmd, msg);
    UInt32 status = kTimeout;
    for (UInt32 i = 0; i < 2000; i++) {    // up to ~20 ms
        status = this->smnRead(q.rsp);
        if (status != 0) { break; }
        IODelay(10);
    }
    if (out != nullptr) { *out = this->smnRead(q.arg); }
    IOLockUnlock(this->lock);
    return status;
}

UInt32 BC250Smu::sendTune(UInt32 queue, UInt32 msg, UInt32 arg)
{
    const UInt32 id = (queue << 8) | msg;
    bool         ok = false;
    switch (id) {
        case kTuneCoreUnlock: ok = arg == kCoreUnlockSmn; break;    // an argument of 0 hangs the SMU
        case kTuneCpuTemp:
        case kTuneGpuTemp: ok = arg >= kTempMin && arg <= kTempMax; break;
        case kTuneExtraVolt: ok = arg <= 1; break;
        case kTuneVidScale: {
            const SInt32 s = static_cast<SInt16>(arg & 0xFFFF);
            ok             = (arg >> 16) == 0 && s >= kScaleMin && s <= kScaleMax;
            break;
        }
        case kTuneCpuBoost: ok = arg >= kCpuMinMHz && arg <= kCpuMaxMHz; break;
        case kTuneGfxFreq: ok = arg >= kGfxMinMHz && arg <= kGfxMaxMHz; break;
        case kTuneGfxVid: ok = arg >= milliVoltsToVid(kGfxMaxMv) && arg <= milliVoltsToVid(kGfxMinMv); break;
        case kTuneGfxUnforceFreq:
        case kTuneGfxUnforceVid: ok = arg == 0; break;
        default: ok = false; break;
    }
    if (!ok || queue >= arrsize(kQueues) || this->hostBridge == nullptr) {
        BCLOG("BC250SMU", "tune: Q%u 0x%02X arg 0x%X refused (not allowlisted or out of range)", queue, msg, arg);
        return kFailed;
    }
    const auto& q = kQueues[queue];
    IOLockLock(this->lock);
    // As amdgpu: never write a command while the previous one has no response yet (that clobbers a message the
    // firmware is still running). Up to ~5 s each way, as bc250_smu's mailbox.
    UInt32 status = 0;
    for (UInt32 i = 0; i < 5000 && (status = this->smnRead(q.rsp)) == 0; i++) { IOSleep(1); }
    if (status == 0) {
        IOLockUnlock(this->lock);
        BCLOG("BC250SMU", "tune: Q%u 0x%02X arg 0x%X not sent: queue still busy", queue, msg, arg);
        return kTimeout;
    }
    BCLOG("BC250SMU", "tune: Q%u 0x%02X arg 0x%X sending", queue, msg, arg);
    this->smnWrite(q.rsp, 0);
    this->smnWrite(q.arg, arg);
    this->smnWrite(q.arg + 4, 0);
    this->smnWrite(q.cmd, msg);
    status = kTimeout;
    for (UInt32 i = 0; i < 5000; i++) {
        status = this->smnRead(q.rsp);
        if (status != 0) { break; }
        IOSleep(1);
    }
    IOLockUnlock(this->lock);
    BCLOG("BC250SMU", "tune: Q%u 0x%02X arg 0x%X -> status 0x%02X", queue, msg, arg, status);
    return status;
}

void BC250Smu::parseTune()
{
    auto& u = this->tune;
    u.cores = bootArg("bc250cores");
    if (u.cores != 0 && u.cores != 8) {
        BCLOG("BC250SMU", "tune: bc250cores=%u ignored (only 8)", u.cores);
        u.cores = 0;
    }
    u.cpuMHz = bootArg("bc250cpumhz");
    if (u.cpuMHz != 0 && (u.cpuMHz < kCpuMinMHz || u.cpuMHz > kCpuMaxMHz)) {
        BCLOG("BC250SMU", "tune: bc250cpumhz=%u ignored (%u-%u)", u.cpuMHz, kCpuMinMHz, kCpuMaxMHz);
        u.cpuMHz = 0;
    }
    u.cpuVmax = bootArg("bc250cpuvmax");
    if (u.cpuVmax == 0) { u.cpuVmax = kCpuVmaxDefault; }
    u.cpuVmax = u.cpuVmax < kCpuVmin ? kCpuVmin : u.cpuVmax > kCpuVmaxLimit ? kCpuVmaxLimit : u.cpuVmax;
    u.cpuTempC = bootArg("bc250cputemp");
    if (u.cpuTempC == 0 && u.cpuMHz != 0) { u.cpuTempC = kCpuTempDefault; }
    if (u.cpuTempC != 0) { u.cpuTempC = u.cpuTempC < kTempMin ? kTempMin : u.cpuTempC > kTempMax ? kTempMax : u.cpuTempC; }
    u.gpuTempC = bootArg("bc250gputemp");
    if (u.gpuTempC != 0) { u.gpuTempC = u.gpuTempC < kTempMin ? kTempMin : u.gpuTempC > kTempMax ? kTempMax : u.gpuTempC; }
    u.gfxMHz = bootArg("bc250gfxmhz");
    if (u.gfxMHz != 0 && (u.gfxMHz < kGfxMinMHz || u.gfxMHz > kGfxMaxMHz)) {
        BCLOG("BC250SMU", "tune: bc250gfxmhz=%u ignored (%u-%u)", u.gfxMHz, kGfxMinMHz, kGfxMaxMHz);
        u.gfxMHz = 0;
    }
    u.gfxMv = bootArg("bc250gfxmv");
    if (u.gfxMv != 0) { u.gfxMv = u.gfxMv < kGfxMinMv ? kGfxMinMv : u.gfxMv > kGfxMaxMv ? kGfxMaxMv : u.gfxMv; }
    // Too little voltage hangs the GPU, and on the BC-250 only a power cut clears that: an explicit voltage may sit
    // at most 50 mV under the safe-point curve for the clock.
    if (u.gfxMHz != 0 && u.gfxMv != 0 && u.gfxMv + kGfxUndervoltMax < gfxSafeMilliVolts(u.gfxMHz)) {
        BCLOG("BC250SMU", "tune: bc250gfxmv=%u too low for %u MHz, using %u mV", u.gfxMv, u.gfxMHz,
            gfxSafeMilliVolts(u.gfxMHz) - kGfxUndervoltMax);
        u.gfxMv = gfxSafeMilliVolts(u.gfxMHz) - kGfxUndervoltMax;
    }
    if (u.cores || u.cpuMHz || u.cpuTempC || u.gpuTempC || u.gfxMHz) {
        BCLOG("BC250SMU", "tune: cores %u, CPU %u MHz (<= %u mV, %u C), GPU temp %u C, GFX %u MHz @ %u mV", u.cores,
            u.cpuMHz, u.cpuVmax, u.cpuTempC, u.gpuTempC, u.gfxMHz, u.gfxMHz ? (u.gfxMv ? u.gfxMv : gfxSafeMilliVolts(u.gfxMHz)) : 0);
    }
}

void BC250Smu::applyTune(const Telemetry&)
{
    auto& u = this->tune;
    // CPU cores (bc250-core-unlock's check: only the stock mask is changed).
    if (u.cores == 8) {
        const UInt32 mask = this->smnRead(kCoreUnlockSmn), smuMask = this->smnRead(kCoreMaskSmn);
        if (mask == kCoreMaskAll) {
            BCLOG("BC250SMU", "tune: core mask 0x%X (SMN 0x%X; 0x%X at 0x%X): all 8 cores enabled", mask, kCoreUnlockSmn,
                smuMask, kCoreMaskSmn);
        }
        else if (mask != kCoreMaskStock) {
            BCLOG("BC250SMU", "tune: core mask 0x%X is not the stock 0x77 (0x%X at 0x%X); not unlocking (may be a "
                              "defective core)", mask, smuMask, kCoreMaskSmn);
        }
        else if (this->sendTune(3, 0x98, kCoreUnlockSmn) == kOk && this->smnRead(kCoreUnlockSmn) == kCoreMaskAll) {
            BCLOG("BC250SMU", "tune: core mask 0x77 -> 0xFF. Restart (warm) to bring up 8 cores; a power-off reverts it. "
                              "Set the AMD_Vanilla core count to 8 before that Restart and to 6 before a shutdown "
                              "(bc250-set-cores.sh)");
        }
        else {
            BCLOG("BC250SMU", "tune: core unlock failed (mask now 0x%X)", this->smnRead(kCoreUnlockSmn));
        }
    }
    // Temperature limits first, as bc250_apply.
    if (u.cpuTempC != 0) { this->sendTune(3, 0x8B, u.cpuTempC); }
    if (u.gpuTempC != 0) { this->sendTune(3, 0x8C, u.gpuTempC); }
}

// GPU clock and voltage, once the GPU is up (the SMU reports no active WGPs before its init): raise the voltage
// before the clock, lower the clock before the voltage.
// CPU boost clock, always with the undervolt (VID-curve scale) that puts the predicted voltage at the ceiling: the
// stock curve asks ~1.52 V at 4000 MHz. Set once; nothing adjusts it at run time (a guard that tightened the undervolt
// when the measured voltage passed the ceiling cut voltage under load, and 8 cores then hung). Applied from the
// poll loop once the desktop is up, so an unstable clock fails after boot instead of during it.
void BC250Smu::applyCpuTune(const Telemetry& t)
{
    auto& u = this->tune;
    u.cpuTried = true;
    if (u.cpuMHz != 0) {
        UInt32 mhz   = u.cpuMHz;
        SInt32 scale = predictScale(mhz, u.cpuVmax);
        while (scale < kScaleMin && mhz > kCpuMinMHz) {
            mhz   = mhz - 50 < kCpuMinMHz ? kCpuMinMHz : mhz - 50;
            scale = predictScale(mhz, u.cpuVmax);
        }
        scale = scale > kScaleMax ? kScaleMax : scale < kScaleMin ? kScaleMin : scale;
        BCLOG("BC250SMU", "tune: CPU %u MHz asked, %u MHz with VID scale %d predicted for <= %u mV (now %u mV at %u MHz)",
            u.cpuMHz, mhz, scale, u.cpuVmax, t.cpuMilliVolts, t.coreMHz[0]);
        if (this->sendTune(3, 0x9A, 1) == kOk && this->sendTune(3, 0x50, static_cast<UInt32>(scale) & 0xFFFF) == kOk &&
            this->sendTune(3, 0x8F, mhz) == kOk)
        {
            u.cpuActive     = true;
            u.cpuScale      = scale;
            u.cpuAppliedMHz = mhz;
        }
        else {
            // Leave the boost clock at stock if any step failed.
            this->sendTune(3, 0x8F, kCpuMinMHz);
            BCLOG("BC250SMU", "tune: CPU overclock not applied (boost 3500 MHz)");
        }
    }
}

void BC250Smu::applyGfxTune(const Telemetry& t)
{
    auto& u = this->tune;
    u.gfxTried = true;
    if (u.gfxMHz != 0) {
        const UInt32 mv   = u.gfxMv ? u.gfxMv : gfxSafeMilliVolts(u.gfxMHz);
        const UInt32 vid  = milliVoltsToVid(mv);
        const bool   up   = mv >= t.gfxVidMilliVolts;
        bool         done = up ? this->sendTune(0, 0x3B, vid) == kOk && this->sendTune(0, 0x39, u.gfxMHz) == kOk :
                                 this->sendTune(0, 0x39, u.gfxMHz) == kOk && this->sendTune(0, 0x3B, vid) == kOk;
        if (done) {
            u.gfxActive = true;
            BCLOG("BC250SMU", "tune: GFX forced to %u MHz @ %u mV (was %u MHz @ %u mV)", u.gfxMHz, mv, t.gfxClockMHz,
                t.gfxVidMilliVolts);
        }
        else {
            this->sendTune(0, 0x3A, 0);
            this->sendTune(0, 0x3C, 0);
            BCLOG("BC250SMU", "tune: GFX clock not applied (firmware control)");
        }
    }
}

void BC250Smu::guardTune(const Telemetry& t)
{
    auto& u = this->tune;
    // Not while Apple's driver brings the GPU up (forcing GFX clock/voltage ~15 s into boot panicked twice): wait 60
    // polls (~60 s, the desktop is up) and for GPU work in GRBM_STATUS. The SMU's active-WGP count is no signal (it
    // stays 0 after a power cycle).
    if (u.cpuMHz != 0 && !u.cpuTried && ++u.cpuWaitPolls >= 60) {
        BCLOG("BC250SMU", "tune: applying CPU %u MHz now", u.cpuMHz);
        this->applyCpuTune(t);
    }
    if (u.gfxMHz != 0 && !u.gfxTried && ++u.gfxWaitPolls >= 60 && t.gpuBusyValid) {
        BCLOG("BC250SMU", "tune: applying GFX %u MHz now (GPU busy %u%%)", u.gfxMHz, t.gpuBusyPercent);
        this->applyGfxTune(t);
    }
    // GPU: back to firmware control if the die gets too hot.
    const SInt32 gpuGuard = static_cast<SInt32>(u.gpuTempC ? u.gpuTempC : kGpuGuardDefault) * 10;
    if (u.gfxActive && t.tctlDeciC >= gpuGuard) {
        this->sendTune(0, 0x3A, 0);
        this->sendTune(0, 0x3C, 0);
        u.gfxActive = false;
        BCLOG("BC250SMU", "tune guard: Tctl %d.%d C >= %d C, GFX clock/voltage released to the firmware", t.tctlDeciC / 10,
            t.tctlDeciC % 10, gpuGuard / 10);
    }
}

void BC250Smu::poll(Telemetry& t)
{
    UInt32 value = 0;
    t.cpuVoltOk = false;
    t.testOk    = this->send(3, 0x01, 123, &value) == kOk && value == 124;
    if (!t.testOk) { return; }
    if (this->send(0, 0x02, 0, &value) == kOk) { t.smuVersion = value; }
    if (this->send(0, 0x3D, 0, &value) == kOk) { t.features = value; }
    if (this->send(0, 0x1E, 0, &value) == kOk) { t.activeWgps = value; }
    if (this->send(0, 0x0F, 0, &value) == kOk) { t.gfxClockMHz = value; }
    if (this->send(0, 0x38, 0, &value) == kOk) { t.gfxVidMilliVolts = vidToMilliVolts(value); }
    if (this->send(3, 0x37, 0, &value) == kOk) { t.gpuMilliVolts = value; }
    if (this->send(3, 0x36, 0, &value) == kOk) {
        t.cpuMilliVolts = value;
        t.cpuVoltOk     = true;
    }
    if (this->send(3, 0x40, 0, &value) == kOk) { t.cpuTempLimitC = value; }
    t.coreMask     = this->smnRead(kCoreMaskSmn) & 0xFF;
    t.coreMaskUnlk = this->smnRead(kCoreUnlockSmn);
    for (UInt32 core = 0; core < 8; core++) {
        t.coreMHz[core] = (t.coreMask & (1U << core)) != 0 && this->send(3, 0x43, core, &value) == kOk ? value : 0;
    }
    const UInt32 tctl = this->smnRead(kTctlSmn);
    SInt32       milli = static_cast<SInt32>((tctl >> 21) & 0x7FF) * 125;
    if (tctl & (1U << 19)) { milli -= 49000; }    // range select: -49 °C offset
    t.tctlDeciC = milli / 100;
    t.valid     = true;
}

size_t BC250Smu::describe(char* out, size_t capacity) const
{
    const auto& t = this->last;
    if (!t.valid) {
        return static_cast<size_t>(snprintf(out, capacity, "no SMU telemetry (test %s)", t.testOk ? "ok" : "failed"));
    }
    int n = snprintf(out, capacity,
        "SMU 0x%08X features 0x%08X | GFX %u MHz, VID %u mV, GPU %u mV, WGPs %u | CPU %u mV, Tctl %d.%d C (limit %u C), "
        "cores 0x%02X:",
        t.smuVersion, t.features, t.gfxClockMHz, t.gfxVidMilliVolts, t.gpuMilliVolts, t.activeWgps, t.cpuMilliVolts,
        t.tctlDeciC / 10, t.tctlDeciC % 10 < 0 ? -(t.tctlDeciC % 10) : t.tctlDeciC % 10, t.cpuTempLimitC, t.coreMask);
    for (UInt32 core = 0; core < 8 && n > 0 && static_cast<size_t>(n) < capacity; core++) {
        n += snprintf(out + n, capacity - n, " %u", t.coreMHz[core]);
    }
    if (n > 0 && static_cast<size_t>(n) < capacity) { n += snprintf(out + n, capacity - n, " MHz"); }
    if (t.gpuBusyValid && n > 0 && static_cast<size_t>(n) < capacity) {
        n += snprintf(out + n, capacity - n, " | GPU busy %u%%", t.gpuBusyPercent);
    }
    if (n > 0 && static_cast<size_t>(n) < capacity) { n += this->fan.describeAppend(out + n, capacity - n); }
    return n > 0 ? static_cast<size_t>(n) : 0;
}

void BC250Smu::publish(const Telemetry& t)
{
    smuTextLen = this->describe(smuText, sizeof(smuText));
    if (smuTextLen >= sizeof(smuText)) { smuTextLen = sizeof(smuText) - 1; }

    auto* gpu = NRed::singleton().getIGPU();
    auto* dict = OSDictionary::withCapacity(16);
    if (gpu != nullptr && dict != nullptr) {
        auto put = [dict](const char* key, UInt64 value) {
            if (auto* number = OSNumber::withNumber(value, 64)) {
                dict->setObject(key, number);
                number->release();
            }
        };
        put("SMU Version", t.smuVersion);
        put("SMU Features", t.features);
        put("GFX Clock (MHz)", t.gfxClockMHz);
        put("GFX VID (mV)", t.gfxVidMilliVolts);
        put("GPU Voltage (mV)", t.gpuMilliVolts);
        put("Active WGPs", t.activeWgps);
        if (t.gpuBusyValid) { put("GPU Busy (%)", t.gpuBusyPercent); }
        put("CPU Voltage (mV)", t.cpuMilliVolts);
        put("CPU Temperature (0.1 C)", static_cast<UInt32>(t.tctlDeciC));
        put("CPU Temperature Limit (C)", t.cpuTempLimitC);
        put("Core Mask", t.coreMask);
        put("Core Mask (SMN 0x5A870)", t.coreMaskUnlk);
        if (auto* cores = OSArray::withCapacity(8)) {
            for (const auto mhz : t.coreMHz) {
                if (auto* number = OSNumber::withNumber(mhz, 32)) {
                    cores->setObject(number);
                    number->release();
                }
            }
            dict->setObject("Core Clocks (MHz)", cores);
            cores->release();
        }
        if (auto* pstates = OSArray::withCapacity(8)) {
            for (const auto mhz : t.pstateMHz) {
                if (auto* number = OSNumber::withNumber(mhz, 32)) {
                    pstates->setObject(number);
                    number->release();
                }
            }
            dict->setObject("P-state Clocks (MHz)", pstates);
            pstates->release();
        }
        this->fan.publish(dict);
        gpu->setProperty("BC250,SMU", dict);
    }
    if (dict != nullptr) { dict->release(); }
}

// GPU activity as Linux's amdgpu estimates it on parts without a busy counter in the SMU metrics: the share of
// samples with GRBM_STATUS.GUI_ACTIVE (bit 31) set. 50 samples 20 ms apart make up the 1 s poll interval.
SInt32 BC250Smu::sampleGpuBusy()
{
    constexpr UInt32 kSamples = 50, kIntervalMs = 20, kGuiActive = 1U << 31;
    const auto&      nred     = NRed::singleton();
    if (static_cast<UInt64>(CyanSkillfish::GRBM_STATUS) * sizeof(UInt32) >= nred.getMMIOLength()) {
        IOSleep(kSamples * kIntervalMs);
        return -1;
    }
    UInt32 active = 0;
    for (UInt32 i = 0; i < kSamples; i++) {
        if ((nred.readReg32(CyanSkillfish::GRBM_STATUS) & kGuiActive) != 0) { active++; }
        IOSleep(kIntervalMs);
    }
    return static_cast<SInt32>(active * 100 / kSamples);
}

void BC250Smu::run()
{
    // The host bridge is matched early, but wait for IOKit to publish it.
    for (UInt32 tries = 0; tries < 120 && !this->findHostBridge(); tries++) { IOSleep(1000); }
    if (this->hostBridge == nullptr) {
        BCLOG("BC250SMU", "host bridge 00:00.0 not found; no SMU telemetry");
        return;
    }
    // NCT6686D fan monitoring (and opt-in control): its own probe, skipped entirely with bc250fan=0.
    this->fan.detect();
    this->parseTune();
    Telemetry t {};
    this->poll(t);
    if (!t.testOk) {
        BCLOG("BC250SMU", "SMU test message failed (queue 3 msg 0x01); no SMU telemetry");
        this->last = t;
        this->publish(t);
        return;
    }
    UInt32 value = 0;
    for (UInt32 pstate = 0; pstate < 8; pstate++) {
        t.pstateMHz[pstate] = this->send(3, 0x3B, pstate, &value) == kOk ? value : 0;
    }
    this->last = t;
    this->publish(t);
    BCLOG("BC250SMU", "SMU 0x%08X, features 0x%08X, core mask 0x%02X (%s), active WGPs %u", t.smuVersion, t.features,
        t.coreMask, t.coreMask == 0xFF ? "8 cores" : t.coreMask == 0x77 ? "6 cores, stock" : "other", t.activeWgps);
    BCLOG("BC250SMU", "P-state clocks (MHz): %u %u %u %u %u %u %u %u", t.pstateMHz[0], t.pstateMHz[1], t.pstateMHz[2],
        t.pstateMHz[3], t.pstateMHz[4], t.pstateMHz[5], t.pstateMHz[6], t.pstateMHz[7]);
    BCLOG("BC250SMU", "%s", smuText);
    this->applyTune(t);

    UInt32 logged = 0;
    while (true) {
        const SInt32 busy = this->sampleGpuBusy();
        Telemetry now = this->last;
        this->poll(now);
        // Fan RPM/PWM (+ EC temperatures) beside the SMU poll; control runs on Tctl inside.
        this->fan.poll(now.tctlDeciC, now.valid);
        now.gpuBusyValid   = busy >= 0;
        now.gpuBusyPercent = busy >= 0 ? static_cast<UInt32>(busy) : 0;
        const bool changed = now.gfxClockMHz != this->last.gfxClockMHz || now.coreMask != this->last.coreMask ||
                             now.tctlDeciC / 50 != this->last.tctlDeciC / 50;    // 5 °C steps
        this->last = now;
        this->publish(now);
        this->guardTune(now);
        if (changed && ++logged <= 200) { BCLOG("BC250SMU", "%s", smuText); }
        BC250HWL::singleton().accelPoolTick();
    }
}
