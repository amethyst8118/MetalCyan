// ASRock BC-250 NCT6686D fan monitoring + opt-in PWM control
//
// Copyright © 2026 NootedRed-BC250 contributors. Licensed under the Thou Shalt Not Profit License version 1.5.
// See LICENSE for details.
//
// The board's Super I/O is a Nuvoton NCT6686D (config port 0x2E, EC base 0xA20 on this board). Only register
// facts (ports, addresses, bit positions, ID values, byte sequences) are used here; no driver code was copied
// (Fred78290/nct6687d and Linux nct6683 are GPL-2.0). UNTESTED on the board: monitoring is read-only and on by
// default, control needs explicit boot-args, and every failure hands the fans back to the firmware and latches
// control off for the rest of the boot.
//
// Owned by BC250Smu: detect() runs once from its telemetry thread, then poll() beside each ~1 s SMU poll. Every
// EC page/index/data sequence holds one IOLock, every wait is bounded, and nothing here panics.

#pragma once
#include <IOKit/IOLocks.h>
#include <IOKit/IOTypes.h>

class OSDictionary;

class BC250Fan
{
public:
    static constexpr UInt32 kChannels = 5;    // EC fan channels 0-4 polled for RPM/PWM
    static constexpr UInt32 kMaxTemps = 3;    // CPU (AMD TSI) + up to two board zones

    // One-shot Super I/O probe + EC base discovery + boot-arg parsing. Skips everything when bc250fan=0.
    // Logs once whether the chip was found; without it every other method is a no-op.
    void detect();

    bool isPresent() const { return this->present; }

    // One call per telemetry poll. tctlDeciC/tctlValid come from the SMU telemetry (the curve and the 85 C
    // guard run on Tctl, and control only starts once it is valid).
    void poll(SInt32 tctlDeciC, bool tctlValid);

    // Appends " | fans <rpm..> rpm pwm <pct..>%[ nct <temps>C]" (nothing when the chip is absent). Returns the
    // bytes written.
    size_t describeAppend(char* out, size_t capacity) const;

    // Adds "Fan Speed (RPM)", "Fan PWM (%)" and (when found) "NCT Temp (0.1 C)" arrays. No-op without the chip.
    void publish(OSDictionary* dict) const;

private:
    UInt8  sioRead(UInt16 port, UInt8 reg);
    void   sioWrite(UInt16 port, UInt8 reg, UInt8 value);
    void   sioExit(UInt16 port);
    UInt8  ecRead(UInt16 addr);
    UInt16 ecRead16(UInt16 addr);
    void   ecWrite(UInt16 addr, UInt8 value);
    // EC fan-config handshake (0xA01 REQ 0x80 / DONE 0x40, polled on 0xCF8). Bounded: ~1 s each phase at most.
    bool   configOpen();
    bool   configCommit();
    bool   setDuty(UInt8 mask, UInt8 duty);
    bool   verifyDuty(UInt8 mask, UInt8 duty);
    void   restoreFirmware();
    void   failSafe(const char* reason);
    UInt8  liveMask() const;
    UInt32 wantedPct(SInt32 tctlDeciC) const;
    static UInt8  pctToDuty(UInt32 pct) { return static_cast<UInt8>((pct * 255 + 50) / 100); }
    static UInt32 rawToPct(UInt8 raw) { return (static_cast<UInt32>(raw) * 100 + 127) / 255; }

    IOLock* lock {nullptr};
    UInt16  ecBase {0};
    bool    present {false};
    bool    disabled {false};

    // Last monitoring sample (written by poll(), read back by describeAppend()/publish() on the same thread).
    UInt16 rpm[kChannels] {};
    UInt8  pwmRaw[kChannels] {};
    SInt32 tempCh[kMaxTemps] {-1, -1, -1};    // monitor-channel indices behind 0x100+2i (tempCh[0] = CPU)
    SInt32 tempDeciC[kMaxTemps] {};
    UInt32 tempCount {0};

    // Boot-args (parsed once in detect(); 0/not given = off).
    UInt32 fixedPct {0};                      // bc250fanpct=N, 30-100
    bool   curve {false};                     // bc250fancurve=1
    UInt32 floorPct {30};                     // bc250fanmin=N, 20-100 (default 30)

    // Control state.
    UInt32 polls {0};
    bool   engaged {false};                   // a duty has been written at least once
    bool   latchedOff {false};                // failed or restored: no more EC writes this boot
    UInt8  mask {0};                          // controlled channels (spinning when control started)
    SInt32 lastDuty {-1};
    UInt8  zeroStrikes[kChannels] {};
};
