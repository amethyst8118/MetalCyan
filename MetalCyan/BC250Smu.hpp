// ASRock BC-250 (AMD Cyan Skillfish) SMU access
//
// Copyright © 2026 NootedRed-BC250 contributors. Licensed under the Thou Shalt Not Profit License version 1.5.
// See LICENSE for details.
//
// The SMU (MP1) is reached as the BC-250 community tools reach it (bc250_smu in bc250-collective/bc250_smu_oc,
// MIT): the SMN index/data pair at 0xB8/0xBC in the host bridge's (00:00.0) PCI config space, and one
// command/response/argument mailbox per queue. Telemetry only: an allowlist of read-only messages, polled once a second by a kernel thread,
// published as the GPU's "BC250,SMU" property and `sysctl debug.bc250.smu`. `bc250smu=0` disables it.
//
// Tuning (all opt-in boot-args, off by default, clamped to the community's limits, every message logged):
//  bc250cores=8          CPU core unlock as rw-r-r-0644/bc250-core-unlock (MIT): if the core presence mask (SMN 0x5A870)
//                        reads the stock 0x77, SMU queue 3 message 0x98 with argument 0x5A870 sets it to 0xFF; the
//                        extra cores appear after a warm reboot (Restart; a power-off reverts it). macOS also needs the
//                        AMD_Vanilla core-count patches set to 8.
//  bc250cpumhz=N         CPU max boost clock, 3500-4500 MHz (queue 3 0x8F), always with an undervolt: the VID-curve
//                        scale (0x50, -50..0) is predicted with bc250_smu_oc's model for bc250cpuvmax and set once
//                        (no run-time adjustment). As bc250_smu_oc: extra voltage off (0x9A 1) first.
//  bc250cpuvmax=N        CPU voltage ceiling, 950-1325 mV (default 1275; never above 1.325 V).
//  bc250cputemp=N        CPU temperature limit (0x8B), 50-100 C (default 90 with bc250cpumhz).
//  bc250gputemp=N        GPU temperature limit (0x8C), 50-100 C.
//  bc250gfxmhz=N         GPU clock, forced (queue 0 0x39), 350-2000 MHz; voltage from the governor's safe points
//                        (350 MHz/700 mV to 2000 MHz/1000 mV) unless bc250gfxmv=N (700-1100 mV) is given (0x3B).
//                        Released (0x3A/0x3C, firmware control) if Tctl passes bc250gputemp (default 90 C).

#pragma once
#include <BC250Fan.hpp>
#include <IOKit/IOLocks.h>
#include <IOKit/IOTypes.h>

class IOPCIDevice;

class BC250Smu
{
public:
    static BC250Smu& singleton();

    // Starts the telemetry thread (once). It waits for the host bridge, then polls.
    void start();

    // SMU mailbox status codes.
    static constexpr UInt32 kOk = 0x01, kFailed = 0xFF, kUnknownCmd = 0xFE, kRejectedPrereq = 0xFD,
                            kRejectedBusy = 0xFC, kTimeout = 0;

    struct Telemetry
    {
        bool   valid {false};
        bool   testOk {false};
        UInt32 smuVersion {0};
        UInt32 features {0};
        UInt32 activeWgps {0};
        UInt32 gfxClockMHz {0};
        UInt32 gfxVidMilliVolts {0};     // From the GFX VID (1.55 V - VID * 6.25 mV)
        UInt32 gpuMilliVolts {0};        // Q3 0x37
        UInt32 cpuMilliVolts {0};        // Q3 0x36
        bool   cpuVoltOk {false};        // cpuMilliVolts was read in the last poll
        UInt32 cpuTempLimitC {0};        // Q3 0x40
        SInt32 tctlDeciC {0};            // SMN THM_TCON_CUR_TMP, 0.1 °C
        UInt32 coreMask {0};             // SMN 0x0115A870: 0x77 = 6 cores (stock), 0xFF = 8
        UInt32 coreMaskUnlk {0};         // SMN 0x5A870, what bc250-core-unlock reads and the unlock sets
        UInt32 coreMHz[8] {};            // Q3 0x43 per core (0 when the core is off or the query fails)
        UInt32 pstateMHz[8] {};          // Q3 0x3B per P-state
        UInt32 gpuBusyPercent {0};       // GRBM_STATUS.GUI_ACTIVE, sampled over the last poll interval
        bool   gpuBusyValid {false};
    };

    const Telemetry& telemetry() const { return this->last; }

    // Opt-in tuning from the boot-args (0 = not requested).
    struct Tune
    {
        UInt32 cores {0};
        UInt32 cpuMHz {0}, cpuVmax {0}, cpuTempC {0}, gpuTempC {0};
        UInt32 gfxMHz {0}, gfxMv {0};
        // Applied state.
        bool   cpuActive {false}, cpuTried {false}, gfxActive {false}, gfxTried {false};
        UInt32 cpuWaitPolls {0}, gfxWaitPolls {0};
        SInt32 cpuScale {0};
        UInt32 cpuAppliedMHz {0};
    };
    // Text summary for the sysctl, NUL-terminated; returns the length.
    size_t describe(char* out, size_t capacity) const;

private:
    IOPCIDevice* hostBridge {nullptr};
    IOLock*      lock {nullptr};
    Telemetry    last {};
    bool         started {false};
    BC250Fan     fan {};

    bool   findHostBridge();
    UInt32 smnRead(UInt32 address);
    void   smnWrite(UInt32 address, UInt32 value);
    // Sends an allowlisted message; *out gets the argument register after completion. Returns the status.
    UInt32 send(UInt32 queue, UInt32 msg, UInt32 arg, UInt32* out);
    void   poll(Telemetry& t);
    void   publish(const Telemetry& t);
    void   run();
    // Tuning: sends an allowlisted write message after checking its argument; logs it. Returns the status.
    UInt32 sendTune(UInt32 queue, UInt32 msg, UInt32 arg);
    void   parseTune();
    void   applyTune(const Telemetry& t);
    void   applyCpuTune(const Telemetry& t);
    void   applyGfxTune(const Telemetry& t);
    void   guardTune(const Telemetry& t);
    Tune   tune {};
    // Sleeps one poll interval while sampling GRBM_STATUS; returns the GUI_ACTIVE percentage (or -1 without MMIO).
    SInt32 sampleGpuBusy();

    friend void bc250SmuThread(void*, int);
};
