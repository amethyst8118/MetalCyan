// ASRock BC-250 (AMD Cyan Skillfish) display engine quirks on top of AMDRadeonX6000Framebuffer's DCN 2.0 code
//
// Copyright © 2026 NootedRed-BC250 contributors. Licensed under the Thou Shalt Not Profit License version 1.5.
// See LICENSE for details.

#pragma once
#include <Headers/kern_patcher.hpp>

// Apple's DAL drives the BC-250's DCN 2.0.1 as if it were Navi 10's DCN 2.0. Three differences break the picture
// (found on the board, cross-checked against Linux's dcn201 code):
//  1. DCN 2.0.1 has no DCN_VM aperture, so HUBP needs the carve-out's system (UMA) address, not its MC address
//     (Linux: dcn201_hwseq.c gpu_addr_to_uma). Fixed where Apple hands planes and cursors to the DAL.
//  2. The DAL sizes DENTIST dividers for Navi 10's VCO (~3.6 GHz); the BC-250's is read from CLK4_CLK_PLL_REQ
//     (Linux: dcn201_clk_mgr.c). A watcher thread rescales the dividers whenever the DAL changes them; the
//     bc250dispmhz boot-arg raises the DISPCLK/DPPCLK ceiling for high pixel-clock modes, and the VCO and each
//     rescale are logged.
//  3. DCN 2.0.1 has 4 pipes, Navi 10 has 6: an MPC split lands on pipe 5 and half the screen is lost. The DAL's
//     dcn20_validate_apply_pipe_split_flags, found by its __PRETTY_FUNCTION__ string, has its split requests
//     withdrawn (one pipe per stream); DPPCLK is kept at least DISPCLK to carry it.
class BC250DCN
{
public:
    static BC250DCN& singleton();

    void processKext(KernelPatcher& patcher, size_t id, mach_vm_address_t slide, size_t size);

private:
    UInt32            vcoKHz{0};
    UInt32            didMin{0x0A};    // Effective minimum DID: the GOP's 0x0A, lowered by bc250dispmhz=N.
    UInt64            fbBase{0}, fbTop{0}, fbOffset{0};
    UInt32            appleVcoKHz{3600000};
    UInt32            lastDispDid{0}, lastDppDid{0};
    mach_vm_address_t orgPrepareSurface{0};
    mach_vm_address_t orgUpdateSurfaceInfo{0};
    mach_vm_address_t orgSetCursorImage{0};
    mach_vm_address_t orgApplyPipeSplitFlags{0};

    bool   toUMA(UInt64& addr) const;
    size_t translateWindow(void* base, size_t length, UInt64* saved, size_t* savedOff,
                           size_t maxSaved);
    void   restoreWindow(void* base, const UInt64* saved, const size_t* savedOff, size_t count);
    void   startClockWatcher();
    void   clockTick();
    UInt32 rescaleDid(UInt32 did) const;
    UInt32 didToKHz(UInt32 did) const;

    static UInt64 wrapPrepareSurface(void* self, const void* displayPath, void* plane);
    static UInt64 wrapUpdateSurfaceInfo(void* self, const void* plane, const void* displayPath);
    static UInt64 wrapSetCursorImage(void* self, const void* cursor);
    static int    wrapApplyPipeSplitFlags(void* dc, void* context, int vlevel, int* split, bool* merge);
    static void   clockWatcherMain(void* arg, int waitResult);
};
