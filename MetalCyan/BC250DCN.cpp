// ASRock BC-250 (AMD Cyan Skillfish) display engine quirks on top of AMDRadeonX6000Framebuffer's DCN 2.0 code
//
// Copyright © 2026 NootedRed-BC250 contributors. Licensed under the Thou Shalt Not Profit License version 1.5.
// See LICENSE for details.

#include <BC250.hpp>
#include <BC250DCN.hpp>
#include <Headers/kern_util.hpp>
#include <IOKit/IOLib.h>
#include <NRed.hpp>
#include <Regs/CyanSkillfish.hpp>
#include <kern/thread.h>
#include <sys/sysctl.h>

static BC250DCN moduleInstance;

BC250DCN& BC250DCN::singleton() { return moduleInstance; }

namespace
{
    // DCN 2.0.1 registers (dcn_2_0_1_offset.h on the DMU segments 0x12/0xC0/0x34C0/0x9000).
    constexpr UInt32 DENTIST_DISPCLK_CNTL                = 0xC0 + 0x64;
    constexpr UInt32 DENTIST_DISPCLK_WDIVIDER_MASK       = 0x7F;
    constexpr UInt32 DENTIST_DPPCLK_WDIVIDER_SHIFT       = 24;
    constexpr UInt32 DENTIST_DISPCLK_CHG_DONE            = 1U << 19;
    constexpr UInt32 DENTIST_DPPCLK_CHG_DONE             = 1U << 20;
    constexpr UInt32 DENTIST_DID_GOP                     = 0x0A;    // Divider 2.5, what the GOP runs at.
    constexpr UInt32 DENTIST_DID_MIN_ABS                 = 0x08;    // Divider 2.0, start of DENTIST range 1.
    constexpr UInt32 DENTIST_DID_MAX                     = 0x7E;    // 0x7F is special (bypass); left alone.
    // CLK4 (clk_11_0_1_offset.h, CLK base 0x16C00): the DENTIST VCO, as dcn201_clk_mgr reads it.
    constexpr UInt32 CLK4_CLK_PLL_REQ                    = 0x16C00 + 0x460E;

    constexpr size_t PLANE_SCAN_BYTES  = 0x200;
    constexpr size_t CURSOR_SCAN_BYTES = 0x80;
    constexpr size_t MAX_SAVED         = 8;

    const char kApplyPipeSplitFlags[] =
        "int dcn20_validate_apply_pipe_split_flags(struct dc *, struct dc_state *, int, int *, _Bool *)";
    constexpr size_t MAX_PIPES = 6;    // Navi 10's DAL, the size of the split[] arrays it passes around.

    // DENTIST divider IDs in quarter steps (dcn20_clk_mgr.c ranges 1 and 2; the 7-bit field never reaches 3).
    UInt32 didToDiv4(UInt32 did) { return did < 64 ? did : 64 + (did - 64) * 2; }
    UInt32 div4ToDid(UInt32 div4) { return div4 < 64 ? div4 : 64 + (div4 - 64) / 2; }

    // Finds the function that references `str` with a RIP-relative LEA, by walking back to its prologue. Every
    // reference has to resolve to the same function, otherwise nothing is returned.
    mach_vm_address_t findFunctionByString(mach_vm_address_t start, size_t size, const char* str, size_t len)
    {
        size_t strOff = 0;
        if (!KernelPatcher::findPattern(str, nullptr, len, reinterpret_cast<const void*>(start), size, &strOff)) {
            return 0;
        }
        const auto strAddr = start + strOff;
        const auto* p      = reinterpret_cast<const UInt8*>(start);

        mach_vm_address_t found = 0;
        for (size_t i = 0; i + 7 <= size; i++) {
            if ((p[i] != 0x48 && p[i] != 0x4C) || p[i + 1] != 0x8D || (p[i + 2] & 0xC7) != 0x05) { continue; }
            SInt32 disp;
            memcpy(&disp, p + i + 3, sizeof(disp));
            if (static_cast<mach_vm_address_t>(static_cast<SInt64>(start + i + 7) + disp) != strAddr) { continue; }
            mach_vm_address_t fn = 0;
            for (size_t j = i; j >= 4 && i - j < 0x8000; j--) {
                if (p[j] == 0x55 && p[j + 1] == 0x48 && p[j + 2] == 0x89 && p[j + 3] == 0xE5) {
                    const UInt8 prev = p[j - 1];
                    if ((j % 16) == 0 || prev == 0xC3 || prev == 0xCC || prev == 0x90 || prev == 0x00) {
                        fn = start + j;
                        break;
                    }
                }
            }
            if (fn == 0 || (found != 0 && found != fn)) { return 0; }
            found = fn;
        }
        return found;
    }
}    // namespace

void BC250DCN::processKext(KernelPatcher& patcher, size_t id, mach_vm_address_t slide, size_t size)
{
    auto& nred = NRed::singleton();
    using namespace CyanSkillfish;
    this->fbBase   = static_cast<UInt64>(nred.readReg32(GCMC_VM_FB_LOCATION_BASE) & 0xFFFFFF) << 24;
    this->fbTop    = static_cast<UInt64>((nred.readReg32(GCMC_VM_FB_LOCATION_TOP) & 0xFFFFFF) + 1) << 24;
    this->fbOffset = static_cast<UInt64>(nred.readReg32(GCMC_VM_FB_OFFSET) & 0xFFFFFF) << 24;

    // 1. Scanout addresses. Each hook is optional: a missing one is logged, not fatal.
    KernelPatcher::RouteRequest requests[] = {
        {"__ZN27AMDRadeonX6000_AmdDalHelper34prepareDalDisplaySurfaceParametersEPK16AmdFbDisplayPathP20AmdFalconUpdate"
         "Plane",
            wrapPrepareSurface, this->orgPrepareSurface},
        {"__ZN27AMDRadeonX6000_AmdDalHelper17updateSurfaceInfoEPK20AmdFalconUpdatePlanePK16AmdFbDisplayPath",
            wrapUpdateSurfaceInfo, this->orgUpdateSurfaceInfo},
        {"__ZN27AMDRadeonX6000_AmdDalHelper14setCursorImageEPK21AmdFbCursorDescriptor", wrapSetCursorImage,
            this->orgSetCursorImage},
    };
    for (auto& request : requests) {
        if (!patcher.routeMultiple(id, &request, 1, slide, size)) {
            BCLOG("BC250DCN", "Failed to route %s", request.symbol);
            patcher.clearError();
        }
    }

    // 3. MPC splits. The DAL is kept from splitting at all (one pipe per stream, like the GOP), since the partner
    // pipe of a split does not come out right on DCN 2.0.1.
    {
        const auto fn = findFunctionByString(slide, size, kApplyPipeSplitFlags, sizeof(kApplyPipeSplitFlags));
        KernelPatcher::RouteRequest request {nullptr, wrapApplyPipeSplitFlags, this->orgApplyPipeSplitFlags};
        request.from = fn;
        if (fn == 0 || !patcher.routeMultiple(id, &request, 1)) {
            BCLOG("BC250DCN", "dcn20_validate_apply_pipe_split_flags not routed; splits may use missing pipes");
            patcher.clearError();
        }
    }

    // 2. DENTIST VCO, as dcn201_clk_mgr_construct reads it (FbMult in units of 100 MHz).
    const auto pllReq  = nred.readReg32(CLK4_CLK_PLL_REQ);
    const auto fbInt   = pllReq & 0x1FF;
    const auto fbFrac  = pllReq >> 16;
    this->vcoKHz       = fbInt * 100000 + static_cast<UInt32>((static_cast<UInt64>(fbFrac) * 100000) >> 16);
    if (this->vcoKHz < 1000000 || this->vcoKHz > 6000000) {
        BCLOG("BC250DCN", "Implausible DENTIST VCO %u kHz (CLK4_CLK_PLL_REQ 0x%X); using 2670000", this->vcoKHz,
            pllReq);
        this->vcoKHz = 2670000;
    }
    // Opt-in display clock ceiling for high pixel-clock modes such as 4K 120 Hz: the smallest DID in
    // [0x08, 0x0A] whose clock fits under the requested MHz. VCO 2670000 + bc250dispmhz=1200 gives 0x09
    // (1186666 kHz); VCO 3000000 already reaches 1200 MHz at 0x0A, so the arg changes nothing. 1200 MHz is
    // Linux's dcn201 max dispclk/dppclk (dcn201_clk_mgr.c max_supported_dispclk_khz).
    UInt32 dispMHz = 0;
    if (PE_parse_boot_argn("bc250dispmhz", &dispMHz, sizeof(dispMHz)) && dispMHz != 0) {
        if (dispMHz < 1000 || dispMHz > 1200) {
            BCLOG("BC250DCN", "Ignoring bc250dispmhz=%u (valid range 1000-1200)", dispMHz);
        } else {
            for (UInt32 d = DENTIST_DID_MIN_ABS; d <= DENTIST_DID_GOP; d++) {
                if (this->didToKHz(d) <= dispMHz * 1000) { this->didMin = d; break; }
            }
            BCLOG("BC250DCN", "bc250dispmhz=%u: minimum DID 0x%X (%u kHz)", dispMHz, this->didMin,
                this->didToKHz(this->didMin));
        }
    }
    BCLOG("BC250DCN", "DENTIST VCO %u kHz (CLK4_CLK_PLL_REQ 0x%X); DISPCLK/DPPCLK ceiling %u kHz", this->vcoKHz,
        pllReq, this->didToKHz(this->didMin));
    this->startClockWatcher();
}

// -- 1. Scanout addresses (MC -> UMA) --

bool BC250DCN::toUMA(UInt64& addr) const
{
    if (addr < this->fbBase || addr >= this->fbTop) { return false; }
    addr = addr - this->fbBase + this->fbOffset;
    return true;
}

// Rewrites every 8-byte aligned MC carve-out address in [base, base + length). The range 0xF400000000+ is
// distinctive enough that nothing but an address lands in it. Optionally records originals for restoreWindow.
size_t BC250DCN::translateWindow(void* base, size_t length, UInt64* saved, size_t* savedOff,
    size_t maxSaved)
{
    if (base == nullptr) { return 0; }
    size_t count = 0;
    auto*  words = static_cast<UInt64*>(base);
    for (size_t i = 0; i < length / sizeof(UInt64); i++) {
        UInt64 value = words[i];
        if (!this->toUMA(value)) { continue; }
        if (saved != nullptr) {
            if (count >= maxSaved) { break; }
            saved[count]    = words[i];
            savedOff[count] = i;
        }
        words[i] = value;
        count += 1;
    }
    return count;
}

void BC250DCN::restoreWindow(void* base, const UInt64* saved, const size_t* savedOff, size_t count)
{
    auto* words = static_cast<UInt64*>(base);
    for (size_t i = 0; i < count; i++) { words[savedOff[i]] = saved[i]; }
}

UInt64 BC250DCN::wrapPrepareSurface(void* self, const void* displayPath, void* plane)
{
    auto&      s   = singleton();
    const auto ret = FunctionCast(wrapPrepareSurface, s.orgPrepareSurface)(self, displayPath, plane);
    s.translateWindow(plane, PLANE_SCAN_BYTES, nullptr, nullptr, 0);
    return ret;
}

UInt64 BC250DCN::wrapUpdateSurfaceInfo(void* self, const void* plane, const void* displayPath)
{
    auto&  s = singleton();
    UInt64 saved[MAX_SAVED];
    size_t off[MAX_SAVED];
    auto*  mutablePlane = const_cast<void*>(plane);
    const auto n        = s.translateWindow(mutablePlane, PLANE_SCAN_BYTES, saved, off, MAX_SAVED);
    const auto ret      = FunctionCast(wrapUpdateSurfaceInfo, s.orgUpdateSurfaceInfo)(self, plane, displayPath);
    s.restoreWindow(mutablePlane, saved, off, n);
    return ret;
}

UInt64 BC250DCN::wrapSetCursorImage(void* self, const void* cursor)
{
    auto&  s = singleton();
    UInt64 saved[MAX_SAVED];
    size_t off[MAX_SAVED];
    auto*  mutableCursor = const_cast<void*>(cursor);
    const auto n = s.translateWindow(mutableCursor, CURSOR_SCAN_BYTES, saved, off, MAX_SAVED);
    const auto ret = FunctionCast(wrapSetCursorImage, s.orgSetCursorImage)(self, cursor);
    s.restoreWindow(mutableCursor, saved, off, n);
    return ret;
}

// -- 3. MPC split pipes --

// Lets the DAL pick its voltage level, then withdraws every MPC split request, so each stream stays on one pipe.
int BC250DCN::wrapApplyPipeSplitFlags(void* dc, void* context, int vlevel, int* split, bool* merge)
{
    auto&      s   = singleton();
    const auto ret = FunctionCast(wrapApplyPipeSplitFlags, s.orgApplyPipeSplitFlags)(dc, context, vlevel, split, merge);
    if (split != nullptr) {
        for (size_t i = 0; i < MAX_PIPES; i++) {
            if (split[i] != 0) {
                split[i] = 0;
            }
        }
    }
    return ret;
}

// -- 2. DENTIST clocks --

// The DAL wanted appleVco / div; give it at least that from the real VCO, capped at didMin (the GOP's 0x0A by
// default, lowered by bc250dispmhz=N).
UInt32 BC250DCN::rescaleDid(UInt32 did) const
{
    if (did < 8 || did > DENTIST_DID_MAX) { return did; }
    const auto div4   = didToDiv4(did);
    const auto target = static_cast<UInt32>(static_cast<UInt64>(div4) * this->vcoKHz / this->appleVcoKHz);
    auto       newDid = div4ToDid(target);
    if (newDid < this->didMin) { newDid = this->didMin; }
    if (newDid > DENTIST_DID_MAX) { newDid = DENTIST_DID_MAX; }
    return newDid;
}

// Real-VCO clock of a DID in kHz (VCO * 4 / div4); 0 outside the DENTIST range.
UInt32 BC250DCN::didToKHz(UInt32 did) const
{
    if (did < 8 || did > DENTIST_DID_MAX) { return 0; }
    return static_cast<UInt32>(static_cast<UInt64>(this->vcoKHz) * 4 / didToDiv4(did));
}

void BC250DCN::clockTick()
{
    auto&      nred = NRed::singleton();
    const auto v    = nred.readReg32(DENTIST_DISPCLK_CNTL);
    const auto disp = v & DENTIST_DISPCLK_WDIVIDER_MASK;
    const auto dpp  = (v >> DENTIST_DPPCLK_WDIVIDER_SHIFT) & DENTIST_DISPCLK_WDIVIDER_MASK;
    if (disp == this->lastDispDid && dpp == this->lastDppDid) { return; }

    // Only a field the DAL changed is rescaled; one still holding our last value is already corrected (the DAL
    // writes DISPCLK and DPPCLK separately, so a tick can land between the two).
    const auto newDisp = disp != this->lastDispDid ? this->rescaleDid(disp) : disp;
    auto       newDpp  = dpp != this->lastDppDid ? this->rescaleDid(dpp) : dpp;
    // Without MPC splits one pipe carries the whole stream: keep DPPCLK at least DISPCLK (a lower DID is faster).
    if (newDpp > newDisp && newDisp >= this->didMin && newDpp <= DENTIST_DID_MAX) { newDpp = newDisp; }
    if (newDisp != disp) {
        nred.writeReg32(DENTIST_DISPCLK_CNTL, (nred.readReg32(DENTIST_DISPCLK_CNTL) & ~DENTIST_DISPCLK_WDIVIDER_MASK) |
                                                  newDisp);
        for (int i = 0; i < 100 && !(nred.readReg32(DENTIST_DISPCLK_CNTL) & DENTIST_DISPCLK_CHG_DONE); i++) {
            IODelay(10);
        }
    }
    if (newDpp != dpp) {
        const auto mask = DENTIST_DISPCLK_WDIVIDER_MASK << DENTIST_DPPCLK_WDIVIDER_SHIFT;
        nred.writeReg32(DENTIST_DISPCLK_CNTL,
            (nred.readReg32(DENTIST_DISPCLK_CNTL) & ~mask) | (newDpp << DENTIST_DPPCLK_WDIVIDER_SHIFT));
        for (int i = 0; i < 100 && !(nred.readReg32(DENTIST_DISPCLK_CNTL) & DENTIST_DPPCLK_CHG_DONE); i++) {
            IODelay(10);
        }
    }
    this->lastDispDid = newDisp;
    this->lastDppDid  = newDpp;
    if (newDisp != disp || newDpp != dpp) {
        BCLOG("BC250DCN", "Display clocks DAL disp 0x%X dpp 0x%X -> disp 0x%X (%u kHz) dpp 0x%X (%u kHz)", disp,
            dpp, newDisp, this->didToKHz(newDisp), newDpp, this->didToKHz(newDpp));
    }
}

void BC250DCN::clockWatcherMain(void*, int)
{
    while (true) {
        singleton().clockTick();
        IOSleep(5);
    }
}

void BC250DCN::startClockWatcher()
{
    thread_t thread = nullptr;
    if (kernel_thread_start(reinterpret_cast<thread_continue_t>(clockWatcherMain), this, &thread) != KERN_SUCCESS) {
        BCLOG("BC250DCN", "Failed to start the DENTIST watcher");
        return;
    }
    thread_deallocate(thread);
}
