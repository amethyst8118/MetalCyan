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
    constexpr UInt32 DENTIST_DID_MIN                     = 0x0A;    // Divider 2.5, what the GOP runs at.
    constexpr UInt32 DENTIST_DID_MAX                     = 0x7E;    // 0x7F is special (bypass); left alone.
    // CLK4 (clk_11_0_1_offset.h, CLK base 0x16C00): the DENTIST VCO, as dcn201_clk_mgr reads it.
    constexpr UInt32 CLK4_CLK_PLL_REQ                    = 0x16C00 + 0x460E;

    constexpr size_t PLANE_SCAN_BYTES  = 0x200;
    constexpr size_t CURSOR_SCAN_BYTES = 0x80;
    constexpr size_t MAX_SAVED         = 8;

    const char kApplyPipeSplitFlags[] =
        "int dcn20_validate_apply_pipe_split_flags(struct dc *, struct dc_state *, int, int *, _Bool *)";
    constexpr size_t MAX_PIPES = 6;    // Navi 10's DAL, the size of the split[] arrays it passes around.

    // dcn20_update_clocks_update_dentist's __func__, which its REG_WAITs pass. It opens with the guard
    // `if (!clks.dppclk_khz || !clks.dispclk_khz) return;` (mov/test/je on clk_mgr_internal +0x18, then +0x10),
    // which wrapUpdateDentist repeats, so the routed function has to start with exactly that.
    const char       kUpdateDentist[]    = "dcn20_update_clocks_update_dentist";
    const UInt8      kDentistGuardDpp[]  = {0x8B, 0x4F, 0x18, 0x85, 0xC9, 0x0F, 0x84};
    const UInt8      kDentistGuardDisp[] = {0x8B, 0x7F, 0x10, 0x85, 0xFF, 0x0F, 0x84};
    constexpr size_t DENTIST_GUARD_SCAN  = 0x40;
    constexpr size_t CLKS_DISPCLK_KHZ    = 0x10;
    constexpr size_t CLKS_DPPCLK_KHZ     = 0x18;

    // DENTIST divider IDs in quarter steps (dcn20_clk_mgr.c ranges 1 and 2; the 7-bit field never reaches 3).
    UInt32 didToDiv4(UInt32 did) { return did < 64 ? did : 64 + (did - 64) * 2; }
    UInt32 div4ToDid(UInt32 div4) { return div4 < 64 ? div4 : 64 + (div4 - 64) / 2; }
    // The clock a DID gives off `vcoKHz` (DID in quarter steps), in MHz; 0 for a DID with no divider.
    UInt32 didToMHz(UInt32 vcoKHz, UInt32 did)
    {
        const auto div4 = didToDiv4(did);
        return div4 == 0 ? 0 : vcoKHz * 4 / div4 / 1000;
    }

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
    BCLOG("BC250DCN", "DENTIST VCO %u kHz (CLK4_CLK_PLL_REQ 0x%X), DAL assumes %u kHz", this->vcoKHz, pllReq,
        this->appleVcoKHz);
    this->clockLock = IOLockAlloc();
    if (this->clockLock == nullptr) {
        BCLOG("BC250DCN", "Failed to allocate the DENTIST lock; dividers left as the DAL sets them");
        return;
    }
    // Catches every DAL divider write, also one equal to what is programmed, which the watcher cannot see.
    {
        const auto fn      = findFunctionByString(slide, size, kUpdateDentist, sizeof(kUpdateDentist));
        size_t     off     = 0;
        const auto guarded = fn != 0 &&
                             KernelPatcher::findPattern(kDentistGuardDpp, nullptr, sizeof(kDentistGuardDpp),
                                 reinterpret_cast<const void*>(fn), DENTIST_GUARD_SCAN, &off) &&
                             KernelPatcher::findPattern(kDentistGuardDisp, nullptr, sizeof(kDentistGuardDisp),
                                 reinterpret_cast<const void*>(fn), DENTIST_GUARD_SCAN, &off);
        KernelPatcher::RouteRequest request {nullptr, wrapUpdateDentist, this->orgUpdateDentist};
        request.from = fn;
        if (!guarded || !patcher.routeMultiple(id, &request, 1)) {
            BCLOG("BC250DCN", "dcn20_update_clocks_update_dentist not routed; a DAL divider equal to ours goes unseen");
            patcher.clearError();
        }
    }
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

// The DAL wanted appleVco / div; give it at least that from the real VCO, never above the GOP's 0x0A.
UInt32 BC250DCN::rescaleDid(UInt32 did) const
{
    if (did < 8 || did > DENTIST_DID_MAX) { return did; }
    const auto div4   = didToDiv4(did);
    const auto target = static_cast<UInt32>(static_cast<UInt64>(div4) * this->vcoKHz / this->appleVcoKHz);
    auto       newDid = div4ToDid(target);
    if (newDid < DENTIST_DID_MIN) { newDid = DENTIST_DID_MIN; }
    if (newDid > DENTIST_DID_MAX) { newDid = DENTIST_DID_MAX; }
    return newDid;
}

// Programs the rescaled dividers. `disp`/`dpp` are the fields as read back; a field `FromDal` holds what the DAL
// just wrote and is rescaled, any other one holds our last value and is kept. The caller holds clockLock.
void BC250DCN::applyDids(UInt32 disp, UInt32 dpp, bool dispFromDal, bool dppFromDal)
{
    auto&      nred    = NRed::singleton();
    const auto newDisp = dispFromDal ? this->rescaleDid(disp) : disp;
    auto       newDpp  = dppFromDal ? this->rescaleDid(dpp) : dpp;
    // Without MPC splits one pipe carries the whole stream: keep DPPCLK at least DISPCLK (a lower DID is faster).
    if (newDpp > newDisp && newDisp >= DENTIST_DID_MIN && newDpp <= DENTIST_DID_MAX) { newDpp = newDisp; }
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
    // Logged only when the DAL's request or the programmed divider moved, not when the DAL repeats itself.
    if ((dispFromDal && disp != this->dalDispDid) || newDisp != this->lastDispDid) {
        BCLOG("BC250DCN", "DENTIST DISPCLK DID 0x%X (was 0x%X) -> 0x%X: DAL %u MHz -> actual %u MHz", disp,
            this->lastDispDid, newDisp, didToMHz(this->appleVcoKHz, disp), didToMHz(this->vcoKHz, newDisp));
    }
    if ((dppFromDal && dpp != this->dalDppDid) || newDpp != this->lastDppDid) {
        BCLOG("BC250DCN", "DENTIST DPPCLK DID 0x%X (was 0x%X) -> 0x%X: DAL %u MHz -> actual %u MHz", dpp,
            this->lastDppDid, newDpp, didToMHz(this->appleVcoKHz, dpp), didToMHz(this->vcoKHz, newDpp));
    }
    if (dispFromDal) { this->dalDispDid = disp; }
    if (dppFromDal) { this->dalDppDid = dpp; }
    this->lastDispDid = newDisp;
    this->lastDppDid  = newDpp;
}

// The DAL writes both dividers here, also a value equal to what is programmed (e.g. 0x11 for 144 Hz after 120 Hz's
// 0x15 was corrected to 0x11), which leaves the register unchanged; so both are rescaled from what it left. The lock
// keeps the watcher out between its DISPCLK and DPPCLK writes.
void BC250DCN::wrapUpdateDentist(void* clkMgr, void* context)
{
    auto& s = singleton();
    IOLockLock(s.clockLock);
    FunctionCast(wrapUpdateDentist, s.orgUpdateDentist)(clkMgr, context);
    SInt32 dispKHz, dppKHz;
    memcpy(&dispKHz, static_cast<const UInt8*>(clkMgr) + CLKS_DISPCLK_KHZ, sizeof(dispKHz));
    memcpy(&dppKHz, static_cast<const UInt8*>(clkMgr) + CLKS_DPPCLK_KHZ, sizeof(dppKHz));
    // Its guard returns without writing when either clock is 0 (no displays): then what is there is ours.
    if (dispKHz != 0 && dppKHz != 0) {
        const auto v = NRed::singleton().readReg32(DENTIST_DISPCLK_CNTL);
        s.applyDids(v & DENTIST_DISPCLK_WDIVIDER_MASK,
            (v >> DENTIST_DPPCLK_WDIVIDER_SHIFT) & DENTIST_DISPCLK_WDIVIDER_MASK, true, true);
    }
    IOLockUnlock(s.clockLock);
}

// Catches DAL writes that bypass wrapUpdateDentist (or all of them, if it is not routed). Only a field that moved
// off our last value can be told apart as the DAL's; the DAL writes DISPCLK and DPPCLK separately, so a tick can
// land between the two.
void BC250DCN::clockTick()
{
    IOLockLock(this->clockLock);
    const auto v    = NRed::singleton().readReg32(DENTIST_DISPCLK_CNTL);
    const auto disp = v & DENTIST_DISPCLK_WDIVIDER_MASK;
    const auto dpp  = (v >> DENTIST_DPPCLK_WDIVIDER_SHIFT) & DENTIST_DISPCLK_WDIVIDER_MASK;
    if (disp != this->lastDispDid || dpp != this->lastDppDid) {
        this->applyDids(disp, dpp, disp != this->lastDispDid, dpp != this->lastDppDid);
    }
    IOLockUnlock(this->clockLock);
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
