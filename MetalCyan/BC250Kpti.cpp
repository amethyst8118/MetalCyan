// ASRock BC-250 (AMD Cyan Skillfish) kernel/user CR3 switches
//
// Copyright © 2026 NootedRed-BC250 contributors. Licensed under the Thou Shalt Not Profit License version 1.5.
// See LICENSE for details.

#include <BC250.hpp>
#include <BC250Kpti.hpp>
#include <i386/proc_reg.h>
#include <sys/errno.h>
#include <sys/sysctl.h>

static BC250Kpti moduleInstance;

BC250Kpti& BC250Kpti::singleton() { return moduleInstance; }

namespace
{
    // Live Darwin 25.6.0 (26.7.1) bytes (the boot code rewrites the trampolines, so only live bytes count). Each
    // anchor includes the preceding instruction because the bare swap sequence occurs at more than one site:
    //   return (_ks_64bit_return): movq 72(%r15), %r15 ; movq %gs:248, %rax ; movq %rax, %cr3   (cpu_ucr3)
    //   entry (common trap entry): movq 16(%rax), %rax ; movq %gs:256(%rax), %rax ; movq %rax, %cr3   (cpu_kernel_cr3)
    // The patch covers the load + CR3 write after the anchor instruction (12 / 11 bytes).
    constexpr UInt8 kRetAnchor[]   = {0x4D, 0x8B, 0x7F, 0x48, 0x65, 0x48, 0x8B, 0x04, 0x25, 0xF8, 0x00, 0x00, 0x00, 0x0F, 0x22, 0xD8};
    constexpr UInt8 kEntryAnchor[] = {0x48, 0x8B, 0x40, 0x10, 0x65, 0x48, 0x8B, 0x80, 0x00, 0x01, 0x00, 0x00, 0x0F, 0x22, 0xD8};
    constexpr UInt8 kRetSwap[]     = {0x65, 0x48, 0x8B, 0x04, 0x25, 0xF8, 0x00, 0x00, 0x00, 0x0F, 0x22, 0xD8};
    constexpr UInt8 kEntrySwap[]   = {0x65, 0x48, 0x8B, 0x80, 0x00, 0x01, 0x00, 0x00, 0x0F, 0x22, 0xD8};
    constexpr UInt8 kNops[12]      = {0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90};

    void (*rendezvous)(void (*)(void*), void*) = nullptr;
    mach_vm_address_t retAddr = 0, entryAddr = 0;
    bool available = false, retPatched = false, entryPatched = false;
    int  level = 0;
    SInt32 cpuCount = 0;

    void serialize()
    {
        UInt32 a = 0, b = 0, c = 0, d = 0;
        __asm__ volatile("cpuid" : "=a"(a), "=b"(b), "=c"(c), "=d"(d) : "a"(0));
    }

    struct CountPass {
        volatile SInt32 cpus;
    };

    void countOnCpu(void* const arg) { OSIncrementAtomic(&static_cast<CountPass*>(arg)->cpus); }

    struct PatchPass {
        volatile SInt32 arrived;
        volatile SInt32 patched;
        SInt32          expected;
        mach_vm_address_t addr[2];
        const UInt8*    bytes[2];
        size_t          len[2];
        bool            ok;
    };

    // Runs on every CPU at once with interrupts off. Everyone waits until every CPU is inside (none is executing
    // the trampolines), CPU 0 rewrites the bytes with write-protect off, then everyone serialises before leaving.
    void patchOnCpu(void* const arg)
    {
        auto* const pass = static_cast<PatchPass*>(arg);
        const auto  slot = OSIncrementAtomic(&pass->arrived);
        while (pass->arrived < pass->expected) { __asm__ volatile("pause"); }
        if (slot == 0) {
            const auto cr0 = get_cr0();
            set_cr0(cr0 & ~static_cast<uintptr_t>(CR0_WP));
            for (int i = 0; i < 2; i++) {
                if (pass->addr[i] != 0) { memcpy(reinterpret_cast<void*>(pass->addr[i]), pass->bytes[i], pass->len[i]); }
            }
            set_cr0(cr0);
            pass->ok = true;
            __sync_synchronize();
            pass->patched = 1;
        }
        while (pass->patched == 0) { __asm__ volatile("pause"); }
        serialize();
    }

    bool applyState(const int target)
    {
        const bool wantRet   = target >= 1;
        const bool wantEntry = target >= 2;
        if (wantRet == retPatched && wantEntry == entryPatched) {
            level = target;
            return true;
        }
        if (cpuCount == 0) {
            CountPass count {};
            rendezvous(countOnCpu, &count);
            cpuCount = count.cpus;
            if (cpuCount <= 0) { return false; }
        }
        PatchPass pass {};
        pass.expected = cpuCount;
        int n = 0;
        if (wantRet != retPatched) {
            pass.addr[n]  = retAddr;
            pass.bytes[n] = wantRet ? kNops : kRetSwap;
            pass.len[n]   = sizeof(kRetSwap);
            n++;
        }
        if (wantEntry != entryPatched) {
            pass.addr[n]  = entryAddr;
            pass.bytes[n] = wantEntry ? kNops : kEntrySwap;
            pass.len[n]   = sizeof(kEntrySwap);
            n++;
        }
        rendezvous(patchOnCpu, &pass);
        if (!pass.ok) { return false; }
        retPatched   = wantRet;
        entryPatched = wantEntry;
        level        = target;
        BCLOG("BC250KPTI", "CR3 swaps level %d on %d CPUs (return %s, entry %s)", level, cpuCount,
            retPatched ? "off" : "stock", entryPatched ? "off" : "stock");
        return true;
    }

    int sysctlHandleKpti(struct sysctl_oid* const oidp, void*, int, struct sysctl_req* const req)
    {
        if (!available) { return ENOTSUP; }
        int        value = level;
        const auto error = sysctl_handle_int(oidp, &value, 0, req);
        if (error != 0 || req->newptr == USER_ADDR_NULL) { return error; }
        if (value < 0 || value > 2) { return EINVAL; }
        if (value == 2 && level == 0) {
            // The entry swap can only go once every thread has moved to the full pmap via the return path.
            BCLOG("BC250KPTI", "kpti=2 from 0: applying level 1 first; write 2 again for the entry swap");
            value = 1;
        }
        return applyState(value) ? 0 : EIO;
    }
}    // namespace

SYSCTL_DECL(_debug_bc250);
SYSCTL_PROC(_debug_bc250, OID_AUTO, kpti, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, nullptr, 0, sysctlHandleKpti,
    "I", "BC-250 kernel/user CR3 swaps (0 stock, 1 no return swap, 2 no swaps)");

void BC250Kpti::processPatcher(KernelPatcher& patcher)
{
    rendezvous = reinterpret_cast<decltype(rendezvous)>(
        patcher.solveSymbol(KernelPatcher::KernelID, "_mp_rendezvous_no_intrs"));
    const auto ksReturn = patcher.solveSymbol(KernelPatcher::KernelID, "_ks_64bit_return");
    if (rendezvous == nullptr || ksReturn == 0) {
        BCLOG("BC250KPTI", "symbols not found; debug.bc250.kpti unavailable");
        patcher.clearError();
        return;
    }
    // Offsets from the symbol drift with jump encodings, so scan bounded windows for the exact swap sequences.
    const auto findSwap = [](mach_vm_address_t from, mach_vm_address_t to, const UInt8* pat, size_t len) {
        mach_vm_address_t found = 0;
        for (auto at = from; at + len <= to; at++) {
            if (memcmp(reinterpret_cast<const void*>(at), pat, len) == 0) {
                if (found != 0) { return static_cast<mach_vm_address_t>(0); }
                found = at;
            }
        }
        return found;
    };
    retAddr = findSwap(ksReturn, ksReturn + 0x120, kRetAnchor, sizeof(kRetAnchor));
    // The entry anchor also occurs at a nearby rdmsr site; the trap entry is the one preceded by swapgs (at -10).
    for (auto at = ksReturn - 0x400; at + sizeof(kEntryAnchor) <= ksReturn; at++) {
        if (memcmp(reinterpret_cast<const void*>(at), kEntryAnchor, sizeof(kEntryAnchor)) == 0 &&
            memcmp(reinterpret_cast<const void*>(at - 10), "\x0F\x01\xF8", 3) == 0)
        {
            entryAddr = at;
            break;
        }
    }
    if (retAddr != 0) { retAddr += 4; }
    if (entryAddr != 0) { entryAddr += 4; }
    if (retAddr == 0 || entryAddr == 0) {
        // The boot code rewrites these trampolines, so the live bytes are the only truth. Dump both windows.
        const auto dump = [](mach_vm_address_t at, size_t len) {
            const auto* const b = reinterpret_cast<const UInt8*>(at);
            for (size_t i = 0; i < len; i += 24) {
                BCLOG("BC250KPTI", "%#lx: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                    at + i, b[i], b[i + 1], b[i + 2], b[i + 3], b[i + 4], b[i + 5], b[i + 6], b[i + 7], b[i + 8],
                    b[i + 9], b[i + 10], b[i + 11], b[i + 12], b[i + 13], b[i + 14], b[i + 15], b[i + 16], b[i + 17],
                    b[i + 18], b[i + 19], b[i + 20], b[i + 21], b[i + 22], b[i + 23]);
            }
        };
        dump(ksReturn, 0xC0);
        dump(ksReturn - 0x280, 0x100);
        BCLOG("BC250KPTI", "CR3 swap sequences not found near _ks_64bit_return (%#lx); debug.bc250.kpti unavailable",
            ksReturn);
        return;
    }
    available = true;
    sysctl_register_oid(&sysctl__debug_bc250_kpti);
    BCLOG("BC250KPTI", "CR3 swaps at %#lx (return) and %#lx (entry); sysctl debug.bc250.kpti", retAddr, entryAddr);
    UInt32 bootLevel = 0;
    if (PE_parse_boot_argn("bc250kpti", &bootLevel, sizeof(bootLevel)) && bootLevel >= 1 && bootLevel <= 2) {
        if (applyState(1) && bootLevel == 2) { applyState(2); }
        BCLOG("BC250KPTI", "bc250kpti=%u: level %d", bootLevel, level);
    }
}
