// ASRock BC-250 (AMD Cyan Skillfish) kernel/user CR3 switches
//
// Copyright © 2026 NootedRed-BC250 contributors. Licensed under the Thou Shalt Not Profit License version 1.5.
// See LICENSE for details.
//
// The kernel's Meltdown double-map swaps CR3 at every syscall/interrupt boundary (the common trap entry loads
// cpu_kernel_cr3, _ks_64bit_return loads cpu_ucr3). CPUs with PCID tag TLB entries so the swap is cheap; this Zen 2
// has no PCID (CPUID leaf 1 ECX bit 17 = 0), so every swap flushes the whole non-global TLB and the next user or
// kernel touches re-walk page tables in GDDR6. Zen CPUs are not Meltdown-vulnerable, so the split buys nothing here.
//
// `sysctl debug.bc250.kpti`: 0 = stock, 1 = the return swap is a NOP (user mode stays on the task's full pmap),
// 2 = the entry swap is a NOP too (only from 1: every thread has already moved to the full pmap). Writing 0 restores
// the stock bytes. Nothing is patched at load; a crash while enabled costs one power cycle and the next boot is
// stock again. User mode then runs with the kernel mapped (supervisor pages stay inaccessible without Meltdown);
// accepted on this board for the same reason Linux never enables KPTI on AMD.
#pragma once
#include <Headers/kern_patcher.hpp>

class BC250Kpti {
public:
    static BC250Kpti& singleton();
    void processPatcher(KernelPatcher& patcher);
};
