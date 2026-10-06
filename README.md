# MetalCyan

Metal acceleration for the ASRock BC-250 on macOS. The BC-250's GPU (AMD Cyan Skillfish: GC 10.1.3, DCN 2.0.1) is
close enough to Navi 10 that Apple's own AMDRadeonX6000 drivers can run it, with a lot of patching. MetalCyan is the
Lilu plugin that does that patching. It started as a fork of [NootedRed](https://github.com/ChefKissInc/NootedRed).

![System Information](docs/images/system-information.jpg)

> [!IMPORTANT]
> MetalCyan's patches are matched to **macOS Tahoe 26.7.1** with the **MacPro7,1** SMBIOS (OpenCore 1.0.8). Other
> macOS versions and other SMBIOS models aren't supported.

For a complete EFI, see [BC-250-Hackintosh-OpenCore](https://github.com/amethyst8118/BC-250-Hackintosh-OpenCore).

## What works

- Metal 3, full acceleration (WindowServer, Safari, Firefox, Metal apps)
- 4K at 60 Hz
- All 4 GB of VRAM usable and CPU-visible
- GPU clock, temperature and load in Activity Monitor, iStat and similar tools
- 40 CU unlock and GPU/CPU overclocking (boot-args, below)

## What doesn't

- **Hardware video decode/encode.** VCN can't be used on this chip: AMD says VCN was never part of the BC-250's
  product definition, the SMU firmware has no way to power it up and there's no signed VCN firmware, so Linux
  doesn't use it either. macOS decodes in software. 4K video plays fine in Firefox.
- **HDMI/DP audio.** The GPU's audio function isn't set up yet. Safari and the TV app refuse to play video without an
  audio output device; a virtual one (BlackHole etc.) gets around it.
- **GPU recovery.** Apple's reset path is Navi 10's and hangs this GPU, so it's blocked. If the GPU hangs, the screen
  freezes until you reboot. It hasn't happened since the VRAM fixes, but there's no way back from it.
- **Shutdown/restart.** WindowServer panics on the way down (`display_mode_did_change ... returns false`). It doesn't
  affect the next boot. Not fixed yet.
- **Sleep.** Not tested.

## Requirements

- macOS Tahoe 26.7.1, MacPro7,1 SMBIOS.
- BIOS: UMA frame buffer (VRAM) at **4 GB**. With the default 512 MB the GPU runs out of memory and freezes on a
  green screen.
- [Lilu](https://github.com/acidanthera/Lilu) 1.7 or newer, loaded before MetalCyan.
- No other GPU kext (WhateverGreen, NootedRed, NootRX).

## Install

Download `MetalCyan-1.0.1-RELEASE.zip` from [Releases](https://github.com/amethyst8118/MetalCyan/releases/tag/v1.0.1)
(or build it, below). Copy `MetalCyan.kext` to `EFI/OC/Kexts` and add it to `Kernel > Add` after Lilu. The EFI repo uses `npci=0x3000`
in boot-args as well.

## Boot-args

| Boot-arg | |
|---|---|
| `-MCOff` | Don't load. You get the firmware framebuffer, no acceleration. |
| `-MCDebug` | Lilu debug logging (DEBUG builds). |
| `-MCBeta` | Load on macOS versions newer than the ones tested. |
| `bc250cu=40` | Enable all 40 CUs (stock is 24). Only applied if every shader array reads the stock harvest value. |
| `bc250gfxmhz=N` | GPU clock, 350-2000 MHz. Above 2000 the SMU firmware stops answering, so higher values are ignored. |
| `bc250gfxmv=N` | GPU voltage, 700-1100 mV, used with `bc250gfxmhz`. At most 50 mV below the stock curve for that clock. |
| `bc250cpumhz=N` | CPU boost clock, 3500-4500 MHz. Always comes with an undervolt worked out from `bc250cpuvmax`. |
| `bc250cpuvmax=N` | CPU voltage ceiling, 950-1325 mV (default 1275). |
| `bc250cputemp=N` | CPU temperature limit, 50-100 °C (default 90 when `bc250cpumhz` is set). |
| `bc250gputemp=N` | GPU temperature at which the forced GPU clock is released, 50-100 °C (default 90). |
| `bc250cores=8` | Ask the SMU for all 8 cores; they'd show up after a Restart. Untested; use the OpenCore driver in the EFI repo instead. |
| `bc250smu=0` | No SMU access at all (no telemetry, no tuning). |
| `bc250dispmhz=N` | Display clock ceiling, 1000-1200 MHz (stock: the GOP's VCO/2.5). For 4K at 120 Hz. 1200 is Linux's limit for DCN 2.0.1. |

Clocks and voltages are applied 60 seconds after boot, so a bad setting can't stop the machine from booting: you can
always get to the desktop and take it out again. If the SMU stops responding (telemetry frozen, CPU stuck at one
clock), remove the setting and power off for 10 seconds. A restart isn't enough.

## 4K at 120 Hz

Untested. Without compression, the BC-250's one DisplayPort 1.4 output carries 4K at 120 Hz only at 8-bit colour
with a reduced-blanking (CVT-RB2) timing, about 1076 MHz pixel clock; CTA timings (1188 MHz) and 10-bit don't fit
the link. MetalCyan keeps each display on one pipe, so the display clock has to carry the whole pixel rate: check
`sysctl -n debug.bc250.log` for the `DENTIST VCO` line, and if the ceiling is below ~1080 MHz add
`bc250dispmhz=1200`. The monitor's EDID must offer a 4K 120 Hz reduced-blanking mode, otherwise macOS won't list
it and an EDID override is needed. Use a certified DP 1.4 (HBR3) cable. If the screen goes black, remove the arg
(or boot with -MCOff). Unlike the SMU settings above, `bc250dispmhz` takes effect at boot, so a bad value is
reverted by removing the arg in the OpenCore picker or config.

DSC: Linux's DCN 2.0.1 driver claims no DSC engines, but the chip has two at Navi 10's DSC0/DSC1 addresses. The
[linux-cachyos-bc250](https://github.com/MastaG/linux-cachyos-bc250) `0010-dcn201-enable-dsc` patch turns them on
and runs 4K 120 Hz RGB with DSC into a DP to HDMI 2.1 adapter. Apple's Navi 10 display code (26.7.1) always creates
six DSC engines at Navi 10's addresses, so DSC0/DSC1 land on the real ones; whether it uses them on the BC-250 is
untested.

DP to HDMI 2.1 adapters: the picture still crosses the BC-250's DP 1.4 link, and HDMI TVs expect the CTA timing
(1188 MHz). That needs DSC (the adapter must advertise it; the Cable Matters 102101 / VMM7100 on firmware 7.02.120
reports none, and its firmware updater runs on any Windows PC with the adapter plugged in) or YCbCr 4:2:0. Either way
the display clock at 1188 MHz needs a VCO of about 3.0 GHz (at 2.67 GHz the steps are 1068 and 1187 MHz). Apple's
display code has the YCbCr 4:2:0 and DP to HDMI 2.1 (FRL) adapter paths; untested on the BC-250.

## Logs

- `sysctl -n debug.bc250.log`: MetalCyan's own log since boot. Only errors and warnings, plus the SMU tuning lines.
  The DENTIST VCO and clock ceiling and each display-clock change are logged there too.
  On a normal boot you'll also see `createPspDirectory call not found` and `PSP: firmware load (handle 10) not done:
  2`. Both are harmless.
- `sysctl -n debug.bc250.smu`: live SMU telemetry (GPU clock and voltage, CPU voltage, Tctl, per-core clocks).

## How it works

Apple's driver is three kexts, and each needs something different.

**Framebuffer (AMDRadeonX6000Framebuffer).** It gets a BC-250 entry in its ASIC tables and three fixes for DCN 2.0.1:

- Scanout and cursor addresses go out as the carve-out's system address, since DCN 2.0.1 has no DCN_VM aperture.
- The display clock dividers are rescaled for the BC-250's DENTIST VCO.
- MPC pipe splits are stopped: DCN 2.0.1 has 4 pipes, and Navi 10's DAL splits onto pipe 5.

**HWServices / HWLibs (TTL, CAIL).** The device tables and IP version gates accept the BC-250. The SMU, VCN, JPEG,
DMCU and MES blocks are stubbed: Linux either leaves them alone on this chip (VCN and JPEG stay powered off, there's
no DMCU, GFX10 runs without MES) or drives them with code Apple doesn't have (SMU 11.0.8). PSP loads the cyan_skillfish2 microcode
(bundled from linux-firmware) instead of Navi 10's, and MetalCyan waits for those loads to finish before the RLC
starts. GVM, GC and SDMA are programmed the way Linux's amdgpu does it on Cyan Skillfish:

- the FB offset is added to page-table and fault addresses
- Cyan Skillfish golden settings
- Linux's RLC start sequence
- no clock or power gating

**Accelerator (AMDRadeonX6000).**

- **VM:** page tables, PTEs and VM programs pointing into VRAM get the carve-out's physical address. TLB invalidations
  are sent in a form this GPU acknowledges, both over MMIO and in the SDMA/PM4 rings. The fault controls match Linux.
- **KIQ:** SET_RESOURCES is sent as Linux sends it.
- **Compute:** compute work runs on the GFX ring. GFX1013's MEC doesn't execute dispatches reliably, and Linux
  doesn't use it either.
- **SDMA1:** its trap enable and completion events are fixed up so its work doesn't sit unnoticed.
- **VRAM:** all 4 GB is made CPU-visible, and the reuse cache that leaked VRAM until the screen went green is off.
- **Blocked:** Navi 10's reset and hang-dump paths, and VCN's properties, so macOS doesn't try a hardware decoder that
  isn't there.

**SMU.** PowerPlay is blocked: Apple's SMU code speaks Navi 10's message set, which the BC-250's SMU 11.0.8 reads
differently. MetalCyan talks to the SMU directly through the same mailbox the BC-250 community tools use, for
telemetry and the tuning boot-args.

## Building

    git clone --recursive https://github.com/amethyst8118/MetalCyan.git
    cd MetalCyan
    make CONFIG=Release zip

Needs clang 19 or newer (for `#embed`) and ld64. On Linux that's cctools-port; on macOS, point `LD64`/`STRIP` at
Xcode's. The zip ends up in `build-linux/Release/`.

## Credits

- [ChefKiss](https://github.com/ChefKissInc) for NootedRed, which this is built on.
- [Acidanthera](https://github.com/acidanthera) for Lilu and MacKernelSDK.
- Linux's amdgpu driver, the reference for nearly every hardware fix here.
- linux-firmware for the cyan_skillfish2 microcode (`MetalCyan/Firmware`, under `LICENSE.amdgpu`).
- The SMU tooling, all of which the SMU code follows (facts only, no code copied from the 40 CU unlock):
  - [bc250-collective/bc250_smu_oc](https://github.com/bc250-collective/bc250_smu_oc): its `bc250_smu` package for
    the mailbox, and the CPU voltage model
  - [rw-r-r-0644/bc250-core-unlock](https://github.com/rw-r-r-0644/bc250-core-unlock) for the core unlock
  - [rw-r-r-0644/bc250-smu-unlock](https://github.com/rw-r-r-0644/bc250-smu-unlock) for the SMU firmware patches the
    EFI's unlock driver applies
  - [duggasco/bc250-40cu-unlock](https://github.com/duggasco/bc250-40cu-unlock) for the 40 CU unlock

## License

Thou Shalt Not Profit License 1.5, same as NootedRed. See `LICENSE`. The firmware in `MetalCyan/Firmware` is AMD's,
under `LICENSE.amdgpu`.
