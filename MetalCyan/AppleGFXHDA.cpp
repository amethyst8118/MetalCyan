// AppleGFXHDA (GPU HDMI/DP audio) patches for the ASRock BC-250
//
// Ported from NootedRed's AppleGFXHDA.cpp (Copyright © 2022-2025 ChefKiss) for the BC-250's audio
// function (PCI 1002:13FF at 01:00.1, ATI 0x1002:0xAA01-family codec). Licensed under the Thou
// Shalt Not Profit License version 1.5. See LICENSE for details.
//
// What was adapted and why:
// - Device gate retargeted from Raven/Renoir (0x15DE/0x1637) to the BC-250 audio function 0x13FF,
//   read from PCI config space (not a property) so an EFI device-id spoof cannot confuse it. The
//   personality injects IOClass AppleGFXHDAEGController (the class Apple uses for Navi 10's AB38,
//   a discrete-style function like 13FF) instead of NootedRed's AppleGFXHDAController. The EG
//   class defines no probe/start override in 26.7.1 (nm shows only ctor/dtor plus association
//   helpers), so routing the base AppleGFXHDAController::probe covers it — no separate EG route.
// - The HDAU provider rename plus built-in and hda-gfx=onboard-1 are kept verbatim: AppleGFXHDA
//   compares an ancestor node name against "HDAU" for head association, and hda-gfx is matched
//   value-agnostically against the GPU-side property MetalCyan already sets (onboard-1).
// - NootedRed's controller parameter block is kept: every forced ivar offset is live in the 550.1
//   disassembly, and the EG class adds no ivars of its own, so it shares the base layout. The
//   values are the proven Raven/Renoir set on the same ATI-HDMI codec family; Apple's EG defaults
//   are untested on 13FF, so forcing is the conservative choice.
// - Factory forcing (Tahiti function group + 1002AAA0 widget for codec vendor 0x1002) is REQUIRED,
//   not optional: the 550.1 factory falls through to a NULL return with a Sound assertion for
//   subsystem words outside its dGPU Tahiti set. NootedRed's widget-fallback bug is fixed here —
//   the fallback calls the widget original, not the function-group original.
// - Everything is gated on BC250::isActive() (Cyan Skillfish) and every solve/route failure is
//   non-fatal (BCLOG + skip, never PANIC), so other boards and other audio functions are untouched.

#include <AppleGFXHDA.hpp>
#include <BC250.hpp>
#include <Headers/kern_iokit.hpp>
#include <Headers/kern_patcher.hpp>
#include <Headers/kern_util.hpp>
#include <IOKit/pci/IOPCIDevice.h>
#include <Kexts.hpp>
#include <libkern/OSTypes.h>
#include <libkern/c++/OSMetaClass.h>

static constexpr UInt32 AMDVendorID = 0x1002;
static constexpr UInt32 BC250HDMIDeviceID = 0x13FF;

static AppleGFXHDA moduleInstance;

AppleGFXHDA &AppleGFXHDA::singleton() { return moduleInstance; }

void AppleGFXHDA::processKext(KernelPatcher &patcher, size_t id, mach_vm_address_t slide, size_t size) {
    if (kextAppleGFXHDA.loadIndex != id) { return; }
    // Containment: only the BC-250 gets any AppleGFXHDA routing.
    if (!BC250::singleton().isActive()) { return; }

    // Solve the forcing targets. Non-fatal: without them the factories simply fall back to the
    // originals (which return NULL for this codec), while the probe path still applies.
    singleton().orgFunctionGroupTahiti =
        patcher.solveSymbol<OSMetaClass *>(id, "__ZN34AppleGFXHDAFunctionGroupATI_Tahiti10gMetaClassE", slide, size);
    if (singleton().orgFunctionGroupTahiti == nullptr) {
        BCLOG("AGFXHDA", "Failed to solve FunctionGroupATI_Tahiti metaclass; function-group forcing disabled");
        patcher.clearError();
    }
    singleton().orgWidget1002AAA0 =
        patcher.solveSymbol<OSMetaClass *>(id, "__ZN26AppleGFXHDAWidget_1002AAA010gMetaClassE", slide, size);
    if (singleton().orgWidget1002AAA0 == nullptr) {
        BCLOG("AGFXHDA", "Failed to solve Widget_1002AAA0 metaclass; widget forcing disabled");
        patcher.clearError();
    }

    KernelPatcher::RouteRequest probeRequest {"__ZN21AppleGFXHDAController5probeEP9IOServicePi", wrapProbe,
        singleton().orgProbe};
    if (!patcher.routeMultiple(id, &probeRequest, 1, slide, size)) {
        BCLOG("AGFXHDA", "Failed to route probe; audio device left unpatched");
        patcher.clearError();
        return;
    }

    KernelPatcher::RouteRequest groupRequest {
        "__ZN31AppleGFXHDAFunctionGroupFactory27createAppleHDAFunctionGroupEP11DevIdStruct",
        wrapCreateAppleHDAFunctionGroup, singleton().orgCreateAppleHDAFunctionGroup};
    if (!patcher.routeMultiple(id, &groupRequest, 1, slide, size)) {
        BCLOG("AGFXHDA", "Failed to route function-group factory; Tahiti forcing disabled");
        patcher.clearError();
    }

    KernelPatcher::RouteRequest widgetRequest {"__ZN24AppleGFXHDAWidgetFactory20createAppleHDAWidgetEP11DevIdStruct",
        wrapCreateAppleHDAWidget, singleton().orgCreateAppleHDAWidget};
    if (!patcher.routeMultiple(id, &widgetRequest, 1, slide, size)) {
        BCLOG("AGFXHDA", "Failed to route widget factory; 1002AAA0 forcing disabled");
        patcher.clearError();
    }
}

IOService *AppleGFXHDA::wrapProbe(IOService *that, IOService *provider, SInt32 *score) {
    const auto dev = OSDynamicCast(IOPCIDevice, provider);
    if (dev == nullptr) { return FunctionCast(wrapProbe, singleton().orgProbe)(that, provider, score); }

    // PCI config space, not a property: an EFI device-id spoof must not change this decision.
    const auto vendorID = WIOKit::readPCIConfigValue(dev, WIOKit::kIOPCIConfigVendorID);
    const auto deviceID = WIOKit::readPCIConfigValue(dev, WIOKit::kIOPCIConfigDeviceID);
    if (vendorID != AMDVendorID || deviceID != BC250HDMIDeviceID) {
        DBGLOG("GFXHDA", "Not the BC-250 audio function, calling original.");
        return FunctionCast(wrapProbe, singleton().orgProbe)(that, provider, score);
    }

    // Load-bearing: AppleGFXHDA associates heads by the ancestor node name "HDAU".
    provider->setName("HDAU");

    UInt8 bytes[] = {0x00};
    dev->setProperty("built-in", bytes, sizeof(bytes));

    that->setProperty("MetalCyanGFXHDA", bytes, sizeof(bytes));

    singleton().controller.provider(that) = provider;
    singleton().controller.vendorID(that) = vendorID;
    singleton().controller.deviceID(that) = deviceID;
    singleton().controller.integratedCodecAddressMask(that) = 0x1;
    singleton().controller.codecAddressMask(that) = 0x1;
    singleton().controller.useRirb(that) = true;
    singleton().controller.timeIntervalFilterOrder(that) = 9;
    singleton().controller.intIndex(that) = 1;
    singleton().controller.inputSampleLatency(that) = 1;
    singleton().controller.outputSampleLatency(that) = 1;
    singleton().controller.inputSafetyOffset(that) = 0x28;
    singleton().controller.outputSafetyOffset(that) = 0x28;
    singleton().controller.inputSafetyOffsetLowPower(that) = 0x28;
    singleton().controller.outputSafetyOffsetLowPower(that) = 0x28;
    singleton().controller.inputEntrySize(that) = 0x3000;
    singleton().controller.outputEntrySize(that) = 0x3000;
    singleton().controller.regAccessReady(that) = true;

    *score = 1;

    char hdaGfxBytes[] = "onboard-1";
    dev->setProperty("hda-gfx", hdaGfxBytes, sizeof(hdaGfxBytes));

    DBGLOG("GFXHDA", "Initialised the BC-250 audio function.");

    return that;
}

void *AppleGFXHDA::wrapCreateAppleHDAFunctionGroup(void *devId) {
    // Selection keys off the HDA codec DevIdStruct, not the PCI ID (offsets verified in 550.1).
    const auto vendorID = getMember<UInt16>(devId, 0x2);
    if (vendorID == AMDVendorID && singleton().orgFunctionGroupTahiti != nullptr) {
        return singleton().orgFunctionGroupTahiti->alloc();
    }
    return FunctionCast(wrapCreateAppleHDAFunctionGroup, singleton().orgCreateAppleHDAFunctionGroup)(devId);
}

void *AppleGFXHDA::wrapCreateAppleHDAWidget(void *devId) {
    const auto vendorID = getMember<UInt16>(devId, 0x2);
    if (vendorID == AMDVendorID && singleton().orgWidget1002AAA0 != nullptr) {
        return singleton().orgWidget1002AAA0->alloc();
    }
    // NOTE: NootedRed's fallback calls the function-group original here; the widget original is correct.
    return FunctionCast(wrapCreateAppleHDAWidget, singleton().orgCreateAppleHDAWidget)(devId);
}
