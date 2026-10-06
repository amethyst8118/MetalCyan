// Master Logic
//
// Copyright © 2022-2025 ChefKiss. Licensed under the Thou Shalt Not Profit License version 1.5.
// See LICENSE for details.

#include <AGDP.hpp>
#include <AppleGFXHDA.hpp>
#include <BC250.hpp>
#include <BC250HWL.hpp>
#include <DriverInjector.hpp>
#include <GPUDriversAMD/ATOMBIOS.hpp>
#include <GPUDriversAMD/CAIL/Result.hpp>
#include <GPUDriversAMD/SMU.hpp>
#include <GPUDriversAMD/TTL/SWIP/SMU.hpp>
#include <Headers/kern_api.hpp>
#include <Headers/kern_devinfo.hpp>
#include <Headers/kern_iokit.hpp>
#include <Headers/kern_patcher.hpp>
#include <Headers/kern_util.hpp>
#include <IOKit/IOLib.h>
#include <IOKit/IOTypes.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <Kexts.hpp>
#include <NRed.hpp>
#include <PenguinWizardry/RuntimeMC.hpp>
#include <Regs/CyanSkillfish.hpp>
#include <Regs/GC.hpp>
#include <Regs/NBIO.hpp>
#include <Regs/SMU.hpp>
#include <X6000FB.hpp>
#include <kern/clock.h>
#include <libkern/OSTypes.h>
#include <libkern/c++/OSMetaClass.h>
#include <mach/i386/vm_types.h>

static NRed moduleInstance;

NRed& NRed::singleton() { return moduleInstance; }

void NRed::init()
{
    SYSLOG("MetalCyan", "MetalCyan, based on NootedRed (c) 2022-2025 ChefKiss");

    lilu.onKextLoadForce(&kextRadeonX6000Framebuffer);
    lilu.onKextLoadForce(&kextRadeonX6000HWServices);
    lilu.onKextLoadForce(&kextRadeonX6000HWLibs);
    lilu.onKextLoadForce(&kextRadeonX6000);
    lilu.onKextLoadForce(&kextAGDP);
    lilu.onKextLoadForce(&kextAppleGFXHDA);
    lilu.onPatcherLoadForce(
        [](void* const, KernelPatcher& patcher)
        {
            singleton().processPatcher();
            DriverInjector::singleton().processPatcher(patcher);
            PenguinWizardry::RuntimeMCManager::singleton().processPatcher(patcher);
        },
        nullptr);

    lilu.onKextLoadForce(
        nullptr, 0,
        [](void* const, KernelPatcher& patcher, const size_t id, const mach_vm_address_t slide, const size_t size)
        {
            AGDP::singleton().processKext(patcher, id, slide, size);
            X6000FB::singleton().processKext(patcher, id, slide, size);
            BC250HWL::singleton().processKext(patcher, id, slide, size);
            AppleGFXHDA::singleton().processKext(patcher, id, slide, size);
        },
        nullptr);
}

void NRed::mapMMIO()
{
    this->iGPU->setMemoryEnable(true);
    this->iGPU->setBusMasterEnable(true);

    this->rmmio =
        this->iGPU->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress5, kIOMapInhibitCache | kIOMapAnywhere);
    PANIC_COND(this->rmmio == nullptr || this->rmmio->getLength() == 0, "NRed", "Failed to map RMMIO");
    this->rmmioPtr = reinterpret_cast<volatile UInt32*>(this->rmmio->getVirtualAddress());
}

void NRed::hwLateInitCyanSkillfish()
{
    namespace CS = CyanSkillfish;

    this->fbOffset    = static_cast<UInt64>(this->readReg32(CS::GCMC_VM_FB_OFFSET) & 0xFFFFFF) << 24;
    this->devRevision = (this->readReg32(CS::RCC_DEV0_EPF0_STRAP0) & CS::RCC_DEV0_EPF0_STRAP0_ATI_REV_ID_MASK)
                        >> CS::RCC_DEV0_EPF0_STRAP0_ATI_REV_ID_SHIFT;
    // Linux `nv.c`: GC 10.1.3 uses `external_rev_id = rev_id + 0x82`.
    this->enumRevision = 0x82;

    SYSLOG("NRed", "BC-250: deviceID = 0x%X pciRevision = 0x%X devRevision = 0x%X fbOffset = 0x%llX",
           this->deviceID, this->pciRevision, this->devRevision, this->fbOffset);
}

void NRed::hwLateInit()
{
    if (this->rmmio != nullptr) { return; }

    this->mapMMIO();

    this->hwLateInitCyanSkillfish();
}

void NRed::processPatcher()
{
    const auto devInfo = DeviceInfo::create();
    assert(devInfo != nullptr);

    devInfo->processSwitchOff();

    PANIC_COND(devInfo->videoBuiltin == nullptr, "NRed", "No iGPU detected by Lilu");
    this->iGPU = OSDynamicCast(IOPCIDevice, devInfo->videoBuiltin);
    PANIC_COND(WIOKit::readPCIConfigValue(this->iGPU, WIOKit::kIOPCIConfigVendorID) != WIOKit::VendorID::ATIAMD, "NRed",
               "iGPU is not an AMD one");

    WIOKit::renameDevice(this->iGPU, "IGPU");
    WIOKit::awaitPublishing(this->iGPU);
    UInt8 builtInBytes[] = {0x00};
    this->iGPU->setProperty("built-in", builtInBytes, sizeof(builtInBytes));
    char slotNameBytes[] = "built-in";
    this->iGPU->setProperty("AAPL,slot-name", slotNameBytes, sizeof(slotNameBytes));
    char hdaGfxBytes[] = "onboard-1";
    this->iGPU->setProperty("hda-gfx", hdaGfxBytes, sizeof(hdaGfxBytes));

    this->deviceID = static_cast<UInt16>(WIOKit::readPCIConfigValue(this->iGPU, WIOKit::kIOPCIConfigDeviceID));
    switch (this->deviceID) {
        case 0x13FE:      // ASRock BC-250
        case 0x143F: {    // Other Cyan Skillfish 2 SKU
            this->attributes.setCyanSkillfish();
        } break;
        default: PANIC("NRed", "Not a Cyan Skillfish GPU: 0x%X", this->deviceID);
    }
    this->pciRevision = static_cast<UInt8>(WIOKit::readPCIConfigValue(this->iGPU, WIOKit::kIOPCIConfigRevisionID));

    char name[128];
    for (size_t i = 0, ii = 0; i < devInfo->videoExternal.size(); i++) {
        auto device = OSDynamicCast(IOPCIDevice, devInfo->videoExternal[i].video);
        if (device == nullptr) { continue; }

        snprintf(name, arrsize(name), "GFX%zu", ii++);
        WIOKit::renameDevice(device, name);
        WIOKit::awaitPublishing(device);
    }

    DeviceInfo::deleter(devInfo);

    if (this->attributes.isCyanSkillfish()) { BC250::singleton().processPatcher(); }
}

void NRed::setProp32(const char* const key, const UInt32 value) const { this->iGPU->setProperty(key, value, 32); }

void NRed::writeReg32(const UInt32 reg, const UInt32 value) const
{
    PANIC_COND((reg * sizeof(UInt32)) >= this->rmmio->getLength(), "NRed", "Register 0x%X is outside of RMMIO", reg);
    this->rmmioPtr[reg] = value;
}

// SMN access through the NBIO PCIE_INDEX2/PCIE_DATA2 pair (byte address).
UInt32 NRed::readSMN32(const UInt32 address) const
{
    this->rmmioPtr[CyanSkillfish::PCIE_INDEX2] = address;
    (void)this->rmmioPtr[CyanSkillfish::PCIE_INDEX2];
    return this->rmmioPtr[CyanSkillfish::PCIE_DATA2];
}

// Port of `amdgpu_device_mm_access` (read direction): indirect VRAM access through MM_INDEX/MM_DATA.
// Works regardless of how much of the carve-out the BAR0 aperture covers.
bool NRed::readVRAM(const UInt64 offset, void* const buffer, const size_t size) const
{
    if ((offset % sizeof(UInt32)) != 0 || (size % sizeof(UInt32)) != 0 || buffer == nullptr) { return false; }

    auto* const out   = static_cast<UInt32*>(buffer);
    UInt32      hiVal = ~0U;
    for (size_t i = 0; i < size / sizeof(UInt32); i += 1) {
        const auto pos = offset + i * sizeof(UInt32);
        this->rmmioPtr[CyanSkillfish::MM_INDEX] = static_cast<UInt32>(pos) | 0x80000000;
        const auto hi = static_cast<UInt32>(pos >> 31);
        if (hi != hiVal) {
            this->rmmioPtr[CyanSkillfish::MM_INDEX_HI] = hi;
            hiVal                                      = hi;
        }
        out[i] = this->rmmioPtr[CyanSkillfish::MM_DATA];
    }
    return true;
}

UInt32 NRed::readReg32(const UInt32 reg) const
{
    if ((reg * sizeof(UInt32)) < this->rmmio->getLength()) { return this->rmmioPtr[reg]; }
    else {
        this->rmmioPtr[PCIE_INDEX2] = reg;
        return this->rmmioPtr[PCIE_DATA2];
    }
}
