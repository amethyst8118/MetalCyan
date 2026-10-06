// IOKit Personality Injector
//
// Copyright © 2025 ChefKiss. Licensed under the Thou Shalt Not Profit License version 1.5.
// See LICENSE for details.

#include <BC250.hpp>
#include <DriverInjector.hpp>
#include <Headers/kern_patcher.hpp>
#include <Headers/kern_util.hpp>
#include <NRed.hpp>
#include <libkern/OSTypes.h>
#include <libkern/c++/OSArray.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSMetaClass.h>
#include <libkern/c++/OSObject.h>
#include <libkern/c++/OSString.h>

static DriverInjector moduleInstance;

static const char bc250_com_apple_kext_AMDRadeonX6000Framebuffer[] = {
#embed "Personalities/BC250/com.apple.kext.AMDRadeonX6000Framebuffer.xml" suffix(, '\0')
};
static const char bc250_com_apple_kext_AMDRadeonX6000HWServices[] = {
#embed "Personalities/BC250/com.apple.kext.AMDRadeonX6000HWServices.xml" suffix(, '\0')
};
static const char bc250_com_apple_driver_AppleGFXHDA[] = {
#embed "Personalities/BC250/com.apple.driver.AppleGFXHDA.xml" suffix(, '\0')
};

DriverInjector::DriverInjector() :
    bc250Drivers{
        Driver("com.apple.kext.AMDRadeonX6000Framebuffer", bc250_com_apple_kext_AMDRadeonX6000Framebuffer),
        Driver("com.apple.kext.AMDRadeonX6000HWServices", bc250_com_apple_kext_AMDRadeonX6000HWServices),
        Driver("com.apple.driver.AppleGFXHDA", bc250_com_apple_driver_AppleGFXHDA),
    }
{ }

DriverInjector& DriverInjector::singleton() { return moduleInstance; }

void DriverInjector::processPatcher(KernelPatcher& patcher)
{
    KernelPatcher::RouteRequest request{"__ZN11IOCatalogue10addDriversEP7OSArrayb", wrapAddDrivers,
                                        this->orgAddDrivers};
    PANIC_COND(!patcher.routeMultipleLong(KernelPatcher::KernelID, &request, 1), "DriverInjector",
               "Failed to route addDrivers");
}

// A copy of AMDRadeonX6000's own Navi 10 accelerator personality, matching the BC-250.
// Copying keeps every property Apple ships with it (Metal/GL plug-in names and so on) on any macOS version.
static void addBC250AcceleratorPersonality(OSArray* const array)
{
    const UInt32 count = array->getCount();
    for (UInt32 i = 0; i < count; i += 1) {
        auto* dict = OSDynamicCast(OSDictionary, array->getObject(i));
        if (dict == nullptr) { continue; }
        auto* ioClass = OSDynamicCast(OSString, dict->getObject("IOClass"));
        if (ioClass == nullptr || !ioClass->isEqualTo("AMDRadeonX6000_AMDNavi10GraphicsAccelerator")) { continue; }

        auto* copy = OSDictionary::withDictionary(dict);
        if (copy == nullptr) { return; }
        copy->removeObject("IOPCIPrimaryMatch");
        copy->removeObject("IOPCISecondaryMatch");
        copy->removeObject("IOPCIClassMatch");
        copy->removeObject("IONameMatch");
        auto* match = OSString::withCString("0x13FE1002 0x143F1002");
        copy->setObject("IOPCIMatch", match);
        OSSafeReleaseNULL(match);
        array->setObject(copy);
        OSSafeReleaseNULL(copy);
        return;
    }
    BCLOG("DriverInjector", "BC-250: no Navi 10 accelerator personality in AMDRadeonX6000");
}

bool DriverInjector::wrapAddDrivers(void* const self, OSArray* const array, const bool doNubMatching)
{
    if (NRed::singleton().getAttributes().isCyanSkillfish() && BC250::singleton().isActive() &&
        !singleton().bc250AccelInjected)
    {
        for (UInt32 i = 0; i < array->getCount(); i += 1) {
            auto* dict = OSDynamicCast(OSDictionary, array->getObject(i));
            auto* id   = dict == nullptr ? nullptr : OSDynamicCast(OSString, dict->getObject("CFBundleIdentifier"));
            if (id != nullptr && id->isEqualTo("com.apple.kext.AMDRadeonX6000")) {
                singleton().bc250AccelInjected = true;
                addBC250AcceleratorPersonality(array);
                break;
            }
        }
    }

    UInt32 driverCount = array->getCount();
    for (UInt32 driverIndex = 0; driverIndex < driverCount; driverIndex += 1) {
        OSObject* object = array->getObject(driverIndex);
        if (object == nullptr) { continue; }
        auto* dict = OSDynamicCast(OSDictionary, object);
        if (dict == nullptr) { continue; }
        auto* bundleIdentifier = OSDynamicCast(OSString, dict->getObject("CFBundleIdentifier"));
        if (bundleIdentifier == nullptr || bundleIdentifier->getLength() == 0) { continue; }

        Driver* toInject = singleton().bc250Drivers;
        // The framebuffer, HWServices and AppleGFXHDA.
        size_t toInjectCount = BC250::singleton().isActive() ? 3 : 0;
        for (size_t identifierIndex = 0; identifierIndex < toInjectCount; identifierIndex += 1) {
            auto& driver = toInject[identifierIndex];

            if ((singleton().matchedDrivers & getBit(identifierIndex)) != 0
                || !bundleIdentifier->isEqualTo(driver.identifier))
            {
                continue;
            }

            singleton().matchedDrivers |= getBit(identifierIndex);

            DBGLOG("DriverInjector", "Matched %s, injecting.", driver.identifier);

            array->merge(driver.personalities);

            break;
        }
    }

    return FunctionCast(wrapAddDrivers, singleton().orgAddDrivers)(self, array, doNubMatching);
}
