// Global Kext Info Definitions
//
// Copyright © 2025 ChefKiss. Licensed under the Thou Shalt Not Profit License version 1.5.
// See LICENSE for details.

#include <Headers/kern_patcher.hpp>
#include <Kexts.hpp>

static const char* pathRadeonX6000Framebuffer =
    "/System/Library/Extensions/AMDRadeonX6000Framebuffer.kext/Contents/MacOS/AMDRadeonX6000Framebuffer";
static const char* pathRadeonX6000HWServices =
    "/System/Library/Extensions/AMDRadeonX6000HWServices.kext/Contents/MacOS/AMDRadeonX6000HWServices";
static const char* pathRadeonX6000HWLibs = "/System/Library/Extensions/AMDRadeonX6000HWServices.kext/Contents/PlugIns/"
                                           "AMDRadeonX6000HWLibs.kext/Contents/MacOS/AMDRadeonX6000HWLibs";
static const char* pathRadeonX6000 = "/System/Library/Extensions/AMDRadeonX6000.kext/Contents/MacOS/AMDRadeonX6000";
static const char* pathAGDP        = "/System/Library/Extensions/AppleGraphicsControl.kext/Contents/PlugIns/"
                                     "AppleGraphicsDevicePolicy.kext/Contents/MacOS/AppleGraphicsDevicePolicy";
static const char* pathAppleGFXHDA = "/System/Library/Extensions/AppleGFXHDA.kext/Contents/MacOS/AppleGFXHDA";

KernelPatcher::KextInfo kextRadeonX6000Framebuffer{
    "com.apple.kext.AMDRadeonX6000Framebuffer", &pathRadeonX6000Framebuffer, 1, {true}, {},
    KernelPatcher::KextInfo::Unloaded,
};

KernelPatcher::KextInfo kextRadeonX6000HWServices{
    "com.apple.kext.AMDRadeonX6000HWServices", &pathRadeonX6000HWServices, 1, {true}, {},
    KernelPatcher::KextInfo::Unloaded,
};

KernelPatcher::KextInfo kextRadeonX6000HWLibs{
    "com.apple.kext.AMDRadeonX6000HWLibs", &pathRadeonX6000HWLibs, 1, {true}, {}, KernelPatcher::KextInfo::Unloaded,
};

KernelPatcher::KextInfo kextRadeonX6000{
    "com.apple.kext.AMDRadeonX6000", &pathRadeonX6000, 1, {true}, {}, KernelPatcher::KextInfo::Unloaded,
};

KernelPatcher::KextInfo kextAGDP{
    "com.apple.driver.AppleGraphicsDevicePolicy", &pathAGDP, 1, {true}, {}, KernelPatcher::KextInfo::Unloaded,
};

KernelPatcher::KextInfo kextAppleGFXHDA{
    "com.apple.driver.AppleGFXHDA", &pathAppleGFXHDA, 1, {true}, {}, KernelPatcher::KextInfo::Unloaded,
};
