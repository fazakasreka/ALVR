#pragma once

#include "TrackedDevice.h"
#include "bindings.h"
#include "openvr_driver_wrap.h"
#include <string>

class FakeViveTracker : public TrackedDevice {
public:
    FakeViveTracker(uint64_t deviceID);
    FakeViveTracker(
        uint64_t deviceID, std::string registeredDeviceType, std::string controllerType
    );
    void OnPoseUpdated(uint64_t targetTimestampNs, const FfiDeviceMotion* motion);

private:
    std::string registered_device_type;
    std::string controller_type = "vive_tracker_waist";

    // TrackedDevice
    bool activate() final;
    void* get_component(const char*) final { return nullptr; }
};
