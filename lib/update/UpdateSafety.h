#pragma once

#include <cstdint>
#include <cstring>

namespace FirmwareUpdate {
enum class VehicleCondition { Unknown, Parked, Active };

inline VehicleCondition vehicleCondition(const char* state) {
    if (!state) return VehicleCondition::Unknown;
    if (std::strcmp(state, "Parked") == 0 || std::strcmp(state, "DisplaySleep") == 0)
        return VehicleCondition::Parked;
    if (std::strcmp(state, "Running") == 0 || std::strcmp(state, "Idle") == 0 ||
        std::strcmp(state, "StartStop") == 0) return VehicleCondition::Active;
    return VehicleCondition::Unknown;
}

// Unknown is not parked. A bench update requires an explicit, authenticated
// confirmation that the device is disconnected from the vehicle. Never saved.
class UpdateSafety {
public:
    static constexpr uint32_t StatusFreshMs = 3000;
    static constexpr uint32_t MaintenanceDurationMs = 10 * 60 * 1000;

    void observe(VehicleCondition condition, uint32_t nowMs) {
        condition_ = condition;
        observedAt_ = nowMs;
        observed_ = true;
        if (condition == VehicleCondition::Active) maintenance_ = false;
    }
    bool maintenance(uint32_t nowMs) const {
        return maintenance_ && nowMs - maintenanceAt_ < MaintenanceDurationMs;
    }
    bool enableMaintenance(uint32_t nowMs, bool disconnectedConfirmed) {
        if (!disconnectedConfirmed || active(nowMs)) return false;
        maintenance_ = true;
        maintenanceAt_ = nowMs;
        return true;
    }
    void disableMaintenance() { maintenance_ = false; }
    bool blocked(uint32_t nowMs) const {
        if (active(nowMs)) return true;
        if (maintenance(nowMs)) return false;
        return !fresh(nowMs) || condition_ != VehicleCondition::Parked;
    }

private:
    bool fresh(uint32_t nowMs) const { return observed_ && nowMs - observedAt_ <= StatusFreshMs; }
    bool active(uint32_t nowMs) const { return fresh(nowMs) && condition_ == VehicleCondition::Active; }
    VehicleCondition condition_ = VehicleCondition::Unknown;
    uint32_t observedAt_ = 0;
    uint32_t maintenanceAt_ = 0;
    bool observed_ = false;
    bool maintenance_ = false;
};

inline UpdateSafety updateSafety;
} // namespace FirmwareUpdate
