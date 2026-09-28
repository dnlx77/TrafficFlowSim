#pragma once
#include "sim/VehicleProfile.hpp"

namespace tfs::sim {
    class IDMModel {
    public:
        static float computeFreeFlowAcceleration(const VehicleProfile& prof, float current_speed, float speed_limit) noexcept;
        static float computeAcceleration(const VehicleProfile& prof, float current_speed, float speed_limit,
                                          float net_distance_s, float approach_rate_dv, float ou_noise = 0.0f) noexcept;
    };
}
