#include "sim/IDMModel.hpp"
#include <cmath>
#include <algorithm>

namespace tfs::sim {

    float IDMModel::computeFreeFlowAcceleration(const VehicleProfile& prof, float current_speed, float speed_limit) noexcept {
        const float v0_star = std::min(prof.desired_speed, speed_limit);
        const float ratio = v0_star > 1e-6f ? (current_speed / v0_star) : 0.0f;
        const float accel = prof.max_accel * (1.0f - ratio * ratio * ratio * ratio);
        return std::clamp(accel, -prof.max_decel, prof.max_accel);
    }

    float IDMModel::computeAcceleration(const VehicleProfile& prof, float current_speed, float speed_limit,
                                         float net_distance_s, float approach_rate_dv, float ou_noise) noexcept {
        const float v0_star = std::min(prof.desired_speed, speed_limit);
        const float v = current_speed;
        const float dv = approach_rate_dv;

        const float sqrt_ab = std::sqrt(prof.max_accel * prof.comfort_decel);
        const float s_star = prof.min_gap + std::max(0.0f, v * prof.time_headway + (v * dv) / (2.0f * sqrt_ab));

        const float gap = std::max(net_distance_s, 0.05f);
        const float speed_ratio = v0_star > 1e-6f ? (v / v0_star) : 0.0f;
        const float gap_ratio = s_star / gap;

        const float noise = (v < 0.1f) ? 0.0f : ou_noise;

        const float accel = prof.max_accel * (1.0f - speed_ratio * speed_ratio * speed_ratio * speed_ratio
                                                - gap_ratio * gap_ratio) + noise;

        return std::clamp(accel, -prof.max_decel, prof.max_accel);
    }
}
