#pragma once
#include <deque>
#include "core/Types.hpp"
#include "core/Vec2.hpp"
#include "core/Random.hpp"
#include "sim/VehicleProfile.hpp"

namespace tfs::sim {
    struct PerceivedLeaderSample {
        float timestamp{0.0f};
        core::VehicleId leader_id{core::INVALID_ID};
        float net_distance_s{1000.0f};
        float leader_speed{0.0f};
    };

    struct Vehicle {
        core::VehicleId id{core::INVALID_ID};
        VehicleProfile profile;
        bool on_connector{false};
        core::LaneRef current_lane;
        core::ConnectorId current_connector{core::INVALID_ID};
        core::ConnectorId planned_next_connector{core::INVALID_ID};
        float s{0.0f};
        float speed{0.0f};
        float acceleration{0.0f};
        float lateral_offset{0.0f};
        float lane_change_cooldown{0.0f};
        core::OrnsteinUhlenbeckProcess noise_process;
        std::deque<PerceivedLeaderSample> perception_history;
        core::Vec2 world_pos;
        float world_heading{0.0f};
        bool braking_light{false};
        int blinker_state{0};
    };
}
