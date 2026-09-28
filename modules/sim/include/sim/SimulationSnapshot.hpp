#pragma once
#include <cstdint>
#include <vector>
#include "core/Types.hpp"
#include "core/Vec2.hpp"
#include "network/TrafficLight.hpp"
#include "sim/VehicleProfile.hpp"

namespace tfs::sim {
    struct VehicleRenderData {
        core::VehicleId id;
        VehicleClass vclass;
        core::Vec2 position;
        float heading;
        float length;
        float width;
        float speed;
        float acceleration;
        bool braking;
        int blinker;
    };

    struct TrafficLightRenderData {
        core::NodeId node_id;
        core::LaneRef lane_ref;
        core::Vec2 stop_line_pos;
        core::Vec2 stop_line_dir;
        network::LightState state;
    };

    struct SimulationSnapshot {
        std::uint64_t tick_count{0};
        double sim_time_sec{0.0};
        std::size_t active_vehicles_count{0};
        float mean_speed_kmh{0.0f};
        std::size_t jammed_vehicles_count{0};
        std::vector<VehicleRenderData> vehicles;
        std::vector<TrafficLightRenderData> traffic_lights;
    };
}
