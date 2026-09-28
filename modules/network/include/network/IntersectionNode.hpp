#pragma once
#include <vector>
#include <unordered_map>
#include "core/Types.hpp"
#include "core/Vec2.hpp"
#include "network/TrafficLight.hpp"

namespace tfs::network {
    enum class IntersectionRule {
        Uncontrolled,
        PriorityYield,
        TrafficLight
    };

    struct IntersectionNode {
        core::NodeId id{core::INVALID_ID};
        core::Vec2 position;
        float radius{12.0f};
        IntersectionRule rule{IntersectionRule::PriorityYield};
        TrafficLightController light_controller;
        std::vector<core::RoadId> connected_roads;
        std::vector<core::ConnectorId> internal_connectors;
        std::unordered_map<core::RoadId, int> road_priority_rank;
    };
}
