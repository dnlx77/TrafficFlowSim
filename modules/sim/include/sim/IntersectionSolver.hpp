#pragma once
#include <unordered_map>
#include <vector>
#include "core/Types.hpp"
#include "sim/Vehicle.hpp"

namespace tfs::network {
    class RoadNetwork;
}

namespace tfs::sim {
    class IntersectionSolver {
    public:
        static bool computeVirtualObstacle(const Vehicle& ego, const network::RoadNetwork& net,
                                            const std::unordered_map<core::ConnectorId, std::vector<const Vehicle*>>& connector_occ,
                                            const std::unordered_map<core::LaneRef, std::vector<const Vehicle*>>& lane_occ,
                                            float& out_obstacle_dist) noexcept;
    };
}
