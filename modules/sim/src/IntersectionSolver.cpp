#include "sim/IntersectionSolver.hpp"
#include <algorithm>
#include "network/RoadNetwork.hpp"

namespace tfs::sim {

    bool IntersectionSolver::computeVirtualObstacle(const Vehicle& ego, const network::RoadNetwork& net,
        const std::unordered_map<core::ConnectorId, std::vector<const Vehicle*>>& connector_occ,
        const std::unordered_map<core::LaneRef, std::vector<const Vehicle*>>& lane_occ,
        float& out_obstacle_dist) noexcept {

        if (ego.on_connector) return false;
        if (ego.planned_next_connector == core::INVALID_ID) return false;

        const network::LaneConnector* connector = net.findConnector(ego.planned_next_connector);
        if (!connector) return false;

        const network::Lane* lane = net.findLane(ego.current_lane);
        if (!lane) return false;

        const network::IntersectionNode* node = net.findNode(connector->parent_node);
        if (!node) return false;

        // The lane's drivable length is set back by the node radius: the real stop line
        // sits at (lane->length() - node->radius), not at the lane's raw end. When the lane
        // forks into several connectors (e.g. straight and left from a shared inner lane),
        // add extra clearance so a queued vehicle doesn't sit exactly where a car taking the
        // other branch still has its rear overlapping the shared hand-off zone.
        constexpr float kForkSafetyMargin = 8.0f;
        const float margin = lane->outgoing_connectors.size() > 1 ? kForkSafetyMargin : 0.0f;
        const float stop_line_s = std::max(0.0f, lane->length() - node->radius - margin);
        const float dist_to_stop = stop_line_s - ego.s;
        if (dist_to_stop > 80.0f || dist_to_stop < 0.0f) return false;

        bool obstacle = false;

        if (node->rule == network::IntersectionRule::TrafficLight) {
            const network::LightState state = node->light_controller.getStateForConnector(connector->id);
            if (state == network::LightState::Red) {
                obstacle = true;
            } else if (state == network::LightState::Yellow) {
                const float required_decel = (ego.speed * ego.speed) / (2.0f * std::max(dist_to_stop, 0.01f));
                obstacle = required_decel <= ego.profile.comfort_decel;
            }
        } else {
            for (const network::ConflictPoint& cp : connector->conflicts) {
                if (!cp.other_has_priority) continue;

                const network::LaneConnector* other_conn = net.findConnector(cp.other_connector);
                if (!other_conn) continue;

                if (auto lane_it = lane_occ.find(other_conn->from_lane); lane_it != lane_occ.end()) {
                    const network::Lane* other_lane = net.findLane(other_conn->from_lane);
                    if (other_lane) {
                        const float other_stop_line_s = std::max(0.0f, other_lane->length() - node->radius);
                        for (const Vehicle* other : lane_it->second) {
                            const float dist_to_conflict = (other_stop_line_s - other->s) + cp.other_s;
                            const float ttc = other->speed > 0.1f ? (dist_to_conflict / other->speed) : 1000.0f;
                            if (ttc < 3.5f) { obstacle = true; break; }
                        }
                    }
                }
                if (obstacle) break;

                if (auto conn_it = connector_occ.find(cp.other_connector); conn_it != connector_occ.end()) {
                    for (const Vehicle* other : conn_it->second) {
                        const float dist_to_conflict = cp.other_s - other->s;
                        if (dist_to_conflict < 0.0f) continue;
                        const float ttc = other->speed > 0.1f ? (dist_to_conflict / other->speed) : 1000.0f;
                        if (ttc < 3.5f) { obstacle = true; break; }
                    }
                }
                if (obstacle) break;
            }
        }

        if (obstacle) {
            out_obstacle_dist = dist_to_stop;
            return true;
        }
        return false;
    }
}
