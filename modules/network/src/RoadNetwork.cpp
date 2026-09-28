#include "network/RoadNetwork.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace tfs::network {

    namespace {
        constexpr float kPi = std::numbers::pi_v<float>;

        float normalizeAngle(float a) noexcept {
            while (a > kPi) a -= 2.0f * kPi;
            while (a <= -kPi) a += 2.0f * kPi;
            return a;
        }

        bool segmentsIntersect(const core::Vec2& p1, const core::Vec2& p2, const core::Vec2& p3, const core::Vec2& p4,
                                float& t, float& u) noexcept {
            const core::Vec2 d1 = p2 - p1;
            const core::Vec2 d2 = p4 - p3;
            const float denom = d1.cross(d2);
            if (std::abs(denom) < 1e-9f) return false;
            const core::Vec2 diff = p3 - p1;
            t = diff.cross(d2) / denom;
            u = diff.cross(d1) / denom;
            return t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f;
        }

        struct LaneEndpoint {
            core::LaneRef ref;
            core::RoadId road_id;
            core::Vec2 point;
            core::Vec2 direction;
            core::LaneIndex idx;
            core::LaneIndex lane_count;
        };
    }

    core::NodeId RoadNetwork::addNode(core::Vec2 pos, IntersectionRule rule) {
        const core::NodeId id = next_node_id_++;
        IntersectionNode node;
        node.id = id;
        node.position = pos;
        node.rule = rule;
        nodes_.emplace(id, std::move(node));
        return id;
    }

    void RoadNetwork::moveNode(core::NodeId id, core::Vec2 new_pos) {
        auto it = nodes_.find(id);
        if (it == nodes_.end()) return;
        it->second.position = new_pos;

        const std::vector<core::RoadId> connected = it->second.connected_roads;
        for (core::RoadId rid : connected) {
            auto rit = roads_.find(rid);
            if (rit == roads_.end()) continue;
            RoadSegment& road = rit->second;
            const core::Vec2 start_pos = nodes_.at(road.start_node).position;
            const core::Vec2 end_pos = nodes_.at(road.end_node).position;
            road.rebuildGeometry(start_pos, end_pos);
        }

        rebuildIntersection(id);
        for (core::RoadId rid : connected) {
            auto rit = roads_.find(rid);
            if (rit == roads_.end()) continue;
            const core::NodeId other = (rit->second.start_node == id) ? rit->second.end_node : rit->second.start_node;
            if (other != id) rebuildIntersection(other);
        }
    }

    core::NodeId RoadNetwork::addNodeWithId(core::NodeId id, core::Vec2 pos, IntersectionRule rule) {
        IntersectionNode node;
        node.id = id;
        node.position = pos;
        node.rule = rule;
        nodes_.emplace(id, std::move(node));
        next_node_id_ = std::max(next_node_id_, id + 1);
        return id;
    }

    core::RoadId RoadNetwork::addRoadWithId(core::RoadId id, core::NodeId start_node, core::NodeId end_node,
                                             std::uint32_t fwd_lanes, std::uint32_t bwd_lanes, float speed_limit,
                                             bool curved, core::Vec2 p1, core::Vec2 p2) {
        auto start_it = nodes_.find(start_node);
        auto end_it = nodes_.find(end_node);
        if (start_it == nodes_.end() || end_it == nodes_.end()) return core::INVALID_ID;

        RoadSegment road;
        road.id = id;
        road.start_node = start_node;
        road.end_node = end_node;
        road.is_curved = curved;
        road.num_forward_lanes = fwd_lanes;
        road.num_backward_lanes = bwd_lanes;
        road.speed_limit = speed_limit;

        const core::Vec2 start_pos = start_it->second.position;
        const core::Vec2 end_pos = end_it->second.position;
        if (curved) road.baseline_curve = core::CubicBezier(start_pos, p1, p2, end_pos);
        road.rebuildGeometry(start_pos, end_pos);

        roads_.emplace(id, std::move(road));
        start_it->second.connected_roads.push_back(id);
        end_it->second.connected_roads.push_back(id);
        next_road_id_ = std::max(next_road_id_, id + 1);

        return id;
    }

    void RoadNetwork::removeNode(core::NodeId id) {
        auto it = nodes_.find(id);
        if (it == nodes_.end()) return;
        const std::vector<core::RoadId> roads_to_remove = it->second.connected_roads;
        for (core::RoadId rid : roads_to_remove) {
            removeRoad(rid);
        }
        nodes_.erase(id);
    }

    core::RoadId RoadNetwork::addRoad(core::NodeId start_node, core::NodeId end_node, std::uint32_t fwd_lanes,
                                       std::uint32_t bwd_lanes, float speed_limit, bool curved,
                                       core::Vec2 p1, core::Vec2 p2) {
        auto start_it = nodes_.find(start_node);
        auto end_it = nodes_.find(end_node);
        if (start_it == nodes_.end() || end_it == nodes_.end()) return core::INVALID_ID;

        const core::RoadId id = next_road_id_++;
        RoadSegment road;
        road.id = id;
        road.start_node = start_node;
        road.end_node = end_node;
        road.is_curved = curved;
        road.num_forward_lanes = fwd_lanes;
        road.num_backward_lanes = bwd_lanes;
        road.speed_limit = speed_limit;

        const core::Vec2 start_pos = start_it->second.position;
        const core::Vec2 end_pos = end_it->second.position;

        if (curved) {
            road.baseline_curve = core::CubicBezier(start_pos, p1, p2, end_pos);
        }
        road.rebuildGeometry(start_pos, end_pos);

        roads_.emplace(id, std::move(road));
        start_it->second.connected_roads.push_back(id);
        end_it->second.connected_roads.push_back(id);

        rebuildIntersection(start_node);
        if (end_node != start_node) rebuildIntersection(end_node);

        return id;
    }

    void RoadNetwork::removeRoad(core::RoadId id) {
        auto it = roads_.find(id);
        if (it == roads_.end()) return;
        const core::NodeId start_node = it->second.start_node;
        const core::NodeId end_node = it->second.end_node;
        roads_.erase(it);

        auto erase_from_connected = [&](core::NodeId nid) {
            auto nit = nodes_.find(nid);
            if (nit == nodes_.end()) return;
            auto& vec = nit->second.connected_roads;
            vec.erase(std::remove(vec.begin(), vec.end(), id), vec.end());
        };
        erase_from_connected(start_node);
        erase_from_connected(end_node);

        rebuildIntersection(start_node);
        if (end_node != start_node) rebuildIntersection(end_node);
    }

    void RoadNetwork::updateRoadControlPoints(core::RoadId id, core::Vec2 p1, core::Vec2 p2) {
        auto it = roads_.find(id);
        if (it == roads_.end()) return;
        RoadSegment& road = it->second;
        road.is_curved = true;
        const core::Vec2 start_pos = nodes_.at(road.start_node).position;
        const core::Vec2 end_pos = nodes_.at(road.end_node).position;
        road.baseline_curve = core::CubicBezier(start_pos, p1, p2, end_pos);
        road.rebuildGeometry(start_pos, end_pos);

        rebuildIntersection(road.start_node);
        if (road.end_node != road.start_node) rebuildIntersection(road.end_node);
    }

    bool RoadNetwork::hasPriority(const LaneConnector& a, const LaneConnector& b, const IntersectionNode& node) const {
        int rank_a = 0, rank_b = 0;
        if (auto ra = node.road_priority_rank.find(a.from_lane.road_id); ra != node.road_priority_rank.end()) rank_a = ra->second;
        if (auto rb = node.road_priority_rank.find(b.from_lane.road_id); rb != node.road_priority_rank.end()) rank_b = rb->second;

        if (rank_a != rank_b) return rank_a > rank_b;

        if (a.turn_type == TurnDirection::Straight && b.turn_type != TurnDirection::Straight) return true;
        if (b.turn_type == TurnDirection::Straight && a.turn_type != TurnDirection::Straight) return false;

        const float angle_a = a.trajectory.tangent(0.0f).angle();
        const float angle_b = b.trajectory.tangent(0.0f).angle();
        const float diff = normalizeAngle(angle_a - angle_b);
        return diff >= 0.0f;
    }

    bool RoadNetwork::findTrajectoryIntersection(const core::CubicBezier& a, const core::CubicBezier& b,
                                                  float& out_s_a, float& out_s_b) const {
        constexpr int kSamples = 24;
        const float len_a = a.totalLength();
        const float len_b = b.totalLength();
        if (len_a < 1e-6f || len_b < 1e-6f) return false;

        std::array<core::Vec2, kSamples + 1> pts_a{};
        std::array<core::Vec2, kSamples + 1> pts_b{};
        for (int i = 0; i <= kSamples; ++i) {
            pts_a[static_cast<std::size_t>(i)] = a.evaluateAtDistance(len_a * static_cast<float>(i) / kSamples);
            pts_b[static_cast<std::size_t>(i)] = b.evaluateAtDistance(len_b * static_cast<float>(i) / kSamples);
        }

        for (int i = 0; i < kSamples; ++i) {
            for (int j = 0; j < kSamples; ++j) {
                float t, u;
                if (segmentsIntersect(pts_a[static_cast<std::size_t>(i)], pts_a[static_cast<std::size_t>(i + 1)],
                                       pts_b[static_cast<std::size_t>(j)], pts_b[static_cast<std::size_t>(j + 1)], t, u)) {
                    out_s_a = len_a * (static_cast<float>(i) + t) / kSamples;
                    out_s_b = len_b * (static_cast<float>(j) + u) / kSamples;
                    return true;
                }
            }
        }
        return false;
    }

    void RoadNetwork::buildTrafficLightPhases(IntersectionNode& node) {
        std::vector<TrafficLightPhase> phases;

        for (core::ConnectorId cid : node.internal_connectors) {
            const LaneConnector& c = connectors_.at(cid);

            std::size_t chosen_phase = phases.size();
            for (std::size_t i = 0; i < phases.size(); ++i) {
                const bool conflicts = std::any_of(c.conflicts.begin(), c.conflicts.end(), [&](const ConflictPoint& cp) {
                    return std::find(phases[i].active_connectors.begin(), phases[i].active_connectors.end(),
                                      cp.other_connector) != phases[i].active_connectors.end();
                });
                if (!conflicts) {
                    chosen_phase = i;
                    break;
                }
            }

            if (chosen_phase == phases.size()) {
                TrafficLightPhase new_phase;
                new_phase.green_duration = 14.0f;
                new_phase.yellow_duration = 3.0f;
                new_phase.all_red_duration = 2.0f;
                phases.push_back(std::move(new_phase));
            }
            phases[chosen_phase].active_connectors.push_back(cid);
        }

        if (phases.empty()) {
            TrafficLightPhase fallback;
            fallback.green_duration = 14.0f;
            fallback.yellow_duration = 3.0f;
            fallback.all_red_duration = 2.0f;
            phases.push_back(std::move(fallback));
        }

        node.light_controller.phases = std::move(phases);
        node.light_controller.current_phase_idx = 0;
        node.light_controller.phase_timer = 0.0f;
        node.light_controller.current_substate = LightState::Green;
    }

    void RoadNetwork::rebuildIntersection(core::NodeId node_id) {
        auto node_it = nodes_.find(node_id);
        if (node_it == nodes_.end()) return;
        IntersectionNode& node = node_it->second;

        const std::vector<core::ConnectorId> old_connectors = node.internal_connectors;
        for (core::ConnectorId cid : old_connectors) {
            connectors_.erase(cid);
        }
        for (core::RoadId rid : node.connected_roads) {
            auto rit = roads_.find(rid);
            if (rit == roads_.end()) continue;
            auto strip = [&](std::vector<core::ConnectorId>& vec) {
                vec.erase(std::remove_if(vec.begin(), vec.end(), [&](core::ConnectorId c) {
                    return std::find(old_connectors.begin(), old_connectors.end(), c) != old_connectors.end();
                }), vec.end());
            };
            for (auto& lane : rit->second.forward_lanes) { strip(lane.incoming_connectors); strip(lane.outgoing_connectors); }
            for (auto& lane : rit->second.backward_lanes) { strip(lane.incoming_connectors); strip(lane.outgoing_connectors); }
        }
        node.internal_connectors.clear();

        std::vector<LaneEndpoint> incoming;
        std::vector<LaneEndpoint> outgoing;

        for (core::RoadId rid : node.connected_roads) {
            auto rit = roads_.find(rid);
            if (rit == roads_.end()) continue;
            RoadSegment& road = rit->second;

            if (road.end_node == node_id) {
                for (auto& lane : road.forward_lanes) {
                    const float len = lane.length();
                    const float s = std::max(0.0f, len - node.radius);
                    incoming.push_back({lane.ref, rid, lane.center_curve.evaluateAtDistance(s),
                                         lane.center_curve.tangentAtDistance(s), lane.ref.lane_idx, road.num_forward_lanes});
                }
                for (auto& lane : road.backward_lanes) {
                    const float s = std::min(lane.length(), node.radius);
                    outgoing.push_back({lane.ref, rid, lane.center_curve.evaluateAtDistance(s),
                                         lane.center_curve.tangentAtDistance(s), lane.ref.lane_idx, road.num_backward_lanes});
                }
            }
            if (road.start_node == node_id) {
                for (auto& lane : road.forward_lanes) {
                    const float s = std::min(lane.length(), node.radius);
                    outgoing.push_back({lane.ref, rid, lane.center_curve.evaluateAtDistance(s),
                                         lane.center_curve.tangentAtDistance(s), lane.ref.lane_idx, road.num_forward_lanes});
                }
                for (auto& lane : road.backward_lanes) {
                    const float len = lane.length();
                    const float s = std::max(0.0f, len - node.radius);
                    incoming.push_back({lane.ref, rid, lane.center_curve.evaluateAtDistance(s),
                                         lane.center_curve.tangentAtDistance(s), lane.ref.lane_idx, road.num_backward_lanes});
                }
            }
        }

        for (const auto& in_ep : incoming) {
            for (const auto& out_ep : outgoing) {
                if (in_ep.road_id == out_ep.road_id) continue;

                const float delta = normalizeAngle(out_ep.direction.angle() - in_ep.direction.angle());

                TurnDirection turn;
                bool allowed;
                if (delta < -0.35f) {
                    turn = TurnDirection::Right;
                    allowed = (in_ep.idx == 0 && out_ep.idx == 0);
                } else if (delta > 0.35f) {
                    turn = TurnDirection::Left;
                    allowed = (in_ep.idx == in_ep.lane_count - 1 && out_ep.idx == out_ep.lane_count - 1);
                } else {
                    turn = TurnDirection::Straight;
                    allowed = (in_ep.idx == out_ep.idx);
                }
                if (!allowed) continue;

                const float dist = in_ep.point.distanceTo(out_ep.point);
                const core::Vec2 p1 = in_ep.point + in_ep.direction.normalized() * (dist / 3.0f);
                const core::Vec2 p2 = out_ep.point - out_ep.direction.normalized() * (dist / 3.0f);

                LaneConnector connector;
                connector.id = next_connector_id_++;
                connector.parent_node = node_id;
                connector.from_lane = in_ep.ref;
                connector.to_lane = out_ep.ref;
                connector.turn_type = turn;
                connector.trajectory = core::CubicBezier(in_ep.point, p1, p2, out_ep.point);
                connector.speed_limit = 8.33f;

                const core::ConnectorId cid = connector.id;
                connectors_.emplace(cid, std::move(connector));
                node.internal_connectors.push_back(cid);

                if (RoadSegment* from_road = findRoad(in_ep.road_id)) {
                    from_road->getLaneMutable(in_ep.ref).outgoing_connectors.push_back(cid);
                }
                if (RoadSegment* to_road = findRoad(out_ep.road_id)) {
                    to_road->getLaneMutable(out_ep.ref).incoming_connectors.push_back(cid);
                }
            }
        }

        for (std::size_t i = 0; i < node.internal_connectors.size(); ++i) {
            LaneConnector& ci = connectors_.at(node.internal_connectors[i]);
            for (std::size_t j = i + 1; j < node.internal_connectors.size(); ++j) {
                LaneConnector& cj = connectors_.at(node.internal_connectors[j]);
                if (ci.from_lane == cj.from_lane) continue;

                float s_i = 0.0f, s_j = 0.0f;
                if (findTrajectoryIntersection(ci.trajectory, cj.trajectory, s_i, s_j)) {
                    const bool i_has_priority = hasPriority(ci, cj, node);
                    ci.conflicts.push_back({cj.id, s_i, s_j, !i_has_priority});
                    cj.conflicts.push_back({ci.id, s_j, s_i, i_has_priority});
                }
            }
        }

        if (node.rule == IntersectionRule::TrafficLight) {
            buildTrafficLightPhases(node);
        }
    }

    const IntersectionNode* RoadNetwork::findNode(core::NodeId id) const {
        auto it = nodes_.find(id);
        return it != nodes_.end() ? &it->second : nullptr;
    }
    IntersectionNode* RoadNetwork::findNode(core::NodeId id) {
        auto it = nodes_.find(id);
        return it != nodes_.end() ? &it->second : nullptr;
    }
    const RoadSegment* RoadNetwork::findRoad(core::RoadId id) const {
        auto it = roads_.find(id);
        return it != roads_.end() ? &it->second : nullptr;
    }
    RoadSegment* RoadNetwork::findRoad(core::RoadId id) {
        auto it = roads_.find(id);
        return it != roads_.end() ? &it->second : nullptr;
    }
    const LaneConnector* RoadNetwork::findConnector(core::ConnectorId id) const {
        auto it = connectors_.find(id);
        return it != connectors_.end() ? &it->second : nullptr;
    }
    LaneConnector* RoadNetwork::findConnector(core::ConnectorId id) {
        auto it = connectors_.find(id);
        return it != connectors_.end() ? &it->second : nullptr;
    }
    const Lane* RoadNetwork::findLane(const core::LaneRef& ref) const {
        const RoadSegment* road = findRoad(ref.road_id);
        if (!road) return nullptr;
        const std::vector<Lane>& lanes = ref.is_forward ? road->forward_lanes : road->backward_lanes;
        for (const Lane& lane : lanes) {
            if (lane.ref.lane_idx == ref.lane_idx) return &lane;
        }
        return nullptr;
    }
    Lane* RoadNetwork::findLaneMutable(const core::LaneRef& ref) {
        RoadSegment* road = findRoad(ref.road_id);
        if (!road) return nullptr;
        std::vector<Lane>& lanes = ref.is_forward ? road->forward_lanes : road->backward_lanes;
        for (Lane& lane : lanes) {
            if (lane.ref.lane_idx == ref.lane_idx) return &lane;
        }
        return nullptr;
    }
}
