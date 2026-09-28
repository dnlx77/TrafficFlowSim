#include "sim/SimulationWorld.hpp"
#include <algorithm>
#include <limits>
#include <utility>
#include "sim/IDMModel.hpp"
#include "sim/MOBILModel.hpp"
#include "sim/IntersectionSolver.hpp"

namespace tfs::sim {

    namespace {
        std::pair<const Vehicle*, const Vehicle*> neighborsInSorted(const std::vector<const Vehicle*>& sorted, const Vehicle* self) {
            const Vehicle* leader = nullptr;
            const Vehicle* follower = nullptr;
            for (std::size_t i = 0; i < sorted.size(); ++i) {
                if (sorted[i] == self) {
                    if (i + 1 < sorted.size()) leader = sorted[i + 1];
                    if (i > 0) follower = sorted[i - 1];
                    break;
                }
            }
            return {leader, follower};
        }

        const Vehicle* firstVehicleInBucket(const std::vector<const Vehicle*>& bucket) {
            if (bucket.empty()) return nullptr;
            return *std::min_element(bucket.begin(), bucket.end(),
                                      [](const Vehicle* a, const Vehicle* b) { return a->s < b->s; });
        }

        struct LeaderLookaheadResult {
            const Vehicle* leader{nullptr};
            float gap{0.0f};
        };

        // Finds the nearest vehicle ahead of `ego`, crossing lane -> connector -> lane
        // boundaries when nothing is found within ego's own current segment. Without this,
        // a stopped vehicle near the end of a short segment would be invisible to a follower
        // that has already progressed onto the next segment, letting it drive straight through.
        LeaderLookaheadResult findLeaderAhead(const Vehicle& ego, const network::RoadNetwork& net,
            const std::unordered_map<core::LaneRef, std::vector<const Vehicle*>>& lane_occ,
            const std::unordered_map<core::ConnectorId, std::vector<const Vehicle*>>& connector_occ) {

            constexpr float kMaxLookahead = 150.0f;
            constexpr int kMaxHops = 5;

            if (!ego.on_connector) {
                if (auto it = lane_occ.find(ego.current_lane); it != lane_occ.end()) {
                    if (const Vehicle* leader = neighborsInSorted(it->second, &ego).first) {
                        return {leader, leader->s - leader->profile.length - ego.s};
                    }
                }
            } else {
                if (auto it = connector_occ.find(ego.current_connector); it != connector_occ.end()) {
                    if (const Vehicle* leader = neighborsInSorted(it->second, &ego).first) {
                        return {leader, leader->s - leader->profile.length - ego.s};
                    }
                }
            }

            float accumulated = 0.0f;
            core::LaneRef cur_lane{};
            core::ConnectorId cur_connector = core::INVALID_ID;
            bool on_conn;

            if (!ego.on_connector) {
                const network::Lane* lane = net.findLane(ego.current_lane);
                if (!lane) return {};

                // A shared lane can feed several outgoing connectors (e.g. straight and
                // left from the same inner lane). They all hand off at the same physical
                // point, so a vehicle that just entered ANY of them can still be blocking
                // the shared zone even if ego is headed for a different one.
                core::ConnectorId primary_cid = ego.planned_next_connector;
                if (primary_cid == core::INVALID_ID && !lane->outgoing_connectors.empty()) {
                    primary_cid = lane->outgoing_connectors.front();
                }

                float radius_setback = 0.0f;
                if (primary_cid != core::INVALID_ID) {
                    if (const network::LaneConnector* pc = net.findConnector(primary_cid)) {
                        if (const network::IntersectionNode* pnode = net.findNode(pc->parent_node)) {
                            radius_setback = pnode->radius;
                        }
                    }
                }
                // Mirrors IntersectionSolver's fork safety margin: when this lane feeds
                // multiple connectors, treat the hand-off point as a bit further back so a
                // vehicle already reacts before it would otherwise overlap traffic that just
                // entered a different branch of the same fork.
                constexpr float kForkSafetyMargin = 8.0f;
                const float margin = lane->outgoing_connectors.size() > 1 ? kForkSafetyMargin : 0.0f;
                const float trigger_s = std::max(0.0f, lane->length() - radius_setback - margin);
                accumulated = trigger_s - ego.s;

                const Vehicle* best_leader = nullptr;
                float best_gap = std::numeric_limits<float>::max();
                for (core::ConnectorId cid : lane->outgoing_connectors) {
                    if (auto it = connector_occ.find(cid); it != connector_occ.end()) {
                        if (const Vehicle* candidate = firstVehicleInBucket(it->second)) {
                            const float gap = accumulated + candidate->s - candidate->profile.length;
                            if (gap < best_gap) {
                                best_gap = gap;
                                best_leader = candidate;
                            }
                        }
                    }
                }
                if (best_leader) return {best_leader, best_gap};

                cur_connector = primary_cid;
                on_conn = true;
            } else {
                const network::LaneConnector* connector = net.findConnector(ego.current_connector);
                if (!connector) return {};
                accumulated = connector->trajectory.totalLength() - ego.s;
                cur_lane = connector->to_lane;
                on_conn = false;
            }

            for (int hop = 0; hop < kMaxHops && accumulated < kMaxLookahead; ++hop) {
                if (on_conn) {
                    if (cur_connector == core::INVALID_ID) break;
                    const network::LaneConnector* connector = net.findConnector(cur_connector);
                    if (!connector) break;

                    if (auto it = connector_occ.find(cur_connector); it != connector_occ.end()) {
                        if (const Vehicle* leader = firstVehicleInBucket(it->second)) {
                            return {leader, accumulated + leader->s - leader->profile.length};
                        }
                    }
                    accumulated += connector->trajectory.totalLength();
                    cur_lane = connector->to_lane;
                    on_conn = false;
                } else {
                    const network::Lane* lane = net.findLane(cur_lane);
                    if (!lane) break;

                    if (auto it = lane_occ.find(cur_lane); it != lane_occ.end()) {
                        if (const Vehicle* leader = firstVehicleInBucket(it->second)) {
                            return {leader, accumulated + leader->s - leader->profile.length};
                        }
                    }
                    accumulated += lane->length();
                    cur_connector = lane->outgoing_connectors.empty() ? core::INVALID_ID : lane->outgoing_connectors.front();
                    on_conn = true;
                }
            }

            return {};
        }
    }

    void SimulationWorld::buildOccupancy(std::unordered_map<core::LaneRef, std::vector<const Vehicle*>>& lane_occ,
                                          std::unordered_map<core::ConnectorId, std::vector<const Vehicle*>>& connector_occ) {
        lane_occ.clear();
        connector_occ.clear();
        for (auto& [id, v] : vehicles_) {
            if (v.on_connector) connector_occ[v.current_connector].push_back(&v);
            else lane_occ[v.current_lane].push_back(&v);
        }
        for (auto& [ref, vec] : lane_occ) {
            std::sort(vec.begin(), vec.end(), [](const Vehicle* a, const Vehicle* b) { return a->s < b->s; });
        }
        for (auto& [cid, vec] : connector_occ) {
            std::sort(vec.begin(), vec.end(), [](const Vehicle* a, const Vehicle* b) { return a->s < b->s; });
        }
    }

    void SimulationWorld::updateTrafficLights(float dt,
        const std::unordered_map<core::ConnectorId, std::vector<const Vehicle*>>& connector_occ) {

        constexpr float kMaxClearanceHold = 15.0f;

        for (auto& [id, node] : network_.nodesMutable()) {
            if (node.rule != network::IntersectionRule::TrafficLight) continue;
            network::TrafficLightController& ctrl = node.light_controller;
            if (ctrl.phases.empty()) continue;

            if (ctrl.current_substate == network::LightState::Red) {
                const network::TrafficLightPhase& phase = ctrl.phases[ctrl.current_phase_idx];
                const bool about_to_advance = (ctrl.phase_timer + dt) >= phase.all_red_duration;

                if (about_to_advance) {
                    bool clear = true;
                    for (core::ConnectorId cid : phase.active_connectors) {
                        if (auto it = connector_occ.find(cid); it != connector_occ.end() && !it->second.empty()) {
                            clear = false;
                            break;
                        }
                    }

                    float& hold = intersection_clearance_hold_[id];
                    if (!clear && hold < kMaxClearanceHold) {
                        hold += dt;
                        continue;
                    }
                    hold = 0.0f;
                }
            }

            ctrl.update(dt);
        }
    }

    void SimulationWorld::updateLaneChanges(float dt, std::unordered_map<core::LaneRef, std::vector<const Vehicle*>>& lane_occ) {
        for (auto& [id, v] : vehicles_) {
            if (v.lane_change_cooldown > 0.0f) {
                v.lane_change_cooldown = std::max(0.0f, v.lane_change_cooldown - dt);
                continue;
            }
            if (v.on_connector) continue;

            const network::RoadSegment* road = network_.findRoad(v.current_lane.road_id);
            if (!road) continue;
            const std::uint32_t lane_count = v.current_lane.is_forward ? road->num_forward_lanes : road->num_backward_lanes;
            if (lane_count <= 1) continue;

            auto cur_it = lane_occ.find(v.current_lane);
            const std::vector<const Vehicle*> empty_vec;
            const std::vector<const Vehicle*>& cur_vec = (cur_it != lane_occ.end()) ? cur_it->second : empty_vec;
            auto [cur_lead, cur_fol] = neighborsInSorted(cur_vec, &v);

            LaneChangeCandidate best;
            best.feasible = false;

            for (int delta : {-1, 1}) {
                const int target_idx_signed = static_cast<int>(v.current_lane.lane_idx) + delta;
                if (target_idx_signed < 0 || target_idx_signed >= static_cast<int>(lane_count)) continue;
                const core::LaneIndex target_idx = static_cast<core::LaneIndex>(target_idx_signed);
                const core::LaneRef target_ref{v.current_lane.road_id, v.current_lane.is_forward, target_idx};

                const network::Lane* target_lane_ptr = network_.findLane(target_ref);
                if (!target_lane_ptr) continue;

                auto tgt_it = lane_occ.find(target_ref);
                const std::vector<const Vehicle*>& target_vec = (tgt_it != lane_occ.end()) ? tgt_it->second : empty_vec;

                const Vehicle* tgt_lead = nullptr;
                const Vehicle* tgt_fol = nullptr;
                for (const Vehicle* other : target_vec) {
                    if (other->s <= v.s) tgt_fol = other;
                    else { tgt_lead = other; break; }
                }

                const LaneChangeCandidate candidate = MOBILModel::evaluate(v, cur_lead, cur_fol, tgt_lead, tgt_fol,
                                                                             target_idx, target_lane_ptr->speed_limit);
                if (candidate.feasible && (!best.feasible || candidate.incentive > best.incentive)) {
                    best = candidate;
                }
            }

            if (best.feasible) {
                if (auto old_it = lane_occ.find(v.current_lane); old_it != lane_occ.end()) {
                    auto& old_vec = old_it->second;
                    old_vec.erase(std::remove(old_vec.begin(), old_vec.end(), &v), old_vec.end());
                }

                v.current_lane.lane_idx = best.target_lane_idx;
                v.lane_change_cooldown = 3.0f;
                // The planned connector belongs to the lane just left behind; force a fresh
                // pick against the new lane's own outgoing connectors.
                v.planned_next_connector = core::INVALID_ID;

                auto& new_vec = lane_occ[v.current_lane];
                new_vec.push_back(&v);
                std::sort(new_vec.begin(), new_vec.end(), [](const Vehicle* a, const Vehicle* b) { return a->s < b->s; });
            }
        }
    }

    void SimulationWorld::updateVehicleDynamics(float dt,
        const std::unordered_map<core::LaneRef, std::vector<const Vehicle*>>& lane_occ,
        const std::unordered_map<core::ConnectorId, std::vector<const Vehicle*>>& connector_occ) {

        for (auto& [id, v] : vehicles_) {
            float speed_limit = 13.89f;
            if (v.on_connector) {
                if (const network::LaneConnector* connector = network_.findConnector(v.current_connector)) {
                    speed_limit = connector->speed_limit;
                }
            } else if (const network::Lane* lane = network_.findLane(v.current_lane)) {
                speed_limit = lane->speed_limit;
            }

            const LeaderLookaheadResult lookahead = findLeaderAhead(v, network_, lane_occ, connector_occ);
            const Vehicle* real_leader = lookahead.leader;

            float net_distance_s = real_leader ? lookahead.gap : 1000.0f;
            float leader_speed = real_leader ? real_leader->speed : speed_limit;
            core::VehicleId leader_id = real_leader ? real_leader->id : core::INVALID_ID;

            float obstacle_dist = 0.0f;
            if (IntersectionSolver::computeVirtualObstacle(v, network_, connector_occ, lane_occ, obstacle_dist)) {
                if (obstacle_dist < net_distance_s) {
                    net_distance_s = obstacle_dist;
                    leader_speed = 0.0f;
                    leader_id = core::INVALID_ID;
                }
            }

            const float now = static_cast<float>(sim_time_sec_);
            v.perception_history.push_back({now, leader_id, net_distance_s, leader_speed});
            while (v.perception_history.size() > 1 && v.perception_history.front().timestamp < now - 3.0f) {
                v.perception_history.pop_front();
            }

            const float target_time = now - v.profile.reaction_delay * reaction_delay_multiplier_;
            const PerceivedLeaderSample* delayed = &v.perception_history.front();
            for (const auto& sample : v.perception_history) {
                if (sample.timestamp <= target_time) delayed = &sample;
                else break;
            }

            const float dv = v.speed - delayed->leader_speed;
            const float noise = ou_noise_enabled_ ? v.noise_process.step(dt, rng_) : 0.0f;
            const float delayed_accel = IDMModel::computeAcceleration(v.profile, v.speed, speed_limit, delayed->net_distance_s, dv, noise);

            // Reaction delay models the "comfort" following behaviour, but a real driver
            // still reacts instinctively to an imminent collision; use the current
            // (undelayed) gap as a hard backstop so delayed perception can never let a
            // vehicle actually drive into the one ahead.
            const float instant_dv = v.speed - leader_speed;
            const float instant_accel = IDMModel::computeAcceleration(v.profile, v.speed, speed_limit, net_distance_s, instant_dv, 0.0f);

            v.acceleration = std::min(delayed_accel, instant_accel);
            v.braking_light = v.acceleration < -0.5f;

            if (!v.on_connector && v.planned_next_connector != core::INVALID_ID) {
                if (const network::LaneConnector* pc = network_.findConnector(v.planned_next_connector)) {
                    switch (pc->turn_type) {
                        case network::TurnDirection::Left: v.blinker_state = -1; break;
                        case network::TurnDirection::Right: v.blinker_state = 1; break;
                        default: v.blinker_state = 0; break;
                    }
                }
            } else {
                v.blinker_state = 0;
            }
        }
    }

    void SimulationWorld::updatePerturbations(float dt) {
        for (auto it = active_brakes_.begin(); it != active_brakes_.end();) {
            auto vit = vehicles_.find(it->first);
            if (vit == vehicles_.end()) {
                it = active_brakes_.erase(it);
                continue;
            }
            Vehicle& v = vit->second;
            if (v.speed > it->second.target_speed) {
                v.acceleration = -v.profile.max_decel;
            }
            it->second.remaining_time -= dt;
            if (it->second.remaining_time <= 0.0f) it = active_brakes_.erase(it);
            else ++it;
        }
    }

    void SimulationWorld::integrate(float dt) {
        for (auto& [id, v] : vehicles_) {
            const float new_speed = std::max(0.0f, v.speed + v.acceleration * dt);
            const float delta_s = std::max(0.0f, v.speed * dt + 0.5f * v.acceleration * dt * dt);
            v.speed = new_speed;
            v.s += delta_s;
        }
    }

    void SimulationWorld::updateTransitionsAndPositions() {
        std::vector<core::VehicleId> to_remove;

        for (auto& [id, v] : vehicles_) {
            if (!v.on_connector) {
                const network::Lane* lane = network_.findLane(v.current_lane);
                if (!lane) {
                    to_remove.push_back(id);
                    continue;
                }

                if (v.planned_next_connector == core::INVALID_ID && !lane->outgoing_connectors.empty()) {
                    const float pick = rng_.uniform(0.0f, static_cast<float>(lane->outgoing_connectors.size()));
                    const std::size_t choice = std::min(static_cast<std::size_t>(pick), lane->outgoing_connectors.size() - 1);
                    v.planned_next_connector = lane->outgoing_connectors[choice];
                }

                // The lane's drivable span is set back by the destination node's radius:
                // the connector actually begins there, not at the lane's raw end.
                float trigger_s = lane->length();
                if (v.planned_next_connector != core::INVALID_ID) {
                    if (const network::LaneConnector* pc = network_.findConnector(v.planned_next_connector)) {
                        if (const network::IntersectionNode* pnode = network_.findNode(pc->parent_node)) {
                            trigger_s = std::max(0.0f, lane->length() - pnode->radius);
                        }
                    }
                }

                if (v.s >= trigger_s) {
                    if (v.planned_next_connector != core::INVALID_ID) {
                        const float overflow = v.s - trigger_s;
                        v.on_connector = true;
                        v.current_connector = v.planned_next_connector;
                        v.planned_next_connector = core::INVALID_ID;
                        v.s = overflow;
                        v.lateral_offset = 0.0f;
                    } else {
                        to_remove.push_back(id);
                        continue;
                    }
                }
            } else {
                const network::LaneConnector* connector = network_.findConnector(v.current_connector);
                if (!connector) {
                    to_remove.push_back(id);
                    continue;
                }
                if (v.s >= connector->trajectory.totalLength()) {
                    const float overflow = v.s - connector->trajectory.totalLength();
                    v.on_connector = false;
                    v.current_lane = connector->to_lane;
                    v.current_connector = core::INVALID_ID;

                    // The connector's exit lands `radius` meters into the destination lane,
                    // not at its raw start (mirrors the setback used to build the connector).
                    float entry_s = overflow;
                    if (const network::IntersectionNode* node = network_.findNode(connector->parent_node)) {
                        if (const network::Lane* new_lane = network_.findLane(v.current_lane)) {
                            entry_s = std::min(new_lane->length(), node->radius) + overflow;
                        }
                    }
                    v.s = entry_s;
                }
            }

            if (v.on_connector) {
                if (const network::LaneConnector* connector = network_.findConnector(v.current_connector)) {
                    v.world_pos = connector->trajectory.evaluateAtDistance(v.s);
                    v.world_heading = connector->trajectory.tangentAtDistance(v.s).angle();
                }
            } else {
                if (const network::Lane* lane = network_.findLane(v.current_lane)) {
                    v.world_pos = lane->center_curve.evaluateAtDistance(v.s);
                    v.world_heading = lane->center_curve.tangentAtDistance(v.s).angle();
                }
            }
        }

        for (core::VehicleId id : to_remove) {
            vehicles_.erase(id);
            active_brakes_.erase(id);
        }
    }

    void SimulationWorld::updateSpawners(float dt) {
        for (Spawner& spawner : spawners_) {
            if (!spawner.update(dt, rng_)) continue;

            bool blocked = false;
            for (const auto& [id, v] : vehicles_) {
                if (!v.on_connector && v.current_lane == spawner.target_lane && v.s < 10.0f) {
                    blocked = true;
                    break;
                }
            }
            if (blocked) continue;

            spawnVehicleManual(spawner.target_lane, 0.0f, spawner.pickVehicleClass(rng_));
        }
    }

    void SimulationWorld::step(float dt) {
        std::unordered_map<core::LaneRef, std::vector<const Vehicle*>> lane_occ;
        std::unordered_map<core::ConnectorId, std::vector<const Vehicle*>> connector_occ;
        buildOccupancy(lane_occ, connector_occ);

        updateTrafficLights(dt, connector_occ);

        updateLaneChanges(dt, lane_occ);

        buildOccupancy(lane_occ, connector_occ);
        updateVehicleDynamics(dt, lane_occ, connector_occ);

        updatePerturbations(dt);
        integrate(dt);
        updateTransitionsAndPositions();
        updateSpawners(dt);

        sim_time_sec_ += static_cast<double>(dt);
        ++tick_count_;
    }

    core::VehicleId SimulationWorld::spawnVehicleManual(const core::LaneRef& lane, float s, VehicleClass vclass) {
        const network::Lane* lane_ptr = network_.findLane(lane);
        if (!lane_ptr) return core::INVALID_ID;

        Vehicle v;
        v.id = next_vehicle_id_++;
        switch (vclass) {
            case VehicleClass::HeavyTruck: v.profile = VehicleProfile::makeTruck(); break;
            case VehicleClass::AggressiveDriver: v.profile = VehicleProfile::makeAggressive(); break;
            default: v.profile = VehicleProfile::makeCar(); break;
        }
        v.current_lane = lane;
        v.s = std::clamp(s, 0.0f, lane_ptr->length());
        v.speed = std::min(v.profile.desired_speed, lane_ptr->speed_limit);
        v.noise_process.sigma = v.profile.ou_sigma;
        v.world_pos = lane_ptr->center_curve.evaluateAtDistance(v.s);
        v.world_heading = lane_ptr->center_curve.tangentAtDistance(v.s).angle();

        const core::VehicleId id = v.id;
        vehicles_.emplace(id, std::move(v));
        return id;
    }

    core::VehicleId SimulationWorld::restoreVehicle(const Vehicle& saved) {
        Vehicle v = saved;
        v.id = next_vehicle_id_++;
        const core::VehicleId id = v.id;
        vehicles_.emplace(id, std::move(v));
        return id;
    }

    void SimulationWorld::applyPerturbationBrake(core::VehicleId vid, float target_speed, float duration) {
        if (vehicles_.find(vid) == vehicles_.end()) return;
        active_brakes_[vid] = ActiveBraking{target_speed, duration};
    }

    core::SpawnerId SimulationWorld::addSpawner(const Spawner& spawner) {
        Spawner s = spawner;
        s.id = next_spawner_id_++;
        const core::SpawnerId id = s.id;
        spawners_.push_back(std::move(s));
        return id;
    }

    void SimulationWorld::removeAllVehicles() {
        vehicles_.clear();
        active_brakes_.clear();
    }

    SimulationSnapshot SimulationWorld::buildSnapshot() const {
        SimulationSnapshot snap;
        snap.tick_count = tick_count_;
        snap.sim_time_sec = sim_time_sec_;
        snap.active_vehicles_count = vehicles_.size();

        float speed_sum = 0.0f;
        std::size_t jammed = 0;
        snap.vehicles.reserve(vehicles_.size());
        for (const auto& [id, v] : vehicles_) {
            VehicleRenderData rd{};
            rd.id = v.id;
            rd.vclass = v.profile.type;
            rd.position = v.world_pos;
            rd.heading = v.world_heading;
            rd.length = v.profile.length;
            rd.width = v.profile.width;
            rd.speed = v.speed;
            rd.acceleration = v.acceleration;
            rd.braking = v.braking_light;
            rd.blinker = v.blinker_state;
            snap.vehicles.push_back(rd);

            speed_sum += v.speed;
            if (v.speed < 2.0f) ++jammed;
        }
        snap.jammed_vehicles_count = jammed;
        snap.mean_speed_kmh = vehicles_.empty() ? 0.0f : (speed_sum / static_cast<float>(vehicles_.size())) * 3.6f;

        for (const auto& [nid, node] : network_.nodes()) {
            if (node.rule != network::IntersectionRule::TrafficLight) continue;

            std::unordered_map<core::LaneRef, network::LightState> lane_states;
            for (core::ConnectorId cid : node.internal_connectors) {
                const network::LaneConnector* c = network_.findConnector(cid);
                if (!c) continue;
                const network::LightState st = node.light_controller.getStateForConnector(cid);
                if (auto it = lane_states.find(c->from_lane); it == lane_states.end()) {
                    lane_states.emplace(c->from_lane, st);
                } else if (st == network::LightState::Green) {
                    it->second = network::LightState::Green;
                } else if (st == network::LightState::Yellow && it->second == network::LightState::Red) {
                    it->second = network::LightState::Yellow;
                }
            }

            for (const auto& [lane_ref, state] : lane_states) {
                const network::Lane* from_lane = network_.findLane(lane_ref);
                if (!from_lane) continue;
                const float stop_s = std::max(0.0f, from_lane->length() - node.radius);

                TrafficLightRenderData td{};
                td.node_id = nid;
                td.lane_ref = lane_ref;
                td.stop_line_pos = from_lane->center_curve.evaluateAtDistance(stop_s);
                td.stop_line_dir = from_lane->center_curve.tangentAtDistance(stop_s);
                td.state = state;
                snap.traffic_lights.push_back(td);
            }
        }

        return snap;
    }
}
