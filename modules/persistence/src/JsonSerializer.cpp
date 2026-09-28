#include "persistence/JsonSerializer.hpp"
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

namespace tfs::persistence {

    namespace {
        using nlohmann::json;

        json vec2ToJson(const core::Vec2& v) {
            return {{"x", v.x}, {"y", v.y}};
        }
        core::Vec2 jsonToVec2(const json& j) {
            return {j.at("x").get<float>(), j.at("y").get<float>()};
        }

        json laneRefToJson(const core::LaneRef& r) {
            return {{"road", r.road_id}, {"fwd", r.is_forward}, {"idx", r.lane_idx}};
        }
        core::LaneRef jsonToLaneRef(const json& j) {
            core::LaneRef r;
            r.road_id = j.at("road").get<core::RoadId>();
            r.is_forward = j.at("fwd").get<bool>();
            r.lane_idx = j.at("idx").get<core::LaneIndex>();
            return r;
        }

        std::string ruleToString(network::IntersectionRule rule) {
            switch (rule) {
                case network::IntersectionRule::Uncontrolled: return "Uncontrolled";
                case network::IntersectionRule::TrafficLight: return "TrafficLight";
                default: return "PriorityYield";
            }
        }
        network::IntersectionRule stringToRule(const std::string& s) {
            if (s == "Uncontrolled") return network::IntersectionRule::Uncontrolled;
            if (s == "TrafficLight") return network::IntersectionRule::TrafficLight;
            if (s == "PriorityYield") return network::IntersectionRule::PriorityYield;
            throw std::runtime_error("Regola incrocio non valida: " + s);
        }

        std::string vclassToString(sim::VehicleClass c) {
            switch (c) {
                case sim::VehicleClass::HeavyTruck: return "HeavyTruck";
                case sim::VehicleClass::AggressiveDriver: return "AggressiveDriver";
                default: return "PassengerCar";
            }
        }
        sim::VehicleClass stringToVclass(const std::string& s) {
            if (s == "HeavyTruck") return sim::VehicleClass::HeavyTruck;
            if (s == "AggressiveDriver") return sim::VehicleClass::AggressiveDriver;
            if (s == "PassengerCar") return sim::VehicleClass::PassengerCar;
            throw std::runtime_error("Classe veicolo non valida: " + s);
        }

        json profileToJson(const sim::VehicleProfile& p) {
            return {
                {"type", vclassToString(p.type)}, {"length", p.length}, {"width", p.width},
                {"desired_speed", p.desired_speed}, {"min_gap", p.min_gap}, {"time_headway", p.time_headway},
                {"max_accel", p.max_accel}, {"comfort_decel", p.comfort_decel}, {"max_decel", p.max_decel},
                {"politeness", p.politeness}, {"lane_change_threshold", p.lane_change_threshold},
                {"reaction_delay", p.reaction_delay}, {"ou_sigma", p.ou_sigma}
            };
        }
        sim::VehicleProfile jsonToProfile(const json& j) {
            sim::VehicleProfile p;
            p.type = stringToVclass(j.at("type").get<std::string>());
            p.length = j.at("length").get<float>();
            p.width = j.at("width").get<float>();
            p.desired_speed = j.at("desired_speed").get<float>();
            p.min_gap = j.at("min_gap").get<float>();
            p.time_headway = j.at("time_headway").get<float>();
            p.max_accel = j.at("max_accel").get<float>();
            p.comfort_decel = j.at("comfort_decel").get<float>();
            p.max_decel = j.at("max_decel").get<float>();
            p.politeness = j.at("politeness").get<float>();
            p.lane_change_threshold = j.at("lane_change_threshold").get<float>();
            p.reaction_delay = j.at("reaction_delay").get<float>();
            p.ou_sigma = j.at("ou_sigma").get<float>();
            return p;
        }

        json nodeToJson(const network::IntersectionNode& node) {
            json j;
            j["id"] = node.id;
            j["pos"] = vec2ToJson(node.position);
            j["radius"] = node.radius;
            j["rule"] = ruleToString(node.rule);

            json ranks = json::array();
            for (const auto& [road_id, rank] : node.road_priority_rank) {
                ranks.push_back({{"road", road_id}, {"rank", rank}});
            }
            j["priority_ranks"] = ranks;

            if (node.rule == network::IntersectionRule::TrafficLight) {
                json phases = json::array();
                for (const auto& phase : node.light_controller.phases) {
                    phases.push_back({{"green", phase.green_duration}, {"yellow", phase.yellow_duration},
                                       {"all_red", phase.all_red_duration}});
                }
                j["light_phases"] = phases;
            }
            return j;
        }

        json roadToJson(const network::RoadSegment& road) {
            json j;
            j["id"] = road.id;
            j["start"] = road.start_node;
            j["end"] = road.end_node;
            j["curved"] = road.is_curved;
            if (road.is_curved) {
                j["p1"] = vec2ToJson(road.baseline_curve.p1());
                j["p2"] = vec2ToJson(road.baseline_curve.p2());
            }
            j["fwd_lanes"] = road.num_forward_lanes;
            j["bwd_lanes"] = road.num_backward_lanes;
            j["lane_width"] = road.lane_width;
            j["speed_limit"] = road.speed_limit;
            return j;
        }

        json spawnerToJson(const sim::Spawner& spawner) {
            json j;
            j["lane"] = laneRefToJson(spawner.target_lane);
            j["flow_rate_vph"] = spawner.flow_rate_vph;
            j["truck_ratio"] = spawner.truck_ratio;
            j["aggressive_ratio"] = spawner.aggressive_ratio;
            j["enabled"] = spawner.enabled;
            return j;
        }

        json vehicleToJson(const sim::Vehicle& v, const network::RoadNetwork& net) {
            json j;
            j["profile"] = profileToJson(v.profile);
            j["on_connector"] = v.on_connector;
            j["lane"] = laneRefToJson(v.current_lane);
            if (v.on_connector) {
                if (const network::LaneConnector* c = net.findConnector(v.current_connector)) {
                    j["from_lane"] = laneRefToJson(c->from_lane);
                    j["to_lane"] = laneRefToJson(c->to_lane);
                }
            }
            j["s"] = v.s;
            j["speed"] = v.speed;
            j["acceleration"] = v.acceleration;
            j["lateral_offset"] = v.lateral_offset;
            j["lane_change_cooldown"] = v.lane_change_cooldown;
            j["noise_state"] = v.noise_process.state;
            j["noise_tau"] = v.noise_process.tau;
            j["noise_sigma"] = v.noise_process.sigma;

            json history = json::array();
            for (const auto& sample : v.perception_history) {
                history.push_back({{"t", sample.timestamp}, {"leader", sample.leader_id},
                                    {"gap", sample.net_distance_s}, {"lspeed", sample.leader_speed}});
            }
            j["perception_history"] = history;
            return j;
        }

        sim::Vehicle jsonToVehicle(const json& j, const network::RoadNetwork& net) {
            sim::Vehicle v;
            v.profile = jsonToProfile(j.at("profile"));
            v.on_connector = j.at("on_connector").get<bool>();
            v.current_lane = jsonToLaneRef(j.at("lane"));
            v.s = j.at("s").get<float>();
            v.speed = j.at("speed").get<float>();
            v.acceleration = j.at("acceleration").get<float>();
            v.lateral_offset = j.at("lateral_offset").get<float>();
            v.lane_change_cooldown = j.at("lane_change_cooldown").get<float>();
            v.noise_process.state = j.at("noise_state").get<float>();
            v.noise_process.tau = j.at("noise_tau").get<float>();
            v.noise_process.sigma = j.at("noise_sigma").get<float>();

            if (v.on_connector && j.contains("from_lane") && j.contains("to_lane")) {
                const core::LaneRef from = jsonToLaneRef(j.at("from_lane"));
                const core::LaneRef to = jsonToLaneRef(j.at("to_lane"));
                v.current_connector = core::INVALID_ID;
                for (const auto& [cid, connector] : net.connectors()) {
                    if (connector.from_lane == from && connector.to_lane == to) {
                        v.current_connector = cid;
                        break;
                    }
                }
                if (v.current_connector == core::INVALID_ID) v.on_connector = false;
            } else {
                v.on_connector = false;
            }

            if (j.contains("perception_history")) {
                for (const auto& h : j.at("perception_history")) {
                    sim::PerceivedLeaderSample sample;
                    sample.timestamp = h.at("t").get<float>();
                    sample.leader_id = h.at("leader").get<core::VehicleId>();
                    sample.net_distance_s = h.at("gap").get<float>();
                    sample.leader_speed = h.at("lspeed").get<float>();
                    v.perception_history.push_back(sample);
                }
            }

            if (v.on_connector) {
                if (const network::LaneConnector* c = net.findConnector(v.current_connector)) {
                    v.world_pos = c->trajectory.evaluateAtDistance(v.s);
                    v.world_heading = c->trajectory.tangentAtDistance(v.s).angle();
                }
            } else if (const network::Lane* lane = net.findLane(v.current_lane)) {
                v.world_pos = lane->center_curve.evaluateAtDistance(v.s);
                v.world_heading = lane->center_curve.tangentAtDistance(v.s).angle();
            }

            return v;
        }
    }

    bool JsonSerializer::saveToFile(const std::filesystem::path& filepath, const sim::SimulationWorld& world,
                                     SaveMode mode, std::string& out_error) {
        try {
            json root;
            root["version"] = 1;
            root["mode"] = (mode == SaveMode::FullState) ? "FullState" : "InfrastructureOnly";

            const network::RoadNetwork& net = world.network();

            json nodesJson = json::array();
            for (const auto& [id, node] : net.nodes()) nodesJson.push_back(nodeToJson(node));
            root["nodes"] = nodesJson;

            json roadsJson = json::array();
            for (const auto& [id, road] : net.roads()) roadsJson.push_back(roadToJson(road));
            root["roads"] = roadsJson;

            json spawnersJson = json::array();
            for (const auto& spawner : world.spawners()) spawnersJson.push_back(spawnerToJson(spawner));
            root["spawners"] = spawnersJson;

            if (mode == SaveMode::FullState) {
                json vehiclesJson = json::array();
                for (const auto& [id, v] : world.vehicles()) vehiclesJson.push_back(vehicleToJson(v, net));
                root["vehicles"] = vehiclesJson;
            }

            std::ofstream file(filepath);
            if (!file.is_open()) {
                out_error = "Impossibile aprire il file per la scrittura";
                return false;
            }
            file << root.dump(2);
            return true;
        } catch (const std::exception& e) {
            out_error = e.what();
            return false;
        }
    }

    bool JsonSerializer::loadFromFile(const std::filesystem::path& filepath, sim::SimulationWorld& world,
                                       std::string& out_error) {
        try {
            std::ifstream file(filepath);
            if (!file.is_open()) {
                out_error = "Impossibile aprire il file per la lettura";
                return false;
            }
            json root;
            file >> root;

            world.removeAllVehicles();
            network::RoadNetwork& net = world.network();
            net = network::RoadNetwork{};
            world.spawners().clear();

            std::vector<std::pair<core::NodeId, json>> pending_ranks;
            std::vector<std::pair<core::NodeId, json>> pending_phases;

            for (const auto& nJson : root.at("nodes")) {
                const core::NodeId id = nJson.at("id").get<core::NodeId>();
                const core::Vec2 pos = jsonToVec2(nJson.at("pos"));
                const float radius = nJson.at("radius").get<float>();
                const network::IntersectionRule rule = stringToRule(nJson.at("rule").get<std::string>());
                net.addNodeWithId(id, pos, rule);
                if (network::IntersectionNode* node = net.findNode(id)) node->radius = radius;
                if (nJson.contains("priority_ranks")) pending_ranks.emplace_back(id, nJson.at("priority_ranks"));
                if (nJson.contains("light_phases")) pending_phases.emplace_back(id, nJson.at("light_phases"));
            }

            for (const auto& rJson : root.at("roads")) {
                const core::RoadId id = rJson.at("id").get<core::RoadId>();
                const core::NodeId start = rJson.at("start").get<core::NodeId>();
                const core::NodeId end = rJson.at("end").get<core::NodeId>();
                const bool curved = rJson.at("curved").get<bool>();
                core::Vec2 p1{}, p2{};
                if (curved) {
                    p1 = jsonToVec2(rJson.at("p1"));
                    p2 = jsonToVec2(rJson.at("p2"));
                }
                const std::uint32_t fwd = rJson.at("fwd_lanes").get<std::uint32_t>();
                const std::uint32_t bwd = rJson.at("bwd_lanes").get<std::uint32_t>();
                const float speed_limit = rJson.at("speed_limit").get<float>();
                net.addRoadWithId(id, start, end, fwd, bwd, speed_limit, curved, p1, p2);
                if (network::RoadSegment* road = net.findRoad(id)) {
                    road->lane_width = rJson.at("lane_width").get<float>();
                }
            }

            for (const auto& [nid, node] : net.nodes()) {
                net.rebuildIntersection(nid);
            }

            for (auto& [nid, ranksJson] : pending_ranks) {
                network::IntersectionNode* node = net.findNode(nid);
                if (!node) continue;
                for (const auto& r : ranksJson) {
                    node->road_priority_rank[r.at("road").get<core::RoadId>()] = r.at("rank").get<int>();
                }
            }
            for (auto& [nid, phasesJson] : pending_phases) {
                network::IntersectionNode* node = net.findNode(nid);
                if (!node) continue;
                std::size_t i = 0;
                for (const auto& p : phasesJson) {
                    if (i >= node->light_controller.phases.size()) break;
                    node->light_controller.phases[i].green_duration = p.at("green").get<float>();
                    node->light_controller.phases[i].yellow_duration = p.at("yellow").get<float>();
                    node->light_controller.phases[i].all_red_duration = p.at("all_red").get<float>();
                    ++i;
                }
            }

            if (root.contains("spawners")) {
                for (const auto& sJson : root.at("spawners")) {
                    sim::Spawner spawner;
                    spawner.target_lane = jsonToLaneRef(sJson.at("lane"));
                    spawner.flow_rate_vph = sJson.at("flow_rate_vph").get<float>();
                    spawner.truck_ratio = sJson.at("truck_ratio").get<float>();
                    spawner.aggressive_ratio = sJson.at("aggressive_ratio").get<float>();
                    spawner.enabled = sJson.at("enabled").get<bool>();
                    world.addSpawner(spawner);
                }
            }

            if (root.contains("vehicles")) {
                for (const auto& vJson : root.at("vehicles")) {
                    world.restoreVehicle(jsonToVehicle(vJson, net));
                }
            }

            return true;
        } catch (const std::exception& e) {
            out_error = e.what();
            return false;
        }
    }
}
