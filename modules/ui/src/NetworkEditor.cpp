#include "ui/NetworkEditor.hpp"

namespace tfs::ui {

    core::NodeId NetworkEditor::findNearestNode(core::Vec2 pos, const network::RoadNetwork& net, float max_dist) {
        core::NodeId best = core::INVALID_ID;
        float best_dist = max_dist;
        for (const auto& [id, node] : net.nodes()) {
            const float d = node.position.distanceTo(pos);
            if (d < best_dist) {
                best_dist = d;
                best = id;
            }
        }
        return best;
    }

    core::RoadId NetworkEditor::findNearestRoad(core::Vec2 pos, const network::RoadNetwork& net, float max_dist,
                                                 core::Vec2& out_closest) {
        core::RoadId best = core::INVALID_ID;
        float best_dist = max_dist;
        for (const auto& [id, road] : net.roads()) {
            core::Vec2 closest;
            float s = 0.0f;
            const float d = road.baseline_curve.projectPoint(pos, closest, s);
            if (d < best_dist) {
                best_dist = d;
                best = id;
                out_closest = closest;
            }
        }
        return best;
    }

    core::LaneRef NetworkEditor::findNearestLane(core::Vec2 pos, const network::RoadNetwork& net, float max_dist,
                                                  core::Vec2& out_closest) {
        core::LaneRef best{};
        float best_dist = max_dist;

        for (const auto& [id, road] : net.roads()) {
            const auto check_lanes = [&](const std::vector<network::Lane>& lanes) {
                for (const network::Lane& lane : lanes) {
                    core::Vec2 closest;
                    float s = 0.0f;
                    const float d = lane.center_curve.projectPoint(pos, closest, s);
                    if (d < best_dist) {
                        best_dist = d;
                        best = lane.ref;
                        out_closest = closest;
                    }
                }
            };
            check_lanes(road.forward_lanes);
            check_lanes(road.backward_lanes);
        }

        return best;
    }

    core::VehicleId NetworkEditor::findNearestVehicle(core::Vec2 pos, const sim::SimulationWorld& world, float max_dist) {
        core::VehicleId best = core::INVALID_ID;
        float best_dist = max_dist;
        for (const auto& [id, v] : world.vehicles()) {
            const float d = v.world_pos.distanceTo(pos);
            if (d < best_dist) {
                best_dist = d;
                best = id;
            }
        }
        return best;
    }

    void NetworkEditor::handleClick(core::Vec2 world_pos, sim::SimulationThread& thread) {
        thread.executeSynchronously([this, world_pos](sim::SimulationWorld& world) {
            network::RoadNetwork& net = world.network();

            switch (tool_) {
                case EditorTool::AddNode: {
                    const core::NodeId id = net.addNode(world_pos);
                    if (network::IntersectionNode* node = net.findNode(id)) node->radius = new_node_radius_;
                    break;
                }
                case EditorTool::AddStraightRoad:
                case EditorTool::AddCurvedRoad: {
                    const core::NodeId hit = findNearestNode(world_pos, net, 15.0f);
                    if (hit == core::INVALID_ID) break;
                    if (pending_start_node_ == core::INVALID_ID) {
                        pending_start_node_ = hit;
                    } else if (pending_start_node_ != hit) {
                        const network::IntersectionNode* start_node = net.findNode(pending_start_node_);
                        const network::IntersectionNode* end_node = net.findNode(hit);
                        if (start_node && end_node) {
                            if (tool_ == EditorTool::AddCurvedRoad) {
                                const core::Vec2 delta = end_node->position - start_node->position;
                                const core::Vec2 offset = delta.perpRight().normalized() * (delta.length() * 0.2f);
                                net.addRoad(pending_start_node_, hit, new_fwd_lanes_, new_bwd_lanes_, 13.89f, true,
                                            start_node->position + delta * (1.0f / 3.0f) + offset,
                                            start_node->position + delta * (2.0f / 3.0f) + offset);
                            } else {
                                net.addRoad(pending_start_node_, hit, new_fwd_lanes_, new_bwd_lanes_);
                            }
                        }
                        pending_start_node_ = core::INVALID_ID;
                    }
                    break;
                }
                case EditorTool::PlaceSpawner: {
                    core::Vec2 closest;
                    const core::LaneRef lane_ref = findNearestLane(world_pos, net, 20.0f, closest);
                    if (lane_ref.road_id == core::INVALID_ID) break;
                    sim::Spawner spawner;
                    spawner.target_lane = lane_ref;
                    world.addSpawner(spawner);
                    break;
                }
                case EditorTool::SpawnSingleVehicle: {
                    core::Vec2 closest;
                    const core::LaneRef lane_ref = findNearestLane(world_pos, net, 20.0f, closest);
                    if (lane_ref.road_id == core::INVALID_ID) break;
                    if (const network::Lane* lane = net.findLane(lane_ref)) {
                        core::Vec2 tmp;
                        float s_out = 0.0f;
                        [[maybe_unused]] const float dist = lane->center_curve.projectPoint(world_pos, tmp, s_out);
                        world.spawnVehicleManual(lane_ref, s_out, spawn_class_);
                    }
                    break;
                }
                case EditorTool::DeleteElement: {
                    const core::NodeId nid = findNearestNode(world_pos, net, 15.0f);
                    if (nid != core::INVALID_ID) {
                        net.removeNode(nid);
                        clearSelection();
                        break;
                    }
                    core::Vec2 closest;
                    const core::RoadId rid = findNearestRoad(world_pos, net, 10.0f, closest);
                    if (rid != core::INVALID_ID) {
                        net.removeRoad(rid);
                        clearSelection();
                    }
                    break;
                }
                case EditorTool::SelectInspect: {
                    const core::VehicleId vid = findNearestVehicle(world_pos, world, 5.0f);
                    if (vid != core::INVALID_ID) {
                        selected_vehicle_ = vid;
                        selected_node_ = core::INVALID_ID;
                        selected_road_ = core::INVALID_ID;
                        break;
                    }
                    const core::NodeId nid = findNearestNode(world_pos, net, 15.0f);
                    if (nid != core::INVALID_ID) {
                        selected_node_ = nid;
                        selected_road_ = core::INVALID_ID;
                        selected_vehicle_ = core::INVALID_ID;
                        break;
                    }
                    core::Vec2 closest;
                    const core::RoadId rid = findNearestRoad(world_pos, net, 10.0f, closest);
                    selected_road_ = rid;
                    selected_node_ = core::INVALID_ID;
                    selected_vehicle_ = core::INVALID_ID;
                    break;
                }
            }
        });
    }

    void NetworkEditor::handleDragStart(core::Vec2 world_pos, sim::SimulationThread& thread) {
        if (tool_ != EditorTool::SelectInspect) return;
        thread.executeSynchronously([this, world_pos](sim::SimulationWorld& world) {
            network::RoadNetwork& net = world.network();
            const core::NodeId nid = findNearestNode(world_pos, net, 15.0f);
            if (nid != core::INVALID_ID) {
                drag_kind_ = DragKind::Node;
                drag_node_ = nid;
                return;
            }
            if (selected_road_ != core::INVALID_ID) {
                if (const network::RoadSegment* road = net.findRoad(selected_road_); road && road->is_curved) {
                    if (road->baseline_curve.p1().distanceTo(world_pos) < 5.0f) {
                        drag_kind_ = DragKind::ControlP1;
                        drag_road_ = selected_road_;
                        return;
                    }
                    if (road->baseline_curve.p2().distanceTo(world_pos) < 5.0f) {
                        drag_kind_ = DragKind::ControlP2;
                        drag_road_ = selected_road_;
                        return;
                    }
                }
            }
            drag_kind_ = DragKind::None;
        });
    }

    void NetworkEditor::handleDrag(core::Vec2 world_pos, sim::SimulationThread& thread) {
        if (drag_kind_ == DragKind::None) return;
        thread.executeSynchronously([this, world_pos](sim::SimulationWorld& world) {
            network::RoadNetwork& net = world.network();
            if (drag_kind_ == DragKind::Node) {
                net.moveNode(drag_node_, world_pos);
            } else if (drag_kind_ == DragKind::ControlP1 || drag_kind_ == DragKind::ControlP2) {
                if (const network::RoadSegment* road = net.findRoad(drag_road_)) {
                    core::Vec2 p1 = road->baseline_curve.p1();
                    core::Vec2 p2 = road->baseline_curve.p2();
                    if (drag_kind_ == DragKind::ControlP1) p1 = world_pos;
                    else p2 = world_pos;
                    net.updateRoadControlPoints(drag_road_, p1, p2);
                }
            }
        });
    }

    void NetworkEditor::handleDragEnd() {
        drag_kind_ = DragKind::None;
    }
}
