#include "ui/ImGuiPanels.hpp"
#include <algorithm>
#include <cstdint>
#include <imgui.h>

namespace tfs::ui {

    namespace {
        struct NodeInspectView {
            bool valid{false};
            core::NodeId id{core::INVALID_ID};
            network::IntersectionRule rule{network::IntersectionRule::PriorityYield};
            float radius{12.0f};
            std::vector<network::TrafficLightPhase> phases;
        };

        struct RoadInspectView {
            bool valid{false};
            core::RoadId id{core::INVALID_ID};
            std::uint32_t fwd_lanes{0};
            std::uint32_t bwd_lanes{0};
            float speed_limit{0.0f};
            bool curved{false};
        };

        struct VehicleInspectView {
            bool valid{false};
            float speed{0.0f};
            float acceleration{0.0f};
            float net_distance_s{-1.0f};
        };
    }

    void ImGuiPanels::render(sim::SimulationThread& thread, NetworkEditor& editor, Camera2D& camera,
                              const sim::SimulationSnapshot& snapshot, sf::Vector2u target_size) {
        renderSimulationControlPanel(thread, camera, snapshot, target_size);
        renderEditorToolbarPanel(editor);
        renderInspectorPanel(thread, editor);
        renderPersistencePanel(thread, snapshot);
    }

    bool ImGuiPanels::loadScenario(sim::SimulationThread& thread, Camera2D& camera, const std::string& path,
                                    sf::Vector2u target_size, std::string& out_error) {
        bool ok = false;
        core::Vec2 min_pos{0.0f, 0.0f};
        core::Vec2 max_pos{0.0f, 0.0f};
        bool has_bounds = false;

        thread.executeSynchronously([&](sim::SimulationWorld& world) {
            ok = persistence::JsonSerializer::loadFromFile(path, world, out_error);
            if (!ok) return;
            for (const auto& [id, node] : world.network().nodes()) {
                if (!has_bounds) {
                    min_pos = max_pos = node.position;
                    has_bounds = true;
                } else {
                    min_pos.x = std::min(min_pos.x, node.position.x);
                    min_pos.y = std::min(min_pos.y, node.position.y);
                    max_pos.x = std::max(max_pos.x, node.position.x);
                    max_pos.y = std::max(max_pos.y, node.position.y);
                }
            }
        });

        if (ok && has_bounds) {
            camera.frameBounds(min_pos, max_pos, target_size);
        }
        return ok;
    }

    void ImGuiPanels::renderSimulationControlPanel(sim::SimulationThread& thread, Camera2D& camera,
                                                     const sim::SimulationSnapshot& snapshot, sf::Vector2u target_size) {
        ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(480.0f, 260.0f), ImGuiCond_FirstUseEver);
        ImGui::Begin("Simulation Control & Phantom Jam Lab");

        if (ImGui::Button(paused_ ? "Play" : "Pause")) {
            paused_ = !paused_;
            thread.setPaused(paused_);
        }
        ImGui::SameLine();
        if (ImGui::Button("Step (10ms)")) {
            thread.stepSingleTick();
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear Vehicles")) {
            thread.executeSynchronously([](sim::SimulationWorld& world) { world.removeAllVehicles(); });
        }

        if (ImGui::SliderFloat("Time Scale", &time_scale_, 0.1f, 20.0f, "%.1fx")) {
            thread.setTimeScale(time_scale_);
        }

        if (ImGui::Checkbox("Ornstein-Uhlenbeck Noise", &ou_noise_enabled_)) {
            const bool enabled = ou_noise_enabled_;
            thread.executeSynchronously([enabled](sim::SimulationWorld& world) { world.setOUNoiseEnabled(enabled); });
        }
        if (ImGui::SliderFloat("Moltiplicatore ritardo reazione", &reaction_delay_multiplier_, 0.0f, 3.0f)) {
            const float mult = reaction_delay_multiplier_;
            thread.executeSynchronously([mult](sim::SimulationWorld& world) { world.setReactionDelayMultiplier(mult); });
        }

        if (ImGui::Button("Trigger Perturbation")) {
            thread.executeSynchronously([](sim::SimulationWorld& world) {
                if (!world.vehicles().empty()) {
                    const core::VehicleId vid = world.vehicles().begin()->first;
                    world.applyPerturbationBrake(vid, 2.0f, 3.0f);
                }
            });
        }

        ImGui::Separator();
        if (ImGui::Button("Load Ring Road Scenario")) {
            std::string err;
            const bool ok = loadScenario(thread, camera, "assets/scenarios/ring_road_phantom_jam.json", target_size, err);
            scenario_status_message_ = ok ? "Scenario caricato." : ("Errore caricamento: " + err);
        }
        ImGui::SameLine();
        if (ImGui::Button("Load 4-Way Intersection Scenario")) {
            std::string err;
            const bool ok = loadScenario(thread, camera, "assets/scenarios/four_way_intersection.json", target_size, err);
            scenario_status_message_ = ok ? "Scenario caricato." : ("Errore caricamento: " + err);
        }
        if (!scenario_status_message_.empty()) {
            ImGui::TextWrapped("%s", scenario_status_message_.c_str());
        }

        ImGui::Separator();
        ImGui::Text("Veicoli attivi: %zu", snapshot.active_vehicles_count);
        ImGui::Text("Velocita media: %.1f km/h", static_cast<double>(snapshot.mean_speed_kmh));
        ImGui::Text("In coda (phantom jam): %zu", snapshot.jammed_vehicles_count);

        ImGui::End();
    }

    void ImGuiPanels::renderEditorToolbarPanel(NetworkEditor& editor) {
        ImGui::SetNextWindowPos(ImVec2(20.0f, 300.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(480.0f, 220.0f), ImGuiCond_FirstUseEver);
        ImGui::Begin("Editor Toolbar & Tool Settings");

        const char* tool_names[] = {"Select/Inspect", "Add Node", "Add Straight Road", "Add Curved Road",
                                     "Place Spawner", "Spawn Single Vehicle", "Delete Element"};
        int tool_idx = static_cast<int>(editor.tool());
        if (ImGui::Combo("Tool attivo", &tool_idx, tool_names, 7)) {
            editor.setTool(static_cast<EditorTool>(tool_idx));
        }

        ImGui::Separator();

        const EditorTool tool = editor.tool();
        switch (tool) {
            case EditorTool::SelectInspect:
                ImGui::TextWrapped("Click per selezionare un nodo, una strada o un veicolo. Trascina un nodo "
                                    "o i punti di controllo di una strada curva selezionata per spostarli.");
                break;
            case EditorTool::AddNode:
                if (ImGui::SliderFloat("Raggio incrocio", &new_node_radius_, 4.0f, 30.0f)) {
                    editor.setNewNodeRadius(new_node_radius_);
                }
                ImGui::TextWrapped("Click per piazzare un nuovo nodo con il raggio impostato sopra.");
                break;
            case EditorTool::AddStraightRoad:
            case EditorTool::AddCurvedRoad: {
                bool lanes_changed = false;
                lanes_changed |= ImGui::SliderInt("Corsie avanti", &new_fwd_lanes_, 0, 4);
                lanes_changed |= ImGui::SliderInt("Corsie indietro", &new_bwd_lanes_, 0, 4);
                if (lanes_changed) {
                    editor.setNewRoadLaneCounts(static_cast<std::uint32_t>(new_fwd_lanes_), static_cast<std::uint32_t>(new_bwd_lanes_));
                }
                ImGui::TextWrapped("Click su un nodo di partenza, poi su un nodo di arrivo per creare la strada.");
                break;
            }
            case EditorTool::PlaceSpawner:
                ImGui::TextWrapped("Click vicino a una strada per aggiungere un generatore di traffico "
                                    "sulla sua corsia piu' vicina.");
                break;
            case EditorTool::SpawnSingleVehicle: {
                const char* class_names[] = {"PassengerCar", "HeavyTruck", "AggressiveDriver"};
                if (ImGui::Combo("Classe veicolo", &spawn_class_index_, class_names, 3)) {
                    editor.setSpawnVehicleClass(static_cast<sim::VehicleClass>(spawn_class_index_));
                }
                ImGui::TextWrapped("Click su una strada per inserire un veicolo della classe scelta sopra.");
                break;
            }
            case EditorTool::DeleteElement:
                ImGui::TextWrapped("Click su un nodo o una strada per eliminarlo.");
                break;
        }

        ImGui::End();
    }

    void ImGuiPanels::renderInspectorPanel(sim::SimulationThread& thread, NetworkEditor& editor) {
        ImGui::SetNextWindowPos(ImVec2(1100.0f, 20.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(480.0f, 320.0f), ImGuiCond_FirstUseEver);
        ImGui::Begin("Inspector");

        const core::NodeId sel_node = editor.selectedNode();
        const core::RoadId sel_road = editor.selectedRoad();
        const core::VehicleId sel_vehicle = editor.selectedVehicle();

        if (sel_node != core::INVALID_ID) {
            NodeInspectView view;
            thread.executeSynchronously([&](sim::SimulationWorld& world) {
                if (const network::IntersectionNode* node = world.network().findNode(sel_node)) {
                    view.valid = true;
                    view.id = node->id;
                    view.rule = node->rule;
                    view.radius = node->radius;
                    view.phases = node->light_controller.phases;
                }
            });

            if (view.valid) {
                ImGui::Text("Nodo #%llu", static_cast<unsigned long long>(view.id));
                const char* rule_names[] = {"Uncontrolled", "PriorityYield", "TrafficLight"};
                int rule_idx = static_cast<int>(view.rule);
                if (ImGui::Combo("Regola", &rule_idx, rule_names, 3)) {
                    const network::IntersectionRule new_rule = static_cast<network::IntersectionRule>(rule_idx);
                    thread.executeSynchronously([&](sim::SimulationWorld& world) {
                        if (network::IntersectionNode* node = world.network().findNode(sel_node)) {
                            node->rule = new_rule;
                        }
                        world.network().rebuildIntersection(sel_node);
                    });
                }
                float radius = view.radius;
                if (ImGui::SliderFloat("Raggio", &radius, 4.0f, 30.0f)) {
                    thread.executeSynchronously([&](sim::SimulationWorld& world) {
                        if (network::IntersectionNode* node = world.network().findNode(sel_node)) {
                            node->radius = radius;
                        }
                        world.network().rebuildIntersection(sel_node);
                    });
                }
                if (view.rule == network::IntersectionRule::TrafficLight) {
                    for (std::size_t i = 0; i < view.phases.size(); ++i) {
                        ImGui::Text("Fase %zu", i);
                        float g = view.phases[i].green_duration;
                        float y = view.phases[i].yellow_duration;
                        float r = view.phases[i].all_red_duration;
                        bool changed = false;
                        changed |= ImGui::SliderFloat(("Verde##" + std::to_string(i)).c_str(), &g, 2.0f, 60.0f);
                        changed |= ImGui::SliderFloat(("Giallo##" + std::to_string(i)).c_str(), &y, 1.0f, 6.0f);
                        changed |= ImGui::SliderFloat(("Rosso tutto##" + std::to_string(i)).c_str(), &r, 0.0f, 6.0f);
                        if (changed) {
                            thread.executeSynchronously([&, i, g, y, r](sim::SimulationWorld& world) {
                                if (network::IntersectionNode* node = world.network().findNode(sel_node)) {
                                    if (i < node->light_controller.phases.size()) {
                                        node->light_controller.phases[i].green_duration = g;
                                        node->light_controller.phases[i].yellow_duration = y;
                                        node->light_controller.phases[i].all_red_duration = r;
                                    }
                                }
                            });
                        }
                    }
                }
            }
        } else if (sel_road != core::INVALID_ID) {
            RoadInspectView view;
            thread.executeSynchronously([&](sim::SimulationWorld& world) {
                if (const network::RoadSegment* road = world.network().findRoad(sel_road)) {
                    view.valid = true;
                    view.id = road->id;
                    view.fwd_lanes = road->num_forward_lanes;
                    view.bwd_lanes = road->num_backward_lanes;
                    view.speed_limit = road->speed_limit;
                    view.curved = road->is_curved;
                }
            });
            if (view.valid) {
                ImGui::Text("Strada #%llu", static_cast<unsigned long long>(view.id));
                ImGui::Text("Corsie avanti: %u, indietro: %u", view.fwd_lanes, view.bwd_lanes);
                ImGui::Text("Limite velocita: %.1f km/h", static_cast<double>(view.speed_limit * 3.6f));
                ImGui::Text("%s", view.curved ? "Tipo: curva" : "Tipo: rettilinea");
            }
        } else if (sel_vehicle != core::INVALID_ID) {
            VehicleInspectView view;
            thread.executeSynchronously([&](sim::SimulationWorld& world) {
                if (auto it = world.vehicles().find(sel_vehicle); it != world.vehicles().end()) {
                    view.valid = true;
                    view.speed = it->second.speed;
                    view.acceleration = it->second.acceleration;
                    view.net_distance_s = it->second.perception_history.empty()
                                               ? -1.0f
                                               : it->second.perception_history.back().net_distance_s;
                }
            });
            if (view.valid) {
                ImGui::Text("Veicolo #%llu", static_cast<unsigned long long>(sel_vehicle));
                ImGui::Text("Velocita: %.1f km/h", static_cast<double>(view.speed * 3.6f));
                ImGui::Text("Accelerazione: %.2f m/s^2", static_cast<double>(view.acceleration));
                if (view.net_distance_s >= 0.0f) {
                    ImGui::Text("Distanza dal leader: %.1f m", static_cast<double>(view.net_distance_s));
                }
                if (ImGui::Button("Force Hard Brake")) {
                    thread.executeSynchronously([&](sim::SimulationWorld& world) {
                        world.applyPerturbationBrake(sel_vehicle, 0.0f, 3.0f);
                    });
                }
            } else {
                ImGui::Text("Nessun elemento selezionato");
            }
        } else {
            ImGui::Text("Nessun elemento selezionato");
        }

        ImGui::End();
    }

    void ImGuiPanels::renderPersistencePanel(sim::SimulationThread& thread, const sim::SimulationSnapshot& snapshot) {
        ImGui::SetNextWindowPos(ImVec2(1100.0f, 360.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(480.0f, 320.0f), ImGuiCond_FirstUseEver);
        ImGui::Begin("Persistence & Telemetry Panel");

        const char* mode_names[] = {"InfrastructureOnly", "FullState"};
        ImGui::Combo("Modalita", &save_mode_index_, mode_names, 2);
        ImGui::InputText("Percorso file", filepath_buffer_, sizeof(filepath_buffer_));

        if (ImGui::Button("Salva")) {
            const persistence::SaveMode mode =
                save_mode_index_ == 1 ? persistence::SaveMode::FullState : persistence::SaveMode::InfrastructureOnly;
            std::string err;
            bool ok = false;
            const std::string path = filepath_buffer_;
            thread.executeSynchronously([&](sim::SimulationWorld& world) {
                ok = persistence::JsonSerializer::saveToFile(path, world, mode, err);
            });
            status_message_ = ok ? "Salvataggio riuscito" : ("Errore: " + err);
        }
        ImGui::SameLine();
        if (ImGui::Button("Carica")) {
            std::string err;
            bool ok = false;
            const std::string path = filepath_buffer_;
            thread.executeSynchronously([&](sim::SimulationWorld& world) {
                ok = persistence::JsonSerializer::loadFromFile(path, world, err);
            });
            status_message_ = ok ? "Caricamento riuscito" : ("Errore: " + err);
        }

        if (!status_message_.empty()) {
            ImGui::TextWrapped("%s", status_message_.c_str());
        }

        speed_history_.push_back(snapshot.mean_speed_kmh);
        if (speed_history_.size() > 300) speed_history_.erase(speed_history_.begin());
        if (!speed_history_.empty()) {
            ImGui::PlotLines("Velocita media (km/h)", speed_history_.data(), static_cast<int>(speed_history_.size()));
        }
        ImGui::Text("Veicoli in coda: %zu", snapshot.jammed_vehicles_count);

        ImGui::End();
    }
}
