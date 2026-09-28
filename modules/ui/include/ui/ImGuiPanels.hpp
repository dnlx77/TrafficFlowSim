#pragma once
#include <string>
#include <vector>
#include <SFML/System/Vector2.hpp>
#include "persistence/JsonSerializer.hpp"
#include "sim/SimulationThread.hpp"
#include "ui/Camera2D.hpp"
#include "ui/NetworkEditor.hpp"

namespace tfs::ui {
    class ImGuiPanels {
    public:
        void render(sim::SimulationThread& thread, NetworkEditor& editor, Camera2D& camera,
                    const sim::SimulationSnapshot& snapshot, sf::Vector2u target_size);

    private:
        float time_scale_{1.0f};
        bool paused_{false};
        bool ou_noise_enabled_{true};
        float reaction_delay_multiplier_{1.0f};

        int new_fwd_lanes_{1};
        int new_bwd_lanes_{1};
        float new_node_radius_{12.0f};
        int spawn_class_index_{0};

        char filepath_buffer_[256]{"scenario.json"};
        int save_mode_index_{0};
        std::string status_message_;
        std::string scenario_status_message_;

        std::vector<float> speed_history_;

        void renderSimulationControlPanel(sim::SimulationThread& thread, Camera2D& camera,
                                           const sim::SimulationSnapshot& snapshot, sf::Vector2u target_size);
        void renderEditorToolbarPanel(NetworkEditor& editor);
        void renderInspectorPanel(sim::SimulationThread& thread, NetworkEditor& editor);
        void renderPersistencePanel(sim::SimulationThread& thread, const sim::SimulationSnapshot& snapshot);

        bool loadScenario(sim::SimulationThread& thread, Camera2D& camera, const std::string& path,
                           sf::Vector2u target_size, std::string& out_error);
    };
}
