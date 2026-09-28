#pragma once
#include <SFML/Graphics/RenderWindow.hpp>
#include "network/RoadNetwork.hpp"
#include "sim/SimulationThread.hpp"
#include "ui/Camera2D.hpp"
#include "ui/ImGuiPanels.hpp"
#include "ui/NetworkEditor.hpp"
#include "ui/WorldRenderer.hpp"

namespace tfs::app {
    class Application {
    public:
        Application();

        int run();

    private:
        sf::RenderWindow window_;
        sim::SimulationThread sim_thread_;
        network::RoadNetwork cached_network_;
        ui::Camera2D camera_;
        ui::WorldRenderer world_renderer_;
        ui::NetworkEditor network_editor_;
        ui::ImGuiPanels imgui_panels_;
        ui::VehicleColorMode color_mode_{ui::VehicleColorMode::BySpeed};

        void handleEvent(const sf::Event& event);
        void loadInitialScenario();
    };
}
