#include "app/Application.hpp"
#include <optional>
#include <imgui-SFML.h>
#include <imgui.h>
#include "core/Logger.hpp"

namespace tfs::app {

    Application::Application()
        : window_(sf::VideoMode({1600u, 900u}), "TrafficFlowSim - Visual Studio 2026", sf::Style::Default) {
        core::Logger::init();
        window_.setFramerateLimit(60);

        if (!ImGui::SFML::Init(window_, false)) {
            core::Logger::error("Impossibile inizializzare ImGui-SFML");
        }
        if (!ImGui::GetIO().Fonts->AddFontFromFileTTF("assets/fonts/JetBrainsMono-Regular.ttf", 16.0f)) {
            core::Logger::warn("Impossibile caricare assets/fonts/JetBrainsMono-Regular.ttf, uso il font di default");
            ImGui::GetIO().Fonts->AddFontDefault();
        }
        if (!ImGui::SFML::UpdateFontTexture()) {
            core::Logger::error("Impossibile costruire la texture del font");
        }

        loadInitialScenario();
        sim_thread_.start();
    }

    void Application::loadInitialScenario() {
        sim::SimulationWorld& world = sim_thread_.initialSetup();
        network::RoadNetwork& net = world.network();

        const core::NodeId n1 = net.addNode({-200.0f, 0.0f});
        const core::NodeId n2 = net.addNode({200.0f, 0.0f});
        const core::RoadId road_id = net.addRoad(n1, n2, 2, 2, 13.89f);

        if (road_id != core::INVALID_ID) {
            world.spawnVehicleManual(core::LaneRef{road_id, true, 0}, 10.0f, sim::VehicleClass::PassengerCar);

            sim::Spawner spawner;
            spawner.target_lane = core::LaneRef{road_id, true, 0};
            spawner.flow_rate_vph = 600.0f;
            world.addSpawner(spawner);
        }

        cached_network_ = net;
        camera_.frameBounds({-200.0f, 0.0f}, {200.0f, 0.0f}, window_.getSize());
    }

    void Application::handleEvent(const sf::Event& event) {
        ImGui::SFML::ProcessEvent(window_, event);

        if (event.is<sf::Event::Closed>()) {
            window_.close();
            return;
        }

        if (ImGui::GetIO().WantCaptureMouse) return;

        camera_.handleEvent(event, window_.getSize());

        if (const auto* pressed = event.getIf<sf::Event::MouseButtonPressed>()) {
            if (pressed->button == sf::Mouse::Button::Left) {
                const core::Vec2 world_pos = camera_.screenToWorld(pressed->position, window_.getSize());
                network_editor_.handleClick(world_pos, sim_thread_);
                network_editor_.handleDragStart(world_pos, sim_thread_);
            }
        } else if (const auto* moved = event.getIf<sf::Event::MouseMoved>()) {
            if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
                const core::Vec2 world_pos = camera_.screenToWorld(moved->position, window_.getSize());
                network_editor_.handleDrag(world_pos, sim_thread_);
            }
        } else if (const auto* released = event.getIf<sf::Event::MouseButtonReleased>()) {
            if (released->button == sf::Mouse::Button::Left) {
                network_editor_.handleDragEnd();
            }
        }
    }

    int Application::run() {
        sf::Clock delta_clock;
        float blink_clock = 0.0f;

        while (window_.isOpen()) {
            while (const std::optional event = window_.pollEvent()) {
                handleEvent(*event);
            }

            const sf::Time dt = delta_clock.restart();
            blink_clock += dt.asSeconds();
            ImGui::SFML::Update(window_, dt);

            sim_thread_.executeSynchronously([this](sim::SimulationWorld& world) {
                cached_network_ = world.network();
            });

            const std::shared_ptr<const sim::SimulationSnapshot> snapshot = sim_thread_.getLatestSnapshot();

            imgui_panels_.render(sim_thread_, network_editor_, camera_, *snapshot, window_.getSize());

            window_.clear(sf::Color(25, 25, 28));
            world_renderer_.render(window_, cached_network_, *snapshot, camera_, color_mode_, blink_clock);
            ImGui::SFML::Render(window_);
            window_.display();
        }

        sim_thread_.stop();
        ImGui::SFML::Shutdown(window_);
        return 0;
    }
}
