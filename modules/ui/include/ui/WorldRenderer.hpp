#pragma once
#include <SFML/Graphics/RenderTarget.hpp>
#include "network/RoadNetwork.hpp"
#include "sim/SimulationSnapshot.hpp"
#include "ui/Camera2D.hpp"

namespace tfs::ui {
    enum class VehicleColorMode {
        BySpeed,
        ByClass,
        ByAcceleration
    };

    class WorldRenderer {
    public:
        void render(sf::RenderTarget& target, const network::RoadNetwork& network,
                    const sim::SimulationSnapshot& snapshot, const Camera2D& camera,
                    VehicleColorMode color_mode, float blink_phase);
    };
}
