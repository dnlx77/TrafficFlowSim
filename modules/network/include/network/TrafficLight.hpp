#pragma once
#include <vector>
#include <cstddef>
#include "core/Types.hpp"

namespace tfs::network {
    enum class LightState {
        Green,
        Yellow,
        Red
    };

    struct TrafficLightPhase {
        float green_duration{14.0f};
        float yellow_duration{3.0f};
        float all_red_duration{2.0f};
        std::vector<core::ConnectorId> active_connectors;
    };

    class TrafficLightController {
    public:
        std::vector<TrafficLightPhase> phases;
        std::size_t current_phase_idx{0};
        float phase_timer{0.0f};
        LightState current_substate{LightState::Green};

        void update(float dt) noexcept;
        [[nodiscard]] LightState getStateForConnector(core::ConnectorId cid) const noexcept;
    };
}
