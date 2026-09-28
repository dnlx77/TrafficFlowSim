#include "network/TrafficLight.hpp"
#include <algorithm>

namespace tfs::network {

    void TrafficLightController::update(float dt) noexcept {
        if (phases.empty()) return;
        phase_timer += dt;
        const TrafficLightPhase& phase = phases[current_phase_idx];

        switch (current_substate) {
            case LightState::Green:
                if (phase_timer >= phase.green_duration) {
                    phase_timer = 0.0f;
                    current_substate = LightState::Yellow;
                }
                break;
            case LightState::Yellow:
                if (phase_timer >= phase.yellow_duration) {
                    phase_timer = 0.0f;
                    current_substate = LightState::Red;
                }
                break;
            case LightState::Red:
                if (phase_timer >= phase.all_red_duration) {
                    phase_timer = 0.0f;
                    current_substate = LightState::Green;
                    current_phase_idx = (current_phase_idx + 1) % phases.size();
                }
                break;
        }
    }

    LightState TrafficLightController::getStateForConnector(core::ConnectorId cid) const noexcept {
        if (phases.empty()) return LightState::Red;
        const TrafficLightPhase& phase = phases[current_phase_idx];
        const bool active = std::find(phase.active_connectors.begin(), phase.active_connectors.end(), cid)
                             != phase.active_connectors.end();
        if (!active) return LightState::Red;
        return current_substate;
    }
}
