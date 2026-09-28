#pragma once
#include "core/Types.hpp"
#include "core/Random.hpp"
#include "sim/VehicleProfile.hpp"

namespace tfs::sim {
    struct Spawner {
        core::SpawnerId id{core::INVALID_ID};
        core::LaneRef target_lane;
        float flow_rate_vph{800.0f};
        float truck_ratio{0.15f};
        float aggressive_ratio{0.20f};
        float time_to_next_spawn{0.0f};
        bool enabled{true};

        bool update(float dt, core::RandomEngine& rng) noexcept;
        [[nodiscard]] VehicleClass pickVehicleClass(core::RandomEngine& rng) const noexcept;
    };
}
