#include "sim/Spawner.hpp"
#include <cmath>
#include <algorithm>

namespace tfs::sim {

    bool Spawner::update(float dt, core::RandomEngine& rng) noexcept {
        if (!enabled) return false;
        time_to_next_spawn -= dt;
        if (time_to_next_spawn <= 0.0f) {
            const float lambda = std::max(flow_rate_vph / 3600.0f, 1e-6f);
            const float u = std::max(rng.uniform(0.0f, 1.0f), 1e-6f);
            time_to_next_spawn = -std::log(u) / lambda;
            return true;
        }
        return false;
    }

    VehicleClass Spawner::pickVehicleClass(core::RandomEngine& rng) const noexcept {
        const float r = rng.uniform(0.0f, 1.0f);
        if (r < truck_ratio) return VehicleClass::HeavyTruck;
        if (r < truck_ratio + aggressive_ratio) return VehicleClass::AggressiveDriver;
        return VehicleClass::PassengerCar;
    }
}
