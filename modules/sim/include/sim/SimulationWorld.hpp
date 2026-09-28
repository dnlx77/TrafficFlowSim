#pragma once
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "core/Types.hpp"
#include "core/Random.hpp"
#include "network/RoadNetwork.hpp"
#include "sim/Vehicle.hpp"
#include "sim/Spawner.hpp"
#include "sim/SimulationSnapshot.hpp"

namespace tfs::sim {
    class SimulationWorld {
    public:
        void step(float dt);

        core::VehicleId spawnVehicleManual(const core::LaneRef& lane, float s, VehicleClass vclass);
        core::VehicleId restoreVehicle(const Vehicle& saved);
        void applyPerturbationBrake(core::VehicleId vid, float target_speed, float duration);
        [[nodiscard]] SimulationSnapshot buildSnapshot() const;

        core::SpawnerId addSpawner(const Spawner& spawner);
        void removeAllVehicles();

        void setOUNoiseEnabled(bool enabled) noexcept { ou_noise_enabled_ = enabled; }
        [[nodiscard]] bool isOUNoiseEnabled() const noexcept { return ou_noise_enabled_; }
        void setReactionDelayMultiplier(float multiplier) noexcept { reaction_delay_multiplier_ = multiplier; }
        [[nodiscard]] float reactionDelayMultiplier() const noexcept { return reaction_delay_multiplier_; }

        [[nodiscard]] network::RoadNetwork& network() noexcept { return network_; }
        [[nodiscard]] const network::RoadNetwork& network() const noexcept { return network_; }
        [[nodiscard]] std::unordered_map<core::VehicleId, Vehicle>& vehicles() noexcept { return vehicles_; }
        [[nodiscard]] const std::unordered_map<core::VehicleId, Vehicle>& vehicles() const noexcept { return vehicles_; }
        [[nodiscard]] std::vector<Spawner>& spawners() noexcept { return spawners_; }
        [[nodiscard]] const std::vector<Spawner>& spawners() const noexcept { return spawners_; }
        [[nodiscard]] core::RandomEngine& randomEngine() noexcept { return rng_; }

    private:
        network::RoadNetwork network_;
        std::unordered_map<core::VehicleId, Vehicle> vehicles_;
        std::vector<Spawner> spawners_;
        core::RandomEngine rng_;

        core::VehicleId next_vehicle_id_{1};
        core::SpawnerId next_spawner_id_{1};
        double sim_time_sec_{0.0};
        std::uint64_t tick_count_{0};
        bool ou_noise_enabled_{true};
        float reaction_delay_multiplier_{1.0f};

        struct ActiveBraking {
            float target_speed;
            float remaining_time;
        };
        std::unordered_map<core::VehicleId, ActiveBraking> active_brakes_;
        std::unordered_map<core::NodeId, float> intersection_clearance_hold_;

        void buildOccupancy(std::unordered_map<core::LaneRef, std::vector<const Vehicle*>>& lane_occ,
                             std::unordered_map<core::ConnectorId, std::vector<const Vehicle*>>& connector_occ);
        void updateTrafficLights(float dt, const std::unordered_map<core::ConnectorId, std::vector<const Vehicle*>>& connector_occ);
        void updateLaneChanges(float dt, std::unordered_map<core::LaneRef, std::vector<const Vehicle*>>& lane_occ);
        void updateVehicleDynamics(float dt,
                                    const std::unordered_map<core::LaneRef, std::vector<const Vehicle*>>& lane_occ,
                                    const std::unordered_map<core::ConnectorId, std::vector<const Vehicle*>>& connector_occ);
        void updatePerturbations(float dt);
        void integrate(float dt);
        void updateTransitionsAndPositions();
        void updateSpawners(float dt);
    };
}
