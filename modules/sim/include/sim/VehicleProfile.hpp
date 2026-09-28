#pragma once

namespace tfs::sim {
    enum class VehicleClass {
        PassengerCar,
        HeavyTruck,
        AggressiveDriver
    };

    struct VehicleProfile {
        VehicleClass type{VehicleClass::PassengerCar};
        float length{4.5f};
        float width{1.9f};
        float desired_speed{33.3f};
        float min_gap{2.0f};
        float time_headway{1.4f};
        float max_accel{1.8f};
        float comfort_decel{2.2f};
        float max_decel{8.5f};
        float politeness{0.25f};
        float lane_change_threshold{0.15f};
        float reaction_delay{0.6f};
        float ou_sigma{0.40f};

        static VehicleProfile makeCar();
        static VehicleProfile makeTruck();
        static VehicleProfile makeAggressive();
    };
}
