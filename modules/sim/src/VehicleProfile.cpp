#include "sim/VehicleProfile.hpp"

namespace tfs::sim {

    VehicleProfile VehicleProfile::makeCar() {
        VehicleProfile p;
        p.type = VehicleClass::PassengerCar;
        return p;
    }

    VehicleProfile VehicleProfile::makeTruck() {
        VehicleProfile p;
        p.type = VehicleClass::HeavyTruck;
        p.length = 12.0f;
        p.width = 2.5f;
        p.desired_speed = 22.0f;
        p.min_gap = 3.0f;
        p.time_headway = 1.8f;
        p.max_accel = 1.0f;
        p.comfort_decel = 1.5f;
        p.max_decel = 6.0f;
        p.politeness = 0.1f;
        p.lane_change_threshold = 0.3f;
        p.reaction_delay = 0.8f;
        p.ou_sigma = 0.2f;
        return p;
    }

    VehicleProfile VehicleProfile::makeAggressive() {
        VehicleProfile p;
        p.type = VehicleClass::AggressiveDriver;
        p.length = 4.5f;
        p.width = 1.9f;
        p.desired_speed = 41.7f;
        p.min_gap = 1.0f;
        p.time_headway = 0.8f;
        p.max_accel = 2.5f;
        p.comfort_decel = 3.0f;
        p.max_decel = 9.0f;
        p.politeness = 0.05f;
        p.lane_change_threshold = 0.05f;
        p.reaction_delay = 0.3f;
        p.ou_sigma = 0.6f;
        return p;
    }
}
