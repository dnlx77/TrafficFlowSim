#pragma once
#include "core/Types.hpp"
#include "sim/Vehicle.hpp"

namespace tfs::sim {
    struct LaneChangeCandidate {
        bool feasible{false};
        core::LaneIndex target_lane_idx{0};
        float incentive{0.0f};
    };

    class MOBILModel {
    public:
        static LaneChangeCandidate evaluate(const Vehicle& ego, const Vehicle* cur_lead, const Vehicle* cur_fol,
                                             const Vehicle* tgt_lead, const Vehicle* tgt_fol,
                                             core::LaneIndex target_idx, float target_speed_limit) noexcept;
    };
}
