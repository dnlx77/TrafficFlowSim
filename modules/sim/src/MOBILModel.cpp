#include "sim/MOBILModel.hpp"
#include "sim/IDMModel.hpp"

namespace tfs::sim {

    namespace {
        constexpr float kSafeBrakingLimit = -4.0f;

        float gapTo(const Vehicle* leader, float follower_s) noexcept {
            if (!leader) return 1000.0f;
            return leader->s - leader->profile.length - follower_s;
        }

        float approachRate(const Vehicle* leader, float follower_speed) noexcept {
            if (!leader) return 0.0f;
            return follower_speed - leader->speed;
        }

        float accelFor(const Vehicle& v, const Vehicle* leader, float speed_limit) noexcept {
            const float gap = gapTo(leader, v.s);
            const float dv = approachRate(leader, v.speed);
            return IDMModel::computeAcceleration(v.profile, v.speed, speed_limit, gap, dv);
        }
    }

    LaneChangeCandidate MOBILModel::evaluate(const Vehicle& ego, const Vehicle* cur_lead, const Vehicle* cur_fol,
                                              const Vehicle* tgt_lead, const Vehicle* tgt_fol,
                                              core::LaneIndex target_idx, float target_speed_limit) noexcept {
        LaneChangeCandidate result;
        result.target_lane_idx = target_idx;

        const float gap_lead_new = gapTo(tgt_lead, ego.s);
        const float gap_fol_new = tgt_fol ? (ego.s - ego.profile.length - tgt_fol->s) : 1000.0f;
        if (gap_lead_new <= ego.profile.min_gap || gap_fol_new <= ego.profile.min_gap) {
            return result;
        }

        const float a_new_fol_before = tgt_fol ? accelFor(*tgt_fol, tgt_lead, target_speed_limit) : 0.0f;
        const float a_new_fol_after = tgt_fol ? accelFor(*tgt_fol, &ego, target_speed_limit) : 0.0f;
        if (tgt_fol && a_new_fol_after < kSafeBrakingLimit) {
            return result;
        }

        const float a_ego = accelFor(ego, cur_lead, target_speed_limit);
        const float a_ego_new = accelFor(ego, tgt_lead, target_speed_limit);

        const float a_old_fol_before = cur_fol ? accelFor(*cur_fol, &ego, target_speed_limit) : 0.0f;
        const float a_old_fol_after = cur_fol ? accelFor(*cur_fol, cur_lead, target_speed_limit) : 0.0f;

        const float p = ego.profile.politeness;
        const float incentive = (a_ego_new - a_ego)
            + p * ((tgt_fol ? (a_new_fol_after - a_new_fol_before) : 0.0f)
                 + (cur_fol ? (a_old_fol_after - a_old_fol_before) : 0.0f));

        if (incentive > ego.profile.lane_change_threshold) {
            result.feasible = true;
            result.incentive = incentive;
        }
        return result;
    }
}
