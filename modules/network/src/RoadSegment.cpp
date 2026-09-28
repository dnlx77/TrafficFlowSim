#include "network/RoadSegment.hpp"
#include <stdexcept>
#include <utility>

namespace tfs::network {

    void RoadSegment::rebuildGeometry(core::Vec2 start_pos, core::Vec2 end_pos) {
        if (!is_curved) {
            baseline_curve = core::CubicBezier::makeStraight(start_pos, end_pos);
        }

        forward_lanes.clear();
        backward_lanes.clear();

        const bool two_way = (num_forward_lanes > 0 && num_backward_lanes > 0);

        if (num_forward_lanes > 0) {
            for (std::uint32_t i = 0; i < num_forward_lanes; ++i) {
                float offset;
                if (two_way) {
                    offset = (static_cast<float>(num_forward_lanes - 1 - i) + 0.5f) * lane_width;
                } else {
                    const float total_width = static_cast<float>(num_forward_lanes) * lane_width;
                    offset = (static_cast<float>(i) + 0.5f) * lane_width - total_width * 0.5f;
                }
                Lane lane;
                lane.ref = core::LaneRef{id, true, i};
                lane.center_curve = baseline_curve.computeOffsetCurve(offset);
                lane.width = lane_width;
                lane.speed_limit = speed_limit;
                forward_lanes.push_back(std::move(lane));
            }
        }

        if (num_backward_lanes > 0) {
            const core::CubicBezier reversed(baseline_curve.p3(), baseline_curve.p2(), baseline_curve.p1(), baseline_curve.p0());
            for (std::uint32_t j = 0; j < num_backward_lanes; ++j) {
                float offset;
                if (two_way) {
                    offset = (static_cast<float>(num_backward_lanes - 1 - j) + 0.5f) * lane_width;
                } else {
                    const float total_width = static_cast<float>(num_backward_lanes) * lane_width;
                    offset = (static_cast<float>(j) + 0.5f) * lane_width - total_width * 0.5f;
                }
                Lane lane;
                lane.ref = core::LaneRef{id, false, j};
                lane.center_curve = reversed.computeOffsetCurve(offset);
                lane.width = lane_width;
                lane.speed_limit = speed_limit;
                backward_lanes.push_back(std::move(lane));
            }
        }
    }

    const Lane& RoadSegment::getLane(const core::LaneRef& ref) const {
        const std::vector<Lane>& lanes = ref.is_forward ? forward_lanes : backward_lanes;
        for (const Lane& lane : lanes) {
            if (lane.ref.lane_idx == ref.lane_idx) return lane;
        }
        throw std::out_of_range("RoadSegment::getLane: lane reference not found");
    }

    Lane& RoadSegment::getLaneMutable(const core::LaneRef& ref) {
        std::vector<Lane>& lanes = ref.is_forward ? forward_lanes : backward_lanes;
        for (Lane& lane : lanes) {
            if (lane.ref.lane_idx == ref.lane_idx) return lane;
        }
        throw std::out_of_range("RoadSegment::getLaneMutable: lane reference not found");
    }
}
