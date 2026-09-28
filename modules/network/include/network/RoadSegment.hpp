#pragma once
#include <cstdint>
#include <vector>
#include "core/Types.hpp"
#include "core/Vec2.hpp"
#include "core/CubicBezier.hpp"
#include "network/Lane.hpp"

namespace tfs::network {
    class RoadSegment {
    public:
        core::RoadId id{core::INVALID_ID};
        core::NodeId start_node{core::INVALID_ID};
        core::NodeId end_node{core::INVALID_ID};
        bool is_curved{false};
        core::CubicBezier baseline_curve;
        std::uint32_t num_forward_lanes{1};
        std::uint32_t num_backward_lanes{1};
        float lane_width{3.5f};
        float speed_limit{13.89f};
        std::vector<Lane> forward_lanes;
        std::vector<Lane> backward_lanes;

        void rebuildGeometry(core::Vec2 start_pos, core::Vec2 end_pos);

        [[nodiscard]] const Lane& getLane(const core::LaneRef& ref) const;
        [[nodiscard]] Lane& getLaneMutable(const core::LaneRef& ref);
    };
}
