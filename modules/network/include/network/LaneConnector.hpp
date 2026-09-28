#pragma once
#include <vector>
#include "core/Types.hpp"
#include "core/CubicBezier.hpp"

namespace tfs::network {
    enum class TurnDirection {
        Straight,
        Left,
        Right,
        UTurn
    };

    struct ConflictPoint {
        core::ConnectorId other_connector{core::INVALID_ID};
        float my_s{0.0f};
        float other_s{0.0f};
        bool other_has_priority{false};
    };

    struct LaneConnector {
        core::ConnectorId id{core::INVALID_ID};
        core::NodeId parent_node{core::INVALID_ID};
        core::LaneRef from_lane;
        core::LaneRef to_lane;
        TurnDirection turn_type{TurnDirection::Straight};
        core::CubicBezier trajectory;
        float speed_limit{8.33f};
        std::vector<ConflictPoint> conflicts;
    };
}
