#pragma once
#include <vector>
#include "core/Types.hpp"
#include "core/CubicBezier.hpp"

namespace tfs::network {
    struct Lane {
        core::LaneRef ref;
        core::CubicBezier center_curve;
        float width{3.5f};
        float speed_limit{13.89f};
        std::vector<core::ConnectorId> outgoing_connectors;
        std::vector<core::ConnectorId> incoming_connectors;

        [[nodiscard]] float length() const noexcept { return center_curve.totalLength(); }
    };
}
