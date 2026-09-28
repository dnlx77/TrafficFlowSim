#pragma once
#include <cstdint>
#include <compare>
#include <functional>

namespace tfs::core {
    using NodeId      = std::uint64_t;
    using RoadId      = std::uint64_t;
    using LaneIndex   = std::uint32_t;
    using ConnectorId = std::uint64_t;
    using VehicleId   = std::uint64_t;
    using SpawnerId   = std::uint64_t;
    inline constexpr std::uint64_t INVALID_ID = 0;

    struct LaneRef {
        RoadId road_id{INVALID_ID};
        bool is_forward{true};
        LaneIndex lane_idx{0};
        auto operator<=>(const LaneRef&) const = default;
        bool operator==(const LaneRef&) const = default;
    };
}

template <>
struct std::hash<tfs::core::LaneRef> {
    std::size_t operator()(const tfs::core::LaneRef& ref) const noexcept {
        std::size_t h1 = std::hash<tfs::core::RoadId>{}(ref.road_id);
        std::size_t h2 = std::hash<bool>{}(ref.is_forward);
        std::size_t h3 = std::hash<tfs::core::LaneIndex>{}(ref.lane_idx);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};
