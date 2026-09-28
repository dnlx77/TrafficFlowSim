#pragma once
#include <cstdint>
#include <unordered_map>
#include "core/Types.hpp"
#include "core/Vec2.hpp"
#include "core/CubicBezier.hpp"
#include "network/RoadSegment.hpp"
#include "network/LaneConnector.hpp"
#include "network/IntersectionNode.hpp"

namespace tfs::network {
    class RoadNetwork {
    public:
        core::NodeId addNode(core::Vec2 pos, IntersectionRule rule = IntersectionRule::PriorityYield);
        void moveNode(core::NodeId id, core::Vec2 new_pos);
        void removeNode(core::NodeId id);

        core::RoadId addRoad(core::NodeId start_node, core::NodeId end_node, std::uint32_t fwd_lanes,
                              std::uint32_t bwd_lanes, float speed_limit = 13.89f, bool curved = false,
                              core::Vec2 p1 = {}, core::Vec2 p2 = {});
        void removeRoad(core::RoadId id);
        void updateRoadControlPoints(core::RoadId id, core::Vec2 p1, core::Vec2 p2);

        void rebuildIntersection(core::NodeId node_id);

        core::NodeId addNodeWithId(core::NodeId id, core::Vec2 pos, IntersectionRule rule);
        core::RoadId addRoadWithId(core::RoadId id, core::NodeId start_node, core::NodeId end_node,
                                    std::uint32_t fwd_lanes, std::uint32_t bwd_lanes, float speed_limit,
                                    bool curved, core::Vec2 p1, core::Vec2 p2);

        [[nodiscard]] const std::unordered_map<core::NodeId, IntersectionNode>& nodes() const noexcept { return nodes_; }
        [[nodiscard]] std::unordered_map<core::NodeId, IntersectionNode>& nodesMutable() noexcept { return nodes_; }
        [[nodiscard]] const std::unordered_map<core::RoadId, RoadSegment>& roads() const noexcept { return roads_; }
        [[nodiscard]] std::unordered_map<core::RoadId, RoadSegment>& roadsMutable() noexcept { return roads_; }
        [[nodiscard]] const std::unordered_map<core::ConnectorId, LaneConnector>& connectors() const noexcept { return connectors_; }
        [[nodiscard]] std::unordered_map<core::ConnectorId, LaneConnector>& connectorsMutable() noexcept { return connectors_; }

        [[nodiscard]] const IntersectionNode* findNode(core::NodeId id) const;
        [[nodiscard]] IntersectionNode* findNode(core::NodeId id);
        [[nodiscard]] const RoadSegment* findRoad(core::RoadId id) const;
        [[nodiscard]] RoadSegment* findRoad(core::RoadId id);
        [[nodiscard]] const LaneConnector* findConnector(core::ConnectorId id) const;
        [[nodiscard]] LaneConnector* findConnector(core::ConnectorId id);
        [[nodiscard]] const Lane* findLane(const core::LaneRef& ref) const;
        [[nodiscard]] Lane* findLaneMutable(const core::LaneRef& ref);

    private:
        std::unordered_map<core::NodeId, IntersectionNode> nodes_;
        std::unordered_map<core::RoadId, RoadSegment> roads_;
        std::unordered_map<core::ConnectorId, LaneConnector> connectors_;

        core::NodeId next_node_id_{1};
        core::RoadId next_road_id_{1};
        core::ConnectorId next_connector_id_{1};

        [[nodiscard]] bool hasPriority(const LaneConnector& a, const LaneConnector& b, const IntersectionNode& node) const;
        [[nodiscard]] bool findTrajectoryIntersection(const core::CubicBezier& a, const core::CubicBezier& b,
                                                       float& out_s_a, float& out_s_b) const;
        void buildTrafficLightPhases(IntersectionNode& node);
    };
}
