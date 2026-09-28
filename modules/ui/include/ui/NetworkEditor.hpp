#pragma once
#include "core/Types.hpp"
#include "core/Vec2.hpp"
#include "network/RoadNetwork.hpp"
#include "sim/SimulationThread.hpp"

namespace tfs::ui {
    enum class EditorTool {
        SelectInspect,
        AddNode,
        AddStraightRoad,
        AddCurvedRoad,
        PlaceSpawner,
        SpawnSingleVehicle,
        DeleteElement
    };

    class NetworkEditor {
    public:
        void setTool(EditorTool tool) noexcept { tool_ = tool; pending_start_node_ = core::INVALID_ID; }
        [[nodiscard]] EditorTool tool() const noexcept { return tool_; }

        void setNewRoadLaneCounts(std::uint32_t fwd, std::uint32_t bwd) noexcept { new_fwd_lanes_ = fwd; new_bwd_lanes_ = bwd; }
        void setNewNodeRadius(float radius) noexcept { new_node_radius_ = radius; }
        void setSpawnVehicleClass(sim::VehicleClass vclass) noexcept { spawn_class_ = vclass; }

        void handleClick(core::Vec2 world_pos, sim::SimulationThread& thread);
        void handleDragStart(core::Vec2 world_pos, sim::SimulationThread& thread);
        void handleDrag(core::Vec2 world_pos, sim::SimulationThread& thread);
        void handleDragEnd();

        void clearSelection() noexcept {
            selected_node_ = core::INVALID_ID;
            selected_road_ = core::INVALID_ID;
            selected_vehicle_ = core::INVALID_ID;
        }

        [[nodiscard]] core::NodeId selectedNode() const noexcept { return selected_node_; }
        [[nodiscard]] core::RoadId selectedRoad() const noexcept { return selected_road_; }
        [[nodiscard]] core::VehicleId selectedVehicle() const noexcept { return selected_vehicle_; }

    private:
        EditorTool tool_{EditorTool::SelectInspect};
        std::uint32_t new_fwd_lanes_{1};
        std::uint32_t new_bwd_lanes_{1};
        float new_node_radius_{12.0f};
        sim::VehicleClass spawn_class_{sim::VehicleClass::PassengerCar};

        core::NodeId pending_start_node_{core::INVALID_ID};
        core::NodeId selected_node_{core::INVALID_ID};
        core::RoadId selected_road_{core::INVALID_ID};
        core::VehicleId selected_vehicle_{core::INVALID_ID};

        enum class DragKind { None, Node, ControlP1, ControlP2 };
        DragKind drag_kind_{DragKind::None};
        core::RoadId drag_road_{core::INVALID_ID};
        core::NodeId drag_node_{core::INVALID_ID};

        static core::NodeId findNearestNode(core::Vec2 pos, const network::RoadNetwork& net, float max_dist);
        static core::RoadId findNearestRoad(core::Vec2 pos, const network::RoadNetwork& net, float max_dist, core::Vec2& out_closest);
        static core::VehicleId findNearestVehicle(core::Vec2 pos, const sim::SimulationWorld& world, float max_dist);
    };
}
