#pragma once
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include "sim/SimulationWorld.hpp"
#include "sim/SimulationSnapshot.hpp"

namespace tfs::sim {
    class SimulationThread {
    public:
        using SimCommand = std::move_only_function<void(SimulationWorld&)>;

        SimulationThread();
        ~SimulationThread();

        void start();
        void stop();
        void setPaused(bool paused);
        [[nodiscard]] bool isPaused() const;
        void setTimeScale(float scale);
        void stepSingleTick();

        void postCommand(SimCommand cmd);
        void executeSynchronously(const std::function<void(SimulationWorld&)>& fn);

        [[nodiscard]] std::shared_ptr<const SimulationSnapshot> getLatestSnapshot() const;

        [[nodiscard]] SimulationWorld& initialSetup() noexcept { return world_; }

    private:
        SimulationWorld world_;
        std::jthread worker_;

        std::atomic<bool> paused_{false};
        std::atomic<float> time_scale_{1.0f};
        std::atomic<bool> single_step_requested_{false};
        std::atomic<std::shared_ptr<const SimulationSnapshot>> latest_snapshot_;

        std::mutex cmd_mutex_;
        std::vector<SimCommand> pending_commands_;

        void run(std::stop_token stoken);
    };
}
