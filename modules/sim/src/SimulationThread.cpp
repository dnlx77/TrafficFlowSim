#include "sim/SimulationThread.hpp"
#include <chrono>
#include <future>

namespace tfs::sim {

    SimulationThread::SimulationThread() {
        latest_snapshot_.store(std::make_shared<const SimulationSnapshot>(), std::memory_order_relaxed);
    }

    SimulationThread::~SimulationThread() {
        stop();
    }

    void SimulationThread::start() {
        if (worker_.joinable()) return;
        worker_ = std::jthread([this](std::stop_token st) { run(st); });
    }

    void SimulationThread::stop() {
        if (worker_.joinable()) {
            worker_.request_stop();
            worker_.join();
        }
    }

    void SimulationThread::setPaused(bool paused) {
        paused_.store(paused, std::memory_order_relaxed);
    }

    bool SimulationThread::isPaused() const {
        return paused_.load(std::memory_order_relaxed);
    }

    void SimulationThread::setTimeScale(float scale) {
        time_scale_.store(scale, std::memory_order_relaxed);
    }

    void SimulationThread::stepSingleTick() {
        single_step_requested_.store(true, std::memory_order_relaxed);
    }

    void SimulationThread::postCommand(SimCommand cmd) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        pending_commands_.push_back(std::move(cmd));
    }

    void SimulationThread::executeSynchronously(const std::function<void(SimulationWorld&)>& fn) {
        if (!worker_.joinable()) {
            fn(world_);
            return;
        }
        auto done = std::make_shared<std::promise<void>>();
        std::future<void> fut = done->get_future();
        postCommand([&fn, done](SimulationWorld& w) {
            fn(w);
            done->set_value();
        });
        fut.wait();
    }

    std::shared_ptr<const SimulationSnapshot> SimulationThread::getLatestSnapshot() const {
        return latest_snapshot_.load(std::memory_order_acquire);
    }

    void SimulationThread::run(std::stop_token stoken) {
        using clock = std::chrono::steady_clock;
        constexpr float kFixedDt = 0.01f;
        auto next_tick = clock::now();

        while (!stoken.stop_requested()) {
            std::vector<SimCommand> commands;
            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                commands.swap(pending_commands_);
            }
            for (auto& cmd : commands) cmd(world_);

            const bool single_step = single_step_requested_.exchange(false, std::memory_order_relaxed);
            const bool is_paused = paused_.load(std::memory_order_relaxed);

            if (!is_paused || single_step) {
                world_.step(kFixedDt);
                latest_snapshot_.store(std::make_shared<const SimulationSnapshot>(world_.buildSnapshot()),
                                        std::memory_order_release);

                const float scale = std::max(time_scale_.load(std::memory_order_relaxed), 0.01f);
                const auto period = std::chrono::duration_cast<clock::duration>(std::chrono::duration<float>(kFixedDt / scale));
                next_tick += period;
                if (clock::now() < next_tick) {
                    std::this_thread::sleep_until(next_tick);
                } else {
                    next_tick = clock::now();
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                next_tick = clock::now();
            }
        }
    }
}
