#include "core/Random.hpp"
#include <cmath>

namespace tfs::core {

    void RandomEngine::seed(std::uint64_t s) {
        engine_.seed(s);
    }

    float RandomEngine::uniform(float min, float max) {
        std::uniform_real_distribution<float> dist(min, max);
        return dist(engine_);
    }

    float RandomEngine::normal(float mean, float stddev) {
        std::normal_distribution<float> dist(mean, stddev);
        return dist(engine_);
    }

    bool RandomEngine::bernoulli(float p) {
        std::bernoulli_distribution dist(p);
        return dist(engine_);
    }

    float OrnsteinUhlenbeckProcess::step(float dt, RandomEngine& rng) noexcept {
        const float drift = -(state / tau) * dt;
        const float diffusion = sigma * std::sqrt(2.0f / tau) * std::sqrt(dt) * rng.normal(0.0f, 1.0f);
        state += drift + diffusion;
        return state;
    }
}
