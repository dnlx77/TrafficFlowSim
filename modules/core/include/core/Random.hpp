#pragma once
#include <cstdint>
#include <random>

namespace tfs::core {
    class RandomEngine {
    public:
        void seed(std::uint64_t s);
        float uniform(float min, float max);
        float normal(float mean, float stddev);
        bool bernoulli(float p);

    private:
        std::mt19937_64 engine_{std::random_device{}()};
    };

    struct OrnsteinUhlenbeckProcess {
        float state{0.0f};
        float tau{4.0f};
        float sigma{0.40f};

        float step(float dt, RandomEngine& rng) noexcept;
    };
}
