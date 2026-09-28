#pragma once
#include <vector>
#include <cstddef>
#include "core/Vec2.hpp"

namespace tfs::core {
    class CubicBezier {
    public:
        CubicBezier() = default;
        CubicBezier(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3);

        static CubicBezier makeStraight(Vec2 start, Vec2 end);

        void rebuildLUT(std::size_t samples = 128);

        [[nodiscard]] Vec2 evaluate(float t) const noexcept;
        [[nodiscard]] Vec2 tangent(float t) const noexcept;
        [[nodiscard]] Vec2 normalRight(float t) const noexcept;
        [[nodiscard]] float totalLength() const noexcept;
        [[nodiscard]] float mapDistanceToT(float s) const noexcept;
        [[nodiscard]] Vec2 evaluateAtDistance(float s) const noexcept;
        [[nodiscard]] Vec2 tangentAtDistance(float s) const noexcept;
        [[nodiscard]] CubicBezier computeOffsetCurve(float lateral_offset) const;
        [[nodiscard]] float projectPoint(Vec2 world_pt, Vec2& out_closest_pt, float& out_distance_s) const noexcept;

        [[nodiscard]] Vec2 p0() const noexcept { return p0_; }
        [[nodiscard]] Vec2 p1() const noexcept { return p1_; }
        [[nodiscard]] Vec2 p2() const noexcept { return p2_; }
        [[nodiscard]] Vec2 p3() const noexcept { return p3_; }

    private:
        struct ArcSample {
            float t;
            float s;
            Vec2 pos;
            Vec2 tangent;
        };

        Vec2 p0_{}, p1_{}, p2_{}, p3_{};
        std::vector<ArcSample> lut_;
        float total_length_{0.0f};
    };
}
