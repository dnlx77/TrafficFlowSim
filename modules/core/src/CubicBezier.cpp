#include "core/CubicBezier.hpp"
#include <algorithm>
#include <limits>

namespace tfs::core {

    CubicBezier::CubicBezier(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3)
        : p0_(p0), p1_(p1), p2_(p2), p3_(p3) {
        rebuildLUT();
    }

    CubicBezier CubicBezier::makeStraight(Vec2 start, Vec2 end) {
        const Vec2 delta = end - start;
        const Vec2 p1 = start + delta * (1.0f / 3.0f);
        const Vec2 p2 = start + delta * (2.0f / 3.0f);
        return CubicBezier(start, p1, p2, end);
    }

    Vec2 CubicBezier::evaluate(float t) const noexcept {
        const float u = 1.0f - t;
        const float b0 = u * u * u;
        const float b1 = 3.0f * u * u * t;
        const float b2 = 3.0f * u * t * t;
        const float b3 = t * t * t;
        return p0_ * b0 + p1_ * b1 + p2_ * b2 + p3_ * b3;
    }

    Vec2 CubicBezier::tangent(float t) const noexcept {
        const float u = 1.0f - t;
        const Vec2 d0 = (p1_ - p0_) * (3.0f * u * u);
        const Vec2 d1 = (p2_ - p1_) * (6.0f * u * t);
        const Vec2 d2 = (p3_ - p2_) * (3.0f * t * t);
        return (d0 + d1 + d2).normalized();
    }

    Vec2 CubicBezier::normalRight(float t) const noexcept {
        return tangent(t).perpRight();
    }

    void CubicBezier::rebuildLUT(std::size_t samples) {
        lut_.clear();
        lut_.reserve(samples + 1);

        float accumulated_s = 0.0f;
        Vec2 prev_pos = evaluate(0.0f);
        lut_.push_back({0.0f, 0.0f, prev_pos, tangent(0.0f)});

        for (std::size_t i = 1; i <= samples; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(samples);
            const Vec2 pos = evaluate(t);
            accumulated_s += pos.distanceTo(prev_pos);
            lut_.push_back({t, accumulated_s, pos, tangent(t)});
            prev_pos = pos;
        }

        total_length_ = accumulated_s;
    }

    float CubicBezier::totalLength() const noexcept {
        return total_length_;
    }

    float CubicBezier::mapDistanceToT(float s) const noexcept {
        if (lut_.empty()) return 0.0f;
        if (s <= 0.0f) return 0.0f;
        if (s >= total_length_) return 1.0f;

        auto it = std::lower_bound(lut_.begin(), lut_.end(), s,
            [](const ArcSample& sample, float value) { return sample.s < value; });

        if (it == lut_.begin()) return it->t;

        const ArcSample& hi = *it;
        const ArcSample& lo = *(it - 1);
        const float span = hi.s - lo.s;
        if (span < 1e-8f) return lo.t;
        const float frac = (s - lo.s) / span;
        return lo.t + frac * (hi.t - lo.t);
    }

    Vec2 CubicBezier::evaluateAtDistance(float s) const noexcept {
        return evaluate(mapDistanceToT(s));
    }

    Vec2 CubicBezier::tangentAtDistance(float s) const noexcept {
        return tangent(mapDistanceToT(s));
    }

    CubicBezier CubicBezier::computeOffsetCurve(float lateral_offset) const {
        const Vec2 new_p0 = p0_ + normalRight(0.0f) * lateral_offset;
        const Vec2 new_p3 = p3_ + normalRight(1.0f) * lateral_offset;
        const Vec2 new_p1 = p1_ + normalRight(1.0f / 3.0f) * lateral_offset;
        const Vec2 new_p2 = p2_ + normalRight(2.0f / 3.0f) * lateral_offset;
        return CubicBezier(new_p0, new_p1, new_p2, new_p3);
    }

    float CubicBezier::projectPoint(Vec2 world_pt, Vec2& out_closest_pt, float& out_distance_s) const noexcept {
        if (lut_.empty()) {
            out_closest_pt = p0_;
            out_distance_s = 0.0f;
            return world_pt.distanceTo(p0_);
        }

        float best_dist_sq = std::numeric_limits<float>::max();
        std::size_t best_idx = 0;

        for (std::size_t i = 0; i < lut_.size(); ++i) {
            const float d = (lut_[i].pos - world_pt).lengthSq();
            if (d < best_dist_sq) {
                best_dist_sq = d;
                best_idx = i;
            }
        }

        Vec2 best_pt = lut_[best_idx].pos;
        float best_s = lut_[best_idx].s;
        float best_dist = std::sqrt(best_dist_sq);

        const auto refine_segment = [&](std::size_t a, std::size_t b) {
            const Vec2& pa = lut_[a].pos;
            const Vec2& pb = lut_[b].pos;
            const Vec2 seg = pb - pa;
            const float seg_len_sq = seg.lengthSq();
            if (seg_len_sq < 1e-12f) return;
            float u = (world_pt - pa).dot(seg) / seg_len_sq;
            u = std::clamp(u, 0.0f, 1.0f);
            const Vec2 proj_pt = pa + seg * u;
            const float dist = world_pt.distanceTo(proj_pt);
            if (dist < best_dist) {
                best_dist = dist;
                best_pt = proj_pt;
                best_s = lut_[a].s + u * (lut_[b].s - lut_[a].s);
            }
        };

        if (best_idx > 0) refine_segment(best_idx - 1, best_idx);
        if (best_idx + 1 < lut_.size()) refine_segment(best_idx, best_idx + 1);

        out_closest_pt = best_pt;
        out_distance_s = best_s;
        return best_dist;
    }
}
