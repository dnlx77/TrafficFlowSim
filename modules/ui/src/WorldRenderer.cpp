#include "ui/WorldRenderer.hpp"
#include <SFML/Graphics/VertexArray.hpp>
#include <algorithm>
#include <cmath>

namespace tfs::ui {

    namespace {
        sf::Color speedToColor(float speed_kmh) {
            const float t = std::clamp(speed_kmh / 90.0f, 0.0f, 1.0f);
            if (t < 1.0f / 3.0f) {
                const float u = t * 3.0f;
                return sf::Color(255, static_cast<std::uint8_t>(120.0f * u), 0);
            }
            if (t < 2.0f / 3.0f) {
                const float u = (t - 1.0f / 3.0f) * 3.0f;
                return sf::Color(static_cast<std::uint8_t>(255.0f * (1.0f - u)) + 0,
                                  static_cast<std::uint8_t>(120.0f + 135.0f * u), 0);
            }
            const float u = (t - 2.0f / 3.0f) * 3.0f;
            return sf::Color(static_cast<std::uint8_t>(255.0f * (1.0f - u)),
                              255, 0);
        }

        sf::Color classToColor(sim::VehicleClass c) {
            switch (c) {
                case sim::VehicleClass::HeavyTruck: return sf::Color(150, 100, 60);
                case sim::VehicleClass::AggressiveDriver: return sf::Color(200, 40, 180);
                default: return sf::Color(60, 130, 220);
            }
        }

        sf::Color accelToColor(float accel) {
            if (accel < 0.0f) {
                const float u = std::clamp(-accel / 8.5f, 0.0f, 1.0f);
                return sf::Color(255, static_cast<std::uint8_t>(255.0f * (1.0f - u)), 0);
            }
            const float u = std::clamp(accel / 1.8f, 0.0f, 1.0f);
            return sf::Color(static_cast<std::uint8_t>(255.0f * (1.0f - u)), 255, 0);
        }

        sf::Color lightStateToColor(network::LightState state) {
            switch (state) {
                case network::LightState::Green: return sf::Color(40, 200, 60);
                case network::LightState::Yellow: return sf::Color(230, 200, 30);
                default: return sf::Color(210, 40, 40);
            }
        }

        void appendRibbon(sf::VertexArray& va, const core::CubicBezier& curve, float half_width, sf::Color color,
                           const Camera2D& camera, sf::Vector2u target_size) {
            const float length = curve.totalLength();
            if (length < 1e-4f) return;
            const int steps = std::clamp(static_cast<int>(length / 2.0f), 1, 200);

            std::vector<sf::Vertex> strip;
            strip.reserve(static_cast<std::size_t>(steps + 1) * 2);
            for (int i = 0; i <= steps; ++i) {
                const float s = length * static_cast<float>(i) / static_cast<float>(steps);
                const core::Vec2 pos = curve.evaluateAtDistance(s);
                const core::Vec2 tangent = curve.tangentAtDistance(s);
                const core::Vec2 normal = tangent.perpRight();
                const core::Vec2 left = pos - normal * half_width;
                const core::Vec2 right = pos + normal * half_width;
                strip.push_back({camera.worldToScreen(left, target_size), color, {}});
                strip.push_back({camera.worldToScreen(right, target_size), color, {}});
            }
            for (std::size_t i = 0; i + 3 < strip.size(); i += 2) {
                va.append(strip[i]);
                va.append(strip[i + 1]);
                va.append(strip[i + 2]);
                va.append(strip[i + 1]);
                va.append(strip[i + 3]);
                va.append(strip[i + 2]);
            }
        }

        void appendQuad(sf::VertexArray& va, core::Vec2 a, core::Vec2 b, core::Vec2 c, core::Vec2 d, sf::Color color,
                         const Camera2D& camera, sf::Vector2u target_size) {
            const sf::Vertex va_ = {camera.worldToScreen(a, target_size), color, {}};
            const sf::Vertex vb_ = {camera.worldToScreen(b, target_size), color, {}};
            const sf::Vertex vc_ = {camera.worldToScreen(c, target_size), color, {}};
            const sf::Vertex vd_ = {camera.worldToScreen(d, target_size), color, {}};
            va.append(va_);
            va.append(vb_);
            va.append(vc_);
            va.append(va_);
            va.append(vc_);
            va.append(vd_);
        }

        void drawGrid(sf::RenderTarget& target, const Camera2D& camera, sf::Vector2u target_size) {
            const core::Vec2 top_left = camera.screenToWorld({0, 0}, target_size);
            const core::Vec2 bottom_right = camera.screenToWorld(
                {static_cast<int>(target_size.x), static_cast<int>(target_size.y)}, target_size);

            sf::VertexArray minor(sf::PrimitiveType::Lines);
            sf::VertexArray major(sf::PrimitiveType::Lines);

            const float step_minor = 10.0f;
            const float step_major = 50.0f;
            const sf::Color minor_color(60, 60, 65);
            const sf::Color major_color(90, 90, 98);

            const float start_x = std::floor(top_left.x / step_minor) * step_minor;
            for (float x = start_x; x <= bottom_right.x; x += step_minor) {
                const bool is_major = std::fmod(std::abs(x), step_major) < 0.01f;
                sf::VertexArray& va = is_major ? major : minor;
                const sf::Color c = is_major ? major_color : minor_color;
                va.append({camera.worldToScreen({x, top_left.y}, target_size), c, {}});
                va.append({camera.worldToScreen({x, bottom_right.y}, target_size), c, {}});
            }
            const float start_y = std::floor(top_left.y / step_minor) * step_minor;
            for (float y = start_y; y <= bottom_right.y; y += step_minor) {
                const bool is_major = std::fmod(std::abs(y), step_major) < 0.01f;
                sf::VertexArray& va = is_major ? major : minor;
                const sf::Color c = is_major ? major_color : minor_color;
                va.append({camera.worldToScreen({top_left.x, y}, target_size), c, {}});
                va.append({camera.worldToScreen({bottom_right.x, y}, target_size), c, {}});
            }

            target.draw(minor);
            target.draw(major);
        }

        void drawLanes(sf::RenderTarget& target, const network::RoadNetwork& network, const Camera2D& camera,
                        sf::Vector2u target_size) {
            sf::VertexArray asphalt(sf::PrimitiveType::Triangles);
            sf::VertexArray markings(sf::PrimitiveType::Lines);

            const sf::Color asphalt_color(45, 45, 50);
            const sf::Color margin_color(230, 230, 230);
            const sf::Color dash_color(200, 200, 60);

            for (const auto& [id, road] : network.roads()) {
                const auto render_lane_group = [&](const std::vector<network::Lane>& lanes) {
                    for (std::size_t i = 0; i < lanes.size(); ++i) {
                        appendRibbon(asphalt, lanes[i].center_curve, lanes[i].width * 0.5f, asphalt_color, camera, target_size);

                        const float length = lanes[i].length();
                        const int dash_count = std::max(1, static_cast<int>(length / 6.0f));
                        for (int d = 0; d < dash_count; ++d) {
                            const float s0 = length * static_cast<float>(d) / static_cast<float>(dash_count);
                            const float s1 = std::min(length, s0 + 3.0f);
                            const core::Vec2 p0 = lanes[i].center_curve.evaluateAtDistance(s0);
                            const core::Vec2 p1 = lanes[i].center_curve.evaluateAtDistance(s1);
                            const bool is_edge = (i == lanes.size() - 1);
                            const sf::Color c = is_edge ? margin_color : dash_color;
                            markings.append({camera.worldToScreen(p0, target_size), c, {}});
                            markings.append({camera.worldToScreen(p1, target_size), c, {}});
                        }
                    }
                };
                render_lane_group(road.forward_lanes);
                render_lane_group(road.backward_lanes);
            }

            target.draw(asphalt);
            target.draw(markings);
        }

        void drawNodesAndLights(sf::RenderTarget& target, const network::RoadNetwork& network,
                                 const sim::SimulationSnapshot& snapshot, const Camera2D& camera,
                                 sf::Vector2u target_size) {
            sf::VertexArray outlines(sf::PrimitiveType::Lines);
            for (const auto& [id, node] : network.nodes()) {
                constexpr int kSegments = 24;
                for (int i = 0; i < kSegments; ++i) {
                    const float a0 = static_cast<float>(i) / kSegments * 6.2831853f;
                    const float a1 = static_cast<float>(i + 1) / kSegments * 6.2831853f;
                    const core::Vec2 p0 = node.position + core::Vec2{std::cos(a0), std::sin(a0)} * node.radius;
                    const core::Vec2 p1 = node.position + core::Vec2{std::cos(a1), std::sin(a1)} * node.radius;
                    outlines.append({camera.worldToScreen(p0, target_size), sf::Color(100, 100, 110), {}});
                    outlines.append({camera.worldToScreen(p1, target_size), sf::Color(100, 100, 110), {}});
                }
            }
            target.draw(outlines);

            sf::VertexArray lights(sf::PrimitiveType::Triangles);
            for (const auto& tl : snapshot.traffic_lights) {
                const core::Vec2 forward = tl.stop_line_dir;
                const core::Vec2 side = forward.perpRight();
                const float half = 1.0f;
                appendQuad(lights, tl.stop_line_pos - side * half, tl.stop_line_pos + side * half,
                           tl.stop_line_pos + side * half + forward * 0.4f, tl.stop_line_pos - side * half + forward * 0.4f,
                           lightStateToColor(tl.state), camera, target_size);
            }
            target.draw(lights);
        }

        void drawVehicles(sf::RenderTarget& target, const sim::SimulationSnapshot& snapshot, const Camera2D& camera,
                           sf::Vector2u target_size, VehicleColorMode color_mode, float blink_phase) {
            sf::VertexArray bodies(sf::PrimitiveType::Triangles);
            sf::VertexArray accents(sf::PrimitiveType::Triangles);

            const bool blink_on = std::fmod(blink_phase, 0.6f) < 0.3f;

            for (const auto& v : snapshot.vehicles) {
                const core::Vec2 forward{std::cos(v.heading), std::sin(v.heading)};
                const core::Vec2 side = forward.perpRight();

                const core::Vec2 front_center = v.position;
                const core::Vec2 rear_center = v.position - forward * v.length;
                const core::Vec2 fl = front_center - side * (v.width * 0.5f);
                const core::Vec2 fr = front_center + side * (v.width * 0.5f);
                const core::Vec2 rl = rear_center - side * (v.width * 0.5f);
                const core::Vec2 rr = rear_center + side * (v.width * 0.5f);

                sf::Color body_color;
                switch (color_mode) {
                    case VehicleColorMode::ByClass: body_color = classToColor(v.vclass); break;
                    case VehicleColorMode::ByAcceleration: body_color = accelToColor(v.acceleration); break;
                    default: body_color = speedToColor(v.speed * 3.6f); break;
                }

                appendQuad(bodies, fl, fr, rr, rl, body_color, camera, target_size);

                if (v.braking) {
                    const float bw = v.width * 0.2f;
                    appendQuad(accents, rl, rl + side * bw, rl + side * bw + forward * 0.3f, rl + forward * 0.3f,
                               sf::Color(255, 20, 20), camera, target_size);
                    appendQuad(accents, rr - side * bw, rr, rr + forward * 0.3f, rr - side * bw + forward * 0.3f,
                               sf::Color(255, 20, 20), camera, target_size);
                }
                if (v.blinker != 0 && blink_on) {
                    const core::Vec2 corner = (v.blinker < 0) ? fl : fr;
                    const float bw = v.width * 0.2f;
                    const core::Vec2 inward = (v.blinker < 0) ? side : (side * -1.0f);
                    appendQuad(accents, corner, corner + inward * bw, corner + inward * bw - forward * 0.3f,
                               corner - forward * 0.3f, sf::Color(255, 165, 0), camera, target_size);
                }
            }

            target.draw(bodies);
            target.draw(accents);
        }
    }

    void WorldRenderer::render(sf::RenderTarget& target, const network::RoadNetwork& network,
                                const sim::SimulationSnapshot& snapshot, const Camera2D& camera,
                                VehicleColorMode color_mode, float blink_phase) {
        const sf::Vector2u target_size = target.getSize();
        target.setView(target.getDefaultView());

        drawGrid(target, camera, target_size);
        drawLanes(target, network, camera, target_size);
        drawNodesAndLights(target, network, snapshot, camera, target_size);
        drawVehicles(target, snapshot, camera, target_size, color_mode, blink_phase);
    }
}
