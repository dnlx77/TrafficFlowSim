#include "ui/Camera2D.hpp"
#include <algorithm>
#include <cmath>

namespace tfs::ui {

    void Camera2D::handleEvent(const sf::Event& event, sf::Vector2u target_size) {
        if (const auto* pressed = event.getIf<sf::Event::MouseButtonPressed>()) {
            if (pressed->button == sf::Mouse::Button::Middle || pressed->button == sf::Mouse::Button::Right) {
                dragging_ = true;
                last_mouse_pos_ = pressed->position;
            }
        } else if (const auto* released = event.getIf<sf::Event::MouseButtonReleased>()) {
            if (released->button == sf::Mouse::Button::Middle || released->button == sf::Mouse::Button::Right) {
                dragging_ = false;
            }
        } else if (const auto* moved = event.getIf<sf::Event::MouseMoved>()) {
            if (dragging_) {
                const sf::Vector2i delta = moved->position - last_mouse_pos_;
                center_.x -= static_cast<float>(delta.x) / zoom_;
                center_.y -= static_cast<float>(delta.y) / zoom_;
                last_mouse_pos_ = moved->position;
            }
        } else if (const auto* scrolled = event.getIf<sf::Event::MouseWheelScrolled>()) {
            const core::Vec2 before = screenToWorld(sf::Vector2i(scrolled->position), target_size);
            const float factor = std::pow(1.1f, scrolled->delta);
            zoom_ = std::clamp(zoom_ * factor, kMinZoom, kMaxZoom);
            const core::Vec2 after = screenToWorld(sf::Vector2i(scrolled->position), target_size);
            center_.x += before.x - after.x;
            center_.y += before.y - after.y;
        }
    }

    sf::View Camera2D::buildSfmlView(sf::Vector2u target_size) const {
        return sf::View(sf::Vector2f{center_.x, center_.y},
                         sf::Vector2f{static_cast<float>(target_size.x) / zoom_, static_cast<float>(target_size.y) / zoom_});
    }

    core::Vec2 Camera2D::screenToWorld(sf::Vector2i screen_pos, sf::Vector2u target_size) const {
        const float half_w = static_cast<float>(target_size.x) * 0.5f;
        const float half_h = static_cast<float>(target_size.y) * 0.5f;
        return {center_.x + (static_cast<float>(screen_pos.x) - half_w) / zoom_,
                center_.y + (static_cast<float>(screen_pos.y) - half_h) / zoom_};
    }

    void Camera2D::frameBounds(core::Vec2 min, core::Vec2 max, sf::Vector2u target_size) noexcept {
        center_ = {(min.x + max.x) * 0.5f, (min.y + max.y) * 0.5f};

        const float width = std::max(max.x - min.x, 1.0f);
        const float height = std::max(max.y - min.y, 1.0f);
        const float margin = 1.2f;

        const float zoom_x = static_cast<float>(target_size.x) / (width * margin);
        const float zoom_y = static_cast<float>(target_size.y) / (height * margin);
        zoom_ = std::clamp(std::min(zoom_x, zoom_y), kMinZoom, kMaxZoom);
    }

    sf::Vector2f Camera2D::worldToScreen(core::Vec2 world_pos, sf::Vector2u target_size) const {
        const float half_w = static_cast<float>(target_size.x) * 0.5f;
        const float half_h = static_cast<float>(target_size.y) * 0.5f;
        return {half_w + (world_pos.x - center_.x) * zoom_, half_h + (world_pos.y - center_.y) * zoom_};
    }
}
