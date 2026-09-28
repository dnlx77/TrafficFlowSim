#pragma once
#include <SFML/Graphics/View.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Event.hpp>
#include "core/Vec2.hpp"

namespace tfs::ui {
    class Camera2D {
    public:
        void handleEvent(const sf::Event& event, sf::Vector2u target_size);
        [[nodiscard]] sf::View buildSfmlView(sf::Vector2u target_size) const;
        [[nodiscard]] core::Vec2 screenToWorld(sf::Vector2i screen_pos, sf::Vector2u target_size) const;
        [[nodiscard]] sf::Vector2f worldToScreen(core::Vec2 world_pos, sf::Vector2u target_size) const;

        [[nodiscard]] float zoom() const noexcept { return zoom_; }
        [[nodiscard]] core::Vec2 center() const noexcept { return center_; }

        void frameBounds(core::Vec2 min, core::Vec2 max, sf::Vector2u target_size) noexcept;

    private:
        core::Vec2 center_{0.0f, 0.0f};
        float zoom_{10.0f};
        bool dragging_{false};
        sf::Vector2i last_mouse_pos_{};

        static constexpr float kMinZoom = 0.5f;
        static constexpr float kMaxZoom = 80.0f;
    };
}
