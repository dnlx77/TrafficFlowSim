#pragma once
#include <memory>
#include <spdlog/spdlog.h>

namespace tfs::core {
    class Logger {
    public:
        static void init();

        template <typename... Args>
        static void info(spdlog::format_string_t<Args...> fmt, Args&&... args) {
            get().info(fmt, std::forward<Args>(args)...);
        }

        template <typename... Args>
        static void warn(spdlog::format_string_t<Args...> fmt, Args&&... args) {
            get().warn(fmt, std::forward<Args>(args)...);
        }

        template <typename... Args>
        static void error(spdlog::format_string_t<Args...> fmt, Args&&... args) {
            get().error(fmt, std::forward<Args>(args)...);
        }

        template <typename... Args>
        static void debug(spdlog::format_string_t<Args...> fmt, Args&&... args) {
            get().debug(fmt, std::forward<Args>(args)...);
        }

    private:
        static spdlog::logger& get();
    };
}
