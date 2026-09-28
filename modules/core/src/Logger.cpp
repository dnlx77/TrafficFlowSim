#include "core/Logger.hpp"
#include <spdlog/sinks/stdout_color_sinks.h>

namespace tfs::core {

    namespace {
        std::shared_ptr<spdlog::logger> g_logger;
    }

    void Logger::init() {
        if (g_logger) return;
        g_logger = spdlog::stdout_color_mt("TFS");
        g_logger->set_pattern("[%H:%M:%S.%e] [%^%l%$] [tid:%t] %v");
        g_logger->set_level(spdlog::level::trace);
    }

    spdlog::logger& Logger::get() {
        if (!g_logger) init();
        return *g_logger;
    }
}
