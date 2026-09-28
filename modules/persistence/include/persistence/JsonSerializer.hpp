#pragma once
#include <filesystem>
#include <string>
#include "sim/SimulationWorld.hpp"

namespace tfs::persistence {
    enum class SaveMode {
        InfrastructureOnly,
        FullState
    };

    class JsonSerializer {
    public:
        static bool saveToFile(const std::filesystem::path& filepath, const sim::SimulationWorld& world,
                                SaveMode mode, std::string& out_error);
        static bool loadFromFile(const std::filesystem::path& filepath, sim::SimulationWorld& world,
                                  std::string& out_error);
    };
}
