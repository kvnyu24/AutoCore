#pragma once

#include <memory>
#include "../sensors/sensor_types.hpp"
#include "../sensors/position.hpp"
#include "../sensors/fusion_engine.hpp"

namespace autocore {
namespace localization {

class LocalizationSystem {
public:
    LocalizationSystem(std::shared_ptr<sensors::FusionEngine> fusionEngine);
    ~LocalizationSystem() = default;

    // Core localization
    void update();
    sensors::Position getCurrentPosition() const;
    float getConfidence() const;

private:
    std::shared_ptr<sensors::FusionEngine> fusionEngine_;
    sensors::Position currentPosition_;
    float confidence_{0.0f};
};

} // namespace localization
} // namespace autocore 