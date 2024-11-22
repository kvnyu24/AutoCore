#pragma once

#include "../../sensors/sensor_types.hpp"
#include "../../common/types.hpp"

namespace autocore {
namespace autonomous {

    struct SceneContext {
        std::vector<sensors::TrackedObject> nearbyObjects;
        std::vector<sensors::Feature> roadFeatures;
        common::Position currentPosition;
        common::Position targetPosition;
        float currentSpeed;
        float targetSpeed;
        bool isEmergency{false};
    };

} // namespace autonomous
} // namespace autocore 