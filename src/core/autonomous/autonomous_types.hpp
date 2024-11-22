#pragma once

#include "../common/types.hpp"

#include "../sensors/sensor_types.hpp"
#include <vector>
#include <chrono>

namespace autocore {
    namespace autonomous {

        using common::Position;
        using common::Velocity;
        using sensors::TrackedObject;
        using sensors::Feature;

        struct Waypoint {
            Position position;
            float speed;
            float heading;
            float curvature;
        };

        struct TrajectoryPoint {
            Position position;
            Velocity velocity;
            double timestamp;
        };


        struct Obstacle {
            Position position;
            float radius;
            float velocity;
            bool isDynamic;
        };

        struct SceneContext {
            std::vector<TrackedObject> nearbyObjects;
            std::vector<Feature> roadFeatures;
            Position currentPosition;
            Position targetPosition;
            float currentSpeed;
            float targetSpeed;
            bool isEmergency{false};
        };

        struct BehaviorState {
            std::string maneuver;
            std::chrono::system_clock::time_point timestamp;
            float riskLevel{0.0f};
            bool isValid{true};
        };

        struct PredictedBehavior {
            TrackedObject object;
            std::vector<TrajectoryPoint> trajectory;
            float confidence{0.0f};
            std::chrono::system_clock::time_point predictionTime;
        };


    } // namespace autonomous
} // namespace autocore 