#pragma once

#include <gmock/gmock.h>
#include "../core/sensors/fusion_engine.hpp"
#include "../core/sensors/slam_engine.hpp"
#include "../core/autonomous/behavior/behavior_planner.hpp"

namespace autocore {
namespace test {

class MockFusionEngine : public sensors::FusionEngine {
public:
    MOCK_METHOD(void, updateFusion, (), (override));
    MOCK_METHOD(void, setFusionParameters, (const FusionParameters& params), (override));
    
    void simulateObstacle(const Position& pos) {
        // Add test implementation
    }
};

class MockSLAMEngine : public sensors::SLAMEngine {
public:
    MOCK_METHOD(void, updateMap, (), (override));
    MOCK_METHOD(std::vector<Feature>, detectFeatures, (const SensorData& data), (override));
    MOCK_METHOD(void, trackFeatures, (const std::vector<Feature>& features), (override));
};

// Add other mock classes as needed

} // namespace test
} // namespace autocore