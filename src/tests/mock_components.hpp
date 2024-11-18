#pragma once

#include <gmock/gmock.h>
#include "../core/sensors/fusion_engine.hpp"
#include "../core/sensors/slam_engine.hpp"

namespace autocore {
namespace tests {

class MockFusionEngine : public sensors::FusionEngine {
public:
    MOCK_METHOD(void, updateFusion, (), (override));
    MOCK_METHOD(void, setFusionParameters, (const sensors::FusionParameters& params), (override));
};

class MockSLAMEngine : public sensors::SLAMEngine {
public:
    MOCK_METHOD(void, updateMap, (), (override));
    MOCK_METHOD(std::vector<sensors::Feature>, detectFeatures, (const sensors::SensorData& data), (override));
    MOCK_METHOD(void, trackFeatures, (const std::vector<sensors::Feature>& features), (override));
};

} // namespace tests
} // namespace autocore