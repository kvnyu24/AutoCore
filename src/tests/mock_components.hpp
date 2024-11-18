#pragma once

#include <gmock/gmock.h>
#include "../core/sensors/fusion_engine.hpp"
#include "../core/sensors/slam_engine.hpp"
#include "../core/bms/battery_manager.hpp"
#include "../core/motor/motor_controller.hpp"
#include "../core/sensors/sensor_manager.hpp"
#include "../core/telematics/telematics_manager.hpp"
#include "../core/diagnostics/diagnostic_manager.hpp"
#include "../core/diagnostics/component_health.hpp"

namespace autocore {
namespace tests {

class MockFusionEngine : public sensors::FusionEngine {
public:
    MOCK_METHOD(void, updateFusion, (), (override));
    MOCK_METHOD(void, setFusionParameters, (const sensors::FusionParameters& params), (override));
    MOCK_METHOD(void, simulateObstacle, (const sensors::Position& position));
};

class MockSLAMEngine : public sensors::SLAMEngine {
public:
    MOCK_METHOD(void, updateMap, (), (override));
    MOCK_METHOD(std::vector<sensors::Feature>, detectFeatures, (const sensors::SensorData& data), (override));
    MOCK_METHOD(void, trackFeatures, (const std::vector<sensors::Feature>& features), (override));
};

class MockBatteryManager : public bms::BatteryManager {
public:
    MOCK_METHOD(float, getStateOfCharge, (), (const, override));
    MOCK_METHOD(bool, detectFaults, (), (override));
    MOCK_METHOD(void, simulateFault, (FaultType fault));
};

class MockMotorController : public motor::MotorController {
public:
    MOCK_METHOD(void, setTargetSpeed, (float speed), (override));
    MOCK_METHOD(float, getCurrentSpeed, (), (const, override));
    MOCK_METHOD(void, simulateWear, (float wearLevel));
};

class MockSensorManager : public sensors::SensorManager {
public:
    MOCK_METHOD(void, updateSensorData, (), (override));
    MOCK_METHOD(bool, registerSensor, (const std::string& name, SensorType type), (override));
    MOCK_METHOD(sensors::SensorData, getFusedData, (), (const, override));
    MOCK_METHOD(void, simulateObstacle, (float distance));
};

class MockTelematicsManager : public telematics::TelematicsManager {
public:
    MOCK_METHOD(bool, sendTelemetryData, (), (override));
    MOCK_METHOD(bool, connectToCloud, (const std::string& apiKey), (override));
};

class MockDiagnosticManager : public diagnostics::DiagnosticManager {
public:
    MOCK_METHOD(diagnostics::SystemHealth, getSystemHealth, (), (const, override));
    MOCK_METHOD(void, updateDiagnostics, (), (override));
    MOCK_METHOD(std::vector<diagnostics::Fault>, getActiveFaults, (), (const, override));
};

} // namespace tests
} // namespace autocore