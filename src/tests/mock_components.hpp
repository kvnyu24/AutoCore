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
#include "../core/diagnostics/fault_detector.hpp"
#include "../core/common/types.hpp"
namespace autocore {
namespace tests {

// Base interface for FusionEngine
class IFusionEngine {
public:
    virtual ~IFusionEngine() = default;
    virtual void updateFusion() = 0;
    virtual void setFusionParameters(const sensors::FusionParameters& params) = 0;
};

// Base interface for SLAMEngine
class ISLAMEngine {
public:
    virtual ~ISLAMEngine() = default;
    virtual void updateMap() = 0;
    virtual std::vector<sensors::Feature> detectFeatures(const sensors::SensorData& data) = 0;
    virtual void trackFeatures(const std::vector<sensors::Feature>& features) = 0;
};

// Mock implementations
class MockFusionEngine : public IFusionEngine {
public:
    MOCK_METHOD(void, updateFusion, (), (override));
    MOCK_METHOD(void, setFusionParameters, (const sensors::FusionParameters& params), (override));
};

class MockSLAMEngine : public ISLAMEngine {
public:
    MOCK_METHOD(void, updateMap, (), (override));
    MOCK_METHOD(std::vector<sensors::Feature>, detectFeatures, (const sensors::SensorData& data), (override));
    MOCK_METHOD(void, trackFeatures, (const std::vector<sensors::Feature>& features), (override));
};

class MockBatteryManager : public virtual bms::BatteryManager {
public:
    virtual ~MockBatteryManager() = default;
    MOCK_METHOD(float, getStateOfCharge, (), (const));
    MOCK_METHOD(bool, detectFaults, ());
    MOCK_METHOD(void, simulateFault, (diagnostics::FaultType fault));
};

class MockMotorController : public motor::MotorController {
public:
    MOCK_METHOD(void, setTargetSpeed, (float speed), (override));
    MOCK_METHOD(float, getCurrentSpeed, (), (const, override));
    MOCK_METHOD(void, simulateWear, (float wearLevel));
};

class MockSensorManager : public sensors::SensorManager {
public:
    MOCK_METHOD(void, updateSensorData, ());
    MOCK_METHOD(bool, registerSensor, (const std::string& name, sensors::SensorType type));
    MOCK_METHOD(sensors::SensorData, getFusedData, (), (const));
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

class MockFaultDetector {
public:
    MOCK_METHOD(void, simulateFault, (diagnostics::FaultType fault));
    MOCK_METHOD(void, clearFault, (diagnostics::FaultType fault));
};

} // namespace tests
} // namespace autocore