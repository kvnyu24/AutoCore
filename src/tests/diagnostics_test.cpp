#include <gtest/gtest.h>
#include "../core/diagnostics/diagnostic_manager.hpp"
#include "mock_components.hpp"

class DiagnosticsTest : public ::testing::Test {
protected:
    std::shared_ptr<MockBatteryManager> bms_;
    std::shared_ptr<MockMotorController> motorController_;
    std::shared_ptr<MockSensorManager> sensorManager_;
    std::unique_ptr<autocore::diagnostics::DiagnosticManager> diagnosticManager_;

    void SetUp() override {
        bms_ = std::make_shared<MockBatteryManager>();
        motorController_ = std::make_shared<MockMotorController>();
        sensorManager_ = std::make_shared<MockSensorManager>();
        
        diagnosticManager_ = std::make_unique<autocore::diagnostics::DiagnosticManager>(
            bms_, motorController_, sensorManager_);
    }
};

TEST_F(DiagnosticsTest, FaultDetection) {
    // Simulate a fault condition
    bms_->simulateFault(FaultType::VOLTAGE_WARNING);
    
    diagnosticManager_->updateDiagnostics();
    auto faults = diagnosticManager_->getActiveFaults();
    
    EXPECT_FALSE(faults.empty());
    EXPECT_EQ(faults[0].type, FaultType::VOLTAGE_WARNING);
}

TEST_F(DiagnosticsTest, PredictiveMaintenance) {
    // Simulate component wear
    motorController_->simulateWear(0.5);
    
    auto schedule = diagnosticManager_->getPredictiveMaintenanceSchedule();
    EXPECT_TRUE(schedule.hasMaintenanceItems());
}

TEST_F(DiagnosticsTest, CANBusMonitoring) {
    CANMessage testMessage{0x100, {0x01, 0x02, 0x03, 0x04}};
    diagnosticManager_->parseCANMessage(testMessage);
    
    auto messages = diagnosticManager_->getCANMessages();
    EXPECT_FALSE(messages.empty());
} 