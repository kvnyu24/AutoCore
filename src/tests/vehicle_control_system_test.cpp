#include <gtest/gtest.h>
#include "../core/vcs/vehicle_control_system.hpp"
#include "mock_components.hpp"

class VehicleControlSystemTest : public ::testing::Test {
protected:
    std::shared_ptr<MockBatteryManager> bms_;
    std::shared_ptr<MockMotorController> motorController_;
    std::shared_ptr<MockSensorManager> sensorManager_;
    std::shared_ptr<MockTelematicsManager> telematicsManager_;
    std::shared_ptr<MockDiagnosticManager> diagnosticManager_;
    std::unique_ptr<autocore::vcs::VehicleControlSystem> vcs_;

    void SetUp() override {
        // Initialize mock components
        bms_ = std::make_shared<MockBatteryManager>();
        motorController_ = std::make_shared<MockMotorController>();
        sensorManager_ = std::make_shared<MockSensorManager>();
        telematicsManager_ = std::make_shared<MockTelematicsManager>();
        diagnosticManager_ = std::make_shared<MockDiagnosticManager>();
        
        // Create VCS with mock components
        vcs_ = std::make_unique<autocore::vcs::VehicleControlSystem>(
            bms_, motorController_, sensorManager_, 
            telematicsManager_, diagnosticManager_);
    }
};

TEST_F(VehicleControlSystemTest, NormalOperation) {
    vcs_->setDrivingMode(DrivingMode::NORMAL);
    vcs_->setTargetSpeed(60.0f); // 60 km/h
    vcs_->executeControlLoop();
    
    auto state = vcs_->getCurrentState();
    EXPECT_NEAR(state.speed, 60.0f, 5.0f);
}

TEST_F(VehicleControlSystemTest, EconomyMode) {
    vcs_->setEconomyMode(true);
    vcs_->setTargetSpeed(100.0f);
    vcs_->executeControlLoop();
    
    // Verify reduced power consumption in economy mode
    auto state = vcs_->getCurrentState();
    EXPECT_LT(state.powerConsumption, 
              state.speed * 0.015f); // Should be less than normal power
}