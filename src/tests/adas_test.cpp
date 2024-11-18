#include <gtest/gtest.h>
#include "../core/adas/adas_manager.hpp"
#include "mock_components.hpp"

class ADASTest : public ::testing::Test {
protected:
    std::shared_ptr<MockSensorManager> sensorManager_;
    std::shared_ptr<MockMotorController> motorController_;
    std::shared_ptr<MockVehicleControlSystem> vcs_;
    std::unique_ptr<evlib::adas::ADASManager> adas_;

    void SetUp() override {
        sensorManager_ = std::make_shared<MockSensorManager>();
        motorController_ = std::make_shared<MockMotorController>();
        vcs_ = std::make_shared<MockVehicleControlSystem>();
        
        adas_ = std::make_unique<evlib::adas::ADASManager>(
            sensorManager_, motorController_, vcs_);
    }
};

TEST_F(ADASTest, AdaptiveCruiseControl) {
    adas_->enableAdaptiveCruiseControl(true);
    adas_->setFollowingDistance(2.0f);  // 2 seconds following distance
    
    // Simulate vehicle ahead
    sensorManager_->simulateVehicleAhead(30.0f);  // 30 meters ahead
    adas_->update();
    
    auto status = adas_->getStatus();
    EXPECT_TRUE(status.adaptiveCruiseControlActive);
    EXPECT_LT(status.currentFollowingDistance, 30.0f);
}

TEST_F(ADASTest, EmergencyBraking) {
    adas_->enableEmergencyBraking(true);
    
    // Simulate sudden obstacle
    sensorManager_->simulateObstacle(5.0f);  // 5 meters ahead
    adas_->update();
    
    EXPECT_TRUE(adas_->emergencyStop());
} 