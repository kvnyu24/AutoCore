#include <gtest/gtest.h>
#include "../core/motor/motor_controller.hpp"
#include "../core/bms/battery_manager.hpp"

class MotorControllerTest : public ::testing::Test {
protected:
    std::shared_ptr<evlib::bms::BatteryManager> bms_;
    std::unique_ptr<evlib::motor::MotorController> motor_;

    void SetUp() override {
        bms_ = std::make_shared<evlib::bms::BatteryManager>();
        motor_ = std::make_unique<evlib::motor::MotorController>(bms_);
    }
};

TEST_F(MotorControllerTest, InitialState) {
    EXPECT_EQ(motor_->getCurrentSpeed(), 0.0f);
    EXPECT_EQ(motor_->getCurrentTorque(), 0.0f);
}

TEST_F(MotorControllerTest, SpeedControl) {
    motor_->setTargetSpeed(1000.0f);
    EXPECT_LE(motor_->getCurrentSpeed(), 1000.0f);
}

TEST_F(MotorControllerTest, RegenerativeBraking) {
    motor_->enableRegenerativeBraking(true);
    motor_->setRegenerativeBrakingLevel(0.5f);
    EXPECT_GT(motor_->getRegenerativeEnergy(), 0.0f);
}