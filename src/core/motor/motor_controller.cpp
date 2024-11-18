#include "motor_controller.hpp"
#include <cmath>
#include <algorithm>

namespace evlib {
namespace motor {

MotorController::MotorController(std::shared_ptr<bms::BatteryManager> bms)
    : motorState_(std::make_unique<MotorState>())
    , inverterController_(std::make_unique<InverterController>())
    , batteryManager_(std::move(bms))
    , regenerativeBrakingEnabled_(false)
    , regenerativeBrakingLevel_(0.0f)
    , targetSpeed_(0.0f)
    , targetTorque_(0.0f) {
}

void MotorController::setTargetSpeed(float speedRPM) {
    targetSpeed_ = std::clamp(speedRPM, 0.0f, motorState_->getMaxSpeed());
    optimizePowerDelivery();
}

void MotorController::setTargetTorque(float torqueNm) {
    targetTorque_ = std::clamp(torqueNm, 0.0f, motorState_->getMaxTorque());
    optimizePowerDelivery();
}

float MotorController::getCurrentSpeed() const {
    return motorState_->getCurrentSpeed();
}

float MotorController::getCurrentTorque() const {
    return motorState_->getCurrentTorque();
}

void MotorController::enableRegenerativeBraking(bool enable) {
    regenerativeBrakingEnabled_ = enable;
    if (enable) {
        // Check if battery can accept regenerative power
        if (batteryManager_->getStateOfCharge() > 0.95f) {
            regenerativeBrakingEnabled_ = false;
        }
    }
}

void MotorController::setRegenerativeBrakingLevel(float level) {
    regenerativeBrakingLevel_ = std::clamp(level, 0.0f, 1.0f);
    if (regenerativeBrakingEnabled_) {
        updateInverterParameters();
    }
}

float MotorController::getEfficiency() const {
    float inputPower = inverterController_->getInputPower();
    float outputPower = motorState_->getCurrentSpeed() * 
                       motorState_->getCurrentTorque() * 
                       (2.0f * M_PI / 60.0f);  // Convert RPM to rad/s
    
    return inputPower > 0 ? (outputPower / inputPower) : 0.0f;
}

void MotorController::optimizePowerDelivery() {
    float batteryVoltage = batteryManager_->getStateOfCharge() * 400.0f; // Assuming 400V system
    inverterController_->setVoltage(batteryVoltage);
    inverterController_->updatePWM(targetSpeed_, targetTorque_);
    monitorTemperature();
}

void MotorController::monitorTemperature() {
    float temp = getTemperature();
    if (temp > 80.0f) {  // Temperature threshold in Celsius
        // Implement thermal derating
        float derating = std::min(1.0f, (100.0f - temp) / 20.0f);
        targetTorque_ *= derating;
    }
}

bool MotorController::performSelfTest() {
    bool inverterOk = inverterController_->selfTest();
    bool motorOk = motorState_->checkSensors();
    bool thermalOk = getTemperature() < 90.0f;
    
    return inverterOk && motorOk && thermalOk;
}

} // namespace motor
} // namespace evlib 