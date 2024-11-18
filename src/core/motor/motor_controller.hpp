#pragma once

#include <memory>
#include <vector>
#include "motor_state.hpp"
#include "inverter_controller.hpp"
#include "../bms/battery_manager.hpp"

namespace autocore {
namespace motor {

class MotorController {
public:
    MotorController(std::shared_ptr<bms::BatteryManager> bms);
    ~MotorController() = default;

    // Core motor control functions
    void setTargetSpeed(float speedRPM);
    void setTargetTorque(float torqueNm);
    float getCurrentSpeed() const;
    float getCurrentTorque() const;

    // Regenerative braking
    void enableRegenerativeBraking(bool enable);
    void setRegenerativeBrakingLevel(float level);  // 0.0 to 1.0
    float getRegenerativeEnergy() const;

    // Motor efficiency and thermal management
    float getEfficiency() const;
    float getTemperature() const;
    void updateThermalState();

    // System status and diagnostics
    bool performSelfTest();
    std::vector<std::string> getDiagnostics() const;

private:
    std::unique_ptr<MotorState> motorState_;
    std::unique_ptr<InverterController> inverterController_;
    std::shared_ptr<bms::BatteryManager> batteryManager_;
    
    bool regenerativeBrakingEnabled_;
    float regenerativeBrakingLevel_;
    float targetSpeed_;
    float targetTorque_;
    
    void optimizePowerDelivery();
    void monitorTemperature();
    void updateInverterParameters();
};

} // namespace motor
} // namespace autocore 