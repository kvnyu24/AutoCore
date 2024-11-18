#pragma once

#include <memory>
#include <vector>

namespace autocore {
namespace motor {

class InverterController {
public:
    InverterController();
    ~InverterController() = default;

    // Power control
    void setVoltage(float voltage);
    void updatePWM(float targetSpeed, float targetTorque);
    float getInputPower() const;
    float getOutputPower() const;

    // Inverter parameters
    void setFrequency(float frequency);
    void setModulationIndex(float index);
    float getCurrentEfficiency() const;

    // Diagnostics and monitoring
    bool selfTest();
    std::vector<std::string> getDiagnostics() const;
    float getTemperature() const;
    bool detectFaults();

private:
    float voltage_;
    float frequency_;
    float modulationIndex_;
    float inputPower_;
    float outputPower_;
    float temperature_;

    void updateEfficiency();
    void monitorTemperature();
    void optimizeModulation();
    bool validateParameters();
};

} // namespace motor
} // namespace autocore
