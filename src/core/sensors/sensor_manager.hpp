#pragma once

#include <memory>
#include <unordered_map>
#include <string>
#include "sensor_types.hpp"
#include "../common/types.hpp"
#include "calibration_manager.hpp"
#include "fusion_engine.hpp"
#include "sensor.hpp"
#include "map.hpp"

namespace autocore {
namespace sensors {

class SensorManager {
public:
    SensorManager();
    ~SensorManager() = default;

    // Sensor registration and management
    bool registerSensor(const std::string& id, SensorType type);
    bool removeSensor(const std::string& id);
    
    // Data collection and processing
    void updateSensorData();
    SensorData getRawData(const std::string& sensorId) const;
    FusedData getFusedData() const;

    // Calibration
    bool calibrateSensor(const std::string& sensorId);
    bool calibrateAllSensors();
    
    // Health monitoring
    std::unordered_map<std::string, SensorHealth> getSensorHealth() const;
    
    // SLAM functionality
    Position getCurrentPosition() const;
    Map getEnvironmentMap() const;

private:
    std::unordered_map<std::string, std::unique_ptr<Sensor>> sensors_;
    std::unique_ptr<CalibrationManager> calibrationManager_;
    std::unique_ptr<FusionEngine> fusionEngine_;
    
    void preprocessSensorData();
    void updateSLAM();
};

} // namespace sensors
} // namespace autocore 