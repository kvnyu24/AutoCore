#pragma once

#include <memory>
#include <vector>
#include <unordered_map>
#include <string>
#include "sensor_types.hpp"
#include "sensor_data.hpp"
#include "../common/types.hpp"

namespace autocore {
namespace sensors {

class CalibrationManager {
public:
    CalibrationManager() = default;
    ~CalibrationManager() = default;

    // Core calibration functions
    bool calibrateSensor(const std::string& sensorId, SensorType type);
    bool calibrateAllSensors();
    bool validateCalibration(const std::string& sensorId) const;
    
    // Calibration parameters
    void setCalibrationMatrix(const std::string& sensorId, 
                            const std::vector<std::vector<float>>& matrix);
    void setDataRange(const std::string& sensorId, float min, float max);
    
    // Calibration data access
    std::vector<std::vector<float>> getCalibrationMatrix(const std::string& sensorId) const;
    std::pair<float, float> getDataRange(const std::string& sensorId) const;
    bool isCalibrated(const std::string& sensorId) const;

    // Data processing
    SensorData applyCalibration(const std::string& sensorId, const RawSensorData& data);
    std::vector<float> applyCalibrationMatrix(
        const std::vector<float>& values,
        const std::vector<std::vector<float>>& matrix);

private:
    struct CalibrationData {
        std::vector<std::vector<float>> calibrationMatrix;
        float dataMin{0.0f};
        float dataMax{1.0f};
        bool isCalibrated{false};
        std::chrono::system_clock::time_point lastCalibration;
    };

    std::unordered_map<std::string, CalibrationData> calibrationData_;
    
    // Internal methods
    bool performSensorCalibration(const std::string& sensorId, SensorType type);
    void updateCalibrationParameters(const std::string& sensorId);
    bool validateCalibrationMatrix(const std::vector<std::vector<float>>& matrix) const;
};

} // namespace sensors
} // namespace autocore
