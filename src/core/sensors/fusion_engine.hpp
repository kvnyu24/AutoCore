#pragma once

#include <vector>
#include "sensor_data.hpp"
#include "kalman_filter.hpp"
#include "fusion_types.hpp"

namespace autocore {
namespace sensors {

class FusionEngine {
public:
    FusionEngine();
    
    // Data fusion methods
    FusedData fuseSensorData(const std::vector<SensorData>& sensorData);
    
    // Filtering and estimation
    void updateKalmanFilter(const SensorData& measurement);
    StateEstimate getStateEstimate() const;
    
    // Configuration
    void setFusionParameters(const FusionParameters& params);
    void enableSensor(SensorType type, bool enabled);
    
private:
    std::unique_ptr<KalmanFilter<double>> kalmanFilter_;
    FusionParameters params_;
    std::vector<SensorType> enabledSensors_;
    
    FusedData performSensorFusion(const std::vector<SensorData>& data);
    void validateMeasurements(const std::vector<SensorData>& data);
};

} // namespace sensors
} // namespace autocore 