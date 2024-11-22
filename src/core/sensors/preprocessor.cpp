#include "preprocessor.hpp"
#include <algorithm>
#include <cmath>
#include "sensor_types.hpp"

namespace autocore {
namespace sensors {

SensorData Preprocessor::process(const RawSensorData& rawData) {
    SensorData processedData;
    
    // Remove noise
    processedData = removeNoise(rawData);
    
    // Calibrate data using sensor-specific parameters
    processedData = applySensorCalibration(processedData);
    
    // Normalize data
    processedData = normalizeData(processedData);
    
    // Apply temporal filtering
    if (previousData_.has_value()) {
        processedData = applyTemporalFilter(processedData, previousData_.value());
    }
    
    previousData_ = processedData;
    return processedData;
}

SensorData Preprocessor::removeNoise(const RawSensorData& data) {
    // Implement noise removal using moving average or median filter
    SensorData filtered;
    // ... implementation
    return filtered;
}

SensorData Preprocessor::applySensorCalibration(const SensorData& data) {
    // Apply calibration parameters
    SensorData calibrated = data;
    calibrated.values = applyCalibrationMatrix(data.values, calibrationMatrix_);
    return calibrated;
}

SensorData Preprocessor::normalizeData(const SensorData& data) {
    SensorData normalized = data;
    // Normalize values to standard range
    for (auto& value : normalized.values) {
        value = (value - dataMin_) / (dataMax_ - dataMin_);
    }
    return normalized;
}

std::vector<float> Preprocessor::applyCalibrationMatrix(
    const std::vector<float>& values,
    const std::vector<std::vector<float>>& matrix) {
    std::vector<float> result(values.size());
    for (size_t i = 0; i < values.size(); ++i) {
        result[i] = 0;
        for (size_t j = 0; j < values.size(); ++j) {
            result[i] += matrix[i][j] * values[j];
        }
    }
    return result;
}

} // namespace sensors
} // namespace autocore 