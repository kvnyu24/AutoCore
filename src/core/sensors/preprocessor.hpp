#pragma once

#include <optional>
#include "sensor_data.hpp"

namespace autocore {
namespace sensors {

class Preprocessor {
public:
    Preprocessor() = default;
    ~Preprocessor() = default;

    // Main processing function for raw sensor data
    SensorData process(const RawSensorData& rawData);

    // Set calibration parameters
    void setCalibrationMatrix(const std::vector<float>& matrix) { calibrationMatrix_ = matrix; }
    void setDataRange(float min, float max) { dataMin_ = min; dataMax_ = max; }

private:
    // Processing steps
    SensorData removeNoise(const RawSensorData& data);
    SensorData applySensorCalibration(const SensorData& data);
    SensorData normalizeData(const SensorData& data);
    SensorData applyTemporalFilter(const SensorData& current, const SensorData& previous);

    // Calibration parameters
    std::vector<float> calibrationMatrix_;
    float dataMin_{0.0f};
    float dataMax_{1.0f};
    std::optional<SensorData> previousData_;
};

} // namespace sensors
} // namespace autocore
