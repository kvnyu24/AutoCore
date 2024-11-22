#pragma once

#include "sensor_types.hpp"

namespace autocore {
namespace sensors {

class Sensor {
public:
    virtual ~Sensor() = default;
    virtual SensorData getData() const = 0;
    virtual bool calibrate() = 0;
    virtual SensorType getType() const = 0;
};

} // namespace sensors
} // namespace autocore 