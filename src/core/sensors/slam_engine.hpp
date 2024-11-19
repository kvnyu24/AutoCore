#pragma once

#include <vector>
#include "sensor_data.hpp"
#include "map.hpp"
#include "../common/types.hpp"

namespace autocore {
namespace sensors {

class SLAMEngine {
public:
    SLAMEngine();
    
    // Core SLAM functionality
    void update(const FusedData& sensorData);
    Position getEstimatedPosition() const;
    Map getCurrentMap() const;
    
    // Feature detection and tracking
    std::vector<Feature> detectFeatures(const SensorData& data);
    void trackFeatures(const std::vector<Feature>& features);
    
    // Loop closure
    bool detectLoopClosure();
    void optimizeMap();

private:
    Map currentMap_;
    Position currentPosition_;
    std::vector<Feature> trackedFeatures_;
    
    void updateParticleFilter();
    void performGraphOptimization();
};

} // namespace sensors
} // namespace autocore 