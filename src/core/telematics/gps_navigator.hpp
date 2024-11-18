#pragma once

#include <memory>
#include <vector>
#include "../sensors/sensor_manager.hpp"
#include "../sensors/sensor_types.hpp"

namespace autocore {
namespace telematics {

struct Route {
    std::vector<sensors::Position> waypoints;
    float estimatedTime;          // in seconds
    float totalDistance;          // in meters
    bool isValid{true};
};

class GPSNavigator {
public:
    GPSNavigator(std::shared_ptr<sensors::SensorManager> sensorManager);
    ~GPSNavigator() = default;

    // Core navigation functions
    sensors::Position getCurrentPosition() const;
    Route calculateRoute(const sensors::Position& destination);
    void updateNavigation();
    
    // Route management
    bool validateRoute(const Route& route) const;
    float getDistanceToDestination() const;
    float getEstimatedTimeToArrival() const;
    
    // Navigation status
    bool isRouteActive() const;
    bool isGPSSignalValid() const;
    std::vector<std::string> getNavigationInstructions() const;

private:
    std::shared_ptr<sensors::SensorManager> sensorManager_;
    Route currentRoute_;
    sensors::Position currentPosition_;
    
    void updatePosition();
    bool validateGPSSignal() const;
    float calculateRouteDistance(const std::vector<sensors::Position>& waypoints) const;
    std::vector<sensors::Position> optimizeRoute(const std::vector<sensors::Position>& waypoints) const;
};

} // namespace telematics
} // namespace autocore
