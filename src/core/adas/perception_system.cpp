#include "perception_system.hpp"
#include <algorithm>

namespace evlib {
namespace adas {

PerceptionSystem::PerceptionSystem(std::shared_ptr<sensors::SensorManager> sensorManager)
    : sensorManager_(std::move(sensorManager)) {
    initializeDetectors();
}

void PerceptionSystem::update() {
    // Get fused sensor data
    auto sensorData = sensorManager_->getFusedData();
    
    // Process different perception tasks in parallel
    std::future<ObjectList> objectsFuture = 
        std::async(std::launch::async, 
                  &PerceptionSystem::detectObjects, this, sensorData);
                  
    std::future<LaneInfo> lanesFuture = 
        std::async(std::launch::async, 
                  &PerceptionSystem::detectLanes, this, sensorData);
                  
    // Wait for results
    currentObjects_ = objectsFuture.get();
    currentLanes_ = lanesFuture.get();
    
    // Update tracking
    updateObjectTracking();
}

ObjectList PerceptionSystem::detectObjects(const sensors::FusedData& sensorData) {
    ObjectList objects;
    
    // Process LiDAR point cloud
    auto lidarObjects = processLidarData(sensorData.lidarData);
    
    // Process camera images
    auto cameraObjects = processCameraData(sensorData.cameraData);
    
    // Fuse detections
    objects = fuseDetections(lidarObjects, cameraObjects);
    
    return objects;
}

void PerceptionSystem::updateObjectTracking() {
    for (auto& object : currentObjects_) {
        // Update Kalman filter for each tracked object
        if (trackedObjects_.contains(object.id)) {
            trackedObjects_[object.id].update(object);
        } else {
            trackedObjects_.emplace(object.id, TrackedObject(object));
        }
        
        // Predict object trajectory
        object.predictedPath = predictObjectTrajectory(object);
    }
}

} // namespace adas
} // namespace evlib 