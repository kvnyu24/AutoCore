#pragma once

#include <memory>
#include <future>
#include <opencv2/core.hpp>
#include "../sensors/sensor_manager.hpp"
#include "../sensors/fusion_engine.hpp"
#include "../sensors/sensor_types.hpp"
#include "../sensors/fusion_types.hpp"
#include "path_planner.hpp"

namespace autocore {
namespace adas {

using sensors::TrackedObject;
using sensors::Position;
using sensors::SensorData;
using sensors::FusedData;

struct PointCluster {
    std::vector<sensors::Position> points;
    sensors::Position centroid;
    float radius;  // Bounding radius
    float confidence{0.0f};
    bool isValid{true};
};

struct LaneInfo {
    std::vector<Position> leftBoundary;
    std::vector<Position> rightBoundary;
    float confidence;
    bool isValid{true};
};

using ObjectList = std::vector<TrackedObject>;

class PerceptionSystem {
public:
    // New helper structs
    struct ImageDetection {
        cv::Rect boundingBox;
        float confidence;
        ObjectType type;
        std::vector<float> features;
    };

    PerceptionSystem(std::shared_ptr<sensors::SensorManager> sensorManager);
    ~PerceptionSystem() = default;

    // Core perception functionality
    void update();
    ObjectList getCurrentObjects() const { return currentObjects_; }
    LaneInfo getCurrentLanes() const { return currentLanes_; }
    
    // Object detection and tracking
    ObjectList detectObjects(const sensors::FusedData& sensorData);
    void updateObjectTracking();
    std::vector<Position> predictObjectTrajectory(const TrackedObject& object);

    // Lane detection
    LaneInfo detectLanes(const sensors::FusedData& sensorData);
    bool validateLaneDetection(const LaneInfo& lanes) const;

    // Scene analysis
    float assessSceneRisk() const;
    bool detectEmergencySituation() const;
    std::vector<std::string> getPerceptionWarnings() const;

private:
    std::shared_ptr<sensors::SensorManager> sensorManager_;
    std::unique_ptr<sensors::KalmanFilter<float>> kalmanFilter_;
    std::vector<std::vector<float>> lidarCalibMatrix_;
    std::vector<std::vector<float>> cameraCalibMatrix_;
    std::unordered_map<uint32_t, TrackedObject> trackedObjects_;
    std::vector<PointCluster> segmentPointCloud(const sensors::SensorData& data);
    ObjectList currentObjects_;
    LaneInfo currentLanes_;
    
    // Processing methods
    ObjectList processLidarData(const SensorData& lidarData);
    ObjectList processCameraData(const SensorData& cameraData);
    ObjectList fuseDetections(const ObjectList& lidarObjects, 
                            const ObjectList& cameraObjects);
    SensorData preprocessLidarData(const sensors::SensorData& lidarData);
    
    void initializeDetectors();
    void updateObjectStates();
    float calculateCollisionRisk(const TrackedObject& object) const;
    bool validateDetection(const TrackedObject& object) const;

    // Object detection helpers
    TrackedObject classifyLidarCluster(const PointCluster& cluster) const;
    SensorData preprocessImageData(const SensorData& imageData);
    std::vector<TrackedObject> detectObjectsInImage(const SensorData& processedImage);
    TrackedObject convertTo3DObject(const ImageDetection& detection, float depth) const;
    std::vector<TrackedObject> matchDetections(
        const std::vector<TrackedObject>& lidarObjects,
        const std::vector<TrackedObject>& cameraObjects);
    Position fuseLidarCameraPosition(
        const Position& lidarPos,
        const Position& cameraPos,
        float lidarConfidence,
        float cameraConfidence);

    // Constants
    static constexpr float LIDAR_CAMERA_MATCH_THRESHOLD = 2.0f;  // meters
    static constexpr float MIN_DETECTION_CONFIDENCE = 0.3f;
    static constexpr float FUSION_DISTANCE_WEIGHT = 0.7f;
};

} // namespace adas
} // namespace autocore