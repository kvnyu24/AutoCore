#include "perception_system.hpp"
#include <algorithm>

namespace autocore {
namespace adas {

PerceptionSystem::PerceptionSystem(std::shared_ptr<sensors::SensorManager> sensorManager)
    : sensorManager_(std::move(sensorManager)) {
    initializeDetectors();
}

void PerceptionSystem::initializeDetectors() {
    // Initialize Kalman filters for tracking
    kalmanFilter_ = std::make_unique<sensors::KalmanFilter<float>>();
    
    // Set up sensor calibration parameters
    auto calibrationManager = sensorManager_->getCalibrationManager();
    if (calibrationManager) {
        lidarCalibMatrix_ = calibrationManager->getCalibrationMatrix("lidar1");
        cameraCalibMatrix_ = calibrationManager->getCalibrationMatrix("camera1");
    }
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
    
    // Get LiDAR and camera data from sensor inputs
    auto lidarData = std::find_if(sensorData.sensorInputs.begin(), 
                                 sensorData.sensorInputs.end(),
                                 [](const sensors::SensorData& data) {
                                     return data.type == sensors::SensorType::LIDAR;
                                 });
                                 
    auto cameraData = std::find_if(sensorData.sensorInputs.begin(), 
                                  sensorData.sensorInputs.end(),
                                  [](const sensors::SensorData& data) {
                                      return data.type == sensors::SensorType::CAMERA;
                                  });
    
    // Process sensor data if available
    ObjectList lidarObjects, cameraObjects;
    if (lidarData != sensorData.sensorInputs.end()) {
        lidarObjects = processLidarData(*lidarData);
    }
    
    if (cameraData != sensorData.sensorInputs.end()) {
        cameraObjects = processCameraData(*cameraData);
    }
    
    // Fuse detections
    objects = fuseDetections(lidarObjects, cameraObjects);
    
    return objects;
}

ObjectList PerceptionSystem::processLidarData(const sensors::SensorData& lidarData) {
    ObjectList detectedObjects;
    
    // Apply calibration and preprocessing
    auto processedData = preprocessLidarData(lidarData);
    
    // Segment point cloud into clusters
    auto clusters = segmentPointCloud(processedData);
    
    // Classify each cluster
    for (const auto& cluster : clusters) {
        if (auto object = classifyLidarCluster(cluster)) {
            detectedObjects.push_back(*object);
        }
    }
    
    return detectedObjects;
}

ObjectList PerceptionSystem::processCameraData(const sensors::SensorData& cameraData) {
    ObjectList detectedObjects;
    
    // Apply image preprocessing
    auto processedImage = preprocessImageData(cameraData);
    
    // Run object detection on image
    auto detections = detectObjectsInImage(processedImage);
    
    // Convert detections to 3D space using camera parameters
    for (const auto& detection : detections) {
        if (auto object = convertTo3DObject(detection)) {
            detectedObjects.push_back(*object);
        }
    }
    
    return detectedObjects;
}

ObjectList PerceptionSystem::fuseDetections(
    const ObjectList& lidarObjects, 
    const ObjectList& cameraObjects) {
    
    ObjectList fusedObjects;
    
    // Match objects between sensors using IOU
    auto matches = matchDetections(lidarObjects, cameraObjects);
    
    // Fuse matched detections
    for (const auto& match : matches) {
        Object fusedObject;
        fusedObject.position = fuseLidarCameraPosition(
            match.lidarObject.position, 
            match.cameraObject.position);
        fusedObject.dimensions = match.lidarObject.dimensions;
        fusedObject.type = match.cameraObject.type;
        fusedObject.confidence = 
            (match.lidarObject.confidence + match.cameraObject.confidence) / 2.0f;
        
        fusedObjects.push_back(fusedObject);
    }
    
    return fusedObjects;
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

Path PerceptionSystem::predictObjectTrajectory(const Object& object) {
    Path predictedPath;
    const float predictionHorizon = 5.0f; // 5 seconds
    const float timeStep = 0.1f; // 100ms
    
    // Get current state from tracked object
    auto trackedObject = trackedObjects_.at(object.id);
    auto state = trackedObject.getState();
    
    // Predict future positions
    for (float t = 0; t < predictionHorizon; t += timeStep) {
        Position predictedPos;
        predictedPos.x = state.position.x + state.velocity.x * t;
        predictedPos.y = state.position.y + state.velocity.y * t;
        predictedPos.z = state.position.z + state.velocity.z * t;
        
        predictedPath.waypoints.push_back(predictedPos);
        predictedPath.timestamps.push_back(t);
    }
    
    return predictedPath;
}

LaneInfo PerceptionSystem::detectLanes(const sensors::FusedData& sensorData) {
    LaneInfo lanes;
    
    // Process camera image for lane detection
    auto cameraData = sensorData.cameraData;
    
    // Apply lane detection algorithm
    auto laneMarkings = detectLaneMarkings(cameraData);
    
    // Fit lane models
    lanes.leftLane = fitLaneModel(laneMarkings.leftMarkings);
    lanes.rightLane = fitLaneModel(laneMarkings.rightMarkings);
    
    // Calculate lane parameters
    lanes.width = calculateLaneWidth(lanes.leftLane, lanes.rightLane);
    lanes.curvature = calculateLaneCurvature(lanes.leftLane, lanes.rightLane);
    
    return lanes;
}

SensorData PerceptionSystem::preprocessLidarData(const sensors::SensorData& lidarData) {
    sensors::Preprocessor preprocessor;
    preprocessor.setCalibrationMatrix(lidarCalibMatrix_);
    return preprocessor.process(lidarData);
}

TrackedObject PerceptionSystem::classifyLidarCluster(const PointCluster& cluster) const {
    TrackedObject object;
    object.position = cluster.centroid;
    
    // Calculate geometric features
    float height = calculateClusterHeight(cluster.points);
    float width = calculateClusterWidth(cluster.points);
    float depth = calculateClusterDepth(cluster.points);
    
    // Classify based on dimensions and density
    if (isVehicleLike(height, width, depth)) {
        object.type = ObjectType::VEHICLE;
    } else if (isPedestrianLike(height, width, depth)) {
        object.type = ObjectType::PEDESTRIAN;
    } else if (isCyclistLike(height, width, depth)) {
        object.type = ObjectType::CYCLIST;
    } else {
        object.type = ObjectType::UNKNOWN;
    }
    
    object.confidence = cluster.confidence;
    return object;
}

SensorData PerceptionSystem::preprocessImageData(const SensorData& imageData) {
    SensorData processed = imageData;
    
    // Convert to appropriate format for neural network
    processed.values = normalizeImageData(imageData.values);
    
    // Apply camera calibration
    processed.values = applyCalibrationMatrix(processed.values, cameraCalibMatrix_);
    
    return processed;
}

std::vector<TrackedObject> PerceptionSystem::detectObjectsInImage(
    const SensorData& processedImage) {
    std::vector<TrackedObject> objects;
    
    // Run object detection neural network
    std::vector<ImageDetection> detections = runObjectDetection(processedImage);
    
    // Filter low confidence detections
    detections.erase(
        std::remove_if(detections.begin(), detections.end(),
            [this](const ImageDetection& det) {
                return det.confidence < MIN_DETECTION_CONFIDENCE;
            }),
        detections.end());
    
    // Convert remaining detections to 3D objects
    for (const auto& detection : detections) {
        float depth = estimateDepth(detection);
        objects.push_back(convertTo3DObject(detection, depth));
    }
    
    return objects;
}

TrackedObject PerceptionSystem::convertTo3DObject(
    const ImageDetection& detection,
    float depth) const {
    TrackedObject object;
    
    // Convert image coordinates to 3D world coordinates
    object.position = imageToWorld(
        detection.boundingBox.x + detection.boundingBox.width/2,
        detection.boundingBox.y + detection.boundingBox.height/2,
        depth
    );
    
    object.type = detection.type;
    object.confidence = detection.confidence;
    
    return object;
}

std::vector<TrackedObject> PerceptionSystem::matchDetections(
    const std::vector<TrackedObject>& lidarObjects,
    const std::vector<TrackedObject>& cameraObjects) {
    std::vector<TrackedObject> fusedObjects;
    std::vector<bool> cameraMatched(cameraObjects.size(), false);
    
    // For each LiDAR detection, find closest camera detection
    for (const auto& lidarObj : lidarObjects) {
        float minDist = LIDAR_CAMERA_MATCH_THRESHOLD;
        int bestMatch = -1;
        
        // Find closest camera detection
        for (size_t i = 0; i < cameraObjects.size(); i++) {
            if (cameraMatched[i]) continue;
            
            float dist = lidarObj.position.distanceTo(cameraObjects[i].position);
            if (dist < minDist) {
                minDist = dist;
                bestMatch = i;
            }
        }
        
        if (bestMatch >= 0) {
            // Fuse matching detections
            fusedObjects.push_back(fuseDetections(
                lidarObj, cameraObjects[bestMatch]));
            cameraMatched[bestMatch] = true;
        } else {
            // Keep LiDAR-only detection
            fusedObjects.push_back(lidarObj);
        }
    }
    
    // Add unmatched camera detections
    for (size_t i = 0; i < cameraObjects.size(); i++) {
        if (!cameraMatched[i]) {
            fusedObjects.push_back(cameraObjects[i]);
        }
    }
    
    return fusedObjects;
}

Position PerceptionSystem::fuseLidarCameraPosition(
    const Position& lidarPos,
    const Position& cameraPos,
    float lidarConfidence,
    float cameraConfidence) {
    
    float totalConfidence = lidarConfidence + cameraConfidence;
    float lidarWeight = lidarConfidence / totalConfidence;
    float cameraWeight = cameraConfidence / totalConfidence;
    
    return Position(
        lidarPos.x * lidarWeight + cameraPos.x * cameraWeight,
        lidarPos.y * lidarWeight + cameraPos.y * cameraWeight,
        lidarPos.z * lidarWeight + cameraPos.z * cameraWeight
    );
}

} // namespace adas
} // namespace autocore 