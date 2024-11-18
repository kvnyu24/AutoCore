#pragma once

#include <memory>
#include <vector>
#include "../sensors/sensor_types.hpp"
#include "../sensors/position.hpp"
#include "perception_system.hpp"

namespace autocore {
namespace adas {

struct PathSegment {
    std::vector<sensors::Position> points;
    float curvature;
    float recommendedSpeed;
    bool isValid{true};
};

struct PlanningConstraints {
    float maxCurvature{0.5f};
    float maxAcceleration{3.0f};
    float maxDeceleration{5.0f};
    float safetyMargin{1.0f};
};

class PathPlanner {
public:
    PathPlanner(std::shared_ptr<PerceptionSystem> perceptionSystem);
    ~PathPlanner() = default;

    // Core path planning
    std::vector<PathSegment> planPath(
        const sensors::Position& currentPos,
        const sensors::Position& targetPos);
    
    // Path validation and optimization
    bool validatePath(const std::vector<PathSegment>& path);
    std::vector<PathSegment> optimizePath(const std::vector<PathSegment>& path);
    
    // Safety checks
    bool checkCollisionFree(const PathSegment& segment);
    float calculateSafetyScore(const std::vector<PathSegment>& path);
    
    // Configuration
    void setConstraints(const PlanningConstraints& constraints);
    void updateSafetyMargins(float lateral, float longitudinal);

private:
    std::shared_ptr<PerceptionSystem> perceptionSystem_;
    PlanningConstraints constraints_;
    
    // Internal planning methods
    std::vector<PathSegment> generateCandidatePaths();
    PathSegment smoothSegment(const PathSegment& segment);
    bool validateSegment(const PathSegment& segment);
    float evaluatePathCost(const std::vector<PathSegment>& path);
    void adjustForDynamicObstacles(std::vector<PathSegment>& path);
};

} // namespace adas
} // namespace autocore 