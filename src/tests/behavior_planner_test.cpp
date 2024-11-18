#include <gtest/gtest.h>
#include "../core/autonomous/behavior/behavior_planner.hpp"
#include "mock_components.hpp"

using namespace autocore::sensors;
using namespace autocore::autonomous;

class BehaviorPlannerTest : public ::testing::Test {
protected:
    std::shared_ptr<autocore::tests::MockFusionEngine> fusionEngine_;
    std::shared_ptr<autocore::tests::MockSLAMEngine> slamEngine_;
    std::unique_ptr<BehaviorPlanner> behaviorPlanner_;

    void SetUp() override {
        fusionEngine_ = std::make_shared<autocore::tests::MockFusionEngine>();
        slamEngine_ = std::make_shared<autocore::tests::MockSLAMEngine>();
        
        behaviorPlanner_ = std::make_unique<BehaviorPlanner>(
            fusionEngine_, slamEngine_);
    }
};

TEST_F(BehaviorPlannerTest, ManeuverValidation) {
    SceneContext context;
    context.timestamp = std::chrono::system_clock::now();
    
    // Simulate normal driving scenario
    auto state = behaviorPlanner_->planBehavior(context);
    EXPECT_TRUE(behaviorPlanner_->validateManeuver(state.maneuver));
}

TEST_F(BehaviorPlannerTest, CollisionAvoidance) {
    // Simulate obstacle ahead
    fusionEngine_->simulateObstacle({20.0f, 0.0f, 0.0f});
    
    SceneContext context;
    auto state = behaviorPlanner_->planBehavior(context);
    
    // Should plan evasive maneuver
    EXPECT_NE(state.maneuver.type, ManeuverType::FOLLOW_LANE);
}