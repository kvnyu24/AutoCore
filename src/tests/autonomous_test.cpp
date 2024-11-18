#include <gtest/gtest.h>
#include "../core/autonomous/autonomous_manager.hpp"
#include "mock_components.hpp"

class AutonomousTest : public ::testing::Test {
protected:
    std::shared_ptr<MockADASManager> adasManager_;
    std::shared_ptr<MockFusionEngine> fusionEngine_;
    std::shared_ptr<MockVehicleControlSystem> vcs_;
    std::unique_ptr<evlib::autonomous::AutonomousManager> autonomousManager_;

    void SetUp() override {
        adasManager_ = std::make_shared<MockADASManager>();
        fusionEngine_ = std::make_shared<MockFusionEngine>();
        vcs_ = std::make_shared<MockVehicleControlSystem>();
        
        autonomousManager_ = std::make_unique<evlib::autonomous::AutonomousManager>(
            adasManager_, fusionEngine_, vcs_);
    }
};

TEST_F(AutonomousTest, SystemInitialization) {
    EXPECT_TRUE(autonomousManager_->checkSystemReadiness());
}

TEST_F(AutonomousTest, TrajectoryPlanning) {
    Position destination{37.7749, -122.4194};
    autonomousManager_->setDestination(destination);
    autonomousManager_->updatePerception();
    autonomousManager_->planTrajectory();
    
    auto state = autonomousManager_->getState();
    EXPECT_EQ(state.planningStatus, PlanningStatus::TRAJECTORY_READY);
}

TEST_F(AutonomousTest, SafetyValidation) {
    // Simulate obstacle detection
    fusionEngine_->simulateObstacle({10.0f, 0.0f, 0.0f});
    
    autonomousManager_->updatePerception();
    EXPECT_TRUE(autonomousManager_->validateSafety());
    
    auto safetyEvents = autonomousManager_->getActiveSafetyEvents();
    EXPECT_FALSE(safetyEvents.empty());
} 