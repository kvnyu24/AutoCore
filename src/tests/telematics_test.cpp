#include <gtest/gtest.h>
#include "../core/telematics/telematics_manager.hpp"
#include "mock_cloud_service.hpp"
#include "../core/sensors/sensor_types.hpp"

class TelematicsTest : public ::testing::Test {
protected:
    std::shared_ptr<MockCloudService> cloudService_;
    std::shared_ptr<autocore::sensors::SensorManager> sensorManager_;
    std::shared_ptr<autocore::diagnostics::DiagnosticManager> diagnosticManager_;
    std::unique_ptr<autocore::telematics::TelematicsManager> telematicsManager_;

    void SetUp() override {
        cloudService_ = std::make_shared<MockCloudService>();
        sensorManager_ = std::make_shared<autocore::sensors::SensorManager>();
        diagnosticManager_ = std::make_shared<autocore::diagnostics::DiagnosticManager>();
        telematicsManager_ = std::make_unique<autocore::telematics::TelematicsManager>(
            sensorManager_, diagnosticManager_);
    }
};

TEST_F(TelematicsTest, CloudConnection) {
    EXPECT_TRUE(telematicsManager_->connectToCloud("test_api_key"));
    EXPECT_TRUE(telematicsManager_->sendTelemetryData());
}

TEST_F(TelematicsTest, OTAUpdate) {
    cloudService_->setMockUpdate("1.2.0", "Critical security update");
    EXPECT_TRUE(telematicsManager_->checkForUpdates());
    EXPECT_TRUE(telematicsManager_->downloadUpdate());
    EXPECT_TRUE(telematicsManager_->installUpdate());
}

TEST_F(TelematicsTest, Navigation) {
    Position destination{37.7749, -122.4194};  // San Francisco coordinates
    auto route = telematicsManager_->calculateRoute(destination);
    EXPECT_FALSE(route.waypoints.empty());
} 