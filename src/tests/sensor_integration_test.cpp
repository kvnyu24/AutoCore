#include <gtest/gtest.h>
#include "../core/sensors/sensor_manager.hpp"

class SensorIntegrationTest : public ::testing::Test {
protected:
    std::unique_ptr<autocore::sensors::SensorManager> sensorManager_;

    void SetUp() override {
        sensorManager_ = std::make_unique<autocore::sensors::SensorManager>();
        // Register test sensors
        sensorManager_->registerSensor("lidar1", SensorType::LIDAR);
        sensorManager_->registerSensor("camera1", SensorType::CAMERA);
        sensorManager_->registerSensor("radar1", SensorType::RADAR);
    }
};

TEST_F(SensorIntegrationTest, SensorRegistration) {
    EXPECT_TRUE(sensorManager_->registerSensor("imu1", SensorType::IMU));
    auto health = sensorManager_->getSensorHealth();
    EXPECT_EQ(health.size(), 4);
}

TEST_F(SensorIntegrationTest, DataFusion) {
    sensorManager_->updateSensorData();
    auto fusedData = sensorManager_->getFusedData();
    EXPECT_TRUE(fusedData.isValid());
}

TEST_F(SensorIntegrationTest, SLAMOperation) {
    sensorManager_->updateSensorData();
    auto position = sensorManager_->getCurrentPosition();
    auto map = sensorManager_->getEnvironmentMap();
    EXPECT_TRUE(map.isInitialized());
}