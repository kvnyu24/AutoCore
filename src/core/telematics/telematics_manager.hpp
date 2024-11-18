#pragma once

#include <memory>
#include <string>
#include <queue>
#include "../sensors/sensor_manager.hpp"
#include "../diagnostics/diagnostic_manager.hpp"
#include "gps_navigator.hpp"
#include "cloud_connector.hpp"
#include "ota_manager.hpp"

namespace evlib {
namespace telematics {

class TelematicsManager {
public:
    TelematicsManager(
        std::shared_ptr<sensors::SensorManager> sensorManager,
        std::shared_ptr<diagnostics::DiagnosticManager> diagnosticManager
    );
    
    // Cloud connectivity
    bool connectToCloud(const std::string& apiKey);
    bool sendTelemetryData();
    bool receiveTelemetryCommands();
    
    // GPS and Navigation
    Position getCurrentPosition() const;
    Route calculateRoute(const Position& destination);
    void updateNavigation();
    
    // OTA Updates
    bool checkForUpdates();
    bool downloadUpdate();
    bool installUpdate();
    UpdateStatus getUpdateStatus() const;
    
    // Real-time monitoring
    VehicleStatus getVehicleStatus() const;
    std::vector<Alert> getActiveAlerts() const;

private:
    std::shared_ptr<sensors::SensorManager> sensorManager_;
    std::shared_ptr<diagnostics::DiagnosticManager> diagnosticManager_;
    std::unique_ptr<GPSNavigator> gpsNavigator_;
    std::unique_ptr<CloudConnector> cloudConnector_;
    std::unique_ptr<OTAManager> otaManager_;
    
    std::queue<TelemetryData> telemetryQueue_;
    
    void processIncomingCommands();
    void aggregateTelemetryData();
    bool validateSecureConnection();
};

} // namespace telematics
} // namespace evlib 