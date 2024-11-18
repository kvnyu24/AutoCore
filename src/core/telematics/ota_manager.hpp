#pragma once

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include "cloud_connector.hpp"
#include "security_manager.hpp"

namespace autocore {
namespace telematics {

enum class UpdateState {
    IDLE,
    UPDATE_AVAILABLE,
    DOWNLOADING,
    READY_TO_INSTALL,
    INSTALLING,
    COMPLETED,
    ERROR
};

struct UpdateInfo {
    std::string version;
    std::string description;
    size_t size;
    std::string checksum;
    std::chrono::system_clock::time_point releaseDate;
    bool isCritical{false};
};

struct UpdateStatus {
    UpdateState state;
    float progress;               // 0.0 to 1.0
    std::string currentVersion;
    std::string targetVersion;
    std::string errorMessage;
};

class OTAManager {
public:
    OTAManager(std::shared_ptr<CloudConnector> cloudConnector);
    ~OTAManager() = default;

    // Update management
    bool checkForUpdates();
    bool downloadUpdate();
    bool installUpdate();
    UpdateStatus getUpdateStatus() const { return currentStatus_; }
    
    // Update configuration
    void setUpdatePolicy(bool autoDownload, bool autoInstall);
    void setMaintenanceWindow(
        const std::chrono::system_clock::time_point& start,
        const std::chrono::system_clock::time_point& end);
    
    // Update validation
    bool validateUpdate();
    bool verifyChecksum(const std::string& checksum);
    
    // Rollback management
    bool createBackup();
    bool performRollback();

private:
    std::shared_ptr<CloudConnector> cloudConnector_;
    std::unique_ptr<SecurityManager> securityManager_;
    UpdateState currentState_;
    UpdateInfo currentUpdate_;
    std::string backupPath_;
    UpdateStatus currentStatus_;
    
    bool downloadUpdateChunks();
    bool validateSecureBootchain();
    bool performInstallation();
    void processUpdateMetadata(const Message& response);
    bool verifySystemCompatibility();
    void updateProgress(float progress);
};

} // namespace telematics
} // namespace autocore
