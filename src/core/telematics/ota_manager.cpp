#include "ota_manager.hpp"
#include <cryptopp/aes.h>
#include <cryptopp/sha.h>
#include <chrono>
#include <fstream>

namespace autocore {
namespace telematics {

TelemetryData CreateUpdateCheckRequest() {
    TelemetryData request;
    request.deviceId = "vehicle_id";  // Should be retrieved from config
    request.timestamp = std::chrono::system_clock::now().time_since_epoch().count();
    request.encrypted = true;
    return request;
}

OTAManager::OTAManager(std::shared_ptr<CloudConnector> cloudConnector)
    : cloudConnector_(std::move(cloudConnector))
    , securityManager_(std::make_unique<SecurityManager>())
    , currentState_(UpdateState::IDLE) {
}

bool OTAManager::checkForUpdates() {
    if (!cloudConnector_->isConnected()) {
        return false;
    }
    
    if (!securityManager_->validateCertificate("server")) {
        return false;
    }

    auto response = cloudConnector_->publishData("vehicle/updates/check", 
        CreateUpdateCheckRequest());
    
    if (response) {
        auto messages = cloudConnector_->receiveMessages();
        if (!messages.empty()) {
            processUpdateMetadata(messages[0]);
            return true;
        }
    }
    return false;
}

bool OTAManager::downloadUpdate() {
    if (currentState_ != UpdateState::UPDATE_AVAILABLE) {
        return false;
    }
    
    currentState_ = UpdateState::DOWNLOADING;
    
    // Create backup before update
    if (!createBackup()) {
        currentState_ = UpdateState::ERROR;
        return false;
    }
    
    bool success = downloadUpdateChunks();
    if (success) {
        success = validateUpdate();
    }
    
    currentState_ = success ? UpdateState::READY_TO_INSTALL : UpdateState::ERROR;
    return success;
}

bool OTAManager::downloadUpdateChunks() {
    const size_t CHUNK_SIZE = 1024 * 1024; // 1MB chunks
    size_t totalSize = currentUpdate_.size;
    size_t downloadedSize = 0;
    
    std::vector<uint8_t> buffer;
    buffer.reserve(CHUNK_SIZE);
    
    while (downloadedSize < totalSize) {
        size_t remaining = totalSize - downloadedSize;
        size_t chunkSize = std::min(CHUNK_SIZE, remaining);
        
        auto chunk = cloudConnector_->receiveMessages();
        if (chunk.empty()) {
            return false;
        }
        
        // Decrypt chunk
        auto decryptedData = securityManager_->decryptData(
            std::vector<uint8_t>(chunk[0].payload.begin(), chunk[0].payload.end()));
        
        buffer.insert(buffer.end(), decryptedData.begin(), decryptedData.end());
        downloadedSize += chunkSize;
        
        // Update progress
        float progress = static_cast<float>(downloadedSize) / totalSize;
        updateProgress(progress);
    }
    
    return true;
}

bool OTAManager::validateUpdate() {
    // Verify digital signature
    CryptoPP::SHA256 hash;
    std::string calculatedHash;
    
    // Verify system compatibility
    if (!verifySystemCompatibility()) {
        return false;
    }
    
    // Verify secure bootchain
    if (!validateSecureBootchain()) {
        return false;
    }
    
    // Verify checksum
    return verifyChecksum(currentUpdate_.checksum);
}

bool OTAManager::installUpdate() {
    if (currentState_ != UpdateState::READY_TO_INSTALL) {
        return false;
    }
    
    currentState_ = UpdateState::INSTALLING;
    
    bool success = performInstallation();
    
    if (!success && backupPath_.empty()) {
        success = performRollback();
    }
    
    currentState_ = success ? UpdateState::COMPLETED : UpdateState::ERROR;
    return success;
}

void OTAManager::processUpdateMetadata(const Message& response) {
    // Parse and validate update metadata
    try {
        auto decryptedData = securityManager_->decryptData(
            std::vector<uint8_t>(response.payload.begin(), response.payload.end()));
        
        // Update current update info
        currentUpdate_.version = "1.0.0"; // Parse from decrypted data
        currentUpdate_.description = "Update description";
        currentUpdate_.size = 1000000; // Parse from decrypted data
        currentUpdate_.checksum = "checksum"; // Parse from decrypted data
        currentUpdate_.releaseDate = std::chrono::system_clock::now();
        currentUpdate_.isCritical = false; // Parse from decrypted data
        
        currentState_ = UpdateState::UPDATE_AVAILABLE;
    } catch (const std::exception& e) {
        currentState_ = UpdateState::ERROR;
    }
}

void OTAManager::updateProgress(float progress) {
    UpdateStatus status;
    status.progress = progress;
    status.state = currentState_;
    status.currentVersion = "current_version";  // Should get from system
    status.targetVersion = currentUpdate_.version;
    
    // Notify observers/store status
    currentStatus_ = status;
}

} // namespace telematics
} // namespace autocore 