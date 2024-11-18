#pragma once

#include <string>
#include <vector>
#include <memory>
#include <chrono>

namespace autocore {
namespace telematics {

struct Certificate {
    std::string cert;
    std::string privateKey;
    std::chrono::system_clock::time_point expiryDate;
};

struct SecurityConfig {
    std::string caPath;
    std::string certPath;
    std::string keyPath;
    bool validatePeer{true};
    int encryptionLevel{256};  // AES bit strength
};

class SecurityManager {
public:
    SecurityManager();
    ~SecurityManager() = default;

    // Certificate management
    bool loadCertificates(const SecurityConfig& config);
    bool rotateCertificates();
    bool validateCertificate(const std::string& cert);
    
    // Encryption/Decryption
    std::vector<uint8_t> encryptData(const std::vector<uint8_t>& data);
    std::vector<uint8_t> decryptData(const std::vector<uint8_t>& encryptedData);
    
    // Security validation
    bool validateServerIdentity(const std::string& serverCert);
    bool validateMessageSignature(const std::vector<uint8_t>& message, 
                                const std::vector<uint8_t>& signature);
    
    // Key management
    void rotateEncryptionKeys();
    bool generateCSR(const std::string& commonName);
    
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    
    Certificate currentCert_;
    SecurityConfig config_;
    
    bool validateKeyPair();
    void scheduleKeyRotation();
    bool storeCertificates();
    void monitorCertificateExpiry();
};

} // namespace telematics
} // namespace autocore
