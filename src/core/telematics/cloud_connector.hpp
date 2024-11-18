#pragma once

#include <string>
#include <vector>
#include "mqtt_client.hpp"
#include "security_manager.hpp"

namespace autocore {
namespace telematics {

class CloudConnector {
public:
    CloudConnector();
    
    // Connection management
    bool connect(const std::string& endpoint, const std::string& apiKey);
    void disconnect();
    bool isConnected() const;
    
    // Data transmission
    bool publishData(const std::string& topic, const TelemetryData& data);
    bool subscribeToTopic(const std::string& topic);
    std::vector<Message> receiveMessages();
    
    // Security
    bool rotateCertificates();
    bool validateServerCertificate();

private:
    std::unique_ptr<MQTTClient> mqttClient_;
    std::unique_ptr<SecurityManager> securityManager_;
    
    bool setupTLS();
    bool handleReconnection();
    void logConnectionMetrics();
};

} // namespace telematics
} // namespace autocore 