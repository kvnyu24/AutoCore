#pragma once

#include <string>
#include <vector>
#include <memory>
#include <functional>

namespace autocore {
namespace telematics {

struct Message {
    std::string topic;
    std::vector<uint8_t> payload;
    uint64_t timestamp;
};

struct TelemetryData {
    std::vector<uint8_t> data;
    std::string deviceId;
    uint64_t timestamp;
    bool encrypted;
};

struct MQTTConfig {
    std::string endpoint;
    std::string clientId;
    int port{8883};  // Default TLS port
    int keepAliveInterval{60};
    bool useTLS{true};
};

class MQTTClient {
public:
    MQTTClient();
    ~MQTTClient() = default;

    // Connection management
    bool connect(const MQTTConfig& config);
    void disconnect();
    bool isConnected() const;
    
    // Message handling
    bool publish(const std::string& topic, const TelemetryData& data);
    bool subscribe(const std::string& topic);
    bool unsubscribe(const std::string& topic);
    std::vector<Message> receiveMessages();
    
    // Callbacks
    void setMessageCallback(std::function<void(const Message&)> callback);
    void setConnectionCallback(std::function<void(bool)> callback);
    
    // Configuration
    void setTLSConfig(const std::string& certPath, const std::string& keyPath);
    void setReconnectInterval(int seconds);
    
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    
    bool validateConnection();
    void handleReconnect();
    bool setupTLSConnection();
    void processIncomingMessages();
};

} // namespace telematics
} // namespace autocore
