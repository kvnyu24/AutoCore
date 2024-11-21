#pragma once

#include <memory>
#include <queue>
#include <vector>
#include <map>
#include <iomanip>
#include <sstream>
#include "diagnostic_data.hpp"

using TimePoint = std::chrono::system_clock::time_point;

namespace autocore {
namespace diagnostics {

struct CANMessage {
    uint32_t id;
    std::vector<uint8_t> data;
    TimePoint timestamp;
    
    // New fields
    uint8_t dlc;                  // Data Length Code
    bool isExtendedId;           // Standard vs Extended ID
    bool isRemoteFrame;          // Remote Transmission Request
    bool isErrorFrame;           // Error Frame indicator
    uint32_t busChannel;         // Multi-channel support
    
    // Helper methods
    bool isValid() const {
        return data.size() <= 8 && data.size() == dlc;
    }
    
    std::string toString() const {
        std::stringstream ss;
        ss << std::hex << std::uppercase 
           << (isExtendedId ? "0x" : "0x") << id 
           << " [" << static_cast<int>(dlc) << "] ";
        for (const auto& byte : data) {
            ss << std::setw(2) << std::setfill('0') 
               << static_cast<int>(byte) << " ";
        }
        return ss.str();
    }
};

// Add this before the CANBusMonitor class definition
struct BusStatistics {
    uint32_t totalMessages{0};
    uint32_t errorFrames{0};
    uint32_t overflowCount{0};
    float busLoad{0.0f};
    float peakBusLoad{0.0f};
    TimePoint lastUpdate;
    std::map<uint32_t, uint32_t> messageFrequency;
};

class CANBusMonitor {
public:
    CANBusMonitor();
    ~CANBusMonitor() = default;

    // Message handling
    void processMessage(const CANMessage& message);
    std::vector<CANMessage> getRecentMessages() const;
    void clearMessageBuffer();

    // Monitoring and diagnostics
    bool checkBusHealth() const;
    float getBusLoad() const;
    std::vector<std::string> getErrorLog() const;

    // Configuration
    void setMessageFilters(const std::vector<uint32_t>& allowedIds);
    void setBusLoadThreshold(float threshold);
    void enableLogging(bool enable);

    // New methods
    void setMessageExpectedRate(uint32_t id, float messagesPerSecond);
    BusStatistics getStatistics() const;
    void resetStatistics();
    void setMaxBufferSize(size_t size) { maxBufferSize_ = size; }
    void setSamplingWindow(uint32_t windowMs) { samplingWindow_ = windowMs; }
    bool hasTimingViolations() const;
    std::map<uint32_t, float> getMessageRates() const;

private:
    std::queue<CANMessage> messageBuffer_;
    std::vector<uint32_t> allowedMessageIds_;
    std::vector<std::string> errorLog_;

    float busLoadThreshold_{0.8f};  // 80% max bus load
    bool loggingEnabled_{true};
    uint32_t errorCount_{0};

    BusStatistics statistics_;
    std::map<uint32_t, TimePoint> lastMessageTimes_;
    std::map<uint32_t, float> expectedMessageRates_;
    size_t maxBufferSize_{1000};
    uint32_t samplingWindow_{1000}; // ms

    void updateBusStatistics(const CANMessage& message);
    bool validateMessage(const CANMessage& message);
    void logError(const std::string& error);
    void pruneOldMessages();
};

} // namespace diagnostics
} // namespace autocore
