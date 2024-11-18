#pragma once

#include <memory>
#include <queue>
#include <vector>
#include "diagnostic_data.hpp"

namespace autocore {
namespace diagnostics {

struct CANMessage {
    uint32_t id;
    std::vector<uint8_t> data;
    std::chrono::system_clock::time_point timestamp;
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

private:
    std::queue<CANMessage> messageBuffer_;
    std::vector<uint32_t> allowedMessageIds_;
    std::vector<std::string> errorLog_;

    float busLoadThreshold_{0.8f};  // 80% max bus load
    bool loggingEnabled_{true};
    uint32_t errorCount_{0};

    void updateBusStatistics(const CANMessage& message);
    bool validateMessage(const CANMessage& message);
    void logError(const std::string& error);
    void pruneOldMessages();
};

} // namespace diagnostics
} // namespace autocore
