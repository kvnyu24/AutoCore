#include "can_bus_monitor.hpp"
#include <algorithm>
#include <chrono>
#include <sstream>
#include <iomanip>

namespace autocore {
namespace diagnostics {

void CANBusMonitor::updateBusStatistics(const CANMessage& message) {
    auto now = std::chrono::system_clock::now();
    statistics_.totalMessages++;
    
    if (message.isErrorFrame) {
        statistics_.errorFrames++;
    }
    
    // Update message frequency
    statistics_.messageFrequency[message.id]++;
    
    // Calculate bus load
    auto windowStart = now - std::chrono::milliseconds(samplingWindow_);
    float messagesPerSecond = static_cast<float>(statistics_.totalMessages) / 
                             (samplingWindow_ / 1000.0f);
    statistics_.busLoad = messagesPerSecond / 2000.0f; // Assuming 500kbps CAN
    
    statistics_.peakBusLoad = std::max(statistics_.busLoad, statistics_.peakBusLoad);
    statistics_.lastUpdate = now;
    
    // Check message timing
    if (expectedMessageRates_.count(message.id)) {
        auto& lastTime = lastMessageTimes_[message.id];
        if (lastTime != TimePoint{}) {
            auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - lastTime).count();
            float expectedInterval = 1000.0f / expectedMessageRates_[message.id];
            
            if (std::abs(interval - expectedInterval) > expectedInterval * 0.2f) {
                logError("Message timing violation for ID: 0x" + 
                        std::to_string(message.id));
            }
        }
        lastTime = now;
    }
}

void CANBusMonitor::setMessageExpectedRate(uint32_t id, float messagesPerSecond) {
    expectedMessageRates_[id] = messagesPerSecond;
}

BusStatistics CANBusMonitor::getStatistics() const {
    return statistics_;
}

void CANBusMonitor::resetStatistics() {
    statistics_ = BusStatistics{};
}

bool CANBusMonitor::validateMessage(const CANMessage& message) {
    if (!message.isValid()) {
        logError("Invalid message format");
        return false;
    }
    
    if (!allowedMessageIds_.empty() && 
        std::find(allowedMessageIds_.begin(), allowedMessageIds_.end(), 
                 message.id) == allowedMessageIds_.end()) {
        logError("Unauthorized message ID: 0x" + std::to_string(message.id));
        return false;
    }
    
    return true;
}

void CANBusMonitor::pruneOldMessages() {
    auto now = std::chrono::system_clock::now();
    while (!messageBuffer_.empty() && 
           messageBuffer_.size() > maxBufferSize_) {
        messageBuffer_.pop();
        statistics_.overflowCount++;
    }
}

} // namespace diagnostics
} // namespace autocore 