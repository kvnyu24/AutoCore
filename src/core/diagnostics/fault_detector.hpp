#pragma once

#include <memory>
#include <vector>
#include "diagnostic_data.hpp"
#include "../ml/anomaly_detector.hpp"
#include "../common/types.hpp"

namespace autocore {
namespace diagnostics {

using common::DiagnosticData;

enum class FaultType {
    TEMPERATURE_WARNING,
    VOLTAGE_WARNING,
    CURRENT_WARNING,
    ANOMALY_DETECTED,
    COMMUNICATION_ERROR,
    SENSOR_ERROR
};

enum class FaultSeverity {
    LOW,
    MEDIUM,
    HIGH,
    CRITICAL
};

struct Fault {
    FaultType type;
    std::string description;
    std::chrono::system_clock::time_point timestamp;
    FaultSeverity severity;

    bool operator<(const Fault& other) const {
        return timestamp < other.timestamp;
    }
    
    bool operator==(const Fault& other) const {
        return type == other.type && 
               timestamp == other.timestamp &&
               severity == other.severity &&
               description == other.description;
    }
};

struct FaultPattern {
    std::vector<FaultType> sequence;
    std::chrono::milliseconds timeWindow;
    FaultSeverity resultingSeverity;
    std::string description;
};

struct TemporalFaultAnalysis {
    std::deque<Fault> recentFaults;
    std::chrono::seconds analysisWindow{3600}; // 1 hour default
    size_t maxFaultHistory{100};
    
    struct PatternMatch {
        FaultPattern pattern;
        std::chrono::system_clock::time_point firstOccurrence;
        int occurrences{0};
    };
    std::map<std::string, PatternMatch> activePatterns;
};

class FaultDetector {
public:
    FaultDetector();
    ~FaultDetector() = default;

    // Fault detection methods
    std::vector<Fault> detectFaults(const DiagnosticData& data);
    void detectRuleBasedFaults(const DiagnosticData& data, std::vector<Fault>& faults);
    void detectAnomalies(const DiagnosticData& data, std::vector<Fault>& faults);
    void detectPatternBasedFaults(const DiagnosticData& data, std::vector<Fault>& faults);

    // Configuration
    void setThresholds(float tempThreshold, float voltageMin, float voltageMax);
    void updateFaultPatterns(const std::vector<FaultPattern>& patterns);

private:
    std::unique_ptr<ml::AnomalyDetector> mlModel_;
    std::vector<FaultPattern> faultPatterns_;
    
    // Thresholds
    float TEMP_THRESHOLD{80.0f};
    float VOLTAGE_MIN_THRESHOLD{280.0f};
    float VOLTAGE_MAX_THRESHOLD{400.0f};

    TemporalFaultAnalysis temporalAnalysis_;
    
    void initializeFaultPatterns();
    void loadMachineLearningModel();
    void processFaults(std::vector<Fault>& faults);
    void analyzeTemporalPatterns(const DiagnosticData& data);
    void escalateSeverity(std::vector<Fault>& faults);
    bool matchesTemporalPattern(const FaultType& faultType, const FaultPattern& pattern);
};

} // namespace diagnostics
} // namespace autocore
