#pragma once

#include <memory>
#include <vector>
#include "../common/types.hpp"
#include "model_config.hpp"

namespace autocore {
namespace ml {

using common::DiagnosticData;

class AnomalyDetector {
public:
    AnomalyDetector();
    ~AnomalyDetector() = default;

    // Core prediction methods
    bool predictAnomaly(const DiagnosticData& data);
    float getAnomalyScore(const DiagnosticData& data) const;
    
    // Model management
    void trainModel(const std::vector<DiagnosticData>& trainingData);
    void updateModel(const DiagnosticData& newData);
    void loadModel(const std::string& modelPath);
    void saveModel(const std::string& modelPath) const;

    // Configuration
    void setThreshold(float threshold) {
        if (threshold < 0.0f || threshold > 1.0f) {
            throw std::invalid_argument("Threshold must be between 0.0 and 1.0");
        }
        anomalyThreshold_ = threshold;
    }

    void setModelParameters(const ModelConfig& config) {
        if (!validateModelConfig(config)) {
            throw std::invalid_argument("Invalid model configuration");
        }
        modelConfig_ = config;
    }
    
    // Feature extraction
    std::vector<float> extractFeatures(const DiagnosticData& data) const;
    void normalizeFeatures(std::vector<float>& features) const;

    // Getters
    float getThreshold() const { return anomalyThreshold_; }
    const ModelConfig& getModelConfig() const { return modelConfig_; }

private:
    float anomalyThreshold_{0.8f};
    ModelConfig modelConfig_;
    std::vector<float> modelWeights_;
    std::vector<float> featureMeans_;
    std::vector<float> featureStdDevs_;

    // Internal methods
    bool validateData(const DiagnosticData& data) const;
    void updateFeatureStatistics(const std::vector<float>& features);
    float calculateMahalanobisDistance(const std::vector<float>& features) const;
    bool validateModelConfig(const ModelConfig& config) const;

    // Feature extraction helpers
    std::vector<float> extractTemperatureFeatures(const DiagnosticData& data) const;
    std::vector<float> extractVoltageFeatures(const DiagnosticData& data) const;
    std::vector<float> extractPerformanceFeatures(const DiagnosticData& data) const;
    std::vector<float> extractTemporalFeatures(const DiagnosticData& data) const;
};

} // namespace ml
} // namespace autocore 