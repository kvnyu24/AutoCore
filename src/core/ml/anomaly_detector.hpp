#pragma once

#include <memory>
#include <vector>
#include "../diagnostics/diagnostic_data.hpp"
#include "model_config.hpp"

namespace autocore {
namespace ml {

using diagnostics::DiagnosticData;

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
    void setThreshold(float threshold) { anomalyThreshold_ = threshold; }
    void setModelParameters(const ModelConfig& config) { modelConfig_ = config; }
    
    // Feature extraction
    std::vector<float> extractFeatures(const DiagnosticData& data) const;
    void normalizeFeatures(std::vector<float>& features) const;

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
};

} // namespace ml
} // namespace autocore 