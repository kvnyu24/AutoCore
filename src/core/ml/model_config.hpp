#pragma once

#include <vector>
#include <string>

namespace autocore {
namespace ml {

enum class ModelType {
    ISOLATION_FOREST,
    ONE_CLASS_SVM,
    AUTOENCODER,
    GAUSSIAN_MIXTURE
};

enum class FeatureNormalization {
    NONE,
    MIN_MAX,
    Z_SCORE,
    ROBUST_SCALE
};

struct ModelConfig {
    // Model type and parameters
    ModelType type{ModelType::ISOLATION_FOREST};
    size_t numFeatures{0};
    size_t numEstimators{100};
    float contaminationFactor{0.1f};
    
    // Feature processing
    FeatureNormalization normalization{FeatureNormalization::Z_SCORE};
    std::vector<std::string> featureNames;
    bool enableFeatureSelection{true};
    
    // Training parameters
    size_t batchSize{32};
    size_t maxEpochs{100};
    float learningRate{0.001f};
    float validationSplit{0.2f};
    
    // Anomaly detection
    float anomalyThreshold{0.95f};
    size_t windowSize{50};
    bool useTemporalFeatures{true};
    
    // Model persistence
    std::string modelPath;
    bool enableCheckpointing{true};
    
    // Validation
    bool validateConfig() const {
        return numFeatures > 0 && 
               numEstimators > 0 && 
               contaminationFactor > 0.0f && 
               contaminationFactor < 1.0f &&
               !featureNames.empty() &&
               featureNames.size() == numFeatures;
    }
};

} // namespace ml
} // namespace autocore
