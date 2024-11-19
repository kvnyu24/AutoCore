#pragma once

#include <vector>
#include <memory>
#include <string>
#include "../../common/types.hpp"
namespace autocore {
namespace sensors {

struct MapPoint {
    Position position;
    float confidence;
    std::vector<size_t> observations;  // Indices of observations that saw this point
};

struct MapFeature {
    std::vector<MapPoint> points;
    std::string type;  // e.g., "wall", "landmark", etc.
    float uncertainty;
};

class Map {
private:
    std::vector<MapFeature> features_;
    std::vector<Position> trajectoryHistory_;
    
    // Internal methods
    void pruneOldFeatures();
    void updateUncertainties();

public:
    Map() = default;
    ~Map() = default;

    // Core functionality
    void addFeature(const MapFeature& feature);
    void updateFeature(size_t featureId, const MapFeature& feature);
    void removeFeature(size_t featureId);
    
    // Query methods
    std::vector<MapFeature> getFeaturesInRange(const Position& center, float radius) const;
    bool isInitialized() const { return !features_.empty(); }
    
    // Map management
    void clear();
    void optimize();
    void merge(const Map& other);
};

} // namespace sensors
} // namespace autocore