#pragma once

#include "../../common/types.hpp"
#include "../types.hpp"
#include <vector>

namespace autocore {
namespace autonomous {

class AStar {
public:
    std::vector<sensors::Position> findPath(
        const sensors::Position& start,
        const sensors::Position& goal,
        const std::vector<Obstacle>& obstacles);
        
private:
    // A* implementation details
};

} // namespace autonomous
} // namespace autocore 