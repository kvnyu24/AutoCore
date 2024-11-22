#pragma once

#include "../../common/types.hpp"
#include "../autonomous_types.hpp"
#include "../../sensors/sensor_types.hpp"
#include <vector>

namespace autocore {
namespace autonomous {

using common::Position;

class AStar {
public:
    std::vector<Position> findPath(
        const Position& start,
        const Position& goal,
        const std::vector<Obstacle>& obstacles);
        
private:
    // A* implementation details
};

} // namespace autonomous
} // namespace autocore 