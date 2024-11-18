# AutoCore - Electric Vehicle Control System Library

AutoCore is a comprehensive C++ library for electric vehicle control systems, providing autonomous driving capabilities, ADAS features, and core vehicle management.

## 🚀 Features

- **Autonomous Driving System**
  - Behavior Planning
  - Trajectory Generation
  - Scene Understanding
  - Safety Monitoring

- **Advanced Driver Assistance (ADAS)**
  - Adaptive Cruise Control
  - Emergency Braking
  - Lane Keeping Assist
  - Collision Avoidance

- **Vehicle Control**
  - Motor Control & Optimization
  - Battery Management
  - Power Distribution
  - Regenerative Braking

- **Diagnostics & Monitoring**
  - Real-time Fault Detection
  - Predictive Maintenance
  - CAN Bus Integration
  - Performance Analytics

- **Autonomous System**
  - Manages autonomous driving capabilities
  - Key features:

## 🛠 Getting Started

### Prerequisites

- CMake 3.15+
- C++17 compatible compiler
- Ninja build system
- Google Test framework

### Building the Project

#### Clone the repository

```bash
git clone https://github.com/kvnyu24/autocore.git
cd autocore
```

#### Run the setup script

```bash
./scripts/setup.sh
```

#### Or build manually

```bash
mkdir build && cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Debug
ninja
```

### Our test suite covers all major components

```bash
cd build
ctest --output-on-failure
```

## 📚 Architecture

### Core Components

1. **Vehicle Control System (VCS)**
   - Central control unit managing vehicle operations
   - Reference implementation:
     - [VehicleControlSystem.hpp](src/core/vcs/vehicle_control_system.hpp)

2. **ADAS Manager**
   - Handles advanced driver assistance features
   - Example usage:
     - [ADASManager.hpp](src/core/adas/adas_manager.hpp)
     - [AdaptiveCruiseControl.hpp](src/core/adas/adaptive_cruise_control.hpp)
     - [EmergencyBraking.hpp](src/core/adas/emergency_braking.hpp)
     - [LaneKeepingAssist.hpp](src/core/adas/lane_keeping_assist.hpp)
     - [CollisionAvoidance.hpp](src/core/adas/collision_avoidance.hpp)

3. **Autonomous System**
   - Manages autonomous driving capabilities
   - Key features:
     - [AutonomousManager.hpp](src/core/autonomous/autonomous_manager.hpp)
     - [BehaviorPlanner.hpp](src/core/autonomous/behavior/behavior_planner.hpp)
     - [TrajectoryPlanner.hpp](src/core/autonomous/planning/trajectory_planner.hpp)

4. **Sensor Integration**
   - Manages sensor fusion and data processing
   - Implementation example:
     - [FusionEngine.hpp](src/core/sensors/fusion_engine.hpp)
     - [LocalizationSystem.hpp](src/core/localization/localization_system.hpp)
     - [Perception.hpp](src/core/perception/perception.hpp)
     - [Prediction.hpp](src/core/prediction/prediction.hpp)
     - [Map.hpp](src/core/map/map.hpp)

## 🔧 Development

### Code Structure

```bash
src/
├── core/
│   ├── autonomous/
│   ├── adas/
│   ├── vcs/
│   ├── sensors/
│   └── diagnostics/
├── tests/
└── utils/
```

### Adding New Features

1. Create test file in `src/tests/`
2. Implement feature in corresponding module
3. Update documentation
4. Submit PR for review

### Testing Guidelines

- Unit tests required for all new features
- Integration tests for component interaction
- Performance benchmarks for critical paths

## 🔒 Safety & Compliance

Our codebase prioritizes safety through:

- Comprehensive testing (see test examples)
- Fault detection and handling
- Redundancy in critical systems
- Real-time monitoring

## 📈 Performance

Key benchmarks:

- Control loop frequency: 1kHz
- Sensor fusion latency: <10ms
- Emergency response time: <50ms

## 🤝 Contributing

1. Fork the repository
2. Create feature branch
3. Add tests and implementation
4. Submit pull request

## 📄 License

This project is licensed under the MIT License - see LICENSE file for details.

## 🙏 Acknowledgments

- Contributors and maintainers
- Open source libraries and tools
- Research papers and references

## 📞 Support

- GitHub Issues for bug reports
- Discussions for feature requests
- Wiki for documentation
