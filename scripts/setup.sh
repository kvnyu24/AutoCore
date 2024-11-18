#!/bin/bash

# Exit on error
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

echo "Setting up AutoCore Library development environment..."

# Check for required tools
check_dependency() {
    if ! command -v $1 &> /dev/null; then
        echo -e "${RED}Error: $1 is required but not installed.${NC}"
        exit 1
    fi
}

check_dependency cmake
check_dependency git
check_dependency clang++
check_dependency ninja

# Create build directory
mkdir -p build
cd build

# Configure dependencies
configure_dependencies() {
    echo "Configuring dependencies..."
    
    # Clone and build third-party libraries if not present
    if [ ! -d "external" ]; then
        mkdir external
        cd external
        
        # Eigen for mathematical operations
        git clone --depth 1 https://gitlab.com/libeigen/eigen.git
        
        # Google Test for unit testing
        git clone --depth 1 https://github.com/google/googletest.git
        
        # Crypto++ for security features
        git clone --depth 1 https://github.com/weidai11/cryptopp.git
        
        cd ..
    fi
}

# Configure CMake
configure_cmake() {
    echo "Configuring CMake..."
    cmake .. \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Debug \
        -DBUILD_TESTING=ON \
        -DENABLE_COVERAGE=ON \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
}

# Build the project
build_project() {
    echo "Building project..."
    ninja
}

# Run tests
run_tests() {
    echo "Running tests..."
    ctest --output-on-failure
}

# Setup git hooks
setup_git_hooks() {
    echo "Setting up git hooks..."
    cp ../scripts/pre-commit ../.git/hooks/
    chmod +x ../.git/hooks/pre-commit
}

# Main setup sequence
main() {
    configure_dependencies
    configure_cmake
    build_project
    run_tests
    setup_git_hooks
    
    echo -e "${GREEN}Setup completed successfully!${NC}"
    echo "You can now start developing with the AutoCore Library."
}

main 