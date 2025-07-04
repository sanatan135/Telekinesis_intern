#!/bin/bash

echo "Building H1 WBC Controller..."

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo "Configuring with CMake..."
cmake .. || {
    echo "CMake configuration failed!"
    exit 1
}

# Build
echo "Building..."
make -j$(nproc) || {
    echo "Build failed!"
    exit 1
}

echo "Build completed successfully!"
echo ""
echo "Run the demo with: ./h1_wbc_demo"
echo "Run simple test with: ./simple_test"
