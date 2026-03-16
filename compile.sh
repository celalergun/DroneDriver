if [ -d "build" ]; then
    echo "Removing existing build directory..."
    rm -rf build
fi
# Activate the virtual environment only if it exists
# Please note that I am using Python 3.14 but my Ubuntu 24.04 has Python 3.12
# Instead of downloading required libraries like python3-xyz, 
# I have created a virtual environment and MAVLink script installed the required libraries there
if [ -d "../venv" ]; then
    echo "Activating virtual environment..."
    source ../venv/bin/activate
fi
echo "Creating build directory and building the project..."
mkdir build && cd build
cmake ..
cmake --build . -j$(nproc)
