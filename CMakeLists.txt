cmake_minimum_required(VERSION 3.10)
project(Throwbot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set to local MatLab Roor folder
set(MATLAB_ROOT /home/noahv/Programs/MatLab)

# Boost (needed by ur_rtde)
find_package(Boost REQUIRED COMPONENTS system thread)

# ur_rtde
find_package(ur_rtde REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# Basler Pylon
set(PYLON_ROOT "/opt/pylon")
set(PYLON_INCLUDE_DIR "${PYLON_ROOT}/include")
file(GLOB PYLON_LIBS "${PYLON_ROOT}/lib/*.so")

add_executable(${PROJECT_NAME} main.cpp Vision.h Vision.cpp
    Trajectory.h Trajectory.cpp matlabComm.hpp matlabComm.cpp)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${ur_rtde_INCLUDE_DIRS}
        ${PYLON_INCLUDE_DIR}
        ${MATLAB_ROOT}/extern/lib/glnxa64
        ${MATLAB_ROOT}/extern/include
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ur_rtde::rtde
        ${PYLON_LIBS}
        ${OpenCV_LIBS} 
        "${MATLAB_ROOT}/extern/bin/glnxa64/libMatlabEngine.so" 
        "${MATLAB_ROOT}/extern/bin/glnxa64/libMatlabDataArray.so" dl pthread
)

set_target_properties(run PROPERTIES
    BUILD_WITH_INSTALL_RPATH true
    INSTALL_RPATH "${MATLAB_ROOT}/extern/bin/glnxa64"
)
