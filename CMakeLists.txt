cmake_minimum_required(VERSION 3.10)
project(Throwbot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

add_executable(${PROJECT_NAME} main.cpp Vision.h Vision.cpp)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${ur_rtde_INCLUDE_DIRS}
        ${PYLON_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ur_rtde::rtde
        ${PYLON_LIBS}
)
