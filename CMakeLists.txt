cmake_minimum_required(VERSION 3.10)
project(Throwbot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Configure these paths to the users Matlab installation folder
# ============================================
set(MATLAB_ROOT "/home/noahv/Programs/MatLab")
set(MATLAB_LIB_DIR "/home/noahv/Programs/MatLab/bin/glnxa64")
# ============================================

# System libraries, needed to avoid conflicts between Matlab Engine and Pylon
find_package(CURL REQUIRED)
find_package(LibXml2 REQUIRED)

# Boost
find_package(Boost REQUIRED COMPONENTS system thread)

# ur_rtde
find_package(ur_rtde REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# Basler Pylon
set(PYLON_ROOT "/opt/pylon")
set(PYLON_INCLUDE_DIR "${PYLON_ROOT}/include")
file(GLOB PYLON_LIBS "${PYLON_ROOT}/lib/*.so")

add_executable(${PROJECT_NAME} main.cpp 
    Vision.h Vision.cpp
    Trajectory.h Trajectory.cpp 
    matlabComm.hpp matlabComm.cpp
    robotControl.h robotControl.cpp
    linearAlgebra.h linearAlgebra.cpp
    Gripper.h Gripper.cpp
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${ur_rtde_INCLUDE_DIRS}
        ${PYLON_INCLUDE_DIR}
        ${MATLAB_ROOT}/extern/lib/glnxa64
        ${MATLAB_ROOT}/extern/include
        ${LIBXML2_INCLUDE_DIR} 
        ${CURL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        # Matlab ICU must load first to include the needed chars
        ${MATLAB_LIB_DIR}/libicuuc.so.74
        ${MATLAB_LIB_DIR}/libicui18n.so.74
        ${MATLAB_LIB_DIR}/libicudata.so.74

        # Load system libs to allow Pylon to use instead of system ICU
        ${CURL_LIBRARIES}
        ${LIBXML2_LIBRARIES}

        # Remaining libs
        ${MATLAB_LIB_DIR}/libmwi18n.so
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ur_rtde::rtde
        ${PYLON_LIBS}
        "${MATLAB_ROOT}/extern/bin/glnxa64/libMatlabEngine.so" 
        "${MATLAB_ROOT}/extern/bin/glnxa64/libMatlabDataArray.so" 
        dl 
        pthread
)

# RPATH SETTINGS
set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${MATLAB_LIB_DIR};${MATLAB_ROOT}/extern/bin/glnxa64;${PYLON_ROOT}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)