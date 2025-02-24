cmake_minimum_required(VERSION 3.10)

# VCPK cmake integration
set(VCPKG_VERSION edge) # optional
include(libs/vcpkg-cmake-integration/vcpkg.cmake)

# Define project
project(supervoxel_segmentation)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


####################################################################################################
# Setup dependencies

# argh
vcpkg_add_package(argh)
find_package(argh CONFIG REQUIRED)

# pcl
vcpkg_add_package(pcl)
find_package(PCL REQUIRED COMPONENTS
    common
    io
    kdtree
    search
    features
)

# omp
find_package(OpenMP REQUIRED)


####################################################################################################
# Add source files
set(SOURCES
    main.cc
)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Add compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -O2
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    pthread
    argh
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
)

# Install
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)