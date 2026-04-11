cmake_minimum_required(VERSION 3.15)
project(roadgeometry CXX)

# Header-only library target.
# C++ users: add_subdirectory() or FetchContent, then link against roadgeometry::roadgeometry.
add_library(roadgeometry INTERFACE)
add_library(roadgeometry::roadgeometry ALIAS roadgeometry)

target_include_directories(roadgeometry INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(roadgeometry INTERFACE cxx_std_20)

# Install rules (for find_package support)
install(TARGETS roadgeometry EXPORT roadgeometryTargets)
install(DIRECTORY include/ DESTINATION include)
install(EXPORT roadgeometryTargets
    FILE roadgeometryTargets.cmake
    NAMESPACE roadgeometry::
    DESTINATION lib/cmake/roadgeometry
)
