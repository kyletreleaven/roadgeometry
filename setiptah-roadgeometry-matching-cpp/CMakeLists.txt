cmake_minimum_required(VERSION 3.15)
project(roadgeometry_matching_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pybind11 CONFIG REQUIRED)

# C++ header-only library (from sibling cpp/ directory)
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../cpp
    ${CMAKE_CURRENT_BINARY_DIR}/roadgeometry_cpp
)

# Python extension module
pybind11_add_module(_cpp MODULE src/_cpp.cpp)
target_link_libraries(_cpp PRIVATE roadgeometry::roadgeometry)

# Install into the setiptah.roadgeometry.matching namespace package
install(TARGETS _cpp DESTINATION setiptah/roadgeometry/matching)
