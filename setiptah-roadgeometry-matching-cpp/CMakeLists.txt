cmake_minimum_required(VERSION 3.15)
project(roadgeometry_matching_cpp CXX)

set(CMAKE_CXX_STANDARD 20)
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

option(ROADGEOMETRY_COVERAGE "Enable coverage instrumentation" OFF)
if(ROADGEOMETRY_COVERAGE)
    target_compile_options(_cpp PRIVATE --coverage)
    target_link_options(_cpp PRIVATE --coverage)
endif()

# Install into the setiptah.roadgeometry.matching namespace package
install(TARGETS _cpp DESTINATION setiptah/roadgeometry/matching)

# ---------------------------------------------------------------------------
# Standalone C++ bench (not installed; build with -DROADGEOMETRY_BUILD_BENCH=ON)
# ---------------------------------------------------------------------------
option(ROADGEOMETRY_BUILD_BENCH "Build standalone C++ bench executable" OFF)
if(ROADGEOMETRY_BUILD_BENCH)
    include(FetchContent)
    FetchContent_Declare(nlohmann_json
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
        URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
    )
    set(JSON_BuildTests OFF CACHE INTERNAL "")
    FetchContent_MakeAvailable(nlohmann_json)

    add_executable(bench_matching ${CMAKE_CURRENT_SOURCE_DIR}/../bench/bench_matching.cpp)
    target_link_libraries(bench_matching PRIVATE roadgeometry::roadgeometry nlohmann_json::nlohmann_json)
    target_compile_options(bench_matching PRIVATE -g -fno-inline)
endif()
