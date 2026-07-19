# Native C++ tests (doctest + CTest). Built only when ROADGEOMETRY_BUILD_TESTS=ON,
# so the pybind11 / scikit-build path is unaffected. See /testing.md.

add_executable(roadgeometry_tests
    test_fragile_mccf.cpp
)

target_link_libraries(roadgeometry_tests PRIVATE roadgeometry::roadgeometry)
# For "third_party/doctest/doctest.h" and "support.hpp" (relative to this dir).
target_include_directories(roadgeometry_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(roadgeometry_tests PRIVATE cxx_std_20)

add_test(NAME roadgeometry_tests COMMAND roadgeometry_tests)
