# Native C++ tests (doctest + CTest). Built only when ROADGEOMETRY_BUILD_TESTS=ON,
# so the pybind11 / scikit-build path is unaffected. See /testing.md.
#
# doctest_main.cpp owns the single main(); every other test_*.cpp just #includes
# the doctest header and defines TEST_CASEs.

function(roadgeometry_add_test name)
    add_executable(${name} doctest_main.cpp ${ARGN})
    target_link_libraries(${name} PRIVATE roadgeometry::roadgeometry)
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_features(${name} PRIVATE cxx_std_20)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

# Green suite.
roadgeometry_add_test(roadgeometry_tests test_fragile_mccf.cpp test_instance.cpp)

# North-star acceptance target: currently RED (references not-yet-built headers).
# Kept separate so the green suite stays buildable/runnable while we drive this
# toward green. Build just the green suite with:
#   cmake --build <dir> --target roadgeometry_tests
roadgeometry_add_test(roadgeometry_northstar test_solver_state.cpp)
