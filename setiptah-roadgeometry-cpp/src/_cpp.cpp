#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(_cpp, m) {
    m.doc() = "C++ backend for roadgeometry";
}
