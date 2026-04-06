#include <pybind11/pybind11.h>
#include "roadgeometry/priority_queue.hpp"

namespace py = pybind11;
using namespace roadgeometry;

// Concrete instantiation for Python: int keys, double priorities.
// This covers the normalized <int, int> road network case.
using IntPriorityQueue = PriorityQueue<int, double>;

PYBIND11_MODULE(_cpp, m) {
    m.doc() = "C++ backend for roadgeometry";

    py::class_<IntPriorityQueue>(m, "PriorityQueue")
        .def(py::init<>())
        .def("push",      &IntPriorityQueue::push)
        .def("peek_min",  &IntPriorityQueue::peek_min)
        .def("pop_min",   &IntPriorityQueue::pop_min)
        .def("contains",  &IntPriorityQueue::contains)
        .def("get",       &IntPriorityQueue::get,
             py::arg("key"),
             py::arg("default_val") = std::numeric_limits<double>::infinity())
        .def("__len__",   &IntPriorityQueue::size)
        .def("__contains__", &IntPriorityQueue::contains);
}
