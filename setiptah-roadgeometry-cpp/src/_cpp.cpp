#include <pybind11/pybind11.h>
#include "roadgeometry/priority_queue.hpp"

namespace py = pybind11;
using namespace roadgeometry;

// Concrete instantiation for Python: int keys, double priorities.
// This covers the normalized <int, int> road network case.
using IntPriorityQueue = PriorityQueue<int, double>;

// Arbitrary Python hashable keys — for drop-in use as priorityDictionary.
struct PyHash {
    std::size_t operator()(const py::object& obj) const {
        return py::hash(obj);
    }
};

struct PyEqual {
    bool operator()(const py::object& a, const py::object& b) const {
        return a.equal(b);
    }
};

using PyPriorityQueue = PriorityQueue<py::object, double, PyHash, PyEqual>;

PYBIND11_MODULE(_cpp, m) {
    m.doc() = "C++ backend for roadgeometry";

    py::class_<IntPriorityQueue>(m, "PriorityQueue")
        .def(py::init<>())
        .def("push",         &IntPriorityQueue::push)
        .def("peek_min",     &IntPriorityQueue::peek_min)
        .def("pop_min",      &IntPriorityQueue::pop_min)
        .def("contains",     &IntPriorityQueue::contains)
        .def("get",          &IntPriorityQueue::get,
             py::arg("key"),
             py::arg("default_val") = std::numeric_limits<double>::infinity())
        .def("__len__",      &IntPriorityQueue::size)
        .def("__contains__", &IntPriorityQueue::contains);

    // Dict-like interface matching priorityDictionary, for drop-in use in Dijkstra.
    // Supports arbitrary Python hashable keys.
    // Constraint: __delitem__ must always be called on the current minimum key.
    py::class_<PyPriorityQueue>(m, "PriorityDict")
        .def(py::init<>())
        .def("__setitem__", &PyPriorityQueue::push)
        .def("__getitem__", [](const PyPriorityQueue& pq, const py::object& key) -> double {
            if (!pq.contains(key))
                throw py::key_error(py::str(key).cast<std::string>());
            return pq.get(key);
        })
        .def("get",         &PyPriorityQueue::get,
             py::arg("key"),
             py::arg("default_val") = std::numeric_limits<double>::infinity())
        .def("smallest",    &PyPriorityQueue::peek_min)
        .def("__delitem__", [](PyPriorityQueue& pq, const py::object& /*key*/) { pq.pop_min(); })
        .def("__len__",     &PyPriorityQueue::size)
        .def("__contains__", &PyPriorityQueue::contains)
        .def("setdefault",  [](PyPriorityQueue& pq, const py::object& key, double val) -> double {
            if (!pq.contains(key))
                pq.push(key, val);
            return pq.get(key);
        });
}
