#pragma once
#include <unordered_map>

namespace roadgeometry {

// Primal-dual result of a min-cost-flow solve: the flow plus the certifying
// potentials the engine maintained. Potentials are computed regardless, so
// returning them is free — they are the optimality certificate and the SSP
// warm-start memo. Shared by the dense and sparse engines.
template <class Edge, class Node>
struct FlowPotential {
    std::unordered_map<Edge, double> flow;
    std::unordered_map<Node, double> potential;
};

} // namespace roadgeometry
