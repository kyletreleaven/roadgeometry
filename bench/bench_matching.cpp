#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "roadgeometry/matching.hpp"

using namespace roadgeometry;
using Road   = int64_t;
using Vertex = int64_t;

int main(int argc, char* argv[])
{
    std::string path    = argc > 1 ? argv[1] : "bench/captured_instance.json";
    int         repeats = argc > 2 ? std::stoi(argv[2]) : 3;

    std::ifstream f(path);
    if (!f) { std::cerr << "Cannot open: " << path << "\n"; return 1; }

    nlohmann::json j;
    f >> j;

    int n_roads = static_cast<int>(j["edge_u"].size());

    std::unordered_map<Road, std::pair<Vertex, Vertex>> endpoints;
    std::unordered_map<Road, double>                    lengths;
    std::unordered_map<Road, bool>                      is_oneway;
    endpoints.reserve(n_roads);
    lengths.reserve(n_roads);
    is_oneway.reserve(n_roads);

    for (int i = 0; i < n_roads; ++i) {
        endpoints[i] = { j["edge_u"][i].get<Vertex>(), j["edge_v"][i].get<Vertex>() };
        lengths[i]   = j["edge_length"][i].get<double>();
        is_oneway[i] = j["edge_oneway"][i].get<bool>();
    }

    auto load_pins = [&](const char* road_key, const char* y_key) {
        std::vector<std::pair<Road, double>> pins;
        auto& roads = j[road_key];
        auto& ys    = j[y_key];
        pins.reserve(roads.size());
        for (size_t i = 0; i < roads.size(); ++i)
            pins.push_back({ roads[i].get<Road>(), ys[i].get<double>() });
        return pins;
    };

    auto P = load_pins("supply_road", "supply_y");
    auto Q = load_pins("demand_road", "demand_y");

    std::cout << "roads=" << n_roads
              << "  supply=" << P.size()
              << "  demand=" << Q.size()
              << "  repeats=" << repeats << "\n\n";

    auto bench = [&](const char* name, auto fn) {
        fn();  // warm-up
        double total = 0.0;
        for (int r = 0; r < repeats; ++r) {
            auto t0 = std::chrono::high_resolution_clock::now();
            fn();
            auto t1 = std::chrono::high_resolution_clock::now();
            total += std::chrono::duration<double>(t1 - t0).count();
        }
        std::cout << name << ": " << (total / repeats) << " s avg\n";
    };

    bench("sparse", [&]() {
        compute_matching<Road, Vertex, true>(P, Q, endpoints, lengths, is_oneway);
    });
    bench("dense", [&]() {
        compute_matching<Road, Vertex, false>(P, Q, endpoints, lengths, is_oneway);
    });

    return 0;
}
