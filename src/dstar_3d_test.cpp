#include "dstar_3d.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

bool isSixConnected(const dstar_3d::Coord& first, const dstar_3d::Coord& second) {
    return std::abs(first.x - second.x) +
               std::abs(first.y - second.y) +
               std::abs(first.z - second.z) ==
           1;
}

void assertValidPath(
    const dstar_3d::Grid& grid,
    const std::vector<dstar_3d::Coord>& path,
    const dstar_3d::Coord& start,
    const dstar_3d::Coord& goal) {
    assert(!path.empty());
    assert(path.front() == start);
    assert(path.back() == goal);

    for (std::size_t index = 0; index < path.size(); ++index) {
        assert(grid.inBounds(path[index]));
        assert(std::isfinite(grid.traversalCost(path[index])));
        if (index > 0) {
            assert(isSixConnected(path[index - 1], path[index]));
        }
    }
}

} // namespace

int main() {
    using dstar_3d::Coord;
    using dstar_3d::DStarLite3D;
    using dstar_3d::Grid;

    Grid grid(5, 4, 3, 0.25);
    const Coord start{0, 1, 1};
    const Coord goal{4, 1, 1};

    // A corner has only three valid neighbours, demonstrating that no diagonal
    // transitions are generated in the 3D planner.
    assert(grid.neighbors(Coord{0, 0, 0}).size() == 3);
    assert(grid.neighbors(Coord{2, 2, 1}).size() == 6U);

    DStarLite3D planner(grid, start, goal);
    planner.computeShortestPath();
    const std::vector<Coord> direct_path = planner.extractPath();

    assertValidPath(grid, direct_path, start, goal);
    // Four x-steps. A diagonal transition is not available.
    assert(direct_path.size() == 5U);
    assert(std::abs(planner.g(start) - 4.0) <= dstar_3d::kEpsilon);

    const Coord obstacle = direct_path[2];
    planner.updateCellState(obstacle, Grid::kOccupied);
    planner.computeShortestPath();
    const std::vector<Coord> repaired_path = planner.extractPath();

    assertValidPath(grid, repaired_path, start, goal);
    for (const Coord& voxel : repaired_path) {
        assert(!(voxel == obstacle));
    }
    assert(repaired_path.size() == 7U);
    assert(std::abs(planner.g(start) - 6.0) <= dstar_3d::kEpsilon);

    std::cout << "D* Lite 3D 6-connected tests passed.\n";
    return 0;
}
