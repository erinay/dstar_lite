#include "astar.hpp"
#include "grid.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

void testEmptyGrid() {
    Grid grid(10, 10, 0.1);

    const Coord start{0, 0};
    const Coord goal{9, 9};

    const auto path = aStar(grid, start, goal);

    assert(path.has_value());
    assert(validatePath(grid, *path, start, goal));

    // Manhattan distance from (0,0) to (9,9):
    // 9 horizontal moves + 9 vertical moves = 18.
    assert(totalPathCost(grid, *path) == 18.0);

    std::cout
        << "Empty grid test passed. Cost = "
        << totalPathCost(grid, *path)
        << '\n';
}

void testWallWithOpening() {
    Grid grid(7, 7, 0.1);

    const Coord start{1, 1};
    const Coord goal{5, 1};

    // Vertical wall at x = 3.
    // The only opening is at (3, 6).
    for (int y = 0; y < 6; ++y) {
        grid.setState({3, y}, 1);
    }

    const auto path = aStar(grid, start, goal);

    assert(path.has_value());
    assert(validatePath(grid, *path, start, goal));

    // The robot must travel down to y = 6,
    // cross the wall opening, then travel back up.
    //
    // Down:       5
    // Across:     4
    // Back up:    5
    // Total:     14
    assert(totalPathCost(grid, *path) == 14.0);

    std::cout
        << "Wall-with-opening test passed. Cost = "
        << totalPathCost(grid, *path)
        << '\n';

    grid.print(start, goal, *path);
}

void testCompletelyBlockedGoal() {
    Grid grid(5, 5, 0.1);

    const Coord start{0, 0};
    const Coord goal{2, 2};

    // Block all four cells surrounding the goal.
    grid.setState({1, 2}, 1);
    grid.setState({3, 2}, 1);
    grid.setState({2, 1}, 1);
    grid.setState({2, 3}, 1);

    const auto path = aStar(grid, start, goal);

    assert(!path.has_value());

    std::cout
        << "Blocked-goal test passed: no path found.\n";
}

void testStartEqualsGoal() {
    Grid grid(5, 5, 0.1);

    const Coord start{2, 2};
    const Coord goal{2, 2};

    const auto path = aStar(grid, start, goal);

    assert(path.has_value());
    assert(path->size() == 1);
    assert(validatePath(grid, *path, start, goal));
    assert(totalPathCost(grid, *path) == 0.0);

    std::cout
        << "Start-equals-goal test passed. Cost = "
        << totalPathCost(grid, *path)
        << '\n';
}

int main() {
    testEmptyGrid();
    testWallWithOpening();
    testCompletelyBlockedGoal();
    testStartEqualsGoal();

    std::cout << "\nAll A* tests passed.\n";

    return 0;
}