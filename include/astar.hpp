#include "grid.hpp"

#include <optional>
#include <vector>

// Four-connected Manhattan-distance heuristic.
double manhattanDistance(const Coord& a, const Coord& b);

// Returns a path including both start and goal.
// Returns std::nullopt when no path exists.
std::optional<std::vector<Coord>> aStar(
    const Grid& grid,
    const Coord& start,
    const Coord& goal
);

// Returns infinity when the path is invalid.
// The starting cell itself has no traversal cost.
double totalPathCost(
    const Grid& grid,
    const std::vector<Coord>& path
);

// Checks:
// 1. Path starts at start.
// 2. Path ends at goal.
// 3. Consecutive cells are four-connected neighbors.
// 4. Every cell is in bounds.
// 5. No occupied cell appears in the path.
bool validatePath(
    const Grid& grid,
    const std::vector<Coord>& path,
    const Coord& start,
    const Coord& goal
);