#include "astar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace {

struct OpenNode {
    Coord coord;
    double g;
    double f;
};

struct OpenNodeCompare {
    bool operator()(
        const OpenNode& left,
        const OpenNode& right
    ) const {
        // std::priority_queue normally places the largest
        // element first. Reverse the comparison to create
        // a minimum-priority queue based on f.
        if (left.f == right.f) {
            return left.g > right.g;
        }

        return left.f > right.f;
    }
};

std::vector<Coord> reconstructPath(
    const std::unordered_map<Coord, Coord, CoordHash>& came_from,
    const Coord& start,
    const Coord& goal
) {
    std::vector<Coord> path;

    Coord current = goal;
    path.push_back(current);

    while (!(current == start)) {
        const auto previous = came_from.find(current);

        if (previous == came_from.end()) {
            return {};
        }

        current = previous->second;
        path.push_back(current);
    }

    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace

double manhattanDistance(
    const Coord& a,
    const Coord& b
) {
    return static_cast<double>(
        std::abs(a.x - b.x) +
        std::abs(a.y - b.y)
    );
}

std::optional<std::vector<Coord>> aStar(
    const Grid& grid,
    const Coord& start,
    const Coord& goal
) {
    // Reject coordinates outside the grid.
    if (!grid.inBounds(start) || !grid.inBounds(goal)) {
        return std::nullopt;
    }

    // Reject occupied start or goal cells.
    if (
        std::isinf(grid.traversalCost(start)) ||
        std::isinf(grid.traversalCost(goal))
    ) {
        return std::nullopt;
    }

    // The start is already the goal.
    if (start == goal) {
        return std::vector<Coord>{start};
    }

    std::priority_queue<
        OpenNode,
        std::vector<OpenNode>,
        OpenNodeCompare
    > open;

    std::unordered_map<Coord, double, CoordHash> g_score;
    std::unordered_map<Coord, Coord, CoordHash> came_from;

    g_score[start] = 0.0;

    open.push(
        OpenNode{
            start,
            0.0,
            manhattanDistance(start, goal)
        }
    );

    while (!open.empty()) {
        const OpenNode current = open.top();
        open.pop();

        const auto current_score = g_score.find(current.coord);

        if (current_score == g_score.end()) {
            continue;
        }

        // Ignore old queue entries whose g value is no
        // longer the best known value.
        if (current.g > current_score->second) {
            continue;
        }

        if (current.coord == goal) {
            std::vector<Coord> path = reconstructPath(
                came_from,
                start,
                goal
            );

            if (path.empty()) {
                return std::nullopt;
            }

            return path;
        }

        for (
            const Coord& neighbor :
            grid.neighbors(current.coord)
        ) {
            const double move_cost =
                grid.traversalCost(neighbor);

            // Do not enter occupied cells.
            if (std::isinf(move_cost)) {
                continue;
            }

            const double tentative_g =
                current.g + move_cost;

            const auto known_score =
                g_score.find(neighbor);

            const bool is_better_path =
                known_score == g_score.end() ||
                tentative_g < known_score->second;

            if (!is_better_path) {
                continue;
            }

            came_from[neighbor] = current.coord;
            g_score[neighbor] = tentative_g;

            const double estimated_total =
                tentative_g +
                manhattanDistance(neighbor, goal);

            open.push(
                OpenNode{
                    neighbor,
                    tentative_g,
                    estimated_total
                }
            );
        }
    }

    return std::nullopt;
}

double totalPathCost(
    const Grid& grid,
    const std::vector<Coord>& path
) {
    if (path.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    // Check the first cell separately because its traversal
    // cost is not included in the total.
    if (
        !grid.inBounds(path.front()) ||
        std::isinf(grid.traversalCost(path.front()))
    ) {
        return std::numeric_limits<double>::infinity();
    }

    double total_cost = 0.0;

    for (std::size_t i = 1; i < path.size(); ++i) {
        const Coord& previous = path[i - 1];
        const Coord& current = path[i];

        if (!grid.inBounds(current)) {
            return std::numeric_limits<double>::infinity();
        }

        // Four-connected coordinates must be exactly one
        // Manhattan-distance unit apart.
        if (manhattanDistance(previous, current) != 1.0) {
            return std::numeric_limits<double>::infinity();
        }

        const double step_cost =
            grid.traversalCost(current);

        if (std::isinf(step_cost)) {
            return std::numeric_limits<double>::infinity();
        }

        total_cost += step_cost;
    }

    return total_cost;
}

bool validatePath(
    const Grid& grid,
    const std::vector<Coord>& path,
    const Coord& start,
    const Coord& goal
) {
    if (path.empty()) {
        return false;
    }

    if (!(path.front() == start)) {
        return false;
    }

    if (!(path.back() == goal)) {
        return false;
    }

    for (const Coord& cell : path) {
        if (!grid.inBounds(cell)) {
            return false;
        }

        if (std::isinf(grid.traversalCost(cell))) {
            return false;
        }
    }

    for (std::size_t i = 1; i < path.size(); ++i) {
        if (
            manhattanDistance(
                path[i - 1],
                path[i]
            ) != 1.0
        ) {
            return false;
        }
    }

    return true;
}