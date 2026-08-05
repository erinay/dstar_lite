#include "dstar_lite.hpp"
#include "grid.hpp"
#include "astar.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <optional>
#include <vector>

int main() {
    // ============================================================
    // Phase 10: Compare static D* Lite results against A*
    // ============================================================
    {
        Grid grid(10, 10, 0.1);

        const Coord start{0, 0};
        const Coord goal{9, 9};

        // Create a wall with one opening.
        for (int y = 0; y < grid.height(); ++y) {
            if (y == 7) {
                continue;
            }

            grid.setState(Coord{4, y}, 1);
        }

        const std::optional<std::vector<Coord>> astar_result =
            aStar(grid, start, goal);

        assert(astar_result.has_value());

        const std::vector<Coord>& astar_path =
            astar_result.value();

        assert(
            validatePath(
                grid,
                astar_path,
                start,
                goal
            )
        );

        const double astar_cost =
            totalPathCost(grid, astar_path);

        DStarLite planner(grid, start, goal);

        planner.computeShortestPath();

        const std::vector<Coord> dstar_path =
            planner.extractPath();

        assert(!dstar_path.empty());

        assert(
            validatePath(
                grid,
                dstar_path,
                start,
                goal
            )
        );

        const double dstar_cost =
            totalPathCost(grid, dstar_path);

        assert(
            std::abs(dstar_cost - astar_cost) <= EPS
        );

        std::cout
            << "Phase 10 comparison passed.\n"
            << "A* cost:      " << astar_cost << '\n'
            << "D* Lite cost: " << dstar_cost << '\n';
    }

    // ============================================================
    // Phase 11: Incremental edge-cost changes
    // ============================================================

    // ------------------------------------------------------------
    // Test 1: An obstacle appears on the current path
    // ------------------------------------------------------------
    {
        Grid grid(10, 5, 0.1);

        const Coord start{0, 2};
        const Coord goal{9, 2};

        DStarLite planner(grid, start, goal);

        planner.computeShortestPath();

        const std::vector<Coord> original_path =
            planner.extractPath();

        assert(!original_path.empty());

        assert(
            validatePath(
                grid,
                original_path,
                start,
                goal
            )
        );

        const double original_cost =
            totalPathCost(grid, original_path);

        assert(
            std::abs(original_cost - 9.0) <= EPS
        );

        const std::size_t obstacle_index =
            original_path.size() / 2;

        const Coord new_obstacle =
            original_path[obstacle_index];

        assert(!(new_obstacle == start));
        assert(!(new_obstacle == goal));

        planner.updateCellState(
            new_obstacle,
            1
        );

        planner.computeShortestPath();

        const std::vector<Coord> repaired_path =
            planner.extractPath();

        assert(!repaired_path.empty());

        assert(
            validatePath(
                grid,
                repaired_path,
                start,
                goal
            )
        );

        const bool obstacle_in_repaired_path =
            std::find(
                repaired_path.begin(),
                repaired_path.end(),
                new_obstacle
            ) != repaired_path.end();

        assert(!obstacle_in_repaired_path);

        const double repaired_cost =
            totalPathCost(grid, repaired_path);

        assert(
            std::abs(repaired_cost - 11.0) <= EPS
        );

        assert(repaired_cost > original_cost);

        const std::optional<std::vector<Coord>> astar_result =
            aStar(grid, start, goal);

        assert(astar_result.has_value());

        const std::vector<Coord>& astar_path =
            astar_result.value();

        assert(
            validatePath(
                grid,
                astar_path,
                start,
                goal
            )
        );

        const double astar_cost =
            totalPathCost(grid, astar_path);

        assert(
            std::abs(repaired_cost - astar_cost) <= EPS
        );

        std::cout
            << "Phase 11 Test 1 passed: obstacle appeared.\n"
            << "Blocked cell: ("
            << new_obstacle.x
            << ", "
            << new_obstacle.y
            << ")\n"
            << "Original cost: "
            << original_cost
            << '\n'
            << "Repaired D* Lite cost: "
            << repaired_cost
            << '\n'
            << "Fresh A* cost: "
            << astar_cost
            << '\n';

        grid.print(
            start,
            goal,
            repaired_path
        );
    }

    // ------------------------------------------------------------
    // Test 2: An obstacle disappears
    // ------------------------------------------------------------
    {
        Grid grid(7, 5, 0.1);

        const Coord start{0, 2};
        const Coord goal{6, 2};

        const Coord removable_obstacle{3, 2};

        grid.setState(
            removable_obstacle,
            1
        );

        DStarLite planner(grid, start, goal);

        planner.computeShortestPath();

        const std::vector<Coord> detour_path =
            planner.extractPath();

        assert(!detour_path.empty());

        assert(
            validatePath(
                grid,
                detour_path,
                start,
                goal
            )
        );

        const bool obstacle_in_detour =
            std::find(
                detour_path.begin(),
                detour_path.end(),
                removable_obstacle
            ) != detour_path.end();

        assert(!obstacle_in_detour);

        const double detour_cost =
            totalPathCost(grid, detour_path);

        assert(
            std::abs(detour_cost - 8.0) <= EPS
        );

        planner.updateCellState(
            removable_obstacle,
            0
        );

        planner.computeShortestPath();

        const std::vector<Coord> shortened_path =
            planner.extractPath();

        assert(!shortened_path.empty());

        assert(
            validatePath(
                grid,
                shortened_path,
                start,
                goal
            )
        );

        const double shortened_cost =
            totalPathCost(grid, shortened_path);

        assert(
            std::abs(shortened_cost - 6.0) <= EPS
        );

        assert(shortened_cost < detour_cost);

        const bool recovered_cell_used =
            std::find(
                shortened_path.begin(),
                shortened_path.end(),
                removable_obstacle
            ) != shortened_path.end();

        assert(recovered_cell_used);

        const std::optional<std::vector<Coord>> astar_result =
            aStar(grid, start, goal);

        assert(astar_result.has_value());

        const std::vector<Coord>& astar_path =
            astar_result.value();

        assert(
            validatePath(
                grid,
                astar_path,
                start,
                goal
            )
        );

        const double astar_cost =
            totalPathCost(grid, astar_path);

        assert(
            std::abs(shortened_cost - astar_cost) <= EPS
        );

        std::cout
            << "Phase 11 Test 2 passed: obstacle disappeared.\n"
            << "Detour cost: "
            << detour_cost
            << '\n'
            << "Repaired D* Lite cost: "
            << shortened_cost
            << '\n'
            << "Fresh A* cost: "
            << astar_cost
            << '\n';

        grid.print(
            start,
            goal,
            shortened_path
        );
    }

    std::cout
        << "All Phase 11 incremental-update tests passed.\n";

    return 0;
}