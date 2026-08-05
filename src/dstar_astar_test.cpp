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

        std::cout<< "original cost: "<<original_cost<<std::endl;
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

    // ============================================================
    // Phase 12: Move the D* Lite start
    // ============================================================

    // ------------------------------------------------------------
    // Test 1: Move the start one step
    // ------------------------------------------------------------
    {
        Grid grid(10, 5, 0.1);

        const Coord initial_start{0, 2};
        const Coord goal{9, 2};

        DStarLite planner(
            grid,
            initial_start,
            goal
        );

        planner.computeShortestPath();

        const std::vector<Coord> initial_path =
            planner.extractPath();

        assert(!initial_path.empty());

        assert(
            validatePath(
                grid,
                initial_path,
                initial_start,
                goal
            )
        );

        const double initial_cost =
            totalPathCost(
                grid,
                initial_path
            );

        assert(
            std::abs(initial_cost - 9.0) <= EPS
        );

        // Move to the next cell on the path.
        assert(initial_path.size() >= 2);

        const Coord next_start =
            initial_path[1];

        const double old_km =
            planner.km();

        planner.moveStart(next_start);

        assert(planner.start() == next_start);

        assert(
            planner.previousStart() ==
            initial_start
        );

        const double expected_km =
            old_km +
            manhattanDistance(
                initial_start,
                next_start
            );

        assert(
            std::abs(
                planner.km() - expected_km
            ) <= EPS
        );

        planner.computeShortestPath();

        const std::vector<Coord> moved_path =
            planner.extractPath();

        assert(!moved_path.empty());

        assert(
            validatePath(
                grid,
                moved_path,
                next_start,
                goal
            )
        );

        const double moved_cost =
            totalPathCost(
                grid,
                moved_path
            );

        assert(
            std::abs(moved_cost - 8.0) <= EPS
        );

        const std::optional<std::vector<Coord>>
            astar_result =
                aStar(
                    grid,
                    next_start,
                    goal
                );

        assert(astar_result.has_value());

        const double astar_cost =
            totalPathCost(
                grid,
                astar_result.value()
            );

        assert(
            std::abs(
                moved_cost - astar_cost
            ) <= EPS
        );

        std::cout
            << "Phase 12 Test 1 passed: "
            << "start moved one step.\n"
            << "Old start: ("
            << initial_start.x
            << ", "
            << initial_start.y
            << ")\n"
            << "New start: ("
            << next_start.x
            << ", "
            << next_start.y
            << ")\n"
            << "Initial cost: "
            << initial_cost
            << '\n'
            << "Remaining cost: "
            << moved_cost
            << '\n'
            << "km: "
            << planner.km()
            << '\n';
    }

    // ------------------------------------------------------------
    // Test 2: Move the start all the way to the goal
    // ------------------------------------------------------------
    {
        Grid grid(10, 10, 0.1);

        const Coord initial_start{0, 0};
        const Coord goal{9, 9};

        // Wall at x = 4 with an opening at y = 7.
        for (int y = 0; y < grid.height(); ++y) {
            if (y == 7) {
                continue;
            }

            grid.setState(
                Coord{4, y},
                1
            );
        }

        DStarLite planner(
            grid,
            initial_start,
            goal
        );

        planner.computeShortestPath();

        Coord current_start =
            initial_start;

        double expected_km = 0.0;
        std::size_t move_count = 0;

        while (!(current_start == goal)) {
            const std::vector<Coord> dstar_path =
                planner.extractPath();

            assert(!dstar_path.empty());

            assert(
                validatePath(
                    grid,
                    dstar_path,
                    current_start,
                    goal
                )
            );

            const std::optional<std::vector<Coord>>
                astar_result =
                    aStar(
                        grid,
                        current_start,
                        goal
                    );

            assert(astar_result.has_value());

            const std::vector<Coord>& astar_path =
                astar_result.value();

            assert(
                validatePath(
                    grid,
                    astar_path,
                    current_start,
                    goal
                )
            );

            const double dstar_cost =
                totalPathCost(
                    grid,
                    dstar_path
                );

            const double astar_cost =
                totalPathCost(
                    grid,
                    astar_path
                );

            // D* Lite must remain optimal after every move.
            assert(
                std::abs(
                    dstar_cost - astar_cost
                ) <= EPS
            );

            // The path needs at least one next state.
            assert(dstar_path.size() >= 2);

            const Coord old_start =
                current_start;

            const Coord next_start =
                dstar_path[1];

            // The robot should move exactly one grid cell.
            const double movement_distance =
                manhattanDistance(
                    old_start,
                    next_start
                );

            assert(
                std::abs(
                    movement_distance - 1.0
                ) <= EPS
            );

            planner.moveStart(next_start);

            expected_km += movement_distance;

            assert(
                planner.previousStart() ==
                old_start
            );

            assert(
                planner.start() ==
                next_start
            );

            assert(
                std::abs(
                    planner.km() -
                    expected_km
                ) <= EPS
            );

            current_start = next_start;
            ++move_count;

            // Protect the test from an accidental infinite loop.
            assert(
                move_count <=
                static_cast<std::size_t>(
                    grid.width() *
                    grid.height()
                )
            );

            planner.computeShortestPath();
        }

        // At the goal, the returned path should contain only
        // the goal and should cost zero.
        const std::vector<Coord> final_path =
            planner.extractPath();

        assert(!final_path.empty());
        assert(final_path.size() == 1);
        assert(final_path.front() == goal);

        assert(
            validatePath(
                grid,
                final_path,
                goal,
                goal
            )
        );

        const double final_cost =
            totalPathCost(
                grid,
                final_path
            );

        assert(
            std::abs(final_cost) <= EPS
        );

        std::cout
            << "Phase 12 Test 2 passed: "
            << "start moved to goal.\n"
            << "Number of moves: "
            << move_count
            << '\n'
            << "Final km: "
            << planner.km()
            << '\n'
            << "Final cost: "
            << final_cost
            << '\n';
    }

    // ------------------------------------------------------------
    // Test 3: Move, then discover an obstacle
    // ------------------------------------------------------------
    {
        Grid grid(10, 5, 0.1);

        const Coord initial_start{0, 2};
        const Coord goal{9, 2};

        DStarLite planner(
            grid,
            initial_start,
            goal
        );

        planner.computeShortestPath();

        const std::vector<Coord> original_path =
            planner.extractPath();

        assert(!original_path.empty());
        assert(original_path.size() >= 5);

        assert(
            validatePath(
                grid,
                original_path,
                initial_start,
                goal
            )
        );

        // First move the robot one step.
        const Coord moved_start =
            original_path[1];

        planner.moveStart(moved_start);

        assert(
            planner.previousStart() ==
            initial_start
        );

        assert(
            planner.start() ==
            moved_start
        );

        assert(
            std::abs(
                planner.km() - 1.0
            ) <= EPS
        );

        // Discover an obstacle farther ahead on the
        // original straight path.
        const Coord new_obstacle =
            original_path[4];

        assert(
            !(new_obstacle == moved_start)
        );

        assert(
            !(new_obstacle == goal)
        );

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
                moved_start,
                goal
            )
        );

        const bool obstacle_in_path =
            std::find(
                repaired_path.begin(),
                repaired_path.end(),
                new_obstacle
            ) != repaired_path.end();

        assert(!obstacle_in_path);

        const double repaired_cost =
            totalPathCost(
                grid,
                repaired_path
            );

        // From x = 1 to x = 9 normally costs 8.
        // Going around one blocked cell adds 2.
        assert(
            std::abs(repaired_cost - 10.0) <= EPS
        );

        const std::optional<std::vector<Coord>>
            astar_result =
                aStar(
                    grid,
                    moved_start,
                    goal
                );

        assert(astar_result.has_value());

        const std::vector<Coord>& astar_path =
            astar_result.value();

        assert(
            validatePath(
                grid,
                astar_path,
                moved_start,
                goal
            )
        );

        const double astar_cost =
            totalPathCost(
                grid,
                astar_path
            );

        assert(
            std::abs(
                repaired_cost - astar_cost
            ) <= EPS
        );

        std::cout
            << "Phase 12 Test 3 passed: "
            << "moved start and repaired path.\n"
            << "Current start: ("
            << moved_start.x
            << ", "
            << moved_start.y
            << ")\n"
            << "New obstacle: ("
            << new_obstacle.x
            << ", "
            << new_obstacle.y
            << ")\n"
            << "Repaired D* Lite cost: "
            << repaired_cost
            << '\n'
            << "Fresh A* cost: "
            << astar_cost
            << '\n';

        grid.print(
            moved_start,
            goal,
            repaired_path
        );
    }

    std::cout
        << "All Phase 10-12 tests passed.\n";

    return 0;
}