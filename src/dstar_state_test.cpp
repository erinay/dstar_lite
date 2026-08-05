#include "dstar_lite.hpp"
#include "grid.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <queue>
#include <vector>

int main() {
    Grid grid(10, 10, 0.1);

    const Coord start{0, 0};
    const Coord goal{9, 9};

    DStarLite planner(grid, start, goal);

    // ============================================================
    // Phase 4: D* Lite state initialization
    // ============================================================

    // Check stored positions.
    assert(planner.start() == start);
    assert(planner.previousStart() == start);
    assert(planner.goal() == goal);

    // km begins at zero.
    assert(std::abs(planner.km()) <= EPS);

    // Verify that every g-value begins at infinity.
    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {
            const Coord cell{x, y};

            assert(std::isinf(planner.g(cell)));
        }
    }

    // Every rhs-value except the goal begins at infinity.
    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {
            const Coord cell{x, y};

            if (cell == goal) {
                assert(std::abs(planner.rhs(cell)) <= EPS);
            } else {
                assert(std::isinf(planner.rhs(cell)));
            }
        }
    }

    // Only the goal should initially be in the queue.
    assert(!planner.openEmpty());
    assert(planner.openSize() == 1);

    const QueueEntry top = planner.openTop();

    assert(top.state == goal);

    std::cout << "Phase 4 D* Lite state tests passed.\n";

    // ============================================================
    // Phase 5: Key calculation
    // ============================================================

    // Initial goal values:
    //
    // g(goal)   = infinity
    // rhs(goal) = 0
    //
    // min(g, rhs) = 0
    //
    // Manhattan distance from (0,0) to (9,9) = 18
    //
    // key(goal) = (18, 0)
    const PriorityKey goal_key = planner.calculateKey(goal);

    assert(std::abs(goal_key.k1 - 18.0) <= EPS);
    assert(std::abs(goal_key.k2 - 0.0) <= EPS);

    // The queue should contain the same key calculated for the goal.
    assert(std::abs(top.key.k1 - goal_key.k1) <= EPS);
    assert(std::abs(top.key.k2 - goal_key.k2) <= EPS);

    // Other cells still have:
    //
    // g   = infinity
    // rhs = infinity
    //
    // Therefore, both key components should be infinity.
    const Coord middle{5, 5};
    const PriorityKey middle_key =
        planner.calculateKey(middle);

    assert(std::isinf(middle_key.k1));
    assert(std::isinf(middle_key.k2));

    const PriorityKey start_key =
        planner.calculateKey(start);

    assert(std::isinf(start_key.k1));
    assert(std::isinf(start_key.k2));

    std::cout << "Goal key: ("
              << goal_key.k1
              << ", "
              << goal_key.k2
              << ")\n";

    std::cout << "Phase 5 key calculation tests passed.\n";

    // ============================================================
    // Phase 5: Lexicographic key comparison
    // ============================================================

    // Smaller first component should come first.
    const PriorityKey key_a{4.0, 10.0};
    const PriorityKey key_b{5.0, 0.0};

    assert(keyLess(key_a, key_b));
    assert(!keyLess(key_b, key_a));

    // If first components match, compare the second components.
    const PriorityKey key_c{5.0, 2.0};
    const PriorityKey key_d{5.0, 3.0};

    assert(keyLess(key_c, key_d));
    assert(!keyLess(key_d, key_c));

    // Values within EPS are treated as approximately equal.
    // Therefore, the second component breaks the tie.
    const PriorityKey key_e{
        5.0 + 0.5 * EPS,
        2.0
    };

    const PriorityKey key_f{
        5.0,
        3.0
    };

    assert(keyLess(key_e, key_f));
    assert(!keyLess(key_f, key_e));

    // If both components differ by less than EPS,
    // neither key should be considered smaller.
    const PriorityKey key_g{
        5.0,
        2.0
    };

    const PriorityKey key_h{
        5.0 + 0.5 * EPS,
        2.0 + 0.5 * EPS
    };

    assert(!keyLess(key_g, key_h));
    assert(!keyLess(key_h, key_g));

    std::cout
        << "Phase 5 lexicographic comparison tests passed.\n";

    // ============================================================
    // Test actual priority-queue ordering
    // ============================================================

    std::priority_queue<
        QueueEntry,
        std::vector<QueueEntry>,
        QueueEntryCompare
    > test_queue;

    test_queue.push(
        QueueEntry{
            Coord{1, 0},
            PriorityKey{8.0, 1.0}
        }
    );

    test_queue.push(
        QueueEntry{
            Coord{2, 0},
            PriorityKey{6.0, 4.0}
        }
    );

    test_queue.push(
        QueueEntry{
            Coord{3, 0},
            PriorityKey{6.0, 2.0}
        }
    );

    const Coord expected_first{3, 0};
    const Coord expected_second{2, 0};
    const Coord expected_third{1, 0};

    assert(test_queue.top().state == expected_first);
    test_queue.pop();

    assert(test_queue.top().state == expected_second);
    test_queue.pop();

    assert(test_queue.top().state == expected_third);

    std::cout
        << "Priority-queue ordering test passed.\n";

    std::cout
        << "\nAll Phase 4 and Phase 5 tests passed.\n";


        // ============================================================
    // Phase 6: Edge-cost function tests
    // ============================================================

    const Coord source{4, 4};
    const Coord free_cell{5, 4};
    const Coord unknown_cell{4, 5};
    const Coord occupied_cell{3, 4};

    grid.setState(source, 0);
    grid.setState(free_cell, 0);

    // Leave unknown_cell unchanged because cells initialize to -1.
    grid.setState(occupied_cell, 1);

    // Free -> free should cost 1.
    assert(
        std::abs(
            planner.cost(source, free_cell) - 1.0
        ) <= EPS
    );

    // Free -> unknown should cost 1.
    assert(
        std::abs(
            planner.cost(source, unknown_cell) - 1.0
        ) <= EPS
    );

    // Free -> occupied should be impossible.
    assert(
        std::isinf(
            planner.cost(source, occupied_cell)
        )
    );

    std::cout << "Phase 6 edge-cost tests passed.\n";

    // ============================================================
    // Phase 8: computeShortestPath
    // ============================================================

    Grid shortest_path_grid(10, 10, 0.1);

    const Coord shortest_path_start{0, 0};
    const Coord shortest_path_goal{9, 9};

    DStarLite shortest_path_planner(
        shortest_path_grid,
        shortest_path_start,
        shortest_path_goal
    );

    shortest_path_planner.computeShortestPath();

    // The goal's cost to itself is zero.
    assert(
        std::abs(
            shortest_path_planner.g(shortest_path_goal)
        ) <= EPS
    );

    // The Manhattan distance from (0,0) to (9,9) is 18.
    assert(
        std::abs(
            shortest_path_planner.g(shortest_path_start) -
            18.0
        ) <= EPS
    );

    // The start should be consistent after planning.
    assert(
        std::abs(
            shortest_path_planner.g(shortest_path_start) -
            shortest_path_planner.rhs(shortest_path_start)
        ) <= EPS
    );

    std::cout
        << "Phase 8 computeShortestPath test passed.\n";

    std::cout
        << "Start cost: "
        << shortest_path_planner.g(shortest_path_start)
        << '\n';

    // ============================================================
    // Phase 9: Extract the path from computed g-values
    // ============================================================

    // ------------------------------------------------------------
    // Test 1: Empty 10 x 10 grid
    // ------------------------------------------------------------
    {
        Grid grid(10, 10, 0.1);

        const Coord start{0, 0};
        const Coord goal{9, 9};

        DStarLite planner(grid, start, goal);

        planner.computeShortestPath();

        const std::vector<Coord> path =
            planner.extractPath();

        // A path should exist.
        assert(!path.empty());

        // The returned path includes both endpoints.
        assert(path.front() == start);
        assert(path.back() == goal);

        /*
        * Manhattan distance:
        *
        * 9 horizontal moves + 9 vertical moves = 18 moves
        *
        * A path with 18 moves contains 19 cells.
        */
        assert(path.size() == 19);

        double path_cost = 0.0;

        for (
            std::size_t i = 1;
            i < path.size();
            ++i
        ) {
            const Coord& from = path[i - 1];
            const Coord& to = path[i];

            // Consecutive cells must be valid neighbors.
            const double edge_cost =
                planner.cost(from, to);

            assert(std::isfinite(edge_cost));
            assert(
                std::abs(edge_cost - 1.0) <= EPS
            );

            path_cost += edge_cost;
        }

        assert(
            std::abs(path_cost - 18.0) <= EPS
        );

        // Since you use the non-optimized stopping condition,
        // the start should be consistent after planning.
        assert(
            std::abs(planner.g(start) - 18.0) <= EPS
        );

        assert(
            std::abs(planner.rhs(start) - 18.0) <= EPS
        );

        std::cout
            << "Phase 9 Test 1 passed: "
            << "empty-grid path extraction.\n";

        std::cout
            << "Path cost: "
            << path_cost
            << '\n';

        grid.print(start, goal, path);
    }

    // ------------------------------------------------------------
    // Test 2: Wall with one opening
    // ------------------------------------------------------------
    {
        Grid grid(10, 10, 0.1);

        const Coord start{1, 1};
        const Coord goal{8, 1};

        /*
        * Create a vertical wall at x = 4.
        *
        * Leave one opening at (4, 7).
        */
        for (int y = 0; y < grid.height(); ++y) {
            if (y == 7) {
                continue;
            }

            const Coord wall_cell{4, y};
            grid.setState(wall_cell, 1);
        }

        DStarLite planner(grid, start, goal);

        planner.computeShortestPath();

        const std::vector<Coord> path =
            planner.extractPath();

        assert(!path.empty());
        assert(path.front() == start);
        assert(path.back() == goal);

        double path_cost = 0.0;
        bool passed_through_opening = false;

        const Coord opening{4, 7};

        for (
            std::size_t i = 0;
            i < path.size();
            ++i
        ) {
            const Coord& cell = path[i];

            assert(grid.inBounds(cell));

            // No occupied cell may appear in the path.
            assert(grid.state(cell) != 1);

            if (cell == opening) {
                passed_through_opening = true;
            }

            if (i == 0) {
                continue;
            }

            const Coord& previous = path[i - 1];

            const double edge_cost =
                planner.cost(previous, cell);

            assert(std::isfinite(edge_cost));
            assert(
                std::abs(edge_cost - 1.0) <= EPS
            );

            path_cost += edge_cost;
        }

        // The wall spans the entire grid except for this opening.
        assert(passed_through_opening);

        /*
        * Shortest route:
        *
        * (1,1) -> (4,7): 3 + 6 = 9 moves
        * (4,7) -> (8,1): 4 + 6 = 10 moves
        *
        * Total = 19 moves.
        */
        assert(
            std::abs(path_cost - 19.0) <= EPS
        );

        assert(path.size() == 20);

        assert(
            std::abs(planner.g(start) - 19.0) <= EPS
        );

        assert(
            std::abs(planner.rhs(start) - 19.0) <= EPS
        );

        std::cout
            << "Phase 9 Test 2 passed: "
            << "path through wall opening.\n";

        std::cout
            << "Path cost: "
            << path_cost
            << '\n';

        grid.print(start, goal, path);
    }

    // ------------------------------------------------------------
    // Test 3: Start equals goal
    // ------------------------------------------------------------
    {
        Grid grid(5, 5, 0.1);

        const Coord start{2, 2};
        const Coord goal{2, 2};

        DStarLite planner(grid, start, goal);

        planner.computeShortestPath();

        const std::vector<Coord> path =
            planner.extractPath();

        // A zero-movement path still contains one cell.
        assert(path.size() == 1);
        assert(path.front() == start);
        assert(path.back() == goal);

        assert(
            std::abs(planner.g(start)) <= EPS
        );

        assert(
            std::abs(planner.rhs(start)) <= EPS
        );

        std::cout
            << "Phase 9 Test 3 passed: "
            << "start equals goal.\n";
    }

    // ------------------------------------------------------------
    // Test 4: No path exists
    // ------------------------------------------------------------
    {
        Grid grid(5, 5, 0.1);

        const Coord start{0, 0};
        const Coord goal{4, 4};

        // Block both possible exits from the corner start.
        const Coord blocked_right{1, 0};
        const Coord blocked_down{0, 1};

        grid.setState(blocked_right, 1);
        grid.setState(blocked_down, 1);

        DStarLite planner(grid, start, goal);

        planner.computeShortestPath();

        const std::vector<Coord> path =
            planner.extractPath();

        // No path should be returned.
        assert(path.empty());

        // In the non-optimized algorithm, an unreachable start
        // ends with g(start) = rhs(start) = infinity.
        assert(std::isinf(planner.g(start)));
        assert(std::isinf(planner.rhs(start)));

        std::cout
            << "Phase 9 Test 4 passed: "
            << "no-path case.\n";
    }

    std::cout
        << "All Phase 9 path extraction tests passed.\n";

    return 0;
}