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

    return 0;
}