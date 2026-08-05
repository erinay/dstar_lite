#include "astar.hpp"
#include "dstar_lite.hpp"
#include "grid.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <optional>
#include <random>
#include <vector>

// Change this to 1 after adding an expansion-count getter to DStarLite.
#ifndef DSTAR_HAS_EXPANSION_COUNTER
#define DSTAR_HAS_EXPANSION_COUNTER 0
#endif

namespace {

constexpr double TEST_EPSILON = 1.0e-9;

bool approximatelyEqual(double a, double b) {
    if (std::isinf(a) && std::isinf(b)) {
        return true;
    }

    return std::abs(a - b) <= TEST_EPSILON;
}

void setAllCells(Grid& grid, int state) {
    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {
            grid.setState(Coord{x, y}, state);
        }
    }
}

std::size_t gridIndex(
    const Grid& grid,
    const Coord& cell
) {
    return static_cast<std::size_t>(
        cell.y * grid.width() + cell.x
    );
}

/*
 * Run both planners on the same current grid and verify:
 *
 * 1. They agree on whether a path exists.
 * 2. Both returned paths are valid.
 * 3. Both paths have the same optimal cost.
 *
 * Returns the D* Lite path. An empty vector means no path.
 */
std::vector<Coord> compareDStarWithAStar(
    DStarLite& planner,
    const Grid& grid,
    const Coord& start,
    const Coord& goal
) {
    planner.computeShortestPath();

    const std::vector<Coord> dstar_path =
        planner.extractPath();

    const std::optional<std::vector<Coord>> astar_result =
        aStar(
            grid,
            start,
            goal
        );

    const bool dstar_found_path =
        !dstar_path.empty();

    const bool astar_found_path =
        astar_result.has_value();

    assert(dstar_found_path == astar_found_path);

    if (!dstar_found_path) {
        return {};
    }

    assert(
        validatePath(
            grid,
            dstar_path,
            start,
            goal
        )
    );

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

    assert(
        approximatelyEqual(
            dstar_cost,
            astar_cost
        )
    );

    return dstar_path;
}

struct RevealResult {
    std::size_t changed_cells = 0;
    std::size_t occupied_cells_revealed = 0;
};

/*
 * Reveal all cells within the given Manhattan sensor radius.
 *
 * ground_truth:
 *     The actual environment, which is not given to D* Lite.
 *
 * belief:
 *     The map used by D* Lite.
 *
 * revealed:
 *     Records which ground-truth cells have been observed.
 */
RevealResult revealAroundRobot(
    const Grid& ground_truth,
    Grid& belief,
    DStarLite& planner,
    const Coord& robot,
    int sensor_radius,
    std::vector<bool>& revealed
) {
    RevealResult result;

    for (
        int dy = -sensor_radius;
        dy <= sensor_radius;
        ++dy
    ) {
        for (
            int dx = -sensor_radius;
            dx <= sensor_radius;
            ++dx
        ) {
            if (std::abs(dx) + std::abs(dy) > sensor_radius) {
                continue;
            }

            const Coord observed{
                robot.x + dx,
                robot.y + dy
            };

            if (!ground_truth.inBounds(observed)) {
                continue;
            }

            revealed[
                gridIndex(
                    ground_truth,
                    observed
                )
            ] = true;

            const int true_state =
                ground_truth.state(observed);

            const int believed_state =
                belief.state(observed);

            if (true_state == believed_state) {
                continue;
            }

            planner.updateCellState(
                observed,
                true_state
            );

            ++result.changed_cells;

            if (true_state == 1) {
                ++result.occupied_cells_revealed;
            }
        }
    }

    return result;
}

void printStepInformation(
    const Grid& belief,
    const Coord& robot,
    const Coord& goal,
    const std::vector<Coord>& path,
    std::size_t step,
    std::size_t changed_cells
) {
    std::cout
        << "\n========================================\n"
        << "Step: " << step << '\n'
        << "Robot: ("
        << robot.x
        << ", "
        << robot.y
        << ")\n"
        << "Goal: ("
        << goal.x
        << ", "
        << goal.y
        << ")\n"
        << "Changed cells: "
        << changed_cells
        << '\n';

    if (path.empty()) {
        std::cout << "Path: none\n";
    } else {
        std::cout
            << "Path cost: "
            << totalPathCost(
                belief,
                path
            )
            << '\n';
    }

    std::cout << "Queue expansions: ";

#if DSTAR_HAS_EXPANSION_COUNTER
    /*
     * Rename this call if your getter has a different name.
     *
     * It should report how many states were expanded during the
     * most recent computeShortestPath() call.
     */
    std::cout << planner.lastExpansionCount();
#else
    std::cout << "not instrumented";
#endif

    std::cout
        << "\n\nBelief map:\n";

    belief.print(
        robot,
        goal,
        path
    );
}

// ================================================================
// Phase 13: Goal-directed exploration in an unknown environment
// ================================================================

void testUnknownEnvironmentExploration() {
    /*
     * ground_truth:
     *
     * .....#....
     * .....#....
     * S....#...G
     * .....#....
     * ..........
     * .....#....
     *
     * The only wall opening is at (5, 4).
     *
     * The belief map starts completely unknown. Because unknown
     * cells cost 1, the first optimistic path is directly toward
     * the goal.
     */
    Grid ground_truth(10, 6, 0.1);
    Grid belief(10, 6, 0.1);

    // The actual environment is initially free.
    setAllCells(
        ground_truth,
        0
    );

    // Add a hidden wall with one opening at y = 4.
    for (int y = 0; y < ground_truth.height(); ++y) {
        if (y == 4) {
            continue;
        }

        ground_truth.setState(
            Coord{5, y},
            1
        );
    }

    // The belief grid remains entirely unknown.
    const Coord initial_start{0, 2};
    const Coord goal{9, 2};

    DStarLite planner(
        belief,
        initial_start,
        goal
    );

    // Before sensing anything, the optimistic path is straight.
    planner.computeShortestPath();

    const std::vector<Coord> optimistic_path =
        planner.extractPath();

    assert(!optimistic_path.empty());

    assert(
        validatePath(
            belief,
            optimistic_path,
            initial_start,
            goal
        )
    );

    const double optimistic_cost =
        totalPathCost(
            belief,
            optimistic_path
        );

    assert(
        approximatelyEqual(
            optimistic_cost,
            9.0
        )
    );

    std::vector<bool> revealed(
        static_cast<std::size_t>(
            belief.width() *
            belief.height()
        ),
        false
    );

    Coord robot = initial_start;

    std::size_t move_count = 0;
    std::size_t total_changed_cells = 0;
    std::size_t total_occupied_revealed = 0;

    constexpr int sensor_radius = 1;

    while (!(robot == goal)) {
        const RevealResult reveal =
            revealAroundRobot(
                ground_truth,
                belief,
                planner,
                robot,
                sensor_radius,
                revealed
            );

        total_changed_cells +=
            reveal.changed_cells;

        total_occupied_revealed +=
            reveal.occupied_cells_revealed;

        const std::vector<Coord> path =
            compareDStarWithAStar(
                planner,
                belief,
                robot,
                goal
            );

        assert(!path.empty());
        assert(path.size() >= 2);

        const Coord next =
            path[1];

        // The next cell is within sensor radius 1 and must
        // therefore already have been observed.
        assert(
            revealed[
                gridIndex(
                    belief,
                    next
                )
            ]
        );

        // This assertion checks simulation safety only.
        // The planner itself was given only the belief map.
        assert(
            ground_truth.state(next) != 1
        );

        std::cout
        << "Move "
        << move_count
        << ": ("
        << robot.x
        << ", "
        << robot.y
        << ") -> ("
        << next.x
        << ", "
        << next.y
        << ")\n";

        planner.moveStart(next);
        robot = next;

        ++move_count;

        // Protect against an accidental infinite loop.
        assert(
            move_count <=
            static_cast<std::size_t>(
                ground_truth.width() *
                ground_truth.height() *
                4
            )
        );
    }

    const std::vector<Coord> final_path =
        compareDStarWithAStar(
            planner,
            belief,
            goal,
            goal
        );

    assert(final_path.size() == 1);
    assert(final_path.front() == goal);

    assert(
        approximatelyEqual(
            totalPathCost(
                belief,
                final_path
            ),
            0.0
        )
    );

    // The hidden wall must actually have been detected.
    assert(total_occupied_revealed > 0);


    std::cout << "move_count: " << move_count << std::endl;

    const std::optional<std::vector<Coord>> ground_truth_result =
    aStar(
        ground_truth,
        initial_start,
        goal
    );

    assert(ground_truth_result.has_value());

    const double ground_truth_optimal_cost =
        totalPathCost(
            ground_truth,
            ground_truth_result.value()
        );

    assert(
        approximatelyEqual(
            ground_truth_optimal_cost,
            13.0
        )
    );

    // Online exploration cannot generally outperform the
    // path planned with complete prior map knowledge.
    assert(
        static_cast<double>(move_count) >=
        ground_truth_optimal_cost
    );

    /*
     * Direct path: 9 moves.
     * Actual path through the opening: 13 moves.
     */
    assert(
        approximatelyEqual(
            ground_truth_optimal_cost,
            13.0
        )
    );

    assert(
        static_cast<double>(move_count) >=
        ground_truth_optimal_cost
);
        /*
     * Confirm that cells outside the sensor history remain unknown
     * in the belief map. This helps catch accidental copying of the
     * complete ground-truth map into the planner's map.
     */
    for (int y = 0; y < belief.height(); ++y) {
        for (int x = 0; x < belief.width(); ++x) {
            const Coord cell{x, y};

            if (
                !revealed[
                    gridIndex(
                        belief,
                        cell
                    )
                ]
            ) {
                assert(
                    belief.state(cell) == -1
                );
            }
        }
    }

    std::cout
        << "Phase 13 passed: unknown-environment exploration.\n"
        << "Initial optimistic cost: "
        << optimistic_cost
        << '\n'
        << "Actual moves: "
        << move_count
        << '\n'
        << "Changed belief cells: "
        << total_changed_cells
        << '\n'
        << "Occupied cells revealed: "
        << total_occupied_revealed
        << '\n';
}

// ================================================================
// Phase 14: Automated randomized comparison against A*
// ================================================================

void testRandomIncrementalUpdates() {
    Grid grid(20, 20, 0.1);

    // Start with a fully known free grid.
    setAllCells(
        grid,
        0
    );

    Coord current_start{0, 0};
    const Coord goal{19, 19};

    DStarLite planner(
        grid,
        current_start,
        goal
    );

    std::mt19937 random_generator(42);

    std::uniform_int_distribution<int> x_distribution(
        0,
        grid.width() - 1
    );

    std::uniform_int_distribution<int> y_distribution(
        0,
        grid.height() - 1
    );

    constexpr std::size_t number_of_updates = 1000;

    std::size_t added_obstacles = 0;
    std::size_t removed_obstacles = 0;
    std::size_t start_moves = 0;
    std::size_t no_path_cases = 0;

    for (
        std::size_t update_number = 0;
        update_number < number_of_updates;
        ++update_number
    ) {
        Coord changed_cell;

        // Never place an obstacle on the current start or goal.
        do {
            changed_cell = Coord{
                x_distribution(random_generator),
                y_distribution(random_generator)
            };
        } while (
            changed_cell == current_start ||
            changed_cell == goal
        );

        const int old_state =
            grid.state(changed_cell);

        const int new_state =
            old_state == 1 ? 0 : 1;

        planner.updateCellState(
            changed_cell,
            new_state
        );

        if (new_state == 1) {
            ++added_obstacles;
        } else {
            ++removed_obstacles;
        }

        std::vector<Coord> path =
            compareDStarWithAStar(
                planner,
                grid,
                current_start,
                goal
            );

        if (path.empty()) {
            ++no_path_cases;
        }

        /*
         * Periodically move the start one cell along the current
         * optimal path. This tests interleaved:
         *
         * - edge-cost increases,
         * - edge-cost decreases,
         * - moving-start updates.
         */
        if (
            update_number % 10 == 0 &&
            path.size() >= 2
        ) {
            const Coord next_start =
                path[1];

            assert(
                manhattanDistance(
                    current_start,
                    next_start
                ) == 1.0
            );

            planner.moveStart(next_start);
            current_start = next_start;

            ++start_moves;

            // Recompare immediately after moving the start.
            path =
                compareDStarWithAStar(
                    planner,
                    grid,
                    current_start,
                    goal
                );
        }

        /*
         * If the robot reaches the goal, restart it at a known-free
         * corner so that the remaining random updates continue to
         * exercise moving-start behavior.
         */
        if (current_start == goal) {
            const Coord restart{0, 0};

            if (grid.state(restart) == 1) {
                planner.updateCellState(
                    restart,
                    0
                );

                ++removed_obstacles;
            }

            planner.moveStart(restart);
            current_start = restart;

            ++start_moves;

            compareDStarWithAStar(
                planner,
                grid,
                current_start,
                goal
            );
        }
    }

    assert(
        added_obstacles +
        removed_obstacles >=
        number_of_updates
    );

    assert(start_moves > 0);

    std::cout
        << "Phase 14 passed: randomized incremental testing.\n"
        << "Random seed: 42\n"
        << "Updates tested: "
        << number_of_updates
        << '\n'
        << "Obstacles added: "
        << added_obstacles
        << '\n'
        << "Obstacles removed: "
        << removed_obstacles
        << '\n'
        << "Start movements: "
        << start_moves
        << '\n'
        << "No-path comparisons: "
        << no_path_cases
        << '\n';
}

// ================================================================
// Phase 15: Step-by-step visualization
// ================================================================

void testExplorationVisualization() {
    Grid ground_truth(8, 6, 0.1);
    Grid belief(8, 6, 0.1);

    setAllCells(
        ground_truth,
        0
    );

    // Hidden wall with an opening at (4, 4).
    for (int y = 0; y < ground_truth.height(); ++y) {
        if (y == 4) {
            continue;
        }

        ground_truth.setState(
            Coord{4, y},
            1
        );
    }

    const Coord initial_start{0, 2};
    const Coord goal{7, 2};

    DStarLite planner(
        belief,
        initial_start,
        goal
    );

    std::vector<bool> revealed(
        static_cast<std::size_t>(
            belief.width() *
            belief.height()
        ),
        false
    );

    Coord robot = initial_start;

    std::size_t step = 0;

    constexpr int sensor_radius = 1;

    while (true) {
        const RevealResult reveal =
            revealAroundRobot(
                ground_truth,
                belief,
                planner,
                robot,
                sensor_radius,
                revealed
            );

        const std::vector<Coord> path =
            compareDStarWithAStar(
                planner,
                belief,
                robot,
                goal
            );

        printStepInformation(
            belief,
            robot,
            goal,
            path,
            step,
            reveal.changed_cells
        );

        if (robot == goal) {
            assert(path.size() == 1);
            break;
        }

        assert(path.size() >= 2);

        const Coord next =
            path[1];

        assert(
            ground_truth.state(next) != 1
        );

        planner.moveStart(next);
        robot = next;

        ++step;

        assert(
            step <=
            static_cast<std::size_t>(
                ground_truth.width() *
                ground_truth.height() *
                4
            )
        );
    }

    std::cout
        << "\nPhase 15 passed: visualization completed.\n";
}

} // namespace

int main() {
    std::cout
        << "Running Phase 13 tests...\n";

    testUnknownEnvironmentExploration();

    std::cout
        << "\nRunning Phase 14 tests...\n";

    testRandomIncrementalUpdates();

    std::cout
        << "\nRunning Phase 15 tests...\n";

    testExplorationVisualization();

    std::cout
        << "\nAll Phase 13-15 tests passed.\n";

    return 0;
}