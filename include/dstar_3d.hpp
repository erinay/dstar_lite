#pragma once

#include <cstddef>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace dstar_3d {

inline constexpr double kInfinity = std::numeric_limits<double>::infinity();
inline constexpr double kEpsilon = 1e-9;

// Integer voxel coordinate in a three-dimensional grid.
struct Coord {
    int x;
    int y;
    int z;

    bool operator==(const Coord& other) const;
};

struct CoordHash {
    std::size_t operator()(const Coord& coord) const;
};

// Voxel occupancy and traversal-cost map used by D* Lite 3D.
// Unknown and free voxels are traversable; occupied voxels are not.
class Grid {
public:
    static constexpr int kUnknown = -1;
    static constexpr int kFree = 0;
    static constexpr int kOccupied = 1;

    Grid(int width, int height, int depth, double resolution);

    bool inBounds(const Coord& voxel) const;

    int state(const Coord& voxel) const;
    void setState(const Coord& voxel, int state);
    void setTraversalCost(const Coord& voxel, double cost);

    // Returns exactly the in-bounds axis-aligned neighbours: +/-x, +/-y, +/-z.
    // No diagonal neighbours are returned.
    std::vector<Coord> neighbors(const Coord& voxel) const;

    // Entering an occupied voxel costs infinity. All other voxels use their
    // configured per-voxel cost, which must be at least one.
    double traversalCost(const Coord& voxel) const;

    int width() const;
    int height() const;
    int depth() const;
    double resolution() const;

private:
    int width_;
    int height_;
    int depth_;
    double resolution_;
    std::vector<int> cells_;
    std::vector<double> traversal_costs_;

    std::size_t index(const Coord& voxel) const;
    void checkBounds(const Coord& voxel) const;
};

struct NodeData {
    double g = kInfinity;
    double rhs = kInfinity;
};

struct PriorityKey {
    double k1;
    double k2;
};

struct QueueEntry {
    Coord state;
    PriorityKey key;
};

bool keyLess(const PriorityKey& left, const PriorityKey& right);

// Makes std::priority_queue expose the lexicographically smallest key first.
struct QueueEntryCompare {
    bool operator()(const QueueEntry& left, const QueueEntry& right) const;
};

// Incremental 3D D* Lite planner. Every motion is one axis-aligned voxel step,
// so its heuristic is the three-dimensional Manhattan distance.
class DStarLite3D {
public:
    DStarLite3D(Grid& grid, const Coord& start, const Coord& goal);

    const NodeData& data(const Coord& voxel) const;
    double g(const Coord& voxel) const;
    double rhs(const Coord& voxel) const;

    PriorityKey calculateKey(const Coord& voxel) const;

    const Coord& start() const;
    const Coord& previousStart() const;
    const Coord& goal() const;
    double km() const;

    // Returns infinity unless from and to are adjacent in the 6-connected grid.
    double cost(const Coord& from, const Coord& to) const;

    void updateVertex(const Coord& voxel);
    void computeShortestPath();
    std::vector<Coord> extractPath() const;

    void moveStart(const Coord& new_start);

    // Call after sensor data changes a voxel. This updates every affected
    // incoming edge and preserves D* Lite's incremental search state.
    void updateCellState(const Coord& voxel, int new_state);
    void updateCell(const Coord& voxel, int new_state, double traversal_cost);

    bool openEmpty() const;
    std::size_t openSize() const;
    QueueEntry openTop() const;

private:
    Grid& grid_;
    std::unordered_map<Coord, NodeData, CoordHash> nodes_;
    Coord start_;
    Coord previous_start_;
    Coord goal_;
    double km_ = 0.0;

    std::priority_queue<
        QueueEntry,
        std::vector<QueueEntry>,
        QueueEntryCompare
    > open_;

    void initialize();
    static double heuristic(const Coord& a, const Coord& b);
};

} // namespace dstar_3d
