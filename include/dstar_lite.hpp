#include "grid.hpp"

#include <cstddef>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

// generate expression for INF, because it's used often
inline constexpr double INF = std::numeric_limits<double>::infinity();
inline constexpr double EPS = 1e-9;;
// INITIALIZATION: for all vertices s
// g = goal distance
// rhs = cost of moving from s' (neighbor) to s, plus neighbor's g-value g(s')
// rhs = g implies consistent costs.
// over consisten (g>rhs), path cost just decreased, like when obstacle is removed 
// under consistent (g<rhs), path cost increased, like when obstacle is added to environment. 
struct NodeData{
    double g = INF;
    double rhs = INF;
};

//Priority key k(s)
struct Key{
    double k1;
    double k2;
};

//Queue entry: needs cooridnate and key
struct PriorityKey {
    double k1; //estimates total path cost, primary priority key!
    double k2; // breaks ties by prioritizing nodes closer to the goal
};

struct QueueEntry {
    Coord state;
    PriorityKey key;
};
bool keyLess(
    const PriorityKey& left,
    const PriorityKey& right
);
// Makes std::priority_queue place the smallest key on top.
struct QueueEntryCompare {
    bool operator()(
        const QueueEntry& left,
        const QueueEntry& right
    ) const;
};

class DStarLite {
public:
    DStarLite(
        Grid& grid,
        const Coord& start,
        const Coord& goal
    );

    // Read a cell's D* Lite data.
    const NodeData& data(const Coord& s) const;

    double g(const Coord& s) const;
    double rhs(const Coord& s) const;

    // Calculate the two-component D* Lite priority key.
    PriorityKey calculateKey(const Coord& s) const;

    // Access stored planner state.
    const Coord& start() const;
    const Coord& previousStart() const;
    const Coord& goal() const;

    double km() const;

    // COST FUNCTION
    double cost(const Coord& from, const Coord& to) const;

    // update Vertext
    void updateVertex(const Coord& s);

    void computeShortestPath();

    // Temporary queue accessors for Phase 4 testing.
    bool openEmpty() const;
    std::size_t openSize() const;
    QueueEntry openTop() const;

private:
    Grid& grid_;

    // Hash map of coord, corresponding g, rhs values
    std::unordered_map<Coord, NodeData, CoordHash> nodes_;

    Coord start_;
    Coord previous_start_;
    Coord goal_;

    double km_; // modifier to account for robot movement between map updates (prevents needfor resorting)

    // Priority queue (element_type, storage contianer, comparison func)
    std::priority_queue<
        QueueEntry,
        std::vector<QueueEntry>,
        QueueEntryCompare
    > open_;

    void initialize();

    static double heuristic(
        const Coord& a,
        const Coord& b
    );
};

// //NOTE: USE PRIORITY QUEUE IN THIS WAY:
// open_.push(entry);  // insert an entry
// open_.top();        // view the smallest-key entry
// open_.pop();        // remove the top entry
// open_.empty();      // check whether it is empty
// open_.size();       // number of entries