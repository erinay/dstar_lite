#include "dstar_lite.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>


bool QueueEntryCompare::operator()(const QueueEntry& left, const QueueEntry& right) const{
    // min function, priority queue usually does max, so we create our own comparison
    return keyLess(right.key, left.key);
}

bool keyLess(const PriorityKey& left, const PriorityKey& right) {
    if (left.k1 < right.k1 - EPS) {
        return true;
    }
    if (left.k1 > right.k1 + EPS) {
        return false;
    }
    return left.k2 < right.k2 - EPS;
}

DStarLite::DStarLite(Grid& grid, const Coord& start, const Coord& goal)
    : grid_(grid), start_(start), previous_start_(start), goal_(goal), km_(0.0) {
    initialize(); } 

    void DStarLite::initialize() {
        // For every grid cell, set rhs(s), g(s) to infty
        for (int y=0; y<grid_.height(); y++){
            for(int x=0; x<grid_.width(); x++){
                const Coord cell{x,y};
                // emplace, inserts new element into container Node Data already intialized to infty
                nodes_.emplace(cell, NodeData{});
            }
        }

        //rhs of goal is 0, .at() accesses values by key, with at() providing bounds checking
        nodes_.at(goal_).rhs=0.0;

        // Priority queue is empty. Insert (s_goal [h(s_start, s_goal; 0]);
        open_.push(QueueEntry{goal_, calculateKey(goal_)});
    }

    // getter functions
    const NodeData& DStarLite::data(const Coord& s) const{
        return nodes_.at(s);
    }

    // Not const, so it can be modified
    double DStarLite::g(const Coord &s) const{
        return data(s).g;
    }
    double DStarLite::rhs(const Coord &s) const{
        return data(s).rhs;
    }
    const Coord& DStarLite::start() const {
        return start_;
    }   
    const Coord& DStarLite::previousStart() const {
        return previous_start_;
    }
    const Coord& DStarLite::goal() const {
    return goal_;
    }

    double DStarLite::km() const {
        return km_;
    }

    double DStarLite::heuristic(const Coord& a, const Coord& b) {
        //using Manhatten distance, assuming not moving diagonal
        return (double)(std::abs(a.x-b.x)+std::abs(a.y-b.y));

    }

    PriorityKey DStarLite::calculateKey(const Coord &s) const{
        const NodeData& node = data(s);
        const double min_cost = std::min(node.g, node.rhs);
        const double k1 = min_cost+heuristic(start_, s)+km_;
        const double k2 = min_cost;
        
        return PriorityKey{k1, k2};
    }

    // Check if queue is empty
    bool DStarLite::openEmpty() const {
        return open_.empty();
    }
    // check number of entries in queue
    std::size_t DStarLite::openSize() const {
        return open_.size();
    }
    //view smallest-key entry
    QueueEntry DStarLite::openTop() const {
        if (open_.empty()) {
            throw std::logic_error(
                "D* Lite priority queue is empty."
            );
        }

        return open_.top();
    }



