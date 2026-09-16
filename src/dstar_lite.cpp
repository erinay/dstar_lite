#include "dstar_lite.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <unordered_set>

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

    double DStarLite::cost(const Coord& from, const Coord& to) const{
        if (!grid_.inBounds(from) || !grid_.inBounds(to)) {
            return INF;
        }
        if (heuristic(from, to) != 1.0) {
            return INF;
        }

        return grid_.traversalCost(to); //traversal cost returns INF if occupied, 1 otherwise
    }


    PriorityKey DStarLite::calculateKey(const Coord &s) const{
        const NodeData& node = data(s);
        const double min_cost = std::min(node.g, node.rhs);
        const double k1 = min_cost+heuristic(start_, s)+km_;
        const double k2 = min_cost;
        
        return PriorityKey{k1, k2};
    }

    bool approximatelyEqual(double a, double b) {
        if (a == b) {
            return true;
        }
        if(!std::isfinite(a) || !std::isfinite(b)){
            return false;
        }
        return std::abs(a - b) <= EPS;
    }

    void DStarLite::updateVertex(const Coord& s) {
        // Goal: recompute rhs() from neighbors, checkes if cell is inconsistent, if inconsistent, inserts new queue entry with current key
        NodeData& node = nodes_.at(s);
        
        //  checks if g, rhs is consistent or not, removal will be done later
        if (!(s==goal_)) {
            double best_rhs=INF;
            
            for (const Coord& successor: grid_.neighbors(s)){
                const double candidate = cost(s, successor)+ g(successor);
                best_rhs = std::min(best_rhs, candidate);
            }
            
            node.rhs = best_rhs;
        }

        if(!approximatelyEqual(node.g, node.rhs)){
            open_.push(QueueEntry{s, calculateKey(s)});
        }

    }

    void DStarLite::computeShortestPath(){
        // push out lowest inconsistent cell (ignore if equal) that's less than key to start position
        constexpr std::size_t MAX_ITER = 1'000'000;
        constexpr bool DEBUG=false; //adding max iteraion guard until this code is good to go

        std::size_t iters = 0;

        while(true){
            const PriorityKey start_key = calculateKey(start_);
            const bool inconsistent_start = !(approximatelyEqual(g(start_), rhs(start_)));

            // stop if start rhs != g open_.top().key < calcKey)start_)
            if (open_.empty()){
                if(inconsistent_start){
                    throw std::runtime_error("Priority queue empty, start is still inconsistent!");
                };
                break;
            }

            const QueueEntry top = open_.top();
            const bool top_key_smaller = keyLess(top.key, start_key);

            if(!top_key_smaller and !inconsistent_start){
                break;
            }
            
            ++iters;
            if(iters>MAX_ITER){
                throw std::runtime_error("Exceeded max iterations");
            }

            // Remove top entry
            open_.pop();
            const Coord u = top.state;
            const PriorityKey kold = top.key;
            const PriorityKey knew = calculateKey(u);

            if constexpr (DEBUG) {
            std::cout
                << "Expanding ("
                << u.x
                << ", "
                << u.y
                << ")"
                << " g=" << g(u)
                << " rhs=" << rhs(u)
                << '\n';
            }
            
            // Deletion Handling (since we never remove u)
            // And first case: kold < calcKey(u):push to queue
            if(keyLess(kold, knew)){
                if(!approximatelyEqual(g(u), rhs(u))){
                    open_.push(QueueEntry{u,knew});
                }
                continue;
            }
            if(approximatelyEqual(g(u), rhs(u))){
                continue;
            }

            NodeData& node = nodes_.at(u); //get node data from hash map
            // Case2: g(u)>rhs(u): set g(u)=rhs(u), update vertex to all poredecessors of u
            if(node.g>node.rhs+EPS){
                node.g=node.rhs;
                for(const Coord& Pred: grid_.neighbors(u)){
                    updateVertex(Pred);
                }
            }

            // Case 3: else, g(u)=infty,[old g is too optimistic, implies path is blocked]
            // for all predecessors and u, update vertex
            else{
                node.g=INF;
                updateVertex(u);

                for(const Coord& Pred: grid_.neighbors(u)){
                    updateVertex(Pred);
                }
            }
        }
    }


    std::vector<Coord> DStarLite::extractPath() const{
        std::vector<Coord> path;

        path.push_back(start_);
        if(start_==goal_){
            return path;
        }

        // no known path to goal
        if(!std::isfinite(g(start_))){
            return {};
        }

        Coord current = start_;

        // Keep track of visited nodes to prevent loops
        std::unordered_set<Coord, CoordHash> visited;
        visited.insert(current);

        const std::size_t max_iters= (std::size_t) (grid_.width() * grid_.height());
        std::size_t iters = 0;

        while(!(current==goal_)){
            bool found_successor=false;
            Coord best_successor{0,0};
            double best_cost_to_goal=INF;

            // iterate through successors to get next path
            for(const Coord& successor: grid_.neighbors(current)){
                const double candidate = cost(current, successor)+g(successor);

                //check if cost is inifinite, if so, we can't go there, 
                if(!std::isfinite(candidate)){
                    continue;
                }

                if(!found_successor || candidate<best_cost_to_goal-EPS){
                    best_cost_to_goal=candidate;
                    best_successor = successor;
                    found_successor = true;
                }
            }

            if(!found_successor){
                return {}; //valid  path does not exist
            }

            const bool inserted = visited.insert(best_successor).second;
            if (!inserted){
                return {}; //already visited, loop start
            }

            current=best_successor;
            path.push_back(current);
            ++iters;

            if(iters>max_iters){
                return {}; //if we have visited more than the given number of cells, we have issues!
            }
        }
        return path;
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

    void DStarLite::updateCellState(const Coord& cell, int new_state){
        const int old_state = grid_.state(cell);

        if (old_state==new_state){
            return; //do nothing
        }


        // check if cost changes
        const double old_cost = grid_.traversalCost(cell);
        grid_.setState(cell, new_state);
        const double new_cost = grid_.traversalCost(cell);

        if (approximatelyEqual(old_cost, new_cost)) {
            return;
        }

        // if any edge cost changes, km = km+h(s_last, s_start) and slast=s_start;
        //***actually, skip this for now, static map */

        //all directed edges with changed edge cost, update edge cost c(u,v), update vertex and nieghbors
        updateVertex(cell);
        for(const Coord& neighbor: grid_.neighbors(cell)){
            updateVertex(neighbor);
        }

    }

    void DStarLite::updateCell(
        const Coord& cell, int new_state, double traversal_cost)
    {
        const double old_cost = grid_.traversalCost(cell);
        grid_.setState(cell, new_state);
        grid_.setTraversalCost(cell, traversal_cost);
        const double new_cost = grid_.traversalCost(cell);

        if (approximatelyEqual(old_cost, new_cost)) {
            return;
        }

        // Changing the cost of entering this cell changes each edge that
        // terminates here, so refresh its predecessors...
        // BUGFIX: ...and refresh `cell` itself too -- see identical note in updateCellState above.
        // Without this, a cell that reverts from occupied/blocked back to free can permanently
        // strand at g=rhs=inf even though it (and its neighbors) are genuinely traversable again.
        updateVertex(cell);
        for (const Coord& neighbor : grid_.neighbors(cell)) {
            updateVertex(neighbor);
        }
    }

    void DStarLite::moveStart(const Coord& new_start){
        const Coord& prev_start = start_;
        // update km = km+h(s_ast, s_start)', let h be manhatten distance 
        km_ += heuristic(prev_start, new_start);
        previous_start_ = prev_start;
        start_ = new_start;

        // Refresh the new start cell and its neighbors here, the
        // moment the robot arrives, using the same self+neighbors pattern as updateCell/updateCellState.
        updateVertex(start_);
        for(const Coord& neighbor : grid_.neighbors(start_)){
            updateVertex(neighbor);
        }
    }


