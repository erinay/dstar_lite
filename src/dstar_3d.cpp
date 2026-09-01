#include "dstar_3d.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace dstar_3d {
namespace {

bool approximatelyEqual(double left, double right) {
    if (left == right) {
        return true;
    }
    if (!std::isfinite(left) || !std::isfinite(right)) {
        return false;
    }
    return std::abs(left - right) <= kEpsilon;
}

bool traversable(const Grid& grid, const Coord& voxel) {
    return grid.inBounds(voxel) && std::isfinite(grid.traversalCost(voxel));
}

} // namespace

bool Coord::operator==(const Coord& other) const {
    return x == other.x && y == other.y && z == other.z;
}

std::size_t CoordHash::operator()(const Coord& coord) const {
    const std::size_t hash_x = std::hash<int>{}(coord.x);
    const std::size_t hash_y = std::hash<int>{}(coord.y);
    const std::size_t hash_z = std::hash<int>{}(coord.z);
    const std::size_t xy = hash_x ^ (hash_y + 0x9e3779b9 + (hash_x << 6) + (hash_x >> 2));
    return xy ^ (hash_z + 0x9e3779b9 + (xy << 6) + (xy >> 2));
}

Grid::Grid(int width, int height, int depth, double resolution)
    : width_(width),
      height_(height),
      depth_(depth),
      resolution_(resolution) {
    if (width <= 0 || height <= 0 || depth <= 0) {
        throw std::invalid_argument("Grid dimensions must be positive.");
    }
    if (!std::isfinite(resolution) || resolution <= 0.0) {
        throw std::invalid_argument("Grid resolution must be finite and positive.");
    }

    const std::size_t cell_count = static_cast<std::size_t>(width_) *
        static_cast<std::size_t>(height_) * static_cast<std::size_t>(depth_);
    cells_.assign(cell_count, kUnknown);
    traversal_costs_.assign(cell_count, 1.0);
}

bool Grid::inBounds(const Coord& voxel) const {
    return voxel.x >= 0 && voxel.x < width_ &&
           voxel.y >= 0 && voxel.y < height_ &&
           voxel.z >= 0 && voxel.z < depth_;
}

std::size_t Grid::index(const Coord& voxel) const {
    return (static_cast<std::size_t>(voxel.z) * static_cast<std::size_t>(height_) +
            static_cast<std::size_t>(voxel.y)) * static_cast<std::size_t>(width_) +
           static_cast<std::size_t>(voxel.x);
}

void Grid::checkBounds(const Coord& voxel) const {
    if (!inBounds(voxel)) {
        throw std::out_of_range("Coordinate is outside the 3D grid.");
    }
}

int Grid::state(const Coord& voxel) const {
    checkBounds(voxel);
    return cells_[index(voxel)];
}

void Grid::setState(const Coord& voxel, int state) {
    checkBounds(voxel);
    cells_[index(voxel)] = state;
}

void Grid::setTraversalCost(const Coord& voxel, double cost) {
    checkBounds(voxel);
    if (!std::isfinite(cost) || cost < 1.0) {
        throw std::invalid_argument("Traversal cost must be finite and at least one.");
    }
    traversal_costs_[index(voxel)] = cost;
}

std::vector<Coord> Grid::neighbors(const Coord& voxel) const {
    checkBounds(voxel);

    const std::array<Coord, 6> candidates{{
        {voxel.x - 1, voxel.y, voxel.z},
        {voxel.x + 1, voxel.y, voxel.z},
        {voxel.x, voxel.y - 1, voxel.z},
        {voxel.x, voxel.y + 1, voxel.z},
        {voxel.x, voxel.y, voxel.z - 1},
        {voxel.x, voxel.y, voxel.z + 1},
    }};

    std::vector<Coord> result;
    result.reserve(candidates.size());
    for (const Coord& candidate : candidates) {
        if (inBounds(candidate)) {
            result.push_back(candidate);
        }
    }
    return result;
}

double Grid::traversalCost(const Coord& voxel) const {
    checkBounds(voxel);
    if (state(voxel) == kOccupied) {
        return kInfinity;
    }
    return traversal_costs_[index(voxel)];
}

int Grid::width() const {
    return width_;
}

int Grid::height() const {
    return height_;
}

int Grid::depth() const {
    return depth_;
}

double Grid::resolution() const {
    return resolution_;
}

bool keyLess(const PriorityKey& left, const PriorityKey& right) {
    if (left.k1 < right.k1 - kEpsilon) {
        return true;
    }
    if (left.k1 > right.k1 + kEpsilon) {
        return false;
    }
    return left.k2 < right.k2 - kEpsilon;
}

bool QueueEntryCompare::operator()(const QueueEntry& left, const QueueEntry& right) const {
    return keyLess(right.key, left.key);
}

DStarLite3D::DStarLite3D(Grid& grid, const Coord& start, const Coord& goal)
    : grid_(grid), start_(start), previous_start_(start), goal_(goal) {
    if (!traversable(grid_, start_) || !traversable(grid_, goal_)) {
        throw std::invalid_argument("Start and goal must be in-bounds, traversable voxels.");
    }
    initialize();
}

void DStarLite3D::initialize() {
    for (int z = 0; z < grid_.depth(); ++z) {
        for (int y = 0; y < grid_.height(); ++y) {
            for (int x = 0; x < grid_.width(); ++x) {
                nodes_.emplace(Coord{x, y, z}, NodeData{});
            }
        }
    }

    nodes_.at(goal_).rhs = 0.0;
    open_.push(QueueEntry{goal_, calculateKey(goal_)});
}

const NodeData& DStarLite3D::data(const Coord& voxel) const {
    return nodes_.at(voxel);
}

double DStarLite3D::g(const Coord& voxel) const {
    return data(voxel).g;
}

double DStarLite3D::rhs(const Coord& voxel) const {
    return data(voxel).rhs;
}

PriorityKey DStarLite3D::calculateKey(const Coord& voxel) const {
    const double min_cost = std::min(g(voxel), rhs(voxel));
    return PriorityKey{min_cost + heuristic(start_, voxel) + km_, min_cost};
}

const Coord& DStarLite3D::start() const {
    return start_;
}

const Coord& DStarLite3D::previousStart() const {
    return previous_start_;
}

const Coord& DStarLite3D::goal() const {
    return goal_;
}

double DStarLite3D::km() const {
    return km_;
}

double DStarLite3D::heuristic(const Coord& a, const Coord& b) {
    return static_cast<double>(
        std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z));
}

double DStarLite3D::cost(const Coord& from, const Coord& to) const {
    if (!grid_.inBounds(from) || !grid_.inBounds(to) || heuristic(from, to) != 1.0) {
        return kInfinity;
    }
    return grid_.traversalCost(to);
}

void DStarLite3D::updateVertex(const Coord& voxel) {
    NodeData& node = nodes_.at(voxel);

    if (!(voxel == goal_)) {
        double best_rhs = kInfinity;
        for (const Coord& successor : grid_.neighbors(voxel)) {
            best_rhs = std::min(best_rhs, cost(voxel, successor) + g(successor));
        }
        node.rhs = best_rhs;
    }

    // Stale entries are ignored when popped. This avoids requiring a priority
    // queue with arbitrary-element deletion.
    if (!approximatelyEqual(node.g, node.rhs)) {
        open_.push(QueueEntry{voxel, calculateKey(voxel)});
    }
}

void DStarLite3D::computeShortestPath() {
    while (!open_.empty() &&
           (keyLess(open_.top().key, calculateKey(start_)) ||
            !approximatelyEqual(g(start_), rhs(start_)))) {
        const QueueEntry entry = open_.top();
        open_.pop();

        const PriorityKey new_key = calculateKey(entry.state);
        if (keyLess(entry.key, new_key)) {
            if (!approximatelyEqual(g(entry.state), rhs(entry.state))) {
                open_.push(QueueEntry{entry.state, new_key});
            }
            continue;
        }

        NodeData& node = nodes_.at(entry.state);
        if (approximatelyEqual(node.g, node.rhs)) {
            continue;
        }

        if (node.g > node.rhs) {
            node.g = node.rhs;
            for (const Coord& predecessor : grid_.neighbors(entry.state)) {
                updateVertex(predecessor);
            }
        } else {
            node.g = kInfinity;
            updateVertex(entry.state);
            for (const Coord& predecessor : grid_.neighbors(entry.state)) {
                updateVertex(predecessor);
            }
        }
    }

    if (!approximatelyEqual(g(start_), rhs(start_))) {
        throw std::runtime_error("D* Lite queue emptied before the start became consistent.");
    }
}

std::vector<Coord> DStarLite3D::extractPath() const {
    if (!traversable(grid_, start_) || !traversable(grid_, goal_) ||
        !std::isfinite(g(start_))) {
        return {};
    }

    std::vector<Coord> path{start_};
    if (start_ == goal_) {
        return path;
    }

    Coord current = start_;
    std::unordered_set<Coord, CoordHash> visited{current};
    const std::size_t max_steps =
        static_cast<std::size_t>(grid_.width()) * static_cast<std::size_t>(grid_.height()) *
        static_cast<std::size_t>(grid_.depth());

    for (std::size_t step = 0; step < max_steps && !(current == goal_); ++step) {
        Coord best_successor{};
        double best_cost = kInfinity;
        bool found_successor = false;

        for (const Coord& successor : grid_.neighbors(current)) {
            const double candidate = cost(current, successor) + g(successor);
            if (candidate < best_cost - kEpsilon) {
                best_cost = candidate;
                best_successor = successor;
                found_successor = true;
            }
        }

        if (!found_successor || !visited.insert(best_successor).second) {
            return {};
        }

        current = best_successor;
        path.push_back(current);
    }

    return current == goal_ ? path : std::vector<Coord>{};
}

void DStarLite3D::moveStart(const Coord& new_start) {
    if (!traversable(grid_, new_start)) {
        throw std::invalid_argument("New start must be an in-bounds, traversable voxel.");
    }
    km_ += heuristic(start_, new_start);
    previous_start_ = start_;
    start_ = new_start;
}

void DStarLite3D::updateCellState(const Coord& voxel, int new_state) {
    if (!grid_.inBounds(voxel)) {
        throw std::out_of_range("Coordinate is outside the 3D grid.");
    }

    const double old_cost = grid_.traversalCost(voxel);
    grid_.setState(voxel, new_state);
    const double new_cost = grid_.traversalCost(voxel);

    if (approximatelyEqual(old_cost, new_cost)) {
        return;
    }

    for (const Coord& predecessor : grid_.neighbors(voxel)) {
        updateVertex(predecessor);
    }
}

void DStarLite3D::updateCell(const Coord& voxel, int new_state, double traversal_cost) {
    if (!grid_.inBounds(voxel)) {
        throw std::out_of_range("Coordinate is outside the 3D grid.");
    }
    const double old_cost = grid_.traversalCost(voxel);
    grid_.setState(voxel, new_state);
    grid_.setTraversalCost(voxel, traversal_cost);
    const double new_cost = grid_.traversalCost(voxel);

    if (approximatelyEqual(old_cost, new_cost)) {
        return;
    }

    // Edge costs are defined by their destination voxel, so only incoming
    // edges (the voxel's six predecessors) need a rhs recomputation.
    for (const Coord& predecessor : grid_.neighbors(voxel)) {
        updateVertex(predecessor);
    }
}

bool DStarLite3D::openEmpty() const {
    return open_.empty();
}

std::size_t DStarLite3D::openSize() const {
    return open_.size();
}

QueueEntry DStarLite3D::openTop() const {
    if (open_.empty()) {
        throw std::logic_error("D* Lite priority queue is empty.");
    }
    return open_.top();
}

} // namespace dstar_3d
