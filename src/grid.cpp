#include "grid.hpp"

#include <algorithm>  // std::find
#include <functional> // std::hash
#include <iostream>
#include <limits>
#include <stdexcept>
#include <cassert>

bool Coord::operator==(const Coord& other) const {
    if (x == other.x && y == other.y) {
        return true;
    }

    return false;
}

std::size_t CoordHash::operator()(const Coord& coord) const {
    // Get hashes for individual member variables.
    const std::size_t hash_x = std::hash<int>{}(coord.x);
    const std::size_t hash_y = std::hash<int>{}(coord.y);

    // Mix both hashes into one hash value.
    // This reduces collisions but cannot guarantee none occur.
    return hash_x ^
           (
               hash_y +
               0x9e3779b9 +
               (hash_x << 6) +
               (hash_x >> 2)
           );
}

Grid::Grid(int width, int height, double resolution)
    : width_(width),
      height_(height),
      resolution_(resolution),
      cells_(width * height, -1)
{
}

std::size_t Grid::index(const Coord& s) const {
    return static_cast<std::size_t>(
        s.y * width_ + s.x
    );
}

void Grid::checkBounds(const Coord& s) const {
    if (!inBounds(s)) {
        throw std::out_of_range(
            "Coordinate is outside the grid."
        );
    }
}

char Grid::cellCharacter(int cell_state) {
    if (cell_state == -1) {
        return '?';
    } else if (cell_state == 0) {
        return '.';
    } else if (cell_state == 1) {
        return '#';
    } else {
        return '!';
    }
}

bool Grid::isInPath(
    const Coord& s,
    const std::vector<Coord>& path
) {
    return std::find(
        path.begin(),
        path.end(),
        s
    ) != path.end();
}

bool Grid::inBounds(const Coord& s) const {
    return s.x >= 0 &&
           s.x < width_ &&
           s.y >= 0 &&
           s.y < height_;
}

int Grid::state(const Coord& s) const {
    checkBounds(s);
    return cells_[index(s)];
}

void Grid::setState(const Coord& s, int state) {
    checkBounds(s);
    cells_[index(s)] = state;
}

std::vector<Coord> Grid::neighbors(const Coord& s) const {
    std::vector<Coord> all_neighbors;

    const std::vector<Coord> candidates = {
        {s.x - 1, s.y},
        {s.x + 1, s.y},
        {s.x, s.y - 1},
        {s.x, s.y + 1}
    };

    // Make sure every returned neighbor is in bounds.
    for (const Coord& candidate : candidates) {
        if (inBounds(candidate)) {
            all_neighbors.push_back(candidate);
        }
    }

    return all_neighbors;
}

double Grid::traversalCost(const Coord& s) const {
    checkBounds(s);

    if (state(s) == 1) {
        return std::numeric_limits<double>::infinity();
    } else if(state(s)==-1){
        return 1.5;
    } else {
        return 1.0
    }
}

void Grid::print(
    const Coord& start,
    const Coord& goal,
    const std::vector<Coord>& path
) const {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            const Coord current{x, y};

            if (current == start) {
                std::cout << 'S';
            } else if (current == goal) {
                std::cout << 'G';
            } else if (isInPath(current, path)) {
                std::cout << '*';
            } else {
                std::cout << cellCharacter(state(current));
            }
        }

        std::cout << '\n';
    }
}

int Grid::width() const {
    return width_;
}

int Grid::height() const {
    return height_;
}

double Grid::resolution() const {
    return resolution_;
}