#pragma once

#include <cstddef>
#include <vector>

// TESTING IN 2D WORLD
struct Coord {
    int x;
    int y;

    // Compare coordinate objects
    bool operator==(const Coord& other) const;
};

struct CoordHash {
    std::size_t operator()(const Coord& coord) const;
};

// Make Grid class
// -1 = unknown, 0 = free, 1 = occupied
class Grid {
private:
    int width_;
    int height_;
    double resolution_;
    std::vector<int> cells_;

    // Convert a 2D coordinate to a vector index
    std::size_t index(const Coord& s) const;

    // Throw an error when a coordinate is outside the grid
    void checkBounds(const Coord& s) const;

    // Convert a cell state to its printed character
    static char cellCharacter(int cell_state);

    // Check whether a coordinate is part of a path
    static bool isInPath(
        const Coord& s,
        const std::vector<Coord>& path
    );

public:
    Grid(int width, int height, double resolution);

    // Bounds check
    bool inBounds(const Coord& s) const;

    // Cell access
    int state(const Coord& s) const;
    void setState(const Coord& s, int state);

    // Four-connected neighbors
    std::vector<Coord> neighbors(const Coord& s) const;

    // Free/unknown cost = 1; occupied cost = infinity
    double traversalCost(const Coord& s) const;

    // Print grid
    void print(
        const Coord& start,
        const Coord& goal,
        const std::vector<Coord>& path = {}
    ) const;

    // Helper getters
    int width() const;
    int height() const;
    double resolution() const;
};