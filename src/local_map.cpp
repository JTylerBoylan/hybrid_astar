#include <local_planner/LocalPlanner.hpp>

using namespace local_planner;

LocalMap::LocalMap() {

    // Set size
    size_x = 10.0;
    size_y = 10.0;

    // Set resolution
    resolution_x = 100;
    resolution_y = 100;

    // Create new map buffer
    map = new bool[resolution_x * resolution_y];

}

LocalMap::~LocalMap() {
    delete[] map;
}

void LocalMap::index(const float x, const float y, int& r, int& c) {
    r = resolution_y / 2 - int(resolution_y * y / size_y);
    c = resolution_x / 2 - int(resolution_x * x / size_x);
}

void LocalMap::traversable(const float x, const float y, const bool b) {
    int r, c;
    index(x,y,r,c);
    map[index(r,c)] = b;
}

bool LocalMap::traversable(const float x, const float y) {
    int r,c;
    index(x,y,r,c);
    return map[index(r,c)];
}

int LocalMap::index(int r, int c) {
    return r * resolution_y + c;
}

