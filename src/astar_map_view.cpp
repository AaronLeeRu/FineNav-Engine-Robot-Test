// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_robot_bringup/astar_map_view.hpp"

namespace finenav::robot_bringup {

AstarMapView::AstarMapView(const OccupancyGrid2D& map) : map_(map) {
    if (map_.hasMap()) {
        const auto& info = map_.grid().info;
        resolution_ = static_cast<double>(info.resolution);
        width_  = info.width;
        height_ = info.height;
    }
}

bool AstarMapView::worldToGrid(double wx, double wy,
                               unsigned int& ix, unsigned int& iy) const {
    if (!map_.hasMap()) return false;
    const auto& origin = map_.grid().info.origin.position;
    int gx = static_cast<int>((wx - origin.x) / resolution_);
    int gy = static_cast<int>((wy - origin.y) / resolution_);
    if (gx < 0 || gx >= static_cast<int>(width_) ||
        gy < 0 || gy >= static_cast<int>(height_)) {
        return false;
    }
    ix = static_cast<unsigned int>(gx);
    iy = static_cast<unsigned int>(gy);
    return true;
}

bool AstarMapView::isWalkable(const core::Position3D& p) const {
    if (!map_.hasMap()) return false;
    unsigned int ix, iy;
    if (!worldToGrid(p.x(), p.y(), ix, iy)) return false;
    const auto& grid = map_.grid();
    int8_t val = grid.data[iy * width_ + ix];
    // Treat unknown (-1) as blocked; occupied (>= threshold) as blocked.
    int threshold = map_.config().occupancy_threshold;
    return val >= 0 && val < threshold;
}

double AstarMapView::getCost(const core::Position3D& p) const {
    if (!map_.hasMap()) return 0.0;
    unsigned int ix, iy;
    if (!worldToGrid(p.x(), p.y(), ix, iy)) return 1.0;  // out-of-bounds = high cost
    const auto& grid = map_.grid();
    int8_t val = grid.data[iy * width_ + ix];
    if (val < 0) return 0.5;     // unknown — medium cost
    // Normalise [0,100] → [0.0, 1.0]
    return std::min(1.0, static_cast<double>(val) / 100.0);
}

double AstarMapView::getStepSize() const {
    return resolution_;
}

}  // namespace finenav::robot_bringup
