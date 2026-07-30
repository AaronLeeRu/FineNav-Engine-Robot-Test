// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_robot_bringup/mppi_map_view.hpp"

#include <algorithm>
#include <cmath>

namespace finenav::robot_bringup {

MPPIMapView::MPPIMapView(const OccupancyGrid2D& map) : map_(map) {
    if (map_.hasMap()) {
        const auto& info = map_.grid().info;
        resolution_ = static_cast<double>(info.resolution);
        width_  = info.width;
        height_ = info.height;
    }
}

bool MPPIMapView::worldToGrid(double wx, double wy,
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

int MPPIMapView::cellCost(unsigned int ix, unsigned int iy) const {
    if (ix >= width_ || iy >= height_) return -1;
    return static_cast<int>(map_.grid().data[iy * width_ + ix]);
}

// ── IMapView interface ──────────────────────────────────────────────────

bool MPPIMapView::isTrackingUnknown() const {
    return track_unknown_;
}

bool MPPIMapView::considerFootprint() const {
    return true;
}

bool MPPIMapView::isCollision(float x, float y, float /*theta*/) const {
    if (!map_.hasMap()) return true;  // no map → treat as collision to be safe

    int threshold = map_.config().occupancy_threshold;
    int radius_cells = static_cast<int>(std::ceil(robot_radius_ / resolution_));

    unsigned int cx, cy;
    if (!worldToGrid(static_cast<double>(x), static_cast<double>(y), cx, cy)) {
        return true;  // outside map → collision
    }

    // Circular footprint: sample cells within radius
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * resolution_;
            if (dist > robot_radius_) continue;

            int gx = static_cast<int>(cx) + dx;
            int gy = static_cast<int>(cy) + dy;
            if (gx < 0 || gx >= static_cast<int>(width_) ||
                gy < 0 || gy >= static_cast<int>(height_)) {
                return true;  // footprint extends off map
            }

            int8_t val = map_.grid().data[gy * width_ + gx];
            if (val < 0 && !track_unknown_) continue;  // unknown, skip
            if (val >= threshold) return true;          // obstacle hit
        }
    }
    return false;
}

float MPPIMapView::getRadius() const {
    return robot_radius_;
}

int MPPIMapView::getCost(const Position3D& pos) const {
    if (!map_.hasMap()) return 0;
    unsigned int ix, iy;
    if (!worldToGrid(pos.x(), pos.y(), ix, iy)) return 254;  // out-of-bounds = lethal
    int8_t val = map_.grid().data[iy * width_ + ix];
    if (val < 0) return track_unknown_ ? 0 : 254;
    // Scale [0,100] → [0, 254] for MPPI CostCritic
    return static_cast<int>(std::min(254.0f, val * 2.54f));
}

float MPPIMapView::costAtPose(float x, float y, float /*theta*/) const {
    if (!map_.hasMap()) return 254.0f;

    int radius_cells = static_cast<int>(std::ceil(robot_radius_ / resolution_));
    unsigned int cx, cy;
    if (!worldToGrid(static_cast<double>(x), static_cast<double>(y), cx, cy)) {
        return 254.0f;
    }

    float max_cost = 0.0f;
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * resolution_;
            if (dist > robot_radius_) continue;

            int gx = static_cast<int>(cx) + dx;
            int gy = static_cast<int>(cy) + dy;
            if (gx < 0 || gx >= static_cast<int>(width_) ||
                gy < 0 || gy >= static_cast<int>(height_)) {
                return 254.0f;
            }

            int8_t val = map_.grid().data[gy * width_ + gx];
            float cost;
            if (val < 0) {
                cost = track_unknown_ ? 0.0f : 254.0f;
            } else {
                cost = std::min(254.0f, static_cast<float>(val) * 2.54f);
            }
            max_cost = std::max(max_cost, cost);
        }
    }
    return max_cost;
}

std::string MPPIMapView::getBaseFrameID() const {
    return "base_link";
}

}  // namespace finenav::robot_bringup
