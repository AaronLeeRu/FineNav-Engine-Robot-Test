// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_robot_bringup/fused_mppi_map_view.hpp"

#include <algorithm>
#include <cmath>

namespace finenav::robot_bringup {

// ==============================================================================
// Constructor
// ==============================================================================

FusedMppiMapView::FusedMppiMapView(
    const OccupancyGrid2D& occ_map,
    finenav::core::LockedMapRO<TemporalVoxelMap>&& temporal_lock)
    : occ_map_(occ_map), temporal_lock_(std::move(temporal_lock))
{
    if (occ_map_.hasMap()) {
        const auto& info = occ_map_.grid().info;
        resolution_ = static_cast<double>(info.resolution);
        width_      = info.width;
        height_     = info.height;
        origin_x_   = info.origin.position.x;
        origin_y_   = info.origin.position.y;
    }
}

// ==============================================================================
// Coordinate conversion
// ==============================================================================

bool FusedMppiMapView::worldToGrid(double wx, double wy,
                                   unsigned int& ix, unsigned int& iy) const {
    if (!occ_map_.hasMap()) return false;
    int gx = static_cast<int>((wx - origin_x_) / resolution_);
    int gy = static_cast<int>((wy - origin_y_) / resolution_);
    if (gx < 0 || gx >= static_cast<int>(width_) ||
        gy < 0 || gy >= static_cast<int>(height_)) {
        return false;
    }
    ix = static_cast<unsigned int>(gx);
    iy = static_cast<unsigned int>(gy);
    return true;
}

// ==============================================================================
// Static grid helpers
// ==============================================================================

int FusedMppiMapView::staticCellCost(unsigned int ix, unsigned int iy) const {
    if (ix >= width_ || iy >= height_) return -1;
    return static_cast<int>(occ_map_.grid().data[iy * width_ + ix]);
}

// ==============================================================================
// Temporal voxel helpers
// ==============================================================================

bool FusedMppiMapView::temporalCollisionInFootprint(
    unsigned int cx, unsigned int cy, int radius_cells,
    bool treat_unknown_as_collision) const
{
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * resolution_;
            if (dist > robot_radius_) continue;

            int gx = static_cast<int>(cx) + dx;
            int gy = static_cast<int>(cy) + dy;
            if (gx < 0 || gx >= static_cast<int>(width_) ||
                gy < 0 || gy >= static_cast<int>(height_)) {
                if (treat_unknown_as_collision) return true;
                continue;
            }

            // Sample temporal map at this world position
            double wx = origin_x_ + gx * resolution_ + resolution_ * 0.5;
            double wy = origin_y_ + gy * resolution_ + resolution_ * 0.5;

            if (temporal_lock_->isDynamicObstacleInColumn(wx, wy)) {
                return true;
            }
        }
    }
    return false;
}

float FusedMppiMapView::temporalMaxCostInFootprint(
    unsigned int cx, unsigned int cy, int radius_cells,
    bool treat_unknown_as_collision) const
{
    float max_cost = 0.0f;

    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * resolution_;
            if (dist > robot_radius_) continue;

            int gx = static_cast<int>(cx) + dx;
            int gy = static_cast<int>(cy) + dy;
            if (gx < 0 || gx >= static_cast<int>(width_) ||
                gy < 0 || gy >= static_cast<int>(height_)) {
                if (treat_unknown_as_collision) return 254.0f;
                continue;
            }

            double wx = origin_x_ + gx * resolution_ + resolution_ * 0.5;
            double wy = origin_y_ + gy * resolution_ + resolution_ * 0.5;

            if (temporal_lock_->isDynamicObstacleInColumn(wx, wy)) {
                return 254.0f;  // dynamic obstacle = lethal
            }
        }
    }
    return max_cost;
}

// ==============================================================================
// IMapView interface
// ==============================================================================

bool FusedMppiMapView::isTrackingUnknown() const {
    return track_unknown_;
}

bool FusedMppiMapView::considerFootprint() const {
    return true;
}

bool FusedMppiMapView::isCollision(float x, float y, float /*theta*/) const {
    if (!occ_map_.hasMap()) return true;  // no map → treat as collision to be safe

    int threshold = occ_map_.config().occupancy_threshold;
    int radius_cells = static_cast<int>(std::ceil(robot_radius_ / resolution_));

    unsigned int cx, cy;
    if (!worldToGrid(static_cast<double>(x), static_cast<double>(y), cx, cy)) {
        return true;  // outside map → collision
    }

    // ── Check static occupancy grid (circular footprint) ──────────────
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

            int8_t val = occ_map_.grid().data[gy * width_ + gx];
            if (val < 0 && !track_unknown_) continue;  // unknown, skip
            if (val >= threshold) return true;          // static obstacle hit
        }
    }

    // ── Check temporal dynamic obstacles ──────────────────────────────
    if (temporalCollisionInFootprint(cx, cy, radius_cells, /*treat_unknown_as_collision=*/false)) {
        return true;
    }

    return false;
}

float FusedMppiMapView::getRadius() const {
    return robot_radius_;
}

int FusedMppiMapView::getCost(const Position3D& pos) const {
    // ── 1. Static occupancy grid cost ─────────────────────────────────
    int static_cost = 0;
    if (!occ_map_.hasMap()) return 0;  // no map → treat as free

    unsigned int ix, iy;
    if (worldToGrid(pos.x(), pos.y(), ix, iy)) {
        int8_t val = occ_map_.grid().data[iy * width_ + ix];
        if (val < 0) {
            static_cost = track_unknown_ ? 0 : 254;
        } else {
            static_cost = static_cast<int>(std::min(254.0f, val * 2.54f));
        }
    } else {
        static_cost = 254;  // out-of-bounds = lethal
    }

    // ── 2. Temporal dynamic obstacle cost ─────────────────────────────
    // Sweep the robot-height column (0 → 2 m) — MPPI queries in 2D,
    // but point-cloud obstacles may be at any elevation.
    int temporal_cost = 0;
    if (temporal_lock_->isDynamicObstacleInColumn(pos.x(), pos.y())) {
        temporal_cost = 254;  // dynamic obstacle = lethal
    }

    return std::max(static_cost, temporal_cost);
}

float FusedMppiMapView::costAtPose(float x, float y, float /*theta*/) const {
    if (!occ_map_.hasMap()) return 254.0f;

    int radius_cells = static_cast<int>(std::ceil(robot_radius_ / resolution_));

    unsigned int cx, cy;
    if (!worldToGrid(static_cast<double>(x), static_cast<double>(y), cx, cy)) {
        return 254.0f;
    }

    float max_cost = 0.0f;

    // ── Scan footprint over static grid ───────────────────────────────
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

            int8_t val = occ_map_.grid().data[gy * width_ + gx];
            float cost;
            if (val < 0) {
                cost = track_unknown_ ? 0.0f : 254.0f;
            } else {
                cost = std::min(254.0f, static_cast<float>(val) * 2.54f);
            }
            max_cost = std::max(max_cost, cost);
        }
    }

    // ── Fuse temporal dynamic cost ────────────────────────────────────
    float temporal_max = temporalMaxCostInFootprint(
        cx, cy, radius_cells, /*treat_unknown_as_collision=*/false);
    max_cost = std::max(max_cost, temporal_max);

    return max_cost;
}

std::string FusedMppiMapView::getBaseFrameID() const {
    return "base_link";
}

}  // namespace finenav::robot_bringup
