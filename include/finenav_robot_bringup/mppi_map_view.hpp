// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_mppi_controller/map_view_def.hpp"
#include "finenav_map_occupancy_grid_2d/occupancy_grid_2d.hpp"

namespace finenav::robot_bringup {

/**
 * @brief Adapts OccupancyGrid2D to nav2_mppi_controller::IMapView for MPPI control.
 *
 * Provides cost queries and circular-footprint collision checking over the
 * OccupancyGrid data. Footprint checks use a configurable robot radius.
 */
class MPPIMapView : public nav2_mppi_controller::IMapView {
public:
    explicit MPPIMapView(const OccupancyGrid2D& map);

    // ── IMapView interface ──────────────────────────────────────────────

    bool isTrackingUnknown() const override;
    bool considerFootprint() const override;

    /// Check whether the circular footprint at (x, y, theta) collides with any obstacle.
    bool isCollision(float x, float y, float theta) const override;

    /// Radius of the circular robot footprint (metres), used by CostCritic.
    float getRadius() const override;

    /// Raw cost at a world position (centre-point query).
    int getCost(const Position3D& pos) const override;

    /// Maximum cost under the footprint at (x, y, theta).
    float costAtPose(float x, float y, float theta) const override;

    /// TF frame of the robot base.
    std::string getBaseFrameID() const override;

    // ── Tuning ──────────────────────────────────────────────────────────

    void setRobotRadius(float r) { robot_radius_ = r; }
    void setTrackUnknown(bool v) { track_unknown_ = v; }

private:
    const OccupancyGrid2D& map_;
    float robot_radius_   = 0.3f;
    bool  track_unknown_  = false;

    double resolution_ = 0.05;
    unsigned int width_  = 0;
    unsigned int height_ = 0;

    /// Convert world (x,y) to grid (ix,iy). Returns false if out of bounds.
    bool worldToGrid(double wx, double wy, unsigned int& ix, unsigned int& iy) const;

    /// Cost at a single grid cell.
    int cellCost(unsigned int ix, unsigned int iy) const;
};

}  // namespace finenav::robot_bringup
