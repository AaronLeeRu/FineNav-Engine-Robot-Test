// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_mppi_controller/map_view_def.hpp"
#include "finenav_map_occupancy_grid_2d/occupancy_grid_2d.hpp"
#include "finenav_map_temporal_voxel_map/temporal_voxel_map.hpp"
#include "finenav_core/map/locked_map_view.hpp"

namespace finenav::robot_bringup {

/**
 * @brief Composite MapView fusing OccupancyGrid2D (static) + TemporalVoxelMap (dynamic).
 *
 * Adapts two independent maps to nav2_mppi_controller::IMapView for MPPI local
 * planning.  Collision and cost queries check both the static occupancy grid
 * and the temporal dynamic-obstacle voxel layer, returning the most
 * conservative (highest cost / collision-positive) result.
 *
 * The OccupancyGrid2D reference is provided by the framework's bind_map_view
 * factory (read lock already held).  The TemporalVoxelMap is accessed via a
 * LockedMapRO that is acquired inside the factory and moved in — this keeps
 * the shared_lock alive for the lifetime of the plan() call.
 */
class FusedMppiMapView : public nav2_mppi_controller::IMapView {
public:
    /// @param occ_map       Static occupancy grid (framework read-locked).
    /// @param temporal_lock RAII read lock on the temporal voxel map.
    FusedMppiMapView(const OccupancyGrid2D& occ_map,
                     finenav::core::LockedMapRO<TemporalVoxelMap>&& temporal_lock);

    // ── IMapView interface ──────────────────────────────────────────────

    bool isTrackingUnknown() const override;
    bool considerFootprint() const override;

    /// Check circular footprint against BOTH the static grid and temporal voxels.
    bool isCollision(float x, float y, float theta) const override;

    float getRadius() const override;

    /// Raw cost at a world position (max of static and dynamic cost).
    int getCost(const Position3D& pos) const override;

    /// Maximum cost under the footprint at (x, y, theta), fused from both maps.
    float costAtPose(float x, float y, float theta) const override;

    std::string getBaseFrameID() const override;

    // ── Tuning ──────────────────────────────────────────────────────────

    void setRobotRadius(float r) { robot_radius_ = r; }
    void setTrackUnknown(bool v) { track_unknown_ = v; }

private:
    const OccupancyGrid2D& occ_map_;
    finenav::core::LockedMapRO<TemporalVoxelMap> temporal_lock_;

    float robot_radius_  = 0.3f;
    bool  track_unknown_ = false;

    // Cached OccupancyGrid2D properties for fast lookup
    double       resolution_ = 0.05;
    unsigned int width_      = 0;
    unsigned int height_     = 0;
    double       origin_x_   = 0.0;
    double       origin_y_   = 0.0;

    /// Convert world (x,y) to static grid (ix,iy). Returns false if out of bounds.
    bool worldToGrid(double wx, double wy, unsigned int& ix, unsigned int& iy) const;

    /// Static grid cost at a single cell.
    int staticCellCost(unsigned int ix, unsigned int iy) const;

    /// Query the temporal voxel map for dynamic obstacles under a footprint.
    bool temporalCollisionInFootprint(unsigned int cx, unsigned int cy,
                                      int radius_cells, bool treat_unknown_as_collision) const;

    /// Maximum temporal cost in a footprint region.
    float temporalMaxCostInFootprint(unsigned int cx, unsigned int cy,
                                     int radius_cells, bool treat_unknown_as_collision) const;
};

}  // namespace finenav::robot_bringup
