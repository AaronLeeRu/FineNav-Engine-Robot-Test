// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>

#include "finenav_mppi_controller/map_view_def.hpp"
#include "finenav_map_occupancy_grid_2d/occupancy_grid_2d.hpp"
#include "finenav_map_temporal_voxel_map/temporal_voxel_map.hpp"
#include "finenav_map_terrain_analysis/terrain_analyzer.hpp"
#include "finenav_collision_model/obb_collision_model.hpp"
#include "finenav_core/map/locked_map_view.hpp"

namespace finenav::robot_bringup {

/**
 * @brief MPPI MapView using TerrainAnalyzer for cost + OBBCollisionModel for footprint.
 *
 * Replaces FusedMppiMapView with terrain-aware queries:
 *   - getCost()   → TerrainAnalyzer::getCost() (ground detection + gradient + inflation)
 *   - isCollision() → OBBCollisionModel edge-based check using terrain costs
 *   - costAtPose() → OBBCollisionModel::checkCostEdge()
 *
 * The TerrainAnalyzer is shared across all MapView instances (one per plan() call) —
 * it is updated each cycle via MapServer's postUpdateHook.
 */
class TerrainMppiMapView : public nav2_mppi_controller::IMapView {
public:
    /// @param occ_map       Static occupancy grid (framework read-locked).
    /// @param temporal_lock RAII read lock on temporal voxel map.
    /// @param terrain       Shared TerrainAnalyzer (updated by postUpdateHook).
    TerrainMppiMapView(const OccupancyGrid2D& occ_map,
                       finenav::core::LockedMapRO<TemporalVoxelMap>&& temporal_lock,
                       std::shared_ptr<TerrainAnalyzer> terrain);

    // ── IMapView interface ──────────────────────────────────────────────

    bool isTrackingUnknown() const override;
    bool considerFootprint() const override;
    bool isCollision(float x, float y, float theta) const override;
    float getRadius() const override;
    int getCost(const Position3D& pos) const override;
    float costAtPose(float x, float y, float theta) const override;
    std::string getBaseFrameID() const override;

private:
    finenav::Pose createPose2D(double x, double y, double theta) const;

    const OccupancyGrid2D& occ_map_;
    finenav::core::LockedMapRO<TemporalVoxelMap> temporal_lock_;
    std::shared_ptr<TerrainAnalyzer> terrain_;

    std::unique_ptr<finenav::OBBCollisionModel> collision_model_;
    bool is_tracking_unknown_ = false;
};

}  // namespace finenav::robot_bringup
