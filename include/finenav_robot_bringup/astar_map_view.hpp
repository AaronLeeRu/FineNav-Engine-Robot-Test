// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_route_planner/imap_view.hpp"
#include "finenav_map_occupancy_grid_2d/occupancy_grid_2d.hpp"

namespace finenav::robot_bringup {

/**
 * @brief Adapts OccupancyGrid2D to astar::IMapView for A* path planning.
 *
 * Converts world coordinates to grid indices and queries occupancy/cost
 * from the underlying OccupancyGrid data.
 */
class AstarMapView : public astar::IMapView {
public:
    explicit AstarMapView(const OccupancyGrid2D& map);

    /// Whether the world point is traversable (not occupied, not unknown).
    bool isWalkable(const core::Position3D& p) const override;

    /// Normalised traversal cost at p: 0.0 (free) to 1.0 (near-lethal).
    double getCost(const core::Position3D& p) const override;

    /// Grid resolution in metres — the planner's step size.
    double getStepSize() const override;

private:
    const OccupancyGrid2D& map_;
    double resolution_ = 0.05;
    unsigned int width_  = 0;
    unsigned int height_ = 0;

    /// Convert world (x,y) to grid (ix,iy).  Returns false if out of bounds.
    bool worldToGrid(double wx, double wy, unsigned int& ix, unsigned int& iy) const;
};

}  // namespace finenav::robot_bringup
