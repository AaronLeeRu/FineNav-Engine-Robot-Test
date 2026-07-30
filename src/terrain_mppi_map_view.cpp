// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_robot_bringup/terrain_mppi_map_view.hpp"

#include <cmath>

namespace finenav::robot_bringup {

// ==============================================================================
// Constructor
// ==============================================================================

TerrainMppiMapView::TerrainMppiMapView(
    const OccupancyGrid2D& occ_map,
    finenav::core::LockedMapRO<TemporalVoxelMap>&& temporal_lock,
    std::shared_ptr<TerrainAnalyzer> terrain)
    : occ_map_(occ_map)
    , temporal_lock_(std::move(temporal_lock))
    , terrain_(std::move(terrain))
{
    // Default symmetric OBB — tune via setRobotDimensions() or override in derived.
    // Dimensions: 1.4 m long × 0.8 m wide × 0.0 m tall (2D planar).
    collision_model_ = std::make_unique<finenav::OBBCollisionModel>(
        finenav::Vector3D(1.4, 0.8, 0.0));
}

// ==============================================================================
// IMapView interface
// ==============================================================================

bool TerrainMppiMapView::isTrackingUnknown() const {
    return is_tracking_unknown_;
}

bool TerrainMppiMapView::considerFootprint() const {
    return true;
}

bool TerrainMppiMapView::isCollision(float x, float y, float theta) const {
    // Fast path: centre-point cost check
    int pose_cost = getCost(Position3D{static_cast<double>(x),
                                       static_cast<double>(y), 0.0});
    if (pose_cost == 254) return true;

    // If centre cost is high enough, do full OBB edge check
    int threshold = terrain_->computeCost(
        static_cast<int>(collision_model_->getOuterDiameter() / 2 /
                         temporal_lock_->grid().getResolution()));
    if (pose_cost >= threshold) {
        finenav::Pose pose = createPose2D(x, y, theta);
        return collision_model_->checkCollisionEdge(pose, [this](const Position3D& p) {
            int c = this->getCost(p);
            return c >= 254;
        });
    }
    return false;
}

float TerrainMppiMapView::getRadius() const {
    return static_cast<float>(collision_model_->getOuterDiameter());
}

int TerrainMppiMapView::getCost(const Position3D& pos) const {
    return terrain_->getCost(pos, temporal_lock_->grid());
}

float TerrainMppiMapView::costAtPose(float x, float y, float theta) const {
    finenav::Pose pose = createPose2D(x, y, theta);
    return static_cast<float>(collision_model_->checkCostEdge(pose, [this](const Position3D& p) {
        return this->getCost(p);
    }));
}

std::string TerrainMppiMapView::getBaseFrameID() const {
    return "base_link";
}

// ==============================================================================
// Helper
// ==============================================================================

finenav::Pose TerrainMppiMapView::createPose2D(double x, double y, double theta) const {
    finenav::Pose pose = finenav::Pose::Identity();
    pose.linear() = Eigen::AngleAxisd(static_cast<double>(theta),
                                      Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = Eigen::Vector3d(x, y, 0.0);
    return pose;
}

}  // namespace finenav::robot_bringup
