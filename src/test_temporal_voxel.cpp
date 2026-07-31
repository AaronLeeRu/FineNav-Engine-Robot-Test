// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Verification test for TemporalVoxelMap + FusedMppiMapView.
//
// Build:  colcon build --packages-select finenav_robot_bringup
// Run:    ros2 run finenav_robot_bringup test_temporal_voxel
//
// Requires no sensors, no TF, no odometry — completely self-contained.

#include <chrono>
#include <iostream>
#include <shared_mutex>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "finenav_map_temporal_voxel_map/temporal_voxel_map.hpp"
#include "finenav_map_occupancy_grid_2d/occupancy_grid_2d.hpp"
#include "finenav_robot_bringup/fused_mppi_map_view.hpp"
#include "finenav_core/map/locked_map_view.hpp"

using namespace finenav;
using finenav::robot_bringup::FusedMppiMapView;

// ── Helpers ──────────────────────────────────────────────────────────────

static int g_passed = 0;
static int g_failed = 0;

#define CHECK(expr, msg) do {                                             \
    if (expr) { ++g_passed; std::cout << "  [PASS] " << msg << "\n"; }    \
    else      { ++g_failed; std::cout << "  [FAIL] " << msg << "\n"; }    \
} while(0)

// ── Helper: create a minimal OccupancyGrid ──────────────────────────────

static nav_msgs::msg::OccupancyGrid makeTestGrid() {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = 0.1;
    grid.info.width  = 100;
    grid.info.height = 100;
    grid.info.origin.position.x = -5.0;
    grid.info.origin.position.y = -5.0;
    // All free, except a central obstacle
    grid.data.resize(100 * 100, 0);
    // Put an obstacle at grid cell (50, 50) — world (0.05, 0.05)
    grid.data[50 * 100 + 50] = 100;
    return grid;
}

// ══════════════════════════════════════════════════════════════════════════
//  Test 1: TemporalVoxelMap basic lifecycle
// ══════════════════════════════════════════════════════════════════════════

static void test_temporal_voxel_basic() {
    std::cout << "\n── Test 1: TemporalVoxelMap basic lifecycle ──\n";

    TemporalVoxelMap map;

    // Configure: 10m cube, 0.2m resolution, centred at origin
    temporal_voxel_map::Params cfg;
    cfg.resolution    = 0.2;
    cfg.map_length    = 10.0;
    cfg.min_range     = 0.0;
    cfg.max_range     = 50.0;
    cfg.min_angle     = -1.57;   // -π/2
    cfg.max_angle     = 1.57;    // +π/2
    cfg.decay_time_sec     = 2.0;
    cfg.decay_time_fov_sec = 0.5;
    map.configure(cfg);

    CHECK(map.isInside(Position3D(0.0, 0.0, 0.0)), "isInside at origin");
    CHECK(map.isInside(Position3D(4.9, 0.0, 0.0)), "isInside near edge");
    CHECK(!map.isInside(Position3D(6.0, 0.0, 0.0)), "NOT isInside outside window");

    // Insert a synthetic point cloud with one point at (2, 1, 0.5) in map frame
    pcl::PointCloud<pcl::PointXYZ> cloud;
    // Point in body frame at (2, 1, 0.3) — body at origin, identity transform
    cloud.push_back(pcl::PointXYZ(2.0f, 1.0f, 0.3f));
    cloud.push_back(pcl::PointXYZ(1.5f, 0.5f, 0.4f));
    cloud.push_back(pcl::PointXYZ(3.0f, 2.0f, 0.2f));

    Eigen::Isometry3d T_map_body = Eigen::Isometry3d::Identity();
    map.insertPointCloud(cloud, T_map_body);

    // All three points are in FOV (azimuth ±π/2 in XY plane, range 0-50) and inside window
    CHECK(map.isDynamicObstacle(Position3D(2.0, 1.0, 0.3)), "dynamic obstacle at (2, 1, 0.3)");
    CHECK(map.isDynamicObstacle(Position3D(1.5, 0.5, 0.4)), "dynamic obstacle at (1.5, 0.5, 0.4)");
    CHECK(map.isDynamicObstacle(Position3D(3.0, 2.0, 0.2)), "dynamic obstacle at (3, 2, 0.2)");
    CHECK(!map.isDynamicObstacle(Position3D(0.0, 0.0, 0.0)), "NO obstacle at origin (never inserted)");
    CHECK(!map.isDynamicObstacle(Position3D(9.0, 9.0, 0.0)), "NO obstacle far away");

    // Prune after 0 delay → all should be gone (age > 0 = expired for fov cells
    // with decay_time_fov_sec=0.5 — but they were just inserted, so age ≈ 0)
    map.pruneExpiredCells(T_map_body);
    CHECK(map.isDynamicObstacle(Position3D(2.0, 1.0, 0.3)),
          "still alive after prune (age < decay_time_fov_sec=0.5)");

    // Wait > 0.5s for FOV decay
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    map.pruneExpiredCells(T_map_body);
    CHECK(!map.isDynamicObstacle(Position3D(2.0, 1.0, 0.3)),
          "EXPIRED after 0.6s > decay_time_fov_sec=0.5");

    std::cout << "  → Test 1 complete\n";
}

// ══════════════════════════════════════════════════════════════════════════
//  Test 2: FusedMppiMapView — fused cost / collision
// ══════════════════════════════════════════════════════════════════════════

static void test_fused_mppi_mapview() {
    std::cout << "\n── Test 2: FusedMppiMapView fused queries ──\n";

    // ── 2a. Set up OccupancyGrid2D with a known obstacle ──────────────
    OccupancyGrid2D occ_map;
    occupancy_grid_2d::Params occ_cfg;
    occ_cfg.occupancy_threshold = 50;
    occ_map.configure(occ_cfg);
    occ_map.update(makeTestGrid());

    CHECK(occ_map.hasMap(), "OccGrid has map data");

    // ── 2b. Set up TemporalVoxelMap with a dynamic obstacle ───────────
    auto temporal_map = std::make_unique<TemporalVoxelMap>();

    temporal_voxel_map::Params tv_cfg;
    tv_cfg.resolution    = 0.2;
    tv_cfg.map_length    = 10.0;
    tv_cfg.min_range     = 0.0;
    tv_cfg.max_range     = 50.0;
    tv_cfg.min_angle     = -1.57;
    tv_cfg.max_angle     = 1.57;
    tv_cfg.decay_time_sec     = 2.0;
    tv_cfg.decay_time_fov_sec = 0.5;
    temporal_map->configure(tv_cfg);

    // Insert a dynamic obstacle at world (1.0, 1.0, 0.3)
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(1.0f, 1.0f, 0.3f));
    Eigen::Isometry3d T_map_body = Eigen::Isometry3d::Identity();
    temporal_map->insertPointCloud(cloud, T_map_body);

    CHECK(temporal_map->isDynamicObstacle(Position3D(1.0, 1.0, 0.3)),
          "temporal dynamic obstacle at (1, 1)");

    // ── 2c. Create FusedMppiMapView ───────────────────────────────────
    // LockedMapRO needs a shared_mutex — create one for test purposes.
    std::shared_mutex test_mutex;
    finenav::core::LockedMapRO<TemporalVoxelMap> temporal_lock(*temporal_map, test_mutex);

    FusedMppiMapView view(occ_map, std::move(temporal_lock));

    // ── 2d. Query costs ───────────────────────────────────────────────
    view.setTrackUnknown(false);

    // Free cell: (3, 3) — no static obstacle, no dynamic obstacle
    int free_cost = view.getCost(Position3D(3.0, 3.0, 0.0));
    CHECK(free_cost == 0, "cost at free cell (3, 3) == 0 (got " + std::to_string(free_cost) + ")");

    // Static obstacle: (0.05, 0.05) — cell (50,50) in grid, value=100
    int static_cost = view.getCost(Position3D(0.05, 0.05, 0.0));
    CHECK(static_cost >= 200, "cost at static obstacle (0.05, 0.05) >= 200 (got "
         + std::to_string(static_cost) + ")");

    // Dynamic obstacle: (1.0, 1.0) — temporal voxel present
    int dynamic_cost = view.getCost(Position3D(1.0, 1.0, 0.0));
    CHECK(dynamic_cost == 254, "cost at dynamic obstacle (1, 1) == 254 (got "
         + std::to_string(dynamic_cost) + ")");

    // ── 2e. Collision check ──────────────────────────────────────────
    view.setRobotRadius(0.2f);

    // Free pose — should not collide
    bool free_collision = view.isCollision(3.0f, 3.0f, 0.0f);
    CHECK(!free_collision, "NO collision at free pose (3, 3)");

    // Pose near static obstacle — should collide
    bool static_collision = view.isCollision(0.05f, 0.05f, 0.0f);
    CHECK(static_collision, "COLLISION at static obstacle (0.05, 0.05)");

    // Pose near dynamic obstacle — should collide
    bool dynamic_collision = view.isCollision(1.0f, 1.0f, 0.0f);
    CHECK(dynamic_collision, "COLLISION at dynamic obstacle (1, 1)");

    // ── 2f. costAtPose ───────────────────────────────────────────────
    float free_cost_pose = view.costAtPose(3.0f, 3.0f, 0.0f);
    CHECK(free_cost_pose == 0.0f, "costAtPose free cell == 0 (got "
         + std::to_string(free_cost_pose) + ")");

    float dyn_cost_pose = view.costAtPose(1.0f, 1.0f, 0.0f);
    CHECK(dyn_cost_pose >= 200.0f, "costAtPose dynamic obstacle >= 200 (got "
         + std::to_string(dyn_cost_pose) + ")");

    std::cout << "  → Test 2 complete\n";
}

// ══════════════════════════════════════════════════════════════════════════
//  Main
// ══════════════════════════════════════════════════════════════════════════

int main() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║  TemporalVoxelMap + FusedMppiMapView Verification        ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n";

    test_temporal_voxel_basic();
    test_fused_mppi_mapview();

    std::cout << "\n──────────────────────────────────────────\n";
    std::cout << "  TOTAL:  " << (g_passed + g_failed) << " tests\n";
    std::cout << "  PASSED: " << g_passed << "\n";
    std::cout << "  FAILED: " << g_failed << "\n";
    std::cout << "──────────────────────────────────────────\n";

    if (g_failed > 0) {
        std::cout << "\n  ❌  SOME TESTS FAILED!\n\n";
        return 1;
    }
    std::cout << "\n  ✅  ALL TESTS PASSED\n\n";
    return 0;
}
