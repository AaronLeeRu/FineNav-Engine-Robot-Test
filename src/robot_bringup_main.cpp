// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// FineNav Robot Bringup — real-robot navigation entry point.
//
// Assemblies the full navigation stack:
//   OccupancyGrid2D (static) ─┬─► AstarPathSearch (route)
//                          └─► TerrainMppiMapView ─► MPPIController (track)
//   TemporalVoxelMap ─► TerrainAnalyzer ─┘
//   Pipeline: route ─► mppi ─► /cmd_vel
//
// Usage:
//   ros2 run finenav_robot_bringup robot_bringup \
//     --ros-args --params-file src/finenav_robot_bringup/config/robot_params.yaml

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "finenav_engine/finenav_engine.hpp"
#include "finenav_map_occupancy_grid_2d/occupancy_grid_2d.hpp"
#include "finenav_map_temporal_voxel_map/temporal_voxel_map.hpp"
#include "finenav_map_terrain_analysis/terrain_analyzer.hpp"
#include "finenav_route_planner/astar_path_search.hpp"
#include "finenav_mppi_controller/controller.hpp"

#include "finenav_robot_bringup/astar_map_view.hpp"
#include "finenav_robot_bringup/terrain_mppi_map_view.hpp"

using namespace finenav;
using finenav::robot_bringup::AstarMapView;
using finenav::robot_bringup::TerrainMppiMapView;

// ── Helper: build map→body Isometry3d from RobotState ──────────────────────
static Eigen::Isometry3d makeTMapBody(const core::RobotState& state) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = state.position;
    T.linear()      = state.orientation.toRotationMatrix();
    return T;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("robot_bringup");
    auto engine = std::make_shared<finenav::FineNavEngine>(node);

    // ── 0. Check bringup mode ─────────────────────────────────────────────
    node->declare_parameter<std::string>("mode", "navigation");
    auto mode = node->get_parameter("mode").as_string();

    if (mode == "localization") {
        RCLCPP_INFO(node->get_logger(),
            "\n"
            "============================================================\n"
            "  FineNav Robot Bringup — LOCALIZATION ONLY\n"
            "============================================================\n"
            "  ESKF localizer is fusing odometry sources from params.\n"
            "  No map, no planners, no navigator.\n"
            "\n"
            "  Verify:\n"
            "    ros2 topic echo /nav_state\n"
            "    ros2 run tf2_ros tf2_echo map base_link\n"
            "  (TF is only broadcast if publish_tf: true in params)\n"
            "============================================================\n");
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }

    // ── Full navigation mode ──────────────────────────────────────────────
    node->declare_parameter<std::string>("point_cloud_topic", "/cloud_registered_body");
    auto pc_topic = node->get_parameter("point_cloud_topic").as_string();

    // ── 1. Set the default behaviour tree ─────────────────────────────────
    engine->setDefaultBehaviorTree("RobotNavigateToPose");

    // ── 2. Map servers ────────────────────────────────────────────────────

    // 2a. Static occupancy grid (global prior map)
    auto occ_map_server = engine->createMapServer<OccupancyGrid2D>("occ_map");

    auto map_injector = occ_map_server->addObservationSource<nav_msgs::msg::OccupancyGrid>(
        "map_source",
        finenav::core::ObservationBufferPolicy{},
        [](const nav_msgs::msg::OccupancyGrid& msg, const rclcpp::Time&,
           const core::RobotState& /*state*/, OccupancyGrid2D& map) {
            map.update(msg);
        });

    auto map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        [injector = std::move(map_injector), node](std::shared_ptr<nav_msgs::msg::OccupancyGrid> msg) {
            auto stamp = rclcpp::Time(msg->header.stamp, node->get_clock()->get_clock_type());
            injector(std::move(*msg), stamp);
        });

    // 2b. Temporal dynamic obstacle map (LiDAR point cloud → rolling voxels)
    auto temporal_map_server = engine->createMapServer<TemporalVoxelMap>("temporal_map");

    // Pre-update hook: prune expired voxels at each MapServer update cycle.
    temporal_map_server->registerPreUpdateHook(
        [](const core::RobotState& state, TemporalVoxelMap& map) {
            map.pruneExpiredCells(makeTMapBody(state));
        });

    // Feed point cloud data via manual injector (same pattern as /map above).
    auto pc_injector = temporal_map_server->addObservationSource<sensor_msgs::msg::PointCloud2>(
        "lidar_source",
        finenav::core::ObservationBufferPolicy{
            .max_buffer_size           = 16,
            .observation_keep_time_sec = 0.5,
        },
        [](const sensor_msgs::msg::PointCloud2& msg, const rclcpp::Time& /*stamp*/,
           const core::RobotState& state, TemporalVoxelMap& map) {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(msg, cloud);
            map.insertPointCloud(cloud, makeTMapBody(state));
        });

    auto pc_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        pc_topic,
        rclcpp::SensorDataQoS(),
        [injector = std::move(pc_injector), node](std::shared_ptr<sensor_msgs::msg::PointCloud2> msg) {
            auto stamp = rclcpp::Time(msg->header.stamp, node->get_clock()->get_clock_type());
            injector(std::move(*msg), stamp);
        });

    RCLCPP_INFO(node->get_logger(),
        "TemporalVoxelMap: subscribing to point cloud on '%s'.",
        pc_topic.c_str());

    // ── 3. TerrainAnalyzer ────────────────────────────────────────────────
    // Processes the TemporalVoxelMap each update cycle to produce ground
    // heights, passability map, and inflated costmap.  Parameters come from
    // the bringup node under the "terrain_analysis" namespace.
    auto terrain_analyzer = std::make_shared<TerrainAnalyzer>();
    auto terrain_param_handle =
        finenav::core::AlgoConfigurator<TerrainAnalyzer>::load(node, *terrain_analyzer);

    // Post-update hook: after new point cloud is fused + expired voxels pruned,
    // update terrain analysis (ground detection → passability → costmap).
    temporal_map_server->registerPostUpdateHook(
        [terrain_analyzer, occ_map_server](const core::RobotState& state,
                                            TemporalVoxelMap& temporal_map) {
            auto occ_lock = occ_map_server->getLockedReadView();
            terrain_analyzer->update(state.position.z(), temporal_map, occ_lock.get());
        });

    // ── 4. Route planning layer (A*, single-shot, static map only) ───────
    auto route_layer = engine->createLayer<AstarPathSearch>(
        "route_layer", core::SingleShotPolicy{});

    route_layer->bind_map_view(
        occ_map_server,
        [](const OccupancyGrid2D& map_data, const std::string&) {
            return std::make_shared<AstarMapView>(map_data);
        });

    // ── 5. Tracking layer (MPPI, continuous at 50 Hz, terrain-aware) ─────
    auto mppi_layer = engine->createLayer<nav2_mppi_controller::MPPIController>(
        "mppi_layer", core::ContinuousPolicy{50.0});

    mppi_layer->bind_map_view(
        occ_map_server,
        [temporal_map_server, terrain_analyzer](const OccupancyGrid2D& occ_map,
                                                 const std::string&) {
            auto temporal_lock = temporal_map_server->getLockedReadView();
            return std::make_shared<TerrainMppiMapView>(
                occ_map, std::move(temporal_lock), terrain_analyzer);
        });

    // ── 6. Wire pipeline: route → mppi ────────────────────────────────────
    engine->wireControlPipeline({
        _root >> route_layer >> mppi_layer
    });

    // ── 7. Ready ──────────────────────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(),
        "FineNav Robot Bringup ready.\n"
        "  Static  map : /map → OccupancyGrid2D\n"
        "  Dynamic map : %s → TemporalVoxelMap\n"
        "  Terrain  : TerrainAnalyzer (ground + passability + costmap)\n"
        "  Route   : route_layer (AstarPathSearch, SingleShot)\n"
        "  Track   : mppi_layer  (MPPIController + TerrainMppiMapView, 50 Hz)\n"
        "  Pipe    : route → mppi\n"
        "  BT      : RobotNavigateToPose\n"
        "\n"
        "Publish a goal on /goal_pose or call the NavigateToPose action to start.",
        pc_topic.c_str());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
