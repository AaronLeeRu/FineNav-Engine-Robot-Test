cmake_minimum_required(VERSION 3.22)
project(finenav_robot_bringup)

set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(finenav_engine REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(xtensor REQUIRED)
find_package(xsimd REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_auto_find_build_dependencies()

# ── MapView adapters (library) ──────────────────────────────────────────────

add_library(robot_map_views SHARED
    src/astar_map_view.cpp
    src/mppi_map_view.cpp
    src/fused_mppi_map_view.cpp
    src/terrain_mppi_map_view.cpp
)
target_include_directories(robot_map_views PUBLIC include)
ament_target_dependencies(robot_map_views
    finenav_core
    finenav_map_occupancy_grid_2d
    finenav_map_temporal_voxel_map
    finenav_map_terrain_analysis
    finenav_collision_model
    finenav_route_planner
    finenav_mppi_controller
)
# finenav_route_planner uses scoped headers; ensure the scoped include dir is found.
# (ament_target_dependencies may not propagate the scoped path for old-style library exports.)
set(_rte_scoped_inc "${finenav_route_planner_DIR}/../../../include/finenav_route_planner")
if(IS_DIRECTORY "${_rte_scoped_inc}")
    target_include_directories(robot_map_views SYSTEM PUBLIC "${_rte_scoped_inc}")
endif()
unset(_rte_scoped_inc)
target_link_libraries(robot_map_views Eigen3::Eigen ${PCL_LIBRARIES})

# ── Bringup executable ─────────────────────────────────────────────────────

find_library(FINENAV_ROUTE_PLANNER_LIB finenav_route_planner_lib
    PATHS "${finenav_route_planner_DIR}/../../../lib"
    NO_DEFAULT_PATH)

add_executable(robot_bringup src/robot_bringup_main.cpp)
target_link_libraries(robot_bringup
    robot_map_views
    ${FINENAV_ROUTE_PLANNER_LIB}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)
ament_target_dependencies(robot_bringup
    rclcpp
    finenav_core
    finenav_util
    finenav_engine
    finenav_route_planner
    finenav_mppi_controller
    finenav_map_temporal_voxel_map
    finenav_map_terrain_analysis
    finenav_collision_model
    geometry_msgs
    nav_msgs
    sensor_msgs
    pcl_conversions
)
# Same scoped-header workaround as robot_map_views above.
set(_rte_scoped_inc "${finenav_route_planner_DIR}/../../../include/finenav_route_planner")
if(IS_DIRECTORY "${_rte_scoped_inc}")
    target_include_directories(robot_bringup SYSTEM PRIVATE "${_rte_scoped_inc}")
endif()
unset(_rte_scoped_inc)

# ── Install ─────────────────────────────────────────────────────────────────

install(TARGETS robot_bringup robot_map_views
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# ── Verification test (standalone, no ROS node) ─────────────────────────────

add_executable(test_temporal_voxel src/test_temporal_voxel.cpp)
target_link_libraries(test_temporal_voxel
    robot_map_views
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)
ament_target_dependencies(test_temporal_voxel
    finenav_core
    finenav_map_occupancy_grid_2d
    finenav_map_temporal_voxel_map
    finenav_mppi_controller
    nav_msgs
)
install(TARGETS test_temporal_voxel
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
