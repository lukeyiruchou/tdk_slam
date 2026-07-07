include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint", -- frame where imu locate
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,    -- close when using odometry
  use_odometry = true,           -- open when using odometry
  use_nav_sat = false,
  use_landmarks = false,
  use_pose_extrapolator = true,
  publish_frame_projected_to_2d = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 3e-2,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 12.0


-- 大幅增加 LiDAR 掃描特徵與子地圖匹配的權重 (預設通常是 1.0)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 40.0
-- 降低對 Odom 平移與旋轉的信任 (預設通常為 10.0 和 40.0)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 5.0

-- 因為上方已開啟 use_online_correlative_scan_matching，此處降低偏離 Odom 的懲罰
-- 讓系統在暴力搜尋時，更自由地用 LiDAR 找最佳吻合點
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.01
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.01

-- 3. 後端位姿圖優化 (Global SLAM)
-- 降低後端優化時對 Odom 約束的權重 (預設通常是 1e5)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e2
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e2

-- -- Ceres scan matcher (defaults)
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

-- -- Real-time correlative scan matcher (defaults)
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- -- Pose graph odometry weights (defaults)
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
