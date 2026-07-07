-- ============================================================
-- relocalization.lua — RELOCALIZE 模式(kidnap / 位姿遺失)
--
-- 使用情境:FSM 進入 RELOCALIZE 狀態,localization_manager
-- 呼叫 start_trajectory「不帶」initial_pose,靠全域搜尋重新定位。
--
-- 設計取向:全域搜尋開到最積極、後端最高頻優化,
-- 用 CPU 換收斂時間。這個模式只該短暫存在 —
-- localization_manager 確認收斂(例如 covariance / constraint
-- score 達標)後,應 finish_trajectory 並切回 localization.lua
-- 重新以已知位姿啟動,不要長時間跑在這組參數上。
-- ============================================================

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = true,
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

-- ============================================================
-- 前端 (Local SLAM)
-- ============================================================
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 12.0

-- Submap 更小:讓第一個可匹配的 submap 更快出現
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 25

-- Kidnap 後 odom 累積誤差不可信,online correlative 搜尋窗放大
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.01
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.01

-- Ceres:重 LiDAR、輕 odom(kidnap 情境下 odom 參考價值低)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 5.0

-- ============================================================
-- 後端 (Global SLAM / Global Relocalization)
-- ============================================================

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- 最高頻優化:constraint 一找到就儘快套用
POSE_GRAPH.optimize_every_n_nodes = 10

-- 全域搜尋開到最積極(這是 kidnap 恢復速度的關鍵)
POSE_GRAPH.constraint_builder.sampling_ratio = 0.7
POSE_GRAPH.global_sampling_ratio = 0.02                       -- 預設 0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 3.      -- 預設 10

-- 接受門檻略降,換收斂速度。
-- 注意:TDK 場地若有對稱結構,0.55 可能誤匹配 —
-- 若測試中出現定位跳到鏡像位置,把 global_localization_min_score 拉回 0.6~0.66
POSE_GRAPH.constraint_builder.min_score = 0.5
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55

-- 全域搜尋不受 linear_search_window 限制,維持預設即可
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)

-- Odom 約束權重壓低:優化時讓全域 constraint 能把位姿「拉」到正確位置,
-- 不被 kidnap 前的 odom 軌跡綁住
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e1
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e1

return options