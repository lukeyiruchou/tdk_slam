#pragma once

// ============================================================================
// localization_manager
//
// 移植自學長的 loc_and_nav 分支，並做了以下修正：
//   1. FinishTrajectory 失敗（例如 trajectory 已被結束）不再直接放棄，
//      而是警告後繼續嘗試 StartTrajectory，避免「finish 成功、start 失敗」
//      之後 retry 永遠卡死的死鎖情境。
//   2. cartographer 設定檔目錄改為 ROS parameter（預設用 ament share 路徑），
//      不再寫死 /home/tdk/... 絕對路徑。
//   3. 移除 service callback 內的 sleep_for(500ms)（會卡住 executor），
//      改用 one-shot timer 延遲呼叫 StartTrajectory。
//   4. 所有失敗路徑（service 不在、finish/start 例外、驗證逾時）都會
//      publish /init_pose_status = false，FSM 端才收得到失敗、能觸發 retry。
//
// 流程：
//   /init_pose_cmd (PoseStamped, frame=world)
//     -> world 轉 map（減去 world->map 靜態偏移）
//     -> Cartographer /finish_trajectory + /start_trajectory（帶 initial pose）
//     -> 用 /robot_pose (world frame) 驗證定位收斂
//     -> 清 Nav2 costmap
//     -> /init_pose_status (Bool) 回報 FSM
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <cartographer_ros_msgs/srv/finish_trajectory.hpp>
#include <cartographer_ros_msgs/srv/start_trajectory.hpp>

#include <string>

namespace tdk_localization {

enum class InitState {
    IDLE,               // 等待新的初始化指令
    TRIGGER_SLAM,       // 正在呼叫 SLAM 端重新初始化
    VERIFYING,          // 比對 /robot_pose 是否到達目標點
    CLEARING_COSTMAP,   // 清除 Nav2 costmap
    SUCCESS             // 初始化成功（保持一段時間後回 IDLE）
};

class LocalizationManager : public rclcpp::Node {
public:
    explicit LocalizationManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~LocalizationManager() = default;

private:
    // ==== callbacks ====
    void onInitCmdReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onRobotPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // ==== state machine actions ====
    void triggerSlamReset();
    void requestStartTrajectory();   // FinishTrajectory 後（延遲 500ms）呼叫
    void verifyPose(double current_x, double current_y, double current_yaw);
    void clearNav2Costmaps();

    // ==== helpers ====
    void publishStatus(bool ok);
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

    // ==== pub / sub ====
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr init_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_toolbox_pub_;

    // ==== service clients ====
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_costmap_cli_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_costmap_cli_;
    rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedPtr carto_finish_cli_;
    rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr carto_start_cli_;

    // ==== timers ====
    rclcpp::TimerBase::SharedPtr fsm_timer_;            // SUCCESS 狀態保持計時
    rclcpp::TimerBase::SharedPtr delayed_start_timer_;  // finish -> start 之間的 one-shot 延遲

    // ==== state ====
    std::string slam_type_;
    std::string carto_config_dir_;
    std::string carto_config_basename_;
    InitState current_state_{InitState::IDLE};
    rclcpp::Time verification_start_time_;
    rclcpp::Time success_start_time_;
    int32_t current_active_trajectory_id_;

    double world_to_map_x_;
    double world_to_map_y_;

    // 目標點（world frame，收到 cmd 時記錄）
    double target_x_;
    double target_y_;
    double target_yaw_;
    // 目標點（map frame，triggerSlamReset 時換算）
    double target_map_x_;
    double target_map_y_;

    double tolerance_dist_;
    double tolerance_yaw_;
    double verify_timeout_sec_;
};

} // namespace tdk_localization
