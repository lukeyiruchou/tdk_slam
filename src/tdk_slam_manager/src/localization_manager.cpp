#include "tdk_slam_manager/localization_manager.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

namespace tdk_localization {

LocalizationManager::LocalizationManager(const rclcpp::NodeOptions & options)
: Node("localization_manager", options) {

    // world -> map 的靜態偏移（要跟 spawn_launch.py 的 static_transform_publisher 一致）
    this->declare_parameter("world_to_map_x", 0.425);
    this->declare_parameter("world_to_map_y", 1.0);
    world_to_map_x_ = this->get_parameter("world_to_map_x").as_double();
    world_to_map_y_ = this->get_parameter("world_to_map_y").as_double();

    this->declare_parameter("slam_type", "cartographer"); // 或 "slam_toolbox"
    slam_type_ = this->get_parameter("slam_type").as_string();

    // cartographer 設定檔目錄：預設從 ament share 解析，不寫死絕對路徑。
    // launch 端也可以直接傳入覆蓋。
    std::string default_config_dir;
    try {
        default_config_dir =
            ament_index_cpp::get_package_share_directory("tdk_slam_manager")
            + "/cartographer_config";
    } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(),
            "cannot resolve tdk_slam_manager share dir: %s", e.what());
    }
    this->declare_parameter("carto_config_dir", default_config_dir);
    this->declare_parameter("carto_config_basename", "localization.lua");
    carto_config_dir_ = this->get_parameter("carto_config_dir").as_string();
    carto_config_basename_ = this->get_parameter("carto_config_basename").as_string();

    // /robot_pose 驗證容差（比賽人工擺放誤差可能到 10cm 以上，放寬到 0.15）
    this->declare_parameter("tolerance_dist", 0.15);
    this->declare_parameter("tolerance_yaw", 0.15);
    this->declare_parameter("verify_timeout_sec", 5.0);
    tolerance_dist_ = this->get_parameter("tolerance_dist").as_double();
    tolerance_yaw_ = this->get_parameter("tolerance_yaw").as_double();
    verify_timeout_sec_ = this->get_parameter("verify_timeout_sec").as_double();

    // 接收 FSM 主程式的初始化訊號（world frame）
    init_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/init_pose_cmd", 10,
        std::bind(&LocalizationManager::onInitCmdReceived, this, std::placeholders::_1));

    // 接收 robot_pose_publisher 發出的全局座標（world frame）
    robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot_pose", 10,
        std::bind(&LocalizationManager::onRobotPoseReceived, this, std::placeholders::_1));

    // 回報 FSM 初始化結果
    status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/init_pose_status", 10);

    // 根據 SLAM pkg 選擇初始化方式
    if (slam_type_ == "slam_toolbox") {
        slam_toolbox_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);
    } else if (slam_type_ == "cartographer") {
        carto_finish_cli_ = this->create_client<cartographer_ros_msgs::srv::FinishTrajectory>(
            "/finish_trajectory");
        carto_start_cli_ = this->create_client<cartographer_ros_msgs::srv::StartTrajectory>(
            "/start_trajectory");
        // cartographer 以 -load_state_filename 啟動時：
        //   trajectory 0 = pbstream 內的凍結軌跡
        //   trajectory 1 = 開機後自動開始的 active 軌跡
        this->current_active_trajectory_id_ = 1;
    } else {
        RCLCPP_ERROR(this->get_logger(),
            "unknown slam_type '%s', expect 'cartographer' or 'slam_toolbox'",
            slam_type_.c_str());
    }

    // Nav2 costmap clear service clients
    global_costmap_cli_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "/global_costmap/clear_entirely_global_costmap");
    local_costmap_cli_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "/local_costmap/clear_entirely_local_costmap");

    RCLCPP_INFO(this->get_logger(),
        "Localization Manager start <slam_type>: %s, world->map offset: (%.3f, %.3f)",
        slam_type_.c_str(), world_to_map_x_, world_to_map_y_);
    RCLCPP_INFO(this->get_logger(),
        "cartographer config: %s/%s",
        carto_config_dir_.c_str(), carto_config_basename_.c_str());

    // SUCCESS 狀態保持 4 秒後回 IDLE，可接受下一次初始化
    fsm_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
            if (this->current_state_ == InitState::SUCCESS) {
                auto current_time = this->get_clock()->now();
                double elapsed_time = (current_time - this->success_start_time_).seconds();
                if (elapsed_time > 4.0) {
                    RCLCPP_INFO(this->get_logger(), "return to IDLE to get new cmd");
                    this->current_state_ = InitState::IDLE;
                }
            }
        }
    );
}

void LocalizationManager::onInitCmdReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 剛成功不久收到重複觸發：直接持續回報成功
    if (current_state_ == InitState::SUCCESS) {
        publishStatus(true);
        return;
    }
    // 正在處理中（TRIGGER_SLAM / VERIFYING / CLEARING）就忽略重複指令，
    // 避免 FSM timeout 重送時打斷進行中的流程。
    if (current_state_ != InitState::IDLE) {
        RCLCPP_WARN(this->get_logger(), "init in progress, ignore duplicated cmd");
        return;
    }

    // 記錄目標初始化位置（world frame）
    this->target_x_ = msg->pose.position.x;
    this->target_y_ = msg->pose.position.y;
    this->target_yaw_ = getYawFromQuaternion(msg->pose.orientation);

    RCLCPP_INFO(this->get_logger(), "target from FSM (world): [%.2f, %.2f, %.2f]",
        this->target_x_, this->target_y_, this->target_yaw_);

    current_state_ = InitState::TRIGGER_SLAM;
    triggerSlamReset();
}

void LocalizationManager::triggerSlamReset() {
    // world -> map（world->map 只有平移、無旋轉，直接相減即可）
    target_map_x_ = this->target_x_ - world_to_map_x_;
    target_map_y_ = this->target_y_ - world_to_map_y_;

    if (slam_type_ == "slam_toolbox") {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = target_map_x_;
        pose_msg.pose.pose.position.y = target_map_y_;
        pose_msg.pose.pose.orientation.z = std::sin(target_yaw_ / 2.0);
        pose_msg.pose.pose.orientation.w = std::cos(target_yaw_ / 2.0);

        slam_toolbox_pub_->publish(pose_msg);
        this->verification_start_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(),
            "transform world[%.2f, %.2f] to map[%.2f, %.2f] and send to slam_toolbox",
            this->target_x_, this->target_y_, target_map_x_, target_map_y_);
        current_state_ = InitState::VERIFYING;
        return;
    }

    if (slam_type_ != "cartographer") {
        publishStatus(false);
        current_state_ = InitState::IDLE;
        return;
    }

    // ---- cartographer ----
    // service 不在（cartographer 沒起來）直接回報失敗，讓 FSM 決定 retry
    if (!carto_finish_cli_->service_is_ready() || !carto_start_cli_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(),
            "cartographer trajectory services not ready, is cartographer_node up?");
        publishStatus(false);
        current_state_ = InitState::IDLE;
        return;
    }

    auto finish_req = std::make_shared<cartographer_ros_msgs::srv::FinishTrajectory::Request>();
    finish_req->trajectory_id = current_active_trajectory_id_;
    RCLCPP_INFO(this->get_logger(),
        "sending request to finish active trajectory ID: %d", current_active_trajectory_id_);

    carto_finish_cli_->async_send_request(finish_req,
        [this](rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedFuture future) {
            try {
                auto res = future.get();
                if (res->status.code != 0) {
                    // 修正重點：finish 失敗最常見的原因是 trajectory 已經被結束
                    // （例如上一輪 finish 成功但 start 失敗）。此時「沒有 active
                    // trajectory」正是我們要的前置條件，直接繼續 start 即可。
                    // 若是其他原因，start 也會失敗並回報 FSM。
                    RCLCPP_WARN(this->get_logger(),
                        "finish trajectory %d returned non-OK (%s), proceed to start anyway",
                        current_active_trajectory_id_, res->status.message.c_str());
                }
                // 等 500ms 讓 cartographer 釋放資料，用 one-shot timer 取代
                // sleep_for，避免卡住 executor（期間會收不到 /robot_pose）。
                delayed_start_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(500),
                    [this]() {
                        delayed_start_timer_->cancel();
                        this->requestStartTrajectory();
                    });
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "finish trajectory exception: %s", e.what());
                publishStatus(false);
                this->current_state_ = InitState::IDLE;
            }
        });
}

void LocalizationManager::requestStartTrajectory() {
    auto start_req = std::make_shared<cartographer_ros_msgs::srv::StartTrajectory::Request>();
    start_req->configuration_directory = carto_config_dir_;
    start_req->configuration_basename = carto_config_basename_;
    start_req->use_initial_pose = true;
    start_req->initial_pose.position.x = target_map_x_;
    start_req->initial_pose.position.y = target_map_y_;
    start_req->initial_pose.orientation.z = std::sin(this->target_yaw_ / 2.0);
    start_req->initial_pose.orientation.w = std::cos(this->target_yaw_ / 2.0);

    RCLCPP_INFO(this->get_logger(),
        "requesting new trajectory with initial pose (map): [%.2f, %.2f]",
        target_map_x_, target_map_y_);

    carto_start_cli_->async_send_request(start_req,
        [this](rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedFuture start_future) {
            try {
                auto start_res = start_future.get();
                if (start_res->status.code == 0) {
                    // 紀錄新的 trajectory id，下次重啟會用到
                    this->current_active_trajectory_id_ = start_res->trajectory_id;
                    RCLCPP_INFO(this->get_logger(),
                        "reinit cartographer success at map [%.2f, %.2f]. new trajectory ID: %d",
                        target_map_x_, target_map_y_, this->current_active_trajectory_id_);
                    this->verification_start_time_ = this->get_clock()->now();
                    this->current_state_ = InitState::VERIFYING;
                } else {
                    RCLCPP_ERROR(this->get_logger(),
                        "start new trajectory fail: %s", start_res->status.message.c_str());
                    publishStatus(false);
                    this->current_state_ = InitState::IDLE;
                }
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "start trajectory exception: %s", e.what());
                publishStatus(false);
                this->current_state_ = InitState::IDLE;
            }
        });
}

void LocalizationManager::onRobotPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (current_state_ != InitState::VERIFYING) return;

    double current_x = msg->pose.position.x;
    double current_y = msg->pose.position.y;
    double current_yaw = getYawFromQuaternion(msg->pose.orientation);

    verifyPose(current_x, current_y, current_yaw);
}

void LocalizationManager::verifyPose(double current_x, double current_y, double current_yaw) {
    double dist = std::sqrt(std::pow(current_x - this->target_x_, 2)
                          + std::pow(current_y - this->target_y_, 2));
    double yaw_diff = std::atan2(std::sin(current_yaw - target_yaw_),
                                 std::cos(current_yaw - this->target_yaw_));
    yaw_diff = std::abs(yaw_diff);

    if (dist < tolerance_dist_ && yaw_diff < tolerance_yaw_) {
        RCLCPP_INFO(this->get_logger(),
            "verify pass, linear error: %.3f m, angular error: %.3f rad", dist, yaw_diff);
        current_state_ = InitState::CLEARING_COSTMAP;
        clearNav2Costmaps();
        return;
    }

    RCLCPP_INFO(this->get_logger(),
        "verify not yet, linear error: %.3f m, angular error: %.3f rad", dist, yaw_diff);

    auto current_time = this->get_clock()->now();
    double elapsed_time = (current_time - verification_start_time_).seconds();
    if (elapsed_time > verify_timeout_sec_) {
        RCLCPP_ERROR(this->get_logger(),
            "waiting over %.1f seconds, init fail", verify_timeout_sec_);
        publishStatus(false);
        current_state_ = InitState::IDLE;
    }
}

void LocalizationManager::clearNav2Costmaps() {
    auto req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    // 不等待回覆：costmap 清除失敗不影響定位正確性，Nav2 之後也會自行更新
    global_costmap_cli_->async_send_request(req);
    local_costmap_cli_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "clearing Nav2 costmaps");

    current_state_ = InitState::SUCCESS;
    publishStatus(true);
    this->success_start_time_ = this->get_clock()->now();
}

void LocalizationManager::publishStatus(bool ok) {
    std_msgs::msg::Bool status_msg;
    status_msg.data = ok;
    status_pub_->publish(status_msg);
}

double LocalizationManager::getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

} // namespace tdk_localization

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tdk_localization::LocalizationManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
