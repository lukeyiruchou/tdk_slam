// ============================================================================
// robot_pose_publisher
//
// 以 30Hz lookup TF: world -> base_footprint，發布機器人「全局座標」
// （相對場地左下角 world(0,0)）到 /robot_pose。
//
// 與學長版本的差異：
//   * lookup 失敗時不 publish（學長版會把上一筆舊 pose / 全零 pose 發出去，
//     啟動初期 TF 樹還沒建好時，可能讓 localization_manager 的 VERIFYING
//     誤判通過或誤判失敗）。
//   * header 補上 frame_id 與 stamp，下游比較好除錯。
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class RobotPosePub : public rclcpp::Node {
public:
    RobotPosePub() : Node("robot_pose_pub"), buffer_(this->get_clock()), listener_(buffer_) {
        this->declare_parameter<std::string>("parent_frame", "world");
        this->declare_parameter<std::string>("child_frame", "base_footprint");
        this->get_parameter("parent_frame", parent_frame_);
        this->get_parameter("child_frame", child_frame_);

        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "robot_pose", rclcpp::QoS(10).reliable().durability_volatile());

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30Hz
            std::bind(&RobotPosePub::lookupTransform, this));

        RCLCPP_INFO(this->get_logger(), "robot_pose_publisher: %s -> %s",
                    parent_frame_.c_str(), child_frame_.c_str());
    }

private:
    void lookupTransform() {
        try {
            auto transformStamped = buffer_.lookupTransform(
                parent_frame_,
                child_frame_,
                rclcpp::Time());  // latest available

            geometry_msgs::msg::PoseStamped robot_pose;
            robot_pose.header.stamp = transformStamped.header.stamp;
            robot_pose.header.frame_id = parent_frame_;
            robot_pose.pose.position.x = transformStamped.transform.translation.x;
            robot_pose.pose.position.y = transformStamped.transform.translation.y;
            robot_pose.pose.position.z = 0.0;
            robot_pose.pose.orientation = transformStamped.transform.rotation;

            // lookup 成功才 publish，避免發出舊資料或全零 pose
            pub_->publish(robot_pose);
        } catch (const tf2::TransformException & ex) {
            // 啟動初期 TF 樹還沒完整是正常的，用 throttle 避免洗版
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "failed to lookup transform %s -> %s: %s",
                parent_frame_.c_str(), child_frame_.c_str(), ex.what());
        }
    }

    std::string parent_frame_;
    std::string child_frame_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotPosePub>());
    rclcpp::shutdown();
    return 0;
}
