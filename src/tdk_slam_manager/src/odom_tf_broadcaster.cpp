#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <string>

// 訂閱 STM32 (micro-ROS) 發出的 /odom，並廣播動態 tf: odom -> base_footprint。
// cartographer 在 provide_odom_frame=false 下需要這條 tf 已存在，才能反推 map -> odom。
class OdomTfBroadcaster : public rclcpp::Node {
public:
    OdomTfBroadcaster() : Node("odom_tf_broadcaster") {
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_footprint");

        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("base_frame", base_frame_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // best_effort 訂閱可同時相容 reliable / best_effort 的 publisher，
        // micro-ROS 端常用 best_effort，這樣設定才保證收得到。
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, qos,
            std::bind(&OdomTfBroadcaster::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "odom_tf_broadcaster 已啟動: %s -> %s (topic: %s)",
                    odom_frame_.c_str(), base_frame_.c_str(), odom_topic_.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped t;

        // 實機 micro-ROS 來源先不要信任 STM32 timestamp，
        // 直接用上位機 ROS time，避免 TF 落後 scan timestamp。
        t.header.stamp = this->get_clock()->now();

        t.header.frame_id = odom_frame_;
        t.child_frame_id = base_frame_;

        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;
        t.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(t);
    }

    std::string odom_topic_;
    std::string odom_frame_;
    std::string base_frame_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}