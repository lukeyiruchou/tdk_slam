#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <string>

class OdomTfBroadcaster : public rclcpp::Node {
public:
    OdomTfBroadcaster() : Node("odom_tf_broadcaster") {
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("corrected_odom_topic", "/odom_filtered");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_footprint");

        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("corrected_odom_topic", corrected_odom_topic_);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("base_frame", base_frame_);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 發布修復後的里程計訊息（供 Cartographer 使用）
        pub_ = this->create_publisher<nav_msgs::msg::Odometry>(corrected_odom_topic_, 10);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, qos,
            std::bind(&OdomTfBroadcaster::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "odom_tf_broadcaster 已啟動: [%s] -> [%s], TF: %s -> %s",
                    odom_topic_.c_str(), corrected_odom_topic_.c_str(),
                    odom_frame_.c_str(), base_frame_.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto current_time = this->get_clock()->now();

        // 1. 補齊 frame_id 與 child_frame_id，並校準時間戳
        nav_msgs::msg::Odometry fixed_odom = *msg;
        fixed_odom.header.stamp = current_time;
        fixed_odom.header.frame_id = odom_frame_;
        fixed_odom.child_frame_id = base_frame_;

        pub_->publish(fixed_odom);

        // 2. 發布 odom -> base_footprint 的 TF
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = odom_frame_;
        t.child_frame_id = base_frame_;

        t.transform.translation.x = fixed_odom.pose.pose.position.x;
        t.transform.translation.y = fixed_odom.pose.pose.position.y;
        t.transform.translation.z = fixed_odom.pose.pose.position.z;
        t.transform.rotation = fixed_odom.pose.pose.orientation;

        tf_broadcaster_->sendTransform(t);
    }

    std::string odom_topic_;
    std::string corrected_odom_topic_;
    std::string odom_frame_;
    std::string base_frame_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
