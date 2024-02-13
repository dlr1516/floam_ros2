#include <iostream>
#include <deque> 
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


class PathSaveNode : public rclcpp::Node {
public:
  PathSaveNode() : Node("path_save_node") {
    this->declare_parameter("input_mode", "tf");
    this->get_parameter("input_mode", input_mode_);
    // input_mode "tf" requires frame_id and child_frame_it
    this->declare_parameter("frame_id", "map");
    this->get_parameter("frame_id", frame_id_);
    this->declare_parameter("child_frame_id", "laser_link");
    this->get_parameter("child_frame_id", child_frame_id_);
    this->declare_parameter("tf_period_sec", 0.100);
    this->get_parameter("tf_period_sec", tf_period_sec_);
    // input_mode "odometry" saves the odometry topic
    this->declare_parameter("odometry_topic", "/odom");
    this->get_parameter("odometry_topic", odometry_topic_);
    // input_mode "trajectory" saves the whole trajectory provided as topic
    this->declare_parameter("path_topic", "/path");
    this->get_parameter("path_topic", path_topic_);
    // output_mode
    this->declare_parameter("output_mode", "quarternion");
    this->get_parameter("output_mode", output_mode_);
    
    // Creates the transform listener with buffer
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    if (input_mode_ == "odometry") {
      RCLCPP_INFO_STREAM(this->get_logger(), "Mode ODOMETRY: listening on topic \"" 
        << odometry_topic_ << "\"");
      odometrySub_ = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_, 10,         
        std::bind(&PathSaveNode::odomCallback, this, std::placeholders::_1));
    }
    else if (input_mode_ == "path") {
      RCLCPP_INFO_STREAM(this->get_logger(), "Mode PATH: listening on topic \"" 
        << path_topic_ << "\"");
      pathSub_ = this->create_subscription<nav_msgs::msg::Path>(path_topic_, 10,         
        std::bind(&PathSaveNode::pathCallback, this, std::placeholders::_1));
    }
    else {
      auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<float>(tf_period_sec_));
      RCLCPP_INFO_STREAM(this->get_logger(), "Mode TF: frame_id \"" 
        << frame_id_ << "\", child_frame_id \"" << child_frame_id_ << "\", period [ms] "
        << std::chrono::duration_cast<std::chrono::milliseconds>(period).count());
      timer_ = this->create_wall_timer(
        period, std::bind(&PathSaveNode::readTfPeriodic, this));
    }
  }
  
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){

  }
  
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg){

  }
  
  void readTfPeriodic() {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(child_frame_id_, frame_id_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO_STREAM(this->get_logger(), 
        "Could not transform child_frame_id \"" << child_frame_id_ << "\" to frame_id \"" 
          << frame_id_ << "\":\n  " << ex.what());
      return;
    }
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSub_;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string input_mode_;
  std::string output_mode_;
  std::string frame_id_;
  std::string child_frame_id_;
  double tf_period_sec_;
  std::string odometry_topic_;
  std::string path_topic_;
};   


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathSaveNode>();
    RCLCPP_INFO(node->get_logger(), "node started");

    rclcpp::spin(node);
    return 0;
}

