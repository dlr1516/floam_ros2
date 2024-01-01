// ros 2 lib
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>

class MarkerVisualizationNode : public rclcpp::Node
{
public:
    MarkerVisualizationNode() : Node("marker_visualization_node")
    {
        laserOdometrySub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&MarkerVisualizationNode::publishMarkers, this, std::placeholders::_1));
        markerPub_ = this->create_publisher<visualization_msgs::msg::Marker>( "/visualization_marker", 100 );
    }

    void publishMarkers(const nav_msgs::msg::Odometry::ConstSharedPtr &odomMsg){
            marker.header.frame_id = "map";
            marker.header.stamp = odomMsg->header.stamp;
            marker.ns = "floam";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1;
            marker.scale.x = 0.03;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            geometry_msgs::msg::Point p;
            p.x = odomMsg->pose.pose.position.x;
            p.y = odomMsg->pose.pose.position.y;
            p.z = odomMsg->pose.pose.position.z;
            marker.points.push_back(p);

            markerPub_->publish( std::move(marker));
    }

private:
    visualization_msgs::msg::Marker marker;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr laserOdometrySub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkerVisualizationNode>();
    RCLCPP_INFO(node->get_logger(), "node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}