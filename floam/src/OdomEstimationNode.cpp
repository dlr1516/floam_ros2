//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros 2 lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "OdomEstimationClass.h"


class OdomEstimationNode : public rclcpp::Node {
public:
    OdomEstimationNode() : Node("odom_estimation_node") {
        
        this->declare_parameter("scan_period", 0.1);
        this->declare_parameter("vertical_angle", 2.0);
        this->declare_parameter("max_dis", 60.0);
        this->declare_parameter("min_dis", 2.0);
        this->declare_parameter("scan_line", 16.0);
        this->declare_parameter("map_resolution", 0.4);
        
        rclcpp::Parameter scan_period_param = this->get_parameter("scan_period");
        rclcpp::Parameter vertical_angle_param = this->get_parameter("vertical_angle");
        rclcpp::Parameter max_dis_param = this->get_parameter("max_dis");
        rclcpp::Parameter min_dis_param = this->get_parameter("min_dis");
        rclcpp::Parameter scan_line_param = this->get_parameter("scan_line");
        rclcpp::Parameter map_resolution_param = this->get_parameter("map_resolution");
        
        
        lidarParam_.setScanPeriod(scan_period_param.get_parameter_value().get<double>());
        lidarParam_.setVerticalAngle(vertical_angle_param.get_parameter_value().get<double>());
        lidarParam_.setLines(scan_line_param.get_parameter_value().get<double>());
        lidarParam_.setMaxDistance(max_dis_param.get_parameter_value().get<double>());
        lidarParam_.setMinDistance(min_dis_param.get_parameter_value().get<double>());
        
        odomEstimation_.init(lidarParam_, map_resolution_param.get_parameter_value().get<double>());
        
        
        edgeLaserCloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/laser_cloud_edge", 100, std::bind(&OdomEstimationNode::velodyneEdgeHandler, this, std::placeholders::_1));
        
        surfLaserCloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/laser_cloud_surf", 100, std::bind(&OdomEstimationNode::velodyneSurfHandler, this, std::placeholders::_1));
        
        laserOdometryPub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 100);

        markerPub_ = this->create_publisher<visualization_msgs::msg::Marker>( "/visualization_marker", 100 );
        
        br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        is_odom_inited_ = false;
        total_time_ = 0;
        total_frame_ = 0;

        odom_estimation_thread_ = std::thread(std::bind(&OdomEstimationNode::odom_estimation, this), this);
    }
    
    void velodyneSurfHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &laserCloudMsg) {
        mutex_lock_.lock();
        pointCloudSurfBuf_.push(laserCloudMsg);
        mutex_lock_.unlock();
        if(!pointCloudEdgeBuf_.empty()){
            odom_estimation_queue_.notify_one();
        }
    }
    
    void velodyneEdgeHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &laserCloudMsg) {
        mutex_lock_.lock();
        pointCloudEdgeBuf_.push(laserCloudMsg);
        mutex_lock_.unlock();
        if(!pointCloudSurfBuf_.empty())
            odom_estimation_queue_.notify_one();
    }
    
    void odom_estimation() {
        const double TOLL = 0.5 * lidarParam_.getScanPeriod();
        
        while (rclcpp::ok()) {
            
            std::unique_lock<std::mutex> lock(mutex_lock_);
            while(pointCloudEdgeBuf_.empty() || pointCloudSurfBuf_.empty()){
                
                odom_estimation_queue_.wait(lock);
            }
            lock.unlock();
            
            using namespace std::chrono;
            mutex_lock_.lock();
            while (!pointCloudSurfBuf_.empty() && !pointCloudEdgeBuf_.empty() &&
                   (pointCloudSurfBuf_.front()->header.stamp.sec < pointCloudEdgeBuf_.front()->header.stamp.sec - duration_cast<seconds>(milliseconds(static_cast<int>(TOLL * 1000))).count())) {
                
                if (pointCloudSurfBuf_.front()->header.stamp.sec < pointCloudEdgeBuf_.front()->header.stamp.sec - duration_cast<seconds>(milliseconds(static_cast<int>(TOLL * 1000))).count()) {
                    pointCloudSurfBuf_.pop();
                }
                if (pointCloudEdgeBuf_.front()->header.stamp.sec < pointCloudSurfBuf_.front()->header.stamp.sec - duration_cast<seconds>(milliseconds(static_cast<int>(TOLL * 1000))).count()) {
                    pointCloudEdgeBuf_.pop();
                }
            }
            
            if (pointCloudSurfBuf_.empty() || pointCloudEdgeBuf_.empty()) {
                
                mutex_lock_.unlock();
                continue;
            }
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudEdgeBuf_.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf_.front(), *pointcloud_surf_in);
            rclcpp::Time pointcloud_time = (pointCloudSurfBuf_.front())->header.stamp;
            pointCloudEdgeBuf_.pop();
            pointCloudSurfBuf_.pop();
            mutex_lock_.unlock();
            
            if (!is_odom_inited_) {
                odomEstimation_.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited_ = true;
                RCLCPP_INFO(get_logger(), "odom inited");
            } else {
                auto start = std::chrono::system_clock::now();
                odomEstimation_.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
                auto end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame_++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time_ += time_temp;
                RCLCPP_INFO(get_logger(), "average odom estimation time %f ms", total_time_ / total_frame_);
            }
            
            Eigen::Quaterniond q_current(odomEstimation_.odom.rotation());
            Eigen::Vector3d t_current = odomEstimation_.odom.translation();
            
            geometry_msgs::msg::TransformStamped transform;
            transform.header.frame_id = "map";
            transform.child_frame_id = "base_link";
            transform.header.stamp = pointcloud_time;
            transform.transform.translation.x = t_current.x();
            transform.transform.translation.y = t_current.y();
            transform.transform.translation.z = t_current.z();
            transform.transform.rotation.x = q_current.x();
            transform.transform.rotation.y = q_current.y();
            transform.transform.rotation.z = q_current.z();
            transform.transform.rotation.w = q_current.w();
            br_->sendTransform(transform);
            
            auto laserOdometry = std::make_unique<nav_msgs::msg::Odometry>();
            laserOdometry->header.frame_id = "map";
            laserOdometry->child_frame_id = "base_link";
            laserOdometry->header.stamp = pointcloud_time;
            laserOdometry->pose.pose.orientation.x = q_current.x();
            laserOdometry->pose.pose.orientation.y = q_current.y();
            laserOdometry->pose.pose.orientation.z = q_current.z();
            laserOdometry->pose.pose.orientation.w = q_current.w();
            laserOdometry->pose.pose.position.x = t_current.x();
            laserOdometry->pose.pose.position.y = t_current.y();
            laserOdometry->pose.pose.position.z = t_current.z();
            laserOdometryPub_->publish(std::move(laserOdometry));

            publishMarkers(t_current, pointcloud_time);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        
    }

    void publishMarkers(Eigen::Vector3d t, rclcpp::Time stamp){
            marker.header.frame_id = "map";
            marker.header.stamp = stamp;
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
            p.x = t.x();
            p.y = t.y();
            p.z = t.z();
            marker.points.push_back(p);

            markerPub_->publish( std::move(marker));
    }
    
    
    
private:
    
    lidar::Lidar lidarParam_;
    OdomEstimationClass odomEstimation_;
    
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pointCloudEdgeBuf_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pointCloudSurfBuf_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr edgeLaserCloudSub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr surfLaserCloudSub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr laserOdometryPub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub_;

    visualization_msgs::msg::Marker marker;
    
    bool is_odom_inited_;
    double total_time_;
    int total_frame_;
    std::thread odom_estimation_thread_;
    std::condition_variable odom_estimation_queue_;
    std::mutex mutex_lock_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomEstimationNode>();
    RCLCPP_INFO(node->get_logger(), "node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
