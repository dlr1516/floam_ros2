//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <condition_variable>

//ros 2 lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "LaserMappingClass.h"
#include "lidar.h"

using namespace std::chrono;

class LaserMappingNode : public rclcpp::Node {
public:
    LaserMappingNode() : Node("laser_mapping_node"){
        
        this->declare_parameter("scan_period",0.1);
        this->declare_parameter("vertical_angle",2.0);
        this->declare_parameter("max_dis",60.0);
        this->declare_parameter("min_dis",2.0);
        this->declare_parameter("scan_line",16.0);
        this->declare_parameter("map_resolution",0.4);
        
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
        
        laserMapping_.init(map_resolution_param.get_parameter_value().get<double>());
        
        laserCloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                                                                                  "/velodyne_points_filtered", 100, std::bind(&LaserMappingNode::velodyneHandler, this, std::placeholders::_1));
        
        odometrySub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                                                                          "/odom", 100, std::bind(&LaserMappingNode::odomCallback, this, std::placeholders::_1));
        
        mapPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map",100);
        laser_mapping_process_ =std::thread(std::bind(&LaserMappingNode::laser_mapping, this), this);
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        mutex_lock_.lock();
        odometryBuf_.push(msg);
        mutex_lock_.unlock();
        if(!pointCloudBuf_.empty())
            laser_mapping_queue_.notify_one();
    }
    
    void velodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg){
        mutex_lock_.lock();
        pointCloudBuf_.push(laserCloudMsg);
        mutex_lock_.unlock();
        if(!odometryBuf_.empty())
            laser_mapping_queue_.notify_one();
    }
    
    void laser_mapping(){
        const double TOLL = 0.5 * lidarParam_.getScanPeriod();
        
        while (rclcpp::ok()) {
            
            std::unique_lock<std::mutex> lock(mutex_lock_);
            
            while(pointCloudBuf_.empty() || odometryBuf_.empty()){
                laser_mapping_queue_.wait(lock);
            }
            lock.unlock();
            
            mutex_lock_.lock();
            while (!odometryBuf_.empty() && !pointCloudBuf_.empty() &&
                   (odometryBuf_.front()->header.stamp.sec < pointCloudBuf_.front()->header.stamp.sec - duration_cast<seconds>(milliseconds(static_cast<int>(TOLL * 1000))).count())) {
                
                if (odometryBuf_.front()->header.stamp.sec < pointCloudBuf_.front()->header.stamp.sec - duration_cast<seconds>(milliseconds(static_cast<int>(TOLL * 1000))).count()) {
                    odometryBuf_.pop();
                }
                if (pointCloudBuf_.front()->header.stamp.sec < odometryBuf_.front()->header.stamp.sec - duration_cast<seconds>(milliseconds(static_cast<int>(TOLL * 1000))).count()) {
                    pointCloudBuf_.pop();
                }
            }
            
            if (odometryBuf_.empty() || pointCloudBuf_.empty()) {
                mutex_lock_.unlock();
                continue;
            }
            
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            
            pcl::fromROSMsg(*pointCloudBuf_.front(),*pointcloud_in);
            rclcpp::Time pointcloud_time = (pointCloudBuf_.front())->header.stamp;
            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.rotate(Eigen::Quaterniond(odometryBuf_.front()->pose.pose.orientation.w,odometryBuf_.front()->pose.pose.orientation.x,odometryBuf_.front()->pose.pose.orientation.y,odometryBuf_.front()->pose.pose.orientation.z));
            current_pose.pretranslate(Eigen::Vector3d(odometryBuf_.front()->pose.pose.position.x,odometryBuf_.front()->pose.pose.position.y,odometryBuf_.front()->pose.pose.position.z));
            
            pointCloudBuf_.pop();
            odometryBuf_.pop();
            mutex_lock_.unlock();
            
            laserMapping_.updateCurrentPointsToMap(pointcloud_in,current_pose);
            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = laserMapping_.getMap();
            sensor_msgs::msg::PointCloud2::SharedPtr PointsMsg(new sensor_msgs::msg::PointCloud2);
            pcl::toROSMsg(*pc_map, *PointsMsg);
            PointsMsg->header.frame_id = "map";
            mapPub_->publish(*PointsMsg);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        
    }
    
    
    
private:
    
    lidar::Lidar lidarParam_;
    LaserMappingClass laserMapping_;
    
    std::queue<nav_msgs::msg::Odometry::SharedPtr> odometryBuf_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudBuf_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr laserCloudSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mapPub_;
    
    std::thread laser_mapping_process_;
    std::condition_variable laser_mapping_queue_;
    std::mutex mutex_lock_;
    
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<LaserMappingNode>();
    RCLCPP_INFO(node->get_logger(), "node started");
    
    rclcpp::spin(node);
    return 0;
    
}
