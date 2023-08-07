//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

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
#include "lidar.h"
#include "LaserProcessingClass.h"

using namespace std::chrono_literals;

class LaserProcessingNode : public rclcpp::Node {
public:
    LaserProcessingNode() : Node("laser_processing_node") {
        this->declare_parameter("scan_period", 0.1);
        this->declare_parameter("vertical_angle", 2.0);
        this->declare_parameter("max_dis", 60.0);
        this->declare_parameter("min_dis", 2.0);
        this->declare_parameter("scan_line", 16.0);
        
        
        rclcpp::Parameter scan_period_param = this->get_parameter("scan_period");
        rclcpp::Parameter vertical_angle_param = this->get_parameter("vertical_angle");
        rclcpp::Parameter max_dis_param = this->get_parameter("max_dis");
        rclcpp::Parameter min_dis_param = this->get_parameter("min_dis");
        rclcpp::Parameter scan_line_param = this->get_parameter("scan_line");
        
        lidar_param.setScanPeriod(scan_period_param.get_parameter_value().get<double>());
        lidar_param.setVerticalAngle(vertical_angle_param.get_parameter_value().get<double>());
        lidar_param.setLines(scan_line_param.get_parameter_value().get<double>());
        lidar_param.setMaxDistance(max_dis_param.get_parameter_value().get<double>());
        lidar_param.setMinDistance(min_dis_param.get_parameter_value().get<double>());
        
        laserProcessing.init(lidar_param);
        
        subLaserCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                                                                                  "/velodyne_points", 100, std::bind(&LaserProcessingNode::velodyneHandler, this, std::placeholders::_1));
        
        pubLaserCloudFiltered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                                                                                       "/velodyne_points_filtered", 100);
        pubEdgePoints_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_edge", 100);
        pubSurfPoints_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surf", 100);
        
        laserProcessingProcess_ = std::thread(&LaserProcessingNode::laserProcessing, this);
    }
    
    void velodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
        std::lock_guard<std::mutex> lock(mutex_lock);
        pointCloudBuf.push(laserCloudMsg);
    }
    
    void laser_processing() {
        while (rclcpp::ok()) {
            if (!pointCloudBuf.empty()) {
                //read data
                sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg;
                {
                    std::lock_guard<std::mutex> lock(mutex_lock);
                    laserCloudMsg = pointCloudBuf.front();
                    pointCloudBuf.pop();
                }
                
                pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::fromROSMsg(*laserCloudMsg, *pointcloud_in);
                rclcpp::Time pointcloud_time = laserCloudMsg->header.stamp;
                
                pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());
                pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());
                
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                //laserProcessing.featureExtraction(pointcloud_in, pointcloud_edge, pointcloud_surf);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                
                float time_temp = elapsed_seconds.count() * 1000;
                
                pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
                *pointcloud_filtered += *pointcloud_edge;
                *pointcloud_filtered += *pointcloud_surf;
                
                sensor_msgs::msg::PointCloud2::SharedPtr laserCloudFilteredMsg(new sensor_msgs::msg::PointCloud2);
                pcl::toROSMsg(*pointcloud_filtered, *laserCloudFilteredMsg);
                laserCloudFilteredMsg->header.stamp = pointcloud_time;
                laserCloudFilteredMsg->header.frame_id = "base_link";
                //pubLaserCloudFiltered_->publish(laserCloudFilteredMsg);
                
                sensor_msgs::msg::PointCloud2::SharedPtr edgePointsMsg(new sensor_msgs::msg::PointCloud2);
                pcl::toROSMsg(*pointcloud_edge, *edgePointsMsg);
                edgePointsMsg->header.stamp = pointcloud_time;
                edgePointsMsg->header.frame_id = "base_link";
                //pubEdgePoints_->publish(edgePointsMsg);
                
                sensor_msgs::msg::PointCloud2::SharedPtr surfPointsMsg(new sensor_msgs::msg::PointCloud2);
                pcl::toROSMsg(*pointcloud_surf, *surfPointsMsg);
                surfPointsMsg->header.stamp = pointcloud_time;
                surfPointsMsg->header.frame_id = "base_link";
                //pubSurfPoints_->publish(surfPointsMsg);
            }
            
            rclcpp::sleep_for(2ms);
        }
    }
    
    
private:
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFiltered_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubEdgePoints_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPoints_;
    
    std::mutex mutex_lock;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudBuf;
    lidar::Lidar lidar_param;
    LaserProcessingClass laserProcessing;
    std::thread laserProcessingProcess_;
    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserProcessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

