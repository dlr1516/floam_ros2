
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



class LaserMappingNode : public rclcpp::Node {
    
public:
    //costrutture del nodo
    LaserMappingNode() : Node("laser_mapping_node"){
        
        
        
        //dichiarazione dei parametri
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
        
        lidar_param.setScanPeriod(scan_period_param.get_parameter_value().get<double>());
        lidar_param.setVerticalAngle(vertical_angle_param.get_parameter_value().get<double>());
        lidar_param.setLines(scan_line_param.get_parameter_value().get<double>());
        lidar_param.setMaxDistance(max_dis_param.get_parameter_value().get<double>());
        lidar_param.setMinDistance(min_dis_param.get_parameter_value().get<double>());
        
        laserMapping.init(map_resolution_param.get_parameter_value().get<double>());
        
        
        //inizializzazione scan
        
        subLaserCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                                                                                  "/velodyne_points_filtered", 100, std::bind(&LaserMappingNode::velodyneHandler, this, std::placeholders::_1));
        
        subOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
                                                                          "/odom", 100, std::bind(&LaserMappingNode::odomCallback, this, std::placeholders::_1));
        
        //inizializzazione publisher
        
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map",100);
        laser_mapping_process_ =std::thread(&LaserMappingNode::laser_mapping, this);
        
        
        
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        
        mutex_lock.lock();
        odometryBuf.push(msg);
        mutex_lock.unlock();
        if(!pointCloudBuf.empty())
            laser_mapping_queue.notify_one();
    }
    
    void velodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg){
        
        mutex_lock.lock();
        pointCloudBuf.push(laserCloudMsg);
        mutex_lock.unlock();
        if(!odometryBuf.empty())
            laser_mapping_queue.notify_one();
        
    }
    
    
    
    void laser_mapping(){
        
        const double TOLL = 0.5 * lidar_param.getScanPeriod();
        
        while (rclcpp::ok()) {
            
            mutex_lock.lock();
            
            std::unique_lock<std::mutex> lock(mutex_lock);
            while(pointCloudBuf.empty() || odometryBuf.empty()){
                
                laser_mapping_queue.wait(lock);
            }
            
            
            using namespace std::chrono;
            
            while (!odometryBuf.empty() && !pointCloudBuf.empty() &&
                   (odometryBuf.front()->header.stamp.sec < pointCloudBuf.front()->header.stamp.sec - duration_cast<seconds>(milliseconds(static_cast<int>(TOLL * 1000))).count())) {
                
                if (odometryBuf.front()->header.stamp.sec < pointCloudBuf.front()->header.stamp.sec - duration_cast<seconds>(milliseconds(static_cast<int>(TOLL * 1000))).count()) {
                    odometryBuf.pop();
                }
                if (pointCloudBuf.front()->header.stamp.sec < odometryBuf.front()->header.stamp.sec - duration_cast<seconds>(milliseconds(static_cast<int>(TOLL * 1000))).count()) {
                    pointCloudBuf.pop();
                }
            }
            
            
            
            if (odometryBuf.empty() || pointCloudBuf.empty()) {
                
                mutex_lock.unlock();
                // Sblocca la mutex prima di continuare
                continue;
            }
            
            
            
            //if time aligned
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            
            pcl::fromROSMsg(*pointCloudBuf.front(),*pointcloud_in);
            rclcpp::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.rotate(Eigen::Quaterniond(odometryBuf.front()->pose.pose.orientation.w,odometryBuf.front()->pose.pose.orientation.x,odometryBuf.front()->pose.pose.orientation.y,odometryBuf.front()->pose.pose.orientation.z));
            current_pose.pretranslate(Eigen::Vector3d(odometryBuf.front()->pose.pose.position.x,odometryBuf.front()->pose.pose.position.y,odometryBuf.front()->pose.pose.position.z));
            
            pointCloudBuf.pop();
            odometryBuf.pop();
            mutex_lock.unlock();
            
            
            
            laserMapping.updateCurrentPointsToMap(pointcloud_in,current_pose);
            
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = laserMapping.getMap();
            
            sensor_msgs::msg::PointCloud2::SharedPtr PointsMsg(new sensor_msgs::msg::PointCloud2);
            pcl::toROSMsg(*pc_map, *PointsMsg); 
            PointsMsg->header.frame_id = "map";
            //map_pub_->publish(*PointsMsg);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        
        
        
        
    }
    
    
    
private:
    
    lidar::Lidar lidar_param;
    LaserMappingClass laserMapping;
    
    std::queue<nav_msgs::msg::Odometry::SharedPtr> odometryBuf;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudBuf;
    
    
    std::mutex mutex_lock;
    std::thread laser_mapping_process_;
    std::condition_variable laser_mapping_queue;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;
    
    
    
    
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<LaserMappingNode>();
    RCLCPP_INFO(node->get_logger(), "node started");
    
    rclcpp::spin(node);
    return 0;
    
}
