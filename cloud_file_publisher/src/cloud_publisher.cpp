#include <iostream>

#include <pcl_conversions/pcl_conversions.h>  // deprecated?
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <glob.h>

void glob(const std::string& globPath, std::vector<std::string>& matchingFiles) {
    glob_t glob_result;
    matchingFiles.clear();
    std::cout << "global path: \"" << globPath << "\"" << std::endl;
    int ret = glob(globPath.c_str(), GLOB_TILDE, NULL, &glob_result);
    if(ret != 0) {
        globfree(&glob_result);
        std::cerr << "glob() failed: return value " << ret << std::endl;
        return;
    }
    for (unsigned int i = 0; i < glob_result.gl_pathc; ++i) {
        matchingFiles.push_back(std::string(glob_result.gl_pathv[i]));
        //std::cout << "found file \"" << matchingFiles.back() << "\"" << std::endl;
    }
    globfree(&glob_result);
}


class CloudPublisherNode : public rclcpp::Node {
   public:
    /**
     * Constructor of the node. It reads the parameters and set the callback
     * to subscribe to point cloud messages.
     */
    CloudPublisherNode() : Node("cloud_publisher") {
    	std::string cloud_file_glob;
    
        // Declare the parameters (the name of the point cloud topic)
        this->declare_parameter("cloud_topic", "/points");
        this->get_parameter("cloud_topic", cloud_topic_);
        this->declare_parameter("cloud_file_glob", "");
        this->get_parameter("cloud_file_glob", cloud_file_glob);

        // Initializes the publisher.
        cloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, 10);
            
        glob(cloud_file_glob, cloud_files_); 
        RCLCPP_INFO_STREAM(this->get_logger(), "cloud_file_glob: \"" << cloud_file_glob << "\": found " << cloud_files_);  
        
        timer_ = this->create_wall_timer(
            50ms, std::bind(&AvoidObstacleNode::commandPeriodic, this));
    }

//    /**
//     * Callback to handle new laser point cloud messages.
//     */
//    void onPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
//        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn(
//            new pcl::PointCloud<pcl::PointXYZI>);

//        // pcl::fromROSMsg() is deprecated, but it is the simpler API to convert
//        // from sensor_msgs::msg::PointCloud2 to pcl::PointCloud<> Warning:
//        // installing external PCL and ROS2 in Ubuntu 22.04 you may have
//        // duplicate versions:
//        // - /usr/include/pcl_conversions: uses ROS1 sensor_msgs::PointCloud2
//        //   void fromROSMsg(const sensor_msgs::PointCloud2 &cloud,
//        //   pcl::PointCloud<T> &pcl_cloud);
//        // - /opt/ros/humble/include/pcl_conversions: uses ROS2 message
//        //   void fromROSMsg(const sensor_msgs::msg::PointCloud2 &cloud,
//        //   pcl::PointCloud<T> &pcl_cloud)
//        pcl::fromROSMsg(
//            *msg,
//            *cloudIn);  // possibly deprecated now, but more straightforward
//        // We use: void toPCL(const sensor_msgs::msg::PointCloud2 &pc2,
//        // pcl::PCLPointCloud2 &pcl_pc2)
//        //   	pcl::PCLPointCloud2::Ptr cloudPcl(new pcl::PCLPointCloud2);
//        //        pcl_conversions::toPCL(*msg, *cloudPcl);

//        std::cout << "received point cloud message with "
//                  << cloudIn->points.size() << " points" << std::endl;
//    }

   private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string cloud_topic_;
    std::vector<std::string> cloud_files_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloudPublisherNode>();
    RCLCPP_INFO(node->get_logger(), "node started");

    rclcpp::spin(node);
    return 0;
}
