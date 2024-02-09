#include <iostream>
#include <deque> 
#include <chrono>

#include <pcl_conversions/pcl_conversions.h>  // deprecated?
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <glob.h>

void glob(const std::string& globPath, std::deque<std::string>& matchingFiles) {
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
        this->declare_parameter("frame_id", "laser_link");
        this->get_parameter("frame_id", frame_id_);
        this->declare_parameter("pub_period_ms", "100.0");
        this->get_parameter("pub_period_ms", pub_period_ms_);

        // Initializes the publisher.
        cloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, 10);
          
        // Collect the list of all the files fitting with the glob  
        glob(cloud_file_glob, cloud_files_); 
        RCLCPP_INFO_STREAM(this->get_logger(), "cloud_file_glob: \"" << cloud_file_glob << "\": found " << cloud_files_.size());
        
        // Initializes the point cloud publisher
        cloudPub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 10);  
            
        auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<float>(pub_period_ms_));
        timer_ = this->create_wall_timer(
            period, std::bind(&CloudPublisherNode::publishPeriodic, this));
    }
    

    void publishPeriodic() {
      if (cloud_files_.empty()) {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "list of cloud files finished: no more cloud to be published");
        return; 
      }
      
      std::string cloudFilename = cloud_files_.front();
      cloud_files_.pop_front();
    
      pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);
      RCLCPP_DEBUG_STREAM(this->get_logger(), "reading file " << cloudFilename);
      if (pcl::io::loadPCDFile<pcl::PointXYZI> (cloudFilename.c_str(), *points) == -1) {
         RCLCPP_ERROR_STREAM(this->get_logger(), "Could not read file: " << cloudFilename);
         return;
       }
      

       //workaround for the PCL headers... http://wiki.ros.org/hydro/Migration#PCL
       sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg(new sensor_msgs::msg::PointCloud2);
       pcl::toROSMsg(*points, *cloudMsg);
       cloudMsg->header.frame_id = frame_id_; //ros::this_node::getName();
       cloudMsg->header.stamp = this->now();
       cloudPub_->publish(*cloudMsg);
    }

   private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string cloud_topic_;
    std::string frame_id_;
    float pub_period_ms_;
    std::deque<std::string> cloud_files_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloudPublisherNode>();
    RCLCPP_INFO(node->get_logger(), "node started");

    rclcpp::spin(node);
    return 0;
}
