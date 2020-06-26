#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ransac/pcl_ransac.hpp>
#include <pcl_ransac_msgs/ToggleModel.h>

namespace ariitk::pcl_ransac_ros {

class PointCloudPublisher {
    public:
    PointCloudPublisher() : cloud_(new pcl::PointCloud<pcl::PointXYZ>){};

    void init(ros::NodeHandle& nh);
    void run();

    private:
    bool serviceCallback(pcl_ransac_msgs::ToggleModel::Request& req, pcl_ransac_msgs::ToggleModel::Response& resp);

    ros::Publisher pc_pub_;

    ros::ServiceServer plane_flag_server_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    ariitk::pcl_ransac::CloudModelPlane plane_;
    ariitk::pcl_ransac::CloudModelSphere sphere_;

    bool do_plane_;

    int width_;
    int height_;
};

}  // namespace ariitk::pcl_ransac_ros
