#include <pcl_ransac_ros/point_cloud_pub.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace ariitk::pcl_ransac_ros {

void PointCloudPublisher::init(ros::NodeHandle& nh) {
    pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);

    plane_flag_server_ = nh.advertiseService("gen_plane", &PointCloudPublisher::serviceCallback, this);

    ros::NodeHandle nh_private("~");
    nh_private.getParam("do_plane", do_plane_);
    nh_private.getParam("width", width_);
    nh_private.getParam("height", height_);

    cloud_->width = width_;
    cloud_->height = height_;
    cloud_->is_dense = false;
}

void PointCloudPublisher::run() {
    if (do_plane_) {
        plane_.generatePointCloud(cloud_);
    } else {
        sphere_.generatePointCloud(cloud_);
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    pc_pub_.publish(msg);
}

bool PointCloudPublisher::serviceCallback(pcl_ransac_msgs::ToggleModel::Request& req, pcl_ransac_msgs::ToggleModel::Response& resp) {
    do_plane_ = req.do_plane.data;
    resp.success.data = true;
    resp.type_set.data = (do_plane_) ? "Plane" : "Sphere";
    return true;
}

}  // namespace ariitk::pcl_ransac_ros
