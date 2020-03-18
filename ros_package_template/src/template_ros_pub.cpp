#include <ros_package_template/template_ros_pub.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace ariitk::ros_package_template {

void TemplateROSPub::init(ros::NodeHandle& nh) {
    pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);

    ros::NodeHandle nh_private("~");
    nh_private.getParam("do_plane", do_plane_);
    nh_private.getParam("width", width_);
    nh_private.getParam("height", height_);

    cloud_->width = width_;
    cloud_->height = height_;
    cloud_->is_dense = false;
}

void TemplateROSPub::run() {
    if(do_plane_) {
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

} 