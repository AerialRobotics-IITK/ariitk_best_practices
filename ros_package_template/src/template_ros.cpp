#include <ros_package_template/template_ros.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace ariitk::ros_package_template {

void TemplateROS::init(ros::NodeHandle& nh) {
    pc_fit_pub_ = nh.advertise<sensor_msgs::PointCloud2>("fit_cloud", 1);
    pc_sub_ = nh.subscribe("cloud", 1, &TemplateROS::cloudCallback, this);

    ros::NodeHandle nh_private("~");
    nh_private.getParam("do_plane", do_plane_);
}

void TemplateROS::run() {
    if(do_plane_) {
        plane_.fitModel(cloud_, 0.01);
    } else {
        sphere_.fitModel(cloud_, 0.01);
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    pc_fit_pub_.publish(msg);
}

void TemplateROS::cloudCallback(const sensor_msgs::PointCloud2& msg) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_);
}

} // namespace ariitk::ros_package_template