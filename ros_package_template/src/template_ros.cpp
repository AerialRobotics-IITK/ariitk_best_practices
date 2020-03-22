#include <ros_package_template/template_ros.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace ariitk::ros_package_template {

void TemplateROS::init(ros::NodeHandle& nh) {
    pc_fit_pub_ = nh.advertise<sensor_msgs::PointCloud2>("fit_cloud", 1);
    model_pub_ = nh.advertise<msg_package_template::Template>("model", 1);

    pc_sub_ = nh.subscribe("cloud", 1, &TemplateROS::cloudCallback, this);

    plane_flag_server_ = nh.advertiseService("fit_plane", &TemplateROS::serviceCallback, this);

    ros::NodeHandle nh_private("~");
    nh_private.getParam("do_plane", do_plane_);
}

void TemplateROS::run() {
    if(cloud_->empty()) {
        return;
    }

    msg_package_template::Template model_data;

    if(do_plane_) {
        plane_.fitModel(cloud_, 0.01);
        model_data.coefficients = plane_.getModelCoefficients();
        model_data.type = msg_package_template::Template::PLANE;
    } else {
        sphere_.fitModel(cloud_, 0.01);
        model_data.coefficients = sphere_.getModelCoefficients();
        model_data.type = msg_package_template::Template::SPHERE;
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    pc_fit_pub_.publish(msg);

    model_pub_.publish(model_data);
}

void TemplateROS::cloudCallback(const sensor_msgs::PointCloud2& msg) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_);
}

bool TemplateROS::serviceCallback(msg_package_template::TemplateService::Request& req, msg_package_template::TemplateService::Response& resp) {
    do_plane_ = req.do_plane.data;
    resp.success.data = true;
    resp.type_set.data = (do_plane_) ? "Plane" : "Sphere";
    return true;
}

} // namespace ariitk::ros_package_template