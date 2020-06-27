#include <pcl_ransac_ros/cloud_model_fit.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace ariitk::pcl_ransac_ros {

void CloudModelFitter::init(ros::NodeHandle& nh) {
    pc_fit_pub_ = nh.advertise<sensor_msgs::PointCloud2>("fit_cloud", 1);
    model_pub_ = nh.advertise<pcl_ransac_msgs::CloudModel>("model", 1);

    pc_sub_ = nh.subscribe("cloud", 1, &CloudModelFitter::cloudCallback, this);

    plane_flag_server_ = nh.advertiseService("fit_plane", &CloudModelFitter::serviceCallback, this);

    ros::NodeHandle nh_private("~");
    nh_private.getParam("do_plane", do_plane_);
}

void CloudModelFitter::run() {
    if (cloud_->empty()) {
        return;
    }
    pcl_ransac_msgs::CloudModel model_data;

    if (do_plane_) {
        plane_.fitModel(cloud_, 0.01);
        model_data.coefficients = plane_.getModelCoefficients();
        model_data.type = pcl_ransac_msgs::CloudModel::PLANE;
    } else {
        sphere_.fitModel(cloud_, 0.01);
        model_data.coefficients = sphere_.getModelCoefficients();
        model_data.type = pcl_ransac_msgs::CloudModel::SPHERE;
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    pc_fit_pub_.publish(msg);

    model_pub_.publish(model_data);
}

void CloudModelFitter::cloudCallback(const sensor_msgs::PointCloud2& msg) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_);
}

bool CloudModelFitter::serviceCallback(pcl_ransac_msgs::ToggleModel::Request& req, pcl_ransac_msgs::ToggleModel::Response& resp) {
    do_plane_ = req.do_plane.data;
    resp.success.data = true;
    resp.type_set.data = (do_plane_) ? "Plane" : "Sphere";
    return true;
}

}  // namespace ariitk::pcl_ransac_ros
