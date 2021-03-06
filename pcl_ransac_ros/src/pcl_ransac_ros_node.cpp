#include <pcl_ransac_ros/cloud_model_fit.hpp>
#include <pcl_ransac_ros/point_cloud_pub.hpp>

using namespace ariitk::pcl_ransac_ros;

int main(int argc, char** argv) {
    ros::init(argc, argv, "model_node");
    ros::NodeHandle nh;

    CloudModelFitter fitter;
    PointCloudPublisher generator;

    fitter.init(nh);
    generator.init(nh);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        generator.run();
        ros::spinOnce();
        fitter.run();
        loop_rate.sleep();
    }

    return 0;
}
