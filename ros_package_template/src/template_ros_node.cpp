#include <ros_package_template/template_ros.hpp>
#include <ros_package_template/template_ros_pub.hpp>

using namespace ariitk::ros_package_template;

int main(int argc, char** argv) {
    ros::init(argc, argv, "template_node");
    ros::NodeHandle nh;
    
    TemplateROS fitter;
    TemplateROSPub generator;
    
    fitter.init(nh);
    generator.init(nh);

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        generator.run();
        ros::spinOnce();
        fitter.run();
        loop_rate.sleep();
    }

    return 0;
}