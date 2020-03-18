#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <lib_package_template/template.hpp>

namespace ariitk::ros_package_template {
    class TemplateROSPub {
        public:
            TemplateROSPub()
                : cloud_(new pcl::PointCloud<pcl::PointXYZ>) {};
            void init(ros::NodeHandle& nh);
            void run();

        private:
            ros::Publisher pc_pub_;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

            ariitk::lib_package_template::ChildTemplatePlane plane_;
            ariitk::lib_package_template::ChildTemplateSphere sphere_;

            bool do_plane_;

            int width_;
            int height_;
    };
}