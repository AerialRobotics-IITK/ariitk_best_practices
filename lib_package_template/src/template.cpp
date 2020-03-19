#include <lib_package_template/template.hpp>

namespace ariitk::lib_package_template {

BaseTemplate::BaseTemplate()
    : inliers_(std::vector<int>()) {}

void BaseTemplate::computeInliers(pcl::RandomSampleConsensus<pcl::PointXYZ>& model) {
    model.getInliers(inliers_);
}

void ChildTemplatePlane::generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    cloud->points.resize(cloud->width * cloud->height);
    for(std::size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
        if(i % 2 == 0) {
            cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
        } else {
            cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
        }
    }
}

void ChildTemplatePlane::fitModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double distance_threshold) {
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
    ransac.setDistanceThreshold(distance_threshold);
    ransac.computeModel();
    computeInliers(ransac);
    pcl::copyPointCloud(*cloud, getModelInliers(), *cloud);
}

void ChildTemplateSphere::generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    cloud->points.resize(cloud->width * cloud->height);
    for(std::size_t i = 0; i < cloud->points.size (); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
        if(i % 5 == 0) {
            cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
        } else if(i % 2 == 0) {
            cloud->points[i].z = sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
                                      - (cloud->points[i].y * cloud->points[i].y));
        } else {
            cloud->points[i].z = -sqrt(1 - (cloud->points[i].x * cloud->points[i].x)
                                        - (cloud->points[i].y * cloud->points[i].y));
        }
    }
}

void ChildTemplateSphere::fitModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double distance_threshold) {
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
    ransac.setDistanceThreshold(distance_threshold);
    ransac.computeModel();
    computeInliers(ransac);
    pcl::copyPointCloud(*cloud, getModelInliers(), *cloud);
}

}  // namespace ariitk::lib_package_template