#include <pcl_ransac/pcl_ransac.hpp>

namespace ariitk::pcl_ransac {

PointCloudModel::PointCloudModel()
    : inliers_(std::vector<int>()) {}

void PointCloudModel::computeModelParameters(pcl::RandomSampleConsensus<pcl::PointXYZ>& model) {
    model.getInliers(inliers_);

    Eigen::VectorXf model_coeff;
    model.getModelCoefficients(model_coeff);

    coefficients_.a = model_coeff.data()[0];
    coefficients_.b = model_coeff.data()[1];
    coefficients_.c = model_coeff.data()[2];
    coefficients_.d = model_coeff.data()[3];
}

void CloudModelPlane::generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
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

void CloudModelPlane::fitModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const double& distance_threshold) {
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
    ransac.setDistanceThreshold(distance_threshold);
    ransac.computeModel();
    computeModelParameters(ransac);
    pcl::copyPointCloud(*cloud, getModelInliers(), *cloud);
}

void CloudModelSphere::generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
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

void CloudModelSphere::fitModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const double& distance_threshold) {
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
    ransac.setDistanceThreshold(distance_threshold);
    ransac.computeModel();
    computeModelParameters(ransac);
    pcl::copyPointCloud(*cloud, getModelInliers(), *cloud);
}

}  // namespace ariitk::pcl_ransac