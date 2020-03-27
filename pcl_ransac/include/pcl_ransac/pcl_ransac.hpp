#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <Eigen/Core>

#include <pcl_ransac_msgs/ModelCoefficients.h>

namespace ariitk::pcl_ransac {

class PointCloudModel {
    public:
        PointCloudModel();
        ~PointCloudModel() {};
        virtual void generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) = 0;
        inline std::vector<int> getModelInliers() const { return inliers_; };
        virtual void fitModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const double& distance_threshold) = 0;
        void computeModelParameters(pcl::RandomSampleConsensus<pcl::PointXYZ>& model);
        inline pcl_ransac_msgs::ModelCoefficients getModelCoefficients() const { return coefficients_; };

    private:
        pcl_ransac_msgs::ModelCoefficients coefficients_;
        std::vector<int> inliers_;
};

class CloudModelPlane : public PointCloudModel {
    public:
        void generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) override;
        void fitModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const double& distance_threshold) override;
};

class CloudModelSphere : public PointCloudModel {
    public:
        void generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) override;
        void fitModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const double& distance_threshold) override;
};

}  // namespace ariitk::pcl_ransac