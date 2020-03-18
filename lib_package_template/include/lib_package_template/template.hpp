#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

namespace ariitk::lib_package_template {
class BaseTemplate {
    public:
        BaseTemplate();
        ~BaseTemplate() {};
        virtual void generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) = 0;
        std::vector<int> getModelInliers() const { return inliers_; }
        virtual void fitModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double distance_threshold) = 0;
        void computeInliers(pcl::RandomSampleConsensus<pcl::PointXYZ>& model);

    private:
        std::vector<int> inliers_;
};

class ChildTemplatePlane : public BaseTemplate {
    public:
        void generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) override;
        void fitModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double distance_threshold) override;
};

class ChildTemplateSphere : public BaseTemplate {
    public:
        void generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) override;
        void fitModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double distance_threshold) override;
};

}  // namespace ariitk::lib_package_template