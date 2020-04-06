#include <gtest/gtest.h>
#include <pcl_ransac/pcl_ransac.hpp>

using namespace ariitk::pcl_ransac;

TEST(PCLRansac, plane_fit_test) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 10;
    cloud->height = 1;
    cloud->is_dense = false;

    CloudModelPlane plane;
    plane.generatePointCloud(cloud);
    plane.fitModel(cloud, 0.01);

    std::vector<int> test_result = plane.getModelInliers();
    ASSERT_EQ(test_result.size(), 5);
}

TEST(PCLRansac, sphere_fit_test) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 20;
    cloud->height = 1;
    cloud->is_dense = false;

    CloudModelSphere sphere;
    sphere.generatePointCloud(cloud);
    sphere.fitModel(cloud, 0.01);
    std::vector<int> test_result = sphere.getModelInliers();
    ASSERT_EQ(test_result.size(), 9);

}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
