#include "test_helper.hpp"

#include "../processing.hpp"
#include "../sample.hpp"
#include "../util.hpp"

#include "yaml.h"
#include "gtest/gtest.h"

#include <memory>
#include <numeric>

#include <Eigen/Core>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

TEST(IoTest, ReadPointCloudPLY)
{
  using namespace pcl_wrapper;

  auto cloud1 = IO::Read< Point3D >("/home/pcl_wrapper/dev/var/test_data/bun_zipper.pcd");
  EXPECT_NE(cloud1, nullptr);

  auto size1 = cloud1->width * cloud1->height;
  EXPECT_NE(size1, 0);

  std::cerr << "point cloud 1" << size1 << std::endl;
  cloud1->points.resize(cloud1->width * cloud1->height);

  std::cout << "Print out part of the point cloud." << std::endl;
  for (size_t i = 0; i < 1000; ++i) {
    float x1 = cloud1->points[i].x;
    float y1 = cloud1->points[i].y;
    float z1 = cloud1->points[i].z;
    std::cout << i << ": " << x1 << ", " << y1 << ", " << z1 << std::endl;
  }
}

TEST(VisulizationTest, SimplePointCloudViewer_StanfordBunny)
{
  using namespace pcl_wrapper;
  auto cloud = IO::Read< Point3D >("/home/pcl_wrapper/dev/var/test_data/bun_zipper.pcd");
  auto size  = cloud->width * cloud->height;

  cloud->points.resize(size);

  for (size_t i = 0; i < size; ++i) {
    if (not std::isnan(cloud->points[i].z)) {
      cloud->points[i].x *= 10.0f;
      cloud->points[i].y *= 10.0f;
      cloud->points[i].z *= 10.0f;
    }
  }
}

TEST(VisulizationTest, SimplePointCloudViewer_Basler)
{
  using namespace pcl_wrapper;
  auto cloud = IO::Read< Point6D >("/home/pcl_wrapper/dev/var/test_data/bas21.pcd");
  auto size  = cloud->width * cloud->height;

  cloud->points.resize(size);

  for (size_t i = 0; i < size; ++i) {
    if (not std::isnan(cloud->points[i].z)) {
      cloud->points[i].x /= 1000.0f;
      cloud->points[i].y /= 1000.0f;
      cloud->points[i].z /= 1000.0f;
      std::cout << int(cloud->points[i].r) << ", "
                << int(cloud->points[i].g) << ", "
                << int(cloud->points[i].b) << std::endl;
    }
  }
}

TEST(UtilTest, ClipPointCloud_StanfordBunny)
{
  using namespace pcl_wrapper;
  auto cloud = IO::Read< Point3D >("/home/pcl_wrapper/dev/var/test_data/bun_zipper.pcd");
  auto size  = cloud->width * cloud->height;

  cloud->points.resize(size);

  for (size_t i = 0; i < size; ++i) {
    if (not std::isnan(cloud->points[i].z)) {
      cloud->points[i].x *= 10.0f;
      cloud->points[i].y *= 10.0f;
      cloud->points[i].z *= 10.0f;
    }
  }

  auto clipped_cloud1 = Tool::ClipCloud< Point3D, PointDimension::X >(cloud, -0.3, 0.3);
  auto clipped_cloud2 = Tool::ClipCloud< Point3D, PointDimension::Y >(clipped_cloud1, -0.3, 1.0);
  auto clipped_cloud3 = Tool::ClipCloud< Point3D, PointDimension::Z >(clipped_cloud2, -0.3, 0.3);
}

TEST(UtilTest, Flatten)
{
  using namespace pcl_wrapper;

  using T1 = std::vector< int >;
  using T2 = std::vector< T1 >;

  auto list
      = T2{
          T1{1, 2, 3},
          T1{4, 5, 6}};

  auto flattened = Flatten(list);

  for (auto& i : flattened) {
    std::cerr << i << " ";
  }
  std::cerr << std::endl;
}

TEST(UtilTest, SamplingCloud_VoxelDownSampler)
{
  using namespace pcl_wrapper;
  using namespace pcl_wrapper_tests;

  auto cloud = read_cloud< Point6D >();

  float leaf_size = 0.01f;

  auto sampler = Sampler< Point6D, SamplerType::DownSampling_VoxelGrid >{cloud};
  auto out     = sampler
                 .SetParameters(decltype(sampler)::Parameters(leaf_size))
                 .Compute();

  std::cerr << "down sampled point cloud to size : " << out->size() << std::endl;

  const Eigen::Vector3f    trans{2.0, 0.0, 0.0};
  const Eigen::Quaternionf no_rot{0.0, 0.0, 0.0, 0.0};

  auto left_points  = pcl::PointCloud< Point6D >::Ptr{new pcl::PointCloud< Point6D >};
  auto right_points = pcl::PointCloud< Point6D >::Ptr{new pcl::PointCloud< Point6D >};

  pcl::transformPointCloud(*cloud, *left_points, -trans, no_rot);
  pcl::transformPointCloud(*out, *right_points, trans, no_rot);

  pcl::visualization::PCLVisualizer v;

  v.setBackgroundColor(0.8, 0.8, 0.8);
  v.addPointCloud(left_points, "left");
  v.addPointCloud(right_points, "right");

  v.spin();
}

TEST(UtilTest, YamlParameterParsing)
{
  YAML::Node config = YAML::LoadFile("../config/keypoint/sift3d.yaml");
  const auto param1 = config[std::string{"min_scale"}].as< float >();
  const auto param2 = config["nr_octaves"].as< int >();
  const auto param3 = config["nr_scales_per_octave"].as< int >();
  const auto param4 = config["min_contrast"].as< float >();

  std::cerr << param1 << ", " << param2 << ", " << param3 << ", " << param4 << std::endl;
}
