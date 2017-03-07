#include "../processing.hpp"
#include "../sample.hpp"
#include "../segmentation.hpp"
#include "../types.hpp"
#include "../util.hpp"

#include "gtest/gtest.h"

#include "test_helper.hpp"

#include <random>

#include <pcl/features/feature.h>
#include <pcl/features/pfh.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/uniform_sampling.h>

// For plane detection
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>

TEST(ProcessingTest, NormalEstimation_BaslerPointCloud)
{
  using namespace pcl_wrapper;
  auto cloud = pcl_wrapper_tests::read_cloud< Point6D >();

  auto normal_estimator = Processing< Point6D, pcl::Normal, ProcessingType::NormalEstimation >{cloud};
  auto normals          = normal_estimator
                     .SetParameters(decltype(normal_estimator)::Parameters{})
                     .Compute();
}

TEST(ProcessingTest, UniformSampling)
{
  using namespace pcl_wrapper;
  using namespace pcl_wrapper_tests;

  auto cloud = pcl_wrapper::IO::Read< Point3D >("/home/pcl_wrapper/dev/var/test_data/bas21.pcd");

  cloud->points.resize(cloud->width * cloud->height);
  auto size = cloud->width * cloud->height;
  for (size_t i = 0; i < size; ++i) {
    if (not std::isnan(cloud->points[i].z)) {
      cloud->points[i].x /= 1000.0f;
      cloud->points[i].y /= 1000.0f;
      cloud->points[i].z /= 1000.0f;
    }
  }

  auto sampler = UniformSampler< Point3D >{cloud};
  auto indices = sampler
                     .SetSearchRadius(0.01f)
                     .Compute();

  std::cout << "Sample size: " << indices.size() << std::endl;
}

TEST(ProcessingTest, StatisticalOutlierRemoval)
{
  using namespace pcl_wrapper;
  auto cloud = pcl_wrapper_tests::read_cloud< Point6D >();

  auto cloudx = Tool::ClipCloud< Point6D, PointDimension::X >(cloud, -0.7, 0.2);
  auto cloudy = Tool::ClipCloud< Point6D, PointDimension::Y >(cloudx, -1.0, -0.2);
  auto cloudz = Tool::ClipCloud< Point6D, PointDimension::Z >(cloudy, 0.0, 5.0);

  cloud = cloudz;

  auto f        = Filter< Point6D, FilterType::StatisticalOutlierRemoval >{cloud};
  auto filtered = f.SetParameters(decltype(f)::Parameters{}).Compute();

  cloud = filtered;

  std::vector< PointCloudPtr< Point6D > > all_segments;

  pcl_wrapper_tests::show_all_segments< Point6D >("StatisticalOutlierRemoval", cloud, all_segments);
}

TEST(ProcessingTest, RadiusOutlierRemoval)
{
  using namespace pcl_wrapper;
  auto cloud = pcl_wrapper_tests::read_cloud< Point6D >();

  auto cloudx = Tool::ClipCloud< Point6D, PointDimension::X >(cloud, -0.7, 0.2);
  auto cloudy = Tool::ClipCloud< Point6D, PointDimension::Y >(cloudx, -1.0, -0.2);
  auto cloudz = Tool::ClipCloud< Point6D, PointDimension::Z >(cloudy, 0.0, 5.0);

  cloud = cloudz;

  auto f        = Filter< Point6D, FilterType::RadiusOutlierRemoval >{cloud};
  auto filtered = f.SetParameters(decltype(f)::Parameters{}).Compute();

  cloud = filtered;

  std::vector< PointCloudPtr< Point6D > > all_segments;

  pcl_wrapper_tests::show_all_segments< Point6D >("RadiusOutlierRemoval", cloud, all_segments);
}

TEST(ProcessingTest, MovingLeastSquares)
{
  using namespace pcl_wrapper;
  auto cloud = pcl_wrapper_tests::read_cloud< Point6D >();

  auto cloudx = Tool::ClipCloud< Point6D, PointDimension::X >(cloud, -0.7, 0.2);
  auto cloudy = Tool::ClipCloud< Point6D, PointDimension::Y >(cloudx, -1.0, -0.2);
  auto cloudz = Tool::ClipCloud< Point6D, PointDimension::Z >(cloudy, 0.0, 5.0);

  cloud = cloudz;

  auto s        = Processing< Point6D, Point6D, ProcessingType::MovingLeastSquares >{cloud};
  auto smoothed = s.SetParameters(decltype(s)::Parameters{}).Compute();

  cloud = smoothed;

  std::vector< PointCloudPtr< Point6D > > all_segments;

  pcl_wrapper_tests::show_all_segments< Point6D >("MovingLeastSquares", cloud, all_segments);
}

TEST(ProcessingTest, DownSampling_VoxelGrid)
{
}

TEST(ProcessingTest, UpSampling)
{
  using namespace pcl_wrapper;
  auto cloud = pcl_wrapper_tests::read_cloud< Point6D >();

  auto cloudx = Tool::ClipCloud< Point6D, PointDimension::X >(cloud, -0.7, 0.2);
  auto cloudy = Tool::ClipCloud< Point6D, PointDimension::Y >(cloudx, -1.0, -0.2);
  auto cloudz = Tool::ClipCloud< Point6D, PointDimension::Z >(cloudy, 0.0, 5.0);

  cloud = cloudz;

  std::vector< PointCloudPtr< Point6D > > all_segments;

  pcl_wrapper_tests::show_all_segments< Point6D >("ProjectionOntoPlane", cloud, all_segments);
}

TEST(ProcessingTest, SIFT3D)
{
  using namespace pcl_wrapper;
  auto cloud = pcl_wrapper::IO::Read< Point6D >("/home/pcl_wrapper/dev/var/test_data/bas21.pcd");

  cloud->points.resize(cloud->width * cloud->height);
  auto size = cloud->width * cloud->height;
  for (size_t i = 0; i < size; ++i) {
    if (not std::isnan(cloud->points[i].z)) {
      cloud->points[i].x /= 1000.0f;
      cloud->points[i].y /= 1000.0f;
      cloud->points[i].z /= 1000.0f;
    }
  }

  using namespace pcl_wrapper;
  auto detector  = Keypoint< Point6D, pcl::PointWithScale, KeypointType::Sift3d >{cloud};
  auto keypoints = detector
                       .SetParameters(decltype(detector)::Parameters{})
                       .Compute();

  std::cerr << "cloud size        : " << cloud->points.size() << std::endl;
  std::cerr << "keypoints size    : " << keypoints->points.size() << std::endl;
}

TEST(ProcessingTest, Visualize_SIFT3D)
{
  using namespace pcl_wrapper;
  auto cloud = pcl_wrapper_tests::read_cloud< Point6D >();

  auto sampler       = Sampler< Point6D, SamplerType::DownSampling_VoxelGrid >{cloud};
  auto sampled_cloud = sampler
                           .SetParameters(decltype(sampler)::Parameters{0.01f})
                           .Compute();

  auto detector  = Keypoint< Point6D, pcl::PointWithScale, KeypointType::Sift3d >{sampled_cloud};
  auto keypoints = detector
                       .SetParameters(decltype(detector)::Parameters{0.01f, 10, 10, 0.0001f})
                       .Compute();

  std::cerr << "cloud size                : " << cloud->points.size() << std::endl;
  std::cerr << "sampled cloud size        : " << sampled_cloud->points.size() << std::endl;
  std::cerr << "keypoints size            : " << keypoints->points.size() << std::endl;

  std::string window_name{"KeypointVisualization_SIFT3D"};
  std::shared_ptr< pcl::visualization::PCLVisualizer >
      viewer{new pcl::visualization::PCLVisualizer{window_name.c_str()}};

  std::string cloud_name{"cloud"};
  std::string keypoints_name{"keypoints"};
  viewer->setBackgroundColor(0.8, 0.8, 0.8);
  pcl::visualization::PointCloudColorHandlerRGBField< pcl::PointXYZRGB > rgb(sampled_cloud);
  viewer->addPointCloud< Point6D >(sampled_cloud, rgb, cloud_name.c_str());

  for (size_t i = 0; i < keypoints->size(); ++i) {
    const auto& p = keypoints->points[i];
    auto        r = p.scale / 10.0f;

    std::stringstream ss("keypoint");
    ss << i;

    viewer->addSphere(p, r, 1.0, 0.0, 0.0, ss.str());
  }

  viewer->spin();
  viewer->close();
}

// Signature of Histograms of OrienTations
TEST(ProcessingTest, FeatureDetection_PFH)
{
  using namespace pcl_wrapper;
  auto cloud = pcl_wrapper_tests::read_cloud< Point6D >();

  auto sampler       = Sampler< Point6D, SamplerType::DownSampling_VoxelGrid >{cloud};
  auto sampled_cloud = sampler
                           .SetParameters(decltype(sampler)::Parameters{0.02f})
                           .Compute();

  auto detector  = Keypoint< Point6D, pcl::PointWithScale, KeypointType::Sift3d >{sampled_cloud};
  auto keypoints = detector
                       .SetParameters(decltype(detector)::Parameters{})
                       .Compute();

  auto normal_estimator = Processing< Point6D, pcl::Normal, ProcessingType::NormalEstimation >{sampled_cloud};
  auto normals          = normal_estimator
                     .SetParameters(decltype(normal_estimator)::Parameters{})
                     .Compute();

  auto keypoint_cloud = Tool::Convert< pcl::PointWithScale, Point6D >(keypoints);

  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  auto feature     = FeatureFromNormals< Point6D, pcl::PFHSignature125 >{sampled_cloud, normals};
  auto descriptors = feature.SetEstimationSpace(keypoint_cloud)
                         .SetSearchRadius(0.08f)
                         .Compute();

  std::cerr << "point cloud size      : " << cloud->size() << std::endl;
  std::cerr << "sampled cloud size    : " << sampled_cloud->size() << std::endl;
  std::cerr << "normals size          : " << normals->size() << std::endl;
  std::cerr << "keypoint size         : " << keypoints->size() << std::endl;
  std::cerr << "feature size          : " << descriptors->size() << std::endl;
}

// Point Feature Histogram
TEST(ProcessingTest, FeatureDetection_SHOT)
{
}

// Point Pair Feature
TEST(ProcessingTest, FeatureDetection_PPF)
{
}

TEST(ProcessingTest, PlaneSegmentation_SAC)
{
  using namespace pcl_wrapper_tests;
  using namespace pcl_wrapper;

  int debug_info_index = 0;

  using namespace pcl_wrapper;
  auto cloud = pcl_wrapper_tests::read_cloud< Point6D >();

  auto cloudx = Tool::ClipCloud< Point6D, PointDimension::X >(cloud, -0.7, 0.2);
  auto cloudy = Tool::ClipCloud< Point6D, PointDimension::Y >(cloudx, -1.0, -0.2);
  auto cloudz = Tool::ClipCloud< Point6D, PointDimension::Z >(cloudy, 0.0, 5.0);

  cloud = cloudz;

  // ---------------------------------------------------------------------------------------------
  // Smoothing using MLS
  // ---------------------------------------------------------------------------------------------

  //    pcl::PointCloud< Point6D >::Ptr smoothed{ new pcl::PointCloud< Point6D > };
  //    pcl::MovingLeastSquares< Point6D, Point6D > mls;

  //    pcl::search::KdTree< Point6D >::Ptr tree(new pcl::search::KdTree< Point6D >);
  //    mls.setComputeNormals(true);

  // Set parameters
  //    mls.setInputCloud(cloud);
  //    mls.setPolynomialFit(true);
  //    mls.setSearchMethod(tree);
  //    mls.setSearchRadius(0.03f);
  //    mls.process(*smoothed);

  auto smoother = Processing< Point6D, Point6D, ProcessingType::MovingLeastSquares >{cloud};
  auto smoothed = smoother
                      .SetParameters(decltype(smoother)::Parameters{})
                      .Compute();

  std::cerr << "point cloud size              : " << cloud->size() << std::endl;
  cloud = smoothed;

  auto sampler       = Sampler< Point6D, SamplerType::DownSampling_VoxelGrid >{cloud};
  auto sampled_cloud = sampler.SetParameters(decltype(sampler)::Parameters{}).Compute();

  auto f1        = Filter< Point6D, FilterType::RadiusOutlierRemoval >{sampled_cloud};
  auto filtered1 = f1.SetParameters(decltype(f1)::Parameters{}).Compute();

  auto f2       = Filter< Point6D, FilterType::StatisticalOutlierRemoval >{filtered1};
  auto filtered = f2.SetParameters(decltype(f2)::Parameters{}).Compute();

  std::cerr << "smoothed cloud size           : " << cloud->size() << std::endl;
  std::cerr << "sampled cloud size            : " << sampled_cloud->size() << std::endl;
  std::cerr << "filtered1 cloud size          : " << filtered1->size() << std::endl;
  std::cerr << "filtered2 cloud size          : " << filtered->size() << std::endl;

  auto seg_cloud = filtered;

  std::vector< PointCloudPtr< Point6D > >  all_planes;
  size_t                                   segments_size;
  std::vector< pcl::ModelCoefficientsPtr > coefs;

  // ---------------------------------------------------------------------------------------------
  // Do SAC plane estimation
  // ---------------------------------------------------------------------------------------------
  do {
    auto s = Segmentation< Point6D, SegmentationType::SacModelPlaneFromNormals >{seg_cloud};

    // Compute normals for current step of segmentation
    // Do segmentation using current cloud
    auto normal_estimator = Processing< Point6D, pcl::Normal, ProcessingType::NormalEstimation >{seg_cloud};
    auto normals          = normal_estimator
                       .SetParameters(decltype(normal_estimator)::Parameters{})
                       .Compute();

    auto segments_indices = s
                                .SetParameters(decltype(s)::Parameters{})
                                .SetNormals(normals)
                                .Segment();

    segments_size = segments_indices.size();

    // Extract segment cloud using each segment indices (only 1 here)
    for (size_t i = 0; i < segments_indices.size(); ++i) {

      const auto& point_idxs = segments_indices[i];
      auto        extractor  = Extractor< Point6D >{seg_cloud, point_idxs};
      auto        segment    = extractor.SetExtractComplementaryCloud(false).Compute();
      auto        c          = s.GetModelCoefficients()[i];
      coefs.push_back(c);

      if (segment->size() >= MIN_SEGMENT_SIZE) {

        all_planes.push_back(segment);
      }

      // Extract the remaining cloud using all the indices
      seg_cloud = extractor.SetExtractComplementaryCloud(true).Compute();
    }

    // Extract plane coefficients from each plane (only 1 here)

    std::stringstream ss;
    ss << "point cloud size: " << seg_cloud->size();

    debug_info(ss.str(), debug_info_index++, 1);

  } while (seg_cloud->size() >= MIN_NUM_POINTS_REQUIRED);

  pcl_wrapper_tests::show_all_segments< Point6D >("PlaneSegmentation_SAC", cloud, all_planes);
}

TEST(ProcessingTest, EuclideanCluster)
{
  int debug_info_index = 0;

  using namespace pcl_wrapper_tests;
  using namespace pcl_wrapper;
  auto cloud = pcl_wrapper_tests::read_cloud< Point6D >();

  auto cloudx = Tool::ClipCloud< Point6D, PointDimension::X >(cloud, -0.7, 0.2);
  auto cloudy = Tool::ClipCloud< Point6D, PointDimension::Y >(cloudx, -1.0, -0.2);
  auto cloudz = Tool::ClipCloud< Point6D, PointDimension::Z >(cloudy, 0.0, 5.0);

  cloud = cloudz;

  auto smoother = Processing< Point6D, Point6D, ProcessingType::MovingLeastSquares >{cloud};
  auto smoothed = smoother
                      .SetParameters(decltype(smoother)::Parameters{})
                      .Compute();

  std::cerr << "point cloud size              : " << cloud->size() << std::endl;
  cloud = smoothed;

  auto sampler       = Sampler< Point6D, SamplerType::DownSampling_VoxelGrid >{cloud};
  auto sampled_cloud = sampler.SetParameters(decltype(sampler)::Parameters{0.005f}).Compute();

  auto f1        = Filter< Point6D, FilterType::RadiusOutlierRemoval >{sampled_cloud};
  auto filtered1 = f1.SetParameters(decltype(f1)::Parameters{}).Compute();

  auto f2       = Filter< Point6D, FilterType::StatisticalOutlierRemoval >{filtered1};
  auto filtered = f2.SetParameters(decltype(f2)::Parameters{}).Compute();

  std::vector< KeypointsPtr< pcl::PointWithScale > > all_keypoints;

  debug_info(std::string("smoothed cloud size     : ") + std::to_string(smoothed->size()), debug_info_index++);
  debug_info(std::string("sampled cloud size      : ") + std::to_string(sampled_cloud->size()), debug_info_index++);
  debug_info(std::string("filtered1 cloud size    : ") + std::to_string(filtered1->size()), debug_info_index++);
  debug_info(std::string("filtered2 cloud size    : ") + std::to_string(filtered->size()), debug_info_index++);

  auto seg_cloud = filtered;

  std::vector< PointCloudPtr< Point6D > > all_planes;
  size_t                                  segments_size;

  // ---------------------------------------------------------------------------------------------
  // Do SAC plane estimation
  // ---------------------------------------------------------------------------------------------

  std::vector< pcl::ModelCoefficients > coeffs;
  debug_info("begin SAC plane estimation", debug_info_index++);
  do {
    auto s = Segmentation< Point6D, SegmentationType::SacModelPlaneFromNormals >{seg_cloud};

    // Compute normals for current step of segmentation
    // Do segmentation using current cloud
    auto normal_estimator = Processing< Point6D, pcl::Normal, ProcessingType::NormalEstimation >{seg_cloud};
    auto normals          = normal_estimator
                       .SetParameters(decltype(normal_estimator)::Parameters{})
                       .Compute();

    auto segments_indices = s
                                .SetParameters(decltype(s)::Parameters{})
                                .SetNormals(normals)
                                .Segment();

    segments_size = segments_indices.size();

    // Extract segment cloud using each segment indices (only 1 here)
    for (auto& point_idxs : segments_indices) {
      auto extractor = Extractor< Point6D >{seg_cloud, point_idxs};
      auto segment   = extractor.SetExtractComplementaryCloud(false).Compute();

      // Reject segment thats is too small
      if (segment->size() >= MIN_SEGMENT_SIZE) {
        all_planes.push_back(segment);
      }

      // Extract the remaining cloud using all the indices
      seg_cloud = extractor.SetExtractComplementaryCloud(true).Compute();
    }

    std::stringstream ss;
    ss << "point cloud size: " << seg_cloud->size();

    debug_info(ss.str(), debug_info_index++, 1);

  } while (seg_cloud->size() >= MIN_NUM_POINTS_REQUIRED);

  debug_info("done SAC plane estimation", debug_info_index++);

  std::vector< PointCloudPtr< Point6D > > all_segments;

  debug_info("begin clustering at each plane", debug_info_index++);
  for (size_t i; i < all_planes.size(); ++i) {
    const auto& p = all_planes[i];

    auto cluster          = Segmentation< Point6D, SegmentationType::EuclideanClusterExtraction >{p};
    auto clusters_indices = cluster.SetParameters(decltype(cluster)::Parameters{}).Segment();

    for (auto& indices : clusters_indices) {
      auto extractor = Extractor< Point6D >{p, indices};
      auto segment   = extractor.SetExtractComplementaryCloud(false).Compute();
      all_segments.push_back(segment);
    }

    std::stringstream ss;
    ss << "detected " << clusters_indices.size() << " in plane";
    debug_info(ss.str(), debug_info_index++, 1);
  }
  debug_info("done clustering", debug_info_index++);

  // ---------------------------------------------------------------------------------------------
  // Visualization part
  // ---------------------------------------------------------------------------------------------
  pcl_wrapper_tests::show_all_segments< Point6D >("EuclideanCluster", cloud, all_segments);
}

TEST(ProcessingTest, ConvexHull)
{
  int debug_info_index = 0;

  using namespace pcl_wrapper_tests;
  using namespace pcl_wrapper;
  auto cloud = pcl_wrapper_tests::read_cloud< Point6D >();

  auto cloudx = Tool::ClipCloud< Point6D, PointDimension::X >(cloud, -0.7, 0.2);
  auto cloudy = Tool::ClipCloud< Point6D, PointDimension::Y >(cloudx, -1.0, -0.2);
  auto cloudz = Tool::ClipCloud< Point6D, PointDimension::Z >(cloudy, 0.0, 5.0);

  cloud = cloudz;

  auto sampler       = Sampler< Point6D, SamplerType::DownSampling_VoxelGrid >{cloud};
  auto sampled_cloud = sampler.SetParameters(decltype(sampler)::Parameters{0.005f}).Compute();

  auto f1        = Filter< Point6D, FilterType::RadiusOutlierRemoval >{sampled_cloud};
  auto filtered1 = f1.SetParameters(decltype(f1)::Parameters{}).Compute();

  auto f2       = Filter< Point6D, FilterType::StatisticalOutlierRemoval >{filtered1};
  auto filtered = f2.SetParameters(decltype(f2)::Parameters{}).Compute();

  std::vector< KeypointsPtr< pcl::PointWithScale > > all_keypoints;

  debug_info(std::string("point cloud siz         : ") + std::to_string(sampled_cloud->size()), debug_info_index++);
  debug_info(std::string("sampled cloud size      : ") + std::to_string(sampled_cloud->size()), debug_info_index++);
  debug_info(std::string("filtered1 cloud size    : ") + std::to_string(filtered1->size()), debug_info_index++);
  debug_info(std::string("filtered2 cloud size    : ") + std::to_string(filtered->size()), debug_info_index++);

  auto seg_cloud = filtered;

  std::vector< PointCloudPtr< Point6D > > all_planes;
  size_t                                  segments_size;

  // ---------------------------------------------------------------------------------------------
  // Do SAC plane estimation
  // ---------------------------------------------------------------------------------------------

  debug_info("begin SAC plane estimation", debug_info_index++);
  do {
    auto s = Segmentation< Point6D, SegmentationType::SacModelPlaneFromNormals >{seg_cloud};

    // Compute normals for current step of segmentation
    // Do segmentation using current cloud
    auto normal_estimator = Processing< Point6D, pcl::Normal, ProcessingType::NormalEstimation >{seg_cloud};

    auto normals = normal_estimator
                       .SetParameters(decltype(normal_estimator)::Parameters{})
                       .Compute();

    auto segments_indices = s
                                .SetParameters(decltype(s)::Parameters{})
                                .SetNormals(normals)
                                .Segment();

    segments_size = segments_indices.size();

    // Extract segment cloud using each segment indices (only 1 here)
    for (auto& point_idxs : segments_indices) {
      auto extractor = Extractor< Point6D >{seg_cloud, point_idxs};
      auto segment   = extractor.SetExtractComplementaryCloud(false).Compute();

      // Reject segment thats is too small
      if (segment->size() >= MIN_SEGMENT_SIZE) {
        all_planes.push_back(segment);
      }

      // Extract the remaining cloud using all the indices
      seg_cloud = extractor.SetExtractComplementaryCloud(true).Compute();
    }

    std::stringstream ss;
    ss << "point cloud size: " << seg_cloud->size();

    debug_info(ss.str(), debug_info_index++, 1);
  } while (seg_cloud->size() >= MIN_NUM_POINTS_REQUIRED);

  debug_info("done SAC plane estimation", debug_info_index++);

  std::vector< PointCloudPtr< Point6D > > all_segments;

  debug_info("begin clustering at each plane", debug_info_index++);
  for (auto& p : all_planes) {
    auto cluster          = Segmentation< Point6D, SegmentationType::EuclideanClusterExtraction >{p};
    auto clusters_indices = cluster.SetParameters(decltype(cluster)::Parameters{}).Segment();

    for (auto& indices : clusters_indices) {
      auto extractor = Extractor< Point6D >{p, indices};
      auto segment   = extractor.SetExtractComplementaryCloud(false).Compute();
      all_segments.push_back(segment);
    }

    std::stringstream ss;
    ss << "detected " << clusters_indices.size() << " in plane";
    debug_info(ss.str(), debug_info_index++, 1);
  }
  debug_info("done clustering", debug_info_index++);

  std::vector< PointCloudPtr< Point6D > > all_hulls;

  for (auto& segment : all_segments) {
    PointCloudPtr< Point6D >    cloud_hull(new PointCloud< Point6D >);
    pcl::ConcaveHull< Point6D > c_hull;
    c_hull.setAlpha(0.1);
    //        pcl::ConvexHull< Point6D > c_hull;
    c_hull.setInputCloud(segment);
    c_hull.reconstruct(*cloud_hull);
    all_hulls.push_back(cloud_hull);
  }

  // ---------------------------------------------------------------------------------------------
  // Visualization part
  // ---------------------------------------------------------------------------------------------
  pcl_wrapper_tests::show_all_segments< Point6D >("Convex Hull", cloud, all_hulls);
}

TEST(ProcessingTest, ProjectionOntoPlane)
{
  int debug_info_index = 0;

  using namespace pcl_wrapper_tests;
  using namespace pcl_wrapper;
  auto cloud = pcl_wrapper_tests::read_cloud< Point6D >();

  auto cloudx = Tool::ClipCloud< Point6D, PointDimension::X >(cloud, -0.7, 0.2);
  auto cloudy = Tool::ClipCloud< Point6D, PointDimension::Y >(cloudx, -1.0, -0.2);
  auto cloudz = Tool::ClipCloud< Point6D, PointDimension::Z >(cloudy, 0.0, 5.0);

  cloud = cloudz;

  auto smoother = Processing< Point6D, Point6D, ProcessingType::MovingLeastSquares >{cloud};
  auto smoothed = smoother
                      .SetParameters(decltype(smoother)::Parameters{})
                      .Compute();

  debug_info(std::string("point cloud size              : ") + std::to_string(cloud->size()), debug_info_index++);
  cloud = smoothed;

  auto sampler       = Sampler< Point6D, SamplerType::DownSampling_VoxelGrid >{cloud};
  auto sampled_cloud = sampler.SetParameters(decltype(sampler)::Parameters{0.005f}).Compute();

  auto f1        = Filter< Point6D, FilterType::RadiusOutlierRemoval >{sampled_cloud};
  auto filtered1 = f1.SetParameters(decltype(f1)::Parameters{}).Compute();

  auto f2       = Filter< Point6D, FilterType::StatisticalOutlierRemoval >{filtered1};
  auto filtered = f2.SetParameters(decltype(f2)::Parameters{}).Compute();

  std::vector< KeypointsPtr< pcl::PointWithScale > > all_keypoints;

  debug_info(std::string("point cloud siz         : ") + std::to_string(sampled_cloud->size()), debug_info_index++);
  debug_info(std::string("sampled cloud size      : ") + std::to_string(sampled_cloud->size()), debug_info_index++);
  debug_info(std::string("filtered1 cloud size    : ") + std::to_string(filtered1->size()), debug_info_index++);
  debug_info(std::string("filtered2 cloud size    : ") + std::to_string(filtered->size()), debug_info_index++);

  auto seg_cloud = filtered;

  std::vector< PointCloudPtr< Point6D > > all_planes;
  size_t                                  segments_size;

  // ---------------------------------------------------------------------------------------------
  // Do SAC plane estimation
  // ---------------------------------------------------------------------------------------------

  std::vector< pcl::ModelCoefficientsPtr > coefs;
  debug_info("begin SAC plane estimation", debug_info_index++);
  do {

    // Compute normals for current step of segmentation
    // Do segmentation using current cloud
    auto normal_estimator = Processing< Point6D, pcl::Normal, ProcessingType::NormalEstimation >{seg_cloud};
    auto normals          = normal_estimator
                       .SetParameters(decltype(normal_estimator)::Parameters{})
                       .Compute();

    auto s                = Segmentation< Point6D, SegmentationType::SacModelPlaneFromNormals >{seg_cloud};
    auto segments_indices = s
                                .SetParameters(decltype(s)::Parameters{})
                                .SetNormals(normals)
                                .Segment();

    segments_size = segments_indices.size();

    // Extract segment cloud using each segment indices (only 1 here)
    for (size_t i = 0; i < segments_indices.size(); ++i) {

      const auto& point_idxs = segments_indices[i];
      auto        extractor  = Extractor< Point6D >{seg_cloud, point_idxs};
      auto        segment    = extractor.SetExtractComplementaryCloud(false).Compute();
      auto        c          = s.GetModelCoefficients()[i];
      coefs.push_back(c);

      // Reject segment thats is too small
      if (segment->size() >= MIN_SEGMENT_SIZE) {
        all_planes.push_back(segment);
      }

      // Extract the remaining cloud using all the indices
      seg_cloud = extractor.SetExtractComplementaryCloud(true).Compute();
    }

    // Extract plane coefficients from each plane (only 1 here)

    std::stringstream ss;
    ss << "point cloud size: " << seg_cloud->size();

    debug_info(ss.str(), debug_info_index++, 1);

  } while (seg_cloud->size() >= MIN_NUM_POINTS_REQUIRED);

  debug_info("done SAC plane estimation", debug_info_index++);

  std::vector< PointCloudPtr< Point6D > > all_segments;

  debug_info("begin clustering at each plane", debug_info_index++);
  for (size_t i; i < all_planes.size(); ++i) {
    const auto& p = all_planes[i];
    const auto& c = coefs[i];

    std::cerr << c->values[0] << ", "
              << c->values[1] << ", "
              << c->values[2] << ", "
              << c->values[3] << std::endl
              << std::endl;

    auto cluster          = Segmentation< Point6D, SegmentationType::EuclideanClusterExtraction >{p};
    auto clusters_indices = cluster.SetParameters(decltype(cluster)::Parameters{}).Segment();

    for (auto& indices : clusters_indices) {
      auto extractor = Extractor< Point6D >{p, indices};
      auto segment   = extractor.SetExtractComplementaryCloud(false).Compute();
      auto projector = Filter< Point6D, FilterType::ProjectUsingParametricModel >{segment};
      auto projected = projector
                           .SetParameters(decltype(projector)::Parameters{pcl::SACMODEL_PLANE, c})
                           .Compute();

      all_segments.push_back(projected);
    }

    std::stringstream ss;
    ss << "detected " << clusters_indices.size() << " in plane";
    debug_info(ss.str(), debug_info_index++, 1);
  }
  debug_info("done clustering", debug_info_index++);

  // ---------------------------------------------------------------------------------------------
  // Visualization part
  // ---------------------------------------------------------------------------------------------
  pcl_wrapper_tests::show_all_segments< Point6D >("ProjectionOntoPlane", cloud, all_segments);
}
