#include "../tests/test_helper.hpp"

#include "../parameters.hpp"
#include "../processing.hpp"
#include "../util.hpp"
#include "../viewer.hpp"

#include <pcl/common/io.h>
#include <pcl/point_traits.h>
#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>

using namespace pcl_wrapper;
using namespace pcl_wrapper_tests;
using namespace std;

template < typename T >
void clip(PointCloudPtr< T >& cloud)
{
  auto cloudx = Tool::ClipCloud< T, PointDimension::X >(cloud, -0.7, 0.2);
  auto cloudy = Tool::ClipCloud< T, PointDimension::Y >(cloudx, -1.0, -0.2);
  auto cloudz = Tool::ClipCloud< T, PointDimension::Z >(cloudy, 0.0, 5.0);
  cloud       = cloudz;
}

template < typename T >
void processing_mls_smoothing(PointCloudPtr< T >& cloud)
{
  auto s = Processing< T, T, ProcessingType::MovingLeastSquares >{cloud};
  cloud  = s.SetParameters(typename decltype(s)::Parameters{}).Compute();
}

template < typename T >
void filter_radius_ourlier_removal(PointCloudPtr< T >& cloud)
{
  auto f = Filter< T, FilterType::RadiusOutlierRemoval >{cloud};
  cloud  = f.SetParameters(typename decltype(f)::Parameters{}).Compute();
}

template < typename T >
void filter_statistical_ourlier_removal(PointCloudPtr< T >& cloud)
{
  auto f = Filter< T, FilterType::StatisticalOutlierRemoval >{cloud};
  cloud  = f.SetParameters(typename decltype(f)::Parameters{}).Compute();
}

//template < typename T >
//vector< PointCloudPtr< T > > single_cloud_segmentation_helper(PointCloudPtr< T >& cloud)
//{
//  int debug_info_index = 0;

//  vector< PointCloudPtr< T > > planes;

//  do {
//    auto n       = Processing< T, pcl::Normal, ProcessingType::NormalEstimation >{cloud};
//    auto normals = n.SetParameters(typename decltype(n)::Parameters{}).Compute();

//    auto s                = Segmentation< T, SegmentationType::SacModelPlaneFromNormals >{cloud};
//    auto segments_indices = s.SetParameters(typename decltype(s)::Parameters{}).SetNormals(normals).Segment();

//    for (auto& point_idxs : segments_indices) {
//      auto e       = Extractor< T >{cloud, point_idxs};
//      auto segment = e.SetExtractComplementaryCloud(false).Compute();

//      if (segment->size() >= MIN_SEGMENT_SIZE)
//        planes.push_back(segment);

//      cloud = e.SetExtractComplementaryCloud(true).Compute();
//    }

//    std::stringstream ss;
//    ss << "remained cloud size: " << cloud->size();

//    debug_info(ss.str(), debug_info_index++, 1);

//  } while (cloud->size() >= MIN_NUM_POINTS_REQUIRED);

//  return planes;
//}

template < typename T >
std::pair< vector< PointCloudPtr< T > >,           // planes
           vector< pcl::ModelCoefficients::Ptr > > // corresponding coefficients
    single_cloud_segmentation_helper(PointCloudPtr< T >& cloud)
{
  int debug_info_index = 0;

  vector< PointCloudPtr< T > >          planes;
  vector< pcl::ModelCoefficients::Ptr > coefs;

  do {
    auto n       = Processing< T, pcl::Normal, ProcessingType::NormalEstimation >{cloud};
    auto normals = n.SetParameters(typename decltype(n)::Parameters{}).Compute();

    auto s                = Segmentation< T, SegmentationType::SacModelPlaneFromNormals >{cloud};
    auto segments_indices = s.SetParameters(typename decltype(s)::Parameters{}).SetNormals(normals).Segment();

    for (auto& coef : s.GetModelCoefficients())
      coefs.push_back(coef);

    for (auto& point_idxs : segments_indices) {
      auto e       = Extractor< T >{cloud, point_idxs};
      auto segment = e.SetExtractComplementaryCloud(false).Compute();

      if (segment->size() >= MIN_SEGMENT_SIZE)
        planes.push_back(segment);

      cloud = e.SetExtractComplementaryCloud(true).Compute();
    }

    std::stringstream ss;
    ss << "remained cloud size: " << cloud->size();

    debug_info(ss.str(), debug_info_index++, 1);

  } while (cloud->size() >= MIN_NUM_POINTS_REQUIRED);

  return make_pair(planes, coefs);
}

template < typename T >
void plane_segmentation(const vector< PointCloudPtr< T > >& clouds,
                        vector< PointCloudPtr< T > >&       segments)
{
  int debug_info_index = 0;

  debug_info("begin SAC plane estimation", debug_info_index++);

  for (auto& cloud : clouds) {
    PointCloudPtr< T > copied{new PointCloud< T >};
    pcl::copyPointCloud(*cloud, *copied);

    // detect planes in a cloud
    auto planes = single_cloud_segmentation_helper< T >(copied);

    // add planes to result
    for (auto& p : planes.first)
      segments.push_back(p);
  }
}

template < typename T >
void plane_segmentation(const vector< PointCloudPtr< T > >&    clouds,
                        vector< PointCloudPtr< T > >&          segments,
                        vector< pcl::ModelCoefficients::Ptr >& coefs)
{
  int debug_info_index = 0;

  debug_info("begin SAC plane estimation", debug_info_index++);

  for (auto& cloud : clouds) {
    PointCloudPtr< T > copied{new PointCloud< T >};
    pcl::copyPointCloud(*cloud, *copied);

    auto plane_tuples = single_cloud_segmentation_helper< T >(copied);

    for (auto& p : plane_tuples.first)
      segments.push_back(p);

    for (auto& c : plane_tuples.second)
      coefs.push_back(c);
  }
}

template < typename T >
vector< PointCloudPtr< T > > cluster_helper(PointCloudPtr< T >& cloud)
{
  int                          debug_info_index = 0;
  vector< PointCloudPtr< T > > all_segments;

  auto cluster          = Segmentation< T, SegmentationType::EuclideanClusterExtraction >{cloud};
  auto clusters_indices = cluster.SetParameters(typename decltype(cluster)::Parameters{}).Segment();

  for (auto& indices : clusters_indices) {
    auto extractor = Extractor< T >{cloud, indices};
    auto segment   = extractor.SetExtractComplementaryCloud(false).Compute();
    all_segments.push_back(segment);
  }

  std::stringstream ss;
  ss << "detected " << clusters_indices.size() << " in plane";
  debug_info(ss.str(), debug_info_index++, 1);

  return all_segments;
}

template < typename T >
void cluster_euclidean_distance(const vector< PointCloudPtr< T > >& clouds,
                                vector< PointCloudPtr< T > >&       segments,
                                std::vector< int >&                 cloud_indices)
{

  int debug_info_index = 0;
  for (size_t i = 0; i < clouds.size(); ++i) {

    const auto& cloud = clouds[i];

    std::stringstream ss;

    PointCloudPtr< T > copied{new PointCloud< T >};
    pcl::copyPointCloud(*cloud, *copied);
    debug_info("begin clustering at each plane", debug_info_index++);

    auto clusters = cluster_helper< T >(copied);

    for (auto& cloud : clusters) {
      segments.push_back(cloud);
      cloud_indices.push_back(i);
    }
    debug_info("done clustering", debug_info_index++);
  }
}

template < typename T >
void projection_onto_plane(const std::vector< PointCloudPtr< T > >&     input,
                           const vector< pcl::ModelCoefficients::Ptr >& coefs,
                           const std::vector< int >                     cloud_indices,
                           std::vector< PointCloudPtr< T > >&           ret)
{
  for (size_t i = 0; i < input.size(); ++i) {
    const auto& cloud = input[i];
    const auto& coef  = coefs[cloud_indices[i]];

    auto projector = Filter< T, FilterType::ProjectUsingParametricModel >{cloud};
    auto projected = projector
                         .SetParameters(typename decltype(projector)::Parameters{pcl::SACMODEL_PLANE, coef})
                         .Compute();
    ret.push_back(projected);
  }
}

template < typename T >
void reconstruct(std::vector< PointCloudPtr< T > >&    inputs,
                 std::vector< pcl::PolygonMesh::Ptr >& meshes)
{
  Parameters::GreedyProjectionTriangulation params;

  for (size_t i = 0; i < inputs.size(); ++i) {
    auto& cloud   = inputs[i];
    auto  ne      = Processing< T, pcl::Normal, ProcessingType::NormalEstimation >{cloud};
    auto  normals = ne.SetParameters(typename decltype(ne)::Parameters{}).Compute();

    pcl::PointCloud< pcl::PointXYZRGBNormal >::Ptr cloud_with_normals{new pcl::PointCloud< pcl::PointXYZRGBNormal >};
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    pcl::PolygonMesh::Ptr mesh{new pcl::PolygonMesh};

    pcl::GreedyProjectionTriangulation< pcl::PointXYZRGBNormal > gp3;
    params.setup(gp3);
    pcl::search::KdTree< pcl::PointXYZRGBNormal >::Ptr tree(new pcl::search::KdTree< pcl::PointXYZRGBNormal >);
    tree->setInputCloud(cloud_with_normals);
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(*mesh);

    //    pcl::Poisson< pcl::PointXYZRGBNormal > poisson;
    //    poisson.setDepth(3);
    //    poisson.setInputCloud(cloud_with_normals);
    //    poisson.reconstruct(*mesh);

    std::cerr << " cloud_size : " << cloud->size() << std::endl;
    std::cerr << " polygon_size : " << mesh->polygons.size() << std::endl;

    meshes.push_back(mesh);
  }
}

template < typename T >
void processing1()
{
  using namespace pcl_wrapper;

  auto cloud = pcl_wrapper_tests::read_cloud< T >();

  PointCloudPtr< T > copied1{new PointCloud< T >};
  PointCloudPtr< T > copied2{new PointCloud< T >};
  PointCloudPtr< T > copied3{new PointCloud< T >};
  PointCloudPtr< T > copied4{new PointCloud< T >};

  pcl::copyPointCloud(*cloud, *copied1);
  pcl::copyPointCloud(*cloud, *copied2);
  pcl::copyPointCloud(*cloud, *copied3);
  pcl::copyPointCloud(*cloud, *copied4);

  clip< T >(copied1);

  clip< T >(copied2);
  filter_statistical_ourlier_removal< T >(copied2);

  clip< T >(copied3);
  filter_statistical_ourlier_removal< T >(copied3);
  filter_radius_ourlier_removal< T >(copied3);

  clip< T >(copied4);
  filter_statistical_ourlier_removal< T >(copied4);
  filter_radius_ourlier_removal< T >(copied4);
  processing_mls_smoothing< T >(copied4);

  vector< decltype(copied4) > clouds{copied4};
  decltype(clouds) segments1{clouds};
  decltype(clouds) segments2;
  decltype(clouds) segments3;

  vector< pcl::ModelCoefficients::Ptr > plane_coefs1;
  plane_segmentation< T >(segments1, segments2, plane_coefs1);

  vector< int > cloud_indices; // denote which segment does a cluster come from
  cluster_euclidean_distance< T >(segments2, segments3, cloud_indices);

  /*
   * Visulization part 1
   */

  std::shared_ptr< pcl::visualization::PCLVisualizer >
      viewer{new pcl::visualization::PCLVisualizer{"pipeline1"}};
  std::vector< int > viewports{0, 1, 2, 3};
  viewer->initCameraParameters();

  viewer->createViewPort(0.0, 0.5, 0.5, 1.0, viewports[0]);
  viewer->createViewPort(0.5, 0.5, 1.0, 1.0, viewports[1]);
  viewer->createViewPort(0.0, 0.0, 0.5, 0.5, viewports[2]);
  viewer->createViewPort(0.5, 0.0, 1.0, 0.5, viewports[3]);

  viewer->setBackgroundColor(0.8, 0.7, 0.7, viewports[0]);
  viewer->setBackgroundColor(0.7, 0.8, 0.7, viewports[1]);
  viewer->setBackgroundColor(0.8, 0.7, 0.8, viewports[2]);
  viewer->setBackgroundColor(0.7, 0.7, 0.8, viewports[3]);

  viewer->addText("1. Clip", 10, 10, 30, 0.0, 0.0, 0.0, "Viewport1", viewports[0]);
  viewer->addText("2. Statistical Outlier Removal", 10, 10, 30, 0.0, 0.0, 0.0, "Viewport2", viewports[1]);
  viewer->addText("3. Radius Outlier Removal", 10, 10, 30, 0.0, 0.0, 0.0, "Viewport3", viewports[2]);
  viewer->addText("4. MLS Smoothing", 10, 10, 30, 0.0, 0.0, 0.0, "Viewport4", viewports[3]);

  viewer->setSize(1024, 768);

  Color color1{128.0 / 255.0, 0.0 / 255.0, 0.0 / 255.0};
  Color color2{0.0 / 255.0, 128.0 / 255.0, 0.0 / 255.0};
  Color color3{128.0 / 255.0, 10.0 / 255.0, 128.0 / 255.0};
  Color color4{100.0 / 255.0, 100.0 / 255.0, 237.0 / 255.0};

  add_cloud< T >(viewer, cloud, {0.3, 0.3, 0.3}, 0.1, 2, viewports[0]);
  add_cloud< T >(viewer, cloud, {0.3, 0.3, 0.3}, 0.1, 2, viewports[1]);
  add_cloud< T >(viewer, cloud, {0.3, 0.3, 0.3}, 0.1, 2, viewports[2]);
  add_cloud< T >(viewer, cloud, {0.3, 0.3, 0.3}, 0.1, 2, viewports[3]);

  add_cloud< T >(viewer, copied1, color1, 0.9, 2, viewports[0]);
  add_cloud< T >(viewer, copied2, color2, 0.9, 2, viewports[1]);
  add_cloud< T >(viewer, copied3, color3, 0.9, 2, viewports[2]);
  add_cloud< T >(viewer, copied4, color4, 0.9, 2, viewports[3]);

  viewer->spin();
  viewer->close();
}

template < typename T >
void processing2()
{
  using namespace pcl_wrapper;

  auto cloud = pcl_wrapper_tests::read_cloud< T >();

  PointCloudPtr< T > copied{new PointCloud< T >};

  pcl::copyPointCloud(*cloud, *copied);

  auto sampler       = Sampler< T, SamplerType::DownSampling_VoxelGrid >{copied};
  auto sampled_cloud = sampler
                           .SetParameters(typename decltype(sampler)::Parameters{})
                           .Compute();

  copied = sampled_cloud;

  clip< T >(copied);
  filter_statistical_ourlier_removal< T >(copied);
  filter_radius_ourlier_removal< T >(copied);
  processing_mls_smoothing< T >(copied);

  vector< decltype(copied) > clouds{copied};
  decltype(clouds) segments1{clouds};
  decltype(clouds) segments2;
  decltype(clouds) segments3;
  decltype(clouds) segments4;
  decltype(clouds) segments5;
  decltype(clouds) segments6;

  vector< pcl::ModelCoefficients::Ptr > plane_coefs1;
  plane_segmentation< T >(segments1, segments2, plane_coefs1);

  vector< int > cloud_indices; // denote which segment does a cluster come from
  cluster_euclidean_distance< T >(segments2, segments3, cloud_indices);

  vector< pcl::ModelCoefficients::Ptr > plane_coefs2;
  plane_segmentation< T >(segments3, segments4, plane_coefs2);

  vector< int > cloud_indices2; // denote which segment does a cluster come from
  cluster_euclidean_distance< T >(segments4, segments5, cloud_indices2);

  projection_onto_plane< T >(segments5, plane_coefs2, cloud_indices2, segments6);

  std::vector< pcl::PolygonMesh::Ptr > meshes;
  reconstruct< T >(segments6, meshes);

  Color color1{128.0 / 255.0, 0.0 / 255.0, 0.0 / 255.0};
  Color color2{0.0 / 255.0, 128.0 / 255.0, 0.0 / 255.0};
  Color color3{128.0 / 255.0, 10.0 / 255.0, 128.0 / 255.0};
  Color color4{100.0 / 255.0, 100.0 / 255.0, 237.0 / 255.0};

  //
  std::shared_ptr< pcl::visualization::PCLVisualizer >
      viewer{new pcl::visualization::PCLVisualizer{"processing2"}};
  std::vector< int > viewports{0, 1, 2, 3};
  viewer->initCameraParameters();
  viewer->setSize(1024, 768);

  viewer->createViewPort(0.0, 0.5, 0.5, 1.0, viewports[0]);
  viewer->createViewPort(0.5, 0.5, 1.0, 1.0, viewports[1]);
  viewer->createViewPort(0.0, 0.0, 0.5, 0.5, viewports[2]);
  viewer->createViewPort(0.5, 0.0, 1.0, 0.5, viewports[3]);

  viewer->setBackgroundColor(0.8, 0.7, 0.7, viewports[0]);
  viewer->setBackgroundColor(0.7, 0.8, 0.7, viewports[1]);
  viewer->setBackgroundColor(0.8, 0.7, 0.8, viewports[2]);
  viewer->setBackgroundColor(0.7, 0.7, 0.8, viewports[3]);

  viewer->addText("1. SAC plane segmentation (using normals)", 10, 10, 30, 0.0, 0.0, 0.0, "Viewport1", viewports[0]);
  viewer->addText("2. Euclidean Distance Clustering", 10, 10, 30, 0.0, 0.0, 0.0, "Viewport2", viewports[1]);
  viewer->addText("3. SAC plane segmentation (using normals)\nProjection using plane model parameters", 10, 10, 30, 0.0, 0.0, 0.0, "Viewport3", viewports[2]);
  viewer->addText("4. Meshing the cloud\nGreedy Projection Triangulation", 10, 10, 30, 0.0, 0.0, 0.0, "Viewport4", viewports[3]);

  add_cloud< T >(viewer, cloud, {0.3, 0.3, 0.3}, 0.1, 2, viewports[0]);
  add_cloud< T >(viewer, cloud, {0.3, 0.3, 0.3}, 0.1, 2, viewports[1]);
  add_cloud< T >(viewer, cloud, {0.3, 0.3, 0.3}, 0.1, 2, viewports[2]);
  add_cloud< T >(viewer, cloud, {0.3, 0.3, 0.3}, 0.1, 2, viewports[3]);

  for_each(begin(segments2), end(segments2),
           [&](const PointCloudPtr< T >& seg) {
             auto c = Color::CreateRandomColor();
             add_cloud< T >(viewer, seg, c, 0.9, 2, viewports[0]);
           });

  for_each(begin(segments3), end(segments3),
           [&](const PointCloudPtr< T >& seg) {
             auto c = Color::CreateRandomColor();
             add_cloud< T >(viewer, seg, c, 0.9, 2, viewports[1]);
           });

  for_each(begin(segments6), end(segments6),
           [&](const PointCloudPtr< T >& seg) {
             auto c = Color::CreateRandomColor();
             add_cloud< T >(viewer, seg, c, 0.9, 2, viewports[2]);
           });

  for_each(begin(meshes), end(meshes),
           [&](const pcl::PolygonMesh::Ptr& mesh) {
             auto c = Color::CreateRandomColor();
             add_mesh(viewer, mesh, c, 0.9, viewports[3]);
           });

  viewer->spin();
  viewer->close();
}

int main(int argc, char** argv)
{
  srand(static_cast< long >(time(NULL)));
  processing1< Point6D >();
  processing2< Point6D >();
}
