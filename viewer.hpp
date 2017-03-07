#ifndef VIEWER_HPP
#define VIEWER_HPP

#include "types.hpp"

#include <memory>
#include <pcl/visualization/cloud_viewer.h>
#include <random>
#include <string>
#include <vector>

namespace pcl_wrapper {

struct Color {
  double r;
  double g;
  double b;

  Color(double _r, double _g, double _b)
      : r{_r}
      , g{_g}
      , b{_b}
  {
  }

  Color()                 = default;
  Color(const Color& rhs) = default;

  static Color CreateRandomColor(double lo = 0.0, double hi = 1.0)
  {
    Color c;

    c.r = static_cast< double >(rand()) / static_cast< double >(RAND_MAX) * (hi - lo) + lo;
    c.g = static_cast< double >(rand()) / static_cast< double >(RAND_MAX) * (hi - lo) + lo;
    c.b = static_cast< double >(rand()) / static_cast< double >(RAND_MAX) * (hi - lo) + lo;

    return c;
  }

  static Color CreateRandomColorBasedOn(const Color& c, double delta = 0.2)
  {
    Color cc;

    cc.r = static_cast< double >(rand()) / static_cast< double >(RAND_MAX) * delta * 2 + c.r - delta;
    cc.g = static_cast< double >(rand()) / static_cast< double >(RAND_MAX) * delta * 2 + c.g - delta;
    cc.b = static_cast< double >(rand()) / static_cast< double >(RAND_MAX) * delta * 2 + c.b - delta;

    return cc;
  }
};

template < typename T >
void add_cloud(const std::shared_ptr< pcl::visualization::PCLVisualizer >& viewer,
               const PointCloudPtr< T >&                                   cloud,
               const Color&                                                c,
               const double                                                alpha,
               const int                                                   point_size,
               const int                                                   viewport = 0)
{
  std::stringstream ss;
  ss << "cloud_" << static_cast< int >(rand());
  auto buf  = ss.str();
  auto name = buf.c_str();

  viewer->addPointCloud< T >(cloud, name, viewport);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c.r, c.g, c.b, name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, alpha, name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, name);
}

void add_mesh(const std::shared_ptr< pcl::visualization::PCLVisualizer >& viewer,
              const pcl::PolygonMesh::Ptr&                                mesh,
              const Color&                                                c,
              const double                                                alpha,
              const int                                                   viewport = 0)
{
  std::stringstream ss;
  ss << "mesh_" << static_cast< int >(rand());
  auto buf  = ss.str();
  auto name = buf.c_str();

  viewer->addPolygonMesh(*mesh, name, viewport);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c.r, c.g, c.b, name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, alpha, name);
}
}

#endif // VIEWER_HPP
