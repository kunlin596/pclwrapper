#ifndef TYPES_HPP
#define TYPES_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace pcl {
namespace visualization {
  class PCLVisualizer;
}
}

namespace pcl_wrapper {

/* ---------------------------------------------------------------------------------------
 * Enum definitions
 * --------------------------------------------------------------------------------------- */
enum class IoEncodingType {
  Binary,
  AscII,
};

enum class PointDimension {
  X,
  Y,
  Z,
  W,
};

struct ProcessingType {
  struct NormalEstimation;
  struct MovingLeastSquares;
};

struct KeypointType {
  struct Sift3d;
  struct Harris3d;
  struct Harris6d;
};

struct SamplerType {
  struct DownSampling_VoxelGrid;
};

struct SegmentationType {
  struct SacModelPlane;
  struct SacModelPlaneFromNormals;
  struct EuclideanClusterExtraction;
};

struct FilterType {
  struct StatisticalOutlierRemoval;
  struct RadiusOutlierRemoval;
  struct ProjectUsingParametricModel;
};

struct SurfaceProcessingType {
  struct GreedyProjectionTriangulation;
};

/* ---------------------------------------------------------------------------------------
 * Some convenient alias definitions
 * --------------------------------------------------------------------------------------- */
// clang-format off
template < typename PointT >    using PointCloud = pcl::PointCloud<PointT>;
template < typename PointT >    using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;
template < typename InPointT >  using InPointCloud = pcl::PointCloud< InPointT >;
template < typename InPointT >  using InPointCloudPtr = typename InPointCloud< InPointT >::Ptr;
template < typename OutPointT > using OutPointCloud = pcl::PointCloud< OutPointT >;
template < typename OutPointT > using OutPointCloudPtr = typename OutPointCloud< OutPointT >::Ptr;
template < typename PointT >    using KdtreeSearchMethod = pcl::search::KdTree< PointT >;
template < typename PointT >    using KdtreeSearchMethodPtr = typename KdtreeSearchMethod< PointT >::Ptr;
template < typename KeypointT > using Keypoints = pcl::PointCloud< KeypointT >;
template < typename KeypointT > using KeypointsPtr = typename pcl::PointCloud< KeypointT >::Ptr;
// clang-format on

using Point3D         = pcl::PointXYZ;
using Point6D         = pcl::PointXYZRGB;
using Point7D         = pcl::PointXYZRGBA;
using PointCloud3D    = pcl::PointCloud<Point3D>;
using PointCloud6D    = pcl::PointCloud<Point6D>;
using PointCloud7D    = pcl::PointCloud<Point7D>;
using NormalCloud     = pcl::PointCloud<pcl::Normal>;
using NormalCloudPtr  = NormalCloud::Ptr;
using Indices         = pcl::PointCloud<int>;
using IndicesPtr      = pcl::PointCloud<int>::Ptr;
using PointIndices    = pcl::PointIndices;
using PointIndicesPtr = pcl::PointIndices::Ptr;
using ViewerPtr       = std::shared_ptr<pcl::visualization::PCLVisualizer>;

/* ---------------------------------------------------------------------------------------
 * Meta-functions for type information
 * --------------------------------------------------------------------------------------- */
///
/// Meta function for determine whether a value is of type pointer
/// Usage:
///     typedef int * IntPtr;
///     typedef int Foo::* FooMemberPtr;
///     typedef int (*FuncPtr)();
///     printf("%d\ n",is_pointer<IntPtr>::value);        // prints 1
///     printf("%d\ n",is_pointer<FooMemberPtr>::value);  // prints 1
///     printf("%d\ n",is_pointer<FuncPtr>::value);       // prints 1
///
///
template <typename T>
struct is_pointer {

  template <typename U>
  static char _is_pointer(U*); ///< Raw

  template <typename X, class Y>
  static char _is_pointer(Y X::*);

  template <typename U>
  static char _is_pointer(U (*)());

  static double _is_pointer(...);

  static T t;

  enum { value = sizeof(_is_pointer(t)) == sizeof(char) };
};

template <typename Function,
          typename OnSuccess,
          typename OnError,
          typename... Args>
void api_exec(Function func, OnSuccess on_success, OnError on_error, Args... args) {
  using ReturnType = typename std::result_of<Function(Args...)>::type;
  static_assert(std::is_integral<ReturnType>::value, "Please only call me with integral types!");

  int err = func(args...);
  if (err == 0) {
    on_success();
  } else {
    on_error(err);
  }
}

template <typename... Args>
struct type_list;

template <typename T>
struct type_list<T> { using head = T; };

template <typename Head, typename... Tail>
struct type_list<Head, Tail...> : type_list<Head> { using tail = type_list<Tail...>; };

template <typename...>
using void_t = void;

//template <typename T, typename = void>
//struct count : std::integral_constant<int, 1> {};

//template <typename T>
//struct count<T, void_t<typename T::tail> > : std::integral_constant<int, 1 + count<typename T::tail>()> {};

template <typename... Args>
struct count;

template <typename... Args>
struct count<type_list<Args...> > : std::integral_constant<int, sizeof...(Args)> {};
}
#endif // TYPES_HPP
