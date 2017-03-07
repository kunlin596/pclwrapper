#ifndef SEGMENTATION_HPP
#define SEGMENTATION_HPP

#include "parameters.hpp"
#include "types.hpp"
#include "util.hpp"

#include <memory>
#include <type_traits>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace {
const int MIN_SEGMENT_SIZE        = 500;
const int MIN_NUM_POINTS_REQUIRED = 2000;
}

namespace pcl_wrapper {

/* ---------------------------------------------------------------------------------------
 * Static segmentation selectors
 * --------------------------------------------------------------------------------------- */

template < typename InPointT, typename SegmentationT >
struct segmentation_t;

template < typename InPointT >
struct segmentation_t< InPointT, SegmentationType::SacModelPlane > {
  using type = pcl::SACSegmentation< InPointT >;
};

template < typename InPointT >
struct segmentation_t< InPointT, SegmentationType::SacModelPlaneFromNormals > {
  using type = pcl::SACSegmentationFromNormals< InPointT, pcl::Normal >;
};

template < typename InPointT >
struct segmentation_t< InPointT, SegmentationType::EuclideanClusterExtraction > {
  using type = pcl::EuclideanClusterExtraction< InPointT >;
};

template < typename SegmentationT >
struct require_normals;

template <>
struct require_normals< SegmentationType::SacModelPlaneFromNormals > {
  static const bool value = true;
};

template <>
struct require_normals< SegmentationType::SacModelPlane > {
  static const bool value = false;
};

template <>
struct require_normals< SegmentationType::EuclideanClusterExtraction > {
  static const bool value = false;
};

// Concluded different kinds of segmentation together
// Consider cluster and plane segmentation are the same
template < typename InPointT, typename SegmentationT >
struct segmentation_delegate {
  static void run(SegmentationT&                            s,
                  std::vector< PointIndices >&              i,
                  std::vector< pcl::ModelCoefficientsPtr >& c)
  {
    PointIndicesPtr             indices{new PointIndices};
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    s.segment(*indices, *coefficients);
    i.push_back(*indices);
    c.push_back(coefficients);
  }
};

template < typename InPointT >
struct segmentation_delegate< InPointT, pcl::EuclideanClusterExtraction< InPointT > > {
  static void run(pcl::EuclideanClusterExtraction< InPointT >& s,
                  std::vector< PointIndices >&                 i,
                  std::vector< pcl::ModelCoefficientsPtr >&    c)
  {
    s.extract(i);
  }
};

/* ---------------------------------------------------------------------------------------
 * Segmentation class
 * --------------------------------------------------------------------------------------- */

template < typename PointT, typename SegmentationT >
class Segmentation {
  using _Self = Segmentation;

  protected:
  using _Segmentation         = typename segmentation_t< PointT, SegmentationT >::type;
  using _SegmentationPtr      = std::shared_ptr< _Segmentation >;
  using _ModelCoefficients    = pcl::ModelCoefficients;
  using _ModelCoefficientsPtr = pcl::ModelCoefficients::Ptr;
  using _ExecFuncPtr          = void (*)(_Segmentation&,
                                std::vector< PointIndices >&,
                                std::vector< pcl::ModelCoefficientsPtr >&);

  public:
  using Segments   = std::vector< PointIndices >;
  using Parameters = typename param_t< SegmentationT >::type;

  explicit Segmentation(InPointCloudPtr< PointT > cloud);

  _Self& SetParameters(const Parameters& p);

  template < typename T2_Segmentation = SegmentationT, // Must be depends on class template
             typename Enabled         = typename std::enable_if< require_normals< T2_Segmentation >::value >::type >
  _Self& SetNormals(NormalCloudPtr normals)
  {
    _seg->setInputNormals(normals);
    return *this;
  }

  _Self& SetInputCloud(const PointCloudPtr< PointT >& cloud)
  {
    _seg->setInputCloud(cloud);
    return *this;
  }

  std::vector< _ModelCoefficientsPtr > GetModelCoefficients() const
  {
    return _model_coefficients;
  }

  Segments                Segment();
  Segments                GetSegments() const;
  PointCloudPtr< PointT > GetCloud() const;

  protected:
  PointCloudPtr< PointT >              _cloud;
  _SegmentationPtr                     _seg;
  Segments                             _segments;
  std::vector< _ModelCoefficientsPtr > _model_coefficients;
  Parameters                           _params;

  static constexpr _ExecFuncPtr _exec_func_ptr
      = &segmentation_delegate< PointT, _Segmentation >::run;
};

template < typename PointT >
struct SegmentTreeNode {
  PointCloudPtr< PointT >                         cloud_data;
  std::vector< std::weak_ptr< SegmentTreeNode > > children;
};
}

#include "_impl/segmentation_impl.tcc"

#endif // SEGMENTATION_HPP
