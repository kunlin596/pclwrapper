/**
 *  @file   estimation.hpp
 *  @author Kun Lin
 *  @brief  estimations
 *  @date   2017/2/22
 *
 *  @section DESCRIPTION
 *
 *  \todo
 *
 */

#ifndef ESTIMATION_HPP
#define ESTIMATION_HPP

#include "parameters.hpp"
#include "types.hpp"

#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>

#include <stdexcept>

namespace pcl_wrapper {

template < typename InPointT, typename OutPointT, typename ProcessingT >
struct processing_t;

template < typename InPointT, typename OutPointT >
struct processing_t< InPointT, OutPointT, ProcessingType::NormalEstimation > {
  using value = pcl::NormalEstimation< InPointT, OutPointT >;
};

template < typename InPointT, typename OutPointT >
struct processing_t< InPointT, OutPointT, ProcessingType::MovingLeastSquares > {
  using value = pcl::MovingLeastSquares< InPointT, OutPointT >;
};

template < typename PointT, typename SurfaceProcessingT >
struct surface_processing_t;

template < typename PointT, typename SurfaceProcessingT >
struct surface_processing_t {
  using value = pcl::GreedyProjectionTriangulation< PointT >;
};

template < typename InPointT,
           typename OutPointT,
           typename ProcessingT >
struct processing_delegate {
  static void run(ProcessingT&               p,
                  PointCloudPtr< InPointT >& ret)
  {
    return;
  }
};

template < typename InPointT,
           typename OutPointT >
struct processing_delegate< InPointT, OutPointT, pcl::NormalEstimation< InPointT, OutPointT > > {
  static void run(pcl::NormalEstimation< InPointT, OutPointT >& p,
                  PointCloudPtr< OutPointT >& ret)
  {
    p.compute(*ret);
  }
};

template < typename InPointT,
           typename OutPointT >
struct processing_delegate< InPointT, OutPointT, pcl::MovingLeastSquares< InPointT, OutPointT > > {
  static void run(pcl::MovingLeastSquares< InPointT, OutPointT >& p,
                  PointCloudPtr< OutPointT >& ret)
  {
    p.process(*ret);
  }
};

/* ---------------------------------------------------------------------------------------
 * Processing class
 * --------------------------------------------------------------------------------------- */
template < typename InPointT,
           typename OutPointT,
           typename ProcessingT >
class Processing {

  using _Self          = Processing;
  using _Processing    = typename processing_t< InPointT, OutPointT, ProcessingT >::value;
  using _ProcessingPtr = std::shared_ptr< _Processing >;
  using _Parameters    = typename param_t< ProcessingT >::type;
  using _ExecFuncPtr   = void (*)(_Processing&, PointCloudPtr< OutPointT >&);

  public:
  using Parameters = _Parameters;

  explicit Processing(PointCloudPtr< InPointT > cloud);

  _Self& SetParameters(const Parameters& p);

  PointCloudPtr< OutPointT > Compute();

  private:
  Parameters                        _params;
  PointCloudPtr< InPointT >         _cloud;
  KdtreeSearchMethodPtr< InPointT > _search_method;
  _ProcessingPtr                    _processing;

  static constexpr _ExecFuncPtr _exec_func_ptr
      = &processing_delegate< InPointT, OutPointT, _Processing >::run;
};

template < typename PointT,
           typename SurfaceProcessingT >
class SurfaceProcessing {
  /// \todo
  //  using _Self          = SurfaceProcessing;
  //  using _Processing    = typename processing_t< InPointT, OutPointT, ProcessingT >::value;
  //  using _ProcessingPtr = std::shared_ptr< _Processing >;
  //  using _Parameters    = typename param_t< ProcessingT >::type;
  //  using _ExecFuncPtr   = void (*)(_Processing&, PointCloudPtr< OutPointT >&);
};
}
#include "_impl/estimation_impl.tcc"
#endif // ESTIMATION_HPP
