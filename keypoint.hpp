#ifndef KEYPOINT_HPP
#define KEYPOINT_HPP

#include "parameters.hpp"
#include "types.hpp"

#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/keypoints/sift_keypoint.h>

namespace pcl_wrapper {

/* ---------------------------------------------------------------------------------------
 * Static keypoint detector selectors
 * --------------------------------------------------------------------------------------- */

template < typename InPointT, typename OutPointT, typename KeypointT >
struct detector_type;

template < typename InPointT, typename OutPointT >
struct detector_type< InPointT, OutPointT, KeypointType::Sift3d > {
    using type = pcl::SIFTKeypoint< InPointT, OutPointT >;
};

template < typename InPointT, typename OutPointT >
struct detector_type< InPointT, OutPointT, KeypointType::Harris3d > {
    using type = pcl::HarrisKeypoint3D< InPointT, OutPointT >;
};

template < typename InPointT, typename OutPointT >
struct detector_type< InPointT, OutPointT, KeypointType::Harris6d > {
    using type = pcl::HarrisKeypoint6D< InPointT, OutPointT >;
};

/* ---------------------------------------------------------------------------------------
 * Keypoint Detector class
 * --------------------------------------------------------------------------------------- */

template < typename InPointT,
           typename OutPointT,
           typename KeypointT >
class Keypoint {

    using _Self          = Keypoint;
    using _DetectorType  = typename detector_type< InPointT, OutPointT, KeypointT >::type;
    using _ParameterType = typename param_t< KeypointT >::type;

public:
    using PointOutType = OutPointT;
    using PointInType  = InPointT;
    using Parameters   = _ParameterType;

    explicit Keypoint(InPointCloudPtr< InPointT > cloud);

    _Self& SetParameters(const Parameters& p);

    OutPointCloudPtr< OutPointT > Compute();

private:
    std::shared_ptr< _DetectorType >  _detector;
    InPointCloudPtr< InPointT >       _cloud;
    KdtreeSearchMethodPtr< InPointT > _search_method;
    Parameters                        _parameters;
};
}
#include "_impl/keypoint_impl.tcc"

#endif // KEYPOINT_HPP
