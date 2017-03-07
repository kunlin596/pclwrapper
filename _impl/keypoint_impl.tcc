#pragma once
#include "../keypoint.hpp"

namespace pcl_wrapper {

template < typename InPointT,
           typename OutPointT,
           typename KeypointT >
Keypoint< InPointT, OutPointT, KeypointT >::Keypoint(InPointCloudPtr< InPointT > cloud)
    : _cloud{ cloud }
    , _detector{ new _DetectorType }
    , _search_method{ new KdtreeSearchMethod< InPointT > }
{
    _detector->setInputCloud(_cloud);
    _detector->setSearchMethod(_search_method);
}

template < typename InPointT,
           typename OutPointT,
           typename KeypointT >
typename Keypoint< InPointT, OutPointT, KeypointT >::_Self&
Keypoint< InPointT, OutPointT, KeypointT >::SetParameters(const Keypoint::_ParameterType& p)
{
    _parameters = p;

    // ISO C++03 14.2/4:
    // When the name of a member template specialization appears after . or -> in a postfix-expression,
    // or after nested-name-specifier in a qualified-id,
    // and the postfix-expression or qualified-id explicitly depends on a template-parameter (14.6.2),
    // the member template name must be prefixed by the keyword template. Otherwise the name is assumed to name a non-template.
    _parameters.template setup(*_detector);
    return *this;
}

template < typename InPointT,
           typename OutPointT,
           typename KeypointT >
OutPointCloudPtr< OutPointT > Keypoint< InPointT, OutPointT, KeypointT >::Compute()
{
    OutPointCloudPtr< OutPointT > keypoints{ new OutPointCloud< OutPointT > };
    _detector->compute(*keypoints);
    return keypoints;
}
}
