#pragma once

#include "../segmentation.hpp"

namespace pcl_wrapper {

template < typename PointT, typename SegmentationT >
Segmentation< PointT, SegmentationT >::Segmentation(InPointCloudPtr< PointT > cloud)
    : _cloud(cloud)
    , _seg(new _Segmentation)
{
    if (_cloud->empty())
        throw std::runtime_error{ "Initial cloud is empty!" };
    _seg->setInputCloud(_cloud);
}

template < typename PointT, typename SegmentationT >
typename Segmentation< PointT, SegmentationT >::_Self&
Segmentation< PointT, SegmentationT >::SetParameters(const Parameters& p)
{
    _params = p;
    _params.setup(*_seg);
    return *this;
}

template < typename PointT, typename SegmentationT >
typename Segmentation< PointT, SegmentationT >::Segments
Segmentation< PointT, SegmentationT >::Segment() try {
    (*_exec_func_ptr)(*_seg, _segments, _model_coefficients);
    return _segments;
} catch (...) {
    std::cerr << "Segmentation unknown error." << std::endl;
}

template < typename PointT, typename SegmentationT >
typename Segmentation< PointT, SegmentationT >::Segments
Segmentation< PointT, SegmentationT >::GetSegments() const
{
    return _segments;
}

template < typename PointT, typename SegmentationT >
PointCloudPtr< PointT >
Segmentation< PointT, SegmentationT >::GetCloud() const
{
    return _cloud;
}
}
