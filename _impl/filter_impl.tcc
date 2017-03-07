#pragma once

#include "../filter.hpp"

namespace pcl_wrapper {

template < typename PointT, typename FilterT >
Filter< PointT, FilterT >::Filter(InPointCloudPtr< PointT > cloud)
    : _filter{ new _Filter{} }
{
    _filter->setInputCloud(cloud);
}

template < typename PointT, typename FilterT >
typename Filter< PointT, FilterT >::_Self&
Filter< PointT, FilterT >::SetParameters(const Filter::Parameters& params)
{
    _params = params;
    _params.setup(*_filter);
    return *this;
}

template < typename PointT, typename FilterT >
OutPointCloudPtr< PointT >
Filter< PointT, FilterT >::Compute()
{
    OutPointCloudPtr< PointT > out{ new OutPointCloud< PointT > };
    _filter->filter(*out);
    return out;
}
}
