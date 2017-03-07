#pragma once

#include "../util.hpp"
#include <boost/core/null_deleter.hpp>
#include <boost/shared_ptr.hpp>

namespace pcl_wrapper {

template < typename PointT >
Extractor< PointT >::Extractor(const InPointCloudPtr< PointT >& cloud,
                               const PointIndices&              indices)
    : _cloud{ cloud }
    , _indices{ indices }
    , _extractor{ new _Extractor{} }
{
    _extractor->setInputCloud(_cloud);
    _extractor->setIndices(boost::shared_ptr< PointIndices const >(&_indices, boost::null_deleter()));
    _extractor->setNegative(false);
}

template < typename PointT >
OutPointCloudPtr< PointT >
Extractor< PointT >::Compute()
{
    OutPointCloudPtr< PointT > out{ new OutPointCloud< PointT > };
    _extractor->filter(*out);
    return out;
}

template < typename PointT >
typename Extractor< PointT >::_Self&
Extractor< PointT >::SetExtractComplementaryCloud(bool b)
{
    _extractor->setNegative(b);
    return *this;
}

template < typename PointT >
typename Extractor< PointT >::_Self&
Extractor< PointT >::SetInputCloud(InPointCloudPtr< PointT > cloud)
{
    _cloud = cloud;
    _extractor->setInputCloud(_cloud);
}

template < typename PointT >
typename Extractor< PointT >::_Self&
Extractor< PointT >::SetIndices(const PointIndices& indices)
{
    _indices = indices;
    _extractor->setIndices(boost::shared_ptr< PointIndices const >(&_indices, boost::null_deleter()));
}
}
