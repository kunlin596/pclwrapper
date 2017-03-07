/**
 *  @file   feature_impl.tcc
 *  @author Kun Lin
 *  @brief  features estimation implementation
 *  @date   2017/2/22
 *
 *  @section DESCRIPTION
 *
 *  \todo
 *
 */

#pragma once
#include "../feature.hpp"

namespace pcl_wrapper {

template < typename InPointT,
           typename OutPointT >
FeatureFromNormals< InPointT, OutPointT >::FeatureFromNormals(
    InPointCloudPtr< InPointT > cloud,
    NormalCloudPtr              normals)
    : _cloud{ cloud }
    , _normals{ normals }
    , _search_method{ new pcl::search::KdTree< InPointT >{} }
    , _estimation{ new _Estimation{} }

{
    _estimation->setSearchSurface(_cloud);
    _estimation->setInputNormals(_normals);

    _estimation->setSearchMethod(_search_method);
}

template < typename InPointT,
           typename OutPointT >
typename FeatureFromNormals< InPointT, OutPointT >::_Self&
FeatureFromNormals< InPointT, OutPointT >::SetIndices(IndicesPtr indices)
{
    _indices = indices;
    _estimation->setIndices(_indices);
    return *this;
}

template < typename InPointT,
           typename OutPointT >
template < typename _T >
typename FeatureFromNormals< InPointT, OutPointT >::_Self&
FeatureFromNormals< InPointT, OutPointT >::SetSearchRadius(_T radius)
{
    _radius = static_cast< float >(radius);
    _estimation->setRadiusSearch(_radius);
    return *this;
}

template < typename InPointT,
           typename OutPointT >
typename FeatureFromNormals< InPointT, OutPointT >::_Self&
FeatureFromNormals< InPointT, OutPointT >::SetEstimationSpace(InPointCloudPtr< InPointT > points)
{
    _estimation->setInputCloud(points);
    return *this;
}

template < typename InPointT,
           typename OutPointT >
OutPointCloudPtr< OutPointT >
FeatureFromNormals< InPointT, OutPointT >::Compute()
{
    OutPointCloudPtr< OutPointT > descriptors(new pcl::PointCloud< OutPointT >);
    _estimation->compute(*descriptors);
    return descriptors;
}
}
