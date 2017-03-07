/**
 *  @file   estimation_impl.tcc
 *  @author Kun Lin
 *  @date   2017/2/22
 *  @brief  Implementation of estimations
 *
 *  @section DESCRIPTION
 *
 *  \todo
 *
 */

#pragma once
#include "../estimation.hpp"

namespace pcl_wrapper {

template < typename InPointT,
           typename OutPointT,
           typename ProcessingT >
Processing< InPointT, OutPointT, ProcessingT >::Processing(PointCloudPtr< InPointT > cloud)
    : _cloud{ cloud }
    , _search_method{ new pcl::search::KdTree< InPointT > }
{
    if (_cloud->empty())
        throw std::runtime_error{ "Cloud is empty." };

    // Because of the implementation of PCL
    // Using the search method (Building KdTree) must happen before input the cloud
    // Otherwise KdTree will use an empty cloud to build the tree.
    // This will cause normal estimation init failure.
    _processing = _ProcessingPtr{ new _Processing };
    _processing->setSearchMethod(_search_method);
    _processing->setInputCloud(cloud);
}

template < typename InPointT,
           typename OutPointT,
           typename ProcessingT >
PointCloudPtr< OutPointT >
Processing< InPointT, OutPointT, ProcessingT >::Compute() try {
    PointCloudPtr< OutPointT > result(new pcl::PointCloud< OutPointT >);
    (*_exec_func_ptr)(*_processing, result);
    return result;
} catch (...) {
    std::cerr << "Caught unknown error." << std::endl;
    return OutPointCloudPtr< OutPointT >{};
}

template < typename InPointT,
           typename OutPointT,
           typename ProcessingT >
typename Processing< InPointT, OutPointT, ProcessingT >::_Self&
Processing< InPointT, OutPointT, ProcessingT >::SetParameters(const Parameters& p)
{
    _params = p;
    _params.setup(*_processing);
    return *this;
}
}
