#pragma once
#include "../sample.hpp"

namespace pcl_wrapper {
template < typename PointT >
UniformSampler< PointT >::UniformSampler(InPointCloudPtr< PointT > cloud)
    : _cloud{ cloud }
    , _radius(0.05f)
    , _sampler{ new pcl::UniformSampling< PointT > }
{
    _sampler->setInputCloud(_cloud);
}

template < typename PointT >
template < typename _T >
typename UniformSampler< PointT >::_Self& UniformSampler< PointT >::SetSearchRadius(_T radius)
{
    _radius = static_cast< float >(radius);
    _sampler->setRadiusSearch(_radius);
    return *this;
}

template < typename PointT >
Indices UniformSampler< PointT >::Compute()
{
    _sampler->compute(_indices);
    return _indices;
}

/* ---------------------------------------------------------------------------------------
     * Down sampler implementation
     * --------------------------------------------------------------------------------------- */

template < typename PointT, typename SamplerT >
Sampler< PointT, SamplerT >::Sampler(InPointCloudPtr< PointT > cloud)
    : _cloud{ cloud }
    , _sampler{ new Sampler::_SamplerT }
{
    _sampler->setInputCloud(_cloud);
}

template < typename PointT, typename SamplerT >
typename Sampler< PointT, SamplerT >::_Self&
Sampler< PointT, SamplerT >::SetParameters(const Sampler::_ParameterT& p)
{
    _parameters = p;
    _parameters.template setup(*_sampler);
    return *this;
}

template < typename PointT, typename SamplerT >
OutPointCloudPtr< PointT >
Sampler< PointT, SamplerT >::Compute()
{
    OutPointCloudPtr< PointT > out{ new OutPointCloud< PointT > };
    _sampler->filter(*out);
    return out;
}
}
