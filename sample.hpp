#ifndef SAMPLE_HPP
#define SAMPLE_HPP

#include "parameters.hpp"
#include "types.hpp"

#include <memory>

#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/point_cloud.h>

namespace pcl_wrapper {

template < typename PointT >
class UniformSampler {
    using _Self               = UniformSampler;
    using _UniformSamplingPtr = std::shared_ptr< pcl::UniformSampling< PointT > >;

public:
    explicit UniformSampler(InPointCloudPtr< PointT > cloud);

    template < typename _T >
    _Self& SetSearchRadius(_T radius);

    Indices Compute();

private:
    _UniformSamplingPtr       _sampler;
    InPointCloudPtr< PointT > _cloud;
    Indices                   _indices;
    float                     _radius;
};

/* ---------------------------------------------------------------------------------------
 * Down sampler class
 * --------------------------------------------------------------------------------------- */

template < typename PointT, typename SamplerT >
struct sampler_t;

template < typename PointT >
struct sampler_t< PointT, SamplerType::DownSampling_VoxelGrid > {
    using type = pcl::VoxelGrid< PointT >;
};

template < typename PointT, typename SamplerT >
class Sampler {
private:
    using _Self       = Sampler;
    using _SamplerT   = typename sampler_t< PointT, SamplerT >::type;
    using _ParameterT = typename param_t< SamplerT >::type;

public:
    using Parameters = _ParameterT;

    explicit Sampler(PointCloudPtr< PointT > cloud);

    _Self& SetParameters(const _ParameterT& parameters);

    PointCloudPtr< PointT > Compute();

private:
    std::shared_ptr< _SamplerT > _sampler;
    PointCloudPtr< PointT >      _cloud;
    _ParameterT                  _parameters;
};
}

#include "_impl/sample_impl.tcc"

#endif // SAMPLE_HPP
