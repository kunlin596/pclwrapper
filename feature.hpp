/**
 *  @file   feature.hpp
 *  @author Kun Lin
 *  @brief  features
 *  @date   2017/2/22
 *
 *  @section DESCRIPTION
 *
 *  \todo
 *
 */

#ifndef FEATURE_HPP
#define FEATURE_HPP

#include "types.hpp"

#include <memory>

#include <pcl/features/pfh.h>
#include <pcl/search/impl/search.hpp>

namespace pcl_wrapper {

/* ---------------------------------------------------------------------------------------
 * Some convenient meta functions for type mapping
 * --------------------------------------------------------------------------------------- */

template < typename InPointT, typename OutPointT >
struct feature_from_normal_estimator;

template < typename InPointT >
struct feature_from_normal_estimator< InPointT, pcl::PFHSignature125 > {
    using type = pcl::PFHEstimation< InPointT, pcl::Normal, pcl::PFHSignature125 >;
};

template < typename InPointT >
struct feature_from_normal_estimator< InPointT, pcl::PFHRGBSignature250 > {
    using type = pcl::PFHEstimation< InPointT, pcl::Normal, pcl::PFHRGBSignature250 >;
};

/* ---------------------------------------------------------------------------------------
 * Features detetion using normals
 * --------------------------------------------------------------------------------------- */

template < typename InPointT, typename OutPointT >
class FeatureFromNormals {

    using _Self       = FeatureFromNormals;
    using _Estimation = typename feature_from_normal_estimator< InPointT, OutPointT >::type;

public:
    using PointInType  = InPointT;
    using PointOutType = OutPointT;

    explicit FeatureFromNormals(InPointCloudPtr< InPointT > cloud,
                                NormalCloudPtr                 normals);

    template < typename _T >
    _Self& SetSearchRadius(_T radius);

    ///
    /// \brief SetSearchSurface
    ///     The surface (points) used to do the estimation
    /// \param points
    /// \return
    ///
    _Self& SetEstimationSpace(InPointCloudPtr< InPointT > points);

    ///
    /// \brief SetIndices
    ///     \todo
    /// \param indices
    /// \return
    ///
    _Self& SetIndices(IndicesPtr indices);

    OutPointCloudPtr< OutPointT > Compute();

private:
    std::shared_ptr< _Estimation >       _estimation;
    InPointCloudPtr< InPointT >       _cloud;
    KdtreeSearchMethodPtr< InPointT > _search_method;
    NormalCloudPtr                       _normals;
    IndicesPtr                           _indices;
    float                                _radius;
};
}
#include "_impl/feature_impl.tcc"
#endif // FEATURE_HPP
