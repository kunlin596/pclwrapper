#ifndef FILTER_HPP
#define FILTER_HPP

#include "parameters.hpp"
#include "types.hpp"

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace pcl_wrapper {

template < typename PointT, typename FilterT >
struct filter_t;

template < typename PointT >
struct filter_t< PointT, FilterType::StatisticalOutlierRemoval > {
    using type = pcl::StatisticalOutlierRemoval< PointT >;
};

template < typename PointT >
struct filter_t< PointT, FilterType::RadiusOutlierRemoval > {
    using type = pcl::RadiusOutlierRemoval< PointT >;
};

template < typename PointT >
struct filter_t< PointT, FilterType::ProjectUsingParametricModel > {
    using type = pcl::ProjectInliers< PointT >;
};

template < typename PointT, typename FilterT >
class Filter {

    using _Self       = Filter;
    using _Filter     = typename filter_t< PointT, FilterT >::type;
    using _FilterPtr  = std::shared_ptr< _Filter >;
    using _ParameterT = typename param_t< FilterT >::type;

public:
    using Parameters = _ParameterT;

    explicit Filter(InPointCloudPtr< PointT > cloud);

    _Self& SetParameters(const Parameters& params);

    OutPointCloudPtr< PointT > Compute();

private:
    _ParameterT _params;
    _FilterPtr  _filter;
};
}

#include "_impl/filter_impl.tcc"

#endif // FILTER_HPP
