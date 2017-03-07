#ifndef PROCESSING_H
#define PROCESSING_H

// needed for undefined reference to [pcl::search::Search<pcl::PointXYZRGBNormal>::getName[abi:cxx11]() const] error
#include <pcl/search/impl/search.hpp>
#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif // PCL_NO_PRECOMPILE

#include "estimation.hpp"
#include "feature.hpp"
#include "filter.hpp"
#include "keypoint.hpp"
#include "sample.hpp"
#include "segmentation.hpp"

#endif // PROCESSING_H
