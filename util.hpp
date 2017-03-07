#ifndef UTIL_HPP
#define UTIL_HPP

#include "types.hpp"

#include <map>
#include <stdexcept>
#include <string>

#include <boost/filesystem.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

// For parsing parameters
#include "yaml-cpp/yaml.h"

namespace {
std::map< pcl_wrapper::PointDimension, const char* > dimension_string = {
    {pcl_wrapper::PointDimension::X, "x"},
    {pcl_wrapper::PointDimension::Y, "y"},
    {pcl_wrapper::PointDimension::Z, "z"},
};
}

namespace pcl_wrapper {

struct IO {

  template < typename _Tp >
  static typename pcl::PointCloud< _Tp >::Ptr Read(const std::string& name) try {
    PointCloudPtr< _Tp > cloud{new PointCloud< _Tp >};

    auto ext = boost::filesystem::extension(name);
    if (ext.empty())
      throw std::runtime_error{"Can't find a propriate file extension of data file!"};

    std::cerr << "Reading a `" << ext << "` file." << std::endl;

    if (ext.compare(".pcd") == 0) {
      if (pcl::io::loadPCDFile< _Tp >(name, *cloud) == -1)
        throw std::runtime_error{"Can't read `.pcd` file."};
    } else if (ext.compare(".ply") == 0) {
      if (pcl::io::loadPLYFile< _Tp >(name, *cloud) == -1)
        throw std::runtime_error{"Can't read `.ply` file."};
    } else {
      throw std::runtime_error{"unsupported data format."};
    }
    return cloud;

  } catch (const std::runtime_error& e) {
    std::cerr << e.what() << std::endl;
    return nullptr;
  } catch (...) {
    std::cerr << "Caught unknown error." << std::endl;
    return nullptr;
  }

  template < typename _Tp >
  static void Save(const std::string&                        name,
                   typename pcl::PointCloud< _Tp >::ConstPtr cloud,
                   IoEncodingType                            encoding_type) try {

    auto ext = boost::filesystem::extension(name);
    if (ext.empty())
      throw std::runtime_error{"Can't find a propriate file extension of data file!"};

    std::cerr << "Saving a `" << ext << "` file." << std::endl;

    if (ext.compare(".pcd") == 0) {
      switch (encoding_type) {
      case IoEncodingType::Binary:
        if (pcl::io::savePCDFileASCII< _Tp >(name, *cloud) == -1)
          throw std::runtime_error{"Can't write `.pcd` file in binary form."};
        break;
      case IoEncodingType::AscII:
        if (pcl::io::savePCDFileASCII< _Tp >(name, *cloud) == -1)
          throw std::runtime_error{"Can't write `.pcd` file in ascii form."};
        break;
      }
    } else if (ext.compare(".ply") == 0) {
      switch (encoding_type) {
      case IoEncodingType::Binary:
        if (pcl::io::savePLYFileASCII< _Tp >(name, *cloud) == -1)
          throw std::runtime_error{"Can't write `.ply` file in binary form."};
        break;
      case IoEncodingType::AscII:
        if (pcl::io::savePLYFileASCII< _Tp >(name, *cloud) == -1)
          throw std::runtime_error{"Can't write `.ply` file in ascii form."};
        break;
      }
    } else {
      throw std::runtime_error{"unsupported data format."};
    }
  } catch (const std::runtime_error& e) {
    std::cerr << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Caught unknown error." << std::endl;
  }
};

struct Tool {
  template < typename PointT, PointDimension D >
  static OutPointCloudPtr< PointT > ClipCloud(typename pcl::PointCloud< PointT >::ConstPtr ptr, double lo, double hi)
  {
    typename pcl::PointCloud< PointT >::Ptr filtered_cloud{new pcl::PointCloud< PointT >{}};

    pcl::PassThrough< PointT > pass;
    pass.setInputCloud(ptr);
    pass.setFilterFieldName(dimension_string[D]);
    pass.setFilterLimits(lo, hi);
    pass.filter(*filtered_cloud);
    return filtered_cloud;
  }

  template < typename InPointT, typename OutPointT >
  static OutPointCloudPtr< OutPointT > Convert(typename PointCloud< InPointT >::ConstPtr from)
  {
    auto to = OutPointCloudPtr< OutPointT >{new OutPointCloud< OutPointT >};
    pcl::copyPointCloud(*from, *to);
    return to;
  }
};

template < template < typename... > class ContainerT = std::vector,
           typename Top,
           typename Sub = typename Top::value_type >
static ContainerT< typename Sub::value_type > Flatten(const Top& all)
{
  using std::begin;
  using std::end;

  ContainerT< typename Sub::value_type > accum;

  for (auto& sub : all) {
    accum.insert(end(accum), begin(sub), end(sub));
  }
  return accum;
}

template < typename PointT >
class Extractor {
  using _Self         = Extractor;
  using _Extractor    = pcl::ExtractIndices< PointT >;
  using _ExtractorPtr = typename _Extractor::Ptr;

  public:
  explicit Extractor(const PointCloudPtr< PointT >& cloud,
                     const PointIndices&            indices);

  _Self& SetExtractComplementaryCloud(bool b);
  _Self& SetInputCloud(InPointCloudPtr< PointT > cloud);
  _Self& SetIndices(const PointIndices& indices);

  PointCloudPtr< PointT > Compute();

  private:
  InPointCloudPtr< PointT > _cloud;
  PointIndices              _indices;
  _ExtractorPtr             _extractor;
};
}

#include "_impl/util_impl.tcc"

#endif // UTIL_HPP
