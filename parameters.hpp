#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include "types.hpp"
#include <boost/core/null_deleter.hpp>
#include <yaml-cpp/yaml.h>

namespace pcl_wrapper {

/* ---------------------------------------------------------------------------------------
 * Various keypoint type definitions
 * --------------------------------------------------------------------------------------- */

static const char* keypoint_config_path     = "config/keypoint.yaml";
static const char* filter_config_path       = "config/filter.yaml";
static const char* segmentation_config_path = "config/segmentation.yaml";
static const char* processing_config_path   = "config/processing.yaml";
static const char* feature_config_path      = "config/feature.yaml";
static const char* surface_config_path      = "config/surface.yaml";

struct Parameters {

  struct NormalEstimation {
    float search_radius;

    NormalEstimation(float _search_radius)
        : search_radius{_search_radius}
    {
    }

    NormalEstimation()
    {
      auto config   = YAML::LoadFile(processing_config_path)["normal_estimation"];
      search_radius = config["search_radius"].as< float >();
    }

    template < typename ProcessingT >
    void setup(ProcessingT& p)
    {
      p.setRadiusSearch(search_radius);
    }
  };

  struct MovingLeastSquares {
    float search_radius;
    bool  set_polynomial_fit;

    MovingLeastSquares(float _search_radius, bool _set_polynomial_fit)
        : search_radius{_search_radius}
        , set_polynomial_fit{_set_polynomial_fit}
    {
    }

    MovingLeastSquares()
    {
      auto config        = YAML::LoadFile(processing_config_path)["moving_least_squares"];
      search_radius      = config["search_radius"].as< float >();
      set_polynomial_fit = config["set_polynomial_fit"].as< bool >();
    }

    template < typename ProcessingT >
    void setup(ProcessingT& p)
    {
      p.setSearchRadius(search_radius);
      p.setPolynomialFit(set_polynomial_fit);
    }
  };

  struct Sift3d {
    float min_scale            = 0.01f;
    int   nr_octaves           = 5;
    int   nr_scales_per_octave = 5;
    float min_contrast         = 0.075f;

    Sift3d(float _min_scale,
           int   _nr_octaves,
           int   _nr_scales_per_octaves,
           float _min_contrast)
        : min_scale{_min_scale}
        , nr_octaves{_nr_octaves}
        , nr_scales_per_octave{_nr_scales_per_octaves}
        , min_contrast{_min_contrast}
    {
      auto config          = YAML::LoadFile(keypoint_config_path)["sift3d"];
      min_scale            = config["min_scale"].as< float >();
      nr_octaves           = config["nr_octaves"].as< int >();
      nr_scales_per_octave = config["nr_scales_per_octave"].as< int >();
      min_contrast         = config["min_contrast"].as< float >();
    }

    Sift3d()
    {
      auto config          = YAML::LoadFile(keypoint_config_path)["sift3d"];
      min_scale            = config["min_scale"].as< float >();
      nr_octaves           = config["nr_octaves"].as< int >();
      nr_scales_per_octave = config["nr_scales_per_octave"].as< int >();
      min_contrast         = config["min_contrast"].as< float >();
    }

    ///
    /// delegate function for seting up the parameters
    ///
    template < typename DetectorT >
    void setup(DetectorT& d)
    {
      d.setScales(min_scale,
                  nr_octaves,
                  nr_scales_per_octave);
      d.setMinimumContrast(min_contrast);
    }
  };

  struct Harris3d {
    /// \todo

    ///
    /// delegate function for seting up the parameters
    ///
    template < typename DetectorT >
    void setup(DetectorT& d)
    {
      // \todo
    }
  };

  struct Harris6d {
    /// \todo

    ///
    /// delegate function for seting up the parameters
    ///
    template < typename DetectorT >
    void setup(DetectorT& d)
    {
      // \todo
    }
  };

  struct VoxelGridSampler {

    float leaf_size;

    VoxelGridSampler(float _leaf_size)
        : leaf_size{_leaf_size}
    {
    }

    VoxelGridSampler()
    {
      auto config = YAML::LoadFile(filter_config_path)["voxel_grid_sampler"];
      leaf_size   = config["leaf_size"].as< float >();
    }

    ///
    /// delegate function for seting up the parameters
    ///
    template < typename SamplerT >
    void setup(SamplerT& s)
    {
      s.setLeafSize(leaf_size, leaf_size, leaf_size);
    }
  };

  struct SacModel_Plane {
    //    enum SacModel
    //    {
    //      SACMODEL_PLANE      = 0,
    //      SACMODEL_LINE       = 1,
    //      SACMODEL_CIRCLE2D   = 2,
    //      SACMODEL_CIRCLE3D   = 3,
    //      SACMODEL_SPHERE     = 4,
    //      SACMODEL_CYLINDER,
    //      SACMODEL_CONE,
    //      SACMODEL_TORUS,
    //      SACMODEL_PARALLEL_LINE,
    //      SACMODEL_PERPENDICULAR_PLANE = 9,
    //      SACMODEL_PARALLEL_LINES      = 10,
    //      SACMODEL_NORMAL_PLANE        = 11,
    //      SACMODEL_NORMAL_SPHERE,
    //      SACMODEL_REGISTRATION,
    //      SACMODEL_REGISTRATION_2D,
    //      SACMODEL_PARALLEL_PLANE,
    //      SACMODEL_NORMAL_PARALLEL_PLANE,
    //      SACMODEL_STICK
    //    };

    int   max_iterations;
    float distance_threshold;
    int   model_type;
    int   method_type;
    bool  is_optimize_coefficients;

    SacModel_Plane(int _max_iterations, float _dist)
        : max_iterations{_max_iterations}
        , distance_threshold{_dist}
        , model_type{0}  // pcl::SACMODEL_PLANE
        , method_type{0} // pcl::SAC_RANSAC
        , is_optimize_coefficients{true}
    {
    }

    SacModel_Plane()
    {
      auto config = YAML::LoadFile(segmentation_config_path)["sac_model_plane"];

      max_iterations           = config["max_iterations"].as< int >();
      model_type               = config["model_type"].as< int >();  // pcl::SACMODEL_PLANE
      method_type              = config["method_type"].as< int >(); // pcl::SAC_RANSAC
      distance_threshold       = config["distance_threshold"].as< float >();
      is_optimize_coefficients = config["is_optimize_coefficients"].as< bool >();
    }

    template < typename SegmentationT >
    void setup(SegmentationT& s)
    {
      s.setMaxIterations(max_iterations);
      s.setOptimizeCoefficients(is_optimize_coefficients);
      s.setDistanceThreshold(distance_threshold);
      s.setModelType(model_type);
      s.setMethodType(method_type);
    }
  };

  struct SacModel_Plane_FromNormals : public SacModel_Plane {

    std::vector< float > axis;
    float                eps_angle;
    float                normal_distance_weight;

    SacModel_Plane_FromNormals(int                         _max_ierations,
                               float                       _dist,
                               const std::vector< float >& _axis,
                               float                       _eps_angle)
        : SacModel_Plane{_max_ierations, _dist}
        , axis{_axis}
        , eps_angle{_eps_angle}
    {
    }

    SacModel_Plane_FromNormals()
        : SacModel_Plane{}
    {
      auto config = YAML::LoadFile(segmentation_config_path)["sac_model_plane_from_normals"];

      axis                   = config["axis"].as< std::vector< float > >();
      eps_angle              = config["eps_angle"].as< float >();
      normal_distance_weight = config["normal_distance_weight"].as< float >();
    }

    template < typename SegmentationT >
    void setup(SegmentationT& s)
    {
      s.setMaxIterations(max_iterations);
      s.setOptimizeCoefficients(is_optimize_coefficients);
      s.setDistanceThreshold(distance_threshold);
      s.setModelType(model_type);
      s.setMethodType(method_type);
      s.setAxis(Eigen::Vector3f{axis[0], axis[1], axis[2]});
      s.setEpsAngle((eps_angle * M_PI) / 180.0f);
      s.setNormalDistanceWeight(normal_distance_weight);
    }
  };

  struct EuclideanClusterExtraction {

    int   min_cluster_size;
    int   max_cluster_size;
    float cluster_tolerance;

    EuclideanClusterExtraction(int   _min_cluster_size,
                               int   _max_cluster_size,
                               float _cluster_tolerance)
        : min_cluster_size{_min_cluster_size}
        , max_cluster_size{_max_cluster_size}
        , cluster_tolerance{_cluster_tolerance}
    {
    }

    EuclideanClusterExtraction()
    {
      auto config = YAML::LoadFile(segmentation_config_path)["euclidean_cluster_extraction"];

      min_cluster_size  = config["min_cluster_size"].as< int >();
      max_cluster_size  = config["max_cluster_size"].as< int >();
      cluster_tolerance = config["cluster_tolerance"].as< float >();
    }

    template < typename SegmentationT >
    void setup(SegmentationT& s)
    {
      s.setMinClusterSize(min_cluster_size);
      s.setMaxClusterSize(max_cluster_size);
      s.setClusterTolerance(cluster_tolerance);
    }
  };

  struct StatisticalOutlierRemoval {
    int   mean_k;
    float std_dev_mul_threshold;

    StatisticalOutlierRemoval(int _mean_k, float _std_dev_mul_thresold)
        : mean_k{_mean_k}
        , std_dev_mul_threshold{_std_dev_mul_thresold}
    {
    }

    StatisticalOutlierRemoval()
    {
      auto config           = YAML::LoadFile(filter_config_path)["statistical_outlier_removal"];
      mean_k                = config["mean_k"].as< int >();
      std_dev_mul_threshold = config["std_dev_mul_threshold"].as< float >();
    }

    template < typename FilterT >
    void setup(FilterT& f)
    {
      f.setMeanK(mean_k);
      f.setStddevMulThresh(std_dev_mul_threshold);
    }
  };

  struct RadiusOutlierRemoval {
    float radius_search;
    int   min_neighbors_in_radius;

    RadiusOutlierRemoval(float _radius_search, int _min_neightors_in_radius)
        : radius_search{_radius_search}
        , min_neighbors_in_radius(_min_neightors_in_radius)
    {
    }

    RadiusOutlierRemoval()
    {
      auto config             = YAML::LoadFile(filter_config_path)["radius_outlier_removal"];
      radius_search           = config["radius_search"].as< float >();
      min_neighbors_in_radius = config["min_neighbors_in_radius"].as< int >();
    }

    template < typename FilterT >
    void setup(FilterT& f)
    {
      f.setRadiusSearch(radius_search);
      f.setMinNeighborsInRadius(min_neighbors_in_radius);
    }
  };

  struct ProjectUsingParametricModel {
    int                       model_type;
    pcl::ModelCoefficientsPtr coefs;

    ProjectUsingParametricModel(int                              model_type,
                                const pcl::ModelCoefficientsPtr& coeffs)
        : model_type{model_type}
        , coefs(coeffs)
    {
    }

    ProjectUsingParametricModel() = default;

    template < typename FilterT >
    void setup(FilterT& f)
    {
      f.setModelType(model_type);
      f.setModelCoefficients(coefs);
    }
  };

  struct GreedyProjectionTriangulation {
    double search_radius;
    double mu;
    int    maximum_nearest_neighbors;
    double maximum_surface_angle;
    double minimum_angle;
    double maximum_angle;
    bool   normal_consistency;

    GreedyProjectionTriangulation()
    {
      auto config = YAML::LoadFile(surface_config_path)["greedy_projection_triangulation"];

      search_radius             = config["search_radius"].as< double >();
      mu                        = config["mu"].as< double >();
      maximum_nearest_neighbors = config["maximum_nearest_neighbors"].as< int >();
      maximum_surface_angle     = config["maximum_surface_angle"].as< double >() / 180.0 * M_PI;
      minimum_angle             = config["minimum_angle"].as< double >() / 180.0 * M_PI;
      maximum_angle             = config["maximum_angle"].as< double >() / 180.0 * M_PI;
      normal_consistency        = config["normal_consistency"].as< bool >();
    }

    template < typename SurfaceProcessingT >
    void setup(SurfaceProcessingT& s)
    {
      s.setSearchRadius(search_radius);
      s.setMu(mu);
      s.setMaximumNearestNeighbors(maximum_nearest_neighbors);
      s.setMaximumSurfaceAngle(maximum_angle); // 45 degrees
      s.setMinimumAngle(minimum_angle);        // 10 degrees
      s.setMaximumAngle(maximum_angle);        // 120 degrees
      s.setNormalConsistency(normal_consistency);
    }
  };
};

/* ---------------------------------------------------------------------------------------
 * Helper meta classes for selecting corresponding parameter type from given keypoint type.
 * --------------------------------------------------------------------------------------- */

/*
 *  Query parameters
 */
template < typename ParamT >
struct param_t;

template <>
struct param_t< KeypointType::Sift3d > {
  using type = Parameters::Sift3d;
};

template <>
struct param_t< KeypointType::Harris3d > {
  using type = Parameters::Harris3d;
};

template <>
struct param_t< KeypointType::Harris6d > {
  using type = Parameters::Harris6d;
};
template <>
struct param_t< SamplerType::DownSampling_VoxelGrid > {
  using type = Parameters::VoxelGridSampler;
};

template <>
struct param_t< SegmentationType::SacModelPlane > {
  using type = Parameters::SacModel_Plane;
};

template <>
struct param_t< SegmentationType::SacModelPlaneFromNormals > {
  using type = Parameters::SacModel_Plane_FromNormals;
};

template <>
struct param_t< SegmentationType::EuclideanClusterExtraction > {
  using type = Parameters::EuclideanClusterExtraction;
};

template <>
struct param_t< FilterType::StatisticalOutlierRemoval > {
  using type = Parameters::StatisticalOutlierRemoval;
};

template <>
struct param_t< FilterType::RadiusOutlierRemoval > {
  using type = Parameters::RadiusOutlierRemoval;
};

template <>
struct param_t< FilterType::ProjectUsingParametricModel > {
  using type = Parameters::ProjectUsingParametricModel;
};

template <>
struct param_t< ProcessingType::NormalEstimation > {
  using type = Parameters::NormalEstimation;
};

template <>
struct param_t< ProcessingType::MovingLeastSquares > {
  using type = Parameters::MovingLeastSquares;
};
template <>
struct param_t< SurfaceProcessingType::GreedyProjectionTriangulation > {
  using type = Parameters::GreedyProjectionTriangulation;
};
}

#endif // PARAMETERS_HPP
