#ifndef TEST_HELPER_HPP
#define TEST_HELPER_HPP

#include "../types.hpp"
#include "../util.hpp"
#include <yaml-cpp/yaml.h>

namespace {
const std::string path_file = "config/test_data_path.yaml";
}

namespace pcl_wrapper_tests {

template < typename InPointT >
pcl_wrapper::OutPointCloudPtr< InPointT > read_cloud()
{
    using namespace pcl_wrapper;

    auto node  = YAML::LoadFile(path_file);
    auto path  = node["path"].as< std::string >();
    auto cloud = IO::Read< InPointT >(path);

    cloud->points.resize(cloud->width * cloud->height);
    auto size = cloud->width * cloud->height;
    for (size_t i = 0; i < size; ++i) {
        if (not std::isnan(cloud->points[i].z)) {
            cloud->points[i].x /= 1000.0f;
            cloud->points[i].y /= 1000.0f;
            cloud->points[i].z /= 1000.0f;
        }
    }

    return cloud;
}

template < typename InPointT >
void show_all_segments(const std::string&                                     window_name,
                       pcl_wrapper::PointCloudPtr< InPointT >&                cloud,
                       std::vector< pcl_wrapper::PointCloudPtr< InPointT > >& segments)
{
    std::string cloud_name{ "initial_cloud" };
    using namespace pcl_wrapper;
    std::shared_ptr< pcl::visualization::PCLVisualizer >
        viewer{ new pcl::visualization::PCLVisualizer{ window_name.c_str() } };
    pcl::visualization::PointCloudColorHandlerRGBField< Point6D > rgb(cloud);
    viewer->setBackgroundColor(0.8, 0.8, 0.8);
    viewer->addPointCloud< Point6D >(cloud, rgb, cloud_name.c_str());

    for (size_t i = 0; i < segments.size(); ++i) {

        std::stringstream ss;
        ss << "segments" << i;

        auto r = static_cast< float >(rand()) / static_cast< float >(RAND_MAX);
        auto g = static_cast< float >(rand()) / static_cast< float >(RAND_MAX);
        auto b = static_cast< float >(rand()) / static_cast< float >(RAND_MAX);

        auto plane_name = ss.str().c_str();

        viewer->addPointCloud< Point6D >(segments[i], plane_name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, plane_name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, plane_name);
    }

    viewer->spin();
    viewer->close();
}

void debug_info(const std::string& title, int index = 0, int indent_level = 0)
{
    std::cerr << " - [DEBUG] (" << std::to_string(index) << ") ";
    for (int i = 0; i < indent_level; ++i)
        std::cerr << "  ";
    std::cerr << title << std::endl;
}
}

#endif // TEST_HELPER_HPP
