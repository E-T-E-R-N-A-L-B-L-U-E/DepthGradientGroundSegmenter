#include "ground_segmenter/ground_segmenter.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include <cmath>
#include <assert.h>

template<typename PointT>
GroundSegmenter<PointT>::GroundSegmenter()
    :_grid_size(-1.f), _min_height(-2.), _max_height(5.)
{
    
}

template<typename PointT>
void GroundSegmenter<PointT>::setInputCloud(const typename::pcl::PointCloud<PointT>::Ptr &input_cloud)
{
    this->_input_cloud = input_cloud;
}


template<typename PointT>
void GroundSegmenter<PointT>::setGridSize(const float &grid_size)
{
    this->_grid_size = grid_size;
}

template<typename PointT>
void GroundSegmenter<PointT>::setMinMaxHeight(const float &min_height, const float &max_height)
{
    _min_height = min_height;
    _max_height = max_height;
}

template<typename PointT>
void GroundSegmenter<PointT>::voxelFilter(const typename::pcl::PointCloud<PointT>::Ptr &input_cloud, typename::pcl::PointCloud<PointT>::Ptr &output_cloud) const
{
    typename::pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(_grid_size, _grid_size, _grid_size);
    voxel_filter.filter(*output_cloud);
}

template<typename PointT>
void GroundSegmenter<PointT>::passthroughFilter(const typename::pcl::PointCloud<PointT>::Ptr &input_cloud, typename::pcl::PointCloud<PointT>::Ptr &output_cloud) const
{
    typename::pcl::PassThrough<PointT> passthrought_filter;
    passthrought_filter.setInputCloud(input_cloud);
    passthrought_filter.setFilterFieldName("z");
    passthrought_filter.setFilterLimits(_min_height, _max_height);
    passthrought_filter.setNegative(false);
    passthrought_filter.filter(*output_cloud);
}

template<typename PointT>
cv::Mat GroundSegmenter<PointT>::toDepth(const float &resolution, const bool &bird_view_flag) const
{
    typename::pcl::PointCloud<PointT>::Ptr filtered_pcd(new typename::pcl::PointCloud<PointT>());

    if (this->_grid_size > 0)
    {
        voxelFilter(this->_input_cloud, filtered_pcd);
        passthroughFilter(filtered_pcd, filtered_pcd);
    } else
    {
        passthroughFilter(this->_input_cloud, filtered_pcd);
    }

    float x_min, x_max, y_min, y_max, z_select;
    auto select = (bird_view_flag ? (std::fmin<float, float>) : (std::fmax<float, float>));
    for (int i = 0; i < filtered_pcd->points.size(); i++)
    {
        if (i == 0)
        {
            x_min = x_max = filtered_pcd->points[i].x;
            y_min = y_max = filtered_pcd->points[i].y;
            z_select = filtered_pcd->points[i].z;
        } else
        {
            x_min = std::fmin(x_min, filtered_pcd->points[i].x);
            x_max = std::fmax(x_max, filtered_pcd->points[i].x);
            y_min = std::fmin(y_min, filtered_pcd->points[i].y);
            y_max = std::fmax(y_max, filtered_pcd->points[i].y);
            z_select = select(z_select, filtered_pcd->points[i].z);
        }
    }

    select = (bird_view_flag ? (std::fmax<float, float>) : (std::fmin<float, float>));
    cv::Mat grid_map = z_select * cv::Mat::ones(static_cast<int>((y_max - y_min) / resolution) + 1, static_cast<int>((x_max - x_min) / resolution) + 1, CV_32FC1);
    for (int i = 0; i < filtered_pcd->points.size(); i++)
    {
        grid_map.at<float>(static_cast<int>((filtered_pcd->points[i].y - y_min) / resolution), static_cast<int>((filtered_pcd->points[i].x - x_min) / resolution))
            = select(filtered_pcd->points[i].z, grid_map.at<float>(static_cast<int>((filtered_pcd->points[i].y - y_min) / resolution), static_cast<int>((filtered_pcd->points[i].x - x_min) / resolution)));
    }
    return grid_map;
}

template<typename PointT>
void GroundSegmenter<PointT>::groundSegment(typename::pcl::PointCloud<PointT>::Ptr &output_cloud, const float pass_angle) const
{
    assert(pass_angle >= 0);
    typename::pcl::PointCloud<PointT>::Ptr filtered_pcd(new typename::pcl::PointCloud<PointT>());

    if (this->_grid_size > 0)
    {
        voxelFilter(this->_input_cloud, filtered_pcd);
        passthroughFilter(filtered_pcd, filtered_pcd);
    } else
    {
        passthroughFilter(this->_input_cloud, filtered_pcd);
    }

    typename::pcl::NormalEstimation<PointT, pcl::Normal> normal;
    typename::pcl::PointCloud<pcl::Normal>::Ptr pcd_normals(new typename::pcl::PointCloud<pcl::Normal>());
    typename::pcl::search::KdTree<PointT>::Ptr kd_tree(new typename::pcl::search::KdTree<PointT>());

    normal.setInputCloud(filtered_pcd);
    normal.setSearchMethod(kd_tree);
    normal.setKSearch(10);
    // normal.setRadiusSearch(0.5);
    normal.compute(*pcd_normals);

    output_cloud->clear();
    for (int i = 0; i < filtered_pcd->points.size(); i++)
    {
        typename::pcl::Normal norm = pcd_normals->points[i];
        float norm_len = std::sqrt(norm.normal_x * norm.normal_x + norm.normal_y * norm.normal_y + norm.normal_z * norm.normal_z);
        if (std::fabs(norm.normal_z / norm_len) < std::sin(pass_angle / 180.F * M_PI))
            output_cloud->push_back(filtered_pcd->points[i]);
    }
}