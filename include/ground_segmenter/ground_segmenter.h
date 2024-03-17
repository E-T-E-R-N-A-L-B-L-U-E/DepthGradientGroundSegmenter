//
// created by linxif2008 on 03/18/2024
//

#ifndef GROUND_SEGMENTER_H
#define GROUND_SEGMENTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

template<typename PointT>
class GroundSegmenter
{
private:
    typename::pcl::PointCloud<PointT>::Ptr _input_cloud;
    float _grid_size;               // (m)
    float _min_height, _max_height; // (m)

    void voxelFilter(const typename::pcl::PointCloud<PointT>::Ptr &input_cloud, typename::pcl::PointCloud<PointT>::Ptr &output_cloud) const;
    void passthroughFilter(const typename::pcl::PointCloud<PointT>::Ptr &input_cloud, typename::pcl::PointCloud<PointT>::Ptr &output_cloud) const;
public:
    GroundSegmenter();

    void setInputCloud(const typename::pcl::PointCloud<PointT>::Ptr &input_cloud);
    void setGridSize(const float &grid_size);
    void setMinMaxHeight(const float &min_height, const float &max_height);
    cv::Mat toGridMap(const float &resolution, const bool &bird_view_flag = true) const;

};

#include "ground_segmenter.tpp"


#endif // GROUND_SEGMENTER_H