#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>

#include "ground_segmenter/ground_segmenter.h"


int main(int argc, char** argv)
{
    std::string pcd_location = PROJECT_DIR"/map/map_game1_clean.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPCDFile(pcd_location, *input_cloud) == -1)
    {
        std::cerr << "failed to load: " << pcd_location << std::endl;
        exit(0);
    }


    Eigen::Matrix4f rotate_matrix(4,4);
    rotate_matrix << 
            0.866, 0, 0.5, 0,
            0, 1, 0, 0,
            -0.5, 0, 0.866, 0,
            0, 0, 0, 1;
    pcl::transformPointCloud(*input_cloud, *input_cloud, rotate_matrix);

    GroundSegmenter<pcl::PointXYZ> ground_segmenter;
    ground_segmenter.setInputCloud(input_cloud);
    ground_segmenter.setGridSize(0.1);
    ground_segmenter.setMinMaxHeight(-1, 2);
    
    cv::Mat depth = ground_segmenter.toDepth(0.05, true);
    float min_depth = 1e10, max_depth = -1e10;
    for (int i = 0; i < depth.rows; i++)
        for (int j = 0; j < depth.cols; j++)
        {
            min_depth = std::fmin(min_depth, depth.at<float>(i, j));
            max_depth = std::fmax(max_depth, depth.at<float>(i, j));
        }
    // std::cout << "max_depth: " << max_depth << std::endl;
    // std::cout << "min_depth: " << min_depth << std::endl;
    
    depth = (depth - min_depth) / (max_depth - min_depth) * 255;
    cv::Mat depth2show;
    depth.convertTo(depth2show, CV_8UC1);

    // int thres = cv::threshold(depth2show, depth2show, 120, 0, cv::THRESH_TOZERO_INV);
    // std::cout << "thres: " << thres << std::endl;
    cv::imshow("depth", depth2show);
    cv::waitKey(0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    auto t1 = std::chrono::system_clock::now();
    ground_segmenter.groundSegment(output_cloud, 30.f);
    auto t2 = std::chrono::system_clock::now();
    std::cout << "segmentation time cost(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;

    // visualization
    pcl::visualization::PCLVisualizer vis("vis");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(output_cloud, 255.0, 0.0, 0.0);
    vis.addPointCloud(output_cloud, target_handler, "target");
    vis.spin();
    vis.close();
    return 0;
}