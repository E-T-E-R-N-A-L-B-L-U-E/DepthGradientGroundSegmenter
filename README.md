# Depth Gradient Ground Segmenter

## Description
This project realized a ground point cloud segmenter based on depth gradient.

The motivation of this project is the poor behavior of the ground segmentation algorithm in RM2023 Sentry code. In RM2023 Sentry code, we use [line fit ground segmentation](https://github.com/HuangCongQing/linefit_ground_segmentation_details/tree/main/linefit_ground_segmentation), which got lots of wrong segmentations when the lidar is scanning on a rotating sentry gimbal.

The concept of this project came from  HIT's  presentation in RM2023 Young Engineers Conference. In the conference, they metioned that the depth gradient reveals the slop of the ground. Place with low slope should be considered as the ground.

