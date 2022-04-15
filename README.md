# Real-time3DScene

## show Real-time 3D reconstruction of scene

2021.11.22

language:C++

## environment:
* open3D_0.10
* opencv3.4
* ASTRA SDK

In this project, ASTRA Depth Camera was used to collect colorImage and depthImage, and the point cloud coordinate data in camera coordinate system was obtained by calculating the internal parameters of the camera. Opencv34 and Open3D_0.10 were used for real-time 3D display.

Reconstruction using open3D is also included in the project, and data sets can be downloaded from the Redwood 3D open source dataset.

##主要步骤
* 制作片段：从输入RGBD序列的短子序列构建局部几何表面（称为片段）；
* 注册片段：片段在全局空间中对齐以检测闭环。本部分使用全局注册、ICP注册、Multiway注册；
* 精细配准：使注册片段后更加紧密对齐，这部分使用ICP注册和Multiway注册；
* 场景整合：整合RGB-D图像以生成场景的网络模型。<br>
![2831c06b60f04413b78fb0921bdf6ede](https://user-images.githubusercontent.com/54426524/163553108-42cbf18b-6d4a-47d4-8010-7e5879611d80.png)
