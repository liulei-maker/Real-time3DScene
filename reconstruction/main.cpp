/****************************************************************************
*----------------------------------------------------------------------------*
*  projectname   : 3D reconstruction                                         *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>      | <Description>                    *
*----------------------------------------------------------------------------*
*  2021/11/15 | 1.0.0     |  liulei       | 3D reconstruction base on Open3D *
*----------------------------------------------------------------------------*
*****************************************************************************/
#include <vector>
#include "Reconstruction.h"
#include "SerialPort.h"
#include "MagCamera.h"
#include "AstraCameraD2C.h"

/*opencv图像显示头文件*/
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;


#define ISHAVE_SERIAL_ROBOT     0
#define ISHAVE_SERIAL_LASER     0
#define ISHAVE_THERMALCAMERA    0
#define ISHAVE_ASTRACAMERA      1
#define ISHAVE_CMANERA          0

using namespace cv;
using namespace open3d;

int main(int argc, char* argv[])
{
    
    MagCamera   *myMagCamera = new MagCamera;
    SerialPort  *robotSerial = new SerialPort;
	AstraCameraD2C* astraCameraD2C = new AstraCameraD2C();

	std::shared_ptr<geometry::PointCloud> pointcloud_ptr(new geometry::PointCloud);
	open3d::visualization::Visualizer visualizer; //创建一个可视化对象
	visualizer.CreateVisualizerWindow("open3d", 640, 480, 50, 50, true); //创建一个窗口
	visualizer.GetRenderOption().point_size_ = 1;//设置点云点的大小
	visualizer.GetRenderOption().background_color_ = Eigen::Vector3d(0, 0, 0);//设置窗口背景色
	visualizer.GetRenderOption().show_coordinate_frame_ = true;
	bool flag = 0;

#if ISHAVE_THERMALCAMERA
    if (!myMagCamera->InitialMagCamera("192.168.1.160"))
        return -1;
#endif // ISHAVE_THERMALCAMERA

#if ISHAVE_ASTRACAMERA
	if (astraCameraD2C->CameraInit(HARDWARE_D2C) != CAMERA_STATUS_SUCCESS)
	{
		printf("camera init failed\n");
		return -1;
	}
#endif // ISHAVE_ASTRACAMERA

#if ISHAVE_SERIAL_ROBOT
    if (!robotSerial->SerialOpen("COM3", 115200))
        return -1;
#endif // ISHAVE_SERIAL_ROBOT

#if ISHAVE_CMANERA 
    if (!capture.isOpened())
        return -1;
#endif // ISHAVE_CMANERA

	while (true)
	{
		cv::Mat colorImage(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_8UC3);
		cv::Mat depthImage(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_16UC1);//640x480

		if (astraCameraD2C->GetStreamData(colorImage, depthImage) == CAMERA_STATUS_SUCCESS)
		{
			imshow("color", colorImage);
			for (size_t i = 0; i < 480; i++)
			{
				for (size_t j = 0; j < 640; j++)
				{
					Eigen::Vector3d pointTemp;
					Eigen::Vector3d colorTemp;

					ushort depthNum = depthImage.at<ushort>(i, j);

					if (depthNum > MIN_DISTANCE && depthNum < MAX_DISTANCE)
					{
						float worldX, worldY, worldZ;
						astraCameraD2C->convertDepthToWorld(j, i, depthNum, worldX, worldY, worldZ);
						pointTemp[0] = -worldX;
						pointTemp[1] = worldY;
						pointTemp[2] = worldZ;

						colorTemp[0] = colorImage.at<cv::Vec3b>(i, j)[2] / 255.0;
						colorTemp[1] = colorImage.at<cv::Vec3b>(i, j)[1] / 255.0;
						colorTemp[2] = colorImage.at<cv::Vec3b>(i, j)[0] / 255.0;
					}
					pointcloud_ptr->points_.push_back(pointTemp);
					pointcloud_ptr->colors_.push_back(colorTemp);
				}
			}

			if (!flag)
			{
				flag = 1;
				visualizer.AddGeometry(pointcloud_ptr);
			}

			visualizer.UpdateGeometry(pointcloud_ptr);//更新显示
			visualizer.PollEvents(); //下头这两个不清楚啥用
			visualizer.UpdateRender();

			colorImage.release();
			depthImage.release();

			pointcloud_ptr->points_.clear();
			pointcloud_ptr->colors_.clear();

			cv::waitKey(10);
		}
	}
    return 0;
}