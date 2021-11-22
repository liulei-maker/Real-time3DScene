#pragma once
#include "d2cSwapper.h"
#include "ObCommon.h"

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#define MIN_DISTANCE 200  //单位毫米
#define MAX_DISTANCE 2000 //单位毫米
#define RESOULTION_X 640.0  //标定时的分辨率
#define RESOULTION_Y 480.0  //标定时的分辨率
#define MAX_FRAME_COUNT 50

typedef struct xnIntrinsic_Params
{
	xnIntrinsic_Params() :
		c_x(320.0), c_y(240.0), f_x(480.0), f_y(480.0)
	{}

	xnIntrinsic_Params(float c_x_, float c_y_, float f_x_, float f_y_) :
		c_x(c_x_), c_y(c_y_), f_x(f_x_), f_y(f_y_)
	{}

	float c_x; //u轴上的归一化焦距
	float c_y; //v轴上的归一化焦距
	float f_x; //主点x坐标
	float f_y; //主点y坐标
}xIntrinsic_Params;


class AstraCameraD2C :public d2cSwapper
{
public:
	AstraCameraD2C();
	virtual ~AstraCameraD2C();

	//函数功能：相机初始化
	int CameraInit(int d2cType);

	//函数功能：相机反初始化
	int CameraUnInit(void);

	//函数功能：获取相机流数据
	int GetStreamData(cv::Mat &cv_rgb, cv::Mat &cv_depth);

	//函数功能：停止流
	int StreamStop(void);

	//函数功能: 获取相机的内外参
	//参数：
	//[out] cameraParam: 相机的内外参
	//返回值：0:表示OK; 非0表示获取参数失败
	int GetCameraParam(OBCameraParams &cameraParam);

	//函数功能：获取Depth分辨率
	//参数：
	//出参[Out] nImageWidth: 图像宽;
	//出参[Out] nImageHeight: 图像高;
	//返回值：成功返回 CAMERA_STATUS_SUCCESS，失败返回 CAMERA_STATUS_DEPTH_GET_RESOLUTION_FAIL
	int GetCameraResolution(int &nImageWidth, int &nImageHeight);

	//函数功能：获取设备的pid
	uint16_t GetDevicePid(void);

	void getCameraParams(openni::Device& Device, xIntrinsic_Params& IrParam);

	void convertDepthToWorld(const ushort& u, const ushort& v, ushort& d, float& worldX, float& worldY, float& worldZ);
private:

	/**** start depth swapper ****/
	int DepthInit(void);
	int DepthUnInit();

	int Depthstart(int width, int height);
	int Depthstop();
	int WaitDepthStream(VideoFrameRef &frame);
	//void CalcDepthHist(VideoFrameRef& frame);
	bool IsLunaDevice(void);

	//depth data
	Device m_device;

	xIntrinsic_Params m_IntrinsicParam; //存储相机内参的全局变量
	float fdx, fdy, u0, v0;

	VideoStream m_depthStream;
	//openni::VideoFrameRef m_depthFrame;
	bool m_bDepthInit;
	bool m_bDepthStart;

	bool m_bDepStreamCreate;


	float* m_histogram;
	int m_ImageWidth;
	int m_ImageHeight;

	/***end depth swapper********/

	/****start color swapper****/
	int ColorStart(int width, int height);
	int ColorStop();
	int WaitColorStream(VideoFrameRef &frame);

	VideoStream m_ColorStream;
	//openni::VideoFrameRef m_ColorFrame;
	bool m_bColorStart;
	bool m_bColorStreamCreate;
	/***end color swapper ******/

private:

	
};

