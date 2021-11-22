#pragma once
#include "d2cSwapper.h"
#include "ObCommon.h"

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#define MIN_DISTANCE 200  //��λ����
#define MAX_DISTANCE 2000 //��λ����
#define RESOULTION_X 640.0  //�궨ʱ�ķֱ���
#define RESOULTION_Y 480.0  //�궨ʱ�ķֱ���
#define MAX_FRAME_COUNT 50

typedef struct xnIntrinsic_Params
{
	xnIntrinsic_Params() :
		c_x(320.0), c_y(240.0), f_x(480.0), f_y(480.0)
	{}

	xnIntrinsic_Params(float c_x_, float c_y_, float f_x_, float f_y_) :
		c_x(c_x_), c_y(c_y_), f_x(f_x_), f_y(f_y_)
	{}

	float c_x; //u���ϵĹ�һ������
	float c_y; //v���ϵĹ�һ������
	float f_x; //����x����
	float f_y; //����y����
}xIntrinsic_Params;


class AstraCameraD2C :public d2cSwapper
{
public:
	AstraCameraD2C();
	virtual ~AstraCameraD2C();

	//�������ܣ������ʼ��
	int CameraInit(int d2cType);

	//�������ܣ��������ʼ��
	int CameraUnInit(void);

	//�������ܣ���ȡ���������
	int GetStreamData(cv::Mat &cv_rgb, cv::Mat &cv_depth);

	//�������ܣ�ֹͣ��
	int StreamStop(void);

	//��������: ��ȡ����������
	//������
	//[out] cameraParam: ����������
	//����ֵ��0:��ʾOK; ��0��ʾ��ȡ����ʧ��
	int GetCameraParam(OBCameraParams &cameraParam);

	//�������ܣ���ȡDepth�ֱ���
	//������
	//����[Out] nImageWidth: ͼ���;
	//����[Out] nImageHeight: ͼ���;
	//����ֵ���ɹ����� CAMERA_STATUS_SUCCESS��ʧ�ܷ��� CAMERA_STATUS_DEPTH_GET_RESOLUTION_FAIL
	int GetCameraResolution(int &nImageWidth, int &nImageHeight);

	//�������ܣ���ȡ�豸��pid
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

	xIntrinsic_Params m_IntrinsicParam; //�洢����ڲε�ȫ�ֱ���
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

