#ifndef _TYPEDEF_H_
#define  _TYPEDEF_H_

//linux�²���APIδʵ�֣������ú� #define MAG_Linux_Not_Implemented
//��C++�������û��ʵ�ֵĺ�������MAG_LinkCameraEx()���䵽MAG_LinkCamera()֮�⣬���ຯ�����þ�ֱ��return

//#define MAG_Linux_Not_Implemented

#ifndef u_long
typedef unsigned long u_long;
#endif

#ifndef UINT
typedef unsigned int UINT;
#endif

#ifndef DWORD
typedef unsigned long DWORD;
#endif

#ifndef BOOL
typedef int BOOL;
#endif

#ifndef USHORT
typedef unsigned short USHORT;
#endif

#ifndef BYTE
typedef unsigned char BYTE;
#endif

#ifndef UCHAR
typedef unsigned char UCHAR;
#endif

#ifndef BOOL
typedef int BOOL;
#endif

#ifndef LONG
typedef long LONG;
#endif

#ifndef WORD
typedef unsigned short WORD;
#endif

#define STREAM_TEMPERATURE	(2)
#define STREAM_VIDEO		(4)
#define STREAM_HYBRID		(STREAM_TEMPERATURE | STREAM_VIDEO)

enum enumInputIo
{
	IoFFC=0,
	IoCaptureMGT,
	IoCaptureBMP,
	IoCustom,
};

enum enumAnalogPlot
{
	AnalogPlotNone=0,
	AnalogPlotCenterCross,
	AnalogPlotMaxTemperature,
	AnalogPlotROI,
};

enum TV_STANDARD
{
    NTSC_M = 0,
    NTSC_J,
    PAL_BDGHI,
    PAL_M,
    PAL_NC
};

enum EX
{
	E1X = 0,
	E2X,
	E4X,
};

enum ColorPalette
{
	Gray0to255=0,//����
	Gray255to0=1,//����
	IronBow=2,//����
	RainBow=3,//�ʺ�
	GlowBow=4,//����
	Autumn=5,//����
	Winter=6,//����
	HotMetal=7,//�Ƚ���
	Jet=8,//����
	RedSaturation=9,//�챥��
	HighContrast=10,//�߶Աȶ�
	Medicine=11,
};

enum PTZProtocol
{
	ProtocolPelcoD = 0,//Pelco_DЭ��
	ProtocolPelcoP,//Pelco_PЭ��
};

enum PTZCmd
{
	PTZStop = 0,//��λ��Ԥ��λֹͣ ����Ϊ0
	PTZRight = 1,//��λ ����Ϊ�˶��ٶ�0~63
	PTZLeft,
	PTZUp,
	PTZDown,
	PTZUpRight = 5,
	PTZUpLeft,
	PTZDownRight,
	PTZDownLeft,
	PTZSetPreset = 9,//Ԥ��λ ����ΪԤ��λ���0~255
	PTZCallPreset,
	PTZClearPreset,
	PTZSetAuxiliary = 12,//�������� ����Ϊ�������ر��0~255
	PTZClearAuxiliary,
	PTZZoomStop = 14,//��ͷֹͣ�䱶 ����Ϊ0
	PTZZoomIn,//��ͷ�Ŵ� ����Ϊ����˶�ʱ��
	PTZZoomOut,//��ͷ��С
	PTZFocusStop = 17,//��ͷֹͣ���� ����Ϊ0
	PTZFocusAuto,//��ͷ�Զ��Խ� ����Ϊ0
	PTZFocusFar,//��ͷ��Զ ����Ϊ����˶�ʱ��
	PTZFocusNear,//��ͷ����
	PTZFocusGoto,//��ͷ��������λ�� ����Ϊ����λ��
};

enum PTZQuery
{
	PTZQueryPan = 0,//��ѯ��̨���½Ƕ�
	PTZQueryTilt,//��ѯ��̨���ҽǶ�
	PTZQueryZoomPosition,//��ѯzoom����λ��
	PTZQueryZoomState,//��ѯ�Ƿ�����zoom
	PTZQueryFocusPosition,//��ѯfocus����λ��
	PTZQueryFocusState,//��ѯ�Ƿ�����ִ���Զ��Խ�
};

enum SDStorageFileType
{
	SDFileBMP = 0,
	SDFileJPG,
	SDFileDDT,
	SDFileMDT,
	SDFileMGT,
	SDFileAVI,
	SDFileMGS,
};

enum enumDisplayMode {
	DmIrOnly = 0,
	DmVisOnly,
	DmPipCenter,
	DmPipTopLeft,
	DmPipTopRight,
	DmPipBottomLeft,
	DmPipBottomRight,
	DmFused0p25,
	DmFused0p50,
	DmFused0p75,
};

typedef struct 
{
	char	charTerminalName[32];//����������, �ɰ��������ַ�
	UINT	intVersion;//Э��汾��
	u_long	intTerminalIp;//���ip, ���ֽڱ�ʾip�����һ��
	u_long	intControllerIp;//���ƶ�ip, ���ֽڱ�ʾip�����һ��
	BYTE	charCameraMAC[6];//���MAC��ַ
	char	pad0[2];
	DWORD	pad1[2];
}struct_TerminalList;


#define CAMNAME_PROTOCOLLEN	(32)
#define TYPENAMELEN		(8)

typedef struct 
{
	DWORD pad0;
	
	char charName[CAMNAME_PROTOCOLLEN];//����������

	BOOL bUseStaticIp;//�Ƿ�ʹ�þ�̬ip
	DWORD dwStaticIp;//��̬ip(��ʹ��)����̫��˳��
	DWORD dwStaticNetMask; //��̬ip����������(��ʹ��)����̫��˳��

	BOOL bMulticastImg;//ͼ�������Ƿ��鲥
	DWORD dwMulticastIp;//ͼ���鲥Ŀ��ip(��ʹ��)����̫��˳��

	DWORD pad1[16];

	DWORD dwSN;//���к�, read only

	DWORD pad2[2];

	DWORD dwStaticGateWay;//��̬����

	DWORD pad3[8];

	UINT intCurrentLensIndex;//��ǰ��ͷ���, 0~2
	
	DWORD pad4[2];

	UINT intFFCFrameTrigger;//FFC֡������������λ֡
	UINT intFFCTemperatureTrigger;//FFC�¶ȴ���������λmC

	DWORD pad5;

	UINT intAccResponse;//��֡�ۼӣ���λ֡, 1,2,4,8,16

	DWORD pad6[3];

	UINT intInputIoFunction;//����Io����, 0-FFC, 1-MGT, 2-BMP,3-�Զ���

	//ģ����Ƶ���ƣ���Ӱ��PC���Ͽ�����Ч��
	ColorPalette intPaletteIndex;//��ɫ��
	BOOL bColorBar;//�Ƿ������ɫ��
	BOOL bSubSectionEnlarge;//�Ƿ�ʹ���˹��ֶλҶ�����
	int intEnlargeX1;//�˹�����ڵ�X1, ��λmC
	int intEnlargeX2;//�˹�����ڵ�X2, ��λmC
	UINT byteEnlargeY1;//�˹�����ڵ�Y1, ��λΪ�Ҷȣ���Χ0~254
	UINT byteEnlargeY2;	//�˹�����ڵ�Y2, ��λΪ�Ҷȣ���Χ1~255
	UINT intAutoEnlargeRange;//�Զ�������С�¶ȷ�Χ����λC���˹��ֶλҶ�������Զ��������ȣ��������2�����鲻С��5
	
	//IO
	UINT intAnalogPlot;//ģ����Ƶ���, 0-��,1-����,2-����,3-���¶���
	int	intAlarmTemp;//IO�����¶�

	//TV standard
	TV_STANDARD intTVStandard;//ģ����Ƶ��ʽ

	BOOL bCheckHeartBeat;//�Ƿ������������
	BOOL bAlwaysAnalogOutput;//�Ƿ�ʼ�����ģ����Ƶ����������̫������ͼ��ʱģ����Ƶ�رգ��Խ��͹���
	
	DWORD pad8[3];

	EX EXLevel;//ģ����Ƶ���ӱ���

	DWORD dwPartnerVisibleIp;//DTXX��Ŀɼ������IP��MAGXXδ����

	UINT intDDE;//ģ����ƵDDEǿ������, 0~32

	DWORD pad9;

	DWORD dwSerialBaudRate;//���ڲ�����
	DWORD dwSerialFeature;//���ڲ�������ʽΪ (ʹ��<<24) | (����λ<<16) | (ֹͣλ<<8) | У��λ������ֹͣλ 0-1, 1-1.5, 2-2��У��λ0-None, 1-Odd, 2-Even, 3-Mark, 4-Space
}struct_CeRegContent;


typedef struct 
{
	float fDistance;//Ŀ�����, ��λm, ��Χ(2*fFocalLength, ��)
	float fEmissivity;//ȫ��ȱʡ�����ʣ���Χ(0,1]
	float fTemp;//����, ��λC
	float fRH;//���ʪ�ȣ���Χ(0~1)
	float fVisDistance;//�ܼ���, ��λkm, ��Χ(0, ��)
	float fRain;//����ǿ��, ��λmm/h, ��Χ(0, ��)
	float fSnow;//��ѩǿ��, ��λmm/h, ��Χ(0, ��)
	float fExtrapara1;//��������1 ��ͷ������
	float fExtrapara2;//��������2 ��ͷ������
	float fTaoAtm;//����͸����, ֻ��
	float fTaoFilter;//(�������ⲿ��)����/�˹�Ƭ͸����
}struct_FixPara;

typedef struct 
{
	DWORD pad0[6];
	int intMaxTemperature;//ȫ��������¶�, ��λmC, δ������
	int intMinTemperature;//ȫ��������¶�, ��λmC, δ������
	int intAveTemperature;//ȫ����ƽ���¶�, ��λmC, δ������
	int intSTDTemperature;//ȫ�����¶ȱ�׼��, ��λmC������ֵ�Բ����ͺŵĸ��������ǲ�����
	UINT intPosMax;//����¶ȳ���λ��, y=int(intPosMax/FPAWIDTH), x=intPosMax-FPAWIDTH*y, ԭ����ͼ�����½�
	UINT intPosMin;//����¶ȳ���λ��, y=int(intPosMin/FPAWIDTH), x=intPosMin-FPAWIDTH*y, ԭ����ͼ�����½�
	DWORD pad1[3];
	UINT intAveNETDt;//ȫ�����¶�ʱ���׼��, ��λmC, ���������¶ȱ仯����������������֣�����ֵ�Բ����ͺŵĸ��������ǲ�����
	DWORD pad2[5];
	UINT intHistTemperature[256];//����ֱ��ͼ
}struct_State;

typedef struct 
{
	DWORD dwFPAWidth;//FPA��ȣ�����ΪFPAWIDTH
	DWORD dwFPAHeight;//FPA�߶ȣ�����ΪFPAHEIGHT
	DWORD dwBMPWidth;//���ͼ����, ����Ϊ4��������, �Ҳ�С��FPA�Ŀ��(FPAWIDTH)
	DWORD dwBMPHeight;//���ͼ��߶�, ��С��FPA�ĸ߶�(FPAHEIGHT)
	DWORD dwColorBarWidth;//��ɫ��ͼ����, ����Ϊ4��������, ��СΪ4
	DWORD dwColorBarHeight;//��ɫ��ͼ��߶�, ��СΪ1
}OutputPara;

typedef struct 
{
	UINT intFPAWidth;//̽��������
	UINT intFPAHeight;

	DWORD pad[2];

	char charName[CAMNAME_PROTOCOLLEN];
	char charType[TYPENAMELEN];

	UINT intMaxFPS;//���ͺŵ����֡��
	UINT intCurrentFPS;//��ǰʵ�����֡��

	UINT intVideoWidth;//(��HDMI, H.264��MPEG�����)������Ƶ����
	UINT intVideoHeight;
}struct_CamInfo;

typedef struct 
{
	BYTE StartHour;
	BYTE StartMinute;
	BYTE EndHour;
	BYTE EndMinute;
}struct_TimeUnit;

enum enumHDMISpec
{
    Hdmi480P50Hz = 0,
    Hdmi480P60Hz,
    Hdmi576P50Hz,
    Hdmi576P60Hz,
    Hdmi720P50Hz,
    Hdmi720P60Hz,
    Hdmi1080P50Hz,
    Hdmi1080P60Hz,
    Hdmi1080P30Hz,
};

typedef struct 
{
    struct_CamInfo BaseInfo;

    UINT intCameraSN;//���������к�

    int intCamTemperature[4];//����¶ȣ�̽�����¶ȣ�����������

    char charLensName[32];//��ʹ�õľ�ͷ��
    float fFocalLength;//m
    int	intCaliBlackbodyRange[2];//�궨�����¶ȷ�Χ, mC

    DWORD dwReserved0[10];

    __int64 timeCurrent;//֡����ʱ��, 64bit time_t
	
    double dblLatitude;			//γ��, �ȣ���γΪ��
    double dblLongitude;		//����, �ȣ�����Ϊ��
    float fAltitude;			//�߶�, m

    int intPaletteIndex;            //�û����õĵ�ɫ�����
    int intTempUnit;				//�û����õ��¶ȵ�λ��0: ���϶ȣ�1�����϶�
    float fEmissivity;              //�û����õķ�����
    float fEnvTemp;                 //�û����õĻ����ͷ����¶�
    float fTaoAtm;                  //�û����õĴ���͸����
    float fTaoFilter;               //�û����õ�(�������ⲿ��)����͸����
    float fObjDist;                 //�û����õ�Ŀ����� m
    BOOL bSubSectionEnlarge;		//�Ƿ����÷ֶ�����
    int intEnlargeX1;               //�ֶ����������t1, mC
    int intEnlargeX2;               //�ֶ����������t2, mC
    UINT byteEnlargeY1;             //�ֶ����������gray1
    UINT byteEnlargeY2;				//�ֶ����������gray2
    UINT intAutoEnlargeRange;       //�Զ����췶Χ��C
    int intBrightOffset;            //�Զ���������΢����-100~100
    int intContrastOffset;          //�Զ�����Աȶ�΢����-100~100

    DWORD dwReserved1[32];
}struct_CamInfoEx;

typedef struct //�����������ǽṹ��
{
	UINT intSize;//���ṹ��ߴ�

	char charName[CAMNAME_PROTOCOLLEN];//����������
	
	DWORD dwReserved[12];

	DWORD dwSN;//���к� ReadOnly

	DWORD dwFlip;//0-none, 1-H flip, 2-V flip, 3-Both
	DWORD dwRotate;//0-none, 1-��ת90��

	DWORD dwStabilizer;//���񿪹� 0-none, 1-H, 2-V, 3-Both, 5-H test, 6-V test

	BOOL bUseStaticIp;//�Ƿ�ʹ�þ�̬ip
	DWORD dwStaticIp;//��̬����
	DWORD dwStaticNetMask;//��̬��������
	DWORD dwStaticGateWay;//��̬����
	DWORD dwStaticDNS;//ָ��DNS

	DWORD dwReserved3[4];

	BOOL bCheckHeartBeat;//�Ƿ������������

	DWORD dwPartnerVisibleIp;//��Ŀɼ������IP

	BOOL bCloudEnable;//�����Ƽ��
	DWORD dwCloudServerIp;//�Ʒ�����IP

	BOOL bNatEnable;//���ö˿�ӳ��
	DWORD dwNatIp;//�˿�ӳ��������IP
	DWORD dwRtspNatPort;//rtsp�˿�ӳ��������Port

	//IO
	enum enumInputIo InputIoFunction;//����Io����

	DWORD dwReserved5[1];

	//Alarm
	int	intAlarmTemp;//IO����ȫ���¶ȣ�������V3T

	UINT intMotionDetectSensitivity;//�ƶ����������0~10, 0Ϊ�ر�
	UINT intMotionDetectX[2];//�ƶ���ⷶΧ
	UINT intMotionDetectY[2];//�ƶ���ⷶΧ

	DWORD dwReserved6[1];

	UINT intSunArea;//Ԥ�ڵ�̫�����(pixels)

	UINT intPeopleCountingLinePos;//0-disable, 1-1/8, 2-2/8 ...
	UINT intPeopleCountingObjSize;//����ͳ��Ŀ�����ֱ��(pixels)

	//Digital Output
	UINT intEncoder;//������Ƶ���뷽ʽ
	UINT intBitRateKBPS;//����
	UINT intIFrame;//I֡���

	enum enumHDMISpec HdmiSpec;//HDMI specification

	enum enumDisplayMode DisplayMode;

	DWORD dwReserved7[3];


	//Analog Output	
	UINT intPaletteIndex;
	enum enumAnalogPlot AnalogPlot;//ģ����Ƶ��� 0-��, 1-����ʮ�ֲ���
	UINT intTVStandard;//ģ����Ƶ��ʽ
	UINT intDetailRatio;//ģ�����DDEǿ�ȣ�0~3��0Ϊ�ر�
	UINT intEX;//ģ����Ƶ���ӱ�����0-none, 1-2X, 2-4X
	BOOL bSeparateMode;//����ģʽ

	int intBrightOffset;//����΢�� -100~100
	int intContrastOffset;//�Աȶ�΢�� -100~100

	BOOL bOSDTime;//�Ƿ�����Ƶ�ϵ��ӵ�ǰʱ��
	BOOL bOSDCamName;//�Ƿ�����Ƶ�ϵ����������
	BOOL bOSDMotionDetect;//�Ƿ�����Ƶ�ϵ����ƶ�����

	DWORD dwReserved9[6];

	//serial
	DWORD dwSerialBaudRate;//���ڲ�����
	DWORD dwSerialByteSize;//��������λ
	DWORD dwSerialStopBits;//����ֹͣλ
	DWORD dwSerilParity;//����У��λ

	DWORD dwReserved10[3];

	//ptz
	enum PTZProtocol protoPTZ;//��̨Э��
	UINT intPTZAddress;//��̨��ַ

	DWORD dwReserved11;

	//alarm time
	struct_TimeUnit alarmTime[7];//����ΪSunday, Monday, Tuesday...Saturday
}struct_CfgPara;

typedef struct 
{
	char charROIName[32];//ROI����
	int x0;//�����½�Ϊԭ�㣬x0����С��x1
	int y0;//y0����С��y1
	int x1;
	int y1;
	int intEmissivity;//������*100������90����0.9
	int intAlarmTemp;//�����¶ȣ���λmC��������ʽΪIO����ͻ�����������˸
	DWORD dwDraw;//��ʾѡ��
	UINT intSamplePeriod;//��������(֡)
	DWORD dwReserved[8];
}struct_RectROI;

struct struct_IrregularROI
{
	char charROIName[32];//ROI����, UTF8
	int intRoiType;//0-point, 1-line, 2-rect, 3-circle, 4-ellipse, 5-polygon, 6-delta, 7-delta3
	int x0;//��Ӿ��Σ������½�Ϊԭ�㣬x0����С��x1
	int y0;//y0����С��y1
	int x1;
	int y1;
	int intEmissivity;//������*100������90����0.9
	int intAlarmTemp;//�����¶ȣ���λmC��������ʽΪIO����ͻ�����������˸
	int intTextPos;//0-auto, 1-center, 2-left, 3-right, 4-top, 5-bottom
	UINT intSamplePeriod;//��������(֡)
	UINT intPtNumber;//��Ч���Ƶ�����
	POINT Points[7];//�û�������Ƶ�
	DWORD dwReserved;
	BOOL bNonvolatile;//�Ƿ�ϵ籣��
};

#define MAX_RECT_ROI_NUM	(4)

typedef struct 
{
	UINT intValidRectROI;//��Чrect ROI����
	struct_RectROI ROI[MAX_RECT_ROI_NUM];
}struct_UserROIs;

#define MAX_RECT_ROI_NUM2	(12)

struct struct_RectROIReport
{
	char charROIName[32];//ROI����, UTF8
	int x0;//�����½�Ϊԭ�㣬x0����С��x1
	int y0;//y0����С��y1
	int x1;
	int y1;
	BOOL bAlarm;//�Ƿ񴥷��˱���
	int intMinTemp;//��λmC
	int intMaxTemp;//��λmC
	int intAveTemp;//��λmC	
	int intMaxPos;
	int intAlarmThreshold[2];//��λmC
	int intTextPos;//0-auto, 1-center, 2-left, 3-right, 4-top, 5-bottom
	int intRoiType;//0-point, 1-line, 2-rect, 3-circle, 4-ellipse, 5-polygon, 6-delta, 7-delta3
	float fEmissivity;//��Ӧ�õķ����ʣ� 0��ʾδ�ṩ
	DWORD dwReserved;
	int intPresetId;//��ǰԤ��λ��ţ�-1δ֪����Ҫ3.0.6.3���Ϲ̼�֧��
};

struct struct_IrregularROIReport
{
	struct struct_RectROIReport baseinfo;

	UINT intColor;//0x00BBGGRR
	UINT intPtNumber;//��Ч���Ƶ�����
	POINT Points[8];//�û�������Ƶ�
	DWORD dwReserved[8];
};

typedef struct 
{
	BYTE MAC[6];//MAC address, it's read only
	BYTE pad[2];
	char charCamName[CAMNAME_PROTOCOLLEN];//camera name, it's read only
	DWORD dwSN;//serial number, it's read only
	BOOL bEnableDHCP;
	DWORD IPAddress;
	DWORD Subnetmask;
	DWORD DefaultGateway;
	DWORD Reserved[16];
}struct_IpV4Cfg;

#endif