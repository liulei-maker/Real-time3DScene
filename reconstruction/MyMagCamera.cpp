#include "MyMagCamera.h"


MyMagCamera::MyMagCamera()
{
	device = new CMagDevice;
	service = new CMagService;
}

MyMagCamera::~MyMagCamera()
{
	device->StopProcessImage();
	device->DisLinkCamera();
	delete device;
	delete service;
	std::cout << "DisLink Camera!" << std::endl;
}

void CALLBACK NewFrame(UINT intChannelIndex, int intCameraTemperature, DWORD dwFFCCounterdown,
	DWORD dwCamState, DWORD dwStreamType, void* pUserData)
{
	CMagDevice* pDevice = (CMagDevice*)pUserData;

	if (dwStreamType == STREAM_TEMPERATURE)//temperature
	{
		const UCHAR* pIrData;
		const BITMAPINFO* pIrInfo;
		if (pDevice->GetOutputBMPdata(&pIrData, &pIrInfo))
		{
			//pIrData�б������BMPͼ���ʵ������
		}
	}
}

bool MyMagCamera::InitialCamera(char* strIP)
{
	service->EnableAutoReConnect(TRUE);//������������
	if (!device->IsInitialized())
	{
		printf("�����ʼ��ʧ�ܣ�\r\n");
		return FALSE;
	}

	if (!device->LinkCamera(strIP))
	{
		printf("�������ʧ�ܣ�\r\n");
		return FALSE;
	}
	const struct_CamInfo* pCamInfo = device->GetCamInfo();
	if (!pCamInfo)
	{
		printf("��ȡ�������ʧ�ܣ�\r\n");
		return FALSE;
	}

	OutputPara paraOut;
	paraOut.dwFPAWidth = pCamInfo->intFPAWidth;
	paraOut.dwFPAHeight = pCamInfo->intFPAHeight;
	paraOut.dwBMPWidth = pCamInfo->intVideoWidth;
	paraOut.dwBMPHeight = pCamInfo->intVideoHeight;
	paraOut.dwColorBarWidth = 16;
	paraOut.dwColorBarHeight = pCamInfo->intVideoHeight;

	if (!device->StartProcessImage(&paraOut, NewFrame, STREAM_TEMPERATURE, &device))
	{
		printf("��������ʧ�ܣ�\r\n");
		device->DisLinkCamera();
		return FALSE;
	}

	Sleep(100);
	return TRUE;
}

void MyMagCamera::SaveThermalBMP(char* saveThermalBMPPath)
{
	WCHAR* saveThermalBMPPath_s;
	swprintf(saveThermalBMPPath_s, 100, L"%hs", saveThermalBMPPath);
	device->Lock();
	if (device->SaveBMP(0, saveThermalBMPPath_s))
	{
		//printf("����BMP�ɹ���\r\n");
	}
	else
	{
		//printf("����BMPʧ�ܣ�\r\n");
	}
	device->Unlock();
}

void MyMagCamera::StorageThermalAvi(const WCHAR* charFilename = NULL, UINT time = 1000, UINT intSamplePeriod = 1)
{
	device->LocalStorageAviStart(charFilename, intSamplePeriod);
	Sleep(time);
	device->LocalStorageAviStart(charFilename, intSamplePeriod);
	device->LocalStorageAviStop();
}

void MyMagCamera::saveTemperatureData(char* saveTemperatureDataPath)
{
	const struct_CamInfo* info;
	info = device->GetCamInfo();
	const UINT dataSize = info->intFPAWidth * info->intFPAHeight;

	/*���Բɼ���֡��������Ҫ��ʱ��*/
	//clock_t startTime, endTime;
	// startTime = clock();//��ʱ��ʼ
	device->Lock();
	int* TemperatureData;
	TemperatureData = new int[dataSize];
	device->GetTemperatureData_Raw(TemperatureData, dataSize * sizeof(int), 1);
	device->Unlock();
	//endTime = clock();
	//cout << "time1:" << endTime - startTime << endl;
	FILE* stream;
	stream = fopen(saveTemperatureDataPath, "w+");
	fwrite(TemperatureData, sizeof(int), dataSize, stream);
	fclose(stream);
	delete[] TemperatureData;
	//endTime = clock();
	//cout << "time2:" << endTime - startTime << endl;
}
void MyMagCamera::StorageThermalMgs(const WCHAR* charFilename = NULL, UINT time = 1000, UINT intSamplePeriod = 1)
{
	device->LocalStorageMgsRecord(charFilename, intSamplePeriod);
	Sleep(time);
	device->LocalStorageMgsStop();

}
void MyMagCamera::StorageMgsStart(const WCHAR* charFilename = NULL, UINT intSamplePeriod = 1)
{
	device->LocalStorageMgsRecord(charFilename, intSamplePeriod);
}
void MyMagCamera::StorageMgsStop()
{
	device->LocalStorageMgsStop();
}
void MyMagCamera::StorageMgsPlay(const WCHAR* charFilename, MAG_FRAMECALLBACK funcFrame, void* pUserData)
{
	device->LocalStorageMgsPlay(charFilename, funcFrame, pUserData);

}