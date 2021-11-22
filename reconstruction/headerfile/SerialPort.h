#ifndef __SERIALPORT_H__
#define __SERIALPORT_H__

#include <windows.h>
#include <iostream>
#include <string>
using namespace std;

class SerialPort
{
public:
	SerialPort(void);
	~SerialPort(void);
	//�򿪴���
	
	bool SerialOpen(LPCSTR COMx, int BaudRate);
	//��������
	int SerialWriteString(string& Buf, int size);
	int SerialWriteChar(unsigned char* Buf, int size);
	//��������
	int SerialReadString(string &OutBuf,int maxSize);
	int SerialReadChar(unsigned char* OutBuf,int maxSize);
private:
	HANDLE m_hComm;//���ھ��
};
#endif
