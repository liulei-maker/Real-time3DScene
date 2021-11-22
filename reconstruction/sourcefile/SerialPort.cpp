#include "SerialPort.h"

SerialPort::SerialPort(void)
{
	this->m_hComm = INVALID_HANDLE_VALUE;
}

SerialPort::~SerialPort(void)
{ 
	if (this->m_hComm != INVALID_HANDLE_VALUE) 
	{
		CloseHandle(this->m_hComm);
		std::cout << "serial close!" << std::endl;
	}
}

/*******************************************************************
* ����     ��  �򿪴���
* COMx     :   ���ں�, ��_T("COM2:")
* BaudRate:    ������
*******************************************************************/
bool SerialPort::SerialOpen(LPCSTR COMx, int BaudRate)
{
	DCB dcb = { 0 };
	this->m_hComm = CreateFileA(COMx,
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		0,//FILE_FLAG_OVERLAPPED,	//ͬ����ʽ �� �ص���ʽ
		0
	);

	if (this->m_hComm == INVALID_HANDLE_VALUE)
	{
		DWORD dwError = GetLastError();
		return FALSE;
	}

	dcb.DCBlength = sizeof(DCB);

	if (!GetCommState(this->m_hComm, &dcb))
	{
		DWORD dwError = GetLastError();
		return FALSE;
	}

	dcb.BaudRate = BaudRate;	//������
	dcb.ByteSize = 8;			//λ��
	dcb.Parity = NOPARITY;		//��ż����
	dcb.StopBits = ONESTOPBIT;	//ֹͣλ��

	if (!SetCommState(this->m_hComm, &dcb))
	{
		DWORD dwError = GetLastError();
		return FALSE;
	}
	if (!PurgeComm(this->m_hComm, PURGE_RXCLEAR))    return -1;

	SetupComm(this->m_hComm, 1024, 1024);
	return TRUE;
}

/**
  serial write
  @param Buf:data buf
  @param size:bytes of Buf
  @return The len of writen
*/
int SerialPort::SerialWriteString(string& Buf, int size)
{
	DWORD dw;
	char charBuf[1024];
	strcpy_s(charBuf, Buf.c_str());
	WriteFile(this->m_hComm, charBuf, size, &dw, NULL);
	return dw;
}
int SerialPort::SerialWriteChar(unsigned char* Buf, int size)
{
	DWORD dw;
	WriteFile(this->m_hComm, Buf, size, &dw, NULL);
	return dw;
}
/**
  serial read
  @param OutBuf:���صõ�������
  @param maxSize:������������
  @return The len of read data
*/
int SerialPort::SerialReadString(string &OutBuf, int maxSize)
{
	DWORD readSize = 0;
	DWORD dwError  = 0;
	BOOL  bReadStatus;
	COMSTAT cs;
	ClearCommError(this->m_hComm, &dwError, &cs);
	if (cs.cbInQue > maxSize)
	{
		PurgeComm(this->m_hComm, PURGE_RXCLEAR);
		return 0;
	}
	char OutBufchar[100]="";
	//memset(OutBufchar, 0, sizeof(OutBuf));
	bReadStatus=ReadFile(this->m_hComm, OutBufchar, cs.cbInQue, &readSize, 0);

	OutBuf = OutBufchar;

	return readSize;
}

int SerialPort::SerialReadChar(unsigned char* OutBuf, int maxSize)
{
	DWORD readSize = 0;
	DWORD dwError = 0;
	BOOL  bReadStatus;
	COMSTAT cs;
	ClearCommError(this->m_hComm, &dwError, &cs);
	if (cs.cbInQue > maxSize)
	{
		PurgeComm(this->m_hComm, PURGE_RXCLEAR);
		return 0;
	}
	bReadStatus = ReadFile(this->m_hComm, OutBuf, cs.cbInQue, &readSize, 0);
	OutBuf[readSize] = '\0';

	return readSize;
}