#ifndef _MAG_ERRORCODE_H_
#define _MAG_ERRORCODE_H_

#define SEV_MAG			(0xE<<28)
#define FACILITY_MAG	(0x4AE<<16)

#define MAG_ERROR_CODE(e)	(SEV_MAG | FACILITY_MAG | ((e) & 0xFFFF) )

//error codes for magnity
#define MAG_INVALIDPARAMETER		MAG_ERROR_CODE(0x1)//���������������
#define MAG_INSUFFICIENTMEMORY		MAG_ERROR_CODE(0x2)//�ڴ�newʧ��
#define MAG_UNCLASSED				MAG_ERROR_CODE(0x3)//δ����Ĵ���
#define MAG_NOTINITIALIZED			MAG_ERROR_CODE(0x4)//��δ����MAG_Initialize()
#define MAG_NOAVAILABLECAMERA		MAG_ERROR_CODE(0x5)//��δ�������
#define MAG_NOAVAILABLEDATA			MAG_ERROR_CODE(0x6)//��û�п����������
#define MAG_COMMUNICATIONERROR		MAG_ERROR_CODE(0x7)//ͨѶ����
#define MAG_COMMUNICATIONTIMEOUT	MAG_ERROR_CODE(0x8)//ͨѶ��ʱ
#define MAG_FILEIOERROR				MAG_ERROR_CODE(0x9)//�ļ���д����
#define MAG_DUPLICATEDOPERATION		MAG_ERROR_CODE(0xA)//�ظ������������ظ�����MAG_NewChannel()
#define MAG_NOTREADY				MAG_ERROR_CODE(0xB)//����ʹ�õ���Դ��δ׼����
#define MAG_OEMERROR				MAG_ERROR_CODE(0xC)//�������Ʒ��𲻷�

//error codes for mag_ethernet
#define MAG_NOTCONNECTED			MAG_ERROR_CODE(0x101)//��δ������̫��
#define MAG_SOCKETERROR				MAG_ERROR_CODE(0x102)//socket����
#define MAG_SOCKETPORTERROR			MAG_ERROR_CODE(0x103)//socket�˿ڴ���

#endif
