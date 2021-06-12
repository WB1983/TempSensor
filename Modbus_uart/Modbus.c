#include "../Modbus_uart/Modbus.h"
#include "../Modbus_CRC/ModbusCRC.h"
#include "stdlib.h"
#include "../mcc_generated_files/pin_manager.h"
#include "../mcc_generated_files/eusart.h"
#include "../pic.h"

TModbusData MOS_tModbusData = {0,0,0,0,0,0};
TModbusPara MOS_tModbusPara = {0};
/*
��Ϊ������ 9600
1λ���ݵ�ʱ��Ϊ 1000000us/9600bit/s=104us
һ���ֽ�Ϊ    104us*10λ  =1040us
���� MODBUSȷ��һ������֡��ɵ�ʱ��Ϊ   1040us*3.5=3.64ms  ->10ms
*/

void MOS_vModbusInit(void)
{
	MOS_tModbusPara.u8DeviceAddress = 4;  //�����豸�ĵ�ַ
	MOS_tModbusData.u8TimeoutStart = 0; //MODbus��ʱ��ֹͣ��ʱ
}

void MOS_vModbusReceiveEnalbe(void)
{
	IO_RB0_SetLow();
}

void MOS_vModbusTransmitEnalbe(void)
{
	IO_RB0_SetHigh();
}

void MOS_vModbudFun3()  //3�Ź����봦��  ---����Ҫ��ȡ���ӻ��ļĴ���
{
  	uint16_t u16RegisterAdd = 0;
	uint16_t u16RegisterLen = 0;
	uint16_t u16TransDataLen = 0;
	uint16_t i = 0;
	uint16_t j = 0;
	uint16_t u16TransmitCRC = 0;

	u16RegisterAdd=MOS_tModbusData.auReceiveBuff[2]*256+MOS_tModbusData.auReceiveBuff[3];  //�õ�Ҫ��ȡ�ļĴ������׵�ַ
	u16RegisterLen=MOS_tModbusData.auReceiveBuff[4]*256+MOS_tModbusData.auReceiveBuff[5];  //�õ�Ҫ��ȡ�ļĴ���������
	
	MOS_tModbusData.auTransmitBuff[i++]=MOS_tModbusPara.u8DeviceAddress;//���豸��ַ
    MOS_tModbusData.auTransmitBuff[i++]=0x03;        //������      
  	u16TransDataLen=u16RegisterLen*2;   //Ҫ���ص������ֽ���*2

	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16TransDataLen%256);
	
	for(j=0;j<u16RegisterLen;j++)
	{
        MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(Reg[u16RegisterAdd+j]/256);
		MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(Reg[u16RegisterAdd+j]%256);		
	}
	u16TransmitCRC=crc16(MOS_tModbusData.auTransmitBuff,i);
	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16TransmitCRC/256);
	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16TransmitCRC%256);
	
	MOS_vModbusTransmitEnalbe();  //Transmission enable
	
	for(j=0; j<i; j++)
	{
	 EUSART_Write(MOS_tModbusData.auTransmitBuff[j]);
	}
	
	MOS_vModbusReceiveEnalbe();  //set to receive state
}

void MOS_vModbudFun6()  //6�Ź����봦��
{
  	uint16_t u16RegisterAdd = 0;
	uint16_t u16value = 0;
	uint16_t i = 0;
	uint16_t u16TransmitCRC = 0;
	uint16_t j = 0;

  	u16RegisterAdd=MOS_tModbusData.auReceiveBuff[2]*256+MOS_tModbusData.auReceiveBuff[3];  //�õ�Ҫ�޸ĵĵ�ַ 
	u16value=MOS_tModbusData.auReceiveBuff[4]*256+MOS_tModbusData.auReceiveBuff[5];     //�޸ĺ��ֵ
	Reg[u16RegisterAdd]=u16value;  //�޸ı��豸��Ӧ�ļĴ���
	
	//����Ϊ��Ӧ����
	
	MOS_tModbusData.auTransmitBuff[i++]=MOS_tModbusPara.u8DeviceAddress;//���豸��ַ
  	MOS_tModbusData.auTransmitBuff[i++]=0x06;        //������ 
  	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16RegisterAdd/256);
	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16RegisterAdd%256);
	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16value/256);
	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16value%256);
	u16TransmitCRC=crc16(MOS_tModbusData.auTransmitBuff,i);
	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16TransmitCRC/256);  //
	MOS_tModbusData.auTransmitBuff[i++]=(uint8_t)(u16TransmitCRC%256);
	
	MOS_vModbusTransmitEnalbe();  //
	
	for(j=0;j<i;j++)
	{
	 EUSART_Write(MOS_tModbusData.auTransmitBuff[j]);
	}
	
	MOS_vModbusReceiveEnalbe();  
}

void MOS_vModBusHandler(void)
{
	uint16_t u16CalculateCRC;
	uint16_t u16ReceivedCRC;

	uint8_t  u16ReceivedAddress;
	uint8_t  u16ReceivedFunctionCode;
	if(0 == MOS_tModbusData.u8OneFrameFinish)  //û���յ�MODbus�����ݰ�
	{
		return;
	}
	
	u16ReceivedAddress = MOS_tModbusData.auReceiveBuff[0];
	u16ReceivedFunctionCode = MOS_tModbusData.auReceiveBuff[1];

	u16CalculateCRC= crc16(&MOS_tModbusData.auReceiveBuff[0], MOS_tModbusData.u8ReceivedByteCount-MOS_CRC_LEN);       //����У����
	
  	u16ReceivedCRC= (MOS_tModbusData.auReceiveBuff[MOS_tModbusData.u8ReceivedByteCount-MOS_CRC_LEN]*256);
	u16ReceivedCRC = u16ReceivedCRC  + MOS_tModbusData.auReceiveBuff[MOS_tModbusData.u8ReceivedByteCount -1];  //�յ���У����
  	if(u16CalculateCRC ==  u16ReceivedCRC)  //���ݰ�����CRCУ�����
	{ 
	  if(u16ReceivedAddress == MOS_tModbusPara.u8DeviceAddress)  //ȷ�����ݰ��Ƿ��Ƿ������豸�� 
		{
		  switch(u16ReceivedFunctionCode)  //����������
			{
				case 0:     break;
				case 1:     break;
				case 2:     break;
				case 3:     MOS_vModbudFun3();    break;   //3�Ź����봦��
				case 4:     break;
				case 5:     break;
				case 6:     MOS_vModbudFun6();     break;   //6�Ź����봦��
				case 7:     break;						
			}
		}
		else if(u16ReceivedAddress == MOS_BROADCAST_ADD)   //�㲥��ַ
		{
		
		}
	}
	
	MOS_tModbusData.u8ReceivedByteCount = 0;   //
    MOS_tModbusData.u8OneFrameFinish = 0;	
}

TModbusData * Mos_PtGetModbusObject(void)
{
	return &MOS_tModbusData;
}
















