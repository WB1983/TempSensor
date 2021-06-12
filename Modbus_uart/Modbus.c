#include "../Modbus_uart/Modbus.h"
#include "../Modbus_CRC/ModbusCRC.h"
#include "stdlib.h"
#include "../mcc_generated_files/pin_manager.h"
#include "../mcc_generated_files/eusart.h"
#include "../pic.h"

TModbusData MOS_tModbusData = {0,0,0,0,0,0};
TModbusPara MOS_tModbusPara = {0};
/*
因为波特率 9600
1位数据的时间为 1000000us/9600bit/s=104us
一个字节为    104us*10位  =1040us
所以 MODBUS确定一个数据帧完成的时间为   1040us*3.5=3.64ms  ->10ms
*/

void MOS_vModbusInit(void)
{
	MOS_tModbusPara.u8DeviceAddress = 4;  //本从设备的地址
	MOS_tModbusData.u8TimeoutStart = 0; //MODbus定时器停止计时
}

void MOS_vModbusReceiveEnalbe(void)
{
	IO_RB0_SetLow();
}

void MOS_vModbusTransmitEnalbe(void)
{
	IO_RB0_SetHigh();
}

void MOS_vModbudFun3()  //3号功能码处理  ---主机要读取本从机的寄存器
{
  	uint16_t u16RegisterAdd = 0;
	uint16_t u16RegisterLen = 0;
	uint16_t u16TransDataLen = 0;
	uint16_t i = 0;
	uint16_t j = 0;
	uint16_t u16TransmitCRC = 0;

	u16RegisterAdd=MOS_tModbusData.auReceiveBuff[2]*256+MOS_tModbusData.auReceiveBuff[3];  //得到要读取的寄存器的首地址
	u16RegisterLen=MOS_tModbusData.auReceiveBuff[4]*256+MOS_tModbusData.auReceiveBuff[5];  //得到要读取的寄存器的数量
	
	MOS_tModbusData.auTransmitBuff[i++]=MOS_tModbusPara.u8DeviceAddress;//本设备地址
    MOS_tModbusData.auTransmitBuff[i++]=0x03;        //功能码      
  	u16TransDataLen=u16RegisterLen*2;   //要返回的数据字节数*2

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

void MOS_vModbudFun6()  //6号功能码处理
{
  	uint16_t u16RegisterAdd = 0;
	uint16_t u16value = 0;
	uint16_t i = 0;
	uint16_t u16TransmitCRC = 0;
	uint16_t j = 0;

  	u16RegisterAdd=MOS_tModbusData.auReceiveBuff[2]*256+MOS_tModbusData.auReceiveBuff[3];  //得到要修改的地址 
	u16value=MOS_tModbusData.auReceiveBuff[4]*256+MOS_tModbusData.auReceiveBuff[5];     //修改后的值
	Reg[u16RegisterAdd]=u16value;  //修改本设备相应的寄存器
	
	//以下为回应主机
	
	MOS_tModbusData.auTransmitBuff[i++]=MOS_tModbusPara.u8DeviceAddress;//本设备地址
  	MOS_tModbusData.auTransmitBuff[i++]=0x06;        //功能码 
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
	if(0 == MOS_tModbusData.u8OneFrameFinish)  //没有收到MODbus的数据包
	{
		return;
	}
	
	u16ReceivedAddress = MOS_tModbusData.auReceiveBuff[0];
	u16ReceivedFunctionCode = MOS_tModbusData.auReceiveBuff[1];

	u16CalculateCRC= crc16(&MOS_tModbusData.auReceiveBuff[0], MOS_tModbusData.u8ReceivedByteCount-MOS_CRC_LEN);       //计算校验码
	
  	u16ReceivedCRC= (MOS_tModbusData.auReceiveBuff[MOS_tModbusData.u8ReceivedByteCount-MOS_CRC_LEN]*256);
	u16ReceivedCRC = u16ReceivedCRC  + MOS_tModbusData.auReceiveBuff[MOS_tModbusData.u8ReceivedByteCount -1];  //收到的校验码
  	if(u16CalculateCRC ==  u16ReceivedCRC)  //数据包符号CRC校验规则
	{ 
	  if(u16ReceivedAddress == MOS_tModbusPara.u8DeviceAddress)  //确认数据包是否是发给本设备的 
		{
		  switch(u16ReceivedFunctionCode)  //分析功能码
			{
				case 0:     break;
				case 1:     break;
				case 2:     break;
				case 3:     MOS_vModbudFun3();    break;   //3号功能码处理
				case 4:     break;
				case 5:     break;
				case 6:     MOS_vModbudFun6();     break;   //6号功能码处理
				case 7:     break;						
			}
		}
		else if(u16ReceivedAddress == MOS_BROADCAST_ADD)   //广播地址
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
















