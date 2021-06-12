#ifndef _MODBUS
#define _MODBUS

#include "../pic.h"
#include "stdint.h"

#define MOS_CRC_LEN           2
#define MOS_BROADCAST_ADD     0

uint16_t Reg[]={
                0x0000,   //本设备寄存器中的值
                0x0001,
                0x0002,
                0x0003,
                0x0004,
                0x0005,
                0x0006,
                0x0007,
                0x0008,
                0x0009,
                0x000A,	
          };
          
typedef struct _TModbusData
{
    uint8_t auReceiveBuff[100];
    uint8_t u8OneFrameFinish;
    uint8_t u8ReceivedByteCount;
    uint8_t u8TimeoutStart;
    uint8_t auTransmitBuff[100];
    uint8_t u8TimeoutCount;//uint is ms
}TModbusData;

typedef struct _TModbusPara
{
    uint8_t u8DeviceAddress;
}TModbusPara;


extern void MOS_vModBusHandler(void);

extern TModbusData * Mos_PtGetModbusObject(void);

extern void MOS_vModbusReceiveEnalbe(void);

extern void MOS_vModbusTransmitEnalbe(void);

extern void MOS_vModbusInit(void);

#endif

