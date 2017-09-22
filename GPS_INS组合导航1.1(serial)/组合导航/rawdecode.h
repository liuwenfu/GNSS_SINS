#pragma once
#include "types.h"

#define MAX_PACKET_LEN          128

//IMU数据状态
#define kStatus_Idle      0
#define kStatus_Cmd       1
#define kStatus_LenLow    2
#define kStatus_LenHigh   3
#define kStatus_CRCLow    4
#define kStatus_CRCHigh   5
#define kStatus_Data      6

typedef struct
{
	int ofs;
	int buf[MAX_PACKET_LEN];
	int payload_len;
	int len;
	int status;
	int crc_header[4];
	int CRCReceived;
	int CRCCalculated;
}Packet_t;  //IMU原始数据包
typedef void(*OnDataReceivedEvent)(Packet_t *pkt);


typedef struct
{
	short int AccRaw[3];
	short int GyoRaw[3];
	short int MagRaw[3];
	float64 atti[3];
}IMUdata_Raw;

typedef struct
{
	float64 Acc[3];
	float64 Gyo[3];
	float64 Mag[3];
	float64 atti[3];
	int Timeflag;
}IMUdata;

extern void crc16_update(int *currectCrc, const int *src, int lengthInBytes);
extern int input_IMU(unsigned int data, Packet_t *RxPkt);
extern void decode_IMU(Packet_t *RxPkt, IMUdata_Raw *imudata);