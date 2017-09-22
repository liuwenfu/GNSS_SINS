#include "rawdecode.h"
#include <stdio.h>

static OnDataReceivedEvent EventHandler;

//IMU数据crc更新
extern void crc16_update(int *currectCrc, const int *src, int lengthInBytes)
{
	int crc = *currectCrc;
	int j;
	for (j = 0; j < lengthInBytes; ++j)
	{
		int i;
		int byte = src[j];
		crc ^= byte << 8;
		for (i = 0; i < 8; ++i)
		{
			int temp = crc << 1;
			if (crc & 0x8000)
			{
				temp ^= 0x1021;
			}
			crc = temp;
		}
	}
	*currectCrc = crc;
}
//IMU数据打包
extern int input_IMU(unsigned int data, Packet_t *RxPkt)
{
	switch (RxPkt->status)
	{
	case kStatus_Idle:
		if (data == 0x5A)
			RxPkt->status = kStatus_Cmd;
		break;
	case kStatus_Cmd:
		if (data == 0xA5)
			RxPkt->status = kStatus_LenLow;
		break;
	case kStatus_LenLow:
		RxPkt->payload_len = data;
		RxPkt->crc_header[2] = data;
		RxPkt->status = kStatus_LenHigh;
		break;
	case kStatus_LenHigh:
		RxPkt->payload_len |= (data << 8);
		RxPkt->crc_header[3] = data;
		RxPkt->status = kStatus_CRCLow;
		break;
	case kStatus_CRCLow:
		RxPkt->CRCReceived = data;
		RxPkt->status = kStatus_CRCHigh;
		break;
	case kStatus_CRCHigh:
		RxPkt->CRCReceived |= (data << 8);
		RxPkt->ofs = 0;
		RxPkt->CRCCalculated = 0;
		RxPkt->status = kStatus_Data;
		break;
	case kStatus_Data:
		RxPkt->buf[RxPkt->ofs++] = data;
		if (RxPkt->ofs >= RxPkt->payload_len)
		{
			crc16_update(&RxPkt->CRCCalculated, RxPkt->crc_header, 4);
			crc16_update(&RxPkt->CRCCalculated, RxPkt->buf, RxPkt->ofs);

			if (RxPkt->CRCCalculated == RxPkt->CRCReceived)
			{
				if (EventHandler != NULL)
				{
					EventHandler(RxPkt);
				}
			}

			RxPkt->status = kStatus_Idle;

			return 1;
		}
	}
	return 0;
}
//IMU数据解析
extern void decode_IMU(Packet_t *RxPkt, IMUdata_Raw *imudata_raw)
{
	if (RxPkt->buf[2] == 0xA0)
	{
		imudata_raw->AccRaw[0] = (short int)(RxPkt->buf[3] + (RxPkt->buf[4] << 8));
		imudata_raw->AccRaw[1] = (short int)(RxPkt->buf[5] + (RxPkt->buf[6] << 8));
		imudata_raw->AccRaw[2] = (short int)(RxPkt->buf[7] + (RxPkt->buf[8] << 8));
	}

	if (RxPkt->buf[9] == 0xB0)
	{

		imudata_raw->GyoRaw[0] = (short int)(RxPkt->buf[10] + (RxPkt->buf[11] << 8));
		imudata_raw->GyoRaw[1] = (short int)(RxPkt->buf[12] + (RxPkt->buf[13] << 8));
		imudata_raw->GyoRaw[2] = (short int)(RxPkt->buf[14] + (RxPkt->buf[15] << 8));
	}

	if (RxPkt->buf[16] == 0xC0)
	{

		imudata_raw->MagRaw[0] = (short int)(RxPkt->buf[17] + (RxPkt->buf[18] << 8));
		imudata_raw->MagRaw[1] = (short int)(RxPkt->buf[19] + (RxPkt->buf[20] << 8));
		imudata_raw->MagRaw[2] = (short int)(RxPkt->buf[21] + (RxPkt->buf[22] << 8));
	}

	if (RxPkt->buf[23] == 0xD0)
	{
		imudata_raw->atti[0] = ((float64)(short int)(RxPkt->buf[24] + (RxPkt->buf[25] << 8))) / 100;
		imudata_raw->atti[1] = ((float64)(short int)(RxPkt->buf[26] + (RxPkt->buf[27] << 8))) / 100;
		imudata_raw->atti[2] = ((float64)(short int)(RxPkt->buf[28] + (RxPkt->buf[29] << 8))) / 10;
	}
}
