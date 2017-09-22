#include "Sins.h"
#include "loosely_coupled_INS_GNSS.h"
#include "datastream.h"
#include "rawdecode.h"
#include  "const.h"
#include "imuprocess.h"
#include "nmea.h"
#include "queue.h"
#include <time.h>

#define NSAMPLES 1
#define QUELENGTH 1000

void main()
{
	/*读入IMU数据,读入GPS数据*/
	static int8s type = STR_FILE;
	static int8s mode = STR_MODE_R;

	stream_t *strIMU = (stream_t *)malloc(sizeof(stream_t));//IMU数据
	char *pathIMU = "COM3";
	int nIMU = 1024;
	unsigned char *buffIMU = (unsigned char *)malloc(nIMU * sizeof(char));
	strinit(strIMU);

	stream_t *strGPS = (stream_t *)malloc(sizeof(stream_t));
	char *pathGPS = "COM4";
	int nGPS = 1024;
	unsigned char *buffGPS = (unsigned char *)malloc(nGPS * sizeof(char));
	strinit(strGPS);

	/*---------IMU数据------------------------------------------------------- */
	//IMU原始数据单位：加速度0.000244g,角速度0.1度/s，地磁场0.1Gauss。输出频率60hz
	unsigned int c;
	Packet_t *RxPkt = (Packet_t *)malloc(sizeof(Packet_t));
	memset(RxPkt, 0, sizeof(Packet_t));
	RxPkt->crc_header[0] = 0x5A; RxPkt->crc_header[1] = 0xA5;

	FILE *result = fopen("Readdata.txt", "w");
	FILE *sinsoutput = fopen("sins.txt", "w");
	FILE *kalman = fopen("kalman.txt", "w");
	FILE *gps = fopen("gpsdata.txt", "w");
	FILE *imu = fopen("imudata.txt", "w");

	//	IMUdata *imudata = (IMUdata *)calloc(1, sizeof(IMUdata));
	IMUdata_Raw imudata_raw = { { 0 } };
	IMUdata imu_data = { 0 };
	IMUdata imudata[1000] = { 0 };

	/*-------------GPS数据---------------------------------------------------*/
	nmeaPARSER *parser = (nmeaPARSER *)calloc(1, sizeof(nmeaPARSER));
	parser->buffer = (unsigned char *)calloc(1024, sizeof(unsigned char));
//	nmeaINFO *info = (nmeaINFO *)calloc(1, sizeof(nmeaINFO));

	/*---------SINS-----------------------------------------------------*/
	bool sins_init_flag = FALSE;//捷联惯导初始化标志
	bool align_flag = FALSE;//粗对准标志
	float64 ts = 0.1;
	float64 nts = NSAMPLES*ts;

	//	sins sinst = { { 0 } };
	sins *sinst = (sins *)calloc(1, sizeof(sins));
	sins *sinsdata = (sins *)calloc(1, sizeof(sins));
	pearth earthp = (pearth)calloc(1, sizeof(earth_para));

	/*--------kalmanfiter数据-----------------------------------------------------*/

	char *kf_updatemode = "";
	S_Kalmanfilter *kf = (S_Kalmanfilter *)calloc(1, sizeof(S_Kalmanfilter));
	//	S_Kalmanfilter kf = { { 0 } };

	SinsFilterOpt *sinsfilter_opt = (SinsFilterOpt *)calloc(1, sizeof(SinsFilterOpt));
	//	SinsFilterOpt sinsfilter_opt = { {0} };

	int32u iTer_allan = 0;
	float64 beta = 0;
	/*-----------------------------------------------------------------------------------*/
	LinkQueue IMUQue;
	int32u qulength = 0;
	Elemtype IMUhead;
	Elemtype IMU_data;

	int64u IMU_TIME = 0;//IMU计时器

	if (stropen(strIMU, STR_SERIAL, STR_MODE_R, pathIMU) && stropen(strGPS, STR_SERIAL, STR_MODE_R, pathGPS))
	//if (stropen(strGPS, STR_SERIAL, STR_MODE_R, pathGPS))
	{
		int16u n0, n1; n1 = n0 = 0;
		Vect wm, vm; //读数据
		Vect wm0, vm0, wmm, vmm;//粗对准
		wm = vm = wm0 = vm0 = wmm = vmm = { 0, 0, 0 };
		Vect Wm[NSAMPLES], Vm[NSAMPLES]; //子样
		memset(Wm, 0, sizeof(Vect)*NSAMPLES);
		memset(Vm, 0, sizeof(Vect)*NSAMPLES);

		Vect atti0, pos0, vel0;
		float64 Cnb0[3 * 3] = { 0 };
		quat qnb0;

		InitQueue(&IMUQue);
//		nmeaINFO *info = (nmeaINFO *)calloc(1, sizeof(nmeaINFO));
//		int ptype, nread = 0;
//		void *pack = 0;
			
		while (1)
		{
			if (strread(strIMU, buffIMU, nIMU) > 0)//读IMU数据
			{
				int16u nimu = 0;
				for (int16u i = 0; i < nIMU; i++)
				{
					c = buffIMU[i];
					if (input_IMU(c, RxPkt) == 1)
					{
						decode_IMU(RxPkt, &imudata_raw);
						
						imu_data.Gyo[0] = imudata_raw.GyoRaw[0] * 0.1*deg; imu_data.Gyo[1] = imudata_raw.GyoRaw[1] * 0.1*deg; imu_data.Gyo[2] = imudata_raw.GyoRaw[2] * 0.1*deg;
						imu_data.Acc[0] = imudata_raw.AccRaw[0] * 0.001;    imu_data.Acc[1] = imudata_raw.AccRaw[1] * 0.001;    imu_data.Acc[2] = imudata_raw.AccRaw[2] * 0.01;
						imu_data.atti[0] = imudata_raw.atti[0] * deg;      imu_data.atti[1] = imudata_raw.atti[1] * deg;      imu_data.atti[2] = imudata_raw.atti[2] * deg;
						imu_data.Timeflag++;

						memcpy(&imudata[nimu++], &imu_data, sizeof(IMUdata));
					}
				}
				
				memset(buffIMU, 0, nIMU*sizeof(char));

				//将IMU观测值放进队列
				qulength = 0;
				for (int i = 0; i < nimu; i++)
				{
					if (qulength < QUELENGTH)
					{
						EnQueue(&IMUQue, imudata[i]);
					    qulength++;
					}
					else
					{
						GetHead(IMUQue, &IMUhead);
						DeQueue(&IMUQue, &IMUhead);
						EnQueue(&IMUQue, imudata[i]);
					}
				}
			}

			//从IMU队列取数据
			while (GetHead(IMUQue, &IMU_data))
			{
				DeQueue(&IMUQue, &IMU_data); 

				time_t t;
				struct tm* lt;
				time(&t);
				lt = localtime(&t);

				wm.x = IMU_data.Gyo[0] * ts; wm.y = IMU_data.Gyo[1] * ts; wm.z = IMU_data.Gyo[2] * ts;
				vm.x = IMU_data.Acc[0] * ts; vm.y = IMU_data.Acc[1] * ts; vm.z = IMU_data.Acc[2] * ts;
						
				fprintf(imu, "timeflag:%d\t %d/%d/%d %d:%d:%d\t %.3f\t %.3f\t %.3f\t %.3f\t %.3f\t %.3f\t %.3f\t %.3f\t %.3f\n", IMU_data.Timeflag, lt->tm_year + 1900, lt->tm_mon, lt->tm_mday, lt->tm_hour, lt->tm_min, lt->tm_sec,
					IMU_data.Gyo[0], IMU_data.Gyo[1], IMU_data.Gyo[2], IMU_data.Acc[0], IMU_data.Acc[1], IMU_data.Acc[2], IMU_data.atti[0] * 180 / PI, IMU_data.atti[1] * 180 / PI, IMU_data.atti[2] * 180 / PI);

				nmeaINFO info = { 0 };
				bool get_gps_pos = false;

				Sleep(100);

				if (strread(strGPS, buffGPS, nGPS) > 0)//读GPS数据
				{			
					nmea_parser_init(parser);
					nmea_parse(parser, buffGPS, nGPS, &info);
			//		memset(buffGPS, 0, sizeof(unsigned char)*nGPS);
					printf("读取到gps数据\n");
					printf("lat:%.6f\tlon:%.6f\telv:%.3f\tPDOP:%.3f\n", info.lat, info.lon, info.elv,info.PDOP);
				
				//	Sleep(100);
					get_gps_pos = false;

					float64 lat, lon;
					int l1, l2;
					lat = info.lat; lon = info.lon;
					l1 = lat / 100; l2 = lon / 100;

					info.timeflag = IMU_data.Timeflag;
					info.lat = (l1 + (lat - l1 * 100) / 60)*deg;
					info.lon = (l2 + (lon - l2 * 100) / 60)*deg;
					info.speed = info.speed * 1000 / 3600;
					info.direction = info.direction*deg;

					if (info.direction == 0) info.speed = 0;
					if (info.lat != 0 && info.lon != 0 && info.elv != 0)
					{
						get_gps_pos = true;

						printf("获取gps位置信息\n");
				//		fprintf(sinsoutput, "timeflag:%d 获取gps pos\t", info.timeflag);
						
				//		fprintf(sinsoutput, "lat:%.6f\t lon:%.6f\t elv:%.3f\t PDOP:%.3f\t HDOP:%.3f\t VDOP:%.3f\t speed:%.3f\t direction:%.3f\n",
				//			info.lat * 180 / PI, info.lon * 180 / PI, info.elv, info.PDOP, info.HDOP,info.VDOP,info.speed, info.direction * 180 / PI);

						fprintf(gps, "timeflag:%d\t %d/%d/%d %d:%d:%d\t lat:%.6f\t lon:%.6f\t elv:%.3f\t PDOP:%.3f\t HDOP:%.3f\t VDOP:%.3f\t speed:%.3f\t dir:%.3f\n", info.timeflag,
							info.utc.year, info.utc.mon, info.utc.day, info.utc.hour, info.utc.minute, info.utc.second,
							info.lat * 180 / PI, info.lon * 180 / PI, info.elv, info.PDOP, info.HDOP, info.VDOP, info.speed, info.direction * 180 / PI);
					}	

				}
				else
				{
					get_gps_pos = false;
				}

				memset(buffGPS, 0, nGPS*sizeof(unsigned char));

				//粗对准
				if (align_flag == false)
				{
					float64 n_wm0, n_vm0,x,lat;
					Vect Wm0, Vm0;

				//	if (n0 < 19)
				//	{
				//		Vect_add_equ(&wmm, wm); Vect_add_equ(&vmm, vm); n0++; continue;
				//	}
					n0++;
					Vect_add_equ(&wmm, wm); Vect_add_equ(&vmm, vm);
					
					if (get_gps_pos == TRUE && n0>19)
					{
						Vect_mul(wmm, 1.0 / n0, &wm0);
						Vect_mul(vmm, 1.0 / n0, &vm0);

						vel0.x = info.speed*sin(info.direction); vel0.y = info.speed*cos(info.direction); vel0.z = 0;
						pos0.x = info.lat; pos0.y = info.lon; pos0.z = info.elv;

						printf("已获取GPS位置信息，粗对准\n");
						fprintf(sinsoutput, "已获取GPS位置信息，粗对准\n");

						earth_para_update(vel0, pos0, earthp);
						align_coarse(wm0, vm0, earthp, Cnb0);
						align_flag = TRUE;

						printf("完成粗对准\n");
						fprintf(sinsoutput, "完成粗对准\t");
						fprintf(sinsoutput, "lat:%6f\t lon:%.6f\t elv:%.3f\n", pos0.x*180/PI, pos0.y*180/PI, pos0.z);
					}
				/*	else
					{
						vel0 = { 0, 0, 0 };
						Vect_norm(wm0, &n_wm0); Vect_norm(wm0, &n_vm0);
						Vect_mul(wm0, 1.0 / n_wm0, &Wm0); Vect_mul(vm0, 1.0 / n_vm0, &Vm0);

						Vect_dot(Wm0, Vm0, &x);
						pos0.x = lat = asin(x);

						printf("无GPS位置信息，粗对准\n");
						fprintf(sinsoutput, "无GPS位置信息，粗对准\n");
					}*/
		              
				}

				//捷联惯导状态初始化
				if (sins_init_flag == FALSE && get_gps_pos==true && align_flag==true)
				{
				//	atti0 = { 0, 0, 0 };
				//	pos0 = { 40.141692*deg, 116.603149*deg, 34.42 };
				//	vel0 = { 0, 0, 0 };
				
					atti0.x = IMU_data.atti[0]; atti0.y = IMU_data.atti[1]; atti0.z = IMU_data.atti[2];
				//	vel0.x = info.speed*sin(info.direction); vel0.y = info.speed*cos(info.direction); vel0.z = 0;
				//	pos0.x = info.lat; pos0.y = info.lon; pos0.z = info.elv;
			
					earth_para_update(vel0, pos0, earthp);
					sinst->Mpv[0 * 3 + 1] = 1.0 / earthp->RMh; sinst->Mpv[1 * 3 + 0] = 1.0 / earthp->RNh; sinst->Mpv[2 * 3 + 2] = 1;

					attsyn2(atti0,&qnb0,Cnb0);
			//		attsyn0(Cnb0, &qnb0, &atti0);
					sins_init1(Cnb0, qnb0, atti0, pos0, vel0, sinst);
					
					sins_init_flag = TRUE;

					printf("惯导初始化:\n");
					fprintf(sinsoutput, "惯导初始化:\n");
					fprintf(sinsoutput, "%.6f,%.6f,%.6f\t", sinst->atti.x * 180 / PI, sinst->atti.y * 180 / PI, sinst->atti.z * 180 / PI);
					fprintf(sinsoutput, "%.3f,%.3f,%.3f\t", sinst->vel.x, sinst->vel.y, sinst->vel.z);
					fprintf(sinsoutput, "%.6f,%.6f,%.6f\n", sinst->pos.x * 180 / PI, sinst->pos.y * 180 / PI, sinst->pos.z);
				
					continue;
				}

				if (n1 < NSAMPLES - 1)
				{
					Wm[n1] = wm; Vm[n1++] = vm; continue;
				}

				Wm[n1] = wm; Vm[n1++] = vm;
				n1 = 0;

				if (sins_init_flag == TRUE)
				{
					sins_update(Wm, Vm, NSAMPLES, ts, sinst, earthp);  //捷联惯导时间更新

			//		sinst->atti.x = IMU_data.atti[0]; sinst->atti.y = IMU_data.atti[1]; sinst->atti.z = IMU_data.atti[2];//

					printf("惯导递推:\n");
					fprintf(sinsoutput, "timeflag:%d\t", IMU_data.Timeflag);
					fprintf(sinsoutput, "惯导递推:\t");
					fprintf(sinsoutput, "%.6f,%.6f,%.6f,\t", sinst->atti.x * 180 / PI, sinst->atti.y * 180 / PI, sinst->atti.z * 180 / PI);
					fprintf(sinsoutput, "%.3f,%.3f,%.3f,\t", sinst->vel.x, sinst->vel.y, sinst->vel.z);
					fprintf(sinsoutput, "%.6f,%.6f,%.6f\n", sinst->pos.x * 180 / PI, sinst->pos.y * 180 / PI, sinst->pos.z);

					kf_updatemode = "T";

			/*		fprintf(sinsoutput, "timeflag:%d\t", IMU_data.Timeflag);
					if (get_gps_pos == true)
					{
						kf_updatemode = "B";
						fprintf(sinsoutput, "量测更新:\t");
						printf("量测更新:\n");
					}
					else
					{
						fprintf(sinsoutput, "时间更新:\t");
						printf("时间更新:\n");
					}

					loosely_coupled_GNSS_INS(earthp, sinst, &info, nts, kf, sinsfilter_opt, kf_updatemode, &iTer_allan, &beta);

					fprintf(sinsoutput, "%.6f,%.6f,%.6f,\t", sinsfilter_opt->atti.x * 180 / PI, sinsfilter_opt->atti.y * 180 / PI, sinsfilter_opt->atti.z * 180 / PI);
					fprintf(sinsoutput, "%.3f,%.3f,%.3f,\t", sinsfilter_opt->vel.x, sinsfilter_opt->vel.y, sinsfilter_opt->vel.z);
					fprintf(sinsoutput, "%.6f,%.6f,%.6f\n", sinsfilter_opt->pos.x * 180 / PI, sinsfilter_opt->pos.y * 180 / PI, sinsfilter_opt->pos.z);*/
				}				
			//	n1 = 0;
	//			
			}
		

			

		/*		for (int32u i = 0; i < 2502; i++)
					{
					if (  iPhasePC == GPSdata[i].time ) // && GPSdata[i].pos.x != 0 && GPSdata[i].pos.y != 0 || GPSdata[i].pos.z != 0)
					{
					kf_updatemode = "B";
					fprintf(sinsoutput, "time:%ld,\t", iPhasePC);

					loosely_coupled_GNSS_INS(earthp, sinst, GPSdata[i], nts, kf, sinsfilter_opt, kf_updatemode,&iTer_allan,&beta);

					//			fprintf(sinsoutput, "量测更新:\t");
					fprintf(sinsoutput, "%.6f,%.6f,%.6f,\t", sinsfilter_opt->atti.x * 180 / PI, sinsfilter_opt->atti.y * 180 / PI, sinsfilter_opt->atti.z * 180 / PI);
					fprintf(sinsoutput, "%.3f,%.3f,%.3f,\t", sinsfilter_opt->vel.x, sinsfilter_opt->vel.y, sinsfilter_opt->vel.z);
					fprintf(sinsoutput, "%.6f,%.6f,%.6f\n", sinsfilter_opt->pos.x * 180 / PI, sinsfilter_opt->pos.y * 180 / PI, sinsfilter_opt->pos.z);

					break;
					}
					}

					if (kf_updatemode == "T")

					{
					//			fprintf(sinsoutput, "时间更新:\t");
					loosely_coupled_GNSS_INS(earthp, sinst, GPSdata[0], nts, kf, sinsfilter_opt, kf_updatemode,&iTer_allan,&beta);
					}

					//			fprintf(sinsoutput, "%.6f,%.6f,%.6f,\t", sinsfilter_opt->atti.x * 180 / PI, sinsfilter_opt->atti.y * 180 / PI, sinsfilter_opt->atti.z * 180 / PI);
					//			fprintf(sinsoutput, "%.3f,%.3f,%.3f,\t", sinsfilter_opt->vel.x, sinsfilter_opt->vel.y, sinsfilter_opt->vel.z);
					//			fprintf(sinsoutput, "%.3f,%.3f,%.3f\n", sinsfilter_opt->pos.x * 180 / PI, sinsfilter_opt->pos.y * 180 / PI, sinsfilter_opt->pos.z);
					//			fprintf(sinsoutput, "\n\n");

					}*/

		}

		fclose(result); fclose(sinsoutput);
	}
}
