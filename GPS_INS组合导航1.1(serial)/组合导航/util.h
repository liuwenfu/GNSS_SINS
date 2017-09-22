#pragma once
#include "types.h"
#include "matrix.h"

//定义三维矢量
typedef struct
{
	float64 x;
	float64 y;
	float64 z;
}Vect;
//typedef Vect *pVect;
//定义四元数
typedef struct
{
	float64 q0;
	float64 q1;
	float64 q2;
	float64 q3;
}quat;

//定义地球参数
typedef struct
{
//	float64 g0;//重力加速度矢量
//	Vect g = { 0, 0, g0 };
	float64 lat;//纬度
	float64 lon;//经度
	float64 heigh; //高度
	float64 RMh;//子午圈曲率半径
	float64 RNh;//卯酉圈曲率半径
	Vect wnie;//地球自转速率在导航系中的投影
	Vect wnen;//导航系相对于地球系的旋转角速率在导航系中的投影
	Vect wnin;//导航系相对于惯性系的旋转角速率在导航系中的投影
	Vect wnien;
	Vect gn;//重力加速度在导航系中的投影
	Vect gcc;//扣除有害加速度之后的重力加速度
//	float64 Cne[3 * 3];
}earth_para;
typedef earth_para *pearth;

typedef struct
{
	float64 sec;
	int16u year;
	int8u mon;
	int8u day;
	int8u hour;
	int8u min;
} UTCTime, *pUTCTime;

typedef struct
{
	int64u time;
	int32u    vFMP;                   /*position fixing mode : 0-fix not 1:Do not care 2:BD2 3:GPS 4:Dual  */
	int32u    vFMV;                   /*Speed fixing mode : 0-fix not 1:Do not care 2:BD2 3:GPS 4:Dual     */
	int32u    dopFlg;                 /*内部使用1：有效0无效                                               */
	int8u     validFlg;				  /*GNSS状态(0:无效，1:gps有效,2:bd2有效 ,3:GPS_BD2全部有效)           */
	int8u     usedNum;				  /*使用卫星数                           */
	int8u     enviroment;             /*定位环境                             */	
	float64   dir; 					  /*方向角 (弧度)                        */

	Vect pos;                         /*位置 lat,lon,atl*/
	Vect vel; 					      /*速度 ve,vn,vu                        */
	Vect ecefPos;					/*ECEF格式位置                       */
	Vect ecefVel;					/*ECEF格式速度                       */

	float64   pdop_p;				  /*GNSS原始：位置精度因子               */
	float64   hdop_p; 				  /*GNSS原始：水平精度因子 (小于2可用)   */
	float64   vdop_p;				  /*GNSS原始：垂直精度因子 (小于2可用)   */
	float64   pdop_v;				  /*GNSS原始：速度精度因子               */
	float64   hdop_v; 				  /*GNSS原始：速度水平精度因子           */
	float64   dop_p; 				  /*水平位置误差因子 (小于10可用)，需要配合dopFlg使用                   */
	float64   dop_v;				  /*速度误差因子（小于0.5可用）， 需要配合dopFlg使用                    */
	float64   dop_a; 				  /*航向角误差因子（目前暂无参考价值）， 需要配合dopFlg使用             */
	UTCTime utc_time;			      /*UTC时间                              */

	int32u usdnumG;				      /*使用的GPS卫星数                      */
	int32u usdnumB;				      /*使用的BD卫星数                       */

	
	int32u optSwitch;
	int32u sig_mode;
	float64   orig_lon;
	float64   orig_lat;
	float64   orig_dir;
	float64   orig_vel;

	float64		vRsdSqureP;
	float64		vRsdSqureV;
	int32s		vDof;
	int32u		klmPVEstCnt;
	//FixInfHandle	hfix;


	int8u		velUsedBDsat;
	int8u		velUsedGPSsat;

}GNSS_SignData;

void Vect_norm(Vect v, float64 *norm);
void Vect_cross(Vect a, Vect b, Vect *out);
void Vect_dot(Vect a, Vect b, float64 *out);
void Vect_mul(Vect a, float64 b, Vect *out);
void Vect_add(Vect a, Vect b, Vect *out);
void Vect_sub(Vect a, Vect b, Vect *out);
void Vect_add_equ(Vect *a, Vect b);
//void Vect2matrix(Vect *v, float64 *m);
//void matrix2Vect(float64 *m, Vect *v);
void askew(Vect v, float64 *Mv);

void rv2q(Vect rv, quat *qo);
void rv2cnb(Vect rv, float64 *Cnb);
void att2cnb(Vect atti, float64 *Cnb);
void cnb2att(float64 *Cnb, Vect *atti);
void att2q(Vect atti, quat *qnb);
void q2att(quat qnb, Vect *atti);
void q2cnb(quat qnb, float64 *Cnb);
void cnb2q(float64 *Cnb, quat *qnb);
void cnb2q1(float64 *Cnb, quat *qnb);
void attsyn0(float64 *Cnb, quat *qnb, Vect *atti);
void attsyn1(quat qnb, float64 *Cnb, Vect *atti);
void attsyn2(Vect atti, quat *qnb, float64 *Cnb);

void qconj(quat qi, quat *qo);
void qnormlz(quat qi, quat *qo);
void qmul(quat qi1, quat qi2, quat *qo);
void qmulv(quat qi, Vect vi, Vect *vo);
void lq2m(quat qi, float64 *mq);
void rq2m(quat qi, float64 *mq);
void rotv(Vect rv, Vect vi, Vect *vo);