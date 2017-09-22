#pragma once
#include "types.h"
#include "util.h"

#define DIMSTATE 15  //系统状态维数

typedef struct
{
	Vect atti;//姿态欧拉角 x:pitch,y:roll,z:yaw
	quat qnb;//姿态四元数
	float64 Cnb[3*3]; //姿态方向余弦阵
	Vect vel;//地速 x:velE,y:velN,z:velU
	Vect pos;//位置 x:latitude,y:longitude,z:height
	float64 Mpv[3*3];//位置矩阵
	float64 Ft[DIMSTATE*DIMSTATE];//状态转移矩阵
	
	Vect wib;//载体系相对于惯性系的旋转角速度
	Vect wnb;//载体系相对导航系的旋转角速度
	Vect web;//载体系相对地球系的旋转角速度
	Vect fb;//比力
	Vect fn;//比力在导航系的投影
	Vect an;//导航系线加速度
	float64 Cne[3 * 3];//位置矩阵
	float64 tauG;//陀螺仪马尔科夫相关时间
	float64 tauA;//加计马尔科夫相关时间
}sins;
typedef sins *psins;

void earth_para_update(Vect vn, Vect pos, pearth earth_p);
void align_coarse(Vect wm, Vect vm, pearth earth_p, float64 *Cnb);
void align_coarse1(Vect wm, Vect vm, pearth earth_p, float64 *Cnb);
void sins_init(Vect atti0, Vect pos0, Vect vel0, psins sins0);
void sins_init1(float64 *Cnb, quat qnb0,Vect atti0,Vect pos0, Vect vel0, psins sinst);
void attupdate(earth_para earth_p, Vect phim, float64 nts, psins sinst);
void velupdate(earth_para earth_p, Vect fn, float64 nts, psins sinst);
void posupdate(earth_para earth_p, float64 nts,Vect vn1, psins sinst);
void posupdate1(earth_para earth_p, float64 nts, Vect vn1, psins sinst);
void sins_update(Vect *wm, Vect *vm, int16u nSamples, float64 ts, psins sinst,pearth earthp);
void sins_setFt(pearth earth_p, psins sinst);

