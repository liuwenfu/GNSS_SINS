
#include "util.h"
#include "matrix.h"
#include <math.h>
#include <stdlib.h>
#include "const.h"

/*说明：欧拉角于四元数之间相互转换的函数未经核实*/

//矢量求模
void Vect_norm(Vect v,float64 *norm)
{
	float64 x, y, z;
	x = v.x*v.x;
	y = v.y*v.y;
	z = v.z*v.z;
	*norm = sqrt(x + y + z);
}

//矢量叉乘
void Vect_cross(Vect a, Vect b, Vect *out)
{
	out->x =a.y*b.z-a.z*b.y ;
	out->y =a.z*b.x-a.x*b.z ;
	out->z =a.x*b.y-a.y*b.x ;
}
//矢量点乘
void Vect_dot(Vect a, Vect b, float64 *out)
{
	float64 x, y, z;
	x = a.x*b.x;
	y = a.y*b.y;
	z = a.z*b.z;
	*out = x + y + z;
}
//矢量数乘
void Vect_mul(Vect a, float64 b,Vect *out)
{
	out->x=a.x *b;
	out->y=a.y *b;
	out->z=a.z *b;
}
//矢量加法
void Vect_add(Vect a, Vect b, Vect *out)
{
	out->x = a.x + b.x;
	out->y = a.y + b.y;
	out->z = a.z + b.z;
}
//矢量减法
void Vect_sub(Vect a, Vect b, Vect *out)
{
	out->x = a.x - b.x;
	out->y = a.y - b.y;
	out->z = a.z - b.z;
}
//矢量加等于运算+=
void Vect_add_equ(Vect *a,Vect b)
{
	a->x += b.x;
	a->y += b.y;
	a->z += b.z;
}
//矢量转换成矩阵
/*void Vect2matrix(Vect *v, Matrix *m)
{
	m->row = 3;
	m->col = 1;
	m->element[0] = v->x;
	m->element[1] = v->y;
	m->element[2] = v->z;
}
//矩阵转换成矢量
void matrix2Vect(Matrix *m,Vect *v)
{
	v->x = m->element[0];
	v->y = m->element[1];
	v->z = m->element[2];
}*/
//矢量的反对称矩阵
void askew(Vect v,float64 *Mv)
{
	Mv[0 * 3 + 0] = 0;     Mv[0 * 3 + 1] = -v.z; Mv[0 * 3 + 2] = v.y;
	Mv[1 * 3 + 0] = v.z;  Mv[1 * 3 + 1] = 0;     Mv[1 * 3 + 2] = -v.x;
	Mv[2 * 3 + 0] = -v.y; Mv[2 * 3 + 1] = v.x;  Mv[2 * 3 + 2] = 0;
}

//共轭四元数
void qconj(quat qi, quat *qo)
{
	qo->q0 = qi.q0;
	qo->q1 = -qi.q1;
	qo->q2 = -qi.q2;
	qo->q3 = -qi.q3;
}
void qnormlz(quat qi, quat *qo)
{
	float nm;
	nm = qi.q0*qi.q0 + qi.q1*qi.q1 + qi.q2*qi.q2 + qi.q3*qi.q3;
/*	if (nm <1.0e-6)
	{
		qo->q0 = 1.0; qo->q1 = 0.0; qo->q2 = 0.0; qo->q3 = 0.0;
	}
	else
	{
	
	}*/
	qo->q0 = qi.q0 / sqrt(nm);
	qo->q1 = qi.q1 / sqrt(nm);
	qo->q2 = qi.q2 / sqrt(nm);
	qo->q3 = qi.q3 / sqrt(nm);

}
//四元数乘法
void qmul(quat qi1, quat qi2, quat *qo)
{
	qo->q0 = qi1.q0*qi2.q0 - qi1.q1*qi2.q1 - qi1.q2*qi2.q2 - qi1.q3*qi2.q3;
	qo->q1 = qi1.q0*qi2.q1 + qi1.q1*qi2.q0 + qi1.q2*qi2.q3 - qi1.q3*qi2.q2;
	qo->q2 = qi1.q0*qi2.q2 + qi1.q2*qi2.q0 + qi1.q3*qi2.q1 - qi1.q1*qi2.q3;
	qo->q3 = qi1.q0*qi2.q3 + qi1.q3*qi2.q0 + qi1.q1*qi2.q2 - qi1.q2*qi2.q1;
}

/*------------------------------------------------------------------
      姿态欧拉角、旋转矢量、方向余弦矩阵（Cnb）、四元数相互转换
------------------------------------------------------------------*/
//等效旋转矢量转化成四元数  已核
void rv2q(Vect rv, quat *qo)
{
	float64 value = 0, value2 = 0;
	float64 s = 0,n2=0;
	quat q;

	float64 a = 0;

	value = sqrt(rv.x * rv.x + rv.y * rv.y + rv.z * rv.z);
	value2 = rv.x * rv.x + rv.y * rv.y + rv.z * rv.z;

	if (value2 < 1.0e-8)//cos(n/2)=1-n2/8+n4/384; sin(n/2)/n=1/2-n2/48+n4/3840 
	{
		q.q0 = 1 - value2*(1.0 / 8 - value2 / 384.0);
		s = 1 / 2 - value2*(1.0 / 48 - value2 / 3840.0);
	}
	else
	{
		q.q0 = cos(value / 2.0);
		s = sin(value / 2.0) / value;
	}
	q.q1 = s*rv.x;
	q.q2 = s*rv.y;
	q.q3 = s*rv.z;

	*qo = q;

	n2 = q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3;
	if (n2>1.000001 || n2<0.999999) qnormlz(q, qo);
}
//等效旋转矢量转换成方向余弦阵 已核  
void rv2cnb(Vect rv, float64 *Cnb)
{
	float64 xx = 0, yy = 0, zz = 0;
	float64 n2=0,n;
	float64 a, b, arvx, arvy, arvz, bxx, bxy, bxz, byy, byz, bzz;

	xx = rv.x*rv.x; yy = rv.y*rv.y; zz = rv.z*rv.z;
	n2 = xx + yy + zz;
	if (n2 < 1.0e-8)
	{
		a = 1 - n2*(1 / 6 - n2 / 120); b = 0.5 - n2*(1 / 24 - n2 / 720);
	}
	else
	{
		n = sqrt(n2);
	    a = sin(n) / n;  b = (1 - cos(n)) / n2;
	}
	arvx = a*rv.x;  arvy = a*rv.y;  arvz = a*rv.z;
	bxx = b*xx;  bxy = b*rv.x*rv.y;  bxz = b*rv.x*rv.z;
	byy = b*yy;  byz = b*rv.y*rv.z;  bzz = b*zz;

	Cnb[0*3+ 0] = 1 - byy - bzz;    Cnb[0*3+ 1] =   -arvz + bxy;    Cnb[0*3+ 2] =   arvy + bxz;
	Cnb[1*3+ 0] =    arvz + bxy;    Cnb[1*3+ 1] = 1 - bxx - bzz;    Cnb[1*3+ 2] =   -arvx + byz;
	Cnb[2*3+ 0] =   -arvy + bxz;    Cnb[2*3+ 1] =    arvx + byz;    Cnb[2*3+ 2] = 1 - bxx - byy;

}

//姿态欧拉角转换成方向余弦阵  参考秦永元 《惯性导航》p297
void att2cnb(Vect atti,float64 *Cnb)
{
	float64 sx, sy, sz, cx, cy, cz;

	sx = sin(atti.x); sy = sin(atti.y); sz = sin(atti.z);
	cx = cos(atti.x); cy = cos(atti.y); cz = cos(atti.z);
	
	Cnb[0*3+ 0] = cy*cz + sx*sy*sz; Cnb[0*3+ 1] = cx*sz;  Cnb[0*3+ 2] = sy*cz - sx*cy*sz;
	Cnb[1*3+ 0] =-cy*sz + sx*sy*cz; Cnb[1*3+ 1] = cx*cz;  Cnb[1*3+ 2] = -sy*sz - sx*cy*cz;
	Cnb[2*3+ 0] = -cx*sy;           Cnb[2*3+ 1] = sx;     Cnb[2*3+ 2] = cx*cy;

}
/*方向余弦阵转换成姿态欧拉角
航向角、横滚角存在多值性  0<=yaw<360,<-180<=roll<180
航向角yaw取值：
Cnb[1,1]>0,yaw>0  位于[0,90]      yaw
Cnb[1,1]<0,yaw<0  位于[90,180]    yaw
Cnb[1,1]<0,yaw>0  位于[180,270]   yaw+360
Cnb[1,1]>0,yaw<0  位于[270,360]   yaw+360
横滚角roll取值:
Cnb[2,2]>0,roll>0  位于[0,90]      roll
Cnb[2,2]<0,roll<0  位于[90,180]    roll+180
Cnb[2,2]<0,roll>0  位于[-180,-90]  roll-180
Cnb[2,2]>0,roll<0  位于[-90,0]     roll
*/
void cnb2att(float64 *Cnb, Vect *atti)
{
	float64 yaw, pitch, roll;
	
	pitch = asin(Cnb[2*3+ 1]);
	
	if (abs(Cnb[2 * 3 + 0]) < 1.0e-8 || abs(Cnb[2 * 3 + 2]) <1.0e-12) roll = 0;
	else
	{
		roll = atan(-Cnb[2*3+ 0]/Cnb[2*3+ 2]);
		if (Cnb[2 * 3 + 2] > 0 )
			roll = roll;
		else if (Cnb[2 * 3 + 2] < 0 && roll < 0)
			roll = roll+PI;
		else if (Cnb[2 * 3 + 2] < 0 && roll > 0)
			roll = roll-PI;
	}

	if (abs(Cnb[0 * 3 + 1]) < 1.0e-8 || abs(Cnb[1 * 3 + 1]) <1.0e-12) yaw = 0;
	else
	{
		yaw = atan(Cnb[0*3+ 1]/Cnb[1*3+ 1]);

		if (Cnb[1 * 3 + 1] < 0)
			yaw = yaw +  PI;
		else if (Cnb[1 * 3 + 1] > 0 && yaw > 0)
			yaw = yaw;
		else if (Cnb[1 * 3 + 1] > 0 && yaw < 0)
			yaw = yaw + 2*PI;


	/*	if (Cnb[0 * 3 + 1] > 0)
			yaw = yaw;
		else
			yaw =-yaw;*/
	}
	atti->x = pitch;
	atti->y = roll;
	atti->z = yaw;
}

void cnb2att1(float64 *Cnb, Vect *atti)
{
	atti->x = asin(Cnb[2 * 3 + 1]);
	atti->y = atan2(-Cnb[2 * 3 + 0], Cnb[2 * 3 + 2]);
	atti->z = atan2(-Cnb[0 * 3 + 1], Cnb[1 * 3 + 1]);
}
//姿态欧拉角转换成四元数 
void att2q(Vect atti,quat *qnb)
{
	float64 sx, sy, sz,cx,cy,cz,n2;
	Vect atti_2;
	quat q;
	Vect_mul(atti, 1.0 / 2, &atti_2);
	sx = sin(atti_2.x); sy = sin(atti_2.y); sz = sin(atti_2.z);
	cx = cos(atti_2.x); cy = cos(atti_2.y); cz = cos(atti_2.z);
	q.q0 = cx*cy*cz - sx*sy*sz;
	q.q1 = sx*cy*cz - cx*sy*sz;
	q.q2 = cx*sy*cz + sx*cy*sz;
	q.q3 = cx*cy*sz + sx*sy*cz;

	//*qnb = q;
	n2 = q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3;
	if (n2>1.000001 ||n2<0.999999) qnormlz(q, qnb);
}

//姿态欧拉角转换成四元数
/*void att2q(Vect *atti, quat *qnb)
{
	float64 sx, sy, sz, cx, cy, cz, tmp,q0,q1,q2,q3;
	Matrix *Cnb = (Matrix *)malloc(sizeof(Matrix));
	Cnb->row = 3; Cnb->col = 3;
	Cnb->element = (float64 *)malloc(sizeof(float64) * 3 * 3);

	sx = sin(atti->x); sy = sin(atti->y); sz = sin(atti->z);
	cx = cos(atti->x); cy = cos(atti->y); cz = cos(atti->z);

	Cnb->element[0 * 3 + 0] = cy*cz + sx*sy*sz; Cnb->element[0 * 3 + 1] = cx*sz;  Cnb->element[0 * 3 + 2] = sy*cz - sx*cy*sz;
	Cnb->element[1 * 3 + 0] = -cy*sz + sx*sy*cz; Cnb->element[1 * 3 + 1] = cx*cz;  Cnb->element[1 * 3 + 2] = -sy*sz - sx*cy*cz;
	Cnb->element[2 * 3 + 0] = -cx*sy;           Cnb->element[2 * 3 + 1] = sx;     Cnb->element[2 * 3 + 2] = cx*cy;

	tmp = 1.0 + Cnb->element[0*3+0] - Cnb->element[1*3+1] - Cnb->element[2*3+2];
	q1 = sqrt(fabs(tmp)) / 2.0;
	tmp = 1.0 - Cnb->element[0*3+0] + Cnb->element[1*3+1] - Cnb->element[2*3+2];
	q2 = sqrt(fabs(tmp)) / 2.0;
	tmp = 1.0 - Cnb->element[0 * 3 + 0] - Cnb->element[1 * 3 + 1] + Cnb->element[2 * 3 + 2];
	q3 = sqrt(fabs(tmp)) / 2.0;
	tmp = 1.0 - q1*q1 - q2*q2 - q3*q3;
	q0 = sqrt(fabs(tmp));

	if (Cnb->element[2*3+1] - Cnb->element[1*3+2] < 0)	// sign decision 
	{
		q1 = -q1;
	}
	if (Cnb->element[0*3+2] - Cnb->element[2*3+0] < 0)
	{
		q2 = -q2;
	}
	if (Cnb->element[1,0] - Cnb->element[0,1] < 0)
	{
		q3 = -q3;
	}

	double nq = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 /= nq; q1 /= nq; q2 /= nq; q3 /= nq;
}*/

//四元数转换成姿态欧拉角  已核
void q2att(quat qnb,Vect *atti)
{
	float64 q00, q01, q02, q03,q11,q12,q13,q22,q23,q33;
	float64 c01, c11, c20,c21,c22;
	float64 yaw, pitch, roll;

	q00 = qnb.q0*qnb.q0; q01 = qnb.q0*qnb.q1; q02 = qnb.q0*qnb.q2; q03 = qnb.q0*qnb.q3;
    q11 = qnb.q1*qnb.q1; q12 = qnb.q1*qnb.q2; q13 = qnb.q1*qnb.q3;
	q22 = qnb.q2*qnb.q2; q23 = qnb.q2*qnb.q3;
	q33 = qnb.q3*qnb.q3;

	c01 = 2 * (q12 - q03);
	c11 = q00 - q11 + q22 - q33;
	c20 = 2 * (q13 - q02); c21 = 2 * (q23 + q01); c22 = q00 - q11 - q22 + q33;

    pitch = asin(c21);

	if (abs(c20) < 1.0e-8 || abs(c22) < 1.0e-8) roll = 0;
	else
	{
//		roll = atan2(-c20, c22);
		roll = atan(-c20 / c22);
		if (c22 > 0)
			roll = roll;
		else if (c22 < 0 && roll < 0)
			roll = roll + PI;
		else if (c22 < 0 && roll > 0)
			roll = roll - PI;

	}
	if (abs(c01) < 1.0e-8 || abs(c11) <1.0e-8) yaw = 0;
	else
	{ 
//		yaw = atan2(c01, c11); 
		yaw = atan(c01 / c11);

		if (c11 < 0)
			yaw = yaw + PI;
		else if (c11 > 0 && yaw > 0)
			yaw = yaw;
		else if (c11 > 0 && yaw < 0)
			yaw = yaw + 2 * PI;

/*		if (c01 > 0)
			yaw = yaw;
		else
			yaw =-yaw;*/
	}
	
	atti->x = pitch; atti->y = roll;atti->z = yaw;
}
//四元数转换成方向余弦阵 已核
void q2cnb(quat qnb,float64 *Cnb)
{
	float64 q00, q01, q02, q03, q11, q12, q13, q22, q23, q33;
	q00 = qnb.q0*qnb.q0; q01 = qnb.q0*qnb.q1; q02 = qnb.q0*qnb.q2; q03 = qnb.q0*qnb.q3;
	q11 = qnb.q1*qnb.q1; q12 = qnb.q1*qnb.q2; q13 = qnb.q1*qnb.q3;
	q22 = qnb.q2*qnb.q2; q23 = qnb.q2*qnb.q3;
	q33 = qnb.q3*qnb.q3;

	Cnb[0*3+0] = q00 + q11 - q22 - q33; Cnb[0*3+ 1] = 2 * (q12 - q03);       Cnb[0*3+ 2] = 2 * (q13 + q02);
	Cnb[1*3+0] = 2 * (q12 + q03);       Cnb[1*3+ 1] = q00 - q11 + q22 - q33; Cnb[1*3+ 2] = 2 * (q23 - q01);
	Cnb[2*3+0] = 2 * (q13 - q02);       Cnb[2*3+ 1] = 2 * (q23 + q01);       Cnb[2*3+ 2] = q00 - q11 - q22 + q33;
}

//方向余弦阵转换成四元数 参考秦永元《惯性导航》p303  精度低
void cnb2q(float64 *Cnb,quat *qnb)
{
	float64 q0, q1, q2, q3,n2;
	quat q;

	q0 = sqrt(abs(1 + Cnb[0*3+0] + Cnb[1*3+1] + Cnb[2*3+2])) / 2.0;
	q1 = sqrt(abs(1 + Cnb[0*3+0] - Cnb[1*3+1] - Cnb[2*3+2])) / 2.0;
	q2 = sqrt(abs(1 - Cnb[0*3+0] + Cnb[1*3+1] - Cnb[2*3+2])) / 2.0;
	q3 = sqrt(abs(1 - Cnb[0*3+0] - Cnb[1*3+1] + Cnb[2*3+2])) / 2.0;

	if (Cnb[2, 1] < Cnb[1, 2]) q1 = -q1;
	if (Cnb[0, 2] < Cnb[2, 0]) q2 = -q2;
	if (Cnb[1, 0] < Cnb[0, 1]) q3 = -q3;

	q.q0 = q0; q.q1 = q1; q.q2 = q2; q.q3 = q3;
//	*qnb = q;

	n2 = q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3;
    if(n2>1.000001 ||n2<0.999999)	qnormlz(q, qnb);
	else *qnb = q;
}
//参考严恭敏 PINS
void cnb2q1(float64 *Cnb, quat *qnb)
{
	float64 qq4,n2;
	quat q;
	if (Cnb[0 * 3 + 0] >= Cnb[1 * 3 + 1] + Cnb[2 * 3 + 2])
	{
		q.q1 = 0.5*sqrt(abs(1 + Cnb[0 * 3 + 0] - Cnb[1 * 3 + 1] - Cnb[2 * 3 + 2])); qq4 = 4 * q.q1;
		q.q0 = (Cnb[2 * 3 + 1] - Cnb[1 * 3 + 2]) / qq4;
		q.q2 = (Cnb[0 * 3 + 1] + Cnb[1 * 3 + 0]) / qq4;
		q.q3 = (Cnb[0 * 3 + 2] + Cnb[2 * 3 + 0]) / qq4;
	}
	else if (Cnb[1 * 3 + 1] >= Cnb[0 * 3 + 0] + Cnb[2 * 3 + 2])
	{
		q.q2 = 0.5*sqrt(abs(1 - Cnb[0 * 3 + 0] + Cnb[1 * 3 + 1] - Cnb[2 * 3 + 2])); qq4 = 4 * q.q2;
		q.q0 = (Cnb[0 * 3 + 2] - Cnb[2 * 3 + 0]) / qq4;
		q.q1 = (Cnb[0 * 3 + 1] + Cnb[1 * 3 + 0]) / qq4;
		q.q3 = (Cnb[1 * 3 + 2] + Cnb[2 * 3 + 1]) / qq4;
	}
	else if (Cnb[2 * 3 + 2] >= Cnb[0 * 3 + 0] + Cnb[1 * 3 + 1])
	{
		q.q3 = 0.5*sqrt(abs(1 - Cnb[0 * 3 + 0] - Cnb[1 * 3 + 1] + Cnb[2 * 3 + 2])); qq4 = 4 * q.q3;
		q.q0 = (Cnb[1 * 3 + 0] - Cnb[0 * 3 + 1])/qq4;
		q.q1 = (Cnb[0 * 3 + 2] + Cnb[2 * 3 + 0]) / qq4;
		q.q2 = (Cnb[1 * 3 + 2] + Cnb[2 * 3 + 1]) / qq4;
	}
	else
	{
		q.q0 = 0.5*sqrt(abs(1 + Cnb[0 * 3 + 0] + Cnb[1 * 3 + 1] + Cnb[2 * 3 + 2])); qq4 = 4 * q.q0;
		q.q1 = (Cnb[2 * 3 + 1] - Cnb[1 * 3 + 2])/qq4;
		q.q2 = (Cnb[0 * 3 + 2] - Cnb[2 * 3 + 0]) / qq4;
		q.q3 = (Cnb[1 * 3 + 0] - Cnb[0 * 3 + 1]) / qq4;
	}

//	n2 = q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3;
//	if (n2 > 1.000001 || n2 < 0.999999)	qnormlz(q, qnb);
//	else *qnb = q;
	*qnb = q;
}
//输入方向余弦阵，输出欧拉角和四元数
void attsyn0(float64 *Cnb, quat *qnb, Vect *atti)
{
	cnb2q1(Cnb, qnb);
	cnb2att(Cnb,atti);
}
//输入四元数，输出方向余弦阵和欧拉角
void attsyn1(quat qnb, float64 *Cnb, Vect *atti)
{
	q2cnb(qnb, Cnb);
	cnb2att(Cnb, atti);
}
//输入欧拉角，输出四元数和方向余弦阵
void attsyn2(Vect atti, quat *qnb, float64 *Cnb)
{
	att2cnb(atti, Cnb);
	cnb2q1(Cnb, qnb);
}

/*-------------------------------------------------------------------
   四元数运算
---------------------------------------------------------------------*/
//对矢量进行四元数操作
void qmulv(quat qi,Vect vi,Vect *vo)
{
	quat qo;
	qo.q0 =             - qi.q1*vi.x - qi.q2*vi.y - qi.q3*vi.z;
	qo.q1 = qi.q0*vi.x               + qi.q2*vi.z - qi.q3*vi.y ;
	qo.q2 = qi.q0*vi.y               + qi.q3*vi.x - qi.q1*vi.z;
	qo.q3 = qi.q0*vi.z               + qi.q1*vi.y - qi.q2*vi.x;

	vo->x = -qo.q0*qi.q1 + qo.q1*qi.q0 - qo.q2*qi.q3 + qo.q3*qi.q2;
	vo->y = -qo.q0*qi.q2 + qo.q2*qi.q0 - qo.q3*qi.q1 + qo.q1*qi.q3;
	vo->z = -qo.q0*qi.q3 + qo.q3*qi.q0 - qo.q1*qi.q2 + qo.q2*qi.q1;
}
/*In quaternion multiplication: q = q1*q2, convert q1 to 4x4 matrix M(q1)
so that q1*q2 equals M(q1)*q2*/
void lq2m(quat qi,float64 *mq)
{
	mq[0*4+ 0] = qi.q0; mq[0*4+ 1] = -qi.q1; mq[0*4+ 2] = -qi.q2; mq[0*4+ 3] = -qi.q3;
	mq[1*4+ 0] = qi.q1; mq[1*4+ 1] =  qi.q0; mq[1*4+ 2] = -qi.q3; mq[1*4+ 3] =  qi.q2;
	mq[2*4+ 0] = qi.q2; mq[2*4+ 1] =  qi.q3; mq[2*4+ 2] =  qi.q0; mq[2*4+ 3] = -qi.q1;
	mq[3*4+ 0] = qi.q3; mq[3*4+ 1] = -qi.q2; mq[3*4+ 2] =  qi.q1; mq[3*4+ 3] =  qi.q0;
}
/* In quaternion multiplication : q = q1*q2, convert q2 to 4x4 matrix M(q2),
so that q1*q2 equals M(q2)*q1*/
void rq2m(quat qi, float64 *mq)
{
	mq[0*4+ 0] = qi.q0; mq[0*4+ 1] = -qi.q1; mq[0*4+ 2] = -qi.q2; mq[0*4+ 3] = -qi.q3;
	mq[1*4+ 0] = qi.q1; mq[1*4+ 1] =  qi.q0; mq[1*4+ 2] =  qi.q3; mq[1*4+ 3] = -qi.q2;
	mq[2*4+ 0] = qi.q2; mq[2*4+ 1] = -qi.q3; mq[2*4+ 2] =  qi.q0; mq[2*4+ 3] =  qi.q1;
	mq[3*4+ 0] = qi.q3; mq[3*4+ 1] =  qi.q2; mq[3*4+ 2] = -qi.q1; mq[3*4+ 3] =  qi.q0;
}
//Rotate a 3x1 vector by a rotation vector
void rotv(Vect rv,Vect vi,Vect *vo)
{
	float64 n_2,n,n2,q0,q1,q2,q3,s;
	quat qo;

	n2 = rv.x*rv.x + rv.y*rv.y + rv.z*rv.z;
	n = sqrt(n2);
	n_2 = n / 2;

	if (n2 < 1.0e-12)
	{
		q0 = 1 - n2*(1 / 8 - n2 / 384); s = 1 / 2 - n2*(1 / 48 - n2 / 3840);
	}
	else
	{
		q0 = cos(n_2); s = sin(n_2) / n;
	}

	q1 = s*rv.x; q2 = s*rv.y; q3 = s*rv.z;

	qo.q0 =          - q1*vi.x - q2*vi.y -q3*vi.z;
	qo.q1 = q0*vi.x            + q2*vi.z -q3*vi.y;
	qo.q2 = q0*vi.y            + q3*vi.x -q1*vi.z ; 
	qo.q3 = q0*vi.z            + q1*vi.y -q2*vi.x;

	vo->x = -qo.q0*q1 + qo.q1*q0 - qo.q2*q3 + qo.q3*q2;
	vo->y = -qo.q0*q2 + qo.q2*q0 - qo.q3*q1 + qo.q1*q3;
	vo->z = -qo.q0*q3 + qo.q3*q0 - qo.q1*q2 + qo.q2*q1;
}