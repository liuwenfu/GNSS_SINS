#include "Sins.h"
#include "const.h"
#include "imuprocess.h"
#include "matrix.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//#include "util.h"


/*捷联惯导数据处理*/

/*地球参数更新
输入：vn   速度
pos  位置
输出：RMh   子午圈曲率半径
RNh   卯酉圈曲率半径
wnie  地球自转角速度在导航系的投影
wnen  导航系相对地球的旋转角速度在导航系中的投影
wnin  导航系相对于惯性系的旋转角速度在导航系中的投影
gn    重力加速度在导航中的投影
gcc   扣除有害加速度的重力加速度
*/
void earth_para_update(Vect vn, Vect pos,pearth earth_p)
{
	float64 sl, cl, tl, sl2, sl4;
	float64 sq, sq_2;
	float64 f_RMh, f_RNh, clRNh, f_clRNh;
	Vect wien,gcc1;
	float64 Cne[3 * 3] = { 0 };

	sl = sin(pos.x); cl = cos(pos.x); tl = sl / cl; sl2 = sl*sl; sl4 = sl2*sl2;
	sq = 1 - earth_e2*sl*sl; sq_2 = sqrt(sq);
	earth_p->lat = pos.x; earth_p->lon = pos.y; earth_p->heigh = pos.z;
	
	earth_p->RNh = earth_Re / sq_2 + pos.z; f_RNh = 1.0 / earth_p->RNh; clRNh = cl*earth_p->RNh; 

	earth_p->RMh = earth_p->RNh*(1 - earth_e2) / sq  + pos.z; f_RMh = 1.0 / earth_p->RMh;

	earth_p->wnie = { 0, wie*cl, wie*sl };

	earth_p->wnen = { -vn.y*f_RMh, vn.x*f_RNh, vn.x*f_RNh*tl };

    Vect_add(earth_p->wnie,earth_p->wnen,&earth_p->wnin); 

	earth_p->gn = { 0, 0, -(g0*(1 + 5.27094e-3*sl2 + 2.32718e-5*sl4) - 3.086e-6*pos.z) };

	Vect_add(earth_p->wnie, earth_p->wnin, &earth_p->wnien);

	Vect_cross(earth_p->wnien, vn, &gcc1);
	Vect_sub(earth_p->gn, gcc1, &earth_p->gcc);

}

//参考严恭敏书籍
void align_coarse1(Vect wm, Vect vm, pearth earth_p, float64 *Cnb)
{
	float64 T11, T12, T13, T21, T22, T23, T31, T32, T33;
	float64 cl, tl, nn, norm_wm, norm_vm,x;
	Vect wbib, fb;
	float64 Cnbtmp[3 * 3] = { 0 };
	float64 CnbtmpInv[3 * 3] = { 0 };

	wbib = fb = { 0, 0, 0 };

//	Vect_mul(wm, 1.0 / nts, &wm); Vect_mul(wm, 1.0 / nts, &vm);
	Vect_norm(wm, &norm_wm); Vect_norm(vm, &norm_vm);
	Vect_mul(wm, 1.0 / norm_wm, &wm); Vect_mul(vm, 1.0 / norm_vm, &vm);
	wbib = wm; fb = vm;

//	Vect_dot(wm,vm,&x);
//	lat = asin(x);

//	if (earth_p->lat == 0)  earth_p->lat = lat;

	cl = cos(earth_p->lat); tl = tan(earth_p->lat);

	T31 = fb.x; T32 = fb.y; T33 = fb.z;
	T21 = wbib.x / cl - T31*tl; T22 = wbib.y / cl - T32*tl; T23 = wbib.z / cl - T33*tl;		nn = sqrt(T21*T21 + T22*T22 + T23*T23);  T21 /= nn, T22 /= nn, T23 /= nn;
	T11 = T22*T33 - T23*T32; T12 = T23*T31 - T21*T33; T13 = T21*T32 - T22*T31;		nn = sqrt(T11*T11 + T12*T12 + T13*T13);  T11 /= nn, T12 /= nn, T13 /= nn;

	Cnb[0 * 3 + 0] = T11; Cnb[0 * 3 + 1] = T12; Cnb[0 * 3 + 2] = T13;
	Cnb[1 * 3 + 0] = T21; Cnb[1 * 3 + 1] = T22; Cnb[1 * 3 + 2] = T23;
	Cnb[2 * 3 + 0] = T31; Cnb[2 * 3 + 1] = T32; Cnb[2 * 3 + 2] = T33;

	//正交化
//	memcpy(CnbtmpInv, Cnbtmp, 3 * 3 * sizeof(float64));
//	Matrixinv(CnbtmpInv,3);
//	MatrixAdd(Cnbtmp,3,3, CnbtmpInv, Cnb);
//	NumMultiMatrix(0.5, Cnb,3,3,Cnb);

}

    //参考秦永元书籍
/*void align_coarse1(Vect wm, Vect vm, pearth earth_p,float64 nts, float64 *Cnb)
{
	float64 T11, T12, T13, T21, T22, T23, T31, T32, T33;
	float64 cl, tl, nn, norm_wm, norm_vm, x;
	Vect wbib, fb;

// 	float64 Cnbtmp[3 * 3] = { 0 };
//	float64 CnbtmpInv[3 * 3] = { 0 };

	wbib = fb = { 0, 0, 0 };

	Vect_mul(wm, 1.0/nts, &wm); Vect_mul(vm, 1.0/nts, &vm);
//	Vect_norm(wm, &norm_wm); Vect_norm(vm, &norm_vm);
//	Vect_mul(wm, 1.0 / norm_wm, &wm); Vect_mul(vm, 1.0 / norm_vm, &vm);

	wbib = wm; fb = vm;
	cl = cos(earth_p->lat); tl = tan(earth_p->lat);

	T31 = fb.x/g0; T32 = fb.y/g0; T33 = fb.z/g0;
	T21 = wbib.x / cl/wie - T31*tl; T22 = wbib.y / cl/wie - T32*tl; T23 = wbib.z / cl/wie - T33*tl;	//	nn = sqrt(T21*T21 + T22*T22 + T23*T23);  T21 /= nn, T22 /= nn, T23 /= nn;
	T11 = T22*T33 - T23*T32; T12 = T23*T31 - T21*T33; T13 = T21*T32 - T22*T31;	//	nn = sqrt(T11*T11 + T12*T12 + T13*T13);  T11 /= nn, T12 /= nn, T13 /= nn;

	Cnb[0 * 3 + 0] = T11; Cnb[0 * 3 + 1] = T12; Cnb[0 * 3 + 2] = T13;
	Cnb[1 * 3 + 0] = T21; Cnb[1 * 3 + 1] = T22; Cnb[1 * 3 + 2] = T23;
	Cnb[2 * 3 + 0] = T31; Cnb[2 * 3 + 1] = T32; Cnb[2 * 3 + 2] = T33;

	//正交化
//	memcpy(CnbtmpInv, Cnbtmp, 3 * 3 * sizeof(float64));
//	Matrixinv(CnbtmpInv,3);
//	MatrixAdd(Cnbtmp,3,3, CnbtmpInv, Cnb);
//	NumMultiMatrix(0.5, Cnb,3,3, Cnb);

}*/

//参考严恭敏PINS工具箱
/*粗对准函数*/
void align_coarse(Vect wm, Vect vm, pearth earth_p,float64 *Cnb)
{
	Vect mean_wm, mean_vm,pos,vn,vectn1,vectn2,vectb1,vectb2,vb1,vb2,vntmp,vbtmp,vnntmp,vbbtmp;
	float64 x, y, z,nw,nv,b1,b2,n1,n2;
//	float64 *matN, *matB, *matNI,*Cnbtmp, *CnbtmpInv;
	int8u n;

	mean_wm = mean_vm = pos = vn = vectn1 = vectn2 = vectb1 = vectb2 = vb1 = vb2 = vntmp = vbtmp = vnntmp = vbbtmp = {0,0,0};
	x= y= z= nw= nv=  b1= b2= n1= n2=0;

	float64 matN[3 * 3] = { 0 }; float64 matB[3 * 3] = { 0 }; float64 matNI[3 * 3] = { 0 }; float64 Cnbtmp[3 * 3] = { 0 };float64 CnbtmpInv[3 * 3] = { 0 };

	vectn1 = earth_p->gn; vectn2 = earth_p->wnie; 
	vectb1 = vm;     vectb2 = wm;
	Vect_mul(vectb1, -1.0, &vectb1);
//	Vect_mul(mean_vm, -1.0 / nts, &vectb1); Vect_mul(mean_vm, 1.0 / nts, &vectb2);

	Vect_norm(vectn1, &n1); Vect_norm(vectn2, &n2); Vect_norm(vectb1, &b1); Vect_norm(vectb2, &b2);
	Vect_mul(vectb1, n1 / b1, &vb1); Vect_mul(vectb2, n2 / b2, &vb2);
//	Vect_mul(vectb1, 1.0 / b1, &vb1); Vect_mul(vectb2, 1.0 / b2, &vb2);
	
	Vect_cross(vectn1, vectn2, &vntmp); Vect_cross(vb1, vb2, &vbtmp);
	Vect_cross(vntmp, vectn1, &vnntmp); Vect_cross(vbtmp, vb1, &vbbtmp);

	matN[0 * 3 + 0] = vectn1.x; matN[0 * 3 + 1] = vectn1.y; matN[0 * 3 + 2] = vectn1.z;
	matN[1 * 3 + 0] = vntmp.x;  matN[1 * 3 + 1] = vntmp.y;  matN[1 * 3 + 2] = vntmp.z;
	matN[2 * 3 + 0] = vnntmp.x; matN[2 * 3 + 1] = vnntmp.y; matN[2 * 3 + 2] = vnntmp.z;

	matB[0 * 3 + 0] = vb1.x;    matB[0 * 3 + 1] = vb1.y;    matB[0 * 3 + 2] = vb1.z;
	matB[1 * 3 + 0] = vbtmp.x;  matB[1 * 3 + 1] = vbtmp.y;  matB[1 * 3 + 2] = vbtmp.z;
	matB[2 * 3 + 0] = vbbtmp.x; matB[2 * 3 + 1] = vbbtmp.y; matB[2 * 3 + 2] = vbbtmp.z;

	Matrixinv(matN, 3);
	MatrixMultiply(Cnbtmp,matN,3,3,matB,3,3);

	//正交化
	memcpy(CnbtmpInv, Cnbtmp, 3 * 3 * sizeof(float64));
	Matrixinv(CnbtmpInv, 3);
	MatrixAdd(Cnbtmp, 3, 3, CnbtmpInv, Cnb);
	NumMultiMatrix(0.5, Cnb,3,3, Cnb);

}

/*捷联惯导初始化
  获取姿态、位置、速度初值
*/
void sins_init(Vect atti0, Vect pos0,Vect vel0,psins sinst)
{
	float64 Cne[3 * 3] = { 0 };

	sinst->atti = atti0;
	sinst->pos = pos0;
	sinst->vel = vel0;

	Cne[0 * 3 + 0] = -sin(pos0.y); Cne[0 * 3 + 1] = cos(pos0.y);
	Cne[1 * 3 + 0] = -sin(pos0.x)*cos(pos0.y); Cne[1 * 3 + 1] = -sin(pos0.x)*sin(pos0.y); Cne[1 * 3 + 2] = cos(pos0.x);
	Cne[2 * 3 + 0] = cos(pos0.x)*cos(pos0.y); Cne[2 * 3 + 1] = cos(pos0.x)*sin(pos0.y); Cne[2 * 3 + 1] = sin(pos0.x);

	att2cnb(atti0, sinst->Cnb);
	att2q(atti0, &sinst->qnb);
	memcpy(sinst->Cne, Cne, 3 * 3 * sizeof(float64));
}

void sins_init1(float64 *Cnb,quat qnb0,Vect atti0, Vect pos0, Vect vel0, psins sinst)
{
	float64 Cne[3 * 3] = { 0 };
//	sinst->Cnb = Cnb;
	memcpy(sinst->Cnb, Cnb, 3 * 3 * sizeof(float64));
	sinst->pos = pos0;
	sinst->vel = vel0;
	Cne[0 * 3 + 0] = -sin(pos0.y); Cne[0 * 3 + 1] = cos(pos0.y);
	Cne[1 * 3 + 0] = -sin(pos0.x)*cos(pos0.y); Cne[1 * 3 + 1] = -sin(pos0.x)*sin(pos0.y); Cne[1 * 3 + 2] = cos(pos0.x);
	Cne[2 * 3 + 0] = cos(pos0.x)*cos(pos0.y); Cne[2 * 3 + 1] = cos(pos0.x)*sin(pos0.y); Cne[2 * 3 + 1] = sin(pos0.x);


	sinst->qnb = qnb0;
	sinst->atti = atti0;
	memcpy(sinst->Cne, Cne, 3 * 3 * sizeof(float64));

	sinst->tauG = 1000;
	sinst->tauA = 1000;
}

void static qupdt1(quat qnb0, Vect rv_ib, quat *qo)
{
	float64 n2, n, n_2, rv_ib0, s;
	quat qb1 = { 0 };

	n2 = rv_ib.x*rv_ib.x + rv_ib.y*rv_ib.y + rv_ib.z*rv_ib.z;

	if (n2 < 1.0e-12)
	{
		rv_ib0 = 1 - n2*(1 / 8 - n2 / 384); s = 1 / 2 - n2*(1 / 48 - n2 / 3840);
	}
	else
	{
		n = sqrt(n2);
		n_2 = n / 2;
		rv_ib0 = cos(n_2); s = sin(n_2) / n;
	}
	rv_ib.x = rv_ib.x*s; rv_ib.y = rv_ib.y*s; rv_ib.z = rv_ib.z*s;

	qb1.q0 = qnb0.q0*rv_ib0 - qnb0.q1*rv_ib.x - qnb0.q2*rv_ib.y - qnb0.q3*rv_ib.z;
	qb1.q1 = qnb0.q0*rv_ib.x + qnb0.q1*rv_ib0 + qnb0.q2*rv_ib.z - qnb0.q3*rv_ib.y;
	qb1.q2 = qnb0.q0*rv_ib.y + qnb0.q2*rv_ib0 + qnb0.q3*rv_ib.x - qnb0.q1*rv_ib.y;
	qb1.q3 = qnb0.q0*rv_ib.z + qnb0.q3*rv_ib0 + qnb0.q1*rv_ib.y - qnb0.q2*rv_ib.x;

	*qo = qb1;
}

void static qupdt2(quat qnb0,Vect rv_ib,Vect rv_in,quat *qo)
{
	float64 n2,n,n_2,rv_ib0,rv_in0,s;
	quat qb1,qnb1;
	qb1 = qnb1 = { 0 };

	n2 = rv_ib.x*rv_ib.x + rv_ib.y*rv_ib.y + rv_ib.z*rv_ib.z;
	
	if (n2 < 1.0e-12)
	{
		rv_ib0 = 1 - n2*(1 / 8 - n2 / 384); s = 1 / 2 - n2*(1 / 48 - n2 / 3840);
	}
	else
	{
		n = sqrt(n2);
		n_2 = n / 2;
		rv_ib0 = cos(n_2); s = sin(n_2) / n;
	}
	rv_ib.x = rv_ib.x*s; rv_ib.y = rv_ib.y*s; rv_ib.z = rv_ib.z*s;

	qb1.q0 = qnb0.q0*rv_ib0  - qnb0.q1*rv_ib.x - qnb0.q2*rv_ib.y - qnb0.q3*rv_ib.z;
	qb1.q1 = qnb0.q0*rv_ib.x + qnb0.q1*rv_ib0 + qnb0.q2*rv_ib.z - qnb0.q3*rv_ib.y;
	qb1.q2 = qnb0.q0*rv_ib.y + qnb0.q2*rv_ib0 + qnb0.q3*rv_ib.x - qnb0.q1*rv_ib.z;
	qb1.q3 = qnb0.q0*rv_ib.z + qnb0.q3*rv_ib0 + qnb0.q1*rv_ib.y - qnb0.q2*rv_ib.x;
	
	//rv2q(-rv_in)
	n2 = rv_in.x*rv_in.x + rv_in.y*rv_in.y + rv_in.z*rv_in.z;
	if (n2 < 1.0e-12)
	{
		rv_in0 = 1 - n2*(1 / 8 - n2 / 384); s =-1 / 2 + n2*(1 / 48 - n2 / 3840);
	}
	else
	{
		n = sqrt(n2);
		n_2 = n / 2;
		rv_in0 = cos(n_2); s = -sin(n_2) / n;
	}
	rv_in.x = rv_ib.x*s; rv_in.y = rv_ib.y*s; rv_in.z = rv_ib.z*s;

	qnb1.q0 = rv_in0*qb1.q0 - rv_in.x*qb1.q1 - rv_in.y*qb1.q2 - rv_in.z*qb1.q3;
	qnb1.q1 = rv_in0*qb1.q1 + rv_in.x*qb1.q0 + rv_in.y*qb1.q3 - rv_in.z*qb1.q2;
	qnb1.q2 = rv_in0*qb1.q2 + rv_in.y*qb1.q0 + rv_in.z*qb1.q1 - rv_in.x*qb1.q3;
	qnb1.q3 = rv_in0*qb1.q3 + rv_in.z*qb1.q0 + rv_in.x*qb1.q2 - rv_in.y*qb1.q1;
	
	*qo = qnb1;

	n2 = qnb1.q0*qnb1.q0 + qnb1.q1*qnb1.q1 + qnb1.q2*qnb1.q2 + qnb1.q3*qnb1.q3;
	if (n2>1.000001 || n2 < 0.999999)   qnormlz(qnb1, qo);
}
//姿态更新函数,采用四元数更新法
void attupdate(earth_para earth_p, Vect phim, float64 nts, psins sinst)
{
	quat qnb0, q0_phim, qnb, qnb_rv_wnin, qnb_phim;
	Vect atti, wnin, rv_wnin;
	float64 Cnb[3 * 3] = { 0 };

	qnb0=q0_phim=qnb=qnb_rv_wnin=qnb_phim = { 0 };
	qnb0 = sinst->qnb;

	wnin = earth_p.wnin;//导航系相对于惯性系的旋转角速度
	Vect_mul(wnin, nts, &rv_wnin);  //求导航系相对于惯性系的等效旋转矢量

/*	rv2q(rv_wnin, &qnb_rv_wnin); //导航系相对于惯性系姿态变化四元数
	rv2q(phim, &qnb_phim);//载体系相对于惯性系姿态变化四元数

	//Cnb四元数更新
	qmul(qnb0, qnb_phim, &q0_phim);
	qmul(qnb_rv_wnin, q0_phim, &qnb);
//	qmul(qnb0, qnb_phim, &sinst->qnb);
	qnormlz(qnb, &sinst->qnb);*/

	qupdt2(qnb0, phim, rv_wnin, &sinst->qnb);
//	qupdt1(qnb0, phim, &sinst->qnb);

	attsyn1(sinst->qnb, Cnb, &atti);
	sinst->atti = atti;
	memcpy(sinst->Cnb, Cnb, 3 * 3 * sizeof(float64));

}
//速度更新
void velupdate(earth_para earth_p, Vect fn, float64 nts, psins sinst)
{
	Vect an, an1, rv, vn0, vn, dv;
	quat qnb0;
	float64 nts_2 = nts / 2.0;

	an = an1 = rv = vn0 = vn = dv = { 0, 0, 0 };
	qnb0 = { 0, 0, 0, 0 };

	vn0 = sinst->vel;
	qnb0 = sinst->qnb;

//	qmulv(qnb0, fb, &sinst->fn);//比力转换到导航系
	Vect_mul(earth_p.wnin, -nts_2, &rv);//导航系等效旋转矢量
	rotv(rv, sinst->fn, &an1);

	an1 = fn;
	Vect_add(an1, earth_p.gcc, &an);//加速度
	Vect_mul(an, nts, &dv);
	Vect_add(vn0, dv, &vn);

//	memcpy(&sinst->vel, &vn, sizeof(Vect));
	sinst->vel = vn;
	sinst->an = an;
}
//位置更新
void posupdate(earth_para earth_p,float64 nts,Vect vn1, psins sinst)
{
	Vect vn_2, vn0, vn01;
	Vect pos0;

	float64 Mpv[3 * 3] = { 0 }; float64 Mpvn[3] = { 0 }; float64 Mpvnt[3] = { 0 };float64 Mpvvnt[3] = { 0 };
	
	Mpv[0 * 3 + 1] = 1.0 / earth_p.RMh; Mpv[1 * 3 + 0] = 1.0 / earth_p.RNh; Mpv[2 * 3 + 2] = 1;
	vn0 = sinst->vel;
	Vect_add(vn0, vn1, &vn01);
	Vect_mul(vn01, 1.0 / 2, &vn_2);
	Mpvn[0] = vn_2.x; Mpvn[1] = vn_2.y; Mpvn[2] = vn_2.z;
	NumMultiMatrix(nts, Mpvn,3,1, Mpvnt);
	MatrixMultiply(Mpvvnt,Mpv,3,3, Mpvnt,3,1);

	sinst->pos.x += Mpvvnt[0]; sinst->pos.y += Mpvvnt[1]; sinst->pos.z += Mpvvnt[2];
	memcpy(sinst->Mpv, Mpv, 3 * 3 * sizeof(float64));
}

void posupdate1(earth_para earth_p, float64 nts, Vect vn1, psins sinst)
{
	Vect vn_2, vn0, vn01;
	

	float64 Mpv[3 * 3] = { 0 }; float64 Mpvn[3] = { 0 }; float64 Mpvnt[3] = { 0 }; float64 Mpvvnt[3] = { 0 };

	Mpv[0 * 3 + 1] = -1.0 / earth_p.RMh; Mpv[1 * 3 + 0] = 1.0 / earth_p.RNh; Mpv[2 * 3 + 0] = tan(earth_p.lat)/earth_p.RNh;
	vn0 = sinst->vel;
	Vect_add(vn0, vn1, &vn01);
	Vect_mul(vn01, 1.0 / 2, &vn_2);
	Mpvn[0] = vn_2.x; Mpvn[1] = vn_2.y; Mpvn[2] = vn_2.z;
	NumMultiMatrix(nts, Mpvn, 3, 1, Mpvnt);
	MatrixMultiply(Mpvvnt, Mpv, 3, 3, Mpvnt, 3, 1);

	sinst->pos.x += Mpvvnt[0]; sinst->pos.y += Mpvvnt[1]; sinst->pos.z += Mpvvnt[2];
	memcpy(sinst->Mpv, Mpv, 3 * 3 * sizeof(float64));
}


/*捷联惯导更新函数
*/
void sins_update(Vect *wm, Vect *vm, int16u nSamples,float64 ts, psins sinst,pearth earthp)
{
	float64 nts, nts2;
	Vect phim, dvbm,rv;
	Vect vn, pos,atti,vn01,pos01,an,dv,dp1;
	Vect web, wnb, wib, wbie,fb,wnin1,fn;
	quat qnb;
	earth_para earthp1 = { {0} };
	float64 dp[3] = { 0 }; float64 dpos[3] = { 0 };
	float64 Mpv[3 * 3] = { 0 };

	phim = dvbm = rv = vn = pos = vn01=pos01=an=dv= web = wnb = wib = wbie = fb = wnin1 = { 0, 0, 0 };

	nts = nSamples*ts; nts2 = nts / 2;

//	IMUupdate(wm, vm, nSamples, &phim, &dvbm);//计算等效旋转矢量、速率增量
	IMUupdate1(wm, vm, nSamples, &phim, &dvbm,2);

	an = sinst->an;
	vn = sinst->vel; pos = sinst->pos;qnb = sinst->qnb;
	memcpy(Mpv, sinst->Mpv, 3 * 3 * sizeof(float64));

	Vect_mul(an, nts2, &dv); Vect_add(vn, dv, &vn01);  //v01
	Vect_mul(vn01, nts2, &dp1); dp[0] = dp1.x; dp[1] = dp1.y; dp[2] = dp1.z;
	MatrixMultiply(dpos, Mpv, 3, 3, dp, 3, 1);
	pos01.x =pos.x+ dpos[0]; pos01.y =pos.y+ dpos[1]; pos01.z =pos.z+ dpos[2];   //pos01

//	vn01 = vn; pos01 = pos;
	earth_para_update(vn01, pos01, earthp); //地球参数更新
	earthp1 = *earthp;

	Vect_mul(phim, 1.0/nts, &wib);//计算载体系相对于惯性系的旋转角速度
	sinst->wib = wib;

	Vect_mul(dvbm, 1.0 / nts,&fb);//计算载体系相对于惯性系的比力
	sinst->fb = fb;

	qmulv(qnb, fb, &fn);
	sinst->fn = fn;
//	sinst->fn = fn=fb;

/*	MatrixTrans(Cnb, Cbn);
	Vect2matrix(&earthp->wnie, mat_wnie);//地球系相对于惯性的旋转角速度

	MatrixMul(Cbn,mat_wnie, mat_wbie);     //将地球系相对惯性系旋转角速度投影到载体系
	matrix2Vect(mat_wbie, &wbie);
	Vect_sub(&wib, &wbie, &web);    //载体系相对地球系的旋转角速度
	memcpy(&sinst->web, &web, sizeof(Vect)); //载体系相对于地球系的旋转角速度

	Vect_mul(&phim, 1.0 / 2, &rv);
	rv2cnb(&rv, mat_rv);
	MatrixMul(Cbn, mat_rv, Cnb_rv);
	MatrixTrans(Cnb_rv,Cnb_rvT);
	Vect2matrix(&earthp->wnin, mat_wnin);//导航系相对于惯性系的旋转角速度
	MatrixMul(Cnb_rvT, mat_wnin, mat_wnin1);
	matrix2Vect(mat_wnin, &wnin1);
	Vect_sub(&wib, &wnin1, &wnb);//载体系相对于导航系的旋转角速度*/

	//姿态更新
	attupdate(earthp1, phim, nts, sinst);

	Vect vn1 = sinst->vel;
	//速度更新
	velupdate(earthp1, fn, nts, sinst);

	//位置更新
	posupdate1(earthp1, nts, vn1,sinst);

}
/*计算状态转移矩阵元素Maa,Mav,Map,Mva,Mvv,Mvp,Mpv,Mpp
*/
void sins_etm(pearth earth_p, psins sinst, float64 *Maa, float64 *Mav, float64 *Map, float64 *Mva, float64 *Mvv, float64 *Mvp, float64 *Mpv, float64 *Mpp)
{
	float64 sl,sl2,cl, tl, scl,secl, secl2, wN, wU, vE, vN;
	float64 f_RMh, f_RNh, f_clRNh, f_RMh2, f_RNh2;
	Vect vn,_wnin,fn,wnien;

	vn = _wnin = fn = wnien = { 0, 0, 0 };
	float64 Avn[3 * 3] = { 0 }, Mp1[3 * 3] = { 0 }, Mp2[3 * 3] = { 0 }, Mp3[3 * 3] = { 0 }, Mv1[3 * 3] = { 0 }, Mv2[ 3 * 3 ] = { 0 };

	sl = sin(earth_p->lat); cl = cos(earth_p->lat); tl = sl / cl; scl = sl*cl,sl2=sl*sl;
	secl = 1.0 / cl; secl2 = secl*secl;

	vn = sinst->vel; fn = sinst->fn; wnien = earth_p->wnien;
	wN = earth_p->wnie.y; wU = earth_p->wnie.z; vE = vn.x; vN = vn.y;

	f_RMh = 1.0 / earth_p->RMh; f_RNh = 1.0 / earth_p->RNh; f_clRNh = 1.0 / (cl*earth_p->RNh);
	f_RMh2 = f_RMh*f_RMh; f_RNh2 = f_RNh*f_clRNh;

    askew(vn, Avn);
	Mp1[1*3+0] = -wU; Mp1[2*3+ 0] = wN; 
	Mp2[0*3+2] = vN*f_RMh2; Mp2[1*3+ 2] = -vE*f_RNh2; Mp2[2*3+ 0] = vE*secl2*f_RNh; Mp2[2*3+ 2] = -vE*tl*f_RNh2; 
	
	Vect_mul(earth_p->wnin, -1, &_wnin); askew(_wnin, Maa);//Maa
	Mav[0*3+ 1] = -f_RMh; Mav[1*3+ 0] = f_RNh; Mav[2*3+ 0] = tl*f_RNh;//Mav
	MatrixAdd(Mp1,3,3, Mp2, Map);//Map

	askew(fn,Mva);//Mva
	askew(wnien, Mv1);
	MatrixMultiply(Mv2,Avn,3,3, Mav,3,3); MatrixSub(Mv2,3,3, Mv1, Mvv);//Mvv
	

	MatrixAdd(Mp1,3,3, Map, Mp3);
	MatrixMultiply(Mvp,Avn,3,3,Mp3,3,3);//Mvp
	Mvp[2 * 3 + 0] = Mvp[2 * 3 + 0] - g0*(5.27094e-3 * 2 * scl + 2.32718e-5 * 4 * sl2*scl); Mvp[2 * 3 + 2] = Mvp[2 * 3 + 2] + 3.086e-6;
	Mpv[0 * 3 + 1] = f_RMh; Mpv[1 * 3 + 0] = f_clRNh; Mpv[2 * 3 + 2] = 1; //Mpv
	Mpp[0 * 3 + 2] = -vN*f_RMh2; Mpp[1 * 3 + 0] = vE*tl*f_clRNh; Mpp[1 * 3 + 2] = -vE*secl*f_RNh2;//Mpp

}

/*计算状态转移矩阵
//	Ft = [ Maa    Mav    Map    -ins.Cnb  O33
//         Mva    Mvv    Mvp     O33      ins.Cnb
//         O33    Mpv    Mpp     O33      O33
//         zeros(6,9)  diag(-1./[ins.tauG;ins.tauA]) ];
*/
void sins_setFt(pearth earth_p,psins sinst)
{
//	float64 *Maa, *Mav, *Map, *Mva, *Mvv, *Mvp, *Mpv, *Mpp, *Cnb;
	float64 Maa[3 * 3] = { 0 }; float64 Mav[3 * 3] = { 0 }; float64  Map[3 * 3] = { 0 }; float64 Mva[3 * 3] = { 0 }; float64 Mvv[3 * 3] = { 0 };
	float64 Mvp[3 * 3] = { 0 }; float64 Mpv[3 * 3] = { 0 }; float64 Mpp[3 * 3] = { 0 };float64 Cnb[3 * 3] = { 0 };

	sins_etm(earth_p, sinst, Maa, Mav, Map, Mva, Mvv, Mvp, Mpv, Mpp);

	//phi:行：0~2；列：1~14
	                                       ;  sinst->Ft[0*15+ 1] = Maa[0*3+ 1]       ;    sinst->Ft[0*15+ 2] = Maa[0*3+ 2]     ;                                                          
    sinst->Ft[1*15+ 0] = Maa[1*3+ 0]       ;                                         ;    sinst->Ft[1*15+ 2] = Maa[1*3+ 2]     ;    
	sinst->Ft[2 * 15 + 0] = Maa[2 * 3 + 0] ;  sinst->Ft[2 * 15 + 1] = Maa[2 * 3 + 1] ;
	
	
	                                       ;   sinst->Ft[0*15+ 4] = Mav[0*3+ 1]      ;                                         ;
	sinst->Ft[1*15+ 3] = Mav[1*3+ 0]       ;                                         ;                                         ;
	sinst->Ft[2 * 15 + 3] = Mav[2 * 3 + 0];                                          ;                                         ;

	                                      ;                                          ;    sinst->Ft[0*15+ 8] = Map[0*3+ 2]     ;
	sinst->Ft[1*15+ 6] = Map[1*3+ 0]      ;                                          ;    sinst->Ft[1*15+ 8] = Map[1*3+ 2]     ;
	sinst->Ft[2 * 15 + 6] = Map[2 * 3 + 0];                                          ;   sinst->Ft[2 * 15 + 8] = Map[2 * 3 + 2];

	sinst->Ft[0 * 15 + 9] = -Cnb[0 * 3 + 0]; sinst->Ft[0 * 15 + 10] = -Cnb[0 * 3 + 1]; sinst->Ft[0 * 15 + 11] = -Cnb[0 * 3 + 2];
	sinst->Ft[1 * 15 + 9] = -Cnb[1 * 3 + 0]; sinst->Ft[1 * 15 + 10] = -Cnb[1 * 3 + 1]; sinst->Ft[1 * 15 + 11] = -Cnb[1 * 3 + 2];
	sinst->Ft[2 * 15 + 9] = -Cnb[2 * 3 + 0]; sinst->Ft[2 * 15 + 10] = -Cnb[2 * 3 + 1]; sinst->Ft[2 * 15 + 11] = -Cnb[2 * 3 + 2];

	//行：3~5 列：0~14
	                                       ; sinst->Ft[3*15+ 1] = Mva[0*3+ 1]         ; sinst->Ft[3*15+ 2] = Mva[0*3+ 2] ;
	sinst->Ft[4*15+ 0] = Mva[1*3+ 0]       ;                                          ; sinst->Ft[4*15+ 2] = Mva[1*3+ 2] ;
	sinst->Ft[5 * 15 + 0] = Mva[2 * 3 + 0] ; sinst->Ft[5 * 15 + 1] = Mva[2 * 3 + 1];

	sinst->Ft[3 * 15 + 3] = Mvv[0 * 3 + 0] ; sinst->Ft[3 * 15 + 4] = Mvv[0 * 3 + 1]   ; sinst->Ft[3 * 15 + 5] = Mvv[0 * 3 + 2];
	sinst->Ft[4 * 15 + 3] = Mvv[1 * 3 + 0] ; sinst->Ft[4 * 15 + 4] = Mvv[1 * 3 + 1]   ; sinst->Ft[4 * 15 + 5] = Mvv[1 * 3 + 2];
	sinst->Ft[5 * 15 + 3] = Mvv[2 * 3 + 0] ; sinst->Ft[5 * 15 + 4] = Mvv[2 * 3 + 1]   ;

	sinst->Ft[3 * 15 + 6] = Mvp[0 * 3 + 0] ;                                          ; sinst->Ft[3 * 15 + 8] = Mvp[0 * 3 + 2];
	sinst->Ft[4 * 15 + 6] = Mvp[1 * 3 + 0] ;                                          ; sinst->Ft[4 * 15 + 8] = Mvp[1 * 3 + 2];
	sinst->Ft[5*15+ 6] = Mvp[2*3+ 0];                                                 ; sinst->Ft[5*15+ 8] = Mvp[2*3+ 2];

	sinst->Ft[3 * 15 + 12] = Cnb[0 * 3 + 0]; sinst->Ft[3 * 15 + 13] = Cnb[0 * 3 + 1]  ; sinst->Ft[3 * 15 + 14] = Cnb[0 * 3 + 2];
	sinst->Ft[4 * 15 + 12] = Cnb[1 * 3 + 0]; sinst->Ft[4 * 15 + 13] = Cnb[1 * 3 + 1]  ; sinst->Ft[4 * 15 + 14] = Cnb[1 * 3 + 2];
	sinst->Ft[5 * 15 + 12] = Cnb[2 * 3 + 0]; sinst->Ft[5 * 15 + 13] = Cnb[2 * 3 + 1]  ; sinst->Ft[5 * 15 + 14] = Cnb[2 * 3 + 2];

	//行：6~8，列：0~14
	                                       ; sinst->Ft[6*15+ 4] = Mpv[0*3+ 1]         ;                                        ;
	sinst->Ft[7*15+ 3] = Mpv[1*3+ 0]       ;                                          ;                                        ;
	                                       ;                                          ; sinst->Ft[8*15+ 5] = Mpv[2*3+ 2]       ;

   
										   ;                                          ; sinst->Ft[6*15+ 8] = Mpp[0*3+ 2]       ;
    sinst->Ft[7*15+ 6] = Mpp[1*3+ 0]       ;                                          ; sinst->Ft[7*15+ 8] = Mpp[1*3+ 2]       ;
   
   //行：9~14，列：0~14
//	sinst->Ft[9 * 15 + 3] = -1.0 / sinst->tauG;
//	                                            sinst->Ft[10 * 15 + 4] = -1.0 / sinst->tauG;
//												                                              sinst->Ft[11 * 15 + 5] = -1.0 / sinst->tauG;
//  sinst->Ft[12 * 15 + 6] = -1.0 / sinst->tauA;
//												sinst->Ft[13 * 15 + 7] = -1.0 / sinst->tauA;
//																							  sinst->Ft[14 * 15 + 8] = -1.0 / sinst->tauA;


}
