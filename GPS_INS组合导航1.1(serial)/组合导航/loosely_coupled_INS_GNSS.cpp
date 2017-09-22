#include "loosely_coupled_INS_GNSS.h"
#include "Sins.h"
#include "const.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

bool kalmanfilter_init_flag = false;

/*gnss_ins松耦合函数
|input:

|output:
*/

/*滤波器初始化 
  系统状态方程：dX=F*X+G*w              //X为状态向量：3个失准角误差参数，3个速度误差参数，3个位置误差参数，3个陀螺常值飘移参数，3个加速度计零偏参数
  量测方程：Z=H*X+v                     //量测观测值：GPS解算位置与SINS位置之差，GPS解算速度与SINS速度之差

  kalman滤波方程：
  X(k/k-1)=Φ(k/k-1)*X(k-1)                                          //状态一步预测
  P(k/k-1)=Φ(k/k-1)*P(k-1)*Φ(k/k-1)T+G(k-1)*Q(k-1)*G(K-1)T          //状态协方差一步预测
  K(k)=P(k/k-1)*H(k)T*[H(k)*P(k/k-1)*H(k)T+R(k)]-1                  //增益
  X(k)= X(k/k-1)+K(k)*[Z(k)-H(k)*X(k/k-1)]                          //状态更新
  P(k)=[I-K(k)*H(k)]*P(k/k-1)                                       //状态协方差更新
*/

int16s filter_init(S_Kalmanfilter *kf,sins *sinst,float64 nts)
{
	float64 dig1[3 * 3] = { 0 };
	float64 dig2[3 * 3] = { 0 };

	
	//状态初值
//	float64 xk[DIMSTATE] = { 2.5e-5, 2.5e-5, 5e-5, 0.01, 0.01, 0.01, 0.1/earth_Re, 0.1/earth_Re, 0.1, 2.5e-6, 2.5e-6, 5e-6, 0.05, 0.5, 0.1 };
	for (int16u i = 0; i < DIMSTATE; i++) 	kf->Xk[i] = 0;//kf->Xk[i] = xk[i];

	//状态初始协方差阵

//	float64 dig_Pk[DIMSTATE] = { 2.5e-4, 5e-4, 1.5e-4, 2.0, 2.0, 3.0, 2.0 / earth_Re, 2.0 / earth_Re, 3.0, 0.5, 0.3, 0.5, 0.2, 0.2, 0.3 };
	float64 dig_Pk[DIMSTATE] = { 2.0e-2, 2.0e-2, 2.0e-2, 0.001, 0.001, 0.02, 2.0/ earth_Re, 2.0 / earth_Re,2.0, 0.02, 0.01, 0.01, 0.001, 0.001, 0.02};
	Matrix_diag2(dig_Pk, DIMSTATE, kf->Pk);

	//状态过程噪声协方差阵 
	float64 dig_Qt[6] = { 0.02, 0.01, 0.006, 0.001, 0.001, 0.02 };
	Matrix_diag2(dig_Qt, 6, kf->Qt);
	NumMultiMatrix(nts, kf->Qt, 6, 6, kf->Qt);

	//量测噪声
	float64 dig_RK1[DIMMEA] = { 0.1, 0.1, 0.01, 2.0 / earth_Re, 2.0 / earth_Re, 2.0 };
	Matrix_diag2(dig_RK1, DIMMEA, kf->Rk1);
//	NumMultiMatrix(nts, kf->Rk1, 6, 6, kf->Rk1);

	//计算量测系数矩阵
	//	MatrixZero(&kf->Hk1, DIMMEA, DIMSTATE);
	for (int8u i = 0; i < DIMMEA; i++)
	{
		for (int8u j = i + 3; j < DIMSTATE; j++)
		{
			if (i + 3 == j)
			{
				kf->Hk1[i*DIMSTATE + j] = 1.0;
				break;
			}
		}
	}

	for (int16u i = 0; i < 3; i++)
	{
		for (int16u j = 0; j < 3; j++)
		{
			if (i == j)
			{
				dig1[i * 3 + j] = kf->Qt[i * DIMMEA + j];
				dig2[i * 3 + j] = kf->Qt[(i + 3) * DIMMEA + j + 3];
			}
		}
	}

	for (int16u i = 0; i < 3; i++)
	{
		for (int16u j = 0; j < 3; j++)
		{
			kf->Qk[i * 15 + j] = dig1[i * 3 + j];
			kf->Qk[(i + 3) * 15 + j + 3] = dig2[i * 3 + j];
		}
	}

//	sinst->vel.x += kf->Xk[3]; sinst->vel.y += kf->Xk[4]; sinst->vel.z += kf->Xk[5];
//	sinst->pos.x += kf->Xk[6]; sinst->pos.y += kf->Xk[7]; sinst->pos.z += kf->Xk[8];

	//VS 2017
	//状态初值
/*	for (int16u i = 0; i < DIMSTATE; i++) kf->Xk[i] = 0.0;

	//状态初始协方差阵
	float64 dig_Pk[DIMSTATE] = { 10.0*deg, 10.0*deg, 10.0*deg, 1.0, 1.0, 1.0, 100.0 / earth_Re, 100.0 / earth_Re, 100.0,
		100.0*dph, 100.0*dph, 100.0*dph, 10.0*mg, 10.0*mg, 10.0*mg };
	Matrix_diag2(dig_Pk, DIMSTATE, kf->Pk);

	//状态过程噪声协方差阵 
//	float64 dig_Qt[DIMSTATE] = { 1.1*dpsh, 1.1*dpsh, 1.1*dpsh, 500.0*ugpsHz, 500.0*ugpsHz, 500.0*ugpsHz, 0.0, 0.0, 0.0,
//		0.0, 0.0, 0.0, 100.0*ugpsh, 100.0*ugpsh, 100.0*ugpsh};
	float64 dig_Qt[6] = { 1.1*dpsh, 1.1*dpsh, 1.1*dpsh, 500.0*ugpsHz, 500.0*ugpsHz, 500.0*ugpsHz };
	Matrix_diag2(dig_Qt, 6, kf->Qt);
	NumMultiMatrix(nts, kf->Qt, 6, 6, kf->Qt);

	//量测噪声
	float64 dig_RK1[DIMMEA] = { 0.5, 0.5, 0.5, 10.0 / earth_Re, 10.0 / earth_Re, 10.0 };
	Matrix_diag2(dig_RK1, DIMMEA, kf->Rk1);*/


	//PINS
	//状态初值
/*	kf->Xk[0] = 30.0*sec; kf->Xk[1] = -30.0*sec; kf->Xk[2] = 20.0*min;
	kf->Xk[3] = 0.1; kf->Xk[4] = 0.1; kf->Xk[5] = 0.1;
	kf->Xk[6] = 1.0 / earth_Re; kf->Xk[7] = 1.0 / earth_Re; kf->Xk[8] = 3;
	kf->Xk[9] = kf->Xk[10]=kf->Xk[11]= 0.03 / hur;
	kf->Xk[12] = kf->Xk[13] = kf->Xk[14] = 100*ug;

	//状态初始协方差阵
	float64 dig_Pk[DIMSTATE] = { 10.0*30.0*sec, 10.0*30.0*sec, 10.0*20.0*min, 1.0, 1.0, 1.0, 10.0 / earth_Re, 10.0 / earth_Re, 30.0,
		10.0*0.03 / hur, 10.0*0.03 / hur, 10.0*0.03 / hur, 1.0*mg, 1.0*mg, 1.0*mg };
	Matrix_diag2(dig_Pk, DIMSTATE, kf->Pk);

	//状态过程噪声协方差阵 
//	float64 dig_Qt[DIMSTATE] = { 1.1*dpsh, 1.1*dpsh, 1.1*dpsh, 500.0*ugpsHz, 500.0*ugpsHz, 500.0*ugpsHz, 0.0, 0.0, 0.0,
//		0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };	
	float64 dig_Qt[DIMSTATE] = { 1.0e-3 / 60, 1.0e-3 / 60, 1.0e-3 / 60, 5.0*ugpsHz, 5.0*ugpsHz, 5.0*ugpsHz };
	Matrix_diag2(dig_Qt, 6, kf->Qt);
	NumMultiMatrix(nts, kf->Qt, 6, 6, kf->Qt);

	//量测噪声
	float64 dig_RK1[DIMMEA] = { 0.01, 0.01, 0.01, 10.0*10.0 / earth_Re / earth_Re, 10.0*10.0 / earth_Re / earth_Re, 30.0*30.0 };
	Matrix_diag2(dig_RK1, DIMMEA, kf->Rk1);*/

	return 1;
}

//量测噪声allan方差实时估计
void allan_var(S_Kalmanfilter *kf, int32u iTer, float64 *Zk1)
{
	float64 dig_Rk1[DIMMEA] = { 0 };
	float64 beta1 = 0;
	
//	beta1 = beta1 / (beta1 +1.0);

	for (int16u i = 0; i < DIMMEA; i++)
	{
		for (int16u j = 0; j < DIMMEA; j++)
		{
			if (i == j)
			{
				dig_Rk1[i] = kf->Rk1[i * DIMMEA + j];
				break;
			}
		}

	//allan方差
		beta1 = 1.0 / (iTer - 1);
		dig_Rk1[i] = (1.0 - beta1)*dig_Rk1[i] + 1.0 / 2.0 * beta1*(kf->Zk1[i] - Zk1[i])*(kf->Zk1[i] - Zk1[i]);
   
	}
	Matrix_diag(dig_Rk1, DIMMEA, kf->Rk1);
}

void filter_update(S_Kalmanfilter *kf, char *updatemode, bool adaptive,float64 beta)
{
//	char *updatemode;

//	float64 *Fkk1T, *Fkk1Pk, *Fkk1PkFkk1T,*k1H, *dP, *I;
//	float64 *Hk1T, *Pxykk_1, *Py0, *Pykk_1
//	float64 *dx, *rk, *ykk_1;
	
	float64 Fkk1T[DIMSTATE*DIMSTATE] = { 0 }; float64 Fkk1Pk[DIMSTATE*DIMSTATE] = { 0 }; float64 Fkk1PkFkk1T[DIMSTATE*DIMSTATE] = { 0 };
	float64 k1H[DIMSTATE*DIMSTATE] = { 0 }; float64 dP[DIMSTATE*DIMSTATE] = { 0 };float64 I[DIMSTATE*DIMSTATE] = { 0 };
	float64 Hk1T[DIMSTATE*DIMMEA] = { 0 }; float64 Pxykk_1[DIMSTATE*DIMMEA] = { 0 }; float64 Py0[DIMMEA*DIMMEA] = { 0 };
	float64 Pykk_1[DIMMEA*DIMMEA] = { 0 };float64 dx[DIMSTATE] = { 0 }; float64 rk[DIMMEA] = { 0 };float64 ykk_1[DIMMEA] = { 0 };
	float64 PkT[DIMSTATE*DIMSTATE] = { 0 }; float64 P[DIMSTATE*DIMSTATE] = { 0 };

	float64 ry[DIMMEA] = { 0 };

//	if (zk == NULL) updatemode = "T"; //时间更新
//	else updatemode = "B"; //时间更新和量测更新

	if (updatemode == "T")
	{
		MatrixMultiply(kf->Xkk1, kf->Fkk1, DIMSTATE, DIMSTATE, kf->Xk, DIMSTATE, 1); //X(k1/k)
		MatrixMultiply(Fkk1Pk, kf->Fkk1, DIMSTATE, DIMSTATE, kf->Pk, DIMSTATE, DIMSTATE);
		MatrixTrans(kf->Fkk1, DIMSTATE, DIMSTATE, Fkk1T);
		MatrixMultiply(Fkk1PkFkk1T,Fkk1Pk,DIMSTATE,DIMSTATE,Fkk1T,DIMSTATE,DIMSTATE);
		MatrixAdd(Fkk1PkFkk1T, DIMSTATE, DIMSTATE, kf->Qk, kf->Pkk1);//P(k1/k)

	//	kf->Xk[DIMSTATE] = kf->Xkk1[DIMSTATE];
	//	kf->Pk[DIMSTATE*DIMSTATE] = kf->Pkk1[DIMSTATE*DIMSTATE];
		memcpy(kf->Xk, kf->Xkk1, sizeof(float64)*DIMSTATE);
		memcpy(kf->Pk, kf->Pkk1, sizeof(float64)*DIMSTATE*DIMSTATE);
	}
	else
	{
		MatrixMultiply(kf->Xkk1, kf->Fkk1, DIMSTATE, DIMSTATE, kf->Xk, DIMSTATE, 1); //X(k1/k)
		MatrixMultiply(Fkk1Pk, kf->Fkk1, DIMSTATE, DIMSTATE, kf->Pk, DIMSTATE, DIMSTATE);
		MatrixTrans(kf->Fkk1, DIMSTATE, DIMSTATE, Fkk1T);
		MatrixMultiply(Fkk1PkFkk1T,Fkk1Pk,DIMSTATE,DIMSTATE,Fkk1T,DIMSTATE,DIMSTATE);
		MatrixAdd(Fkk1PkFkk1T, DIMSTATE, DIMSTATE, kf->Qk, kf->Pkk1);//P(k1/k)

		MatrixTrans(kf->Hk1, DIMMEA, DIMSTATE, Hk1T);
		MatrixMultiply(kf->Pxykk_1, kf->Pkk1, DIMSTATE, DIMSTATE, Hk1T, DIMSTATE, DIMMEA);//P(k1/k)*H(k1)T     15x6
		MatrixMultiply(kf->Py0, kf->Hk1, DIMMEA, DIMSTATE, kf->Pxykk_1, DIMSTATE, DIMMEA);//H(k1)*P(k/k-1)*H(k1)T     6x6
		MatrixMultiply(ykk_1, kf->Hk1, DIMMEA, DIMSTATE, kf->Xkk1, DIMSTATE, 1);//H(k1)*X(k1/k)      6x1
//		MatrixSub(kf->Zk1, DIMMEA, 1, ykk_1, rk);//Zk-Hk*X(k-1,k)         6x1
		MatrixSub(kf->Zk1, DIMMEA, 1, ykk_1, kf->rk);//Z(k1)-H(k1)*X(k1/k)         6x1

		if (adaptive == true)
		{
			for (int16u i = 0; i < DIMMEA; i++)
			{
				for (int16u j = 0; j < DIMMEA; j++)
				{
					if (i == j)
					{
						kf->Rk1[i*DIMMEA + j] = (1 - beta)*kf->Rk1[i*DIMMEA + j] + beta*(kf->rk[i] * kf->rk[i] - kf->Py0[i*DIMMEA + j]);
						break;
					}
				}
			}
		}

		MatrixAdd(kf->Py0, DIMMEA, DIMMEA, kf->Rk1, Pykk_1); //H(k1)*P(k1/k)*H(k1)T+R(k1)  6x6
		memcpy(kf->Pykk_1, Pykk_1, sizeof(float64)*DIMMEA*DIMMEA);
		Matrixinv(Pykk_1, DIMMEA);   //6x6 [H(k1)*P(k1/k)*H(k1)T+R(k1)]-1
		MatrixMultiply(kf->K1, kf->Pxykk_1, DIMSTATE, DIMMEA, Pykk_1, DIMMEA, DIMMEA);//增益  15x6   P(k1/k)*H(k1)T*[H(k1)*P(k1/k)*H(k1)T+R(k1)]-1 

		MatrixMultiply(kf->dx, kf->K1, DIMSTATE, DIMMEA, kf->rk, DIMMEA, 1);         //15x1  K1*[Z(k1)-H(k1)*X(k1/k)]
		MatrixAdd(kf->Xkk1, DIMSTATE, 1, kf->dx, kf->Xk);   //Xk更新

		MatrixMultiply(k1H, kf->K1, DIMSTATE, DIMMEA, kf->Hk1, DIMMEA, DIMSTATE);   //15x15  K1*H(k1)
		Matrix_I(DIMSTATE, I);//形成单位阵
		MatrixSub(I,DIMSTATE,DIMSTATE,k1H,dP);   //I-K1*H(k1)
		MatrixMultiply(kf->Pk, dP, DIMSTATE, DIMSTATE, kf->Pkk1, DIMSTATE, DIMSTATE); //Pk更新

		//Pk对称化
		MatrixTrans(kf->Pk, DIMSTATE, DIMSTATE, PkT);
		MatrixAdd(kf->Pk, DIMSTATE, DIMSTATE, PkT, P);
		NumMultiMatrix(1.0 / 2, P, DIMSTATE, DIMSTATE, kf->Pk);

	}

}

void filter_feedback(S_Kalmanfilter *kf, psins sinst, SinsFilterOpt *sinsfilter_opt)
{
	quat qnb0,qnb,dqnb;
	Vect vel0,pos0,phi,dvel,dpos,dwb,deb,atti;

	float64 Cnb[3 * 3] = { 0 };

	qnb0 = sinst->qnb; vel0 = sinst->vel; pos0 = sinst->pos;
	phi.x = -kf->Xk[0];  phi.y = -kf->Xk[1];   phi.z = -kf->Xk[2];  //失准角误差
	dvel.x = -kf->Xk[3]; dvel.y = -kf->Xk[4]; dvel.z = -kf->Xk[5]; //速度误差
	dpos.x = -kf->Xk[6]; dpos.y = -kf->Xk[7]; dpos.z = -kf->Xk[8];//位置误差
	dwb.x = -kf->Xk[9];  dwb.y = -kf->Xk[10];  dwb.z = -kf->Xk[11];//陀螺仪零偏误差
	deb.x = -kf->Xk[12]; deb.y = -kf->Xk[13]; deb.z = -kf->Xk[14];//加速度计零偏

	//姿态反馈更新
	rv2q(phi, &dqnb);
	qmul(qnb0, dqnb, &qnb);
//	q2att(qnb, &atti); q2cnb(qnb, Cnb);
	attsyn1(qnb, Cnb, &atti);
	sinst->qnb = sinsfilter_opt->qnb=qnb;
	sinst->atti = sinsfilter_opt->atti = atti;
	memcpy(sinst->Cnb, Cnb, 3 * 3 * sizeof(float64));

	//速度反馈更新
	Vect_add(vel0, dvel, &sinsfilter_opt->vel);
	sinst->vel = sinsfilter_opt->vel;

	//位置反馈更新
	Vect_add(pos0, dpos, &sinsfilter_opt->pos);
	sinst->pos = sinsfilter_opt->pos;

//	kf->Xk[5] = 0; kf->Xk[8] = 0;

}

int8u loosely_coupled_GNSS_INS(pearth earth_p, psins sinst, nmeaINFO *info, float64 nts, S_Kalmanfilter *kf, SinsFilterOpt *sinsfilter_opt, char *kf_updatemode, int32u *iTer_allan, float64 *beta)
{
	bool adaptive = false;
	
//	float64 nts=1 * ts;
//	float64	Zk[6] = {0};
	float64 Cnb[3 * 3] = { 0 };
	float64 CnbT[3 * 3] = { 0 };
	float64 Cnb1[3 * 3] = { 0 };
	float64 Cnb2[3 * 3] = { 0 };
	float64 CCnb1[3 * 3] = { 0 };
	float64 CCnb2[3 * 3] = { 0 };
	float64 dig1[3 * 3] = { 0 };
	float64 dig2[3 * 3] = { 0 };

	float64 Zk1[6] = { 0 };

	if (kalmanfilter_init_flag == false)
	{
		filter_init(kf, sinst,nts);//滤波器初始化
		kalmanfilter_init_flag = true;
	//	*iTer_allan = 1;
	//	return 1;
	}

	float64 I[DIMSTATE * DIMSTATE] = { 0 }; float64 Ft[DIMSTATE * DIMSTATE] = { 0 }; float64 Fts[DIMSTATE * DIMSTATE] = { 0 };
	float64 F[DIMSTATE * DIMSTATE] = { 0 }; float64 Qt[DIMSTATE * DIMSTATE] = { 0 };
	
	//  Fk=I+Ft*nts
	//	Qk=Qt*nts
	
	sins_setFt(earth_p, sinst);
	NumMultiMatrix(nts, sinst->Ft,DIMSTATE,DIMSTATE, Fts);
	Matrix_I(DIMSTATE, I);
	MatrixAdd(I,DIMSTATE,DIMSTATE, Fts, F); //计算状态转移矩阵Fk
	memcpy(&kf->Fkk1, &F, DIMSTATE*DIMSTATE*sizeof(float64));

   //计算状态过程噪声Qk
/*	for (int16u i = 0; i < 3; i++)
	{
		for (int16u j = 0; j < 3; j++)
		{
			if (i == j)
			{
				dig1[i * 3 + j] = kf->Qt[i * 6 + j];
				dig2[i * 3 + j] = kf->Qt[(i + 3) * 6 + j + 3];
			}		
		}
	}
	memcpy(Cnb, sinst->Cnb, sizeof(float64) * 3 * 3);
	MatrixTrans(Cnb, 3, 3, CnbT);
	MatrixMultiply(Cnb1, Cnb, 3, 3, dig1, 3, 3);
	MatrixMultiply(CCnb1, Cnb1, 3, 3, CnbT, 3, 3);
	MatrixMultiply(Cnb2, Cnb, 3, 3, dig2, 3, 3);
	MatrixMultiply(CCnb2, Cnb2, 3, 3, CnbT, 3, 3);

	for (int16u i = 0; i < 3; i++)
	{
		for (int16u j = 0; j < 3; j++)
		{
			kf->Qk[i * 15 + j] = CCnb1[i * 3 + j];
			kf->Qk[(i+3) * 15 + j+3] = CCnb2[i * 3 + j];
		}
	}

	for (int16u i = 0; i < 3; i++)
	{
		for (int16u j = 0; j < 3; j++)
		{
			kf->Qk[i * 15 + j] = dig1[i * 3 + j];
			kf->Qk[(i + 3) * 15 + j + 3] = dig2[i * 3 + j];
		}
	}*/

	if (kf_updatemode == "B")
	{
		//保存上一时刻观测值
		for (int16u i = 0; i < DIMMEA; i++)
		{
			Zk1[i] = kf->Zk1[i];
		}

		//计算当前时刻观测值Zk
		kf->Zk1[0] = sinst->vel.x - info->speed*sin(info->direction);
		kf->Zk1[1] = sinst->vel.y - info->speed*sin(info->direction);
		kf->Zk1[2] = sinst->vel.z -0;
	//	kf->Zk1[0] = 0;
	//	kf->Zk1[1] = 0;
	//	kf->Zk1[2] = 0;
		kf->Zk1[3] = sinst->pos.x - info->lat;
		kf->Zk1[4] = sinst->pos.y - info->lon;
		kf->Zk1[5] = sinst->pos.z - info->elv;

		*iTer_allan = *iTer_allan + 1;

		//计算量测噪声allan方差
		if (*iTer_allan == 1) *beta = 1.0;
		else
		{
			allan_var(kf, *iTer_allan, Zk1);
		//	*beta = *beta / (*beta + 1.0);
		//	adaptive = true;
		}
	}

	filter_update(kf, kf_updatemode, adaptive,*beta); //滤波更新

	if (kf_updatemode=="B")
	    filter_feedback(kf, sinst, sinsfilter_opt);
	else
	{
		sinsfilter_opt->atti = sinst->atti;
		sinsfilter_opt->pos = sinst->pos;
		sinsfilter_opt->vel = sinst->vel;
	}

	return 2;

}