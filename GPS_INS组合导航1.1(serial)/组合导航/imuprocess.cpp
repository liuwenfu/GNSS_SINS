#include "imuprocess.h"

int8u prefirst = 1;
//int8u vm_prefirst = 1;
Vect wm_1, vm_1;

static double conefactors[5][4] = { {0},			// coning coefficients
	{ 2. / 3 },										// 2
	{ 9. / 20, 27. / 20 },							// 3
	{ 54. / 105, 92. / 105, 214. / 105 },				// 4
	{ 250. / 504, 525. / 504, 650. / 504, 1375. / 504 }	// 5
};


/*IMU数据处理函数：由IMU角增量，速度增量计算圆锥误差、划桨误差
输入:  wm          角增量
       vm          速度增量
	   nSample     子样数
输出： phim        经过圆锥误差补偿的等效旋转矢量
       dvbm        经过划桨误差补偿的速度增量
*/
//多项式等效旋转矢量误差补偿
/*void IMUupdate1(const pVect wm, const pVect vm, int nSamples)
{
	int32s i;
	int8u prefirst = 1;

	Vect wm_1, vm_1, cm, sm, wmm, vmm;
	Vect phim, dvbm;

	float64 *pcf = conefactors[nSamples - 1];
	if (nSamples == 1)// one-plus-previous sample
	{
		if (prefirst == 1)
		{
			wm_1 = wm[0]; vm_1 = vm[0]; prefirst = 0;
		}
		wm_1 = wm[0]; vm_1 = vm[0];
	
		cm.x = 1.0 / 12 * wm_1.x; cm.y = 1.0 / 12 * wm_1.y; cm.z = 1.0 / 12 * wm_1.z;
		sm.x = 1.0 / 12 * vm_1.x; sm.y = 1.0 / 12 * vm_1.y; sm.z = 1.0 / 12 * vm_1.z;
	}
	if (nSamples > 1) prefirst = 1;
	for (i = 0; i<nSamples - 1; i++)
	{
		cm.x += pcf[i] * wm[i].x; cm.y += pcf[i] * wm[i].y; cm.z += pcf[i] * wm[i].z;
		sm.x += pcf[i] * vm[i].x; sm.y += pcf[i] * vm[i].y; sm.z += pcf[i] * vm[i].z;
		wmm.x += wm[i].x; wmm.y += wm[i].y; wmm.z += wm[i].z;
		vmm.x += vm[i].x; vmm.y += vm[i].y; vmm.z += vm[i].z;
	}
	wmm.x += wm[i].x; wmm.y += wm[i].y; wmm.z += wm[i].z;
	vmm.x += vm[i].x; vmm.y += vm[i].y; vmm.z += vm[i].z;
	phim.x = wmm.x + cm.x*wm[i].x; phim.y = wmm.y + cm.y*wm[i].y; phim.z = wmm.z + cm.z*wm[i].z;
	dvbm.x = vmm.x + 1.0 / 2 * wmm.x + vmm.x + (cm.x*vm[i].x + sm.x*wm[i].x);
	dvbm.y = vmm.y + 1.0 / 2 * wmm.y + vmm.y + (cm.y*vm[i].y + sm.y*wm[i].y);
	dvbm.z = vmm.z + 1.0 / 2 * wmm.z + vmm.z + (cm.z*vm[i].z + sm.z*wm[i].z);
}

/*求经过圆锥误差补偿后等效旋转矢量函数
    及经过旋转误差补偿、划桨误差补偿后的速度增量
	输入：wm            角增量
	     vm            速度增量
		 nSamples      子样数
	输出：phim          经误差校正后的等效旋转矢量
	     dvbm          经误差校正后的速度增量
*/
void extern IMUupdate(Vect *wm, Vect *vm, int16u nSamples, Vect *phim,Vect *dvbm)
{
	int32s i;
	Vect cm,cm_12, sm,sm_12,sm1,sm2, wmm, vmm,cmm,smm,wmp,vmp,smp;
	Vect dphim, scullm,rotm,rotm_2;//圆锥误差，划桨误差，旋转误差
	Vect dvbm1;
	float64 *pcf = conefactors[nSamples-1];

	cm = cm_12 = sm = sm_12 = sm1 = sm2 = wmm = vmm = cmm = smm = wmp = vmp = smp = dphim = scullm = rotm = rotm_2 = dvbm1 = {0,0,0};

	if (nSamples == 1)// one-plus-previous sample
	{
		if (prefirst == 1)
		{
			wm_1 = wm[0]; vm_1 = vm[0]; prefirst = 0; return;
		}
		Vect_cross(wm_1, wm[0], &cm);
		Vect_cross(wm_1, vm[0], &sm1);
		Vect_cross(vm_1, wm[0], &sm2);

		Vect_mul(cm, 1.0 / 12,&cm_12);
		dphim = cm_12;//圆锥误差
		Vect_add(sm1, sm2, &sm);
		Vect_mul(sm, 1.0 / 12,&sm_12);
		scullm = sm_12;//划桨误差
		wmm=wm_1 = wm[0]; vmm=vm_1 = vm[0];
	}
	else
	{
		for (i = 0; i < nSamples - 1; i++)
		{
			Vect_add_equ(&wmm, wm[i]);
			Vect_add_equ(&vmm, vm[i]);
			Vect_mul(wm[i], pcf[i],&wmp);
			Vect_mul(vm[i], pcf[i],&vmp);
			Vect_add_equ(&cmm, wmp); //cmm
			Vect_add_equ(&smm, vmp);//smm
		}
		Vect_cross(cmm, wm[i],&cm);
		dphim = cm;//圆锥误差
		Vect_cross(cmm, vm[i], &sm1);//sm1
		Vect_cross(smm, wm[i], &sm2);//sm2
		Vect_add(sm1, sm2, &sm);//sm
		scullm = sm;//划桨误差
		
		Vect_add_equ(&wmm, wm[i]);//角增量
		Vect_add_equ(&vmm, vm[i]);//速度增量
		
	}
	Vect_cross(wmm, vmm, &rotm);
	Vect_mul(rotm, 1.0 / 2,&rotm_2);//旋转误差
	Vect_add(wmm, dphim, phim);//phim为经过圆锥误差补偿的等效旋转矢量
	Vect_add(vmm, rotm_2, &dvbm1);
	Vect_add(dvbm1, scullm, dvbm);//dvbm为经过旋转误差补偿和划桨误差补偿后的速度增量
}

/* coneoptimal  0 多项式误差补偿
                1 优化的圆锥误差补偿
				2 单子样+前一子样
*/
void extern IMUupdate1(Vect *wm, Vect *vm, int16u nSamples, Vect *phim, Vect *dvbm, int16u coneoptimal)
{
	int32s i;
	Vect cm, cm_12, sm, sm_12, sm1, sm2, wmm, vmm, cmm, smm, wmp, vmp, smp;
	Vect dphim, scullm, rotm, rotm_2;//圆锥误差，划桨误差，旋转误差
	Vect dvbm1;
	float64 *pcf = conefactors[nSamples - 1];
	Vect cm01, cm02, cm03, cm12, cm13, cm23, cm24;
	Vect w0v1, w0v2,w0v3, w1v2,w1v3,w2v3, v0w1, v0w2, v0w3,v1w2,v1w3,v2w2,v2w3;
	Vect wm2a;

	cm = cm_12 = sm = sm_12 = sm1 = sm2 = wmm = vmm = cmm = smm = wmp = vmp = smp = dphim = scullm = rotm = rotm_2 = dvbm1 = { 0, 0, 0 };
	cm01 = cm02 = cm03 = cm12 = cm13 = cm23 =cm24= { 0 };
	w0v1 = w0v2 = w0v3 = w1v2 = w1v3 = w2v3 = v0w1 = v0w2 = v0w3 = v1w2 = v1w3 = v2w2 = v2w3 = wm2a={0};

	if (nSamples == 1)
	{
//		wmm = wm[0]; vmm = vm[0];
  		if (prefirst == 1)
		{
			wm_1 = wm[0]; vm_1 = vm[0]; prefirst = 0; *phim = wm[0]; *dvbm = vm[0];
			return;
		}
		
		if (coneoptimal == 0)
		{
			dphim = { 0 };//圆锥误差
			scullm = { 0 };//划桨误差
		}
		else if (coneoptimal == 1)
		{
			dphim = { 0 };//圆锥误差
			Vect_cross(wm_1, vm[0], &sm1);
			Vect_cross(vm_1, wm[0], &sm2);
			Vect_add(sm1, sm2, &sm);
			Vect_mul(sm, 1.0 / 12, &sm_12);
			scullm = sm_12;//划桨误差
		}
		else
		{
			Vect_cross(wm_1, wm[0], &cm);
			Vect_mul(cm, 1.0 / 12, &cm_12);
			dphim = cm_12;//圆锥误差

			Vect_cross(wm_1, vm[0], &sm1);
			Vect_cross(vm_1, wm[0], &sm2);
			Vect_add(sm1, sm2, &sm);
			Vect_mul(sm, 1.0 / 12, &sm_12);
			scullm = sm_12;//划桨误差
		}
		wmm = wm_1 = wm[0]; vmm = vm_1 = vm[0];
	}
	else
	{
		for (i = 0; i < nSamples - 1; i++)
		{
			Vect_add_equ(&wmm, wm[i]);
			Vect_add_equ(&vmm, vm[i]);
		}
		if (coneoptimal == 0)
		{
			for (i = 0; i < nSamples-1; i++)
			{
				Vect_mul(wm[i], pcf[i], &wmp);
				Vect_mul(vm[i], pcf[i], &vmp);
				Vect_add_equ(&cmm, wmp); //cmm
				Vect_add_equ(&smm, vmp);//smm
			}
			Vect_cross(cmm, wm[i], &cm);
			dphim = cm;//圆锥误差
			Vect_cross(cmm, vm[i], &sm1);//sm1
			Vect_cross(smm, wm[i], &sm2);//sm2
			Vect_add(sm1, sm2, &sm);//sm
			scullm = sm;//划桨误差

		}
		else if (coneoptimal == 1)
		{
			if (nSamples == 1)
			{
				dphim = { 0 };
				scullm = { 0 };
			}
			else if (nSamples == 2)
			{
				Vect_cross(wm[0], wm[1], &cm);
				Vect_mul(cm, 2.0 / 3, &cm);
				dphim = cm;//圆锥误差

				Vect_cross(wm[0], vm[1], &sm1);Vect_cross(vm[0], wm[1], &sm2);
				Vect_add(sm1, sm2, &sm);Vect_mul(sm, 2.0 / 3, &sm);
				scullm = sm;//划桨误差

			}
			else if(nSamples == 3)
			{
				Vect_cross(wm[0], wm[2], &cm02);
				Vect_mul(cm02, 33.0 / 80, &cm02);
				Vect_sub(wm[2], wm[0], &wm2a);
				Vect_cross(wm[1], wm2a, &cm12);
				Vect_mul(cm12, 57.0 / 80, &cm12);
				Vect_add(cm02, cm12, &dphim);//圆锥误差
	//			Vect_sub(dphim, wm[0], &dphim);

				Vect_cross(wm[0], vm[2], &w0v2);Vect_mul(w0v2, 33.0 / 80, &w0v2);
				Vect_cross(vm[0], wm[2], &v0w2);Vect_mul(v0w2, 33.0 / 80, &v0w2);
				Vect_cross(wm[0], vm[1], &w0v1);Vect_mul(w0v1, 57.0 / 80, &w0v1);
				Vect_cross(wm[1], vm[2], &w1v2);Vect_mul(w1v2, 57.0 / 80, &w1v2);
				Vect_cross(vm[0], wm[1], &v0w1);Vect_mul(v0w1, 57.0 / 80, &v0w1);
				Vect_cross(vm[1], wm[2], &v1w2);Vect_mul(v1w2, 57.0 / 80, &v1w2);
				Vect_add(w0v2, v0w2, &scullm);
				Vect_add_equ(&scullm, w0v1);
				Vect_add_equ(&scullm, w1v2);
				Vect_add_equ(&scullm, v0w1);
				Vect_add_equ(&scullm, v1w2);//划桨误差

			//	dphim = { 0 }; scullm = { 0 };
			}
			else
			{
				Vect_cross(wm[0], wm[1], &cm01); Vect_mul(cm01, 736.0 / 945, &cm01);
				Vect_cross(wm[2], wm[3], &cm23); Vect_mul(cm23, 736.0 / 945, &cm23);
				Vect_cross(wm[0], wm[2], &cm02); Vect_mul(cm02, 334.0 / 945, &cm02);
				Vect_cross(wm[1], wm[3], &cm13); Vect_mul(cm13, 334.0 / 945, &cm13);
				Vect_cross(wm[0], wm[3], &cm03); Vect_mul(cm03, 526.0 / 945, &cm03);
				Vect_cross(wm[1], wm[2], &cm12); Vect_mul(cm12, 654.0 / 945, &cm12);
				Vect_add(cm01, cm23, &dphim);
				Vect_add_equ(&dphim,cm02);
				Vect_add_equ(&dphim,cm13);
				Vect_add_equ(&dphim,cm03);
				Vect_add_equ(&dphim,cm12);//圆锥误差

				Vect_cross(wm[0], vm[1], &w0v1); Vect_mul(w0v1, 736.0 / 945, &w0v1);
				Vect_cross(wm[2], vm[3], &w2v3); Vect_mul(w2v3, 736.0 / 945, &w2v3);
				Vect_cross(vm[0], wm[1], &v0w1); Vect_mul(v0w1, 736.0 / 945, &v0w1);
				Vect_cross(vm[2], wm[3], &v2w3); Vect_mul(v2w3, 736.0 / 945, &v2w3);
				Vect_cross(wm[0], vm[2], &w0v2); Vect_mul(w0v2, 334.0 / 945, &w0v2);
				Vect_cross(wm[1], vm[3], &w1v3); Vect_mul(w1v3, 334.0 / 945, &w1v3);
				Vect_cross(vm[0], wm[2], &v0w2); Vect_mul(v0w2, 334.0 / 945, &v0w2);
				Vect_cross(vm[1], wm[3], &v1w3); Vect_mul(v1w3, 334.0 / 945, &v1w3);
				Vect_cross(wm[0], vm[3], &w0v3); Vect_mul(w0v3, 526.0 / 945, &w0v3);
				Vect_cross(vm[0], wm[3], &v0w3); Vect_mul(v0w3, 526.0 / 945, &v0w3);
				Vect_cross(wm[1], vm[2], &w1v2); Vect_mul(w1v2, 654.0 / 945, &w1v2);
				Vect_cross(vm[1], wm[2], &v1w2); Vect_mul(v1w2, 654.0 / 945, &v1w2);
				Vect_add(w0v1,w2v3,&scullm);
				Vect_add_equ(&scullm,v0w1);
				Vect_add_equ(&scullm,v2w3);
				Vect_add_equ(&scullm,w0v2);
				Vect_add_equ(&scullm,w1v3);
				Vect_add_equ(&scullm,v0w2);
				Vect_add_equ(&scullm,v1w3);
				Vect_add_equ(&scullm,w0v3);
				Vect_add_equ(&scullm,v0w3);
				Vect_add_equ(&scullm,w1v2);
				Vect_add_equ(&scullm,v1w2);

			}
		}
		else
		{
			if (nSamples == 1)
			{
				dphim = { 0 };
				scullm = { 0 };
			}
			else if (nSamples == 2)
			{
				Vect_cross(wm[0], wm[1], &cm);
				Vect_mul(cm, 2.0 / 3, &cm);
				dphim = cm;//圆锥误差

				Vect_cross(wm[0], vm[1], &sm1); Vect_cross(vm[0], wm[1], &sm2);
				Vect_add(sm1, sm2, &sm); Vect_mul(sm, 2.0 / 3, &sm);
				scullm = sm;//划桨误差

			}
			else if (nSamples == 3)
			{
				Vect_cross(wm[0], wm[1], &cm01); Vect_mul(cm01, 27.0 / 40, &cm01);
				Vect_cross(wm[0], wm[2], &cm02); Vect_mul(cm02, 9.0 / 20, &cm02);
				Vect_cross(wm[1], wm[2], &cm12); Vect_mul(cm12, 27.0 / 40, &cm12);
				Vect_add(cm01, cm02, &dphim);//圆锥误差
				Vect_add_equ(&dphim, cm12);

				Vect_cross(wm[0], vm[2], &w0v2); Vect_mul(w0v2, 33.0 / 80, &w0v2);
				Vect_cross(vm[0], wm[2], &v0w2); Vect_mul(v0w2, 33.0 / 80, &v0w2);
				Vect_cross(wm[0], vm[1], &w0v1); Vect_mul(w0v1, 57.0 / 80, &w0v1);
				Vect_cross(wm[1], vm[2], &w1v2); Vect_mul(w1v2, 57.0 / 80, &w1v2);
				Vect_cross(vm[0], wm[1], &v0w1); Vect_mul(v0w1, 57.0 / 80, &v0w1);
				Vect_cross(vm[1], wm[2], &v1w2); Vect_mul(v1w2, 57.0 / 80, &v1w2);
				Vect_add(w0v2, v0w2, &scullm);
				Vect_add_equ(&scullm, w0v1);
				Vect_add_equ(&scullm, w1v2);
				Vect_add_equ(&scullm, v0w1);
				Vect_add_equ(&scullm, v1w2);//划桨误差
			}
			else
			{
				Vect_cross(wm[2], wm[3], &cm23); Vect_mul(cm23, 232.0 / 315, &cm23);
				Vect_cross(wm[1], wm[3], &cm13); Vect_mul(cm13, 46.0 / 105, &cm13);
				Vect_cross(wm[0], wm[3], &cm03); Vect_mul(cm03, 18.0 / 35, &cm03);
				Vect_cross(wm[1], wm[2], &cm12); Vect_mul(cm12, 178.0 / 315, &cm12);
				Vect_cross(wm[0], wm[2], &cm02); Vect_mul(cm02, 46.0 / 105, &cm02);
				Vect_cross(wm[0], wm[1], &cm01); Vect_mul(cm01, 232.0 / 315, &cm01);
				Vect_add(cm23, cm13, &dphim);
				Vect_add_equ(&dphim, cm03);
				Vect_add_equ(&dphim, cm12);
				Vect_add_equ(&dphim, cm02);
				Vect_add_equ(&dphim, cm01);//圆锥误差

				Vect_cross(wm[0], vm[1], &w0v1); Vect_mul(w0v1, 736.0 / 945, &w0v1);
				Vect_cross(wm[2], vm[3], &w2v3); Vect_mul(w2v3, 736.0 / 945, &w2v3);
				Vect_cross(vm[0], wm[1], &v0w1); Vect_mul(v0w1, 736.0 / 945, &v0w1);
				Vect_cross(vm[2], wm[3], &v2w3); Vect_mul(v2w3, 736.0 / 945, &v2w3);
				Vect_cross(wm[0], vm[2], &w0v2); Vect_mul(w0v2, 334.0 / 945, &w0v2);
				Vect_cross(wm[1], vm[3], &w1v3); Vect_mul(w1v3, 334.0 / 945, &w1v3);
				Vect_cross(vm[0], wm[2], &v0w2); Vect_mul(v0w2, 334.0 / 945, &v0w2);
				Vect_cross(vm[1], wm[3], &v1w3); Vect_mul(v1w3, 334.0 / 945, &v1w3);
				Vect_cross(wm[0], vm[3], &w0v3); Vect_mul(w0v3, 526.0 / 945, &w0v3);
				Vect_cross(vm[0], wm[3], &v0w3); Vect_mul(v0w3, 526.0 / 945, &v0w3);
				Vect_cross(wm[1], vm[2], &w1v2); Vect_mul(w1v2, 654.0 / 945, &w1v2);
				Vect_cross(vm[1], wm[2], &v1w2); Vect_mul(v1w2, 654.0 / 945, &v1w2);
				Vect_add(w0v1, w2v3, &scullm);
				Vect_add_equ(&scullm, v0w1);
				Vect_add_equ(&scullm, v2w3);
				Vect_add_equ(&scullm, w0v2);
				Vect_add_equ(&scullm, w1v3);
				Vect_add_equ(&scullm, v0w2);
				Vect_add_equ(&scullm, v1w3);
				Vect_add_equ(&scullm, w0v3);
				Vect_add_equ(&scullm, v0w3);
				Vect_add_equ(&scullm, w1v2);
				Vect_add_equ(&scullm, v1w2);
			}
		}

		Vect_add_equ(&wmm, wm[i]);//角增量
		Vect_add_equ(&vmm, vm[i]);//速度增量
	}

	Vect_cross(wmm, vmm, &rotm);
	Vect_mul(rotm, 1.0 / 2, &rotm_2);//旋转误差
	Vect_add(wmm, dphim, phim);//phim为经过圆锥误差补偿的等效旋转矢量
	Vect_add(vmm, rotm_2, &dvbm1);
	Vect_add(dvbm1, scullm, dvbm);//dvbm为经过旋转误差补偿和划桨误差补偿后的速度增量
}
