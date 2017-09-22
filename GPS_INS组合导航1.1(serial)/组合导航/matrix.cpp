#include "matrix.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

//单位阵
void Matrix_I(int8u n,float64 *Imat)
{
	for (int8u i = 0; i < n; i++)
	{
		for (int8u j = 0; j < n; j++)
		{
			if (i == j)
				Imat[i*n + j] = 1.0;
			else
				Imat[i*n + j] = 0;
		}
	}
}
//角对称矩阵
void Matrix_diag(float64 *dig,int8u n,float64 *diag)
{
	for (int8u i = 0; i < n; i++)
	{
		for (int8u j = 0; j < n; j++)
		{
			if (i == j)
				diag[i*n + j] = dig[i];
			else
				diag[i*n + j] = 0;
		}
	}
}

//角对称矩阵
void Matrix_diag2(float64 *dig, int8u n, float64 *diag)
{
	for (int8u i = 0; i < n; i++)
	{
		for (int8u j = 0; j < n; j++)
		{
			if (i == j)
				diag[i*n + j] = dig[i]*dig[i];
			else
				diag[i*n + j] = 0;
		}
	}
}
//矩阵相加
int16u MatrixAdd(const float64 *clA,int16u row,int16u col, const float64 *clB, float64 *pclResult)
{
	int16u i = 0;                         //循环控制变量

	//如果返回矩阵指针为空则返回1
	if (pclResult == NULL)
	{
		return 1;
	}
	//两个矩阵相加
	for (i = 0; i < row * col; i++)
	{
		pclResult[i] = clA[i] + clB[i];
	}
	return 0;

}
//矩阵相减
int16u MatrixSub(const float64 *clA, int16u row,int16u col,const float64 *clB, float64 *pclResult)
{
	int16u i = 0;                         //循环控制变量

	//如果返回矩阵指针为空则返回1
	if (pclResult == NULL)
	{
		return 1;
	}
	//两个矩阵相减
	for (i = 0; i < row * col; i++)
	{
		pclResult[i] = clA[i] - clB[i];
	}
	return 0;
}

//矩阵转置
extern int MatrixTrans(const double *clA, int row, int colomn, double *pclDes)
{
	int i = 0;                        //循环变量
	int j = 0;                        //循环变量

	if (pclDes == NULL)
	{
		return 1;
	}
	//矩阵转置
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < colomn; j++)
		{
 			pclDes[i + j*row] = clA[i*colomn + j];
		}
	}
	//执行成功则返回0
	return 0;
}
//数乘矩阵
int16u NumMultiMatrix(const float64 dwNum, float64 *clMatrix, int16u row,int16u col,float64 *pclMatrix)
{
	int16u i = 0;                       //循环变量

	//判断指针是否为空
	if (pclMatrix == NULL)
	{
		//如果为空则返回为1
		return 1;
	}
	for (i = 0; i <row * col; i++)
	{
		pclMatrix[i] = clMatrix[i] * dwNum;
	}
	return 0;
}

//矩阵乘法
extern int MatrixMultiply(float64 *matrixOut, float64 *lhs, int lrow, int lcolomn, float64 *rhs, int rrow, int rcolomn)
{	//断定左边矩阵的列数与右边矩阵的行数相等
	if (lcolomn != rrow) return 0;
	//生成矩阵新对象，用lhs的行作为新阵的行数，用rhs的列数作为新阵的列数
	float64 *matrix_temp = (float64 *)calloc(lrow*rcolomn,sizeof(float64));
//	float64 matrix_temp[20 * 20] = { 0.0 };

//	for (int i = 0; i < lrow; i++)
//	for (int j = 0; j < rcolomn; j++)
//		matrix_temp[i*lcolomn + j] = 0;

	for (int i = 0; i < lrow; i++)
	{
		for (int j = 0; j < rcolomn; j++)
		{
			for (int k = 0; k < lcolomn; k++)
			{
				matrix_temp[i*rcolomn + j] +=lhs[i*lcolomn + k] * rhs[k*rcolomn + j];
			}		
		}
	}

	for (int i = 0; i < lrow; i++)
	for (int j = 0; j < rcolomn; j++)
		matrixOut[i*rcolomn + j] = matrix_temp[i*rcolomn + j];		//将最后结果转放入mOut矩阵中

	free(matrix_temp);
	return 1;		//返回结果矩阵mOut
}

extern int *imat(int n, int m)
{
	int *p;

	if (n <= 0 || m <= 0) return NULL;
	if (!(p = (int *)malloc(sizeof(int)*n*m))) {
		return NULL;
	}
	return p;
};

extern double *mat(int n, int m)
{
	double *p;

	if (n <= 0 || m <= 0) return NULL;
	if (!(p = (double *)malloc(sizeof(double)*n*m))) {
		return NULL;
	}
	return p;
}

extern void matcpy(double *A, const double *B, int n, int m)
{
	memcpy(A, B, sizeof(double)*n*m);
}

/* LU decomposition ----------------------------------------------------------*/
static int ludcmp(double *A, int n, int *indx, double *d)
{
	double big, s, tmp, *vv = mat(n, 1);
	int i, imax = 0, j, k;

	*d = 1.0;
	for (i = 0; i<n; i++)
	{
		big = 0.0;
		for (j = 0; j<n; j++)
		if ((tmp = fabs(A[i + j*n]))>big)
			big = tmp;
		if (big>0.0)
			vv[i] = 1.0 / big;
		else
		{
			free(vv); return -1;
		}
	}
	for (j = 0; j<n; j++)
	{
		for (i = 0; i<j; i++)
		{
			s = A[i + j*n]; for (k = 0; k<i; k++) s -= A[i + k*n] * A[k + j*n]; A[i + j*n] = s;
		}
		big = 0.0;
		for (i = j; i<n; i++)
		{
			s = A[i + j*n];
			for (k = 0; k<j; k++)
				s -= A[i + k*n] * A[k + j*n];
			A[i + j*n] = s;
			if ((tmp = vv[i] * fabs(s)) >= big)
			{
				big = tmp; imax = i;
			}
		}
		if (j != imax)
		{
			for (k = 0; k<n; k++)
			{
				tmp = A[imax + k*n]; A[imax + k*n] = A[j + k*n]; A[j + k*n] = tmp;
			}
			*d = -(*d); vv[imax] = vv[j];
		}
		indx[j] = imax;
		if (A[j + j*n] == 0.0)
		{
			free(vv); return -1;
		}
		if (j != n - 1)
		{
			tmp = 1.0 / A[j + j*n]; for (i = j + 1; i<n; i++) A[i + j*n] *= tmp;
		}
	}
	free(vv);
	return 0;
}

/* LU back-substitution ------------------------------------------------------*/
static void lubksb(const double *A, int n, const int *indx, double *b)
{
	double s;
	int i, ii = -1, ip, j;

	for (i = 0; i<n; i++) {
		ip = indx[i]; s = b[ip]; b[ip] = b[i];
		if (ii >= 0) for (j = ii; j<i; j++) s -= A[i + j*n] * b[j]; else if (s) ii = i;
		b[i] = s;
	}
	for (i = n - 1; i >= 0; i--) {
		s = b[i]; for (j = i + 1; j<n; j++) s -= A[i + j*n] * b[j]; b[i] = s / A[i + i*n];
	}
}

extern int Matrixinv(double *A, int n)
{
	double d, *B;
	int i, j, *indx;

	indx = imat(n, 1); B = mat(n, n); matcpy(B, A, n, n);
	if (ludcmp(B, n, indx, &d))
	{
		free(indx); free(B); return -1;
	}
	for (j = 0; j<n; j++)
	{
		for (i = 0; i<n; i++) A[i + j*n] = 0.0; A[j + j*n] = 1.0;
		lubksb(B, n, indx, A + j*n);
	}
	free(indx); free(B);
	return 0;
}