#include "stdafx.h"
#include <math.h>
#include "UserMath.h"

#ifdef USE_USER_RANDOM
#include <stdlib.h>
#endif


#ifdef USE_USER_VALUE_LIMIT
#include "MathConstant.h"
#endif

#ifdef USE_USER_VECTOR
double dot(const double  x1[3],const double  x2[3])
{
	 return (x1[0]*x2[0]+x1[1]*x2[1]+x1[2]*x2[2]);
}
void cross(const  double  x1[3],const double  x2[3],double  x3[3])
{
     x3[0]=x1[1]*x2[2]-x1[2]*x2[1];
     x3[1]=x1[2]*x2[0]-x1[0]*x2[2];
	 x3[2]=x1[0]*x2[1]-x1[1]*x2[0];
}
double GetLength( const double  x1[3])
{
	 return (sqrt(x1[0]*x1[0]+x1[1]*x1[1]+x1[2]*x1[2]));
}
void Unit(double X[3])
{
	unsigned char i;
	double length = GetLength(X);
	if ( length > 0.0 )
		for(i=0;i<3;i++) X[i] /= length;
	else
		for(i=0;i<3;i++) X[i] = 0.0;
}
#endif

#ifdef USE_USER_QUATERNION
void QuaterMultiply(const double  In1[4],const double  In2[4],double  Out[4])
{
	Out[0] = In1[0]*In2[0]-In1[1]*In2[1]-In1[2]*In2[2]-In1[3]*In2[3];
	Out[1] = In1[0]*In2[1]+In1[1]*In2[0]+In1[2]*In2[3]-In1[3]*In2[2];
	Out[2] = In1[0]*In2[2]-In1[1]*In2[3]+In1[2]*In2[0]+In1[3]*In2[1];
	Out[3] = In1[0]*In2[3]+In1[1]*In2[2]-In1[2]*In2[1]+In1[3]*In2[0];
}
void QuaternionTransfer(double Q[4])
{
	unsigned char i = 0;
	for(i=1;i<4;i++) Q[i] = -Q[i];
}	
void QuaterDivided(const double  In1[4],const double  In2[4],double  Out[4])
{
	unsigned char i = 0;
	double In3[4] = {0};
	for(i=0;i<4;i++)	In3[i] = In2[i];
	QuaternionTransfer(In3);
	QuaterMultiply(In1,In3,Out);
}
void QuaterConjugateMultiply(const double  In1[4],const double  In2[4],double  Out[4])
{
	unsigned char i = 0;
	double In3[4] = {0};
	for(i=0;i<4;i++)	In3[i] = In1[i];
	QuaternionTransfer(In3);
	QuaterMultiply(In3,In2,Out);
}
void QuaterUnit(double In[4])
{
	unsigned char i = 0;
	double temp  = 0.0;
	temp = sqrt(In[0]*In[0]+In[1]*In[1]+In[2]*In[2]+In[3]*In[3]);
	if ( temp > 0.0 )
	{
		for(i=0;i<4;i++)	In[i] /= temp;
	}
	else
	{
		In[0] = 1.0;
		for(i=1;i<4;i++)	In[i] = 0.0;
	}
}
void GetQuaternionMatrix4X3(const double In[4], double Out[4][3])
{
	Out[0][0] = -In[1];	Out[0][1] = -In[2];	Out[0][2] = -In[3];
	Out[1][0] =  In[0];	Out[1][1] = -In[3];	Out[1][2] =  In[2];
	Out[2][0] =  In[3];	Out[2][1] =  In[0];	Out[2][2] = -In[1];
	Out[3][0] = -In[2];	Out[3][1] =  In[1];	Out[3][2] =  In[0];
}
#endif

#ifdef USE_USER_VALUE_LIMIT
double	fsign(const double X)
{
	if ( X >= 0 ) return 1.0;
	else	return -1.0;
}
double	fLimit(const double x, const double LimitValue)
{
	if ( fabs(x) < LimitValue )
		return x;
	else
		return LimitValue*fsign(x);
}
double fLimitInPI(const double X)
{
	unsigned short iNo = 0;
	double Y = 0.0;
	iNo = (unsigned short) floor(fabs(fabs(X)-PI)/2.0/PI);
	if ( X >  PI )
		Y = X - 2*PI*(iNo+1);
	else if ( X < -PI )
		Y = X + 2*PI*(iNo+1);
	else
		Y = X;
	return Y;
}
double fLimitInOne(const double X)
{
	if ( X > 1.0 ) return 1.0;
	if ( X < -1.0 ) return -1.0;
	return X;
}
#endif

#ifdef USE_USER_RAW_TO_ENGINEERING
double	RawToEngineering(unsigned char iLen, unsigned long uiValue,double ValueRange)
{
	double Range;
	Range = pow(2.0,iLen)-1;
	if (uiValue == (unsigned long)(Range)/2 )
		return 0.0;
	return ( -ValueRange + uiValue/Range*2*ValueRange);
}
double	EngineeringToRaw(double Value, double ValueRange, unsigned char iLen)
{
	double Range,Percent;
	if ( fabs(Value) > ValueRange ) Value = fsign(Value)*ValueRange;
	Range 	= pow(2.0,iLen)-1;
	Percent = 0.5*Value/ValueRange+0.5;
	return Percent*Range;
}
double	RawToEngineeringRange(unsigned char iLen, unsigned long uiValue,double MinValue, double MaxValue)
{
	double Range,Value;
	Range = pow(2.0,iLen)-1;
	Value = MinValue + uiValue/Range*(MaxValue-MinValue);
	return Value;
}
double	EngineeringToRawRange(double Value, double MinValue, double MaxValue, unsigned char iLen)
{
	double Range,Percent;
	if ( Value < MinValue ) Value = MinValue;
	if ( Value > MaxValue ) Value = MaxValue;
	Range 	= pow(2.0,iLen)-1;
	Percent = (Value-MinValue)/(MaxValue-MinValue);
	return Percent*Range;
}
#endif

#ifdef USE_USER_FRAME
void RotateFrame(const double  R[3][3], double  Vec[3])
{
	unsigned char  i,j;
	double  VTmp[3];

	for (i=0;i<3;i++) VTmp[i] = Vec[i];
	for (i=0;i<3;i++) 
	{
		Vec[i] = 0.0;
		for (j=0;j<3;j++) 
		{
			Vec[i] += R[i][j]*VTmp[j];
		}
	}
}
void InverseRotateFrame(const double  R[3][3],double  Vec[3])
{
	unsigned char i,j;
	double  VTmp[3];

	for (i=0;i<3;i++) VTmp[i] = Vec[i];
	for (i=0;i<3;i++) 
	{
		Vec[i] = 0.0;
		for (j=0;j<3;j++) 
		{
			Vec[i] += R[j][i]*VTmp[j];
		}
	}
}
void GetRoi(const double R[3],const double V[3], double Mat[3][3])
{
	unsigned char i = 0;
	double x[3],y[3],z[3];
	double tmp = 0.0;
	tmp = GetLength(R);
	if ( tmp <= 0.0 )  return;
	for (i=0;i<3;i++)
	{
	    z[i] = -R[i]/tmp;
		x[i] = V[i];
	}
	cross(z,x,y);
	Unit(y);
	cross(y,z,x);
	for (i=0;i<3;i++)
	{
		  Mat[0][i] = x[i];
		  Mat[1][i] = y[i];
		  Mat[2][i] = z[i];
	}
}
void GetRbi(const double Q[4],double R[3][3])
{
	R[0][0] = Q[0]*Q[0]+Q[1]*Q[1]-Q[2]*Q[2]-Q[3]*Q[3];
	R[0][1] = 2*(Q[1]*Q[2]+Q[0]*Q[3]);
	R[0][2] = 2*(Q[1]*Q[3]-Q[0]*Q[2]);
	R[1][0] = 2*(Q[1]*Q[2]-Q[0]*Q[3]);
	R[1][1] = Q[0]*Q[0]-Q[1]*Q[1]+Q[2]*Q[2]-Q[3]*Q[3];
	R[1][2] = 2*(Q[2]*Q[3]+Q[0]*Q[1]);
	R[2][0] = 2*(Q[1]*Q[3]+Q[0]*Q[2]);
	R[2][1] = 2*(Q[2]*Q[3]-Q[0]*Q[1]);
	R[2][2] = Q[0]*Q[0]-Q[1]*Q[1]-Q[2]*Q[2]+Q[3]*Q[3];
}
void GetRei(const double Ag,double R[3][3])
{
    R[0][0] = cos(Ag);
	R[0][1] = sin(Ag);
	R[0][2] = 0;
	R[1][0] = -sin(Ag);
	R[1][1] = cos(Ag);
	R[1][2] = 0;
	R[2][0] = 0;
	R[2][1] = 0;
	R[2][2] = 1;
}
void GetRvi(const double A, const double B, double R[3][3])  
{
	R[0][0] = -cos(A)*sin(B);
	R[0][1] = -sin(A)*sin(B);
	R[0][2] = cos(B);
	R[1][0] = -sin(A);
	R[1][1] = cos(A);
	R[1][2] = 0;
	R[2][0] = -cos(B)*cos(A);
	R[2][1] = -sin(A)*cos(B);
	R[2][2] = -sin(B);
}
#endif

#ifdef USE_USER_MATRIX
void MatrixProduct(const unsigned long iN, const unsigned long iM, const unsigned long iK, const double * A1, const double * A2, double * A3)
{
  unsigned long  i,j,k;

  for (i=0;i<iN;i++) 
  {
	  for(k=0;k<iK;k++)
	  {
		  A3[i*iK+k] = 0;
		  for (j=0;j<iM;j++)
		  {
			  A3[i*iK+k] += A1[i*iM+j]*A2[j*iK+k];
		  }
      }
  }
}
void MatrixProductTranspose(const unsigned long iN, const unsigned long iM, const unsigned long iK, const double * A1, const double * A2, double * A3)
{
  unsigned long   i,j,k;

  for (i=0;i<iN;i++) 
  {
	  for(k=0;k<iK;k++)
	  {
		  A3[i*iK+k] = 0;
		  for (j=0;j<iM;j++)
		  {
			  A3[i*iK+k] += A1[i*iM+j]*A2[k*iM+j];
		  }
      }
  }
}
void MatrixTranspose(const unsigned long iN, const unsigned long iM, const double * A1, double * A2)
{
  unsigned long   i,j;

  for (i=0;i<iN;i++) 
  {
	  for(j=0;j<iM;j++)
	  {
		  A2[j*iN+i] = A1[i*iM+j];
      }
  }
}
void MatrixDeterminant( const double A[3][3], double * ReturnValue)
{
	* ReturnValue =	A[0][0]*A[1][1]*A[2][2] 
					+ A[0][1]*A[1][2]*A[2][0] 
					+ A[0][2]*A[1][0]*A[2][1] 
					- A[0][2]*A[1][1]*A[2][0] 
					- A[0][1]*A[1][0]*A[2][2] 
					- A[0][0]*A[1][2]*A[2][1];
}
signed char MatrixInverse3X3(const double A1[3][3] ,double A2[3][3])
{
	double detA1 = 0.0;
	MatrixDeterminant(A1,&detA1);
	if(fabs(detA1) < 0.01)		return -1;
	A2[0][0] = ( A1[1][1]*A1[2][2] - A1[1][2]*A1[2][1])/detA1;
	A2[1][0] = -( A1[1][0]*A1[2][2] - A1[1][2]*A1[2][0])/detA1;
	A2[2][0] = ( A1[1][0]*A1[2][1] - A1[1][2]*A1[2][0])/detA1;

	A2[0][1] = -( A1[0][1]*A1[2][2] - A1[0][2]*A1[2][1])/detA1;
	A2[1][1] = ( A1[0][0]*A1[2][2] - A1[0][2]*A1[2][0])/detA1;
	A2[2][1] = -( A1[0][0]*A1[2][1] - A1[0][1]*A1[2][0])/detA1;

	A2[0][2] = ( A1[0][1]*A1[1][2] - A1[0][2]*A1[1][2])/detA1;
	A2[1][2] = -( A1[0][0]*A1[1][2] - A1[0][2]*A1[1][0])/detA1;
	A2[2][2] = ( A1[0][0]*A1[1][1] - A1[0][1]*A1[1][0])/detA1;
	return 0;
}
#endif

#ifdef USE_USER_EULAR_TO_MATRIX
void EularYXZToMatrix(const struct SEularAngle *pE,double  R[3][3])
{
	double S1,C1,S2,C2,S3,C3;
	S1 = sin(pE->Roll);
	C1 = cos(pE->Roll);
	S2 = sin(pE->Pitch);
	C2 = cos(pE->Pitch);
	S3 = sin(pE->Yaw);
	C3 = cos(pE->Yaw);

	R[0][0] =  C2*C3 - S1*S2*S3;
	R[0][1] =  C3*S1*S2 + C2*S3;
	R[0][2] = -(C1*S2);
	R[1][0] = -(C1*S3);
	R[1][1] =  C1*C3;
	R[1][2] =  S1;
	R[2][0] =  C3*S2 + S1*C2*S3;
	R[2][1] = -(C2*C3*S1)  + S2*S3;
	R[2][2] =  C1*C2;

}
void Eular313ToMatrix(const struct SEularAngle  *pE,double  dtRotationMatrix[3][3])
{
	double Sin1,Cos1,Sin2,Cos2,Sin3,Cos3;
	Sin1 = sin(pE->Roll);
	Cos1 = cos(pE->Roll);
	Sin2 = sin(pE->Pitch);
	Cos2 = cos(pE->Pitch);
	Sin3 = sin(pE->Yaw);
	Cos3 = cos(pE->Yaw);

	//R[0][0] =  Cos2*Cos3;
	//R[0][1] =  Cos2*Sin3;
	//R[0][2] = -Sin2;
	//R[1][0] =  Sin1*Sin2*Cos3 - Cos1*Sin3;
	//R[1][1] =  Sin1*Sin2*Sin3 + Cos1*Cos3;
	//R[1][2] =  Sin1*Cos2;
	//R[2][0] =  Sin1*Sin3+Cos1*Sin2*Cos3;
	//R[2][1] = -Sin1*Cos3+Cos1*Sin2*Sin3;
	//R[2][2] =  Cos1*Cos2;

	dtRotationMatrix[0][0] = Cos1 * Cos2;
	dtRotationMatrix[0][1] = Cos1 * Sin2 *	Sin3 - Sin1* Cos3;
	dtRotationMatrix[0][2] = Cos1*Sin2 * Cos3+Sin1*Sin3;

	dtRotationMatrix[1][0] = Sin1*Cos2;
	dtRotationMatrix[1][1] = Sin1*Sin2*Sin3+Cos1*Cos3;
	dtRotationMatrix[1][2] = Sin1*Sin2*Cos3-Cos1*Sin3;
	dtRotationMatrix[2][0] = -Sin2;
	dtRotationMatrix[2][1] = Cos2*Sin3;
	dtRotationMatrix[2][2] = Cos2*Cos3;
}
void EularYXZExtract(const double Rbo[3][3],struct SEularAngle *pE)
{
	pE->Roll =  asin(fLimitInOne(Rbo[1][2]));
	if ( cos(pE->Roll) > 0.0 )
		pE->Pitch = atan2(-Rbo[0][2],Rbo[2][2]);
	else
		pE->Pitch = atan2(Rbo[0][2],-Rbo[2][2]);
	if ( cos(pE->Roll) > 0.0 )
		pE->Yaw  = atan2(-Rbo[1][0],Rbo[1][1]);
	else
		pE->Yaw  = atan2(Rbo[1][0],-Rbo[1][1]);
}
#endif

#ifdef USE_USER_QUATERNION_TO_MATRIX
void QuaternionToMatrix(const double Q[4],double R[3][3])
{
	R[0][0] = Q[0]*Q[0]+Q[1]*Q[1]-Q[2]*Q[2]-Q[3]*Q[3];
	R[0][1] = 2*(Q[1]*Q[2]+Q[0]*Q[3]);
	R[0][2] = 2*(Q[1]*Q[3]-Q[0]*Q[2]);
	R[1][0] = 2*(Q[1]*Q[2]-Q[0]*Q[3]);
	R[1][1] = Q[0]*Q[0]-Q[1]*Q[1]+Q[2]*Q[2]-Q[3]*Q[3];
	R[1][2] = 2*(Q[2]*Q[3]+Q[0]*Q[1]);
	R[2][0] = 2*(Q[1]*Q[3]+Q[0]*Q[2]);
	R[2][1] = 2*(Q[2]*Q[3]-Q[0]*Q[1]);
	R[2][2] = Q[0]*Q[0]-Q[1]*Q[1]-Q[2]*Q[2]+Q[3]*Q[3];
}
void QuaternionExtract(const double  Mat[3][3],double  Q[4])
{
    double test0,test1,test2,test3;
	test0 = fabs(1+Mat[0][0]+Mat[1][1]+Mat[2][2]);
    test1 = fabs(1+Mat[0][0]-Mat[1][1]-Mat[2][2]);
    test2 = fabs(1-Mat[0][0]+Mat[1][1]-Mat[2][2]);
	test3 = fabs(1-Mat[0][0]-Mat[1][1]+Mat[2][2]);
	if((test0>=test1)&&(test0>=test2)&&(test0>=test3))
	{
	    Q[0] = 0.5*sqrt(test0);
        Q[1] = 0.25/Q[0]*(Mat[1][2]-Mat[2][1]);
        Q[2] = 0.25/Q[0]*(Mat[2][0]-Mat[0][2]);
        Q[3] = 0.25/Q[0]*(Mat[0][1]-Mat[1][0]);
    }
    if((test1>=test0)&&(test1>=test2)&&(test1>=test3))
	{
	    Q[1] = 0.5*sqrt(test1);
        Q[2] = 0.25/Q[1]*(Mat[0][1]+Mat[1][0]);
        Q[3] = 0.25/Q[1]*(Mat[0][2]+Mat[2][0]);
        Q[0] = 0.25/Q[1]*(Mat[1][2]-Mat[2][1]);
		if(Q[0]<0)
		{
			Q[0]=-Q[0];
			Q[1]=-Q[1];
			Q[2]=-Q[2];
			Q[3]=-Q[3];
		}
    }
    if((test2>=test0)&&(test2>=test1)&&(test2>=test3))
	{
	
	    Q[2] = 0.5*sqrt(test2);
        Q[3] = 0.25/Q[2]*(Mat[1][2]+Mat[2][1]);
        Q[1] = 0.25/Q[2]*(Mat[0][1]+Mat[1][0]);
        Q[0] = 0.25/Q[2]*(Mat[2][0]-Mat[0][2]);
		if(Q[0]<0)
		{
			Q[0]=-Q[0];
			Q[1]=-Q[1];
			Q[2]=-Q[2];
			Q[3]=-Q[3];
		}
    }
    if((test3>=test0)&&(test3>=test2)&&(test3>=test1))
	{
	    Q[3] = 0.5*sqrt(test3);
        Q[0] = 0.25/Q[3]*(Mat[0][1]-Mat[1][0]);
        Q[1] = 0.25/Q[3]*(Mat[0][2]+Mat[2][0]);
        Q[2] = 0.25/Q[3]*(Mat[1][2]+Mat[2][1]);
		if(Q[0]<0)
		{
			Q[0]=-Q[0];
			Q[1]=-Q[1];
			Q[2]=-Q[2];
			Q[3]=-Q[3];
		}
    }
}
#endif

#ifdef USE_USER_SPHERE_TO_RECTANGULAR
void	RectAngularToSphere(const double X[3],struct SSphereElements *pRAE)
{
	pRAE->Distance = GetLength(X);
	pRAE->Azimuth = atan2(X[1],X[0]);
	pRAE->Elevation = asin(fLimitInOne(X[2]/pRAE->Distance));
}
void	SphereToRectAngular(const struct SSphereElements *pRAE,double X[3])
{
	X[0] = pRAE->Distance *cos(pRAE->Elevation )*cos(pRAE->Azimuth) ;
	X[1] = pRAE->Distance *cos(pRAE->Elevation )*sin(pRAE->Azimuth) ;
	X[2] = pRAE->Distance *sin(pRAE->Elevation );
}
#endif

#ifdef USE_USER_RANDOM
double floatRandom(const double Amplify)
{
	double X = 0.0;
	long lX = 0;
	lX = rand();
	X = 2*Amplify/RAND_MAX*(lX-RAND_MAX/2);
	return X;
}
double mgrn1(const double u,const double g,double *r)
{ 
	long i = 0,m = 0;
	double s,w,v,t;
	s=65536.0; w=2053.0; v=13849.0;	t=0.0;
	for (i=1; i<=12; i++)
	{ 
		*r = (*r)*w+v; 
		m =(long)(*r/s);
		*r = *r-m*s; 
		t = t+(*r)/s;
	}
	t=u+g*(t-6.0);
	return(t);
}
double NormalDistributionParameter(const double Mean,const double ThreeVariance)
{
	static double r;
	r = (double) rand();
	return mgrn1(Mean,ThreeVariance/3.0,&r);
}
double NormalDistribution(const struct SNormalNoise* pNoise)
{
	static double r;
	r = (double) rand();
	return mgrn1(pNoise->Constant,pNoise->Random/3.0,&r);
}
double AddSinNoise(const double TimeNow,const struct SSinNoise* pNoise)
{
	return (pNoise->Amplitude*sin(pNoise->Frequency*TimeNow+pNoise->Argument));
}
#endif
