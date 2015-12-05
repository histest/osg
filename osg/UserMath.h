#ifndef __USER_MATH_H
#define __USER_MATH_H

#include "MathUsage.h"

#ifdef USE_USER_VECTOR
double	dot(const double  x1[3],const double  x2[3]);
void	cross( const double  x1[3],const double  x2[3],double  x3[3]);
double	GetLength( const double  x1[3]);
void	Unit(double x[3]);
#endif

#ifdef USE_USER_QUATERNION
void QuaterMultiply(const double  In1[4],const double  In2[4],double  Out[4]);
void QuaternionTransfer(double  In[4]);
void QuaterDivided(const double  In1[4],const double  In2[4],double  Out[4]);
void QuaterConjugateMultiply(const double  In1[4],const double  In2[4],double  Out[4]);
void QuaterUnit(double In[4]);
void GetQuaternionMatrix4X3(const double In[4], double Out[4][3]);
#endif

#ifdef USE_USER_VALUE_LIMIT
double	fsign(const double X);
double	fLimit(const double x, const double LimitValue);
double	fLimitInPI(const double X);
double	fLimitInOne(const double X);
#endif

#ifdef USE_USER_FRAME
void	RotateFrame(const double  R[3][3], double  Vec[3]);
void	InverseRotateFrame(const double  R[3][3],double  Vec[3]);
void	GetRbi(const double Q[4],double R[3][3]);
void	GetRoi(const double R[3],const double V[3], double Mat[3][3]);
void    GetRei(const double Ag,double R[3][3]);
void	GetRvi(const double Longitude, const double Latitude, double Rvi[3][3]);
#endif

#ifdef USE_USER_MATRIX
void		MatrixProduct(const unsigned long iN, const unsigned long iM, const unsigned long iK, const double*  A1,const double * A2,double * A3);
void		MatrixProductTranspose(const unsigned long iN, const unsigned long iM, const unsigned long iK, const double*  A1,const double * A2,double * A3);
void		MatrixTranspose(const unsigned long iN,const unsigned long iM, const double * A1,double * A2);
void		MatrixDeterminant( const double A[3][3], double * ReturnValue);
signed char MatrixInverse3X3(const double A1[3][3] ,double A2[3][3]);
#endif

#ifdef USE_USER_RAW_TO_ENGINEERING
struct SValueRange
{
	double MinValue;
	double MaxValue;
};
double	RawToEngineering(unsigned char iLen, unsigned long iValue,double ValueRange);
double	RawToEngineeringRange(unsigned char iLen, unsigned long iValue,double MinValue, double MaxValue);
double	EngineeringToRaw(double Value, double ValueRange, unsigned char iLen);
double	EngineeringToRawRange(double Value, double MinValue, double MaxValue, unsigned char iLen);
#endif

#ifdef USE_USER_EULAR_TO_MATRIX
struct SEularAngle
{
	double Roll;
	double Pitch;
	double Yaw;
};
void	EularYXZToMatrix(const struct SEularAngle *E,double R[3][3]);
void	Eular313ToMatrix(const struct SEularAngle *E,double R[3][3]);
void	EularYXZExtract(const double Mat[3][3],struct SEularAngle *E);
#endif

#ifdef USE_USER_QUATERNION_TO_MATRIX
void		QuaternionToMatrix(const double Q[4], double  R[3][3]);
void		QuaternionExtract(const double  Mat[3][3],double  Q[4]);
#endif

#ifdef USE_USER_SPHERE_TO_RECTANGULAR
struct SSphereElements
{
	double Distance;
	double Azimuth;
	double Elevation;
};
void		RectAngularToSphere(const double X[3],struct SSphereElements *RAE);
void		SphereToRectAngular(const struct SSphereElements *RAE,double X[3]);
#endif

#ifdef USE_USER_RANDOM
struct SNormalNoise
{
	double Constant;
	double Random;
};
struct SSinNoise
{
	double Amplitude;
	double Frequency;
	double Argument;
};
double floatRandom(const double Amplify);
double NormalDistributionParameter(const double Mean,const double ThreeVariance);
double NormalDistribution(const struct SNormalNoise *Noise);
double AddSinNoise(const double TimeNow,const struct SSinNoise *Noise);
#endif

#endif
