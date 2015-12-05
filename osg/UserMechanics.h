#ifndef __USER_MECHANICS_H
#define __USER_MECHANICS_H

#include "MathUsage.h"
#include "UserMath.h"

#ifdef USE_USER_MATRIX
signed char MatrixInverse(const unsigned char n, double* A);
signed char bginv(double a[],unsigned long m,unsigned long n,double aa[], double eps,double u[], double v[], unsigned long ka);
#endif

#ifdef USE_USER_FRAME
void	GetRdi(const double R[3],const double V[3],double Rdi[3][3]);
void	GetREi(const double R[3],double Mat[3][3]);
void	GetRba(const double A, const double B, double R[3][3]);
void    GetRsi(const double R[3],const double V[3],const double Rsun[3],double Rsi[3][3]);
void	GetRbE(const double Azimuth, const double Elevation, double R[3][3]);
void	GetRx(const double Angle,double R[3][3]);
void	GetRy(const double Angle,double R[3][3]);
void	GetRz(const double Angle,double R[3][3]);
#endif

#ifdef USE_USER_EULAR_TO_MATRIX
void	EularXYZToMatrix(const struct SEularAngle *E,double  R[3][3]);
void	EularXYZExtract(const double  Mat[3][3],struct SEularAngle *E);
void	EularXZYToMatrix(const struct SEularAngle *E,double  R[3][3]);
void	EularXZYExtract(const double  Mat[3][3],struct SEularAngle *E);
void	Eular313Extract(const double  Mat[3][3],struct SEularAngle *E);
#endif

#ifdef USE_USER_EULAR_TO_QUATERNION
void	QuaternionToEularXYZ(const double Q[4],struct SEularAngle *E);
void	EularXYZToQuaternion(const struct SEularAngle *E,double  Q[4]);
void	QuaternionToEularYXZ(const double Q[4],struct SEularAngle *E);
void	EularYXZToQuaternion(const struct SEularAngle *E,double  Q[4]);
void	QuaternionToEularXZY(const double  Q[4],struct SEularAngle *E);
void	EularXZYToQuaternion(const struct SEularAngle *E,double  Q[4]);
#endif

#ifdef USE_USER_TIME
struct SSimulationDateTime
{
	unsigned short	Year;
	unsigned char	Month;
	unsigned char	Day;
	unsigned char	Hour;
	unsigned char	Minute;
	unsigned char	Second;
	unsigned short	Millisecond;
	unsigned short	Microsecond;
};

#pragma pack(push,1)
union UGPSTime
{
	struct SGPSTime
	{
		unsigned short usWeeks;
		unsigned long  ulMillisecond;
		unsigned short usMicrosecond;
	} stcTime;
	long long llTime;
};
#pragma pack(pop)

void	AddTimeSpan(struct SSimulationDateTime *StartTime,const long long llDeltaTime);
long long GetGPSFormat(const struct SSimulationDateTime *stcGMTTime,const long long llUTCOffsetMicroSecond);
struct SSimulationDateTime GetTimeFromGPSFormat(const long long lTime,const long long llUTCOffsetMicroSecond);
unsigned long GregorianToModifierJulianData(const struct SSimulationDateTime *stcTime);
double J2000(const struct SSimulationDateTime *stcTime);
double JulianDateTDT(const struct SSimulationDateTime *pstcTime);
struct SSimulationDateTime ModifierJulianDataToGregorian(const unsigned long ulMJD);
unsigned char DayOfMonth(const unsigned short usYear,const unsigned char ucMonth);
#endif

#ifdef USE_USER_SMOOTH
double GroupAverage(const unsigned long ulNo,double Value[]);
#endif

#endif
