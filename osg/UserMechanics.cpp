

#include "stdafx.h"
#include "UserMath.h"
#include "UserMechanics.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#ifdef USE_USER_TIME
#include <string.h>
#endif

#ifdef USE_USER_MATRIX
void _Delete(void **p);
#define New(type,count) (type*)calloc(count,sizeof(type))
#define Delete(x) _Delete((void**)&x)
void _Delete(void **p)
{
	free(*p);
	*p = NULL;
}
signed char MatrixInverse(const unsigned char n, double* a)
{
   int *is,*js,i,j,k,l,u,v;
   double d,p;
   js = New(int,n);
   is = New(int,n);

   for(k=0;k<=n-1;k++)
   { 
	   d=0.0;   
    for(i=k;i<=n-1;i++)
    for(j=k;j<=n-1;j++)
    { 
		l=i*n+j;p=fabs(a[l]);
        if(p>d)
        { 
			d=p;is[k]=i;js[k]=j;
		}
    }

    if(d+1.0==1.0)
    { 
		Delete(is);
		Delete(js);
		return(0);
    }

    if(is[k]!=k)
      for (j=0;j<=n-1;j++)
      { u=k*n+j;
        v=is[k]*n+j;
        p=a[u];a[u]=a[v];a[v]=p;
      }

    if(js[k]!=k)
      for(i=0;i<=n-1;i++)
      {  u=i*n+k;
         v=i*n+js[k];
         p=a[u];a[u]=a[v];a[v]=p;
      }

    l=k*n+k;
    a[l]=1.0/a[l];
    for(j=0;j<=n-1;j++)
       if(j!=k)
        { u=k*n+j;
          a[u]=a[u]*a[l];
        }

    for(i=0;i<=n-1;i++)
       if(i!=k)
          for(j=0;j<=n-1;j++)
        if(j!=k)
          { u=i*n+j;
            a[u]=a[u]-a[i*n+k]*a[k*n+j];
          }

    for(i=0;i<=n-1;i++)
      if(i!=k)
      {u=i*n+k; a[u]=-a[u]*a[l]; }
       }  

       for(k=n-1;k>=0;k--)
     { if(js[k]!=k)
         for(j=0;j<=n-1;j++)
         {  u=k*n+j;
        v=js[k]*n+j;
        p=a[u];a[u]=a[v];a[v]=p;
          }

       if(is[k]!=k)
         for(i=0;i<=n-1;i++)
         { u=i*n+k; v=i*n+is[k];
           p=a[u]; a[u]=a[v];a[v]=p;
         }
     }
	   Delete(is);
	   Delete(js);
  return(1);
}
int LUDATN(double* A, int N, double* LU, int *APVT)
{
	const double ZERO = 0.0;
	const double ONE = 1.0;
	const double FOUR = 4.0;
	const double SIXTN = 16.0;
	const double SIXTH = 0.0625;

	int IER,RN,JM1,IM1,JP1,IMAX;
	double P,Q,SUM,WREL,D1,D2,BIG,BIGA;
	int i,j,k;
	
//	double *EQUIL = new double[N];	
	double *EQUIL = New(double,N);

	IMAX = 0;

	  IER = 0;
      RN = N;
      WREL = ZERO;
      D1 = ONE;
      D2 = ZERO;
      BIGA = ZERO;
	
	  for (i=0;i<N;i++)
	  {
		  BIG = ZERO;
		  for (j=0;j<N;j++)
		  {
			  P = A[i*N+j];
			  LU[i*N+j] = P;
			  P = fabs(P);
			  if ( P > BIG ) 
				  BIG = P;
		  }
		  if (BIG > BIGA) BIGA = BIG;
		  if (BIG == ZERO) 
		  {
//			  delete[] EQUIL;
			  Delete( EQUIL );
			  return -1;
		  }
		  EQUIL[i] = ONE/BIG;
	  }

	  for(j=0;j<N;j++)
	  {
		  JM1 = j-1;
		  for (i=0;i<=JM1;i++)
		  {
			  SUM = LU[i*N+j];
			  IM1 = i-1;
			  for (k=0;k<=IM1;k++)
			  {
				  SUM = SUM - LU[i*N+k]*LU[k*N+j];
			  }
              LU[i*N+j] = SUM;
		  }
		  P = ZERO;
		  for(i=j;i<N;i++)
		  {
			  SUM = LU[i*N+j];
			  for(k=0;k<=JM1;k++)
			  {
				  SUM = SUM-LU[i*N+k]*LU[k*N+j];
			  }
			  LU[i*N+j] = SUM;
		      Q = EQUIL[i]*fabs(SUM);
			  if (P < Q)
			  {
				  P = Q;
				  IMAX = i;
			  }
		  }
		  if (RN+P == RN) 
		  {
//			  delete[] EQUIL;
			  Delete( EQUIL );
			  return -1;
		  }
		  if (j != IMAX)
		  {
			  D1 = -D1;
			  for(k=0;k<N;k++)
			  {
 				  P = LU[IMAX*N+k];
                  LU[IMAX*N+k] = LU[j*N+k];
                  LU[j*N+k] = P;
			  }
              EQUIL[IMAX] = EQUIL[j];
		  }
		  APVT[j] = IMAX;
          D1 = D1*LU[j*N+j];
		  while (fabs(D1) > ONE)
		  {
			  D1 = D1*SIXTH;
			  D2 = D2+FOUR;
		  }
		  while (fabs(D1) < SIXTH)
		  {
			  D1 = D1*SIXTN;
			  D2 = D2-FOUR;
		  }
          JP1 = j+1;
          P = LU[j*N+j];
		  for(i=JP1;i<N;i++)
		  {
			  LU[i*N+j] = LU[i*N+j]/P;
		  }
	  }

	  Delete( EQUIL );
	  return 0;
}

int LUELMN(double* A, int N, double *X, int* APVT)
{
	double SUM;
	int i,j,IP,IW,IB,IP1,IM1;
/*
	for (i=0;i<N;i++)
	{
		X[i] = B[i*N];
	}
*/	
    IW = -1;
	for (i=0;i<N;i++)
	{
		IP = APVT[i];
		SUM = X[IP];
		X[IP] = X[i];
		if (IW == -1 )
		{
			if ( SUM != 0.0 )
				IW = i;
		}
		else
		{
			IM1 = i - 1;
			for ( j=IW;j<=IM1;j++)
			{
				SUM = SUM - A[i*N+j]*X[j];
			}
		}
		X[i] = SUM;
	}
	for (IB=0;IB<N;IB++)
	{
		i = N-1-IB;
		IP1 = i + 1;
		SUM = X[i];
		for(j=IP1;j<N;j++)
		{
			SUM = SUM-A[i*N+j]*X[j];
		}
		X[i] = SUM/A[i*N+i];
	}
	return 0;
}
int LEQTF (double* A, int M, int N, double* B, double* BA)
{

	int i,j,intF;
	int *APVT;
	double *LUA;
	double *BL;

//	APVT = new int[N];
//	LUA = new double[N*N];
//	BL = new double[N];

	APVT = New(int,N);
	LUA = New(double,N*N);
	BL = New( double,N);

	intF = LUDATN(A,N,LUA,APVT);
	if (intF != 0 ) 
	{
//		delete[] APVT;
//		delete[] LUA;
//		delete[] BL;
		Delete(APVT);
		Delete(LUA);
		Delete(BL);
		return intF;
	}
	for(j=0;j<M;j++)
	{
		for(i=0;i<N;i++)
		{
			BL[i] = B[i*M+j];
		}

		if ( LUELMN(LUA,N,BL,APVT) != 0 )
		{
//			delete[] APVT;
//			delete[] LUA;
//			delete[] BL;
			Delete(APVT);
			Delete(LUA);
			Delete(BL);
			return -3;
		}
		for(i=0;i<N;i++)
		{
			BA[i*M+j] = BL[i];
		}
	}
//	delete[] APVT;
//	delete[] LUA;
//	delete[] BL;
	Delete(APVT);
	Delete(LUA);
	Delete(BL);
	return 0;
}

  static void ppp(double a[],double e[],double s[],double v[],int m,int n)
  { int i,j,p,q;
    double d;
    if (m>=n) i=n;
    else i=m;
    for (j=1; j<=i-1; j++)
      { a[(j-1)*n+j-1]=s[j-1];
        a[(j-1)*n+j]=e[j-1];
      }
    a[(i-1)*n+i-1]=s[i-1];
    if (m<n) a[(i-1)*n+i]=e[i-1];
    for (i=1; i<=n-1; i++)
    for (j=i+1; j<=n; j++)
      { p=(i-1)*n+j-1; q=(j-1)*n+i-1;
        d=v[p]; v[p]=v[q]; v[q]=d;
      }
    return;
  }

  static void sss(double fg[2],double cs[2])
  { double r,d;
    if ((fabs(fg[0])+fabs(fg[1]))==0.0)
      { cs[0]=1.0; cs[1]=0.0; d=0.0;}
    else 
      { d=sqrt(fg[0]*fg[0]+fg[1]*fg[1]);
        if (fabs(fg[0])>fabs(fg[1]))
          { d=fabs(d);
            if (fg[0]<0.0) d=-d;
          }
        if (fabs(fg[1])>=fabs(fg[0]))
          { d=fabs(d);
            if (fg[1]<0.0) d=-d;
          }
        cs[0]=fg[0]/d; cs[1]=fg[1]/d;
      }
    r=1.0;
    if (fabs(fg[0])>fabs(fg[1])) r=cs[1];
    else
      if (cs[0]!=0.0) r=1.0/cs[0];
    fg[0]=d; fg[1]=r;
    return;
  }


  int bmuav(double a[],int m,int n,double u[],double v[],double eps,int ka)
  { int i,j,k,l,it,ll,kk,ix,iy,mm,nn,iz,m1,ks;
    double d,dd,t,sm,sm1,em1,sk,ek,b,c,shh,fg[2],cs[2];
    double *s,*e,*w;
/*
    s=new double[ka];
    e=new double[ka];
    w=new double[ka];
*/
    s=New(double,ka);
    e=New(double,ka);
    w=New(double,ka);
    it=60; k=n;
    if (m-1<n) k=m-1;
    l=m;
    if (n-2<m) l=n-2;
    if (l<0) l=0;
    ll=k;
    if (l>k) ll=l;
    if (ll>=1)
      { for (kk=1; kk<=ll; kk++)
          { if (kk<=k)
              { d=0.0;
                for (i=kk; i<=m; i++)
                  { ix=(i-1)*n+kk-1; d=d+a[ix]*a[ix];}
                s[kk-1]=sqrt(d);
                if (s[kk-1]!=0.0)
                  { ix=(kk-1)*n+kk-1;
                    if (a[ix]!=0.0)
                      { s[kk-1]=fabs(s[kk-1]);
                        if (a[ix]<0.0) s[kk-1]=-s[kk-1];
                      }
                    for (i=kk; i<=m; i++)
                      { iy=(i-1)*n+kk-1;
                        a[iy]=a[iy]/s[kk-1];
                      }
                    a[ix]=1.0+a[ix];
                  }
                s[kk-1]=-s[kk-1];
              }
            if (n>=kk+1)
              { for (j=kk+1; j<=n; j++)
                  { if ((kk<=k)&&(s[kk-1]!=0.0))
                      { d=0.0;
                        for (i=kk; i<=m; i++)
                          { ix=(i-1)*n+kk-1;
                            iy=(i-1)*n+j-1;
                            d=d+a[ix]*a[iy];
                          }
                        d=-d/a[(kk-1)*n+kk-1];
                        for (i=kk; i<=m; i++)
                          { ix=(i-1)*n+j-1;
                            iy=(i-1)*n+kk-1;
                            a[ix]=a[ix]+d*a[iy];
                          }
                      }
                    e[j-1]=a[(kk-1)*n+j-1];
                  }
              }
            if (kk<=k)
              { for (i=kk; i<=m; i++)
                  { ix=(i-1)*m+kk-1; iy=(i-1)*n+kk-1;
                    u[ix]=a[iy];
                  }
              }
            if (kk<=l)
              { d=0.0;
                for (i=kk+1; i<=n; i++)
                  d=d+e[i-1]*e[i-1];
                e[kk-1]=sqrt(d);
                if (e[kk-1]!=0.0)
                  { if (e[kk]!=0.0)
                      { e[kk-1]=fabs(e[kk-1]);
                        if (e[kk]<0.0) e[kk-1]=-e[kk-1];
                      }
                    for (i=kk+1; i<=n; i++)
                      e[i-1]=e[i-1]/e[kk-1];
                    e[kk]=1.0+e[kk];
                  }
                e[kk-1]=-e[kk-1];
                if ((kk+1<=m)&&(e[kk-1]!=0.0))
                  { for (i=kk+1; i<=m; i++) w[i-1]=0.0;
                    for (j=kk+1; j<=n; j++)
                      for (i=kk+1; i<=m; i++)
                        w[i-1]=w[i-1]+e[j-1]*a[(i-1)*n+j-1];
                    for (j=kk+1; j<=n; j++)
                      for (i=kk+1; i<=m; i++)
                        { ix=(i-1)*n+j-1;
                          a[ix]=a[ix]-w[i-1]*e[j-1]/e[kk];
                        }
                  }
                for (i=kk+1; i<=n; i++)
                  v[(i-1)*n+kk-1]=e[i-1];
              }
          }
      }
    mm=n;
    if (m+1<n) mm=m+1;
    if (k<n) s[k]=a[k*n+k];
    if (m<mm) s[mm-1]=0.0;
    if (l+1<mm) e[l]=a[l*n+mm-1];
    e[mm-1]=0.0;
    nn=m;
    if (m>n) nn=n;
    if (nn>=k+1)
      { for (j=k+1; j<=nn; j++)
          { for (i=1; i<=m; i++)
              u[(i-1)*m+j-1]=0.0;
            u[(j-1)*m+j-1]=1.0;
          }
      }
    if (k>=1)
      { for (ll=1; ll<=k; ll++)
          { kk=k-ll+1; iz=(kk-1)*m+kk-1;
            if (s[kk-1]!=0.0)
              { if (nn>=kk+1)
                  for (j=kk+1; j<=nn; j++)
                    { d=0.0;
                      for (i=kk; i<=m; i++)
                        { ix=(i-1)*m+kk-1;
                          iy=(i-1)*m+j-1;
                          d=d+u[ix]*u[iy]/u[iz];
                        }
                      d=-d;
                      for (i=kk; i<=m; i++)
                        { ix=(i-1)*m+j-1;
                          iy=(i-1)*m+kk-1;
                          u[ix]=u[ix]+d*u[iy];
                        }
                    }
                  for (i=kk; i<=m; i++)
                    { ix=(i-1)*m+kk-1; u[ix]=-u[ix];}
                  u[iz]=1.0+u[iz];
                  if (kk-1>=1)
                    for (i=1; i<=kk-1; i++)
                      u[(i-1)*m+kk-1]=0.0;
              }
            else
              { for (i=1; i<=m; i++)
                  u[(i-1)*m+kk-1]=0.0;
                u[(kk-1)*m+kk-1]=1.0;
              }
          }
      }
    for (ll=1; ll<=n; ll++)
      { kk=n-ll+1; iz=kk*n+kk-1;
        if ((kk<=l)&&(e[kk-1]!=0.0))
          { for (j=kk+1; j<=n; j++)
              { d=0.0;
                for (i=kk+1; i<=n; i++)
                  { ix=(i-1)*n+kk-1; iy=(i-1)*n+j-1;
                    d=d+v[ix]*v[iy]/v[iz];
                  }
                d=-d;
                for (i=kk+1; i<=n; i++)
                  { ix=(i-1)*n+j-1; iy=(i-1)*n+kk-1;
                    v[ix]=v[ix]+d*v[iy];
                  }
              }
          }
        for (i=1; i<=n; i++)
          v[(i-1)*n+kk-1]=0.0;
        v[iz-n]=1.0;
      }
    for (i=1; i<=m; i++)
    for (j=1; j<=n; j++)
      a[(i-1)*n+j-1]=0.0;
    m1=mm; it=60;
    while (1==1)
      { if (mm==0)
          { ppp(a,e,s,v,m,n);
            free(s); free(e); free(w); return(1);
          }
        if (it==0)
          { ppp(a,e,s,v,m,n);
            free(s); free(e); free(w); return(-1);
          }
        kk=mm-1;
	while ((kk!=0)&&(fabs(e[kk-1])!=0.0))
          { d=fabs(s[kk-1])+fabs(s[kk]);
            dd=fabs(e[kk-1]);
            if (dd>eps*d) kk=kk-1;
            else e[kk-1]=0.0;
          }
        if (kk==mm-1)
          { kk=kk+1;
            if (s[kk-1]<0.0)
              { s[kk-1]=-s[kk-1];
                for (i=1; i<=n; i++)
                  { ix=(i-1)*n+kk-1; v[ix]=-v[ix];}
              }
            while ((kk!=m1)&&(s[kk-1]<s[kk]))
              { d=s[kk-1]; s[kk-1]=s[kk]; s[kk]=d;
                if (kk<n)
                  for (i=1; i<=n; i++)
                    { ix=(i-1)*n+kk-1; iy=(i-1)*n+kk;
                      d=v[ix]; v[ix]=v[iy]; v[iy]=d;
                    }
                if (kk<m)
                  for (i=1; i<=m; i++)
                    { ix=(i-1)*m+kk-1; iy=(i-1)*m+kk;
                      d=u[ix]; u[ix]=u[iy]; u[iy]=d;
                    }
                kk=kk+1;
              }
            it=60;
            mm=mm-1;
          }
        else
          { ks=mm;
            while ((ks>kk)&&(fabs(s[ks-1])!=0.0))
              { d=0.0;
                if (ks!=mm) d=d+fabs(e[ks-1]);
                if (ks!=kk+1) d=d+fabs(e[ks-2]);
                dd=fabs(s[ks-1]);
                if (dd>eps*d) ks=ks-1;
                else s[ks-1]=0.0;
              }
            if (ks==kk)
              { kk=kk+1;
                d=fabs(s[mm-1]);
                t=fabs(s[mm-2]);
                if (t>d) d=t;
                t=fabs(e[mm-2]);
                if (t>d) d=t;
                t=fabs(s[kk-1]);
                if (t>d) d=t;
                t=fabs(e[kk-1]);
                if (t>d) d=t;
                sm=s[mm-1]/d; sm1=s[mm-2]/d;
                em1=e[mm-2]/d;
                sk=s[kk-1]/d; ek=e[kk-1]/d;
                b=((sm1+sm)*(sm1-sm)+em1*em1)/2.0;
                c=sm*em1; c=c*c; shh=0.0;
                if ((b!=0.0)||(c!=0.0))
                  { shh=sqrt(b*b+c);
                    if (b<0.0) shh=-shh;
                    shh=c/(b+shh);
                  }
                fg[0]=(sk+sm)*(sk-sm)-shh;
                fg[1]=sk*ek;
                for (i=kk; i<=mm-1; i++)
                  { sss(fg,cs);
                    if (i!=kk) e[i-2]=fg[0];
                    fg[0]=cs[0]*s[i-1]+cs[1]*e[i-1];
                    e[i-1]=cs[0]*e[i-1]-cs[1]*s[i-1];
                    fg[1]=cs[1]*s[i];
                    s[i]=cs[0]*s[i];
                    if ((cs[0]!=1.0)||(cs[1]!=0.0))
                      for (j=1; j<=n; j++)
                        { ix=(j-1)*n+i-1;
                          iy=(j-1)*n+i;
                          d=cs[0]*v[ix]+cs[1]*v[iy];
                          v[iy]=-cs[1]*v[ix]+cs[0]*v[iy];
                          v[ix]=d;
                        }
                    sss(fg,cs);
                    s[i-1]=fg[0];
                    fg[0]=cs[0]*e[i-1]+cs[1]*s[i];
                    s[i]=-cs[1]*e[i-1]+cs[0]*s[i];
                    fg[1]=cs[1]*e[i];
                    e[i]=cs[0]*e[i];
                    if (i<m)
                      if ((cs[0]!=1.0)||(cs[1]!=0.0))
                        for (j=1; j<=m; j++)
                          { ix=(j-1)*m+i-1;
                            iy=(j-1)*m+i;
                            d=cs[0]*u[ix]+cs[1]*u[iy];
                            u[iy]=-cs[1]*u[ix]+cs[0]*u[iy];
                            u[ix]=d;
                          }
                  }
                e[mm-2]=fg[0];
                it=it-1;
              }
            else
              { if (ks==mm)
                  { kk=kk+1;
                    fg[1]=e[mm-2]; e[mm-2]=0.0;
                    for (ll=kk; ll<=mm-1; ll++)
                      { i=mm+kk-ll-1;
                        fg[0]=s[i-1];
                        sss(fg,cs);
                        s[i-1]=fg[0];
                        if (i!=kk)
                          { fg[1]=-cs[1]*e[i-2];
                            e[i-2]=cs[0]*e[i-2];
                          }
                        if ((cs[0]!=1.0)||(cs[1]!=0.0))
                          for (j=1; j<=n; j++)
                            { ix=(j-1)*n+i-1;
                              iy=(j-1)*n+mm-1;
                              d=cs[0]*v[ix]+cs[1]*v[iy];
                              v[iy]=-cs[1]*v[ix]+cs[0]*v[iy];
                              v[ix]=d;
                            }
                      }
                  }
                else
                  { kk=ks+1;
                    fg[1]=e[kk-2];
                    e[kk-2]=0.0;
                    for (i=kk; i<=mm; i++)
                      { fg[0]=s[i-1];
                        sss(fg,cs);
                        s[i-1]=fg[0];
                        fg[1]=-cs[1]*e[i-1];
                        e[i-1]=cs[0]*e[i-1];
                        if ((cs[0]!=1.0)||(cs[1]!=0.0))
                          for (j=1; j<=m; j++)
                            { ix=(j-1)*m+i-1;
                              iy=(j-1)*m+kk-2;
                              d=cs[0]*u[ix]+cs[1]*u[iy];
                              u[iy]=-cs[1]*u[ix]+cs[0]*u[iy];
                              u[ix]=d;
                            }
                      }
                  }
              }
          }
      }
    return(1);
  }


signed char bginv(double a[],unsigned long m,unsigned long n,double aa[], double eps,double u[], double v[], unsigned long ka)
{ unsigned long i,j,k,l,t,p,q,f;
i=bmuav(a,m,n,u,v,eps,ka);
if (i<0) return(-1);
j=n;
if (m<n) j=m;
j=j-1;
k=0;
while ((k<=j)&&(a[k*n+k]!=0.0)) k=k+1;
k=k-1;
for (i=0; i<=n-1; i++)
for (j=0; j<=m-1; j++)
    { t=i*m+j; aa[t]=0.0;
    for (l=0; l<=k; l++)
        { f=l*n+i; p=j*m+l; q=l*n+l;
        aa[t]=aa[t]+v[f]*u[p]/a[q];
        }
    }
return(1);
}

#endif


#ifdef USE_USER_FRAME
void GetRdi(const double R[3],const double V[3],double Rdi[3][3])
{
	unsigned char i=0;
	double x[3]={0.0},y[3]={0.0},z[3]={0.0};
	double TempLength =GetLength(R);
	for (i=0;i<3;i++)
	{
		x[i] = R[i]/TempLength;
		y[i] = V[i];
	}
	cross(x,y,z);
	Unit(z);
	cross(z,x,y);
	for (i=0;i<3;i++)
	{
	/*	Rdi[0][i] = x[i];
		Rdi[1][i] = y[i];
		Rdi[2][i] = z[i];*/

		Rdi[i][0] = x[i];
		Rdi[i][1] = y[i];
		Rdi[i][2] = z[i];
	}
}


void GetRba(const double A, const double B, double R[3][3])
{
	R[0][0] = cos(A)*cos(B);
	R[0][1] = -cos(A)*sin(B);
	R[0][2] = -sin(A);
	R[1][0] = sin(B);
	R[1][1] = cos(B);
	R[1][2] = 0;
	R[2][0] = cos(B)*sin(A);
	R[2][1] = -sin(A)*sin(B);
	R[2][2] = cos(A);
}
void GetREi(const double R[3],double REi[3][3])
{
	unsigned char i = 0;

	double x[3],y[3],z[3];
	double tmp = GetLength(R);
	for (i=0;i<3;i++)  z[i] = -R[i]/tmp;
	x[0] = 0;	x[1] = 0;	x[2] = 1.0;
	cross(x,z,y);
	tmp = GetLength(y);
	for (i=0;i<3;i++)	y[i] /= tmp;
	cross(y,z,x);
	tmp = GetLength(x);
	for (i=0;i<3;i++) x[i] /= tmp;
	for (i=0;i<3;i++)
	{
		  REi[0][i] = x[i];
		  REi[1][i] = y[i];
		  REi[2][i] = z[i];
	}
}
void    GetRsi(const double P[3],const double V[3],const double Psun[3],double R[3][3])
{
    unsigned char i = 0;
	double x[3],y[3],z[3],H[3];
	double tmp = GetLength(Psun);
	for (i=0;i<3;i++)	y[i] = -Psun[i]/tmp;
	cross(P,V,H);
	cross(y,H,x);
	tmp = GetLength(x);
	for (i=0;i<3;i++) x[i] /= tmp;
	cross(x,y,z);
	for (i=0;i<3;i++)
	{
		  R[0][i] = x[i];
		  R[1][i] = y[i];
		  R[2][i] = z[i];
	}
}
void	GetRx(const double Angle,double R[3][3])
{
	R[0][0] = 1.0;	R[0][1] = 0.0;			R[0][2] = 0.0;
	R[1][0] = 0.0;	R[1][1] = cos(Angle);	R[1][2] = sin(Angle);
	R[2][0] = 0.0;	R[2][1] = -sin(Angle);	R[2][2] = cos(Angle);
}
void	GetRy(const double Angle,double R[3][3])
{
	R[0][0] = cos(Angle);	R[0][1] = 0.0;	R[0][2] = -sin(Angle);
	R[1][0] = 0.0;			R[1][1] = 1.0;	R[1][2] = 0.0;
	R[2][0] = sin(Angle);	R[2][1] = 0.0;	R[2][2] = cos(Angle);
}
void	GetRz(const double Angle,double R[3][3])
{
	R[0][0] = cos(Angle);	R[0][1] = sin(Angle);	R[0][2] = 0.0;
	R[1][0] = -sin(Angle);	R[1][1] = cos(Angle);	R[1][2] = 0.0;
	R[2][0] = 0.0;			R[2][1] = 0.0;			R[2][2] = 1.0;
}
void GetRbE(const double Azimuth, const double Elevation, double R[3][3])
{
	R[0][0] =  cos(Azimuth)*cos(Elevation);	R[0][1] =  cos(Elevation)*sin(Azimuth);	R[0][2] = sin(Elevation);
	R[1][0] = -sin(Azimuth);				R[1][1] =  cos(Azimuth);				R[1][2] = 0.0;
	R[2][0] = -cos(Azimuth)*sin(Elevation);	R[2][1] = -sin(Azimuth)*sin(Elevation);	R[2][2] = cos(Elevation);
}
#endif

#ifdef USE_USER_EULAR_TO_MATRIX
void EularXYZToMatrix(const struct SEularAngle  *pE,double  R[3][3])
{
	double Sin1,Cos1,Sin2,Cos2,Sin3,Cos3;
	Sin1 = sin(pE->Roll);
	Cos1 = cos(pE->Roll);
	Sin2 = sin(pE->Pitch);
	Cos2 = cos(pE->Pitch);
	Sin3 = sin(pE->Yaw);
	Cos3 = cos(pE->Yaw);

	R[0][0] =  Cos2*Cos3;
	R[0][1] =  Cos2*Sin3;
	R[0][2] = -Sin2;
	R[1][0] =  Sin1*Sin2*Cos3 - Cos1*Sin3;
	R[1][1] =  Sin1*Sin2*Sin3 + Cos1*Cos3;
	R[1][2] =  Sin1*Cos2;
	R[2][0] =  Sin1*Sin3+Cos1*Sin2*Cos3;
	R[2][1] = -Sin1*Cos3+Cos1*Sin2*Sin3;
	R[2][2] =  Cos1*Cos2;
}
void EularXYZExtract(const double  Mat[3][3],struct SEularAngle *pE)
{
	pE->Pitch = asin(fLimitInOne(-Mat[0][2]));
	if ( cos(pE->Pitch) > 0.0 )
		pE->Yaw = atan2(Mat[0][1],Mat[0][0]);
	else
		pE->Yaw = atan2(-Mat[0][1],-Mat[0][0]);
	if ( cos(pE->Pitch) > 0.0 )
        pE->Roll = atan2(Mat[1][2],Mat[2][2]);
	else
		pE->Roll = atan2(-Mat[1][2],-Mat[2][2]);
}
void	EularXZYToMatrix(const struct SEularAngle *pE,double  R[3][3])
{
	double S1,C1,S2,C2,S3,C3;
	S1 = sin(pE->Roll);
	C1 = cos(pE->Roll);
	S2 = sin(pE->Pitch);
	C2 = cos(pE->Pitch);
	S3 = sin(pE->Yaw);
	C3 = cos(pE->Yaw);

	R[0][0] =  C2*C3;
	R[0][1] =  S3;
	R[0][2] = -C3*S2;
	R[1][0] =  S2*S1-C2*C1*S3;
	R[1][1] =  C1*C3;
	R[1][2] =  C2*S1+C1*S1*S3;
	R[2][0] =  C1*S2 + C2*S1*S3;
	R[2][1] = -C3*S1;
	R[2][2] =  C2*C1-S2*S1*S3;	
}
void	EularXZYExtract(const double Rbo[3][3],struct SEularAngle *pE)
{
	pE->Yaw  =  asin(fLimitInOne(Rbo[0][1]));
	if ( cos(pE->Yaw) > 0.0 )
		pE->Pitch = atan2(-Rbo[0][2],Rbo[0][0]);
	else
		pE->Pitch = atan2(Rbo[0][2],-Rbo[0][0]);
	if ( cos(pE->Yaw) > 0.0 )
		pE->Roll  = atan2(-Rbo[2][1],Rbo[1][1]);
	else
		pE->Roll  = atan2(Rbo[2][1],-Rbo[1][1]);
}
void	Eular313Extract(const double Rbo[3][3],struct SEularAngle *pE)
{
	pE->Roll  =  asin(fLimitInOne(Rbo[1][2]));
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

#ifdef USE_USER_EULAR_TO_QUATERNION
void QuaternionToEularXYZ(const double Q[4],struct SEularAngle *pE)
{
	pE->Yaw = atan2(2*(Q[0]*Q[3]+Q[1]*Q[2]),(1-2*(Q[2]*Q[2]+Q[3]*Q[3])));
	pE->Pitch = asin(fLimitInOne(2*(Q[0]*Q[2]-Q[1]*Q[3])));
	pE->Roll = atan2(2*(Q[0]*Q[1]+Q[2]*Q[3]),(1-2*(Q[1]*Q[1]+Q[2]*Q[2])));
}

void EularXYZToQuaternion(const struct SEularAngle *pE,double  Q[4])
{
	double siny,cosy,sinp,cosp,sinr,cosr;
	siny = sin(pE->Yaw /2);
	cosy = cos(pE->Yaw/2);
	sinp = sin(pE->Pitch/2);
	cosp = cos(pE->Pitch/2);
	sinr = sin(pE->Roll/2);
	cosr = cos(pE->Roll/2);
	Q[0] =  cosr*cosp*cosy+sinr*sinp*siny;
	Q[1] =  sinr*cosp*cosy-cosr*sinp*siny;
	Q[2] =  cosr*sinp*cosy+sinr*cosp*siny;
	Q[3] =  cosr*cosp*siny-sinr*sinp*cosy;
}
void	QuaternionToEularXZY(const double Q[4],struct SEularAngle *pE)
{
	pE->Roll  = atan2(-2*(Q[2]*Q[3]+Q[0]*Q[1]),Q[0]*Q[0]-Q[1]*Q[1]+Q[2]*Q[2]-Q[3]*Q[3]);
	pE->Pitch = atan2( 2*(Q[0]*Q[2]-Q[1]*Q[3]),Q[0]*Q[0]+Q[1]*Q[1]-Q[2]*Q[2]-Q[3]*Q[3]);
	pE->Yaw	= asin(fLimitInOne(2*Q[1]*Q[2]+2*Q[0]*Q[3]));
}
void QuaternionToEularYXZ(const double Q[4],struct SEularAngle *pE)
{
	pE->Roll = asin(fLimitInOne(2*(Q[2]*Q[3]+Q[0]*Q[1])));
	pE->Pitch = atan2(-2*(Q[1]*Q[3]-Q[0]*Q[2]),(2*(Q[0]*Q[0]+Q[3]*Q[3])-1));
	pE->Yaw = atan2(-2*(Q[1]*Q[2]-Q[0]*Q[3]),(2*(Q[0]*Q[0]+Q[2]*Q[2])-1));
}

void EularYXZToQuaternion(const struct SEularAngle *pE,double  Q[4])
{
	double siny,cosy,sinp,cosp,sinr,cosr;
	siny = sin(pE->Yaw /2);
	cosy = cos(pE->Yaw/2);
	sinp = sin(pE->Pitch/2);
	cosp = cos(pE->Pitch/2);
	sinr = sin(pE->Roll/2);
	cosr = cos(pE->Roll/2);
	Q[0] =  cosr*cosp*cosy-sinr*sinp*siny;
	Q[1] =  sinr*cosp*cosy-cosr*sinp*siny;
	Q[2] =  cosr*sinp*cosy+sinr*cosp*siny;
	Q[3] =  sinr*sinp*cosy+cosr*cosp*siny;
}
void EularXZYToQuaternion(const struct SEularAngle *pE,double  Q[4])
{
	double siny,cosy,sinp,cosp,sinr,cosr;
	siny = sin(pE->Yaw /2);
	cosy = cos(pE->Yaw/2);
	sinp = sin(pE->Pitch/2);
	cosp = cos(pE->Pitch/2);
	sinr = sin(pE->Roll/2);
	cosr = cos(pE->Roll/2);
	Q[0] =  cosr*cosp*cosy-sinr*sinp*siny;
	Q[1] =  sinr*cosp*cosy+cosr*sinp*siny;
	Q[2] =  cosr*sinp*cosy+sinr*cosp*siny;
	Q[3] = -sinr*sinp*cosy+cosr*cosp*siny;
}
#endif


#ifdef USE_USER_TIME
long long TotalMicroseconds(const struct SSimulationDateTime* pstcTime)
{
	return (long long)( pstcTime->Hour*3600000000ll + pstcTime->Minute*60000000ll + pstcTime->Second*1000000ll + pstcTime->Millisecond*1000ll + pstcTime->Microsecond );
}
void ToHour(const long long llInTime, struct SSimulationDateTime *pstcTime)
{
	long long llTime = 0;
	llTime = llInTime;
	if ( llTime < 0 )
	{
		pstcTime->Hour = 0;
		pstcTime->Minute = 0;
		pstcTime->Second = 0;
		pstcTime->Millisecond = 0;
		pstcTime->Microsecond = 0;
		return;
	}
	pstcTime->Hour = (unsigned char) (llTime/3600000000ll);			llTime = llTime%3600000000ll;
	pstcTime->Minute = (unsigned char ) ( llTime/60000000 );		llTime = llTime%60000000;
	pstcTime->Second = (unsigned char) (llTime/1000000);			llTime = llTime%1000000;			
	pstcTime->Millisecond = (unsigned short) (llTime/1000);
	pstcTime->Microsecond = (unsigned short) (llTime%1000);
}
void AddTimeSpan(struct SSimulationDateTime *pstcStartTime,const long long llDeltaTime)
{
	unsigned long ulMJD = 0;
	short sDay = 0;
	long long llTotalMicrosecond = 0;

	llTotalMicrosecond = TotalMicroseconds(pstcStartTime) + llDeltaTime;
	sDay = (short) (llTotalMicrosecond/86400000000);
	if ( llTotalMicrosecond < 0 )	sDay--;
	ulMJD = GregorianToModifierJulianData(pstcStartTime) + sDay ;
	*pstcStartTime = ModifierJulianDataToGregorian(ulMJD);
	llTotalMicrosecond -= sDay*86400000000;
	ToHour(llTotalMicrosecond,pstcStartTime);
}
long long GetGPSFormat(const struct SSimulationDateTime *pstcGMTTime,const long long llUTCOffsetMicroSecond)
{
	const struct SSimulationDateTime cTime19800106 = {1980,1,6,0,0,0,0,0};
	struct SSimulationDateTime stcGMTTimeTemp = *pstcGMTTime;
	unsigned short usAllDay = 0 ;
	unsigned char ucDay  = 0;
	union UGPSTime unGPSTime = {0};
	AddTimeSpan(&stcGMTTimeTemp,llUTCOffsetMicroSecond*(-1));
	usAllDay = (unsigned short) (GregorianToModifierJulianData(&stcGMTTimeTemp) - GregorianToModifierJulianData(&cTime19800106));
	unGPSTime.stcTime.usWeeks = usAllDay/7;
	ucDay = usAllDay%7;
	unGPSTime.stcTime.ulMillisecond = stcGMTTimeTemp.Millisecond + (ucDay*24*3600 + stcGMTTimeTemp.Hour*3600 + stcGMTTimeTemp.Minute*60 + stcGMTTimeTemp.Second)*1000;
	unGPSTime.stcTime.usMicrosecond = stcGMTTimeTemp.Microsecond;
	
	return unGPSTime.llTime;

}
struct SSimulationDateTime GetTimeFromGPSFormat(const long long llTime,const long long llUTCOffsetMicroSecond)
{
	union UGPSTime unGPSTime;
	struct SSimulationDateTime stcGMTTime = {0};
	const struct SSimulationDateTime cTime19800106 = {1980,1,6,0,0,0,0,0};
	unsigned long ulMJD  = 0;
	unGPSTime.llTime = llTime;
	ulMJD = unGPSTime.stcTime.usWeeks * 7 + unGPSTime.stcTime.ulMillisecond / (3600*24*1000) + GregorianToModifierJulianData(&cTime19800106);
	stcGMTTime = ModifierJulianDataToGregorian(ulMJD);
	stcGMTTime.Hour = (unsigned char) ( unGPSTime.stcTime.ulMillisecond%(3600*24*1000)/(3600*1000) );
	stcGMTTime.Minute = (unsigned char) ( unGPSTime.stcTime.ulMillisecond%(3600*1000)/(60 *1000) );
	stcGMTTime.Second = (unsigned char) ( unGPSTime.stcTime.ulMillisecond%(60*1000)/1000 );
	stcGMTTime.Millisecond = unGPSTime.stcTime.ulMillisecond%1000;
	stcGMTTime.Microsecond = unGPSTime.stcTime .usMicrosecond;
	AddTimeSpan(&stcGMTTime,llUTCOffsetMicroSecond);
	return stcGMTTime;

}
unsigned long GregorianToModifierJulianData(const struct SSimulationDateTime *pstcTime)
{
	double  A, B, D, JD, M, Y;
	struct SSimulationDateTime stcTime0 = *pstcTime;
	if ( stcTime0.Month < 3 )
	{
       stcTime0.Month = stcTime0.Month + 12;
       stcTime0.Year  = stcTime0.Year - 1;
    }
	Y = (double) stcTime0.Year;
	M = (double) stcTime0.Month;
	D = (double) stcTime0.Day;
	A  = floor(Y/100.0);
	B  = 2.0 - A + floor(A/4.0);
	JD = floor(365.25*(Y+4716.0)) + floor(30.6001*(M+1.0)) + D + B - 1524.5;
	JD = floor(JD+0.5);
	return (unsigned long)JD;
}

struct SSimulationDateTime ModifierJulianDataToGregorian(const unsigned long ulMJD)
{
	unsigned long J = ulMJD;
	unsigned long N = 4*(J+68569)/146097;
	unsigned long L1 = J+68569-(N*146097+3)/4;
	unsigned long Y1 = 4000*(L1+1)/1461001;
	unsigned long L2 = L1-1461*Y1/4+31;
	unsigned long M1 = 80*L2/2447;
	unsigned long L3 = M1/11;
	struct SSimulationDateTime stcTime = {0};
	stcTime.Day = (unsigned char)(L2-2447*M1/80);
	stcTime.Month = (unsigned char)(M1+2-12*L3);
	stcTime.Year = (unsigned short)(100*(N-49)+Y1+L3);
	stcTime.Hour = 0;	stcTime.Minute = 0;	stcTime.Second = 0;	stcTime.Millisecond = 0;	stcTime.Microsecond = 0;
	return stcTime;
}
unsigned char DayOfMonth(const unsigned short usYear,const unsigned char ucMonth)
{
    unsigned char w;
    switch (ucMonth) 
	{
		case 2:
			if( (usYear%4 == 0 && usYear%100 != 0) || usYear%400 == 0 )
				 w = 29;
			else
				 w = 28;
			break;
		case 4:  case 6:	case 9:  case 11:
			w = 30;
			break;
		default:
			w = 31;
    }
    return(w);
}
#endif
#ifdef USE_USER_SMOOTH
double GroupAverage(const unsigned long ulNo,double Value[])
{
	unsigned long i = 0;
	unsigned long ulMaxNo = 0;
	unsigned long ulMinNo = 0;
	double MinValue = 0.0;
	double MaxValue = 0.0;
	double AverageValue = 0.0;

	if ( ulNo == 1 ) return Value[0];
	if ( ulNo == 2 ) return ((Value[0]+Value[1])/2.0);

	MinValue = Value[0];	
	MaxValue = Value[0];

	for(i=1;i<ulNo;i++)
	{
		if ( Value[i] > MaxValue ) 
		{
			ulMaxNo = i;
			MaxValue = Value[i];
		}
		if ( Value[i] < MinValue )
		{
			ulMinNo = i;
			MinValue = Value[i];
		}
	}

	AverageValue = 0.0;
	for(i=0;i<ulNo;i++)
	{
		if (  i != ulMaxNo && i != ulMinNo )
			AverageValue += Value[i];
	}
	AverageValue /= (ulNo-2);

	return AverageValue;
}

#endif