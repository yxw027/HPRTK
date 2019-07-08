/* ---------------------------------------------------------------------------------------------------
* tide.cpp : tidal displacement corrections function class
* references :
*     [1] D.D.McCarthy, IERS Technical Note 21, IERS Conventions 1996, July 1996
*     [2] D.D.McCarthy and G.Petit, IERS Technical Note 32, IERS Conventions
*         2003, November 2003
*     [3] D.A.Vallado, Fundamentals of Astrodynamics and Applications 2nd ed,
*         Space Technology Library, 2004
*     [4] J.Kouba, A Guide to using International GNSS Service (IGS) products,
*         May 2009
*     [5] G.Petit and B.Luzum (eds), IERS Technical Note No. 36, IERS
*         Conventions (2010), 2010
* ------------------------------------------------------------------------------------------------- */

#include "GNSS/TidModel/tide.h"

#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
#define SQR(x)      ((x)*(x))

#define AS2R        (D2R/3600.0)    /* arc sec to radian */
#define GME         3.986004415E+14 /* earth gravitational constant */
#define GMS         1.327124E+20    /* sun gravitational constant */
#define GMM         4.902801E+12    /* moon gravitational constant */

/* earth, ocean loading and pole tide function ---------------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
tidecorr_t::tidecorr_t(){
	tut=gtime_t();
	for (int i=0; i<3; i++){
		E[i*3]=E[i*3+1]=E[i*3+2]=0.0;
		mode_dr[0][i]=mode_dr[1][i]=mode_dr[2][i]=0.0;
		denu[0][i]=denu[1][i]=0.0;
		sun_ecef[i]=moon_ecef[i]=0.0;
	}
	gmst=0.0;
	for (int i=0; i<5; i++) erpv[i]=0.0;

	tide_opt=0; ocean_par[0]=NULL; ocean_par[1]=NULL; er_par=NULL;
}
tidecorr_t::~tidecorr_t(){
	ocean_par[0]=NULL; ocean_par[1]=NULL; er_par=NULL;
}
/* Implementaion functions -------------------------------------------------------- */
/* protected functions ------------------------------------------------------------ */
/*iers mean pole (ref [7] eq.7.25) ------------------------------------------------ */
void tidecorr_t::iers_mean_pole(double *xp_bar,double *yp_bar){
	const double ep2000[]={ 2000,1,1,0,0,0 };
	double y,y2,y3;

	gtime_t t2000(ep2000);
	y=tut.timediff(t2000)/86400.0/365.25;

	if (y<3653.0/365.25) { /* until 2010.0 */
		y2=y*y; y3=y2*y;
		*xp_bar= 55.974+1.8243*y+0.18413*y2+0.007024*y3; /* (mas) */
		*yp_bar=346.346+1.7896*y-0.10729*y2-0.000908*y3;
	}
	else { /* after 2010.0 */
		*xp_bar= 23.513+7.6141*y; /* (mas) */
		*yp_bar=358.891-0.6287*y;
	}
}
/* solar/lunar tides (ref [2] 7) -------------------------------------------------- */
void tidecorr_t::tide_sl(const double *eu,const double *rp,double GMp,
	const double *pos,double *dr){
	const double H3=0.292,L3=0.015;
	double r,ep[3],latp,lonp,p,K2,K3,a,H2,L2,dp,du,cosp,sinl,cosl;
	int i;

	if ((r=norm(rp,3))<=0.0) return;

	for (i=0; i<3; i++) ep[i]=rp[i]/r;

	K2=GMp/GME*SQR(RE_WGS84)*SQR(RE_WGS84)/(r*r*r);
	K3=K2*RE_WGS84/r;
	latp=asin(ep[2]); lonp=atan2(ep[1],ep[0]);
	cosp=cos(latp); sinl=sin(pos[0]); cosl=cos(pos[0]);

	/* step1 in phase (degree 2) */
	p=(3.0*sinl*sinl-1.0)/2.0;
	H2=0.6078-0.0006*p;
	L2=0.0847+0.0002*p;
	a=dot(ep,eu,3);
	dp=K2*3.0*L2*a;
	du=K2*(H2*(1.5*a*a-0.5)-3.0*L2*a*a);

	/* step1 in phase (degree 3) */
	dp+=K3*L3*(7.5*a*a-1.5);
	du+=K3*(H3*(2.5*a*a*a-1.5*a)-L3*(7.5*a*a-1.5)*a);

	/* step1 out-of-phase (only radial) */
	du+=3.0/4.0*0.0025*K2*sin(2.0*latp)*sin(2.0*pos[0])*sin(pos[1]-lonp);
	du+=3.0/4.0*0.0022*K2*cosp*cosp*cosl*cosl*sin(2.0*(pos[1]-lonp));

	dr[0]=dp*ep[0]+du*eu[0];
	dr[1]=dp*ep[1]+du*eu[1];
	dr[2]=dp*ep[2]+du*eu[2];
}
/* compute pole tide (ref [7] eq.7.26) -------------------------------------------- */
void tidecorr_t::tide_pole(){
	double xp_bar,yp_bar,m1,m2,cosl,sinl;

	/* iers mean pole (mas) */
	iers_mean_pole(&xp_bar,&yp_bar);

	/* ref [7] eq.7.24 */
	m1= erpv[0]/AS2R-xp_bar*1E-3; /* (as) */
	m2=-erpv[1]/AS2R+yp_bar*1E-3;

	/* sin(2*theta) = sin(2*phi), cos(2*theta)=-cos(2*phi) */
	cosl=cos(blhpos[1]);
	sinl=sin(blhpos[1]);
	denu[1][0]=  9E-3*cos(blhpos[0])    *(m1*sinl-m2*cosl); /* de= Slambda (m) */
	denu[1][1]= -9E-3*cos(2.0*blhpos[0])*(m1*cosl+m2*sinl); /* dn=-Stheta  (m) */
	denu[1][2]=-32E-3*sin(2.0*blhpos[0])*(m1*cosl+m2*sinl); /* du= Sr      (m) */
}
/* compute ocean-loading tide (ref [2] 7) ----------------------------------------- */
void tidecorr_t::tide_ocean(int rovbas){
	if (!ocean_par[rovbas]) return ;
	const double args[][5]={
		{ 1.40519E-4, 2.0,-2.0, 0.0, 0.00 },  /* M2 */
		{ 1.45444E-4, 0.0, 0.0, 0.0, 0.00 },  /* S2 */
		{ 1.37880E-4, 2.0,-3.0, 1.0, 0.00 },  /* N2 */
		{ 1.45842E-4, 2.0, 0.0, 0.0, 0.00 },  /* K2 */
		{ 0.72921E-4, 1.0, 0.0, 0.0, 0.25 },  /* K1 */
		{ 0.67598E-4, 1.0,-2.0, 0.0,-0.25 },  /* O1 */
		{ 0.72523E-4,-1.0, 0.0, 0.0,-0.25 },  /* P1 */
		{ 0.64959E-4, 1.0,-3.0, 1.0,-0.25 },  /* Q1 */
		{ 0.53234E-5, 0.0, 2.0, 0.0, 0.00 },  /* Mf */
		{ 0.26392E-5, 0.0, 1.0,-1.0, 0.00 },  /* Mm */
		{ 0.03982E-5, 2.0, 0.0, 0.0, 0.00 }   /* Ssa */
	};
	const double ep1975[]={ 1975,1,1,0,0,0 };
	double fday,days,t,t2,t3,a[5],ang,dp[3]={ 0 };
	int i,j;

	/* angular argument: see subroutine arg.f for reference [1] */
	gtime_t time=tut,t1975(ep1975);
	time.time2epoch();
	fday=time.ep[3]*3600.0+time.ep[4]*60.0+time.ep[5];
	time.ep[3]=time.ep[4]=time.ep[5]=0.0;
	days=(time.epoch2time(time.ep)->timediff(t1975))/86400.0+1.0;
	t=(27392.500528+1.000000035*days)/36525.0;
	t2=t*t; t3=t2*t;

	a[0]=fday;
	a[1]=(279.69668+36000.768930485*t+3.03E-4*t2)*D2R; /* H0 */
	a[2]=(270.434358+481267.88314137*t-0.001133*t2+1.9E-6*t3)*D2R; /* S0 */
	a[3]=(334.329653+4069.0340329577*t-0.010325*t2-1.2E-5*t3)*D2R; /* P0 */
	a[4]=2.0*PI;

	/* displacements by 11 constituents */
	for (i=0; i<11; i++) {
		ang=0.0;
		for (j=0; j<5; j++) ang+=a[j]*args[i][j];
		for (j=0; j<3; j++) dp[j]+=ocean_par[rovbas][j+i*6]*cos(ang-ocean_par[rovbas][j+3+i*6]*D2R);
	}
	denu[0][0]=-dp[1];
	denu[0][1]=-dp[2];
	denu[0][2]= dp[0];
}
/* compute solid earth tide (ref [2] 7) ------------------------------------------- */
void tidecorr_t::tide_solid(){
	double dr1[3],dr2[3],eu[3],du,dn,sinl,sin2l;

	/* step1: time domain */
	eu[0]=E[2]; eu[1]=E[5]; eu[2]=E[8];
	tide_sl(eu,sun_ecef,GMS,blhpos,dr1);
	tide_sl(eu,moon_ecef,GMM,blhpos,dr2);

	/* step2: frequency domain, only K1 radial */
	sin2l=sin(2.0*blhpos[0]);
	du=-0.012*sin2l*sin(gmst+blhpos[1]);

	mode_dr[0][0]=dr1[0]+dr2[0]+du*E[2];
	mode_dr[0][1]=dr1[1]+dr2[1]+du*E[5];
	mode_dr[0][2]=dr1[2]+dr2[2]+du*E[8];

	/* eliminate permanent deformation */
	if (tide_opt&8) {
		sinl=sin(blhpos[0]);
		du=0.1196*(1.5*sinl*sinl-0.5);
		dn=0.0247*sin2l;
		mode_dr[0][0]+=du*E[2]+dn*E[1];
		mode_dr[0][1]+=du*E[5]+dn*E[4];
		mode_dr[0][2]+=du*E[8]+dn*E[7];
	}
}
/* public funtions ---------------------------------------------------------------- */
/* initialize ocean loading parameter pointer ------------------------------------- */
void tidecorr_t::init_otl(prcopt_t *opt,nav_t *nav){
	tide_opt=opt->tidecorr;
	ocean_par[0]=nav->ocean_par[0];
	ocean_par[1]=nav->ocean_par[1];
}
/* initialize earth rotation parameter pointor ------------------------------------ */
void tidecorr_t::init_erp(prcopt_t *opt,nav_t *nav){
	er_par=&nav->erp;
}
/* compute tidal displacement corretion ----------------------------------------------
* args   : gtime_t time     I   time in utc
*          int     rovbas   I   0:rover,1:base
*          double  *xyz     I   site position (ecef) (m)
*          double  *dr      O   displacement by earth tides (ecef) (m)
* return : none
* notes  : see ref [1], [2] chap 7
*          see ref [4] 5.2.1, 5.2.2, 5.2.3
*          ver.2.4.0 does not use ocean loading and pole tide corrections
* --------------------------------------------------------------------------------- */
void tidecorr_t::tidecorr(gtime_t time,const int rovbas,const double *xyz,double *dr){
	for (int i=0; i<3; i++) dr[i]=0.0;

	if (er_par) geterpv(er_par,time,erpv);
	
	tut=time; tut.timeadd(erpv[2]);

	if ((radius=norm(xyz,3))<=0.0) return;

	/* station blh position */
	blhpos[0]=asin(xyz[2]/radius);
	blhpos[1]=atan2(xyz[1],xyz[0]);
	xyz2enu(blhpos,E);

	/* solid earth tide */
	if (tide_opt&1){
		/* sun and moon position in ecef */
		sunmoonpos(time,erpv,sun_ecef,moon_ecef,&gmst);
		/* compute solid tide correction */
		tide_solid();
		/* update earth tide correction vector */
		for (int i=0; i<3; i++) dr[i]+=mode_dr[0][i];
	}
	/* ocean-loading tide */
	if ((tide_opt&2)&&ocean_par[rovbas]){
		tide_ocean(rovbas);
		matmul_pnt("TN",3,1,3,1.0,E,denu[0],0.0,mode_dr[1]);
		for (int i=0; i<3; i++) dr[i]+=mode_dr[1][i];
	}
	/* pole tide */
	if (tide_opt&4){
		tide_pole();
		matmul_pnt("TN",3,1,3,1.0,E,denu[1],0.0,mode_dr[2]);
		for (int i=0; i<3; i++) dr[i]+=mode_dr[2][i];
	}
}