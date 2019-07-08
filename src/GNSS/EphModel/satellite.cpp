#include "GNSS/EphModel/satellite.h"

#include "BaseFunction/basefunction.h"

/* constants and macros --------------------------------------------------------------------------- */
/* for broadcast ephemeris */
#define SQR(x)   ((x)*(x))
#define RE_GLO   6378136.0        /* radius of earth (m)            ref [2] */
#define MU_GPS   3.9860050E14     /* gravitational constant         ref [1] */
#define MU_GLO   3.9860044E14     /* gravitational constant         ref [2] */
#define MU_GAL   3.986004418E14   /* earth gravitational constant   ref [7] */
#define MU_CMP   3.986004418E14   /* earth gravitational constant   ref [9] */
#define J2_GLO   1.0826257E-3     /* 2nd zonal harmonic of geopot   ref [2] */
#define OMGE_GLO 7.292115E-5      /* earth angular velocity (rad/s) ref [2] */
#define OMGE_GAL 7.2921151467E-5  /* earth angular velocity (rad/s) ref [7] */
#define OMGE_CMP 7.292115E-5      /* earth angular velocity (rad/s) ref [9] */
#define SIN_5 -0.0871557427476582 /* sin(-5.0 deg) */
#define COS_5  0.9961946980917456 /* cos(-5.0 deg) */
#define ERREPH_GLO 5.0            /* error of glonass ephemeris (m) */
#define TSTEP    60.0             /* integration step glonass ephemeris (s) */
#define RTOL_KEPLER 1E-13         /* relative tolerance for Kepler equation */
#define DEFURASSR 0.15            /* default accurary of ssr corr (m) */
#define MAXECORSSR 10.0           /* max orbit correction of ssr (m) */
#define MAXCCORSSR (1E-6*CLIGHT)  /* max clock correction of ssr (m) */
#define MAXAGESSR 90.0            /* max age of ssr orbit and clock (s) */
#define MAXAGESSR_HRCLK 10.0      /* max age of ssr high-rate clock (s) */
#define STD_BRDCCLK 30.0          /* error of broadcast clock (m) */
#define MAX_ITER_KEPLER 30        /* max number of iteration of Kelpler */
/* for precise ephemeris */
#define NMAX        10              /* order of polynomial interpolation */
#define MAXDTE      900.0           /* max time difference to ephem time (s) */
#define EXTERR_CLK  1E-3            /* extrapolation error for clock (m/s) */
#define EXTERR_EPH  5E-7            /* extrapolation error for ephem (m/s^2) */
/* for qzss lex ephemeris */
#define LEXEPHMAXAGE    360.0           /* max age of lex ephemeris (s) */

static const double ura_eph[]={         /* ura values (ref [3] 20.3.3.3.1.1) */
	2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
	3072.0,6144.0,0.0
};
static const double ura_nominal[]={     /* ura nominal values */
	2.0,2.8,4.0,5.7,8.0,11.3,16.0,32.0,64.0,128.0,256.0,512.0,1024.0,
	2048.0,4096.0,8192.0
};

/* satellite functions ---------------------------------------------------------------------------- */
/* parent class of satellite functions ------------------------------------------------------------ */
/* Constructors ------------------------------------------------------------------- */
satellite_t::satellite_t(){
}
satellite_t::~satellite_t(){
	satantfunc=NULL;
}
/* Implementation functions ------------------------------------------------------- */
int satellite_t::satclk(obsd_t *data,const nav_t *nav){
	return 0;
}
int satellite_t::satpos(obsd_t *data,int iode,const nav_t *nav){
	return 0;
}
/* compute satellite positions and clocks ----------------------------------------- */
int satellite_t::satposclk(obs_t *obs,const nav_t *nav){
	int num=0;

	for (int i=0;i<obs->n&&i<MAXOBS;i++){
		/* reset satellite data of observation */
		obs->data[i].satreset();

		/* search any psuedorange */
		if (obs->data[i].sigtime_opsr()!=1) continue;

		/* satellite clock bias to correct signal time */
		if (!satclk(&obs->data[i],nav)) {
			obs->data[i].errmsg = "no satellite clock bias!";
			continue;
		}
		obs->data[i].sigtime_sclk();

		if (!satpos(&obs->data[i],999,nav)){
			obs->data[i].errmsg = "no ephemeris!";
			continue;
		}
		if (obs->data[i].svh!=-1) num++;
	}
	/* end */
	return num;
}

/* subclass of satellite functions ---------------------------------------------------------------- */
/* broadcast ephemeris ---------------------------------------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
broadcast_t::broadcast_t(){
}
broadcast_t::~broadcast_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* broadcast functions ------------------------------------------------------------ */

/* variance by ura ephemeris (ref [1] 20.3.3.3.1.1) ------------------------------- */
double broadcast_t::var_uraeph(int ura)
{
	return ura<0||15<ura ? SQR(8192.0) : SQR(ura_nominal[ura]);
}
/* variance by ura ssr (ref [4]) -------------------------------------------------- */
double broadcast_t::var_urassr(int ura)
{
	double std;
	if (ura<= 0) return SQR(DEFURASSR);
	if (ura>=63) return SQR(5.4665);
	std=(pow(3.0,(ura>>3)&7)*(1.0+(ura&7)/4.0)-1.0)*1E-3;
	return SQR(std);
}
/* glonass orbit differential equations ------------------------------------------- */
void broadcast_t::deq(const double *x,double *xdot,const double *acc)
{
	double a,b,c,r2=dot(x,x,3),r3=r2*sqrt(r2),omg2=SQR(OMGE_GLO);

	if (r2<=0.0) {
		xdot[0]=xdot[1]=xdot[2]=xdot[3]=xdot[4]=xdot[5]=0.0;
		return;
	}
	/* ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
	a=1.5*J2_GLO*MU_GLO*SQR(RE_GLO)/r2/r3; /* 3/2*J2*mu*Ae^2/r^5 */
	b=5.0*x[2]*x[2]/r2;                    /* 5*z^2/r^2 */
	c=-MU_GLO/r3-a*(1.0-b);                /* -mu/r^3-a(1-b) */
	xdot[0]=x[3]; xdot[1]=x[4]; xdot[2]=x[5];
	xdot[3]=(c+omg2)*x[0]+2.0*OMGE_GLO*x[4]+acc[0];
	xdot[4]=(c+omg2)*x[1]-2.0*OMGE_GLO*x[3]+acc[1];
	xdot[5]=(c-2.0*a)*x[2]+acc[2];
}
/* glonass position and velocity by numerical integration ------------------------- */
void broadcast_t::glorbit(double t,double *x,const double *acc)
{
	double k1[6],k2[6],k3[6],k4[6],w[6];
	int i;

	deq(x,k1,acc); for (i=0; i<6; i++) w[i]=x[i]+k1[i]*t/2.0;
	deq(w,k2,acc); for (i=0; i<6; i++) w[i]=x[i]+k2[i]*t/2.0;
	deq(w,k3,acc); for (i=0; i<6; i++) w[i]=x[i]+k3[i]*t;
	deq(w,k4,acc);
	for (i=0; i<6; i++) x[i]+=(k1[i]+2.0*k2[i]+2.0*k3[i]+k4[i])*t/6.0;
}
/* select GPS/GAL/QZS/CMP ephemeris ----------------------------------------------- */
int broadcast_t::seleph(obsd_t *data,int iode,const nav_t *nav){
	double t,tmax,tmin;
	int neph=-1;

	switch (satsys(data->sat,NULL)){
		case SYS_QZS: tmax=MAXDTOE_QZS+1.0; break;
		case SYS_GAL: tmax=MAXDTOE_GAL+1.0; break;
		case SYS_CMP: tmax=MAXDTOE_CMP+1.0; break;
		default: tmax=MAXDTOE+1.0; break;
		}
	tmin=tmax+1.0;

	for (int i=0; i<nav->n; i++) {
		if (nav->eph[i].sat!=data->sat) continue;
		if (iode>=0&&nav->eph[i].iode!=iode) continue;
		if ((t=fabs(nav->eph[i].toe.timediff(data->time)))>tmax) continue;
		if (iode>=0) return i;
		if (t<=tmin) { neph=i; tmin=t; } /* toe closest to time */
	}
	if (iode>=0||neph<0) { 
		data->errmsg="no broadcast ephemeris!";
		return -1;
	}
	return neph;
}
/* select GLO ephemeris ----------------------------------------------------------- */
int broadcast_t::selgeph(obsd_t *data, int iode, const nav_t *nav){
	double t,tmax=MAXDTOE_GLO,tmin=tmax+1.0;
	int ngeph=-1;

	for (int i=0; i<nav->ng; i++) {
		if (nav->geph[i].sat!=data->sat) continue;
		if (iode>=0&&nav->geph[i].iode!=iode) continue;
		if ((t=fabs(nav->geph[i].toe.timediff(data->time)))>tmax) continue;
		if (iode>=0) return i;
		if (t<=tmin) { ngeph=i; tmin=t; } /* toe closest to time */
	}
	if (iode>=0||ngeph<0) {
		data->errmsg="no glonass broadcast ephemeris!";
		return -1;
	}
	return ngeph;
}
/* select SBS ephemeris ----------------------------------------------------------- */
int broadcast_t::selseph(obsd_t *data,const nav_t *nav){
	double t,tmax=MAXDTOE_SBS,tmin=tmax+1.0;
	int nseph=-1;

	for (int i=0; i<nav->ns; i++) {
		if (nav->seph[i].sat!=data->sat) continue;
		if ((t=fabs(nav->seph[i].t0.timediff(data->time)))>tmax) continue;
		if (t<=tmin) { nseph=i; tmin=t; } /* toe closest to time */
	}
	if (nseph<0) {
		data->errmsg="no SBS broadcast ephemeris!";
		return -1;
	}
	return nseph;
}
/* position functions ------------------------------------------------------------- */
/* position from GPS/GAL/QZS/CMP ephemeris ---------------------------------------- */
void broadcast_t::eph2pos(obsd_t *data,const eph_t eph){
	double tk,M,E,Ek,sinE,cosE,u,r,i,O,sin2u,cos2u,x,y,sinO,cosO,cosi,mu,omge;
	double xg,yg,zg,sino,coso;
	int n,sys,prn;

	if (eph.A<=0.0) {
		data->satreset();
		return;
	}
	tk=data->sigtime.timediff(eph.toe);

	switch ((sys=satsys(eph.sat,&prn))) {
	case SYS_GAL: mu=MU_GAL; omge=OMGE_GAL; break;
	case SYS_CMP: mu=MU_CMP; omge=OMGE_CMP; break;
	default:      mu=MU_GPS; omge=OMGE;     break;
	}
	M=eph.M0+(sqrt(mu/(eph.A*eph.A*eph.A))+eph.deln)*tk;

	for (n=0,E=M,Ek=0.0; fabs(E-Ek)>RTOL_KEPLER&&n<MAX_ITER_KEPLER; n++) {
		Ek=E; E-=(E-eph.e*sin(E)-M)/(1.0-eph.e*cos(E));
	}
	if (n>=MAX_ITER_KEPLER) {
		return;
	}
	sinE=sin(E); cosE=cos(E);

	u=atan2(sqrt(1.0-eph.e*eph.e)*sinE,cosE-eph.e)+eph.omg;
	r=eph.A*(1.0-eph.e*cosE);
	i=eph.i0+eph.idot*tk;
	sin2u=sin(2.0*u); cos2u=cos(2.0*u);
	u+=eph.cus*sin2u+eph.cuc*cos2u;
	r+=eph.crs*sin2u+eph.crc*cos2u;
	i+=eph.cis*sin2u+eph.cic*cos2u;
	x=r*cos(u); y=r*sin(u); cosi=cos(i);

	/* beidou geo satellite (ref [9]) */
	if (sys==SYS_CMP&&prn<=5) {
		O=eph.OMG0+eph.OMGd*tk-omge*eph.toes;
		sinO=sin(O); cosO=cos(O);
		xg=x*cosO-y*cosi*sinO;
		yg=x*sinO+y*cosi*cosO;
		zg=y*sin(i);
		sino=sin(omge*tk); coso=cos(omge*tk);
		data->posvel[0]= xg*coso+yg*sino*COS_5+zg*sino*SIN_5;
		data->posvel[1]=-xg*sino+yg*coso*COS_5+zg*coso*SIN_5;
		data->posvel[2]=-yg*SIN_5+zg*COS_5;
	}
	else {
		O=eph.OMG0+(eph.OMGd-omge)*tk-omge*eph.toes;
		sinO=sin(O); cosO=cos(O);
		data->posvel[0]=x*cosO-y*cosi*sinO;
		data->posvel[1]=x*sinO+y*cosi*cosO;
		data->posvel[2]=y*sin(i);
	}
	tk=data->sigtime.timediff(eph.toc);
	data->dts[0]=eph.f0+eph.f1*tk+eph.f2*tk*tk;

	/* relativity correction */
	data->dts[0]-=2.0*sqrt(mu*eph.A)*eph.e*sinE/SQR(CLIGHT);

	/* position and clock error variance */
	data->svar=var_uraeph(eph.sva);
}
/* position bias from GLO ephemeris ----------------------------------------------- */
void broadcast_t::geph2pos(obsd_t *data,const geph_t geph){
	double t,tt,x[6];
	int i;

	t=data->sigtime.timediff(geph.toe);

	data->dts[0]=-geph.taun+geph.gamn*t;

	for (i=0; i<3; i++) {
		x[i]=geph.pos[i];
		x[i+3]=geph.vel[i];
	}
	for (tt=t<0.0 ? -TSTEP : TSTEP; fabs(t)>1E-9; t-=tt) {
		if (fabs(t)<TSTEP) tt=t;
		glorbit(tt,x,geph.acc);
	}
	for (i=0; i<3; i++) data->posvel[i]=x[i];

	data->svar=SQR(ERREPH_GLO);
}
/* position bias from SBS ephemeris ----------------------------------------------- */
void broadcast_t::seph2pos(obsd_t *data,const seph_t seph){
	double t;
	int i;

	t=data->sigtime.timediff(seph.t0);

	for (i=0; i<3; i++) {
		data->posvel[i]=seph.pos[i]+seph.vel[i]*t+seph.acc[i]*t*t/2.0;
	}
	data->dts[0]=seph.af0+seph.af1*t;

	data->svar=var_uraeph(seph.sva);
}
/* clock functions ---------------------------------------------------------------- */
/* clock bias from GPS/GAL/QZS/CMP ephemeris -------------------------------------- */
void broadcast_t::eph2clk(obsd_t *data,const eph_t eph){
	double t;

	t=data->sigtime.timediff(eph.toc);

	for (int i=0; i<2; i++) {
		t-=eph.f0+eph.f1*t+eph.f2*t*t;
	}
	data->dts[0]=eph.f0+eph.f1*t+eph.f2*t*t;
}
/* clock bias from GLO ephemeris -------------------------------------------------- */
void broadcast_t::geph2clk(obsd_t *data,const geph_t geph){
	double t;

	t=data->sigtime.timediff(geph.toe);

	for (int i=0; i<2; i++) {
		t-=-geph.taun+geph.gamn*t;
	}
	data->dts[0]=-geph.taun+geph.gamn*t;
}
/* clock bias from SBS ephemeris -------------------------------------------------- */
void broadcast_t::seph2clk(obsd_t *data,const seph_t seph){
	double t;

	t=data->sigtime.timediff(seph.t0);

	for (int i=0; i<2; i++) {
		t-=seph.af0+seph.af1*t;
	}
	data->dts[0]=seph.af0+seph.af1*t;
}
/* broadcast satellite position function ------------------------------------------ */
int broadcast_t::broadpos(obsd_t *data,int iode,const nav_t *nav){
	obsd_t d1E_3;
	/* satellite system */
	int sys=satsys(data->sat,NULL);
	data->svh=-1;

	if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP){
		int neph=seleph(data,iode,nav);
		if (neph>=0) eph2pos(data,nav->eph[neph]);
		else return 0;
		d1E_3.sigtime=data->sigtime; d1E_3.sigtime.timeadd(1E-3);
		eph2pos(&d1E_3,nav->eph[neph]);
		if (data->svar<129.0) data->svh = nav->eph[neph].svh;
	}
	else if (sys==SYS_GLO){
		int ngeph=selgeph(data,iode,nav);
		if (ngeph>=0) geph2pos(data,nav->geph[ngeph]);
		else return 0;
		d1E_3.sigtime=data->sigtime; d1E_3.sigtime.timeadd(1E-3);
		geph2pos(&d1E_3,nav->geph[ngeph]);
		data->svh=nav->geph[ngeph].svh;
	}
	else if (sys==SYS_SBS){
		int nseph=selseph(data,nav);
		if (nseph>=0) seph2pos(data,nav->seph[nseph]);
		else return 0;
		d1E_3.sigtime=data->sigtime; d1E_3.sigtime.timeadd(1E-3);
		seph2pos(&d1E_3,nav->seph[nseph]);
		if (data->svar<129.0) data->svh=nav->seph[nseph].svh;
	}
	else return 0;

	/* satellite velocity and clock drift by differential approx */
	for (int i=0; i<3; i++) data->posvel[i+3]=(d1E_3.posvel[i]-data->posvel[i])/1E-3;
	data->dts[1]=(d1E_3.dts[0]-data->dts[0])/1E-3;

	/* no satellite antenna offset correction becasue broadcast navigation ephemeris
	 * refer to the antenna phase center of satellite */

	return 1;
}
/* broadcast satellite clocks function -------------------------------------------- */
int broadcast_t::satclk(obsd_t *data,const nav_t *nav){
	/* satellite system */
	int sys=satsys(data->sat,NULL);
	if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP){
		int neph = seleph(data,-1,nav);
		if (neph>=0) eph2clk(data,nav->eph[neph]);
		else return 0;
	}
	else if (sys==SYS_GLO){
		int ngeph = selgeph(data,-1,nav);
		if (ngeph>=0) geph2clk(data,nav->geph[ngeph]);
		else return 0;
	}
	else if (sys==SYS_SBS){
		int nseph = selseph(data,nav);
		if (nseph>=0) seph2clk(data,nav->seph[nseph]);
		else return 0;
	}
	else return 0;
	
	return 1;
}
/* broadcast satellite position function called by satposclk() ------------ */
int broadcast_t::satpos(obsd_t *data,int flag,const nav_t *nav){
	data->svh=0;
	return broadpos(data,-1,nav);
}


/* broadcast ephemeris with sbas correction ------------------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
broadsbas_t::broadsbas_t(){
}
broadsbas_t::~broadsbas_t(){
}
/* Implementation functions -------------------------------------------------------- */
/* broadcast satellite position function with sbas correction ---------------------- */
int broadsbas_t::satpos(obsd_t *data,int iode,const nav_t *nav){
	int nsbas;
	/* search sbas satellite correciton */
	for (nsbas=0; nsbas<nav->sbssat.nsat; nsbas++) 
		if (nav->sbssat.sat[nsbas].sat==data->sat) break;
	if (nsbas>=nav->sbssat.nsat){
		broadpos(data,-1,nav);
		data->svh=-1;
		return 0;
	}
	/* satellite postion and clock from broadcast ephemeris */
	if (!broadpos(data,nav->sbssat.sat[nsbas].lcorr.iode,nav)) return 0;
	if (nav->sbssat.sbssatcorr(data)) return 1;
	data->svh=-1;
	return 0;
}

/* broadcast ephemeris with ssr_apc correction ---------------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
broadssrapc_t::broadssrapc_t(){
}
broadssrapc_t::~broadssrapc_t(){
}
/* base functions ----------------------------------------------------------------- */
/* broadcast satellite position function with ssr_apc correction ------------------ */
int broadssrapc_t::satpos(obsd_t *data,int iode,const nav_t *nav){
	/* pointer to the ssr_t in nav */
	const ssr_t *ssr=nav->ssr+data->sat-1;
	/* satellite system */
	int sys=satsys(data->sat,NULL);
	int neph=-1;
	double t1,t2,t3,er[3],ea[3],ec[3],rc[3],deph[3],dclk,dant[3]={ 0 },tk;


	if (!ssr->t0[0].time) {
		return 0;
	}
	if (!ssr->t0[1].time) {
		return 0;
	}
	/* inconsistency between orbit and clock correction */
	if (ssr->iod[0]!=ssr->iod[1]) {
		data->svh=-1;
		return 0;
	}
	t1=data->sigtime.timediff(ssr->t0[0]);
	t2=data->sigtime.timediff(ssr->t0[1]);
	t3=data->sigtime.timediff(ssr->t0[2]);

	/* ssr orbit and clock correction (ref [4]) */
	if (fabs(t1)>MAXAGESSR||fabs(t2)>MAXAGESSR) {
		data->svh=-1;
		return 0;
	}
	if (ssr->udi[0]>=1.0) t1-=ssr->udi[0]/2.0;
	if (ssr->udi[1]>=1.0) t2-=ssr->udi[0]/2.0;

	for (int i=0; i<3; i++) deph[i]=ssr->deph[i]+ssr->ddeph[i]*t1;
	dclk=ssr->dclk[0]+ssr->dclk[1]*t2+ssr->dclk[2]*t2*t2;

	/* ssr highrate clock correction (ref [4]) */
	if (ssr->iod[0]==ssr->iod[2]&&ssr->t0[2].time&&fabs(t3)<MAXAGESSR_HRCLK) {
		dclk+=ssr->hrclk;
	}
	if (norm(deph,3)>MAXECORSSR||fabs(dclk)>MAXCCORSSR) {
		data->svh=-1;
		return 0;
	}
	/* satellite postion and clock by broadcast ephemeris */
	if (!broadpos(data,ssr->iode,nav)) return 0;

	/* satellite clock for gps, galileo and qzss */
	sys=satsys(data->sat,NULL);
	if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP) {
		if ((neph=seleph(data,ssr->iode,nav))<0) return 0;

		/* satellite clock by clock parameters */
		tk=data->sigtime.timediff(nav->eph[neph].toc);
		data->dts[0]=nav->eph[neph].f0+nav->eph[neph].f1*tk+nav->eph[neph].f2*tk*tk;
		data->dts[1]=nav->eph[neph].f1+2.0*nav->eph[neph].f2*tk;

		/* relativity correction */
		data->dts[0]-=2.0*dot(data->posvel,data->posvel+3,3)/CLIGHT/CLIGHT;
	}
	/* radial-along-cross directions in ecef */
	if (!normv3(data->posvel+3,ea)) return 0;
	cross3(data->posvel,data->posvel+3,rc);
	if (!normv3(rc,ec)) {
		data->svh=-1;
		return 0;
	}
	cross3(ea,ec,er);

	/* satellite antenna offset correction */
	for (int i=0; i<3; i++) {
		data->posvel[i]+=-(er[i]*deph[0]+ea[i]*deph[1]+ec[i]*deph[2])+dant[i];
	}
	/* t_corr = t_sv - (dts(brdc) + dclk(ssr) / CLIGHT) (ref [10] eq.3.12-7) */
	data->dts[0]+=dclk/CLIGHT;

	/* variance by ssr ura */
	data->svar=var_urassr(ssr->ura);

	return 1;
}
/* broadcast ephemeris with ssr_com correction ---------------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
broadssrcom_t::broadssrcom_t(){
}
broadssrcom_t::~broadssrcom_t(){
}
/* base functions ----------------------------------------------------------------- */
/* broadcast satellite position function with ssr_com correction ------------------ */
int broadssrcom_t::satpos(obsd_t *data,int iode,const nav_t *nav){
	/* pointer to the ssr_t in nav */
	const ssr_t *ssr=nav->ssr+data->sat-1;
	/* satellite system */
	int sys=satsys(data->sat,NULL);
	int neph=-1;
	double t1,t2,t3,er[3],ea[3],ec[3],rc[3],deph[3],dclk,dant[3]={ 0 },tk;


	if (!ssr->t0[0].time) {
		return 0;
	}
	if (!ssr->t0[1].time) {
		return 0;
	}
	/* inconsistency between orbit and clock correction */
	if (ssr->iod[0]!=ssr->iod[1]) {
		data->svh=-1;
		return 0;
	}
	t1=data->sigtime.timediff(ssr->t0[0]);
	t2=data->sigtime.timediff(ssr->t0[1]);
	t3=data->sigtime.timediff(ssr->t0[2]);

	/* ssr orbit and clock correction (ref [4]) */
	if (fabs(t1)>MAXAGESSR||fabs(t2)>MAXAGESSR) {
		data->svh=-1;
		return 0;
	}
	if (ssr->udi[0]>=1.0) t1-=ssr->udi[0]/2.0;
	if (ssr->udi[1]>=1.0) t2-=ssr->udi[0]/2.0;

	for (int i=0; i<3; i++) deph[i]=ssr->deph[i]+ssr->ddeph[i]*t1;
	dclk=ssr->dclk[0]+ssr->dclk[1]*t2+ssr->dclk[2]*t2*t2;

	/* ssr highrate clock correction (ref [4]) */
	if (ssr->iod[0]==ssr->iod[2]&&ssr->t0[2].time&&fabs(t3)<MAXAGESSR_HRCLK) {
		dclk+=ssr->hrclk;
	}
	if (norm(deph,3)>MAXECORSSR||fabs(dclk)>MAXCCORSSR) {
		data->svh=-1;
		return 0;
	}
	/* satellite postion and clock by broadcast ephemeris */
	if (!broadpos(data,ssr->iode,nav)) return 0;

	/* satellite clock for gps, galileo and qzss */
	sys=satsys(data->sat,NULL);
	if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP) {
		if ((neph=seleph(data,ssr->iode,nav))<0) return 0;

		/* satellite clock by clock parameters */
		tk=data->sigtime.timediff(nav->eph[neph].toc);
		data->dts[0]=nav->eph[neph].f0+nav->eph[neph].f1*tk+nav->eph[neph].f2*tk*tk;
		data->dts[1]=nav->eph[neph].f1+2.0*nav->eph[neph].f2*tk;

		/* relativity correction */
		data->dts[0]-=2.0*dot(data->posvel,data->posvel+3,3)/CLIGHT/CLIGHT;
	}
	/* radial-along-cross directions in ecef */
	if (!normv3(data->posvel+3,ea)) return 0;
	cross3(data->posvel,data->posvel+3,rc);
	if (!normv3(rc,ec)) {
		data->svh=-1;
		return 0;
	}
	cross3(ea,ec,er);

	/* satellite antenna offset correction */
	satantfunc->satantoff(data,nav);

	for (int i=0; i<3; i++) {
		data->posvel[i]+=-(er[i]*deph[0]+ea[i]*deph[1]+ec[i]*deph[2]);
	}
	/* t_corr = t_sv - (dts(brdc) + dclk(ssr) / CLIGHT) (ref [10] eq.3.12-7) */
	data->dts[0]+=dclk/CLIGHT;

	/* variance by ssr ura */
	data->svar=var_urassr(ssr->ura);

	return 1;
}

/* precise ephemeris ------------------------------------------------------------------------------ */
/* Constg */
/* Constructors ------------------------------------------------------------------- */
preciseph_t::preciseph_t(){
}
preciseph_t::~preciseph_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* base precise ephemeris functions ----------------------------------------------- */
/* polynomial interpolation by Lagrange's algorithm (time interpolation) -- */
double preciseph_t::interpolLaR(const double *dt,gtime_t *ptime,const double *ppos,
	int n){
	int i,j;
	double item=1.0,value=0.0;

	for (i=0; i<n; i++) {
		item=1.0;
		for (j=0; j<i; j++) item*=dt[j]/ptime[j].timediff(ptime[i]);
		for (j=i+1; j<n; j++) item*=dt[j]/ptime[j].timediff(ptime[i]);
		item*=ppos[i];
		value+=item;
	}

	return value;
}
/* precise satellite position of one epoch -------------------------------------------
* use 9 orders Lagrange interpolation
----------------------------------------------------------------------------------- */
int preciseph_t::precisepos(obsd_t *data,const nav_t *nav){
	gtime_t TNMAX[NMAX];
	double t[NMAX],p[3][NMAX],c[2],std=0.0,s[3],sinl,cosl;
	int i,j,k,index;

	data->posvel[0]=data->posvel[1]=data->posvel[2]=data->dts[0]=0.0;

	if (nav->ne<NMAX||
		data->sigtime.timediff(nav->peph[0].time)<-MAXDTE||
		data->sigtime.timediff(nav->peph[nav->ne-1].time)>MAXDTE) {
		data->errmsg="no percise ephemeris!";
		return 0;
	}
	/* binary search */
	for (i=0,j=nav->ne-1; i<j;) {
		k=(i+j)/2;
		if (nav->peph[k].time.timediff(data->sigtime)<0.0) i=k+1; else j=k;
	}
	index=i<=0 ? 0 : i-1;

	/* polynomial interpolation for orbit */
	i=index+1-(NMAX)/2;
	if (i<0) i=0; else if (i+NMAX>nav->ne) i=nav->ne-NMAX;

	for (j=0; j<NMAX; j++) {
		t[j]=nav->peph[i+j].time.timediff(data->sigtime);
		TNMAX[j]=nav->peph[i+j].time;
		if (norm(nav->peph[i+j].pos[data->sat-1],3)<=0.0) {
			return 0;
		}
	}
	for (j=0; j<NMAX; j++) {
#if 0
		p[0][j]=pos[0];
		p[1][j]=pos[1];
#else
		/* correciton for earh rotation ver.2.4.0 */
		sinl=sin(OMGE*t[j]);
		cosl=cos(OMGE*t[j]);
		p[0][j]=
			cosl*nav->peph[i+j].pos[data->sat-1][0]-sinl*nav->peph[i+j].pos[data->sat-1][1];
		p[1][j]=
			sinl*nav->peph[i+j].pos[data->sat-1][0]+cosl*nav->peph[i+j].pos[data->sat-1][1];
#endif
		p[2][j]=nav->peph[i+j].pos[data->sat-1][2];
	}
	for (i=0; i<3; i++) {
		data->posvel[i]=interpolLaR(t,TNMAX,p[i],NMAX);
	}

	/* satellite position variance */
	for (i=0; i<3; i++) s[i]=nav->peph[index].std[data->sat-1][i];
	std=norm(s,3);
	if (t[0]>0.0) std+=EXTERR_EPH*SQR(t[0])/2.0; /* extrapolation error for orbit */
	else if (t[NMAX-1]<0.0) std+=EXTERR_EPH*SQR(t[NMAX-1])/2.0;
	data->svar=SQR(std);

	/* linear interpolation for clock */
	t[0]=data->sigtime.timediff(nav->peph[index].time);
	t[1]=data->sigtime.timediff(nav->peph[index+1].time);
	c[0]=nav->peph[index].pos[data->sat-1][3];
	c[1]=nav->peph[index+1].pos[data->sat-1][3];

	if (t[0]<=0.0) {
		if ((data->dts[0]=c[0])!=0.0) {
			std=nav->peph[index].std[data->sat-1][3]*CLIGHT-EXTERR_CLK*t[0];
		}
	}
	else if (t[1]>=0.0) {
		if ((data->dts[0]=c[1])!=0.0) {
			std=nav->peph[index+1].std[data->sat-1][3]*CLIGHT+EXTERR_CLK*t[1];
		}
	}
	else if (c[0]!=0.0&&c[1]!=0.0) {
		data->dts[0]=(c[1]*t[0]-c[0]*t[1])/(t[0]-t[1]);
		i=t[0]<-t[1] ? 0 : 1;
		std=nav->peph[index+i].std[data->sat-1][3]+EXTERR_CLK*fabs(t[i]);
	}
	else {
		data->dts[0]=0.0;
	}
	scvar=SQR(std);
	return 1;
}
/* precise satellite clocks function ---------------------------------------------- */
int preciseph_t::satclk(obsd_t *data,const nav_t *nav){
	double t[2],c[2],std;
	int i,j,k,index;

	if (nav->nc<2||
		data->sigtime.timediff(nav->pclk[0].time)<-MAXDTE||
		data->sigtime.timediff(nav->pclk[nav->nc-1].time)>MAXDTE) {
		data->errmsg="no precise clock!";
		return 1;
	}
	/* binary search */
	for (i=0,j=nav->nc-1; i<j;) {
		k=(i+j)/2;
		if (nav->pclk[k].time.timediff(data->sigtime)<0.0) i=k+1; else j=k;
	}
	index=i<=0 ? 0 : i-1;

	/* linear interpolation for clock */
	t[0]=data->sigtime.timediff(nav->pclk[index].time);
	t[1]=data->sigtime.timediff(nav->pclk[index+1].time);
	c[0]=nav->pclk[index].clk[data->sat-1];
	c[1]=nav->pclk[index+1].clk[data->sat-1];

	if (t[0]<=0.0) {
		if ((data->dts[0]=c[0])==0.0) return 0;
		std=nav->pclk[index].std[data->sat-1]*CLIGHT-EXTERR_CLK*t[0];
	}
	else if (t[1]>=0.0) {
		if ((data->dts[0]=c[1])==0.0) return 0;
		std=nav->pclk[index+1].std[data->sat-1]*CLIGHT+EXTERR_CLK*t[1];
	}
	else if (c[0]!=0.0&&c[1]!=0.0) {
		data->dts[0]=(c[1]*t[0]-c[0]*t[1])/(t[0]-t[1]);
		i=t[0]<-t[1] ? 0 : 1;
		std=nav->pclk[index+i].std[data->sat-1]*CLIGHT+EXTERR_CLK*fabs(t[i]);
	}
	else {
		data->errmsg="precise clock outage!";
		return 0;
	}
	scvar=SQR(std);
	return 1;
}
/* precise satellite position function -------------------------------------------- */
int preciseph_t::satpos(obsd_t *data,int iode,const nav_t *nav){
	double tt=1E-3;
	int i;
	/* signal time add 1E-3s */
	obsd_t data_tt;
	data_tt.sigtime=data->sigtime;
	data_tt.sigtime.timeadd(tt);
	data_tt.sat=data->sat;
	

	if (data->sat<=0||MAXSAT<data->sat) return 0;

	/* satellite position and clock bias */
	if (!precisepos(data,nav)||
		!satclk(data,nav)) return 0;

	if (!precisepos(&data_tt,nav)||
		!satclk(&data_tt,nav)) return 0;

	/* satellite velocity */
	for (i=0; i<3; i++) {
		data->posvel[i+3]=(data_tt.posvel[i]-data->posvel[i])/tt;
	}
	/* satellite antenna offset correction */
	satantfunc->satantoff(data,nav);

	/* relativistic effect correction and satellite clock drift */
	if (data->dts[0]!=0.0) {
		data->dts[0]=
			data->dts[0]-2.0*dot(data->posvel,data->posvel+3,3)/CLIGHT/CLIGHT;
		data->dts[1]=
			(data_tt.dts[0]-data->dts[0])/tt;
	}
	else { /* no precise clock */
		data->dts[0]=data->dts[1]=0.0;
	}
	data->svar=data->svar+scvar;

	return 1;
}

/* qzss lex ephemeris ----------------------------------------------------------------------------- */
/* Consturctors ------------------------------------------------------------------- */
qzsslex_t::qzsslex_t(){
}
qzsslex_t::~qzsslex_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* base functions ----------------------------------------------------------------- */
/* ura value ---------------------------------------------------------------------- */
double qzsslex_t::vareph(int ura){
	const double uraval[]={
		0.08,0.11,0.15,0.21,0.30,0.43,0.60,0.85,1.20,1.70,2.40,3.40,4.85,6.85,
		9.65,9.65
	};
	if (ura<0||15<ura) ura=15;
	return uraval[ura];
}
/* qzss lex satellite clocks function --------------------------------------------- */
int qzsslex_t::satclk(obsd_t *data,const nav_t *nav){
	/* nav->lexeph[data->sat-1] */
	int sat = data->sat;
	double dt;

	if (nav->lexeph[sat-1].sat!=sat || nav->lexeph[sat-1].toe.time==0){
		data->errmsg="no lex ephemeris!";
		return 0;
	}
	if ((dt=data->sigtime.timediff(nav->lexeph[sat-1].toe))>LEXEPHMAXAGE){
		data->errmsg="lex ephemeris age error!";
		return 0;
	}
#if 0
	if (nav->lexeph[sat-1].health&0x18) {
		data->errmsg="lex ephemeris unhealthy!";
		return 0;
	}
#endif
	data->dts[0]=nav->lexeph[sat-1].af0+nav->lexeph[sat-1].af1*dt;
	data->dts[1]=nav->lexeph[sat-1].af1;

	return 1;
}
/* qzss lex satellite position function ------------------------------------------- */
int qzsslex_t::satpos(obsd_t *data,int iode,const nav_t *nav){
	/* nav->lexeph[data->sat-1] */
	int sat = data->sat;
	double dt,t2,t3;

	if (nav->lexeph[sat-1].sat!=sat || nav->lexeph[sat-1].toe.time==0){
		data->errmsg="no lex ephemeris!";
		return 0;
	}
	if ((dt=data->sigtime.timediff(nav->lexeph[sat-1].toe))>LEXEPHMAXAGE){
		data->errmsg="lex ephemeris age error!";
		return 0;
	}
#if 0
	if (nav->lexeph[sat-1].health&0x18) {
		data->errmsg="lex ephemeris unhealthy!";
		return 0;
	}
#endif
	/* satellite position */
	t2=dt*dt/2.0; t3=t2*dt/3.0;
	for (int i=0; i<3; i++) {
		data->posvel[i]=nav->lexeph[sat-1].pos[i]+nav->lexeph[sat-1].vel[i]*dt +
			nav->lexeph[sat-1].acc[i]*t2+nav->lexeph[sat-1].jerk[i]*t3;
		data->posvel[i+3]=nav->lexeph[sat-1].vel[i] +
			nav->lexeph[sat-1].acc[i]*dt+nav->lexeph[sat-1].jerk[i]*t2;
	}

	/* satellite clock */
	data->dts[0]=nav->lexeph[sat-1].af0+nav->lexeph[sat-1].af1*dt;
	data->dts[1]=nav->lexeph[sat-1].af1;

	/* relativistic effect correction */
	data->dts[0]-=2.0*dot(data->posvel,data->posvel+3,3)/CLIGHT/CLIGHT;

	data->svar=vareph(nav->lexeph[sat-1].ura);

	return 1;
}