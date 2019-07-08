#include "GNSS/IonModel/ionosphere.h"

#include "BaseFunction/basefunction.h"

/* ionosphere reference ------------------------------------------------------------------------------
*     [1] IS-QZSS v.1.1, Quasi-Zenith Satellite System Navigation Service
*         Interface Specification for QZSS, Japan Aerospace Exploration Agency,
*         July 31, 2009
*     [2] RTCA/DO-229C, Minimum operational performanc standards for global
*         positioning system/wide area augmentation system airborne equipment,
*         RTCA inc, November 28, 2001
* --------------------------------------------------------------------------------------------------*/
/* constant --------------------------------------------------------------------------------------- */
#define SQR(x)      ((x)*(x))
#define ERR_ION     10.0				/* ionospheric delay std (m) */
#define ERR_BRDCI   0.5					/* broadcast iono model error factor */
#define VAR_NOTEC   SQR(30.0)			/* variance of no tec */
#define MIN_EL      0.0					/* min elevation angle (rad) */
#define MIN_HGT     -1000.0				/* min user height (m) */
#define ION_HGT		450					/* defualt ionosphere height (km) */

/* ionosphere model class ------------------------------------------------------------------------- */
/* parent ionosphere functions -------------------------------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
ioncorr_t::ioncorr_t(){
}
ioncorr_t::~ioncorr_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* base functions ----------------------------------------------------------------- */
/* ionospheric pierce point position -------------------------------------------------
* see ref[2]
* posp   : ionospheric pierce point position {lat, lon} (rad)
* re     : earth radius                                 (km)
* hion   : ionosphere single-layer height               (km)
* return : mapping function
* --------------------------------------------------------------------------------- */
double ioncorr_t::ionppp(const double pos[3],const obsd_t *obs,const double re,
	const double hion,double *posp){
	double cosaz,rp,ap,sinap,tanap;

	/* single layer mapping function(M-SLM) */
	rp=re/(re+hion)*sin(0.9782*(PI/2.0-obs->azel[1]));
	ap=PI/2.0-obs->azel[1]-asin(rp);
	sinap=sin(ap);
	tanap=tan(ap);
	cosaz=cos(obs->azel[0]);
	posp[0]=asin(sin(pos[0])*cos(ap)+cos(pos[0])*sinap*cosaz);

	if ((pos[0]> 70.0*D2R&& tanap*cosaz>tan(PI/2.0-pos[0]))||
		(pos[0]<-70.0*D2R&&-tanap*cosaz>tan(PI/2.0+pos[0]))) {
		posp[1]=pos[1]+PI-asin(sinap*sin(obs->azel[0])/cos(posp[0]));
	}
	else {
		posp[1]=pos[1]+asin(sinap*sin(obs->azel[0])/cos(posp[0]));
	}
	return 1.0/sqrt(1.0-rp*rp);
}
/* base ionosphere mapping function --------------------------------------- */
double ioncorr_t::ionmapf(const double *pos,const double *azel) {
	if (pos[2]>=HION) return 1.0;
	return 1.0/cos(asin((RE_WGS84+pos[2])/(RE_WGS84+HION)*sin(0.9782*(PI/2.0-azel[1]))));
}
/* virtual compute ionospere correction (vertical delay of GPS L1)--------------------
* pos[3]   : lat lon h {rad,m}
* --------------------------------------------------------------------------------- */
int ioncorr_t::correction(obsd_t *obs, const nav_t *nav, const double pos[3]){
	obs->ionmap=0.0;
	obs->dion=0.0;
	obs->ionvar=SQR(ERR_ION);
	return 1;
}

/* subclass ionosphere functions ------------------------------------------------------------------ */
/* ionosphere free combination -------------------------------------------------------------------- */
LCion_t::LCion_t(){
}
LCion_t::~LCion_t(){
}
/* return 0 delya ----------------------------------------------------------------- */
int LCion_t::correction(obsd_t *obs,const nav_t *nav,const double pos[3]){
	obs->ionmap=0.0;
	obs->dion=0.0;
	obs->ionvar=SQR(0.001);
	return 1;
}

/* broadcast ionosphere correction (vertical delay of GPS L1)-------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
broadion_t::broadion_t(){
}
broadion_t::~broadion_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* base functions ----------------------------------------------------------------- */
/* ionosphere Klobuchar model correction (vertical delay of GPS L1) ------------------
* paramter : a0,a1,a2,a3,b0,b1,b2,b3
* pos[3]   : lat lon h {rad,m}
* --------------------------------------------------------------------------------- */
int broadion_t::klobion(obsd_t *obs,const double ionpara[8],const double pos[3]){
	const double ion_default[]={ /* 2004/1/1 */
		0.1118E-07,-0.7451E-08,-0.5961E-07, 0.1192E-06,
		0.1167E+06,-0.2294E+06,-0.1311E+06, 0.1049E+07
	};
	double tt,psi,phi,lam,amp,per,x;
	int week;
	gtime_t obstime=obs->time;

	if (pos[2]<-1E3||obs->azel[1]<=0) return 0;
	if (norm(ionpara,8)<=0.0) return 0;

	/* earth centered angle (semi-circle) */
	psi=0.0137/(obs->azel[1]/PI+0.11)-0.022;

	/* subionospheric latitude/longitude (semi-circle) */
	phi=pos[0]/PI+psi*cos(obs->azel[0]);
	if (phi> 0.416) phi= 0.416;
	else if (phi<-0.416) phi=-0.416;
	lam=pos[1]/PI+psi*sin(obs->azel[0])/cos(phi*PI);

	/* geomagnetic latitude (semi-circle) */
	phi+=0.064*cos((lam-1.617)*PI);

	/* local time (s) */
	tt=43200.0*lam+obstime.time2gpst(&week);
	tt-=floor(tt/86400.0)*86400.0; /* 0<=tt<86400 */

	/* slant factor */
	obs->ionmap=1.0+16.0*pow(0.53-obs->azel[1]/PI,3.0);

	/* ionospheric delay */
	amp=ionpara[0]+phi*(ionpara[1]+phi*(ionpara[2]+phi*ionpara[3]));
	per=ionpara[4]+phi*(ionpara[5]+phi*(ionpara[6]+phi*ionpara[7]));
	amp=amp<    0.0 ? 0.0 : amp;
	per=per<72000.0 ? 72000.0 : per;
	x=2.0*PI*(tt-50400.0)/per;

	obs->dion = CLIGHT*(fabs(x)<1.57 ? 5E-9+amp*(1.0+x*x*(-0.5+x*x/24.0)) : 5E-9);
	return 1;
}
/* compute broadcast ionospere correction (vertical delay of GPS L1)------------------
* --------------------------------------------------------------------------------- */
int broadion_t::correction(obsd_t *obs, const nav_t *nav, const double pos[3]){
	if (!klobion(obs,nav->ion_gps,pos)) return 0;
	obs->ionvar=SQR(obs->dion*ERR_BRDCI); /* ERR_BRDCI can be redefined */
	return 1;
}

/* qzss broadcast ionosphere correction (vertical delay of GPS L1)--------------------------------- */
/* Constructors ------------------------------------------------------------------- */
qzssion_t::qzssion_t(){
}
qzssion_t::~qzssion_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* compute qzss broadcast ionospere correction (vertical delay of GPS L1)-------------
* pos[3]   : lat lon h {rad,m}
* --------------------------------------------------------------------------------- */
int qzssion_t::correction(obsd_t *obs,const nav_t *nav,const double pos[3]){
	if (!klobion(obs,nav->ion_qzs,pos)) return 0;
	obs->ionvar=SQR(obs->dion*ERR_BRDCI); /* ERR_BRDCI can be redefined */
	return 1;
}

/* sbas ionosphere correction (vertical delay of GPS L1)------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
sbasion_t::sbasion_t(){
}
sbasion_t::~sbasion_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* base functions ----------------------------------------------------------------- */
/* search igps -------------------------------------------------------------------- */
void sbasion_t::searchigp(gtime_t time,const double pos[2],const sbsion_t *ion,
	const sbsigp_t **igp,double &x,double &y){
	int i,latp[2],lonp[4];
	double lat=pos[0]*R2D,lon=pos[1]*R2D;
	const sbsigp_t *p;

	if (lon>=180.0) lon-=360.0;
	if (-55.0<=lat&&lat<55.0) {
		latp[0]=(int)floor(lat/5.0)*5;
		latp[1]=latp[0]+5;
		lonp[0]=lonp[1]=(int)floor(lon/5.0)*5;
		lonp[2]=lonp[3]=lonp[0]+5;
		x=(lon-lonp[0])/5.0;
		y=(lat-latp[0])/5.0;
	}
	else {
		latp[0]=(int)floor((lat-5.0)/10.0)*10+5;
		latp[1]=latp[0]+10;
		lonp[0]=lonp[1]=(int)floor(lon/10.0)*10;
		lonp[2]=lonp[3]=lonp[0]+10;
		x=(lon-lonp[0])/10.0;
		y=(lat-latp[0])/10.0;
		if (75.0<=lat&&lat<85.0) {
			lonp[1]=(int)floor(lon/90.0)*90;
			lonp[3]=lonp[1]+90;
		}
		else if (-85.0<=lat&&lat<-75.0) {
			lonp[0]=(int)floor((lon-50.0)/90.0)*90+40;
			lonp[2]=lonp[0]+90;
		}
		else if (lat>=85.0) {
			for (i=0; i<4; i++) lonp[i]=(int)floor(lon/90.0)*90;
		}
		else if (lat<-85.0) {
			for (i=0; i<4; i++) lonp[i]=(int)floor((lon-50.0)/90.0)*90+40;
		}
	}
	for (i=0; i<4; i++) if (lonp[i]==180) lonp[i]=-180;
	for (i=0; i<=MAXBAND; i++) {
		for (p=ion[i].igp; p<ion[i].igp+ion[i].nigp; p++) {
			if (p->t0.time==0) continue;
			if (p->lat==latp[0]&&p->lon==lonp[0]&&p->give>0) igp[0]=p;
			else if (p->lat==latp[1]&&p->lon==lonp[1]&&p->give>0) igp[1]=p;
			else if (p->lat==latp[0]&&p->lon==lonp[2]&&p->give>0) igp[2]=p;
			else if (p->lat==latp[1]&&p->lon==lonp[3]&&p->give>0) igp[3]=p;
			if (igp[0]&&igp[1]&&igp[2]&&igp[3]) return;
		}
	}
}
/* variance of ionosphere correction (give=GIVEI+1) ----------------------- */
double sbasion_t::varicorr(int udre){
	const double var[14]={
		0.052,0.0924,0.1444,0.283,0.4678,0.8315,1.2992,1.8709,2.5465,3.326,
		5.1968,20.7870,230.9661,2078.695
	};
	return 0<udre&&udre<=14 ? var[udre-1] : 0.0;
}
/* compute sbas ionospere correction (vertical delay of GPS L1)-----------------------
* pos[3]   : lat lon h {rad,m}
* --------------------------------------------------------------------------------- */
int sbasion_t::correction(obsd_t *obs,const nav_t *nav,const double pos[3]){
	const double re=6378.1363,hion=350.0;
	int i,err=0;
	double posp[2],x=0.0,y=0.0,t,w[4]={ 0 };
	const sbsigp_t *igp[4]={ 0 }; /* {ws,wn,es,en} */

	obs->dion=obs->ionvar=0.0;
	if (pos[2]<-100.0||obs->azel[1]<=0) return 1;

	/* ipp (ionospheric pierce point) position */
	obs->ionmap=ionppp(pos,obs,re,hion,posp);

	/* search igps around ipp */
	searchigp(obs->time,posp,nav->sbsion,igp,x,y);

	/* weight of igps */
	if (igp[0]&&igp[1]&&igp[2]&&igp[3]) {
		w[0]=(1.0-x)*(1.0-y); w[1]=(1.0-x)*y; w[2]=x*(1.0-y); w[3]=x*y;
	}
	else if (igp[0]&&igp[1]&&igp[2]) {
		w[1]=y; w[2]=x;
		if ((w[0]=1.0-w[1]-w[2])<0.0) err=1;
	}
	else if (igp[0]&&igp[2]&&igp[3]) {
		w[0]=1.0-x; w[3]=y;
		if ((w[2]=1.0-w[0]-w[3])<0.0) err=1;
	}
	else if (igp[0]&&igp[1]&&igp[3]) {
		w[0]=1.0-y; w[3]=x;
		if ((w[1]=1.0-w[0]-w[3])<0.0) err=1;
	}
	else if (igp[1]&&igp[2]&&igp[3]) {
		w[1]=1.0-x; w[2]=1.0-y;
		if ((w[3]=1.0-w[1]-w[2])<0.0) err=1;
	}
	else err=1;

	if (err) {
		obs->errmsg="no sbas iono correction!";
		return 0;
	}
	for (i=0; i<4; i++) {
		if (!igp[i]) continue;
		t=obs->time.timediff(igp[i]->t0);
		obs->dion+=w[i]*igp[i]->delay;
		obs->ionvar+=w[i]*varicorr(igp[i]->give)*9E-8*fabs(t);
	}

	return 1;
}

/* ionex ionosphere correction (vertical delay of GPS L1)------------------------------------------ */
/* Constructors ------------------------------------------------------------------- */
ionexion_t::ionexion_t(){
}
ionexion_t::~ionexion_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* base functions ----------------------------------------------------------------- */
/* data index (i:lat,j:lon,k:hgt) ----------------------------------------- */
int ionexion_t::dataindex(int i,int j,int k,const int *ndata){
	if (i<0||ndata[0]<=i||j<0||ndata[1]<=j||k<0||ndata[2]<=k) return -1;
	return i+ndata[0]*(j+ndata[1]*k);
}
/* interpolate tec grid data ---------------------------------------------- */
int ionexion_t::interptec(const tec_t *tec,int k,const double *posp,double &value,
	double &rms){
	double dlat,dlon,a,b,d[4]={ 0 },r[4]={ 0 };
	int i,j,n,index;

	value=rms=0.0;

	if (tec->lats[2]==0.0||tec->lons[2]==0.0) return 0;

	dlat=posp[0]*R2D-tec->lats[0];
	dlon=posp[1]*R2D-tec->lons[0];
	if (tec->lons[2]>0.0) dlon-=floor(dlon/360)*360.0; /*  0<=dlon<360 */
	else                  dlon+=floor(-dlon/360)*360.0; /* -360<dlon<=0 */

	a=dlat/tec->lats[2];
	b=dlon/tec->lons[2];
	i=(int)floor(a); a-=i;
	j=(int)floor(b); b-=j;

	/* get gridded tec data */
	for (n=0; n<4; n++) {
		if ((index=dataindex(i+(n%2),j+(n<2 ? 0 : 1),k,tec->ndata))<0) continue;
		d[n]=tec->data[index];
		r[n]=tec->rms[index];
	}
	if (d[0]>0.0&&d[1]>0.0&&d[2]>0.0&&d[3]>0.0) {

		/* bilinear interpolation (inside of grid) */
		value=(1.0-a)*(1.0-b)*d[0]+a*(1.0-b)*d[1]+(1.0-a)*b*d[2]+a*b*d[3];
		rms  =(1.0-a)*(1.0-b)*r[0]+a*(1.0-b)*r[1]+(1.0-a)*b*r[2]+a*b*r[3];
	}
	/* nearest-neighbour extrapolation (outside of grid) */
	else if (a<=0.5&&b<=0.5&&d[0]>0.0) { value=d[0]; rms=r[0]; }
	else if (a> 0.5&&b<=0.5&&d[1]>0.0) { value=d[1]; rms=r[1]; }
	else if (a<=0.5&&b> 0.5&&d[2]>0.0) { value=d[2]; rms=r[2]; }
	else if (a> 0.5&&b> 0.5&&d[3]>0.0) { value=d[3]; rms=r[3]; }
	else {
		i=0;
		for (n=0; n<4; n++) if (d[n]>0.0) { i++; value+=d[n]; rms+=r[n]; }
		if (i==0) return 0;
		value/=i; rms/=i;
	}
	return 1;
}
/* ionosphere delay by tec grid data -------------------------------------- */
int ionexion_t::iondelay(const tec_t *tec,const double *pos,obsd_t *obs,
	double *delay,double *var){
	const double fact=40.30E16/FREQ1/FREQ1; /* tecu->L1 iono (m) */
	double mf,posp[2]={ 0 },vtec,rms,hion;
	int i;

	*delay=*var=0.0;

	for (i=0; i<tec->ndata[2]; i++) { /* for a layer */

		hion=tec->hgts[0]+tec->hgts[2]*i;

		/* ionospheric pierce point position */
		obs->ionmap=ionppp(pos,obs,tec->rb,hion,posp);

		/* earth rotation correction (sun-fixed coordinate) */
		/*posp[1]+=2.0*PI*obs->time.timediff(tec->time)/86400.0;*/

		/* interpolate tec grid data */
		if (!interptec(tec,i,posp,vtec,rms)) return 0;

		*delay+=fact*vtec;
		*var+=fact*fact*rms*rms;
	}

	return 1;
}
/* compute ionex ionospere correction (vertical delay of GPS L1)----------------------
* pos[3]   : lat lon h {rad,m}
* --------------------------------------------------------------------------------- */
int ionexion_t::correction(obsd_t *obs,const nav_t *nav,const double pos[3]){
	double dels[2],vars[2],a,tt;
	int i,stat[2];

	if (obs->azel[1]<MIN_EL||pos[2]<MIN_HGT) {
		obs->dion=0.0;
		obs->ionvar=VAR_NOTEC;
		return 1;
	}
	for (i=0; i<nav->nt; i++) {
		if (nav->tec[i].time.timediff(obs->time)>0.0) break;
	}
	if (i==0||i>=nav->nt) {
		obs->errmsg="tec grid out of period!";
		return 0;
	}
	if ((tt=nav->tec[i].time.timediff(nav->tec[i-1].time))==0.0) {
		obs->errmsg="tec grid time interval error!";
		return 0;
	}
	/* ionospheric delay by tec grid data */
	stat[0]=iondelay(&nav->tec[i-1],pos,obs,dels,vars);
	stat[1]=iondelay(&nav->tec[i],pos,obs,dels+1,vars+1);

	if (!stat[0]&&!stat[1]) {
		obs->errmsg="tec grid out of area pos!";
		return 0;
	}
	if (stat[0]&&stat[1]) { /* linear interpolation by time */
		a=obs->time.timediff(nav->tec[i-1].time)/tt;
		obs->dion  =dels[0]*(1.0-a)+dels[1]*a;
		obs->ionvar=vars[0]*(1.0-a)+vars[1]*a;
	}
	else if (stat[0]) { /* nearest-neighbour extrapolation by time */
		obs->dion  =dels[0];
		obs->ionvar=vars[0];
	}
	else {
		obs->dion  =dels[1];
		obs->ionvar=vars[1];
	}

	return 1;
}

/* lex ionosphere correction (vertical delay of GPS L1)-------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
lexioncor_t::lexioncor_t(){
}
lexioncor_t::~lexioncor_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* base functions ----------------------------------------------------------------- */
/* compute lex ionospere correction (vertical delay of GPS L1)------------------------
* pos[3]   : lat lon h {rad,m}
* --------------------------------------------------------------------------------- */
int lexioncor_t::correction(obsd_t *obs,const nav_t *nav,const double pos[3]){
	const double re=6378.137,hion=350.0;
#if 0
	const double dl1=(141.0-129.0)/(45.5-34.7);
	const double dl2=(129.0-126.7)/(34.7-26.0);
#endif
	double tt,sinlat,coslat,sinaz,cosaz,cosel,rp,ap,sinap,cosap,latpp,lonpp;
	double dlat,dlon,Enm,F;
	int n,m;

	obs->dion=obs->ionvar=0.0;

	if (pos[2]<-100.0||obs->azel[1]<=0.0) return 1;

	tt=obs->time.timediff(nav->lexion.t0);

	/* check time span */
	if (fabs(tt)>nav->lexion.tspan) {
		obs->errmsg="lex iono age error!";
		return 0;
	}
	/* check user position range (ref [1] 4.1.5) */
#if 0
	if (pos[0]> 45.5*D2R||pos[0]< 26.0*D2R||
		pos[1]>146.0*D2R||
		pos[1]<129.0*D2R+dl1*(pos[0]-34.7*D2R)||
		pos[1]<126.7*D2R+dl2*(pos[0]-26.0*D2R)) {
		return 0;
	}
#endif
	/* ionospheric pierce point position */
	sinlat=sin(pos[0]);
	coslat=cos(pos[0]);
	sinaz=sin(obs->azel[0]);
	cosaz=cos(obs->azel[0]);
	cosel=cos(obs->azel[1]);
	rp=re/(re+hion)*cosel;
	ap=PI/2.0-obs->azel[1]-asin(rp);
	sinap=sin(ap);
	cosap=cos(ap);
	latpp=asin(sinlat*cosap+coslat*sinap*cosaz);
	lonpp=pos[1]+atan(sinap*sinaz/(cosap*coslat-sinap*cosaz*sinlat));

	/* inclination factor */
	obs->ionmap=1.0/sqrt(1.0-rp*rp);

	/* delta latitude/longitude (rad) */
	dlat=latpp-nav->lexion.pos0[0];
	dlon=lonpp-nav->lexion.pos0[1];

	/* slant ionosphere delay (L1) */
	for (n=0; n<=2; n++) for (m=0; m<=1; m++) {
		Enm=nav->lexion.coef[n][m];
		obs->dion+=Enm*pow(dlat,n)*pow(dlon,m);
	}

	return 1;
}

/* constrained ionosphere model correction (vertical delay of GPS L1)------------------------------ */
/* Constructors ------------------------------------------------------------------- */
constion_t::constion_t() {
}
constion_t::~constion_t() {
}
/* compute constrained ionosphere model correction (vertical delay of GPS L1)------ */
int constion_t::correction(obsd_t *obs,const nav_t *nav,const double pos[3]) {
	/* return constrained ionex model corrections */
	if (nav->nt>2) return ionexion_t::correction(obs,nav,pos);
	
	/* return constrained broadcast model corrections */
	if (norm(nav->ion_gps,8)>=0) return broadion_t::correction(obs,nav,pos);

	/*double posp[2]={ 0.0 };
	obs->ionmap=ionppp(pos,obs,RE_WGS84/1000.0,450.0,posp);*/

	return 0;
}