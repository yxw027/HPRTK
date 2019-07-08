#include "GNSS/TroModel/troposphere.h"

#include "BaseFunction/basefunction.h"

/* troposphere reference -----------------------------------------------------------------------------
*    [1]  A.E.Niell, Global mapping functions for the atmosphere delay at radio
*         wavelengths, Jounal of geophysical research, 1996
* --------------------------------------------------------------------------------------------------*/
/* constant --------------------------------------------------------------------------------------- */
#define SQR(x)      ((x)*(x))
#define ERR_TROP    3.0         /* tropspheric delay std (m) */
#define ERR_SAAS    0.3         /* saastamoinen model error std (m) */
#define REL_HUMI    0.7         /* relative humidity for saastamoinen model */

/* troposphere model class ------------------------------------------------------------------------ */
/* Constructors ------------------------------------------------------------------- */
tromod_t::tromod_t(){
	Ptro[0]=Ptro[1]=Ptro[2]=0.0;
	Atro[0]=Atro[1]=Atro[2]=0.0;
}
tromod_t::~tromod_t(){

}

/* parent troposphere functions ------------------------------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
trocorr_t::trocorr_t(){
	model=tromod_t();
}
trocorr_t::~trocorr_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* base functions ----------------------------------------------------------------- */
/* triangle functions ------------------------------------------------------------- */
double trocorr_t::mapf(double el,double a,double b,double c){
	double sinel=sin(el);
	return (1.0+a/(1.0+b/(1.0+c)))/(sinel+(a/(sinel+b/(sinel+c))));
}
/* troposphere interpc function --------------------------------------------------- */
double trocorr_t::interpc(const double coef[],const double lat){
	int i=(int)(lat/15.0);
	if (i<1) return coef[0]; else if (i>4) return coef[4];
	return coef[i-1]*(1.0-lat/15.0+i)+coef[i]*(lat/15.0-i);
}
/* NMF troposphere mapping function --------------------------------------------------
* compute tropospheric mapping function by NMF
* ref [1]
* --------------------------------------------------------------------------------- */
double trocorr_t::nmftropmapf(const obsd_t *obs,const double pos[3],double *mapfw){
	/* verif the height of receiver */
	if (pos[2]<-1000.0||pos[2]>20000.0) {
		if (mapfw) *mapfw=0.0;
		return 0.0;
	}

	/* compute maping function */
	/* ref [1] table 3 */
	/* hydro-ave-a,b,c, hydro-amp-a,b,c, wet-a,b,c at latitude 15,30,45,60,75 */
	const double coef[][5]={
		{ 1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3 },
		{ 2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3 },
		{ 62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3 },

		{ 0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5 },
		{ 0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5 },
		{ 0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5 },

		{ 5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4 },
		{ 1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3 },
		{ 4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2 }
	};
	const double aht[]={ 2.53E-5, 5.49E-3, 1.14E-3 }; /* height correction */

	double y,cosy,ah[3],aw[3],dm,el=obs->azel[1],lat=pos[0]*R2D,hgt=pos[2];
	int i;

	gtime_t epoch=obs->time;

	if (el<=0.0) {
		if (mapfw) *mapfw=0.0;
		return 0.0;
	}
	/* year from doy 28, added half a year for southern latitudes */
	y=(epoch.time2doy()-28.0)/365.25+(lat<0.0 ? 0.5 : 0.0);

	cosy=cos(2.0*PI*y);
	lat=fabs(lat);

	for (i=0; i<3; i++) {
		ah[i]=interpc(coef[i],lat)-interpc(coef[i+3],lat)*cosy;
		aw[i]=interpc(coef[i+6],lat);
	}
	/* ellipsoidal height is used instead of height above sea level */
	dm=(1.0/sin(el)-mapf(el,aht[0],aht[1],aht[2]))*hgt/1E3;

	if (mapfw) *mapfw=mapf(el,aw[0],aw[1],aw[2]);

	return mapf(el,ah[0],ah[1],ah[2])+dm;
}
/* get meterological parameters --------------------------------------------------- */
void trocorr_t::getmet(double lat,double *met){
	static const double metprm[][10]={ /* lat=15,30,45,60,75 */
		{ 1013.25,299.65,26.31,6.30E-3,2.77,  0.00, 0.00,0.00,0.00E-3,0.00 },
		{ 1017.25,294.15,21.79,6.05E-3,3.15, -3.75, 7.00,8.85,0.25E-3,0.33 },
		{ 1015.75,283.15,11.66,5.58E-3,2.57, -2.25,11.00,7.24,0.32E-3,0.46 },
		{ 1011.75,272.15, 6.78,5.39E-3,1.81, -1.75,15.00,5.36,0.81E-3,0.74 },
		{ 1013.00,263.65, 4.11,4.53E-3,1.55, -0.50,14.50,3.39,0.62E-3,0.30 }
	};
	int i,j;
	double a;
	lat=fabs(lat);
	if (lat<=15.0) for (i=0; i<10; i++) met[i]=metprm[0][i];
	else if (lat>=75.0) for (i=0; i<10; i++) met[i]=metprm[4][i];
	else {
		j=(int)(lat/15.0); a=(lat-j*15.0)/15.0;
		for (i=0; i<10; i++) met[i]=(1.0-a)*metprm[j-1][i]+a*metprm[j][i];
	}
}
/* sbas troposphere model (sbas) -------------------------------------------------- */
int trocorr_t::sbascorr(obsd_t *obs,const double pos[3]){
	const double k1=77.604,k2=382000.0,rd=287.054,gm=9.784,g=9.80665;
	static double pos_[3]={ 0 },zh=0.0,zw=0.0;
	int i;
	double c,met[10],sinel=sin(obs->azel[1]),h=pos[2],m;
	gtime_t epoch=obs->time;

	if (pos[2]<-100.0||10000.0<pos[2]||obs->azel[1]<=0) {
		obs->trovar=0.0;
		return 1;
	}
	if (zh==0.0||fabs(pos[0]-pos_[0])>1E-7||fabs(pos[1]-pos_[1])>1E-7||
		fabs(pos[2]-pos_[2])>1.0) {
		getmet(pos[0]*R2D,met);
		c=cos(2.0*PI*(epoch.time2doy()-(pos[0]>=0.0 ? 28.0 : 211.0))/365.25);
		for (i=0; i<5; i++) met[i]-=met[i+5]*c;
		zh=1E-6*k1*rd*met[0]/gm;
		zw=1E-6*k2*rd/(gm*(met[4]+1.0)-met[3]*rd)*met[2]/met[1];
		zh*=pow(1.0-met[3]*h/met[1],g/(rd*met[3]));
		zw*=pow(1.0-met[3]*h/met[1],(met[4]+1.0)*g/(rd*met[3])-1.0);
		for (i=0; i<3; i++) pos_[i]=pos[i];
	}
	m=1.001/sqrt(0.002001+sinel*sinel);
	obs->trovar=0.12*0.12*m*m;
	obs->dtro=(zh+zw)*m;
	return 1;
}
/* standard troposphere model (saastamoinen) -------------------------------------- */
int trocorr_t::saascorr(obsd_t *obs,const double pos[3],const double azel[2],
	const double humi){
	const double temp0=18.0; /* temparature at sea level */
	double hgt,pres,temp,e,z,trph,trpw;
	double B,humidity;

	if (pos[2]<-100.0||1E4<pos[2]||azel[1]<=0) { 
		obs->dtro=0.0;
		obs->trovar=SQR(ERR_SAAS/(sin(azel[1])+0.1));
		return 1; 
	}

	/* standard atmosphere */
	hgt=pos[2]<0.0 ? 0.0 : pos[2];

	B=1.1561-1.5915E-4*hgt+9.5788E-9*hgt*hgt-2.9393E-13*hgt*hgt*hgt;
	pres=1013.25*pow(1.0-2.2557E-5*hgt,5.225);
	temp=temp0-6.5E-3*hgt+273.16;
	humidity=0.5*exp(-6.396E-4*hgt);
	if (humi==0.0) humidity=0.0;

	e=humidity*exp(-37.2465+0.213166*temp-0.000256908*temp*temp);

	/* saastamoninen model */
	z=PI/2.0-azel[1];
	trph=0.002277*(pres-B*tan(z)*tan(z))/cos(z);
	trpw=0.002277*(1255.0/temp+0.05)*e/cos(z);
	
	obs->dtro=trph+trpw;
	obs->trovar=SQR(ERR_SAAS/(sin(azel[1])+0.1));

	return 1;
}
/* virtual troposphere correction ----------------------------------------------------
* pos[3]   : lat lon h
* humidity : relative humidity
* --------------------------------------------------------------------------------- */
int trocorr_t::correction(obsd_t *obs,const nav_t *nav,const double pos[3],
	const double humi){
	obs->dtro=0.0;
	obs->trovar=SQR(ERR_TROP);
	return 1;
}

/* subclass troposphere functions ----------------------------------------------------------------- */
/* saastamoinen model troposphere correction ------------------------------------------------------ */
/* Constructors ------------------------------------------------------------------- */
saastro_t::saastro_t(){
}
saastro_t::~saastro_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* saastamoinen model troposphere correction -----------------------------------------
* pos[3]   : lat lon h
* humidity : relative humidity
* --------------------------------------------------------------------------------- */
int saastro_t::correction(obsd_t *obs,const nav_t *nav,const double pos[3],
	const double humi){
	return saascorr(obs,pos,obs->azel,REL_HUMI);
}

/* sbas model troposphere correction -------------------------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
sbastro_t::sbastro_t(){
}
sbastro_t::~sbastro_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* sbas model troposphere correction -------------------------------------------------
* pos[3]   : lat lon h
* humidity : not used!!!
* --------------------------------------------------------------------------------- */
int sbastro_t::correction(obsd_t *obs,const nav_t *nav,const double pos[3],
	const double humi){
	return sbascorr(obs,pos);
}

/* estimated model troposphere correction --------------------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
estitro_t::estitro_t(){
}
estitro_t::~estitro_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* estimated model troposphere correction --------------------------------------------
* pos[3]   : lat lon h
* humidity : not used!!!
* --------------------------------------------------------------------------------- */
int estitro_t::correction(obsd_t *obs,const nav_t *nav,const double pos[3],
	const double humi){
	const double zazel[]={ 0.0,PI/2.0 };
	double m_h,m_w,cotz,grad_n,grad_e;

	/* zenith hydrostatic delay */
	saascorr(obs,pos,zazel,0.0);

	/* mapping function */
	m_h=nmftropmapf(obs,pos,&m_w);

	if (obs->azel[1]>0.0) {

		/* m_w=m_0+m_0*cot(el)*(Gn*cos(az)+Ge*sin(az)): ref [6] */
		cotz=1.0/tan(obs->azel[1]);
		grad_n=cotz*cos(obs->azel[0]);
		grad_e=cotz*sin(obs->azel[0]);
		/*m_w+=grad_n*x[1]+grad_e*x[2];*/
		model.Atro[1]=grad_n;
		model.Atro[2]=grad_e;
	}
	model.Atro[0]=m_w;
	obs->trovar=SQR(0.001);
	obs->dtro = m_h*obs->dtro+m_w*(model.Ptro[0]-obs->dtro)+grad_n*model.Ptro[1]+grad_e*model.Ptro[2];
	return 1;
}