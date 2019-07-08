#include "GNSS/AntModel/antenna.h"

#include "BaseFunction/basefunction.h"

/* Constant --------------------------------------------------------------------------------------- */
#define SQR(x)      ((x)*(x))

/* Functions -------------------------------------------------------------------------------------- */
/* interpolate antenna phase center variation --------------------------------*/
static double interpvar(double ang,const double *var)
{
	double a=ang/5.0; /* ang=0-90 */
	int i=(int)a;
	if (i<0) return var[0]; else if (i>=18) return var[18];
	return var[i]*(1.0-a+i)+var[i+1]*(a-i);
}

/* satellite antenna phase center offest ---------------------------------------------------------- */
/* Constuctors -------------------------------------------------------------------- */
satantenna_t::satantenna_t(){
}
satantenna_t::~satantenna_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* satellite antenna phase center offest for one obsd_t --------------------------- */
void satantenna_t::satantoff(obsd_t *data, const nav_t *nav){
	const double *lam=nav->lam[data->sat-1];
	const pcv_t *pcv=nav->pcvs+data->sat-1;
	double ex[3],ey[3],ez[3],es[3],r[3],rsun[3],gmst,erpv[5]={ 0 };
	double gamma,C1,C2,dant1,dant2;
	int i,j=0,k=1;

	/* gps time to utc time */
	gtime_t utcTime = data->sigtime;
	utcTime.gpst2utc();

	/* sun position in ecef */
	sunmoonpos(utcTime,erpv,rsun,NULL,&gmst);

	/* unit vectors of satellite fixed coordinates */
	for (i=0; i<3; i++) r[i]=-data->posvel[i];
	if (!normv3(r,ez)) return;
	for (i=0; i<3; i++) r[i]=rsun[i]-data->posvel[i];
	if (!normv3(r,es)) return;
	cross3(ez,es,r);
	if (!normv3(r,ey)) return;
	cross3(ey,ez,ex);

	if (NFREQ>=3&&(satsys(data->sat,NULL)&(SYS_GAL|SYS_SBS))) k=2;

	if (NFREQ<2||lam[j]==0.0||lam[k]==0.0) return;

	gamma=SQR(lam[k])/SQR(lam[j]);
	C1=gamma/(gamma-1.0);
	C2=-1.0 /(gamma-1.0);

	/* iono-free LC */
	for (i=0; i<3; i++) {
		dant1=pcv->off[j][0]*ex[i]+pcv->off[j][1]*ey[i]+pcv->off[j][2]*ez[i];
		dant2=pcv->off[k][0]*ex[i]+pcv->off[k][1]*ey[i]+pcv->off[k][2]*ez[i];
		data->posvel[i]+=C1*dant1+C2*dant2;
	}
}

/* receiver antenna phase center offest ----------------------------------------------------------- */
/* Constuctors -------------------------------------------------------------------- */
recantenna_t::recantenna_t(){
}
recantenna_t::~recantenna_t(){
}
/* Implementation functions ------------------------------------------------------- */
/* receiver antenna phase center offest for one obsd_t ---------------------------- */
void recantenna_t::recantoff(const prcopt_t *opt,int rovbas,obsd_t *data){
	double e[3],off[3],cosel=cos(data->azel[1]);

	e[0]=sin(data->azel[0])*cosel;
	e[1]=cos(data->azel[0])*cosel;
	e[2]=sin(data->azel[1]);

	for (int i=0; i<NFREQ; i++) {
		for (int j=0; j<3; j++) off[j]=(opt->posopt[0]*opt->pcvr[rovbas].off[i][j])+opt->antdel[rovbas][j];

		data->dant[i]=-dot(off,e,3)+
			(opt->posopt[0] ? interpvar(90.0-data->azel[1]*R2D,opt->pcvr[rovbas].var[i]) : 0.0);
	}
}