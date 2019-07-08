#include "GNSS/DataClass/data.h"
#include "BaseFunction/basefunction.h"
/* Constant */
/* variance of fast correction (udre=UDRE+1) ---------------------------------*/
static double varfcorr(int udre)
{
	const double var[14]={
		0.052,0.0924,0.1444,0.283,0.4678,0.8315,1.2992,1.8709,2.5465,3.326,
		5.1968,20.7870,230.9661,2078.695
	};
	return 0<udre&&udre<=14 ? var[udre-1] : 0.0;
}
/* fast correction degradation -----------------------------------------------*/
static double degfcorr(int ai)
{
	const double degf[16]={
		0.00000,0.00005,0.00009,0.00012,0.00015,0.00020,0.00030,0.00045,
		0.00060,0.00090,0.00150,0.00210,0.00270,0.00330,0.00460,0.00580
	};
	return 0<ai&&ai<=15 ? degf[ai] : 0.0058;
}
/* Implementation functions ----------------------------------------------------------------------- */
/* long term correction ----------------------------------------------------------- */
int sbssat_t::sbslongcorr(obsd_t *data,double &dclk) const{
	/* number of sat (sbssatp_t) */
	int nsbs;
	
	for (nsbs=0; nsbs<nsat; nsbs++){
		if (sat[nsbs].sat!=data->sat||sat[nsbs].lcorr.t0.time==0) continue;
		double dt=data->sigtime.timediff(sat[nsbs].lcorr.t0);
		if (fabs(dt)>MAXSBSAGEL) { 
			data->errmsg="sbas long-term correction expired!";
			return 0;
		}
		for (int i=0; i<3; i++) data->posvel[i]=sat[nsbs].lcorr.dpos[i]+sat[nsbs].lcorr.dvel[i]*dt;
		dclk=sat[nsbs].lcorr.daf0+sat[nsbs].lcorr.daf1*dt;
		
		return 1;
	}
	
	/* if sbas satellite without correction, no correction applied */
	if (satsys(data->sat,NULL)==SYS_SBS) return 1;

	return 0;
}
/* fast correction ---------------------------------------------------------------- */
int sbssat_t::sbsfastcorr(obsd_t *data,double &prc) const{
	int nsbs;

	for (nsbs=0; nsbs<nsat; nsbs++){
		if (sat[nsbs].sat!=data->sat) continue;
		if (sat->fcorr.t0.time==0) break;
		double dt=data->sigtime.timediff(sat[nsbs].fcorr.t0);

		/* expire age of correction or UDRE==14 (not monitored) */
		if (fabs(dt)>MAXSBSAGEF||sat[nsbs].fcorr.udre>=15) continue;
		prc=sat[nsbs].fcorr.prc;
#ifdef RRCENA
		if (sat[nsbs].fcorr.ai>0&&fabs(dt)<=8.0*sat[nsbs].fcorr.dt) {
			prc+=sat[nsbs].fcorr.rrc*t;
		}
#endif

		data->svar=varfcorr(sat[nsbs].fcorr.udre)+degfcorr(sat[nsbs].fcorr.ai)*dt*dt/2.0;

		return 1;
	}

	return 0;
}

/* sbas satellite ephemeris and clock correction ---------------------------------- */
int sbssat_t::sbssatcorr(obsd_t *data) const {
	double dclk=0.0,prc=0.0;

	/* sbas long term corrections (rs) */
	if (!sbslongcorr(data,dclk)) return 0;
	/* sbas fast corrections (var) */
	if (!sbsfastcorr(data,prc)) return 0;

	data->dts[0]=dclk+prc/CLIGHT;

	return 1;
}