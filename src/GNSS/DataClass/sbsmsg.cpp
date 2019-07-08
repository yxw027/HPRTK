/* Data Classes File */
/* --------------------------------------------------------------------------
* sbsmsg.cpp : sbas functions
*
*          Copyright (C) 2007-2016 by T.TAKASU, All rights reserved.
*
* option : -DRRCENA  enable rrc correction
*
* references :
*     [1] RTCA/DO-229C, Minimum operational performanc standards for global
*         positioning system/wide area augmentation system airborne equipment,
*         RTCA inc, November 28, 2001
*     [2] IS-QZSS v.1.1, Quasi-Zenith Satellite System Navigation Service
*         Interface Specification for QZSS, Japan Aerospace Exploration Agency,
*         July 31, 2009
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/10/14 1.0  new
*           2009/01/24 1.1  modify sbspntpos() api
*                           improve fast/ion correction update
*           2009/04/08 1.2  move function crc24q() to rcvlog.c
*                           support glonass, galileo and qzss
*           2009/06/08 1.3  modify sbsupdatestat()
*                           delete sbssatpos()
*           2009/12/12 1.4  support glonass
*           2010/01/22 1.5  support ems (egnos message service) format
*           2010/06/10 1.6  added api:
*                               sbssatcorr(),sbstropcorr(),sbsioncorr(),
*                               sbsupdatecorr()
*                           changed api:
*                               sbsreadmsgt(),sbsreadmsg()
*                           deleted api:
*                               sbspntpos(),sbsupdatestat()
*           2010/08/16 1.7  not reject udre==14 or give==15 correction message
*                           (2.4.0_p4)
*           2011/01/15 1.8  use api ionppp()
*                           add prn mask of qzss for qzss L1SAIF
*           2016/07/29 1.9  crc24q() -> rtk_crc24q()
*---------------------------------------------------------------------------- */
#include "GNSS/DataClass/data.h"
#include "BaseFunction/basefunction.h"
/* Constant ----------------------------------------------------------------- */
/* sbas igp definition ------------------------------------------------------ */
static const short
x1[]={ -75,-65,-55,-50,-45,-40,-35,-30,-25,-20,-15,-10,-5,  0,  5, 10, 15, 20,
25, 30, 35, 40, 45, 50, 55, 65, 75, 85 },
x2[]={ -55,-50,-45,-40,-35,-30,-25,-20,-15,-10, -5,  0,  5, 10, 15, 20, 25, 30,
35, 40, 45, 50, 55 },
x3[]={ -75,-65,-55,-50,-45,-40,-35,-30,-25,-20,-15,-10,-5,  0,  5, 10, 15, 20,
25, 30, 35, 40, 45, 50, 55, 65, 75 },
x4[]={ -85,-75,-65,-55,-50,-45,-40,-35,-30,-25,-20,-15,-10,-5,  0,  5, 10, 15,
20, 25, 30, 35, 40, 45, 50, 55, 65, 75 },
x5[]={ -180,-175,-170,-165,-160,-155,-150,-145,-140,-135,-130,-125,-120,-115,
-110,-105,-100,-95,-90,-85,-80,-75,-70,-65,-60,-55,-50,-45,
-40,-35,-30,-25,-20,-15,-10,-5,   0,   5,  10,  15,  20,  25,
30,  35,  40,  45,  50,  55,  60,  65,  70,  75,  80,  85,  90,  95,
100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165,
170, 175 },
x6[]={ -180,-170,-160,-150,-140,-130,-120,-110,-100,-90,-80,-70,-60,-50,
-40,-30,-20,-10,   0,  10,  20,  30,  40,  50,  60,  70,  80,  90,
100, 110, 120, 130, 140, 150, 160, 170 },
x7[]={ -180,-150,-120,-90,-60,-30,   0,  30,  60,  90, 120, 150 },
x8[]={ -170,-140,-110,-80,-50,-20,  10,  40,  70, 100, 130, 160 };
extern const sbsigpband_t igpband1[9][8]={ /* band 0-8 */
	{ { -180,x1,  1, 28 },{ -175,x2, 29, 51 },{ -170,x3, 52, 78 },{ -165,x2, 79,101 },
	{ -160,x3,102,128 },{ -155,x2,129,151 },{ -150,x3,152,178 },{ -145,x2,179,201 } },
	{ { -140,x4,  1, 28 },{ -135,x2, 29, 51 },{ -130,x3, 52, 78 },{ -125,x2, 79,101 },
	{ -120,x3,102,128 },{ -115,x2,129,151 },{ -110,x3,152,178 },{ -105,x2,179,201 } },
	{ { -100,x3,  1, 27 },{ -95,x2, 28, 50 },{ -90,x1, 51, 78 },{ -85,x2, 79,101 },
	{ -80,x3,102,128 },{ -75,x2,129,151 },{ -70,x3,152,178 },{ -65,x2,179,201 } },
	{ { -60,x3,  1, 27 },{ -55,x2, 28, 50 },{ -50,x4, 51, 78 },{ -45,x2, 79,101 },
	{ -40,x3,102,128 },{ -35,x2,129,151 },{ -30,x3,152,178 },{ -25,x2,179,201 } },
	{ { -20,x3,  1, 27 },{ -15,x2, 28, 50 },{ -10,x3, 51, 77 },{ -5,x2, 78,100 },
	{ 0,x1,101,128 },{ 5,x2,129,151 },{ 10,x3,152,178 },{ 15,x2,179,201 } },
	{ { 20,x3,  1, 27 },{ 25,x2, 28, 50 },{ 30,x3, 51, 77 },{ 35,x2, 78,100 },
	{ 40,x4,101,128 },{ 45,x2,129,151 },{ 50,x3,152,178 },{ 55,x2,179,201 } },
	{ { 60,x3,  1, 27 },{ 65,x2, 28, 50 },{ 70,x3, 51, 77 },{ 75,x2, 78,100 },
	{ 80,x3,101,127 },{ 85,x2,128,150 },{ 90,x1,151,178 },{ 95,x2,179,201 } },
	{ { 100,x3,  1, 27 },{ 105,x2, 28, 50 },{ 110,x3, 51, 77 },{ 115,x2, 78,100 },
	{ 120,x3,101,127 },{ 125,x2,128,150 },{ 130,x4,151,178 },{ 135,x2,179,201 } },
	{ { 140,x3,  1, 27 },{ 145,x2, 28, 50 },{ 150,x3, 51, 77 },{ 155,x2, 78,100 },
	{ 160,x3,101,127 },{ 165,x2,128,150 },{ 170,x3,151,177 },{ 175,x2,178,200 } }
};
extern const sbsigpband_t igpband2[2][5]={ /* band 9-10 */
	{ { 60,x5,  1, 72 },{ 65,x6, 73,108 },{ 70,x6,109,144 },{ 75,x6,145,180 },
	{ 85,x7,181,192 } },
	{ { -60,x5,  1, 72 },{ -65,x6, 73,108 },{ -70,x6,109,144 },{ -75,x6,145,180 },
	{ -85,x8,181,192 } }
};
/* Implementation functions ------------------------------------------------- */
/* decode half long term correction (vel code=0) ---------------------------- */
int sbsmsg_t::decode_longcorr0(int p,sbssat_t *sbssat){
	int i,n=getbitu(msg,p,6);

	if (n==0||n>MAXSAT) return 0;

	sbssat->sat[n-1].lcorr.iode=getbitu(msg,p+6,8);

	for (i=0; i<3; i++) {
		sbssat->sat[n-1].lcorr.dpos[i]=getbits(msg,p+14+9*i,9)*0.125;
		sbssat->sat[n-1].lcorr.dvel[i]=0.0;
	}
	sbssat->sat[n-1].lcorr.daf0=getbits(msg,p+41,10)*P2_31;
	sbssat->sat[n-1].lcorr.daf1=0.0;
	sbssat->sat[n-1].lcorr.t0.gpst2time(week,tow);

	return 1;
}
/* decode half long term correction (vel code=1) ---------------------------- */
int sbsmsg_t::decode_longcorr1(int p,sbssat_t *sbssat){
	int i,n=getbitu(msg,p,6),t;

	if (n==0||n>MAXSAT) return 0;

	sbssat->sat[n-1].lcorr.iode=getbitu(msg,p+6,8);

	for (i=0; i<3; i++) {
		sbssat->sat[n-1].lcorr.dpos[i]=getbits(msg,p+14+i*11,11)*0.125;
		sbssat->sat[n-1].lcorr.dvel[i]=getbits(msg,p+58+i* 8,8)*P2_11;
	}
	sbssat->sat[n-1].lcorr.daf0=getbits(msg,p+47,11)*P2_31;
	sbssat->sat[n-1].lcorr.daf1=getbits(msg,p+82,8)*P2_39;
	t=(int)getbitu(msg,p+90,13)*16-(int)tow%86400;
	if (t<=-43200) t+=86400;
	else if (t>  43200) t-=86400;
	sbssat->sat[n-1].lcorr.t0.gpst2time(week,tow+t);

	return 1;
}
/* decode half long term correction ----------------------------------------- */
int sbsmsg_t::decode_longcorrh(int p,sbssat_t *sbssat){
	if (getbitu(msg,p,1)==0) { /* vel code=0 */
		if (sbssat->iodp==(int)getbitu(msg,p+103,2)) {
			return decode_longcorr0(p+ 1,sbssat)&&
				decode_longcorr0(p+52,sbssat);
		}
	}
	else if (sbssat->iodp==(int)getbitu(msg,p+104,2)) {
		return decode_longcorr1(p+1,sbssat);
	}
	return 0;
}
/* decode type 1: prn masks ------------------------------------------------- */
int sbsmsg_t::decode_sbstype1(sbssat_t *sbssat){
	int i,n,sat;

	for (i=1,n=0; i<=210&&n<MAXSAT; i++) {
		if (getbitu(msg,13+i,1)) {
			if (i<= 37) sat=satno(SYS_GPS,i);    /*   0- 37: gps */
			else if (i<= 61) sat=satno(SYS_GLO,i-37); /*  38- 61: glonass */
			else if (i<=119) sat=0;                   /*  62-119: future gnss */
			else if (i<=138) sat=satno(SYS_SBS,i);    /* 120-138: geo/waas */
			else if (i<=182) sat=0;                   /* 139-182: reserved */
			else if (i<=192) sat=satno(SYS_SBS,i+10); /* 183-192: qzss ref [2] */
			else if (i<=202) sat=satno(SYS_QZS,i);    /* 193-202: qzss ref [2] */
			else             sat=0;                   /* 203-   : reserved */
			sbssat->sat[n++].sat=sat;
		}
	}
	sbssat->iodp=getbitu(msg,224,2);
	sbssat->nsat=n;

	return 1;
}
/* decode type 2-5,0: fast corrections -------------------------------------- */
int sbsmsg_t::decode_sbstype2(sbssat_t *sbssat){
	int i,j,iodf,type,udre;
	double prc,dt;
	gtime_t t0;

	if (sbssat->iodp!=(int)getbitu(msg,16,2)) return 0;

	type=getbitu(msg,8,6);
	iodf=getbitu(msg,14,2);

	for (i=0; i<13; i++) {
		if ((j=13*((type==0 ? 2 : type)-2)+i)>=sbssat->nsat) break;
		udre=getbitu(msg,174+4*i,4);
		t0 =sbssat->sat[j].fcorr.t0;
		prc=sbssat->sat[j].fcorr.prc;
		sbssat->sat[j].fcorr.t0.gpst2time(week,tow);
		sbssat->sat[j].fcorr.prc=getbits(msg,18+i*12,12)*0.125f;
		sbssat->sat[j].fcorr.udre=udre+1;
		dt=sbssat->sat[j].fcorr.t0.timediff(t0);
		if (t0.time==0||dt<=0.0||18.0<dt||sbssat->sat[j].fcorr.ai==0) {
			sbssat->sat[j].fcorr.rrc=0.0;
			sbssat->sat[j].fcorr.dt=0.0;
		}
		else {
			sbssat->sat[j].fcorr.rrc=(sbssat->sat[j].fcorr.prc-prc)/dt;
			sbssat->sat[j].fcorr.dt=dt;
		}
		sbssat->sat[j].fcorr.iodf=iodf;
	}
	return 1;
}
/* decode type 6: integrity info -------------------------------------------- */
int sbsmsg_t::decode_sbstype6(sbssat_t *sbssat){
	int i,iodf[4],udre;

	for (i=0; i<4; i++) {
		iodf[i]=getbitu(msg,14+i*2,2);
	}
	for (i=0; i<sbssat->nsat&&i<MAXSAT; i++) {
		if (sbssat->sat[i].fcorr.iodf!=iodf[i/13]) continue;
		udre=getbitu(msg,22+i*4,4);
		sbssat->sat[i].fcorr.udre=udre+1;
	}
	return 1;
}
/* decode type 7: fast correction degradation factor ------------------------ */
int sbsmsg_t::decode_sbstype7(sbssat_t *sbssat){
	int i;

	if (sbssat->iodp!=(int)getbitu(msg,18,2)) return 0;

	sbssat->tlat=getbitu(msg,14,4);

	for (i=0; i<sbssat->nsat&&i<MAXSAT; i++) {
		sbssat->sat[i].fcorr.ai=getbitu(msg,22+i*4,4);
	}
	return 1;
}
/* decode type 9: geo navigation message ------------------------------------ */
int sbsmsg_t::decode_sbstype9(nav_t *nav){
	seph_t seph=seph_t();
	int i,sat,t;

	if (!(sat=satno(SYS_SBS,prn))) {
		return 0;
	}
	t=(int)getbitu(msg,22,13)*16-(int)tow%86400;
	if (t<=-43200) t+=86400;
	else if (t>  43200) t-=86400;
	seph.sat=sat;
	seph.t0.gpst2time(week,tow+t);
	seph.tof.gpst2time(week,tow);
	seph.sva=getbitu(msg,35,4);
	seph.svh=seph.sva==15 ? 1 : 0; /* unhealthy if ura==15 */

	seph.pos[0]=getbits(msg,39,30)*0.08;
	seph.pos[1]=getbits(msg,69,30)*0.08;
	seph.pos[2]=getbits(msg,99,25)*0.4;
	seph.vel[0]=getbits(msg,124,17)*0.000625;
	seph.vel[1]=getbits(msg,141,17)*0.000625;
	seph.vel[2]=getbits(msg,158,18)*0.004;
	seph.acc[0]=getbits(msg,176,10)*0.0000125;
	seph.acc[1]=getbits(msg,186,10)*0.0000125;
	seph.acc[2]=getbits(msg,196,10)*0.0000625;

	seph.af0=getbits(msg,206,12)*P2_31;
	seph.af1=getbits(msg,218,8)*P2_39/2.0;

	i=prn-MINPRNSBS;
	if (!nav->seph.size()||fabs(nav->seph[i].t0.timediff(seph.t0))<1E-3) { /* not change */
		return 0;
	}
	nav->seph[NSATSBS+i]=nav->seph[i]; /* previous */
	nav->seph[i]=seph;                 /* current */

	return 1;
}
/* decode type 18: ionospheric grid point masks ----------------------------- */
int sbsmsg_t::decode_sbstype18(sbsion_t *sbsion){
	const sbsigpband_t *p;
	int i,j,n,m,band=getbitu(msg,18,4);

	if (0<=band&&band<= 8) { p=igpband1[band]; m=8; }
	else if (9<=band&&band<=10) { p=igpband2[band-9]; m=5; }
	else return 0;

	sbsion[band].iodi=(short)getbitu(msg,22,2);

	for (i=1,n=0; i<=201; i++) {
		if (!getbitu(msg,23+i,1)) continue;
		for (j=0; j<m; j++) {
			if (i<p[j].bits||p[j].bite<i) continue;
			sbsion[band].igp[n].lat=band<=8 ? p[j].y[i-p[j].bits] : p[j].x;
			sbsion[band].igp[n++].lon=band<=8 ? p[j].x : p[j].y[i-p[j].bits];
			break;
		}
	}
	sbsion[band].nigp=n;

	return 1;
}
/* decode type 24: mixed fast/long term correction -------------------------- */
int sbsmsg_t::decode_sbstype24(sbssat_t *sbssat){
	int i,j,iodf,blk,udre;

	if (sbssat->iodp!=(int)getbitu(msg,110,2)) return 0; /* check IODP */

	blk =getbitu(msg,112,2);
	iodf=getbitu(msg,114,2);

	for (i=0; i<6; i++) {
		if ((j=13*blk+i)>=sbssat->nsat) break;
		udre=getbitu(msg,86+4*i,4);

		sbssat->sat[j].fcorr.t0.gpst2time(week,tow);
		sbssat->sat[j].fcorr.prc =getbits(msg,14+i*12,12)*0.125f;
		sbssat->sat[j].fcorr.udre=udre+1;
		sbssat->sat[j].fcorr.iodf=iodf;
	}
	return decode_longcorrh(120,sbssat);
}
/* decode type 25: long term satellite error correction --------------------- */
int sbsmsg_t::decode_sbstype25(sbssat_t *sbssat){
	return decode_longcorrh(14,sbssat)&&decode_longcorrh(120,sbssat);
}
/* decode type 26: ionospheric deley corrections ---------------------------- */
int sbsmsg_t::decode_sbstype26(sbsion_t *sbsion){
	int i,j,block,delay,give,band=getbitu(msg,14,4);

	if (band>MAXBAND||sbsion[band].iodi!=(int)getbitu(msg,217,2)) return 0;

	block=getbitu(msg,18,4);

	for (i=0; i<15; i++) {
		if ((j=block*15+i)>=sbsion[band].nigp) continue;
		give=getbitu(msg,22+i*13+9,4);

		delay=getbitu(msg,22+i*13,9);
		sbsion[band].igp[j].t0.gpst2time(week,tow);
		sbsion[band].igp[j].delay=delay==0x1FF ? 0.0f : delay*0.125f;
		sbsion[band].igp[j].give=give+1;

		if (sbsion[band].igp[j].give>=16) {
			sbsion[band].igp[j].give=0;
		}
	}

	return 1;
}

/* update sbas corrections to nav -------------------------------------------
* correct satellite position and clock bias with sbas satellite corrections
* args   : gtime_t time     I   reception time
*          int    sat       I   satellite
*          nav_t  *nav      I   navigation data
*          double *rs       IO  sat position and corrected {x,y,z} (ecef) (m)
*          double *dts      IO  sat clock bias and corrected (s)
*          double *var      O   sat position and clock variance (m^2)
* return : status (1:ok,0:no correction)
* notes  : before calling the function, sbas satellite correction parameters
*          in navigation data (nav->sbssat) must be set by callig
*          sbsupdatecorr().
*          satellite clock correction include long-term correction and fast
*          correction.
*          sbas clock correction is usually based on L1C/A code. TGD or DCB has
*          to be considered for other codes
*----------------------------------------------------------------------------- */
int sbsmsg_t::sbsupdatecorr(nav_t *nav){
	int type=getbitu(msg,8,6),stat=-1;

	if (week==0) return -1;

	switch (type) {
	case  0: stat=decode_sbstype2(&nav->sbssat); break;
	case  1: stat=decode_sbstype1(&nav->sbssat); break;
	case  2:
	case  3:
	case  4:
	case  5: stat=decode_sbstype2(&nav->sbssat); break;
	case  6: stat=decode_sbstype6(&nav->sbssat); break;
	case  7: stat=decode_sbstype7(&nav->sbssat); break;
	case  9: stat=decode_sbstype9(nav);          break;
	case 18: stat=decode_sbstype18(nav->sbsion); break;
	case 24: stat=decode_sbstype24(&nav->sbssat); break;
	case 25: stat=decode_sbstype25(&nav->sbssat); break;
	case 26: stat=decode_sbstype26(nav->sbsion); break;
	case 63: break; /* null message */
	}
	return stat ? type : -1;
}