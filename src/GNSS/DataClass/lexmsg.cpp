/* Data Classes File */
/*---------------------------------------------------------------------------
* qzslex.c : qzss lex functions
*
* references :
*     [1] IS-QZSS v.1.1, Quasi-Zenith Satellite System Navigation Service
*         Interface Specification for QZSS, Japan Aerospace Exploration Agency,
*         July 31, 2009
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2011/05/27 1.0  new
*           2011/07/01 1.1  support 24bytes header format for lexconvbin()
*           2013/03/27 1.2  support message type 12
*           2013/05/11 1.3  fix bugs on decoding message type 12
*           2013/09/01 1.4  consolidate mt 12 handling codes provided by T.O.
*           2016/07/29 1.5  crc24q() -> rtk_crc24q()
*---------------------------------------------------------------------------- */

#include "Decode/rtcm.h"
#include "BaseFunction/basefunction.h"

static const char rcsid[]="$Id:$";

#define LEXFRMLEN       2000            /* lex frame length (bits) */
#define LEXHDRLEN       49              /* lex header length (bits) */
#define LEXRSLEN        256             /* lex reed solomon length (bits) */
#define LEXFRMPREAMB    0x1ACFFC1Du     /* lex frame preamble */
#define LEXEPHMAXAGE    360.0           /* max age of lex ephemeris (s) */
#define LEXIONMAXAGE    3600.0          /* max age of lex ionos correction (s) */
#define RTCM3PREAMB     0xD3            /* rtcm ver.3 frame preamble */

#define LEXHEADLEN      24              /* lex binary header length (bytes) */

/* ura value -----------------------------------------------------------------*/
static double vareph(int ura)
{
	const double uraval[]={
		0.08,0.11,0.15,0.21,0.30,0.43,0.60,0.85,1.20,1.70,2.40,3.40,4.85,6.85,
		9.65,9.65
	};
	if (ura<0||15<ura) ura=15;
	return uraval[ura];
}
/* get signed 33bit field ----------------------------------------------------*/
static double getbits_33(const unsigned char *buff,int pos)
{
	return (double)getbits(buff,pos,32)*2.0+getbitu(buff,pos+32,1);
}
/* Implementation functions ----------------------------------------------------------------------- */
/* decode tof and toe field (ref [1] 5.7.2.2.1.1) ----------------------------*/
int lexmsg_t::decode_lextof(int i,gtime_t &tof,gtime_t &toe){
	double tt,toes;
	int tow,week;

	tow =getbitu(msg,i,20);      i+=20;
	week=getbitu(msg,i,13);      i+=13;
	toes=getbitu(msg,i,16)*15.0; i+=16;
	tof.gpst2time(week,tow);
	toe.gpst2time(week,toes);

	tt=toe.timediff(tof);
	if (tt<-302400.0) toe.timeadd(604800.0);
	else if (tt> 302400.0) toe.timeadd(-604800.0);

	tof.time2str(3);
	toe.time2str(3);
	return i;
}
/* decode signal health field (ref [1] 5.7.2.2.1.1) --------------------------*/
int lexmsg_t::decode_lexhealth(int i,gtime_t tof,nav_t *nav){
	int j,sat;
	unsigned char health;

	for (j=0; j<35; j++) {
		health=getbitu(msg,i,5); i+= 5;

		if (j<3) sat=satno(SYS_QZS,j+193);
		else     sat=satno(SYS_GPS,j-2);
		if (!sat) continue;

		nav->lexeph[sat-1].tof=tof;
		nav->lexeph[sat-1].health=health;
	}
	return i;
}
/* decode ephemeris and sv clock field (ref [1] 5.7.2.2.1.2) -----------------*/
int lexmsg_t::decode_lexeph(int i,gtime_t toe,nav_t *nav){
	lexeph_t eph=lexeph_t();
	gtime_t tof;
	unsigned char health;
	int j,nPrn,sat;

	nPrn        =getbitu(msg,i,8);       i+= 8;
	eph.ura    =getbitu(msg,i,4);       i+= 4;
	eph.pos[0]=getbits_33(msg,i)*P2_6;  i+=33;
	eph.pos[1]=getbits_33(msg,i)*P2_6;  i+=33;
	eph.pos[2]=getbits_33(msg,i)*P2_6;  i+=33;
	eph.vel[0]=getbits(msg,i,28)*P2_15; i+=28;
	eph.vel[1]=getbits(msg,i,28)*P2_15; i+=28;
	eph.vel[2]=getbits(msg,i,28)*P2_15; i+=28;
	eph.acc[0]=getbits(msg,i,24)*P2_24; i+=24;
	eph.acc[1]=getbits(msg,i,24)*P2_24; i+=24;
	eph.acc[2]=getbits(msg,i,24)*P2_24; i+=24;
	eph.jerk[0]=getbits(msg,i,20)*P2_32; i+=20;
	eph.jerk[1]=getbits(msg,i,20)*P2_32; i+=20;
	eph.jerk[2]=getbits(msg,i,20)*P2_32; i+=20;
	eph.af0    =getbits(msg,i,26)*P2_35; i+=26;
	eph.af1    =getbits(msg,i,20)*P2_48; i+=20;
	eph.tgd    =getbits(msg,i,13)*P2_35; i+=13;
	for (j=0; j<7; j++) {
		eph.isc[j]=getbits(msg,i,13)*P2_35; i+=13;
	}
	if (nPrn==255) return i; /* no satellite */

	if (1<=nPrn&&nPrn<= 32) sat=satno(SYS_GPS,nPrn);
	else if (193<=nPrn&&nPrn<=195) sat=satno(SYS_QZS,nPrn);
	else {
		return i;
	}
	eph.toe=toe;
	eph.sat=sat;
	tof   =nav->lexeph[sat-1].tof;
	health=nav->lexeph[sat-1].health;
	nav->lexeph[sat-1]=eph;
	nav->lexeph[sat-1].tof   =tof;
	nav->lexeph[sat-1].health=health;

	return i;
}
/* decode ionosphere correction field (ref [1] 5.7.2.2.1.3) ------------------*/
int lexmsg_t::decode_lexion(int i,gtime_t tof,nav_t *nav){
	lexion_t ion=lexion_t();
	int tow,week;

	tow=getbitu(msg,i,20); i+=20;

	if (tow==0xFFFFF) { /* correction not available */
		return i+192;
	}
	week=getbitu(msg,i,13); i+=13;
	ion.t0.gpst2time(week,tow);
	ion.tspan     =getbitu(msg,i,8)*60.0; i+= 8; /* time span (s) */
	ion.pos0[0]   =getbits(msg,i,19)*1E-5; i+=19; /* latitude  (rad) */
	ion.pos0[1]   =getbits(msg,i,20)*1E-5; i+=20; /* longitude (rad) */
	ion.coef[0][0]=getbits(msg,i,22)*1E-3; i+=22;
	ion.coef[1][0]=getbits(msg,i,22)*1E-2; i+=22;
	ion.coef[2][0]=getbits(msg,i,22)*1E-2; i+=22;
	ion.coef[0][1]=getbits(msg,i,22)*1E-2; i+=22;
	ion.coef[1][1]=getbits(msg,i,22)*1E-2; i+=22;
	ion.coef[2][1]=getbits(msg,i,22)*1E-1; i+=22;
	nav->lexion=ion;

	return i;
}
/* convert lex type 12 to rtcm ssr message -----------------------------------*/
int lexmsg_t::lex2rtcm(int i,unsigned char *buff){
	unsigned int crc;
	int j,ns,type,n=0;

	if (i+12>=LEXFRMLEN-LEXHDRLEN-LEXRSLEN) return 0;

	switch ((type=getbitu(msg,i,12))) {

	case 1057: ns=getbitu(msg,i+62,6); n=68+ns*135; break; /* gps */
	case 1058: ns=getbitu(msg,i+61,6); n=67+ns* 76; break;
	case 1059: ns=getbitu(msg,i+61,6); n=67;
		for (j=0; j<ns; j++) n+=11+getbitu(msg,i+n+6,5)*19; break;
	case 1060: ns=getbitu(msg,i+62,6); n=68+ns*205; break;
	case 1061: ns=getbitu(msg,i+61,6); n=67+ns* 12; break;
	case 1062: ns=getbitu(msg,i+61,6); n=67+ns* 28; break;
	case 1063: ns=getbitu(msg,i+59,6); n=65+ns*134; break; /* glonass */
	case 1064: ns=getbitu(msg,i+58,6); n=64+ns* 75; break;
	case 1065: ns=getbitu(msg,i+58,6); n=64;
		for (j=0; j<ns; j++) n+=10+getbitu(msg,i+n+5,5)*19; break;
	case 1066: ns=getbitu(msg,i+59,6); n=65+ns*204; break;
	case 1067: ns=getbitu(msg,i+58,6); n=64+ns* 11; break;
	case 1068: ns=getbitu(msg,i+58,6); n=64+ns* 27; break;
	case 1240: ns=getbitu(msg,i+62,6); n=68+ns*135; break; /* galileo */
	case 1241: ns=getbitu(msg,i+61,6); n=67+ns* 76; break;
	case 1242: ns=getbitu(msg,i+61,6); n=67;
		for (j=0; j<ns; j++) n+=11+getbitu(msg,i+n+6,5)*19; break;
	case 1243: ns=getbitu(msg,i+62,6); n=68+ns*205; break;
	case 1244: ns=getbitu(msg,i+61,6); n=67+ns* 12; break;
	case 1245: ns=getbitu(msg,i+61,6); n=67+ns* 28; break;
	case 1246: ns=getbitu(msg,i+62,4); n=66+ns*133; break; /* qzss */
	case 1247: ns=getbitu(msg,i+61,4); n=65+ns* 74; break;
	case 1248: ns=getbitu(msg,i+61,4); n=65;
		for (j=0; j<ns; j++) n+=9+getbitu(msg,i+n+4,5)*19; break;
	case 1249: ns=getbitu(msg,i+62,4); n=66+ns*203; break;
	case 1250: ns=getbitu(msg,i+61,4); n=65+ns* 10; break;
	case 1251: ns=getbitu(msg,i+61,4); n=65+ns* 26; break;
	default:
		//if (type) trace(2,"lex 12: unsupported type=%4d\n",type);
		return 0;
	}
	n=(n+7)/8; /* message length (bytes) */

	if (i+n*8>LEXFRMLEN-LEXRSLEN) {
		return 0;
	}
	/* save rtcm message to buffer */
	setbitu(buff,0,8,RTCM3PREAMB);
	setbitu(buff,8,6,0);
	setbitu(buff,14,10,n);
	for (j=0; j<n; j++) {
		buff[j+3]=getbitu(msg,i+j*8,8);
	}
	crc=rtk_crc24q(buff,3+n);
	setbitu(buff,24+n*8,24,crc);
	return n;
}
/* decode type 10: ephemeris data and clock (ref [1] 5.7.2.2.1,1) ----------- */
int lexmsg_t::decode_lextype10(nav_t *nav,gtime_t tof){
	gtime_t toe;
	int i=0,j;

	/* decode tof and toe field */
	i=decode_lextof(i,tof,toe);
	/* decode signal health field */
	i=decode_lexhealth(i,tof,nav);
	/* decode ephemeris and sv clock field */
	for (j=0; j<3; j++) {
		i=decode_lexeph(i,toe,nav);
	}
	return 1;
}
/* decode type 11: ephemeris data and clock (ref [1] 5.7.2.2.1,1) ----------- */
int lexmsg_t::decode_lextype11(nav_t *nav,gtime_t tof){
	gtime_t toe;
	int i=0,j;

	/* decode tof and toe field */
	i=decode_lextof(i,tof,toe);
	/* decode signal health field */
	i=decode_lexhealth(i,tof,nav);
	/* decode ephemeris and sv clock field */
	for (j=0; j<2; j++) {
		i=decode_lexeph(i,toe,nav);
	}
	/* decode ionosphere correction field */
	decode_lexion(i,tof,nav);

	return 1;
}
/* decode type 12: madoca orbit and clock correction ------------------------ */
int lexmsg_t::decode_lextype12(nav_t *nav,gtime_t tof){
	static rtcm_3 stock_rtcm=rtcm_3();
	rtcm_3 rtcm=rtcm_3();
	double tow;
	unsigned char buff[1200];
	int i=0,j,k,l,n,week;

	tow =getbitu(msg,i,20); i+=20;
	week=getbitu(msg,i,13); i+=13;
	tof.gpst2time(week,tow);

	/* copy rtcm ssr corrections */
	for (k=0; k<MAXSAT; k++) {
		rtcm.ssr[k]=nav->ssr[k];
		rtcm.ssr[k].update=0;
	}
	/* convert lex type 12 to rtcm ssr message */
	while ((n=lex2rtcm(i,buff))) {

		rtcm.time=tof;

		for (j=0; j<n+6; j++) {

			/* input rtcm ssr message */
			if (rtcm.decode(buff[j])==-1) continue;

			/* update ssr corrections in nav data */
			for (k=0; k<MAXSAT; k++) {
				if (!rtcm.ssr[k].update) continue;

				rtcm.ssr[k].update=0;

				if (rtcm.ssr[k].t0[3].time){      /* ura */
					stock_rtcm.ssr[k].t0[3]=rtcm.ssr[k].t0[3];
					stock_rtcm.ssr[k].udi[3]=rtcm.ssr[k].udi[3];
					stock_rtcm.ssr[k].iod[3]=rtcm.ssr[k].iod[3];
					stock_rtcm.ssr[k].ura=rtcm.ssr[k].ura;
				}
				if (rtcm.ssr[k].t0[2].time){      /* hr-clock correction*/

												  /* convert hr-clock correction to clock correction*/
					stock_rtcm.ssr[k].t0[1]=rtcm.ssr[k].t0[2];
					stock_rtcm.ssr[k].udi[1]=rtcm.ssr[k].udi[2];
					stock_rtcm.ssr[k].iod[1]=rtcm.ssr[k].iod[2];
					stock_rtcm.ssr[k].dclk[0]=rtcm.ssr[k].hrclk;
					stock_rtcm.ssr[k].dclk[1]=stock_rtcm.ssr[k].dclk[2]=0.0;

					/* activate orbit correction(60.0s is tentative) */
					if ((stock_rtcm.ssr[k].iod[0]==rtcm.ssr[k].iod[2]) &&
						(stock_rtcm.ssr[k].t0[0].timediff(rtcm.ssr[k].t0[2]) < 60.0)){
						rtcm.ssr[k] = stock_rtcm.ssr[k];
					}
					else continue; /* not apply */
				}
				else if (rtcm.ssr[k].t0[0].time){ /* orbit correction*/
					stock_rtcm.ssr[k].t0[0]=rtcm.ssr[k].t0[0];
					stock_rtcm.ssr[k].udi[0]=rtcm.ssr[k].udi[0];
					stock_rtcm.ssr[k].iod[0]=rtcm.ssr[k].iod[0];
					for (l=0; l<3; l++) {
						stock_rtcm.ssr[k].deph[l]=rtcm.ssr[k].deph[l];
						stock_rtcm.ssr[k].ddeph[l]=rtcm.ssr[k].ddeph[l];
					}
					stock_rtcm.ssr[k].iode=rtcm.ssr[k].iode;
					stock_rtcm.ssr[k].refd=rtcm.ssr[k].refd;

					/* activate clock correction(60.0s is tentative) */
					if ((stock_rtcm.ssr[k].iod[1]==rtcm.ssr[k].iod[0]) &&
						(stock_rtcm.ssr[k].t0[1].timediff(rtcm.ssr[k].t0[0]) < 60.0)){
						rtcm.ssr[k] = stock_rtcm.ssr[k];
					}
					else continue; /* not apply */
				}
				/* apply */
				nav->ssr[k]=rtcm.ssr[k];
			}
		}
		i+=n*8;
	}
	return 1;
}
/* decode type 20: gsi experiment message (ref [1] 5.7.2.2.2) --------------- */
int lexmsg_t::decode_lextype20(nav_t *nav,gtime_t tof){ 
	return 0; /* not supported */ 
}
/* update lex corrections ---------------------------------------------------
* update lex correction parameters in navigation data with a lex message
* args   : lexmsg_t *msg    I   lex message
*          nav_t    *nav    IO  navigation data
*          gtime_t  *tof    O   time of frame
* return : status (1:ok,0:error or not supported type)
*---------------------------------------------------------------------------- */
int lexmsg_t::lexupdatecorr(nav_t *nav,gtime_t &tof){
	switch (type) {
	case 10: return decode_lextype10(nav,tof); /* jaxa */
	case 11: return decode_lextype11(nav,tof); /* jaxa */
	case 12: return decode_lextype12(nav,tof); /* jaxa */
	case 20: return decode_lextype20(nav,tof); /* gsi */
	}
	return 0;
}