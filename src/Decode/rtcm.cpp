#include "Decode/rtcm.h"
#include "BaseFunction/basefunction.h"

/* decode rtcm class ---------------------------------------------------------------------------------
* references :
*     [1] RTCM Recommended Standards for Differential GNSS (Global Navigation
*         Satellite Systems) Service version 2.3, August 20, 2001
*     [2] RTCM Standard 10403.1 for Differential GNSS (Global Navigation
*         Satellite Systems) Services - Version 3, Octobar 27, 2006
*     [3] RTCM 10403.1-Amendment 3, Amendment 3 to RTCM Standard 10403.1
*     [4] RTCM Paper, April 12, 2010, Proposed SSR Messages for SV Orbit Clock,
*         Code Biases, URA
*     [5] RTCM Paper 012-2009-SC104-528, January 28, 2009 (previous ver of [4])
*     [6] RTCM Paper 012-2009-SC104-582, February 2, 2010 (previous ver of [4])
*     [7] RTCM Standard 10403.1 - Amendment 5, Differential GNSS (Global
*         Navigation Satellite Systems) Services - version 3, July 1, 2011
*     [8] RTCM Paper 019-2012-SC104-689 (draft Galileo ephmeris messages)
*     [9] RTCM Paper 163-2012-SC104-725 (draft QZSS ephemeris message)
*     [10] RTCM Paper 059-2011-SC104-635 (draft Galileo and QZSS ssr messages)
*     [11] RTCM Paper 034-2012-SC104-693 (draft multiple signal messages)
*     [12] RTCM Paper 133-2012-SC104-709 (draft QZSS MSM messages)
*     [13] RTCM Paper 122-2012-SC104-707.r1 (draft MSM messages)
*     [14] RTCM Standard 10403.2, Differential GNSS (Global Navigation Satellite
*          Systems) Services - version 3, February 1, 2013
*     [15] RTCM Standard 10403.2, Differential GNSS (Global Navigation Satellite
*          Systems) Services - version 3, with amendment 1/2, november 7, 2013
*     [16] Proposal of new RTCM SSR Messages (ssr_1_gal_qzss_sbas_dbs_v05)
 * ------------------------------------------------------------------------------------------------ */

/* Constant */
#define RTCM2PREAMB 0x66        /* rtcm ver.2 frame preamble */
#define RTCM3PREAMB 0xD3        /* rtcm ver.3 frame preamble */

/* RTCM version 3 --------------------------------------------------------------------------------- */
/* Constant */
#define PRUNIT_GPS  299792.458  /* rtcm ver.3 unit of gps pseudorange (m) */
#define PRUNIT_GLO  599584.916  /* rtcm ver.3 unit of glonass pseudorange (m) */
#define RANGE_MS    (CLIGHT*0.001)      /* range in 1 ms */

#define P2_10       0.0009765625          /* 2^-10 */
#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define P2_66       1.355252715606880E-20 /* 2^-66 */

/* ssr 3 and 7 signal and tracking mode ids ----------------------------------*/
static const int codes_gps[]={
	CODE_L1C,CODE_L1P,CODE_L1W,CODE_L1Y,CODE_L1M,CODE_L2C,CODE_L2D,CODE_L2S,
	CODE_L2L,CODE_L2X,CODE_L2P,CODE_L2W,CODE_L2Y,CODE_L2M,CODE_L5I,CODE_L5Q,
	CODE_L5X
};
static const int codes_glo[]={
	CODE_L1C,CODE_L1P,CODE_L2C,CODE_L2P
};
static const int codes_gal[]={
	CODE_L1A,CODE_L1B,CODE_L1C,CODE_L1X,CODE_L1Z,CODE_L5I,CODE_L5Q,CODE_L5X,
	CODE_L7I,CODE_L7Q,CODE_L7X,CODE_L8I,CODE_L8Q,CODE_L8X,CODE_L6A,CODE_L6B,
	CODE_L6C,CODE_L6X,CODE_L6Z
};
static const int codes_qzs[]={
	CODE_L1C,CODE_L1S,CODE_L1L,CODE_L2S,CODE_L2L,CODE_L2X,CODE_L5I,CODE_L5Q,
	CODE_L5X,CODE_L6S,CODE_L6L,CODE_L6X,CODE_L1X
};
static const int codes_bds[]={
	CODE_L1I,CODE_L1Q,CODE_L1X,CODE_L7I,CODE_L7Q,CODE_L7X,CODE_L6I,CODE_L6Q,
	CODE_L6X
};
static const int codes_sbs[]={
	CODE_L1C,CODE_L5I,CODE_L5Q,CODE_L5X
};

/* msm signal id table -------------------------------------------------------*/
const char *msm_sig_gps[32]={
	/* GPS: ref [13] table 3.5-87, ref [14][15] table 3.5-91 */
	""  ,"1C","1P","1W","1Y","1M",""  ,"2C","2P","2W","2Y","2M", /*  1-12 */
	""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X", /* 13-24 */
	""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"                      /* 25-32 */
};
const char *msm_sig_glo[32]={
	/* GLONASS: ref [13] table 3.5-93, ref [14][15] table 3.5-97 */
	""  ,"1C","1P",""  ,""  ,""  ,""  ,"2C","2P",""  ,"3I","3Q",
	"3X",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
	""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};
const char *msm_sig_gal[32]={
	/* Galileo: ref [15] table 3.5-100 */
	""  ,"1C","1A","1B","1X","1Z",""  ,"6C","6A","6B","6X","6Z",
	""  ,"7I","7Q","7X",""  ,"8I","8Q","8X",""  ,"5I","5Q","5X",
	""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};
const char *msm_sig_qzs[32]={
	/* QZSS: ref [15] table 3.5-103 */
	""  ,"1C",""  ,""  ,""  ,""  ,""  ,""  ,"6S","6L","6X",""  ,
	""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X",
	""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"
};
const char *msm_sig_sbs[32]={
	/* SBAS: ref [13] table 3.5-T+005 */
	""  ,"1C",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
	""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,"5I","5Q","5X",
	""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};
const char *msm_sig_cmp[32]={
	/* BeiDou: ref [15] table 3.5-106 */
	""  ,"1I","1Q","1X",""  ,""  ,""  ,"6I","6Q","6X",""  ,""  ,
	""  ,"7I","7Q","7X",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
	""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};
/* ssr update intervals ------------------------------------------------------*/
static const double ssrudint[16]={
	1,2,5,10,15,30,60,120,240,300,600,900,1800,3600,7200,10800
};

/* RTCM control struct type --------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
/* Constructor ------------------------------------------------------------------------------------ */
rtcm_t::rtcm_t(){
	int i,j;
	/* num */
	staid=stah=seqno=outtype=obsflag=0;
	ephsat=nbyte=nbit=len=0;
	/* string */
	msg=string(128,0);
	msgtype=msmtype[0]=msmtype[1]=msmtype[2]=
		msmtype[3]=msmtype[4]=msmtype[5]="";
	
	/* multi */
	for (i=0; i<MAXSAT; i++){
		for (j=0; j<NFREQ+NEXOBS; j++){
			cp[i][j]=lock[i][j]=loss[i][j]=0;
		}
	}
}
rtcm_t::~rtcm_t(){
}
/* get observation data index --------------------------------------------------------------------- */
int rtcm_t::obsindex(gtime_t time,int sat){
	int i,j;

	for (i=0; i<obs.n; i++) {
		if (obs.data[i].sat==sat) return i; /* field already exists */
	}
	if (i>=MAXOBS) return -1; /* overflow */

							  /* add new field */
	obs.data[i].time=time;
	obs.data[i].sat=sat;
	for (j=0; j<NFREQ; j++) {
		obs.data[i].L[j]=obs.data[i].P[j]=0.0;
		obs.data[i].D[j]=0.0;
		obs.data[i].SNR[j]=obs.data[i].LLI[j]=obs.data[i].code[j]=0;
	}
	obs.n++;
	return i;
}
int rtcm_t::decode(unsigned char data){
	return 0;
}

/* RTCM 2 control struct type ------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
rtcm_2::rtcm_2(){
}
rtcm_2::~rtcm_2(){
}
/* decode type 1/9: differential gps correction/partial correction set ---------------------------- */
int rtcm_2::decode_type1(){
	int i=48,fact,udre,prn,sat,iod;
	double prc,rrc;

	while (i+40<=len*8) {
		fact=getbitu(buff,i,1); i+= 1;
		udre=getbitu(buff,i,2); i+= 2;
		prn =getbitu(buff,i,5); i+= 5;
		prc =getbits(buff,i,16); i+=16;
		rrc =getbits(buff,i,8); i+= 8;
		iod =getbits(buff,i,8); i+= 8;
		if (prn==0) prn=32;
		if (prc==0x80000000||rrc==0xFFFF8000) {
			continue;
		}
		if (dgps) {
			sat=satno(SYS_GPS,prn);
			dgps[sat-1].t0=time;
			dgps[sat-1].prc=prc*(fact ? 0.32 : 0.02);
			dgps[sat-1].rrc=rrc*(fact ? 0.032 : 0.002);
			dgps[sat-1].iod=iod;
			dgps[sat-1].udre=udre;
		}
	}
	return 7;
}
/* decode type 3: reference station parameter ----------------------------------------------------- */
int rtcm_2::decode_type3(){
	int i=48;

	if (i+96<=len*8) {
		sta.pos[0]=getbits(buff,i,32)*0.01; i+=32;
		sta.pos[1]=getbits(buff,i,32)*0.01; i+=32;
		sta.pos[2]=getbits(buff,i,32)*0.01;
	}
	else return -1;
	return 5;
}
/* decode type 14: gps time of week --------------------------------------------------------------- */
int rtcm_2::decode_type14(){
	double zcnt;
	int i=48,week,hour,leaps;

	zcnt=getbitu(buff,24,13);
	if (i+24<=len*8) {
		week =getbitu(buff,i,10); i+=10;
		hour =getbitu(buff,i,8); i+= 8;
		leaps=getbitu(buff,i,6);
	}
	else return -1;
	week=adjgpsweek(week);
	time.gpst2time(week,hour*3600.0+zcnt*0.6);
	nav.leaps=leaps;
	return 6;
}
/* decode type 16: gps special message ------------------------------------------------------------ */
int rtcm_2::decode_type16(){
	int i=48,n=0;

	msg=string(128,0);
	while (i+8<=len*8&&n<90) {
		msg[n++]=getbitu(buff,i,8); i+=8;
	}
	msg[n]='\0';
	return 9;
}
/* decode type 17: gps ephemerides ---------------------------------------------------------------- */
int rtcm_2::decode_type17(){
	eph_t eph=eph_t();
	double toc,sqrtA;
	int i=48,week,prn,sat;

	if (i+480<=len*8) {
		week      =getbitu(buff,i,10);              i+=10;
		eph.idot  =getbits(buff,i,14)*P2_43*SC2RAD; i+=14;
		eph.iode  =getbitu(buff,i,8);              i+= 8;
		toc       =getbitu(buff,i,16)*16.0;         i+=16;
		eph.f1    =getbits(buff,i,16)*P2_43;        i+=16;
		eph.f2    =getbits(buff,i,8)*P2_55;        i+= 8;
		eph.crs   =getbits(buff,i,16)*P2_5;         i+=16;
		eph.deln  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
		eph.cuc   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.e     =getbitu(buff,i,32)*P2_33;        i+=32;
		eph.cus   =getbits(buff,i,16);              i+=16;
		sqrtA     =getbitu(buff,i,32)*P2_19;        i+=32;
		eph.toes  =getbitu(buff,i,16);              i+=16;
		eph.OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.cic   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.i0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.cis   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.crc   =getbits(buff,i,16)*P2_5;         i+=16;
		eph.OMGd  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
		eph.M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.iodc  =getbitu(buff,i,10);              i+=10;
		eph.f0    =getbits(buff,i,22)*P2_31;        i+=22;
		prn       =getbitu(buff,i,5);              i+= 5+3;
		eph.tgd[0]=getbits(buff,i,8)*P2_31;        i+= 8;
		eph.code  =getbitu(buff,i,2);              i+= 2;
		eph.sva   =getbitu(buff,i,4);              i+= 4;
		eph.svh   =getbitu(buff,i,6);              i+= 6;
		eph.flag  =getbitu(buff,i,1);
	}
	else return -1;

	if (prn==0) prn=32;
	sat=satno(SYS_GPS,prn);
	eph.sat=sat;
	eph.week=adjgpsweek(week);
	eph.toe.gpst2time(eph.week,eph.toes);
	eph.toc.gpst2time(eph.week,toc);
	eph.ttr=time;
	eph.A=sqrtA*sqrtA;
	nav.eph[sat-1]=eph;
	ephsat=sat;
	return 2;
}
/* decode type 18: rtk uncorrected carrier-phase -------------------------------------------------- */
int rtcm_2::decode_type18(){
	gtime_t time;
	double usec,cp,tt;
	int i=48,index,freq,sync=1,code,sys,prn,sat,losses;

	if (i+24<=len*8) {
		freq=getbitu(buff,i,2); i+= 2+2;
		usec=getbitu(buff,i,20); i+=20;
	}
	else return -1;
	if (freq&0x1) return -1;

	freq>>=1;

	while (i+48<=len*8&&obs.n<MAXOBS) {
		sync=getbitu(buff,i,1); i+= 1;
		code=getbitu(buff,i,1); i+= 1;
		sys =getbitu(buff,i,1); i+= 1;
		prn =getbitu(buff,i,5); i+= 5+3;
		losses=getbitu(buff,i,5); i+= 5;
		cp  =getbits(buff,i,32); i+=32;
		if (prn==0) prn=32;
		if (!(sat=satno(sys ? SYS_GLO : SYS_GPS,prn))) {
			continue;
		}
		time.timeadd(usec*1E-6);
		if (sys) time.utc2gpst(); /* convert glonass time to gpst */

		tt=obs.data[0].time.timediff(time);
		if (obsflag||fabs(tt)>1E-9) {
			obs.n=obsflag=0;
		}
		if ((index=obsindex(time,sat))>=0) {
			obs.data[index].L[freq]=-cp/256.0;
			obs.data[index].LLI[freq]=loss[sat-1][freq]!=losses;
			obs.data[index].code[freq]=
				!freq ? (code ? CODE_L1P : CODE_L1C) : (code ? CODE_L2P : CODE_L2C);
			loss[sat-1][freq]=losses;
		}
	}
	obsflag=!sync;
	return sync ? 0 : 1;
}
/* decode type 19: rtk uncorrected pseudorange ---------------------------------------------------- */
int rtcm_2::decode_type19(){
	gtime_t time;
	double usec,pr,tt;
	int i=48,index,freq,sync=1,code,sys,prn,sat;

	if (i+24<=len*8) {
		freq=getbitu(buff,i,2); i+= 2+2;
		usec=getbitu(buff,i,20); i+=20;
	}
	else return -1;

	if (freq&0x1) return -1;

	freq>>=1;

	while (i+48<=len*8&&obs.n<MAXOBS) {
		sync=getbitu(buff,i,1); i+= 1;
		code=getbitu(buff,i,1); i+= 1;
		sys =getbitu(buff,i,1); i+= 1;
		prn =getbitu(buff,i,5); i+= 5+8;
		pr  =getbitu(buff,i,32); i+=32;
		if (prn==0) prn=32;
		if (!(sat=satno(sys ? SYS_GLO : SYS_GPS,prn))) {
			continue;
		}
		time.timeadd(usec*1E-6);
		if (sys) time.utc2gpst(); /* convert glonass time to gpst */

		tt=obs.data[0].time.timediff(time);
		if (obsflag||fabs(tt)>1E-9) {
			obs.n=obsflag=0;
		}
		if ((index=obsindex(time,sat))>=0) {
			obs.data[index].P[freq]=pr*0.02;
			obs.data[index].code[freq]=
				!freq ? (code ? CODE_L1P : CODE_L1C) : (code ? CODE_L2P : CODE_L2C);
		}
	}
	obsflag=!sync;
	return sync ? 0 : 1;
}
/* decode type 22: extended reference station parameter ------------------------------------------- */
int rtcm_2::decode_type22(){
	double del[2][3]={ { 0 } },hgt=0.0;
	int i=48,j,noh;

	if (i+24<=len*8) {
		del[0][0]=getbits(buff,i,8)/25600.0; i+=8;
		del[0][1]=getbits(buff,i,8)/25600.0; i+=8;
		del[0][2]=getbits(buff,i,8)/25600.0; i+=8;
	}
	else return -1;
	if (i+24<=len*8) {
		i+=5; noh=getbits(buff,i,1); i+=1;
		hgt=noh ? 0.0 : getbitu(buff,i,18)/25600.0;
		i+=18;
	}
	if (i+24<=len*8) {
		del[1][0]=getbits(buff,i,8)/1600.0; i+=8;
		del[1][1]=getbits(buff,i,8)/1600.0; i+=8;
		del[1][2]=getbits(buff,i,8)/1600.0;
	}
	sta.deltype=1; /* xyz */
	for (j=0; j<3; j++) sta.del[j]=del[0][j];
	sta.hgt=hgt;
	return 5;
}
/* decode type 23: antenna type definition record ------------------------------------------------- */
int rtcm_2::decode_type23(){
	return 0;
}
/* decode type 24: antenna reference point (arp) -------------------------------------------------- */
int rtcm_2::decode_type24(){
	return 0;
}
/* decode type 31: differential glonass correction ------------------------------------------------ */
int rtcm_2::decode_type31(){
	return 0;
}
/* decode type 32: differential glonass reference station parameters ------------------------------ */
int rtcm_2::decode_type32(){
	return 0;
}
/* decode type 34: glonass partial differential correction set ------------------------------------ */
int rtcm_2::decode_type34(){
	return 0;
}
/* decode type 36: glonass special message -------------------------------------------------------- */
int rtcm_2::decode_type36(){
	return 0;
}
/* decode type 37: gnss system time offset -------------------------------------------------------- */
int rtcm_2::decode_type37(){
	return 0;
}
/* decode type 59: proprietary message ------------------------------------------------------------ */
int rtcm_2::decode_type59(){
	return 0;
}
/* adjust hourly rollover of rtcm 2 time ---------------------------------------------------------- */
void rtcm_2::adjhour(double zcnt){
	double tow,hour,sec;
	int week;

	/* if no time, get cpu time */
	if (time.time==0) time.timeget()->utc2gpst();
	tow=time.time2gpst(&week);
	hour=floor(tow/3600.0);
	sec=tow-hour*3600.0;
	if (zcnt<sec-1800.0) zcnt+=3600.0;
	else if (zcnt>sec+1800.0) zcnt-=3600.0;
	time.gpst2time(week,hour*3600+zcnt);
}
/* decode rtcm ver.2 message ---------------------------------------------------------------------- */
int rtcm_2::decode_rtcm2(){
	double zcnt;
	string str;
	int staid,seqno,stah,ret=0,type=getbitu(buff,8,6);

	if ((zcnt=getbitu(buff,24,13)*0.6)>=3600.0) {
		return -1;
	}
	adjhour(zcnt);
	staid=getbitu(buff,14,10);
	seqno=getbitu(buff,37,3);
	stah =getbitu(buff,45,3);
	if (seqno-seqno!=1&&seqno-seqno!=-7) {
	}
	seqno=seqno;
	stah =stah;

	if (outtype)
		msgtype="RTCM "+int2str(2," ",type,str)+" ("+int2str(4," ",len,str)+") zcnt="+
		doul2str(7,1," ",zcnt,str)+" staid="+int2str(3," ",staid,str)+" seqno="+to_string(seqno);
	if (type==3||type==22||type==23||type==24) {
		if (staid!=0&&staid!=staid) {
		}
		staid=staid;
	}
	if (staid!=0&&staid!=staid) {
		return -1;
	}
	switch (type) {
	case  1: ret=decode_type1(); break;
	case  3: ret=decode_type3(); break;
	case  9: ret=decode_type1(); break;
	case 14: ret=decode_type14(); break;
	case 16: ret=decode_type16(); break;
	case 17: ret=decode_type17(); break;
	case 18: ret=decode_type18(); break;
	case 19: ret=decode_type19(); break;
	case 22: ret=decode_type22(); break;
	case 23: ret=decode_type23(); break; /* not supported */
	case 24: ret=decode_type24(); break; /* not supported */
	case 31: ret=decode_type31(); break; /* not supported */
	case 32: ret=decode_type32(); break; /* not supported */
	case 34: ret=decode_type34(); break; /* not supported */
	case 36: ret=decode_type36(); break; /* not supported */
	case 37: ret=decode_type37(); break; /* not supported */
	case 59: ret=decode_type59(); break; /* not supported */
	}
	if (ret>=0) {
		if (1<=type&&type<=99) nmsg2[type]++; else nmsg2[0]++;
	}
	return ret;
}
/* implementation funuctions ---------------------------------------------------------------------- */
/* input rtcm 2 message from stream ------------------------------------------------------------------
* fetch next rtcm 2 message and input a message from byte stream
* args   : rtcm_t *rtcm IO   rtcm control struct
*          unsigned char data I stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 5: input station pos/ant parameters,
*                  6: input time parameter, 7: input dgps corrections,
*                  9: input special message)
* notes  : before firstly calling the function, time in rtcm control struct has
*          to be set to the approximate time within 1/2 hour in order to resolve
*          ambiguity of time in rtcm messages.
*          supported msgs RTCM ver.2: 1,3,9,14,16,17,18,19,22
*          refer [1] for RTCM ver.2
--------------------------------------------------------------------------------------------------- */
int rtcm_2::decode(unsigned char data){
	unsigned char preamb;
	int i;

	if ((data&0xC0)!=0x40) return 0; /* ignore if upper 2bit != 01 */

	for (i=0; i<6; i++,data>>=1) { /* decode 6-of-8 form */
		word=(word<<1)+(data&1);

		/* synchronize frame */
		if (nbyte==0) {
			preamb=(unsigned char)(word>>22);
			if (word&0x40000000) preamb^=0xFF; /* decode preamble */
			if (preamb!=RTCM2PREAMB) continue;

			/* check parity */
			if (!decode_word(word,buff)) continue;
			nbyte=3; nbit=0;
			continue;
		}
		if (++nbit<30) continue; else nbit=0;

		/* check parity */
		if (!decode_word(word,buff+nbyte)) {
			nbyte=0; word&=0x3;
			continue;
		}
		nbyte+=3;
		if (nbyte==6) len=(buff[5]>>3)*3+6;
		if (nbyte<len) continue;
		nbyte=0; word&=0x3;

		/* decode rtcm2 message */
		return decode_rtcm2();
	}
	return 0;
}
/* RTCM 3 control struct type ------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
rtcm_3::rtcm_3(){
}
rtcm_3::~rtcm_3(){
}
/* get sign-magnitude bits ------------------------------------------------------------------------ */
double rtcm_3::getbitg(int pos,int len){
	double value=getbitu(buff,pos+1,len-1);
	return getbitu(buff,pos,1) ? -value : value;
}
/* adjust daily rollover of glonass time ---------------------------------------------------------- */
void rtcm_3::adjday_glot(double tod){
	gtime_t t0;
	double tow,tod_p;
	int week;

	if (time.time==0) time.timeget();
	t0=*time.timeadd(10800.0); /* glonass time */
	time.utc2gpst();
	tow=t0.time2gpst(&week);
	tod_p=fmod(tow,86400.0); tow-=tod_p;
	if (tod<tod_p-43200.0) tod+=86400.0;
	else if (tod>tod_p+43200.0) tod-=86400.0;
	t0.gpst2time(week,tow+tod);
	time=*t0.timeadd(-10800.0)->utc2gpst();
}
/* adjust weekly rollover of gps time ------------------------------------------------------------- */
void rtcm_3::adjweek(double tow){
	double tow_p;
	int week;

	/* if no time, get cpu time */
	if (time.time==0) time.timeget()->utc2gpst();
	tow_p=time.time2gpst(&week);
	if (tow<tow_p-302400.0) tow+=604800.0;
	else if (tow>tow_p+302400.0) tow-=604800.0;
	time.gpst2time(week,tow);
}
/* adjust weekly rollover of bdt time ------------------------------------------------------------- */
int rtcm_3::adjbdtweek(int week,double sec){
	if (week<1) return -1;
	gtime_t t0,tws;
	t0.timeget()->utc2gpst()->gpst2bdt()->time2bdt(NULL);

	tws.bdt2time(week,sec);
	double dt=fabs(t0.timediff(tws));
	if (dt>21600.0) return -1;
	return week;
}
/* adjust carrier-phase rollover ------------------------------------------------------------------ */
double rtcm_3::adjcp(int sat,int freq,double cccp){
	if (cp[sat-1][freq]==0.0);
	else if (cccp<cp[sat-1][freq]-750.0) cccp+=1500.0;
	else if (cccp>cp[sat-1][freq]+750.0) cccp-=1500.0;
	cp[sat-1][freq]=cccp;
	return cccp;
}
/* loss-of-lock indicator ------------------------------------------------------------------------- */
int rtcm_3::lossoflock(int sat,int freq,int lllock){
	int lli=(!lllock&&!lock[sat-1][freq])||lllock<lock[sat-1][freq];
	lock[sat-1][freq]=(unsigned short)lllock;
	return lli;
}
/* s/n ratio -------------------------------------------------------------------------------------- */
unsigned char rtcm_3::snratio(double snr){
	return (unsigned char)(snr<=0.0||255.5<=snr ? 0.0 : snr*4.0+0.5);
}
/* test station id consistency -------------------------------------------------------------------- */
int rtcm_3::test_staid(int staid){
	size_t p;
	int type,id;

	/* test station id option */
	if ((p=opt.find("-STA="))!=string::npos&&str2int(opt.substr(p+5),id)==1) {
		if (staid!=id) return 0;
	}
	/* save station id */
	if (staid==0||obsflag) {
		staid=staid;
	}
	else if (staid!=staid) {
		type=getbitu(buff,24,12);

		/* reset station id if station id error */
		staid=0;
		return 0;
	}
	return 1;
}
/* get signed 38bit field ------------------------------------------------------------------------- */
double rtcm_3::getbits_38(const unsigned char *buff,int pos){
	return (double)getbits(buff,pos,32)*64.0+getbitu(buff,pos+32,6);
}
/* decode type 1001-1004 message header ----------------------------------------------------------- */
int rtcm_3::decode_head1001(int &sync){
	double tow;
	string str;
	int i=24,staid,nsat,type;

	type=getbitu(buff,i,12); i+=12;

	if (i+52<=len*8) {
		staid=getbitu(buff,i,12);       i+=12;
		tow  =getbitu(buff,i,30)*0.001; i+=30;
		sync=getbitu(buff,i,1);       i+= 1;
		nsat =getbitu(buff,i,5);
	}
	else return -1;
	/* test station id */
	if (!test_staid(staid)) return -1;

	adjweek(tow);

	if (outtype) {
		msgtype+=" "+time.time2str(2)+" nsat="+int2str(2," ",nsat,str)+" sync="+to_string(sync);
	}
	return nsat;
}
/* decode type 1001: L1-only gps rtk observation -------------------------------------------------- */
int rtcm_3::decode_type1001(){
	int sync;
	if (decode_head1001(sync)<0) return -1;
	obsflag=!sync;
	return sync ? 0 : 1;
}
/* decode type 1002: extended L1-only gps rtk observables ----------------------------------------- */
int rtcm_3::decode_type1002(){
	double pr1,cnr1,tt,cp1;
	int i=24+64,j,index,nsat,sync,prn,code,sat,ppr1,lock1,amb,sys;

	if ((nsat=decode_head1001(sync))<0) return -1;

	for (j=0; j<nsat&&obs.n<MAXOBS&&i+74<=len*8; j++) {
		prn  =getbitu(buff,i,6); i+= 6;
		code =getbitu(buff,i,1); i+= 1;
		pr1  =getbitu(buff,i,24); i+=24;
		ppr1 =getbits(buff,i,20); i+=20;
		lock1=getbitu(buff,i,7); i+= 7;
		amb  =getbitu(buff,i,8); i+= 8;
		cnr1 =getbitu(buff,i,8); i+= 8;
		if (prn<40) {
			sys=SYS_GPS;
		}
		else {
			sys=SYS_SBS; prn+=80;
		}
		if (!(sat=satno(sys,prn))) continue;
		tt=obs.data[0].time.timediff(time);
		if (obsflag||fabs(tt)>1E-9) {
			obs.n=obsflag=0;
		}
		if ((index=obsindex(time,sat))<0) continue;
		pr1=pr1*0.02+amb*PRUNIT_GPS;
		if (ppr1!=(int)0xFFF80000) {
			obs.data[index].P[0]=pr1;
			cp1=adjcp(sat,0,ppr1*0.0005/WaveLengths[0]);
			obs.data[index].L[0]=pr1/WaveLengths[0]+cp1;
		}
		obs.data[index].LLI[0]=lossoflock(sat,0,lock1);
		obs.data[index].SNR[0]=snratio(cnr1*0.25);
		obs.data[index].code[0]=code ? CODE_L1P : CODE_L1C;
	}
	return sync ? 0 : 1;
}
/* decode type 1003: L1&L2 gps rtk observables ---------------------------------------------------- */
int rtcm_3::decode_type1003(){
	int sync;
	if (decode_head1001(sync)<0) return -1;
	obsflag=!sync;
	return sync ? 0 : 1;
}
/* decode type 1004: extended L1&L2 gps rtk observables ------------------------------------------- */
int rtcm_3::decode_type1004(){
	const int L2codes[]={ CODE_L2X,CODE_L2P,CODE_L2D,CODE_L2W };
	double pr1,cnr1,cnr2,tt,cp1,cp2;
	int i=24+64,j,index,nsat,sync,prn,sat,code1,code2,pr21,ppr1,ppr2;
	int lock1,lock2,amb,sys;

	if ((nsat=decode_head1001(sync))<0) return -1;

	for (j=0; j<nsat&&obs.n<MAXOBS&&i+125<=len*8; j++) {
		prn  =getbitu(buff,i,6); i+= 6;
		code1=getbitu(buff,i,1); i+= 1;
		pr1  =getbitu(buff,i,24); i+=24;
		ppr1 =getbits(buff,i,20); i+=20;
		lock1=getbitu(buff,i,7); i+= 7;
		amb  =getbitu(buff,i,8); i+= 8;
		cnr1 =getbitu(buff,i,8); i+= 8;
		code2=getbitu(buff,i,2); i+= 2;
		pr21 =getbits(buff,i,14); i+=14;
		ppr2 =getbits(buff,i,20); i+=20;
		lock2=getbitu(buff,i,7); i+= 7;
		cnr2 =getbitu(buff,i,8); i+= 8;
		if (prn<40) {
			sys=SYS_GPS;
		}
		else {
			sys=SYS_SBS; prn+=80;
		}
		if (!(sat=satno(sys,prn))) {
			continue;
		}
		tt=obs.data[0].time.timediff(time);
		if (obsflag||fabs(tt)>1E-9) {
			obs.n=obsflag=0;
		}
		if ((index=obsindex(time,sat))<0) continue;
		pr1=pr1*0.02+amb*PRUNIT_GPS;
		if (ppr1!=(int)0xFFF80000) {
			obs.data[index].P[0]=pr1;
			cp1=adjcp(sat,0,ppr1*0.0005/WaveLengths[0]);
			obs.data[index].L[0]=pr1/WaveLengths[0]+cp1;
		}
		obs.data[index].LLI[0]=lossoflock(sat,0,lock1);
		obs.data[index].SNR[0]=snratio(cnr1*0.25);
		obs.data[index].code[0]=code1 ? CODE_L1P : CODE_L1C;

		if (pr21!=(int)0xFFFFE000) {
			obs.data[index].P[1]=pr1+pr21*0.02;
		}
		if (ppr2!=(int)0xFFF80000) {
			cp2=adjcp(sat,1,ppr2*0.0005/WaveLengths[1]);
			obs.data[index].L[1]=pr1/WaveLengths[1]+cp2;
		}
		obs.data[index].LLI[1]=lossoflock(sat,1,lock2);
		obs.data[index].SNR[1]=snratio(cnr2*0.25);
		obs.data[index].code[1]=L2codes[code2];
	}
	obsflag=!sync;
	return sync ? 0 : 1;
}
/* decode type 1005: stationary rtk reference station arp ----------------------------------------- */
int rtcm_3::decode_type1005(){
	double rr[3];
	string str;
	int i=24+12,j,staid,itrf;

	if (i+140==len*8) {
		staid=getbitu(buff,i,12); i+=12;
		itrf =getbitu(buff,i,6); i+= 6+4;
		rr[0]=getbits_38(buff,i); i+=38+2;
		rr[1]=getbits_38(buff,i); i+=38+2;
		rr[2]=getbits_38(buff,i);
	}
	else return -1;
	if (outtype) {
		msgtype+=" staid="+int2str(4," ",staid,str);
	}
	/* test station id */
	if (!test_staid(staid)) return -1;

	sta.deltype=0; /* xyz */
	for (j=0; j<3; j++) {
		sta.pos[j]=rr[j]*0.0001;
		sta.del[j]=0.0;
	}
	sta.hgt=0.0;
	sta.itrf=itrf;
	return 5;
}
/* decode type 1006: stationary rtk reference station arp with height ----------------------------- */
int rtcm_3::decode_type1006(){
	double rr[3],anth;
	string str;
	int i=24+12,j,staid,itrf;

	if (i+156<=len*8) {
		staid=getbitu(buff,i,12); i+=12;
		itrf =getbitu(buff,i,6); i+= 6+4;
		rr[0]=getbits_38(buff,i); i+=38+2;
		rr[1]=getbits_38(buff,i); i+=38+2;
		rr[2]=getbits_38(buff,i); i+=38;
		anth =getbitu(buff,i,16);
	}
	else return -1;
	if (outtype) {
		msgtype+=" staid="+int2str(4," ",staid,str);
	}
	/* test station id */
	if (!test_staid(staid)) return -1;

	sta.deltype=1; /* xyz */
	for (j=0; j<3; j++) {
		sta.pos[j]=rr[j]*0.0001;
		sta.del[j]=0.0;
	}
	sta.hgt=anth*0.0001;
	sta.itrf=itrf;
	return 5;
}
/* decode type 1007: antenna descriptor ----------------------------------------------------------- */
int rtcm_3::decode_type1007(){
	char des[32]="";
	string str;
	int i=24+12,j,staid,n,setup;

	n=getbitu(buff,i+12,8);

	if (i+28+8*n<=len*8) {
		staid=getbitu(buff,i,12); i+=12+8;
		for (j=0; j<n&&j<31; j++) {
			des[j]=(char)getbitu(buff,i,8); i+=8;
		}
		setup=getbitu(buff,i,8);
	}
	else return -1;
	if (outtype) {
		msgtype+=" staid="+int2str(4," ",staid,str);
	}
	/* test station id */
	if (!test_staid(staid)) return -1;

	sta.antdes=des; sta.antdes[n]='\0';
	sta.antsetup=setup;
	sta.antsno[0]='\0';
	return 5;
}
/* decode type 1008: antenna descriptor & serial number ------------------------------------------- */
int rtcm_3::decode_type1008(){
	char des[32]="",sno[32]="";
	string str;
	int i=24+12,j,staid,n,m,setup;

	n=getbitu(buff,i+12,8);
	m=getbitu(buff,i+28+8*n,8);

	if (i+36+8*(n+m)<=len*8) {
		staid=getbitu(buff,i,12); i+=12+8;
		for (j=0; j<n&&j<31; j++) {
			des[j]=(char)getbitu(buff,i,8); i+=8;
		}
		setup=getbitu(buff,i,8); i+=8+8;
		for (j=0; j<m&&j<31; j++) {
			sno[j]=(char)getbitu(buff,i,8); i+=8;
		}
	}
	else return -1;
	if (outtype) {
		msgtype+=" staid="+int2str(4," ",staid,str);
	}
	/* test station id */
	if (!test_staid(staid)) return -1;

	sta.antdes=des; sta.antdes[n]='\0';
	sta.antsetup=setup;
	sta.antsno=sno; sta.antsno[m]='\0';
	return 5;
}
/* decode type 1009-1012 message header ----------------------------------------------------------- */
int rtcm_3::decode_head1009(int &sync){
	double tod;
	string str;
	int i=24,staid,nsat,type;

	type=getbitu(buff,i,12); i+=12;

	if (i+49<=len*8) {
		staid=getbitu(buff,i,12);       i+=12;
		tod  =getbitu(buff,i,27)*0.001; i+=27; /* sec in a day */
		sync=getbitu(buff,i,1);			i+= 1;
		nsat =getbitu(buff,i,5);
	}
	else return -1;
	/* test station id */
	if (!test_staid(staid)) return -1;

	adjday_glot(tod);

	if (outtype) {
		msgtype+=" "+time.time2str(2) +" nsat="+int2str(2," ",nsat,str)+" sync="+to_string(sync);
	}
	return nsat;
}
/* decode type 1009: L1-only glonass rtk observables ---------------------------------------------- */
int rtcm_3::decode_type1009(){
	int sync;
	if (decode_head1009(sync)<0) return -1;
	obsflag=!sync;
	return sync ? 0 : 1;
}
/* decode type 1010: extended L1-only glonass rtk observables ------------------------------------- */
int rtcm_3::decode_type1010(){
	double pr1,cnr1,tt,cp1,lam1;
	int i=24+61,j,index,nsat,sync,prn,sat,code,freq,ppr1,lock1,amb,sys=SYS_GLO;

	if ((nsat=decode_head1009(sync))<0) return -1;

	for (j=0; j<nsat&&obs.n<MAXOBS&&i+79<=len*8; j++) {
		prn  =getbitu(buff,i,6); i+= 6;
		code =getbitu(buff,i,1); i+= 1;
		freq =getbitu(buff,i,5); i+= 5;
		pr1  =getbitu(buff,i,25); i+=25;
		ppr1 =getbits(buff,i,20); i+=20;
		lock1=getbitu(buff,i,7); i+= 7;
		amb  =getbitu(buff,i,7); i+= 7;
		cnr1 =getbitu(buff,i,8); i+= 8;
		if (!(sat=satno(sys,prn))) {
			continue;
		}
		tt=obs.data[0].time.timediff(time);
		if (obsflag||fabs(tt)>1E-9) {
			obs.n=obsflag=0;
		}
		if ((index=obsindex(time,sat))<0) continue;
		pr1=pr1*0.02+amb*PRUNIT_GLO;
		if (ppr1!=(int)0xFFF80000) {
			obs.data[index].P[0]=pr1;
			lam1=CLIGHT/(FREQ1_GLO+DFRQ1_GLO*(freq-7));
			cp1=adjcp(sat,0,ppr1*0.0005/lam1);
			obs.data[index].L[0]=pr1/lam1+cp1;
		}
		obs.data[index].LLI[0]=lossoflock(sat,0,lock1);
		obs.data[index].SNR[0]=snratio(cnr1*0.25);
		obs.data[index].code[0]=code ? CODE_L1P : CODE_L1C;
	}
	return sync ? 0 : 1;
}
/* decode type 1011: L1&L2 glonass rtk observables ------------------------------------------------ */
int rtcm_3::decode_type1011(){
	int sync;
	if (decode_head1009(sync)<0) return -1;
	obsflag=!sync;
	return sync ? 0 : 1;
}
/* decode type 1012: extended L1&L2 glonass rtk observables --------------------------------------- */
int rtcm_3::decode_type1012(){
	double pr1,cnr1,cnr2,tt,cp1,cp2,lam1,lam2;
	int i=24+61,j,index,nsat,sync,prn,sat,freq,code1,code2,pr21,ppr1,ppr2;
	int lock1,lock2,amb,sys=SYS_GLO;

	if ((nsat=decode_head1009(sync))<0) return -1;

	for (j=0; j<nsat&&obs.n<MAXOBS&&i+130<=len*8; j++) {
		prn  =getbitu(buff,i,6); i+= 6;
		code1=getbitu(buff,i,1); i+= 1;
		freq =getbitu(buff,i,5); i+= 5;
		pr1  =getbitu(buff,i,25); i+=25;
		ppr1 =getbits(buff,i,20); i+=20;
		lock1=getbitu(buff,i,7); i+= 7;
		amb  =getbitu(buff,i,7); i+= 7;
		cnr1 =getbitu(buff,i,8); i+= 8;
		code2=getbitu(buff,i,2); i+= 2;
		pr21 =getbits(buff,i,14); i+=14;
		ppr2 =getbits(buff,i,20); i+=20;
		lock2=getbitu(buff,i,7); i+= 7;
		cnr2 =getbitu(buff,i,8); i+= 8;
		if (!(sat=satno(sys,prn))) {
			continue;
		}
		tt=obs.data[0].time.timediff(time);
		if (obsflag||fabs(tt)>1E-9) {
			obs.n=obsflag=0;
		}
		if ((index=obsindex(time,sat))<0) continue;
		pr1=pr1*0.02+amb*PRUNIT_GLO;
		if (ppr1!=(int)0xFFF80000) {
			lam1=CLIGHT/(FREQ1_GLO+DFRQ1_GLO*(freq-7));
			obs.data[index].P[0]=pr1;
			cp1=adjcp(sat,0,ppr1*0.0005/lam1);
			obs.data[index].L[0]=pr1/lam1+cp1;
		}
		obs.data[index].LLI[0]=lossoflock(sat,0,lock1);
		obs.data[index].SNR[0]=snratio(cnr1*0.25);
		obs.data[index].code[0]=code1 ? CODE_L1P : CODE_L1C;

		if (pr21!=(int)0xFFFFE000) {
			obs.data[index].P[1]=pr1+pr21*0.02;
		}
		if (ppr2!=(int)0xFFF80000) {
			lam2=CLIGHT/(FREQ2_GLO+DFRQ2_GLO*(freq-7));
			cp2=adjcp(sat,1,ppr2*0.0005/lam2);
			obs.data[index].L[1]=pr1/lam2+cp2;
		}
		obs.data[index].LLI[1]=lossoflock(sat,1,lock2);
		obs.data[index].SNR[1]=snratio(cnr2*0.25);
		obs.data[index].code[1]=code2 ? CODE_L2P : CODE_L2C;
	}
	obsflag=!sync;
	return sync ? 0 : 1;
}
/* decode type 1013: system parameters ------------------------------------------------------------ */
int rtcm_3::decode_type1013(){
	return 0;
}
/* decode type 1019: gps ephemerides -------------------------------------------------------------- */
int rtcm_3::decode_type1019(){
	eph_t eph=eph_t();
	double toc,sqrtA;
	string str;
	int i=24+12,prn,sat,week,sys=SYS_GPS;

	if (i+476<=len*8) {
		prn       =getbitu(buff,i,6);               i+= 6;
		week      =getbitu(buff,i,10);              i+=10;
		eph.sva   =getbitu(buff,i,4);               i+= 4;
		eph.code  =getbitu(buff,i,2);               i+= 2;
		eph.idot  =getbits(buff,i,14)*P2_43*SC2RAD; i+=14;
		eph.iode  =getbitu(buff,i,8);               i+= 8;
		toc       =getbitu(buff,i,16)*16.0;         i+=16;
		eph.f2    =getbits(buff,i,8)*P2_55;         i+= 8;
		eph.f1    =getbits(buff,i,16)*P2_43;        i+=16;
		eph.f0    =getbits(buff,i,22)*P2_31;        i+=22;
		eph.iodc  =getbitu(buff,i,10);              i+=10;
		eph.crs   =getbits(buff,i,16)*P2_5;         i+=16;
		eph.deln  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
		eph.M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.cuc   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.e     =getbitu(buff,i,32)*P2_33;        i+=32;
		eph.cus   =getbits(buff,i,16)*P2_29;        i+=16;
		sqrtA     =getbitu(buff,i,32)*P2_19;        i+=32;
		eph.toes  =getbitu(buff,i,16)*16.0;         i+=16;
		eph.cic   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.cis   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.i0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.crc   =getbits(buff,i,16)*P2_5;         i+=16;
		eph.omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.OMGd  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
		eph.tgd[0]=getbits(buff,i,8)*P2_31;         i+= 8;
		eph.svh   =getbitu(buff,i,6);               i+= 6;
		eph.flag  =getbitu(buff,i,1);               i+= 1;
		eph.fit   =getbitu(buff,i,1) ? 0.0 : 4.0; /* 0:4hr,1:>4hr */
	}
	else return -1;
	if (prn>=40) {
		sys=SYS_SBS; prn+=80;
	}

	if (outtype) {
		msgtype+=" prn="+int2str(2," ",prn,str)+" iode="+int2str(3," ",eph.iode,str)+" iodc="
			+int2str(3," ",eph.iodc,str)+" week="+to_string(week)+" toe="+
			doul2str(6,0," ",eph.toes,str)+" toc="+doul2str(6,0," ",toc,str)+
			" svh="+int2str(2," ",eph.svh,str);
	}
	if (!(sat=satno(sys,prn))) return -1;
	eph.sat=sat;
	eph.week=adjgpsweek(week);
	eph.toe.gpst2time(eph.week,eph.toes);
	eph.toc.gpst2time(eph.week,toc);
	eph.ttr=time;
	eph.A=sqrtA*sqrtA;
	if (opt.find("-EPHALL")==string::npos) {
		if (eph.iode==nav.eph[sat-1].iode) return 0; /* unchanged */
	}
	nav.eph[sat-1]=eph;
	ephsat=sat;
	return 2;
}
/* decode type 1020: glonass ephemerides ---------------------------------------------------------- */
int rtcm_3::decode_type1020(){
	geph_t geph=geph_t();
	double tk_h,tk_m,tk_s,toe,tow,tod,tof;
	string str;
	int i=24+12,prn,sat,week,tb,bn,sys=SYS_GLO;

	if (i+348<=len*8) {
		prn        =getbitu(buff,i,6);           i+= 6;
		geph.frq   =getbitu(buff,i,5)-7;         i+= 5+2+2;
		tk_h       =getbitu(buff,i,5);           i+= 5;
		tk_m       =getbitu(buff,i,6);           i+= 6;
		tk_s       =getbitu(buff,i,1)*30.0;      i+= 1;
		bn         =getbitu(buff,i,1);           i+= 1+1;
		tb         =getbitu(buff,i,7);           i+= 7;
		geph.vel[0]=getbitg(i,24)*P2_20*1E3;     i+=24;
		geph.pos[0]=getbitg(i,27)*P2_11*1E3;     i+=27;
		geph.acc[0]=getbitg(i,5)*P2_30*1E3;      i+= 5;
		geph.vel[1]=getbitg(i,24)*P2_20*1E3;     i+=24;
		geph.pos[1]=getbitg(i,27)*P2_11*1E3;     i+=27;
		geph.acc[1]=getbitg(i,5)*P2_30*1E3;      i+= 5;
		geph.vel[2]=getbitg(i,24)*P2_20*1E3;     i+=24;
		geph.pos[2]=getbitg(i,27)*P2_11*1E3;     i+=27;
		geph.acc[2]=getbitg(i,5)*P2_30*1E3;      i+= 5+1;
		geph.gamn  =getbitg(i,11)*P2_40;         i+=11+3;
		geph.taun  =getbitg(i,22)*P2_30;
	}
	else return -1;
	if (!(sat=satno(sys,prn))) {
		return -1;
	}

	if (outtype) {
		msgtype+=" prn="+int2str(2," ",prn,str)+" tk="+doul2str(2,0,"0",tk_h,str)+":"+
			doul2str(2,0,"0",tk_m,str)+":"+doul2str(2,0,"0",tk_s,str)+" frq="+
			int2str(2," ",geph.frq,str)+" bn="+to_string(bn)+" tb="+to_string(tb);
	}
	geph.sat=sat;
	geph.svh=bn;
	geph.iode=tb&0x7F;
	if (time.time==0) time.timeget();
	tow=time.time2gpst(&week);
	time.utc2gpst();
	tod=fmod(tow,86400.0); tow-=tod;
	tof=tk_h*3600.0+tk_m*60.0+tk_s-10800.0; /* lt->utc */
	if (tof<tod-43200.0) tof+=86400.0;
	else if (tof>tod+43200.0) tof-=86400.0;
	geph.tof.gpst2time(week,tow+tof)->utc2gpst();
	toe=tb*900.0-10800.0; /* lt->utc */
	if (toe<tod-43200.0) toe+=86400.0;
	else if (toe>tod+43200.0) toe-=86400.0;
	geph.toe.gpst2time(week,tow+toe)->utc2gpst(); /* utc->gpst */

	if (opt.find("-EPHALL")==string::npos) {
		if (fabs(geph.toe.timediff(nav.geph[prn-1].toe))<1.0&&
			geph.svh==nav.geph[prn-1].svh) return 0; /* unchanged */
	}
	nav.geph[prn-1]=geph;
	ephsat=sat;
	return 2;
}
/* decode type 1021: helmert/abridged molodenski -------------------------------------------------- */
int rtcm_3::decode_type1021(){
	return 0;
}
/* decode type 1022: moledenski-badekas transfromation -------------------------------------------- */
int rtcm_3::decode_type1022(){
	return 0;
}
/* decode type 1023: residual, ellipoidal grid representation ------------------------------------- */
int rtcm_3::decode_type1023(){
	return 0;
}
/* decode type 1024: residual, plane grid representation ------------------------------------------ */
int rtcm_3::decode_type1024(){
	return 0;
}
/* decode type 1025: projection (types except LCC2SP,OM) ------------------------------------------ */
int rtcm_3::decode_type1025(){
	return 0;
}
/* decode type 1026: projection (LCC2SP - lambert conic conformal (2sp)) -------------------------- */
int rtcm_3::decode_type1026(){
	return 0;
}
/* decode type 1027: projection (type OM - oblique mercator) -------------------------------------- */
int rtcm_3::decode_type1027(){
	return 0;
}
/* decode type 1030: network rtk residual --------------------------------------------------------- */
int rtcm_3::decode_type1030(){
	return 0;
}
/* decode type 1031: glonass network rtk residual ------------------------------------------------- */
int rtcm_3::decode_type1031(){
	return 0;
}
/* decode type 1032: physical reference station position information ------------------------------ */
int rtcm_3::decode_type1032(){
	return 0;
}
/* decode type 1033: receiver and antenna descriptor ---------------------------------------------- */
int rtcm_3::decode_type1033(){
	char des[32]="",sno[32]="",rec[32]="",ver[32]="",rsn[32]="";
	string str;
	int i=24+12,j,staid,n,m,n1,n2,n3,setup;

	n =getbitu(buff,i+12,8);
	m =getbitu(buff,i+28+8*n,8);
	n1=getbitu(buff,i+36+8*(n+m),8);
	n2=getbitu(buff,i+44+8*(n+m+n1),8);
	n3=getbitu(buff,i+52+8*(n+m+n1+n2),8);

	if (i+60+8*(n+m+n1+n2+n3)<=len*8) {
		staid=getbitu(buff,i,12); i+=12+8;
		for (j=0; j<n&&j<31; j++) {
			des[j]=(char)getbitu(buff,i,8); i+=8;
		}
		setup=getbitu(buff,i,8); i+=8+8;
		for (j=0; j<m&&j<31; j++) {
			sno[j]=(char)getbitu(buff,i,8); i+=8;
		}
		i+=8;
		for (j=0; j<n1&&j<31; j++) {
			rec[j]=(char)getbitu(buff,i,8); i+=8;
		}
		i+=8;
		for (j=0; j<n2&&j<31; j++) {
			ver[j]=(char)getbitu(buff,i,8); i+=8;
		}
		i+=8;
		for (j=0; j<n3&&j<31; j++) {
			rsn[j]=(char)getbitu(buff,i,8); i+=8;
		}
	}
	else {
		return -1;
	}
	if (outtype) {
		msgtype+=" staid="+int2str(4," ",staid,str);
	}
	/* test station id */
	if (!test_staid(staid)) return -1;

	sta.antdes=des; sta.antdes[n] ='\0';
	sta.antsetup=setup;
	sta.antsno=sno; sta.antsno[m] ='\0';
	sta.rectype=rec; sta.rectype[n1]='\0';
	sta.recver=ver; sta.recver[n2]='\0';
	sta.recsno=rsn; sta.recsno[n3]='\0';
	return 5;
}
/* decode type 1034: gps network fkp gradient ----------------------------------------------------- */
int rtcm_3::decode_type1034(){
	return 0;
}
/* decode type 1035: glonass network fkp gradient ------------------------------------------------- */
int rtcm_3::decode_type1035(){
	return 0;
}
/* decode type 1037: glonass network rtk ionospheric correction difference ------------------------ */
int rtcm_3::decode_type1037(){
	return 0;
}
/* decode type 1038: glonass network rtk geometic correction difference --------------------------- */
int rtcm_3::decode_type1038(){
	return 0;
}
/* decode type 1039: glonass network rtk combined correction difference --------------------------- */
int rtcm_3::decode_type1039(){
	return 0;
}
/* decode type 1042: beidou ephemerides (tentative mt and format) --------------------------------- */
int rtcm_3::decode_type1042(){
	eph_t eph=eph_t();
	double toc,sqrtA;
	string str;
	int i=24+12,prn,sat,week,sys=SYS_CMP;

	if (i+500<=len*8) {
		prn       =getbitu(buff,i,6);               i+= 6;
		week      =getbitu(buff,i,13);              i+=13;
		eph.sva   =getbitu(buff,i,4);               i+= 4;
		eph.idot  =getbits(buff,i,14)*P2_43*SC2RAD; i+=14;
		eph.iode  =getbitu(buff,i,5);               i+= 5;
		toc       =getbitu(buff,i,17)*8.0;          i+=17;
		eph.f2    =getbits(buff,i,11)*P2_66;        i+=11;
		eph.f1    =getbits(buff,i,22)*P2_50;        i+=22;
		eph.f0    =getbits(buff,i,24)*P2_33;        i+=24;
		eph.iodc  =getbitu(buff,i,5);               i+=5;
		eph.crs   =getbits(buff,i,18)*P2_6;         i+=18;
		eph.deln  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
		eph.M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.cuc   =getbits(buff,i,18)*P2_31;        i+=18;
		eph.e     =getbitu(buff,i,32)*P2_33;        i+=32;
		eph.cus   =getbits(buff,i,18)*P2_31;        i+=18;
		sqrtA     =getbitu(buff,i,32)*P2_19;        i+=32;
		eph.toes  =getbitu(buff,i,17)*8.0;          i+=17;
		eph.cic   =getbits(buff,i,18)*P2_31;        i+=18;
		eph.OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.cis   =getbits(buff,i,18)*P2_31;        i+=18;
		eph.i0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.crc   =getbits(buff,i,18)*P2_6;         i+=18;
		eph.omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.OMGd  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
		eph.tgd[0]=getbits(buff,i,10)*1E-10;        i+=10;
		eph.tgd[1]=getbits(buff,i,10)*1E-10;        i+=10;
		eph.svh   =getbitu(buff,i,1);               i+= 1;
	}
	else return -1;

	if (outtype) {
		msgtype+=" prn="+int2str(2," ",prn,str)+" iode="+int2str(3," ",eph.iode,str)+" iodc="
			+int2str(3," ",eph.iodc,str)+" week="+to_string(week)+" toe="+
			doul2str(6,0," ",eph.toes,str)+" toc="+doul2str(6,0," ",toc,str)+
			" svh="+int2str(2," ",eph.svh,str);
	}
	if (!(sat=satno(sys,prn))) {
		return -1;
	}
	eph.sat=sat;
	eph.week=adjbdtweek(week,toc);
	if (eph.week==-1) return -1;
	eph.toe.bdt2time(eph.week,eph.toes)->bdt2gpst()->time2str(3); /* bdt -> gpst */
	eph.toc.bdt2time(eph.week,toc)->bdt2gpst()->time2str(3);      /* bdt -> gpst */
	eph.ttr=time;
	eph.A=sqrtA*sqrtA;
	if (opt.find("-EPHALL")==string::npos) {
		if (eph.iode==nav.eph[sat-1].iode) return 0; /* unchanged */
	}
	nav.eph[sat-1]=eph;
	ephsat=sat;
	return 2;
}
/* decode type 1044: qzss ephemerides (ref [15]) -------------------------------------------------- */
int rtcm_3::decode_type1044(){
	eph_t eph=eph_t();
	double toc,sqrtA;
	string str;
	int i=24+12,prn,sat,week,sys=SYS_QZS;

	if (i+473<=len*8) {
		prn       =getbitu(buff,i,4)+192;          i+= 4;
		toc       =getbitu(buff,i,16)*16.0;         i+=16;
		eph.f2    =getbits(buff,i,8)*P2_55;        i+= 8;
		eph.f1    =getbits(buff,i,16)*P2_43;        i+=16;
		eph.f0    =getbits(buff,i,22)*P2_31;        i+=22;
		eph.iode  =getbitu(buff,i,8);              i+= 8;
		eph.crs   =getbits(buff,i,16)*P2_5;         i+=16;
		eph.deln  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
		eph.M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.cuc   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.e     =getbitu(buff,i,32)*P2_33;        i+=32;
		eph.cus   =getbits(buff,i,16)*P2_29;        i+=16;
		sqrtA     =getbitu(buff,i,32)*P2_19;        i+=32;
		eph.toes  =getbitu(buff,i,16)*16.0;         i+=16;
		eph.cic   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.cis   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.i0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.crc   =getbits(buff,i,16)*P2_5;         i+=16;
		eph.omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.OMGd  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
		eph.idot  =getbits(buff,i,14)*P2_43*SC2RAD; i+=14;
		eph.code  =getbitu(buff,i,2);              i+= 2;
		week      =getbitu(buff,i,10);              i+=10;
		eph.sva   =getbitu(buff,i,4);              i+= 4;
		eph.svh   =getbitu(buff,i,6);              i+= 6;
		eph.tgd[0]=getbits(buff,i,8)*P2_31;        i+= 8;
		eph.iodc  =getbitu(buff,i,10);              i+=10;
		eph.fit   =getbitu(buff,i,1) ? 0.0 : 2.0; /* 0:2hr,1:>2hr */
	}
	else return -1;

	if (outtype) {
		msgtype+=" prn="+int2str(2," ",prn,str)+" iode="+int2str(3," ",eph.iode,str)+" iodc="
			+int2str(3," ",eph.iodc,str)+" week="+to_string(week)+" toe="+
			doul2str(6,0," ",eph.toes,str)+" toc="+doul2str(6,0," ",toc,str)+
			" svh="+int2str(2," ",eph.svh,str);
	}
	if (!(sat=satno(sys,prn))) {
		return -1;
	}
	eph.sat=sat;
	eph.week=adjgpsweek(week);
	eph.toe.gpst2time(eph.week,eph.toes);
	eph.toc.gpst2time(eph.week,toc);
	eph.ttr=time;
	eph.A=sqrtA*sqrtA;
	if (opt.find("-EPHALL")==string::npos) {
		if (eph.iode==nav.eph[sat-1].iode&&
			eph.iodc==nav.eph[sat-1].iodc) return 0; /* unchanged */
	}
	nav.eph[sat-1]=eph;
	ephsat=sat;
	return 2;
}
/* decode type 1045: galileo satellite ephemerides (ref [15]) ------------------------------------- */
int rtcm_3::decode_type1045(){
	eph_t eph=eph_t();
	double toc,sqrtA;
	string str;
	int i=24+12,prn,sat,week,e5a_hs,e5a_dvs,rsv,sys=SYS_GAL;

	if (i+484<=len*8) {
		prn       =getbitu(buff,i,6);              i+= 6;
		week      =getbitu(buff,i,12);              i+=12;
		eph.iode  =getbitu(buff,i,10);              i+=10;
		eph.sva   =getbitu(buff,i,8);              i+= 8;
		eph.idot  =getbits(buff,i,14)*P2_43*SC2RAD; i+=14;
		toc       =getbitu(buff,i,14)*60.0;         i+=14;
		eph.f2    =getbits(buff,i,6)*P2_59;        i+= 6;
		eph.f1    =getbits(buff,i,21)*P2_46;        i+=21;
		eph.f0    =getbits(buff,i,31)*P2_34;        i+=31;
		eph.crs   =getbits(buff,i,16)*P2_5;         i+=16;
		eph.deln  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
		eph.M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.cuc   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.e     =getbitu(buff,i,32)*P2_33;        i+=32;
		eph.cus   =getbits(buff,i,16)*P2_29;        i+=16;
		sqrtA     =getbitu(buff,i,32)*P2_19;        i+=32;
		eph.toes  =getbitu(buff,i,14)*60.0;         i+=14;
		eph.cic   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.cis   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.i0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.crc   =getbits(buff,i,16)*P2_5;         i+=16;
		eph.omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.OMGd  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
		eph.tgd[0]=getbits(buff,i,10)*P2_32;        i+=10; /* E5a/E1 */
		e5a_hs    =getbitu(buff,i,2);              i+= 2; /* OSHS */
		e5a_dvs   =getbitu(buff,i,1);              i+= 1; /* OSDVS */
		rsv       =getbitu(buff,i,7);
	}
	else return -1;

	if (outtype) {
		msgtype+=" prn="+int2str(2," ",prn,str)+" iode="+int2str(3," ",eph.iode,str)+
			" week="+to_string(week)+" toe="+doul2str(6,0," ",eph.toes,str)+
			" toc="+doul2str(6,0," ",toc,str)+" hs="+to_string(e5a_hs)+" dvs"+
			to_string(e5a_dvs);
	}
	if (!(sat=satno(sys,prn))) {
		return -1;
	}
	eph.sat=sat;
	eph.week=week;
	eph.toe.gst2time(eph.week,eph.toes);
	eph.toc.gst2time(eph.week,toc);
	eph.ttr=time;
	eph.A=sqrtA*sqrtA;
	eph.svh=(e5a_hs<<4)+(e5a_dvs<<3);
	eph.code=2; /* data source = f/nav e5a */
	if (opt.find("-EPHALL")==string::npos) {
		if (eph.iode==nav.eph[sat-1].iode) return 0; /* unchanged */
	}
	nav.eph[sat-1]=eph;
	ephsat=sat;
	return 2;
}
/* decode type 1046: galileo satellite ephemerides (extension for IGS MGEX) ----------------------- */
int rtcm_3::decode_type1046(){
	eph_t eph=eph_t();
	double toc,sqrtA;
	string str;
	int i=24+12,prn,sat,week,e5a_hs,e5a_dvs,rsv,sys=SYS_GAL;

	if (i+484<=len*8) {
		prn       =getbitu(buff,i,6);              i+= 6;
		week      =getbitu(buff,i,12);              i+=12;
		eph.iode  =getbitu(buff,i,10);              i+=10;
		eph.sva   =getbitu(buff,i,8);              i+= 8;
		eph.idot  =getbits(buff,i,14)*P2_43*SC2RAD; i+=14;
		toc       =getbitu(buff,i,14)*60.0;         i+=14;
		eph.f2    =getbits(buff,i,6)*P2_59;        i+= 6;
		eph.f1    =getbits(buff,i,21)*P2_46;        i+=21;
		eph.f0    =getbits(buff,i,31)*P2_34;        i+=31;
		eph.crs   =getbits(buff,i,16)*P2_5;         i+=16;
		eph.deln  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
		eph.M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.cuc   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.e     =getbitu(buff,i,32)*P2_33;        i+=32;
		eph.cus   =getbits(buff,i,16)*P2_29;        i+=16;
		sqrtA     =getbitu(buff,i,32)*P2_19;        i+=32;
		eph.toes  =getbitu(buff,i,14)*60.0;         i+=14;
		eph.cic   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.cis   =getbits(buff,i,16)*P2_29;        i+=16;
		eph.i0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.crc   =getbits(buff,i,16)*P2_5;         i+=16;
		eph.omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.OMGd  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
		eph.tgd[0]=getbits(buff,i,10)*P2_32;        i+=10; /* E5a/E1 */
		e5a_hs    =getbitu(buff,i,2);              i+= 2; /* OSHS */
		e5a_dvs   =getbitu(buff,i,1);              i+= 1; /* OSDVS */
		rsv       =getbitu(buff,i,7);
	}
	else return -1;

	if (outtype) {
		msgtype+=" prn="+int2str(2," ",prn,str)+" iode="+int2str(3," ",eph.iode,str)+
			" week="+to_string(week)+" toe="+doul2str(6,0," ",eph.toes,str)+
			" toc="+doul2str(6,0," ",toc,str)+" hs="+to_string(e5a_hs)+" dvs"+
			to_string(e5a_dvs);
	}
	if (!(sat=satno(sys,prn))) {
		return -1;
	}
	eph.sat=sat;
	eph.week=week;
	eph.toe.gst2time(eph.week,eph.toes);
	eph.toc.gst2time(eph.week,toc);
	eph.ttr=time;
	eph.A=sqrtA*sqrtA;
	eph.svh=(e5a_hs<<4)+(e5a_dvs<<3);
	eph.code=2; /* data source = f/nav e5a */
	if (opt.find("-EPHALL")==string::npos) {
		if (eph.iode==nav.eph[sat-1].iode) return 0; /* unchanged */
	}
	nav.eph[sat-1]=eph;
	ephsat=sat;
	return 2;
}
/* decode type 63: beidou ephemerides (rtcm draft) ------------------------------------------------ */
int rtcm_3::decode_type63(){
	eph_t eph=eph_t();
	double toc,sqrtA;
	string str;
	int i=24+12,prn,sat,week,sys=SYS_CMP;

	if (i+499<=len*8) {
		prn       =getbitu(buff,i,6);               i+= 6;
		week      =getbitu(buff,i,13);              i+=13;
		eph.sva   =getbitu(buff,i,4);               i+= 4;
		eph.idot  =getbits(buff,i,14)*P2_43*SC2RAD; i+=14;
		eph.iode  =getbitu(buff,i,5);               i+= 5; /* AODE */
		toc       =getbitu(buff,i,17)*8.0;          i+=17;
		eph.f2    =getbits(buff,i,11)*P2_66;        i+=11;
		eph.f1    =getbits(buff,i,22)*P2_50;        i+=22;
		eph.f0    =getbits(buff,i,24)*P2_33;        i+=24;
		eph.iodc  =getbitu(buff,i,5);               i+= 5; /* AODC */
		eph.crs   =getbits(buff,i,18)*P2_6;         i+=18;
		eph.deln  =getbits(buff,i,16)*P2_43*SC2RAD; i+=16;
		eph.M0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.cuc   =getbits(buff,i,18)*P2_31;        i+=18;
		eph.e     =getbitu(buff,i,32)*P2_33;        i+=32;
		eph.cus   =getbits(buff,i,18)*P2_31;        i+=18;
		sqrtA     =getbitu(buff,i,32)*P2_19;        i+=32;
		eph.toes  =getbitu(buff,i,17)*8.0;          i+=17;
		eph.cic   =getbits(buff,i,18)*P2_31;        i+=18;
		eph.OMG0  =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.cis   =getbits(buff,i,18)*P2_31;        i+=18;
		eph.i0    =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.crc   =getbits(buff,i,18)*P2_6;         i+=18;
		eph.omg   =getbits(buff,i,32)*P2_31*SC2RAD; i+=32;
		eph.OMGd  =getbits(buff,i,24)*P2_43*SC2RAD; i+=24;
		eph.tgd[0]=getbits(buff,i,10)*1E-10;        i+=10;
		eph.tgd[1]=getbits(buff,i,10)*1E-10;        i+=10;
		eph.svh   =getbitu(buff,i,1);               i+= 1;
	}
	else return -1;

	if (outtype) {
		msgtype+=" prn="+int2str(2," ",prn,str)+" iode="+int2str(3," ",eph.iode,str)+" iodc="
			+int2str(3," ",eph.iodc,str)+" week="+to_string(week)+" toe="+
			doul2str(6,0," ",eph.toes,str)+" toc="+doul2str(6,0," ",toc,str)+
			" svh="+int2str(2," ",eph.svh,str);
	}
	if (!(sat=satno(sys,prn))) {
		return -1;
	}
	eph.sat=sat;
	eph.week=adjbdtweek(week,toc);
	if (eph.week==-1) return -1;
	eph.toe.bdt2time(eph.week,eph.toes)->bdt2gpst(); /* bdt -> gpst */
	eph.toc.bdt2time(eph.week,toc)->bdt2gpst();      /* bdt -> gpst */
	eph.ttr=time;
	eph.A=sqrtA*sqrtA;
	if (opt.find("-EPHALL")==string::npos) {
		if (eph.iode==nav.eph[sat-1].iode) return 0; /* unchanged */
	}
	nav.eph[sat-1]=eph;
	ephsat=sat;
	return 2;
}
/* decode ssr 1,4 message header ------------------------------------------------------------------ */
int rtcm_3::decode_ssr1_head(int sys,int &sync,int &iod,double &udint,int &refd,int &hsize){
	double tod,tow;
	string str;
	int i=24+12,nsat,udi,provid=0,solid=0,ns=6;

#ifndef SSR_QZSS_DRAFT_V05
	ns=sys==SYS_QZS ? 4 : 6;
#endif
	if (i+(sys==SYS_GLO ? 53 : 50+ns)>len*8) return -1;

	if (sys==SYS_GLO) {
		tod=getbitu(buff,i,17); i+=17;
		adjday_glot(tod);
	}
	else {
		tow=getbitu(buff,i,20); i+=20;
		adjweek(tow);
	}
	udi   =getbitu(buff,i,4); i+= 4;
	sync =getbitu(buff,i,1); i+= 1;
	refd =getbitu(buff,i,1); i+= 1; /* satellite ref datum */
	iod  =getbitu(buff,i,4); i+= 4; /* iod */
	provid=getbitu(buff,i,16); i+=16; /* provider id */
	solid =getbitu(buff,i,4); i+= 4; /* solution id */
	nsat  =getbitu(buff,i,ns); i+=ns;
	udint=ssrudint[udi];

	if (outtype) {
		msgtype+=" "+time.time2str(2)+" nsat="+int2str(2," ",nsat,str)+" sync="+to_string(sync)+
			" iod="+int2str(2," ",iod,str)+" udi="+int2str(2," ",udi,str)+" sync="+
			to_string(sync);
	}
	hsize=i;
	return nsat;
}
/* decode ssr 2,3,5,6 message header -------------------------------------------------------------- */
int rtcm_3::decode_ssr2_head(int sys,int &sync,int &iod,double &udint,int &hsize){
	double tod,tow;
	string str;
	int i=24+12,nsat,udi,provid=0,solid=0,ns=6;

#ifndef SSR_QZSS_DRAFT_V05
	ns=sys==SYS_QZS ? 4 : 6;
#endif
	if (i+(sys==SYS_GLO ? 52 : 49+ns)>len*8) return -1;

	if (sys==SYS_GLO) {
		tod=getbitu(buff,i,17); i+=17;
		adjday_glot(tod);
	}
	else {
		tow=getbitu(buff,i,20); i+=20;
		adjweek(tow);
	}
	udi   =getbitu(buff,i,4); i+= 4;
	sync =getbitu(buff,i,1); i+= 1;
	iod  =getbitu(buff,i,4); i+= 4;
	provid=getbitu(buff,i,16); i+=16; /* provider id */
	solid =getbitu(buff,i,4); i+= 4; /* solution id */
	nsat  =getbitu(buff,i,ns); i+=ns;
	udint=ssrudint[udi];

	if (outtype) {
		msgtype+=" "+time.time2str(2)+" nsat="+int2str(2," ",nsat,str)+" sync="+to_string(sync)+
			" iod="+int2str(2," ",iod,str)+" udi="+int2str(2," ",udi,str)+" sync="+
			to_string(sync);
	}
	hsize=i;
	return nsat;
}
/* decode ssr 7 message header ------------------------------------------------------------ */
int rtcm_3::decode_ssr7_head(int sys,int &sync,int &iod,double &udint,int &dispe,int &mw,int &hsize){
	double tod,tow;
	string str;
	int i=24+12,nsat,udi,provid=0,solid=0,ns=6;

#ifndef SSR_QZSS_DRAFT_V05
	ns=sys==SYS_QZS ? 4 : 6;
#endif
	if (i+(sys==SYS_GLO ? 54 : 51+ns)>len*8) return -1;

	if (sys==SYS_GLO) {
		tod=getbitu(buff,i,17); i+=17;
		adjday_glot(tod);
	}
	else {
		tow=getbitu(buff,i,20); i+=20;
		adjweek(tow);
	}
	udi   =getbitu(buff,i,4); i+= 4;
	sync =getbitu(buff,i,1); i+= 1;
	iod  =getbitu(buff,i,4); i+= 4;
	provid=getbitu(buff,i,16); i+=16; /* provider id */
	solid =getbitu(buff,i,4); i+= 4; /* solution id */
	dispe=getbitu(buff,i,1); i+= 1; /* dispersive bias consistency ind */
	mw   =getbitu(buff,i,1); i+= 1; /* MW consistency indicator */
	nsat  =getbitu(buff,i,ns); i+=ns;
	udint=ssrudint[udi];

	if (outtype) {
		msgtype+=" "+time.time2str(2)+" nsat="+int2str(2," ",nsat,str)+" sync="+to_string(sync)+
			" iod="+int2str(2," ",iod,str)+" udi="+int2str(2," ",udi,str)+" sync="+
			to_string(sync);
	}
	hsize=i;
	return nsat;
}
/* decode ssr 1: orbit corrections ---------------------------------------------------------------- */
int rtcm_3::decode_ssr1(int sys){
	double udint,deph[3],ddeph[3];
	int i,j,k,type,sync,iod,nsat,prn,sat,iode,iodcrc,refd=0,np,ni,nj,offp;

	type=getbitu(buff,24,12);

	if ((nsat=decode_ssr1_head(sys,sync,iod,udint,refd,i))<0) {
		return -1;
	}
	switch (sys) {
	case SYS_GPS: np=6; ni= 8; nj= 0; offp=  0; break;
	case SYS_GLO: np=5; ni= 8; nj= 0; offp=  0; break;
	case SYS_GAL: np=6; ni=10; nj= 0; offp=  0; break;
	case SYS_QZS: np=4; ni= 8; nj= 0; offp=192; break;
	case SYS_CMP: np=6; ni=10; nj=24; offp=  1; break;
	case SYS_SBS: np=6; ni= 9; nj=24; offp=120; break;
	default: return sync ? 0 : 10;
	}
	for (j=0; j<nsat&&i+121+np+ni+nj<=len*8; j++) {
		prn     =getbitu(buff,i,np)+offp; i+=np;
		iode    =getbitu(buff,i,ni);      i+=ni;
		iodcrc  =getbitu(buff,i,nj);      i+=nj;
		deph[0]=getbits(buff,i,22)*1E-4; i+=22;
		deph[1]=getbits(buff,i,20)*4E-4; i+=20;
		deph[2]=getbits(buff,i,20)*4E-4; i+=20;
		ddeph[0]=getbits(buff,i,21)*1E-6; i+=21;
		ddeph[1]=getbits(buff,i,19)*4E-6; i+=19;
		ddeph[2]=getbits(buff,i,19)*4E-6; i+=19;

		if (!(sat=satno(sys,prn))) {
			continue;
		}
		ssr[sat-1].t0[0]=time;
		ssr[sat-1].udi[0]=udint;
		ssr[sat-1].iod[0]=iod;
		ssr[sat-1].iode=iode;     /* sbas/bds: toe/t0 modulo */
		ssr[sat-1].iodcrc=iodcrc; /* sbas/bds: iod crc */
		ssr[sat-1].refd=refd;

		for (k=0; k<3; k++) {
			ssr[sat-1].deph[k]=deph[k];
			ssr[sat-1].ddeph[k]=ddeph[k];
		}
		ssr[sat-1].update=1;
	}
	return sync ? 0 : 10;
}
/* decode ssr 2: clock corrections ---------------------------------------------------------------- */
int rtcm_3::decode_ssr2(int sys){
	double udint,dclk[3];
	int i,j,k,type,sync,iod,nsat,prn,sat,np,offp;

	type=getbitu(buff,24,12);

	if ((nsat=decode_ssr2_head(sys,sync,iod,udint,i))<0) {
		return -1;
	}
	switch (sys) {
	case SYS_GPS: np=6; offp=  0; break;
	case SYS_GLO: np=5; offp=  0; break;
	case SYS_GAL: np=6; offp=  0; break;
	case SYS_QZS: np=4; offp=192; break;
	case SYS_CMP: np=6; offp=  1; break;
	case SYS_SBS: np=6; offp=120; break;
	default: return sync ? 0 : 10;
	}
	for (j=0; j<nsat&&i+70+np<=len*8; j++) {
		prn    =getbitu(buff,i,np)+offp; i+=np;
		dclk[0]=getbits(buff,i,22)*1E-4; i+=22;
		dclk[1]=getbits(buff,i,21)*1E-6; i+=21;
		dclk[2]=getbits(buff,i,27)*2E-8; i+=27;

		if (!(sat=satno(sys,prn))) {
			continue;
		}
		ssr[sat-1].t0[1]=time;
		ssr[sat-1].udi[1]=udint;
		ssr[sat-1].iod[1]=iod;

		for (k=0; k<3; k++) {
			ssr[sat-1].dclk[k]=dclk[k];
		}
		ssr[sat-1].update=1;
	}
	return sync ? 0 : 10;
}
/* decode ssr 3: satellite code biases ------------------------------------------------------------ */
int rtcm_3::decode_ssr3(int sys){
	const int *codes;
	double udint,bias,cbias[MAXCODE];
	int i,j,k,type,mode,sync,iod,nsat,prn,sat,nbias,np,offp,ncode;

	type=getbitu(buff,24,12);

	if ((nsat=decode_ssr2_head(sys,sync,iod,udint,i))<0) {
		return -1;
	}
	switch (sys) {
	case SYS_GPS: np=6; offp=  0; codes=codes_gps; ncode=17; break;
	case SYS_GLO: np=5; offp=  0; codes=codes_glo; ncode= 4; break;
	case SYS_GAL: np=6; offp=  0; codes=codes_gal; ncode=19; break;
	case SYS_QZS: np=4; offp=192; codes=codes_qzs; ncode=13; break;
	case SYS_CMP: np=6; offp=  1; codes=codes_bds; ncode= 9; break;
	case SYS_SBS: np=6; offp=120; codes=codes_sbs; ncode= 4; break;
	default: return sync ? 0 : 10;
	}
	for (j=0; j<nsat&&i+5+np<=len*8; j++) {
		prn  =getbitu(buff,i,np)+offp; i+=np;
		nbias=getbitu(buff,i,5);      i+= 5;

		for (k=0; k<MAXCODE; k++) cbias[k]=0.0;
		for (k=0; k<nbias&&i+19<=len*8; k++) {
			mode=getbitu(buff,i,5);      i+= 5;
			bias=getbits(buff,i,14)*0.01; i+=14;
			if (mode<=ncode) {
				cbias[codes[mode]-1]=(float)bias;
			}
			else; /* not supported */
		}
		if (!(sat=satno(sys,prn))) {
			continue;
		}
		ssr[sat-1].t0[4]=time;
		ssr[sat-1].udi[4]=udint;
		ssr[sat-1].iod[4]=iod;

		for (k=0; k<MAXCODE; k++) {
			ssr[sat-1].cbias[k]=(float)cbias[k];
		}
		ssr[sat-1].update=1;
	}
	return sync ? 0 : 10;
}
/* decode ssr 4: combined orbit and clock corrections --------------------------------------------- */
int rtcm_3::decode_ssr4(int sys){
	double udint,deph[3],ddeph[3],dclk[3];
	int i,j,k,type,nsat,sync,iod,prn,sat,iode,iodcrc,refd=0,np,ni,nj,offp;

	type=getbitu(buff,24,12);

	if ((nsat=decode_ssr1_head(sys,sync,iod,udint,refd,i))<0) {
		return -1;
	}
	switch (sys) {
	case SYS_GPS: np=6; ni= 8; nj= 0; offp=  0; break;
	case SYS_GLO: np=5; ni= 8; nj= 0; offp=  0; break;
	case SYS_GAL: np=6; ni=10; nj= 0; offp=  0; break;
	case SYS_QZS: np=4; ni= 8; nj= 0; offp=192; break;
	case SYS_CMP: np=6; ni=10; nj=24; offp=  1; break;
	case SYS_SBS: np=6; ni= 9; nj=24; offp=120; break;
	default: return sync ? 0 : 10;
	}
	for (j=0; j<nsat&&i+191+np+ni+nj<=len*8; j++) {
		prn     =getbitu(buff,i,np)+offp; i+=np;
		iode    =getbitu(buff,i,ni);      i+=ni;
		iodcrc  =getbitu(buff,i,nj);      i+=nj;
		deph[0]=getbits(buff,i,22)*1E-4; i+=22;
		deph[1]=getbits(buff,i,20)*4E-4; i+=20;
		deph[2]=getbits(buff,i,20)*4E-4; i+=20;
		ddeph[0]=getbits(buff,i,21)*1E-6; i+=21;
		ddeph[1]=getbits(buff,i,19)*4E-6; i+=19;
		ddeph[2]=getbits(buff,i,19)*4E-6; i+=19;

		dclk[0]=getbits(buff,i,22)*1E-4; i+=22;
		dclk[1]=getbits(buff,i,21)*1E-6; i+=21;
		dclk[2]=getbits(buff,i,27)*2E-8; i+=27;

		if (!(sat=satno(sys,prn))) {
			continue;
		}
		ssr[sat-1].t0[0]=ssr[sat-1].t0[1]=time;
		ssr[sat-1].udi[0]=ssr[sat-1].udi[1]=udint;
		ssr[sat-1].iod[0]=ssr[sat-1].iod[1]=iod;
		ssr[sat-1].iode=iode;
		ssr[sat-1].iodcrc=iodcrc;
		ssr[sat-1].refd=refd;

		for (k=0; k<3; k++) {
			ssr[sat-1].deph[k]=deph[k];
			ssr[sat-1].ddeph[k]=ddeph[k];
			ssr[sat-1].dclk[k]=dclk[k];
		}
		ssr[sat-1].update=1;
	}
	return sync ? 0 : 10;
}
/* decode ssr 5: ura ------------------------------------------------------------------------------ */
int rtcm_3::decode_ssr5(int sys){
	double udint;
	int i,j,type,nsat,sync,iod,prn,sat,ura,np,offp;

	type=getbitu(buff,24,12);

	if ((nsat=decode_ssr2_head(sys,sync,iod,udint,i))<0) {
		return -1;
	}
	switch (sys) {
	case SYS_GPS: np=6; offp=  0; break;
	case SYS_GLO: np=5; offp=  0; break;
	case SYS_GAL: np=6; offp=  0; break;
	case SYS_QZS: np=4; offp=192; break;
	case SYS_CMP: np=6; offp=  1; break;
	case SYS_SBS: np=6; offp=120; break;
	default: return sync ? 0 : 10;
	}
	for (j=0; j<nsat&&i+6+np<=len*8; j++) {
		prn=getbitu(buff,i,np)+offp; i+=np;
		ura=getbitu(buff,i,6);      i+= 6;

		if (!(sat=satno(sys,prn))) {
			continue;
		}
		ssr[sat-1].t0[3]=time;
		ssr[sat-1].udi[3]=udint;
		ssr[sat-1].iod[3]=iod;
		ssr[sat-1].ura=ura;
		ssr[sat-1].update=1;
	}
	return sync ? 0 : 10;
}
/* decode ssr 6: high rate clock correction ------------------------------------------------------- */
int rtcm_3::decode_ssr6(int sys){
	double udint,hrclk;
	int i,j,type,nsat,sync,iod,prn,sat,np,offp;

	type=getbitu(buff,24,12);

	if ((nsat=decode_ssr2_head(sys,sync,iod,udint,i))<0) {
		return -1;
	}
	switch (sys) {
	case SYS_GPS: np=6; offp=  0; break;
	case SYS_GLO: np=5; offp=  0; break;
	case SYS_GAL: np=6; offp=  0; break;
	case SYS_QZS: np=4; offp=192; break;
	case SYS_CMP: np=6; offp=  1; break;
	case SYS_SBS: np=6; offp=120; break;
	default: return sync ? 0 : 10;
	}
	for (j=0; j<nsat&&i+22+np<=len*8; j++) {
		prn  =getbitu(buff,i,np)+offp; i+=np;
		hrclk=getbits(buff,i,22)*1E-4; i+=22;

		if (!(sat=satno(sys,prn))) {
			continue;
		}
		ssr[sat-1].t0[2]=time;
		ssr[sat-1].udi[2]=udint;
		ssr[sat-1].iod[2]=iod;
		ssr[sat-1].hrclk=hrclk;
		ssr[sat-1].update=1;
	}
	return sync ? 0 : 10;
}
/* decode ssr 7: phase bias ----------------------------------------------------------------------- */
int rtcm_3::decode_ssr7(int sys){
	const int *codes;
	double udint,bias,std,pbias[MAXCODE],stdpb[MAXCODE];
	int i,j,k,type,mode,sync,iod,nsat,prn,sat,nbias,ncode,np,mw,offp,sii,swl;
	int dispe,sdc,yaw_ang,yaw_rate;

	type=getbitu(buff,24,12);

	if ((nsat=decode_ssr7_head(sys,sync,iod,udint,dispe,mw,i))<0) {
		return -1;
	}
	switch (sys) {
	case SYS_GPS: np=6; offp=  0; codes=codes_gps; ncode=17; break;
	case SYS_GLO: np=5; offp=  0; codes=codes_glo; ncode= 4; break;
	case SYS_GAL: np=6; offp=  0; codes=codes_gal; ncode=19; break;
	case SYS_QZS: np=4; offp=192; codes=codes_qzs; ncode=13; break;
	case SYS_CMP: np=6; offp=  1; codes=codes_bds; ncode= 9; break;
	default: return sync ? 0 : 10;
	}
	for (j=0; j<nsat&&i+5+17+np<=len*8; j++) {
		prn     =getbitu(buff,i,np)+offp; i+=np;
		nbias   =getbitu(buff,i,5);      i+= 5;
		yaw_ang =getbitu(buff,i,9);      i+= 9;
		yaw_rate=getbits(buff,i,8);      i+= 8;

		for (k=0; k<MAXCODE; k++) pbias[k]=stdpb[k]=0.0;
		for (k=0; k<nbias&&i+49<=len*8; k++) {
			mode=getbitu(buff,i,5); i+= 5;
			sii =getbitu(buff,i,1); i+= 1; /* integer-indicator */
			swl =getbitu(buff,i,2); i+= 2; /* WL integer-indicator */
			sdc =getbitu(buff,i,4); i+= 4; /* discontinuity counter */
			bias=getbits(buff,i,20); i+=20; /* phase bias (m) */
			std =getbitu(buff,i,17); i+=17; /* phase bias std-dev (m) */
			if (mode<=ncode) {
				pbias[codes[mode]-1]=bias*0.0001; /* (m) */
				stdpb[codes[mode]-1]=std *0.0001; /* (m) */
			}
			else; /* not supported */
		}
		if (!(sat=satno(sys,prn))) {
			continue;
		}
		ssr[sat-1].t0[5]=time;
		ssr[sat-1].udi[5]=udint;
		ssr[sat-1].iod[5]=iod;
		ssr[sat-1].yaw_ang =yaw_ang / 256.0*180.0; /* (deg) */
		ssr[sat-1].yaw_rate=yaw_rate/8192.0*180.0; /* (deg/s) */

		for (k=0; k<MAXCODE; k++) {
			ssr[sat-1].pbias[k]=pbias[k];
			ssr[sat-1].stdpb[k]=(float)stdpb[k];
		}
	}
	return 20;
}
/* get signal index ------------------------------------------------------------------------------- */
void rtcm_3::sigindex(int sys,const unsigned char *code,const int *freq,int n,int *ind){
	int i,nex,pri,pri_h[8]={ 0 },index[8]={ 0 },ex[32]={ 0 };

	/* test code priority */
	for (i=0; i<n; i++) {
		if (!code[i]) continue;

		if (freq[i]>NFREQ) { /* save as extended signal if freq > NFREQ */
			ex[i]=1;
			continue;
		}
		/* code priority */
		pri=getcodepri(sys,code[i],opt);

		/* select highest priority signal */
		if (pri>pri_h[freq[i]-1]) {
			if (index[freq[i]-1]) ex[index[freq[i]-1]-1]=1;
			pri_h[freq[i]-1]=pri;
			index[freq[i]-1]=i+1;
		}
		else ex[i]=1;
	}
	/* signal index in obs data */
	for (i=nex=0; i<n; i++) {
		if (ex[i]==0) ind[i]=freq[i]-1;
		else if (nex<NEXOBS) ind[i]=NFREQ+nex++;
		else { /* no space in obs data */
			ind[i]=-1;
		}
	}
}
/* save obs data in msm message ------------------------------------------------------------------- */
void rtcm_3::save_msm_obs(int sys,msm_h_t *h,const double *r,const double *pr,const double *ccp,
	const double *rr,const double *rrf,const double *cnr,const int *llock,const int *ex,
	const int *half){
	const char *sig[32];
	string signal;
	double tt,wl;
	unsigned char code[32];
	int i,j,k,q,type,prn,sat,fn,index=0,freq[32],ind[32];

	type=getbitu(buff,24,12);

	switch (sys) {
	case SYS_GPS: q=0; break;
	case SYS_GLO: q=1; break;
	case SYS_GAL: q=2; break;
	case SYS_QZS: q=3; break;
	case SYS_SBS: q=4; break;
	case SYS_CMP: q=5; break;
	}
	/* id to signal */
	for (i=0; i<h->nsig; i++) {
		switch (sys) {
		case SYS_GPS: sig[i]=msm_sig_gps[h->sigs[i]-1]; break;
		case SYS_GLO: sig[i]=msm_sig_glo[h->sigs[i]-1]; break;
		case SYS_GAL: sig[i]=msm_sig_gal[h->sigs[i]-1]; break;
		case SYS_QZS: sig[i]=msm_sig_qzs[h->sigs[i]-1]; break;
		case SYS_SBS: sig[i]=msm_sig_sbs[h->sigs[i]-1]; break;
		case SYS_CMP: sig[i]=msm_sig_cmp[h->sigs[i]-1]; break;
		default: sig[i]=""; break;
		}
		signal=sig[i];
		/* signal to rinex obs type */
		code[i]=obs2code(signal,freq+i);

		/* freqency index for beidou */
		if (sys==SYS_CMP) {
			if (freq[i]==5) freq[i]=2; /* B2 */
			else if (freq[i]==4) freq[i]=3; /* B3 */
		}
		if (code[i]!=CODE_NONE) {
			if (q>=0&&q<=5) msmtype[q]="L"+signal+(i<h->nsig-1 ? "," : "");
		}
		else {
			if (q>=0&&q<=5)  msmtype[q]="("+to_string(h->sigs[i])+
				")"+(i<h->nsig-1 ? "," : "");
		}
	}

	/* get signal index */
	sigindex(sys,code,freq,h->nsig,ind);

	for (i=j=0; i<h->nsat; i++) {

		prn=h->sats[i];
		if (sys==SYS_QZS) prn+=MINPRNQZS-1;
		else if (sys==SYS_SBS) prn+=MINPRNSBS-1;

		if ((sat=satno(sys,prn))) {
			tt=obs.data[0].time.timediff(time);
			if (obsflag||fabs(tt)>1E-9) {
				obs.n=obsflag=0;
			}
			index=obsindex(time,sat);
		}
		else {
		}
		for (k=0; k<h->nsig; k++) {
			if (!h->cellmask[k+i*h->nsig]) continue;

			if (sat&&index>=0&&ind[k]>=0) {

				/* satellite carrier wave length */
				wl=satwavelen(sat,freq[k]-1,&nav);

				/* glonass wave length by extended info */
				if (sys==SYS_GLO&&ex&&ex[i]<=13) {
					fn=ex[i]-7;
					wl=CLIGHT/((freq[k]==2 ? FREQ2_GLO : FREQ1_GLO)+
						(freq[k]==2 ? DFRQ2_GLO : DFRQ1_GLO)*fn);
				}
				/* pseudorange (m) */
				if (r[i]!=0.0&&pr[j]>-1E12) {
					obs.data[index].P[ind[k]]=r[i]+pr[j];
				}
				/* carrier-phase (cycle) */
				if (r[i]!=0.0&&ccp[j]>-1E12&&wl>0.0) {
					obs.data[index].L[ind[k]]=(r[i]+ccp[j])/wl;
				}
				/* doppler (hz) */
				if (rr&&rrf&&rrf[j]>-1E12&&wl>0.0) {
					obs.data[index].D[ind[k]]=(float)(-(rr[i]+rrf[j])/wl);
				}
				obs.data[index].LLI[ind[k]]=
					lossoflock(sat,ind[k],llock[j])+(half[j] ? 3 : 0);
				obs.data[index].SNR[ind[k]]=(unsigned char)(cnr[j]*4.0);
				obs.data[index].code[ind[k]]=code[k];
			}
			j++;
		}
	}
}
/* decode type msm message header ----------------------------------------------------------------- */
int rtcm_3::decode_msm_head(int sys,int &sync,int &iod,msm_h_t *h,int &hsize){
	msm_h_t h0={ 0 };
	double tow,tod;
	string str;
	int i=24,j,dow,mask,staid,type,ncell=0;

	type=getbitu(buff,i,12); i+=12;

	*h=h0;
	if (i+157<=len*8) {
		staid     =getbitu(buff,i,12);       i+=12;

		if (sys==SYS_GLO) {
			dow   =getbitu(buff,i,3);        i+= 3;
			tod   =getbitu(buff,i,27)*0.001; i+=27;
			adjday_glot(tod);
		}
		else if (sys==SYS_CMP) {
			tow   =getbitu(buff,i,30)*0.001; i+=30;
			tow+=14.0; /* BDT -> GPST */
			adjweek(tow);
		}
		else {
			tow   =getbitu(buff,i,30)*0.001; i+=30;
			adjweek(tow);
		}
		sync      =getbitu(buff,i,1);       i+= 1;
		iod       =getbitu(buff,i,3);       i+= 3;
		h->time_s =getbitu(buff,i,7);       i+= 7;
		h->clk_str=getbitu(buff,i,2);       i+= 2;
		h->clk_ext=getbitu(buff,i,2);       i+= 2;
		h->smooth =getbitu(buff,i,1);       i+= 1;
		h->tint_s =getbitu(buff,i,3);       i+= 3;
		for (j=1; j<=64; j++) {
			mask=getbitu(buff,i,1); i+=1;
			if (mask) h->sats[h->nsat++]=j;
		}
		for (j=1; j<=32; j++) {
			mask=getbitu(buff,i,1); i+=1;
			if (mask) h->sigs[h->nsig++]=j;
		}
	}
	else return -1;

	/* test station id */
	if (!test_staid(staid)) return -1;

	if (h->nsat*h->nsig>64*32) {
		return -1;
	}
	if (i+h->nsat*h->nsig>len*8) {
		return -1;
	}
	for (j=0; j<h->nsat*h->nsig; j++) {
		h->cellmask[j]=getbitu(buff,i,1); i+=1;
		if (h->cellmask[j]) ncell++;
	}
	hsize=i;

	if (outtype) {
		msgtype+=" "+time.time2str(2)+" staid="+int2str(3," ",staid,str)+" nsat="+int2str(2," ",h->nsat,str)+
			" nsig="+int2str(2," ",h->nsig,str)+" iod="+int2str(2," ",iod,str)+" ncell="+
			int2str(2," ",ncell,str)+" sync="+to_string(sync);
	}
	return ncell;
}
/* decode unsupported msm message ----------------------------------------------------------------- */
int rtcm_3::decode_msm0(int sys){
	msm_h_t h={ 0 };
	int i,sync,iod;
	if (decode_msm_head(sys,sync,iod,&h,i)<0) return -1;
	obsflag=!sync;
	return sync ? 0 : 1;
}
/* decode msm 4: full pseudorange and phaserange plus cnr ----------------------------------------- */
int rtcm_3::decode_msm4(int sys){
	msm_h_t h={ 0 };
	double r[64],pr[64],ccp[64],cnr[64];
	int i,j,type,sync,iod,ncell,rng,rng_m,prv,cpv,llock[64],half[64];

	type=getbitu(buff,24,12);

	/* decode msm header */
	if ((ncell=decode_msm_head(sys,sync,iod,&h,i))<0) return -1;

	if (i+h.nsat*18+ncell*48>len*8) {
		return -1;
	}
	for (j=0; j<h.nsat; j++) r[j]=0.0;
	for (j=0; j<ncell; j++) pr[j]=ccp[j]=-1E16;

	/* decode satellite data */
	for (j=0; j<h.nsat; j++) { /* range */
		rng  =getbitu(buff,i,8); i+= 8;
		if (rng!=255) r[j]=rng*RANGE_MS;
	}
	for (j=0; j<h.nsat; j++) {
		rng_m=getbitu(buff,i,10); i+=10;
		if (r[j]!=0.0) r[j]+=rng_m*P2_10*RANGE_MS;
	}
	/* decode signal data */
	for (j=0; j<ncell; j++) { /* pseudorange */
		prv=getbits(buff,i,15); i+=15;
		if (prv!=-16384) pr[j]=prv*P2_24*RANGE_MS;
	}
	for (j=0; j<ncell; j++) { /* phaserange */
		cpv=getbits(buff,i,22); i+=22;
		if (cpv!=-2097152) ccp[j]=cpv*P2_29*RANGE_MS;
	}
	for (j=0; j<ncell; j++) { /* lock time */
		llock[j]=getbitu(buff,i,4); i+=4;
	}
	for (j=0; j<ncell; j++) { /* half-cycle ambiguity */
		half[j]=getbitu(buff,i,1); i+=1;
	}
	for (j=0; j<ncell; j++) { /* cnr */
		cnr[j]=getbitu(buff,i,6)*1.0; i+=6;
	}
	/* save obs data in msm message */
	save_msm_obs(sys,&h,r,pr,ccp,NULL,NULL,cnr,llock,NULL,half);

	obsflag=!sync;
	return sync ? 0 : 1;
}
/* decode msm 5: full pseudorange, phaserange, phaserangerate and cnr ----------------------------- */
int rtcm_3::decode_msm5(int sys){
	msm_h_t h={ 0 };
	double r[64],rr[64],pr[64],ccp[64],rrf[64],cnr[64];
	int i,j,type,sync,iod,ncell,rng,rng_m,rate,prv,cpv,rrv,llock[64];
	int ex[64],half[64];

	type=getbitu(buff,24,12);

	/* decode msm header */
	if ((ncell=decode_msm_head(sys,sync,iod,&h,i))<0) return -1;

	if (i+h.nsat*36+ncell*63>len*8) {
		return -1;
	}
	for (j=0; j<h.nsat; j++) {
		r[j]=rr[j]=0.0; ex[j]=15;
	}
	for (j=0; j<ncell; j++) pr[j]=ccp[j]=rrf[j]=-1E16;

	/* decode satellite data */
	for (j=0; j<h.nsat; j++) { /* range */
		rng  =getbitu(buff,i,8); i+= 8;
		if (rng!=255) r[j]=rng*RANGE_MS;
	}
	for (j=0; j<h.nsat; j++) { /* extended info */
		ex[j]=getbitu(buff,i,4); i+= 4;
	}
	for (j=0; j<h.nsat; j++) {
		rng_m=getbitu(buff,i,10); i+=10;
		if (r[j]!=0.0) r[j]+=rng_m*P2_10*RANGE_MS;
	}
	for (j=0; j<h.nsat; j++) { /* phaserangerate */
		rate =getbits(buff,i,14); i+=14;
		if (rate!=-8192) rr[j]=rate*1.0;
	}
	/* decode signal data */
	for (j=0; j<ncell; j++) { /* pseudorange */
		prv=getbits(buff,i,15); i+=15;
		if (prv!=-16384) pr[j]=prv*P2_24*RANGE_MS;
	}
	for (j=0; j<ncell; j++) { /* phaserange */
		cpv=getbits(buff,i,22); i+=22;
		if (cpv!=-2097152) ccp[j]=cpv*P2_29*RANGE_MS;
	}
	for (j=0; j<ncell; j++) { /* lock time */
		llock[j]=getbitu(buff,i,4); i+=4;
	}
	for (j=0; j<ncell; j++) { /* half-cycle ambiguity */
		half[j]=getbitu(buff,i,1); i+=1;
	}
	for (j=0; j<ncell; j++) { /* cnr */
		cnr[j]=getbitu(buff,i,6)*1.0; i+=6;
	}
	for (j=0; j<ncell; j++) { /* phaserangerate */
		rrv=getbits(buff,i,15); i+=15;
		if (rrv!=-16384) rrf[j]=rrv*0.0001;
	}
	/* save obs data in msm message */
	save_msm_obs(sys,&h,r,pr,ccp,rr,rrf,cnr,llock,ex,half);

	obsflag=!sync;
	return sync ? 0 : 1;
}
/* decode msm 6: full pseudorange and phaserange plus cnr (high-res) ------------------------------ */
int rtcm_3::decode_msm6(int sys){
	msm_h_t h={ 0 };
	double r[64],pr[64],ccp[64],cnr[64];
	int i,j,type,sync,iod,ncell,rng,rng_m,prv,cpv,llock[64],half[64];

	type=getbitu(buff,24,12);

	/* decode msm header */
	if ((ncell=decode_msm_head(sys,sync,iod,&h,i))<0) return -1;

	if (i+h.nsat*18+ncell*65>len*8) {
		return -1;
	}
	for (j=0; j<h.nsat; j++) r[j]=0.0;
	for (j=0; j<ncell; j++) pr[j]=ccp[j]=-1E16;

	/* decode satellite data */
	for (j=0; j<h.nsat; j++) { /* range */
		rng  =getbitu(buff,i,8); i+= 8;
		if (rng!=255) r[j]=rng*RANGE_MS;
	}
	for (j=0; j<h.nsat; j++) {
		rng_m=getbitu(buff,i,10); i+=10;
		if (r[j]!=0.0) r[j]+=rng_m*P2_10*RANGE_MS;
	}
	/* decode signal data */
	for (j=0; j<ncell; j++) { /* pseudorange */
		prv=getbits(buff,i,20); i+=20;
		if (prv!=-524288) pr[j]=prv*P2_29*RANGE_MS;
	}
	for (j=0; j<ncell; j++) { /* phaserange */
		cpv=getbits(buff,i,24); i+=24;
		if (cpv!=-8388608) ccp[j]=cpv*P2_31*RANGE_MS;
	}
	for (j=0; j<ncell; j++) { /* lock time */
		llock[j]=getbitu(buff,i,10); i+=10;
	}
	for (j=0; j<ncell; j++) { /* half-cycle ambiguity */
		half[j]=getbitu(buff,i,1); i+=1;
	}
	for (j=0; j<ncell; j++) { /* cnr */
		cnr[j]=getbitu(buff,i,10)*0.0625; i+=10;
	}
	/* save obs data in msm message */
	save_msm_obs(sys,&h,r,pr,ccp,NULL,NULL,cnr,llock,NULL,half);

	obsflag=!sync;
	return sync ? 0 : 1;
}
/* decode msm 7: full pseudorange, phaserange, phaserangerate and cnr (h-res) --------------------- */
int rtcm_3::decode_msm7(int sys){
	msm_h_t h={ 0 };
	double r[64],rr[64],pr[64],ccp[64],rrf[64],cnr[64];
	int i,j,type,sync,iod,ncell,rng,rng_m,rate,prv,cpv,rrv,llock[64];
	int ex[64],half[64];

	type=getbitu(buff,24,12);

	/* decode msm header */
	if ((ncell=decode_msm_head(sys,sync,iod,&h,i))<0) return -1;

	if (i+h.nsat*36+ncell*80>len*8) {
		return -1;
	}
	for (j=0; j<h.nsat; j++) {
		r[j]=rr[j]=0.0; ex[j]=15;
	}
	for (j=0; j<ncell; j++) pr[j]=ccp[j]=rrf[j]=-1E16;

	/* decode satellite data */
	for (j=0; j<h.nsat; j++) { /* range */
		rng  =getbitu(buff,i,8); i+= 8;
		if (rng!=255) r[j]=rng*RANGE_MS;
	}
	for (j=0; j<h.nsat; j++) { /* extended info */
		ex[j]=getbitu(buff,i,4); i+= 4;
	}
	for (j=0; j<h.nsat; j++) {
		rng_m=getbitu(buff,i,10); i+=10;
		if (r[j]!=0.0) r[j]+=rng_m*P2_10*RANGE_MS;
	}
	for (j=0; j<h.nsat; j++) { /* phaserangerate */
		rate =getbits(buff,i,14); i+=14;
		if (rate!=-8192) rr[j]=rate*1.0;
	}
	/* decode signal data */
	for (j=0; j<ncell; j++) { /* pseudorange */
		prv=getbits(buff,i,20); i+=20;
		if (prv!=-524288) pr[j]=prv*P2_29*RANGE_MS;
	}
	for (j=0; j<ncell; j++) { /* phaserange */
		cpv=getbits(buff,i,24); i+=24;
		if (cpv!=-8388608) ccp[j]=cpv*P2_31*RANGE_MS;
	}
	for (j=0; j<ncell; j++) { /* lock time */
		llock[j]=getbitu(buff,i,10); i+=10;
	}
	for (j=0; j<ncell; j++) { /* half-cycle amiguity */
		half[j]=getbitu(buff,i,1); i+=1;
	}
	for (j=0; j<ncell; j++) { /* cnr */
		cnr[j]=getbitu(buff,i,10)*0.0625; i+=10;
	}
	for (j=0; j<ncell; j++) { /* phaserangerate */
		rrv=getbits(buff,i,15); i+=15;
		if (rrv!=-16384) rrf[j]=rrv*0.0001;
	}
	/* save obs data in msm message */
	save_msm_obs(sys,&h,r,pr,ccp,rr,rrf,cnr,llock,ex,half);

	obsflag=!sync;
	return sync ? 0 : 1;
}
/* decode type 1230: glonass L1 and L2 code-phase biases ------------------------------------------ */
int rtcm_3::decode_type1230(){
	return 0;
}
/* decode rtcm ver.3 message ---------------------------------------------------------------------- */
int rtcm_3::decode_rtcm3(){
	double tow;
	gtime_t t0;
	string str;
	int ret=0,type=getbitu(buff,24,12),week;

	if (outtype) {
		msgtype="RTCM "+int2str(4," ",type,str)+" ("+int2str(4," ",len,str)+"):";
	}
	/* real-time input option */
	if (opt.find("-RT_INP")!=string::npos) {
		tow=t0.timeget()->utc2gpst()->time2gpst(&week);
		time.gpst2time(week,floor(tow));
	}
	switch (type) {
	case 1001: ret=decode_type1001(); break; /* not supported */
	case 1002: ret=decode_type1002(); break;
	case 1003: ret=decode_type1003(); break; /* not supported */
	case 1004: ret=decode_type1004(); break;
	case 1005: ret=decode_type1005(); break;
	case 1006: ret=decode_type1006(); break;
	case 1007: ret=decode_type1007(); break;
	case 1008: ret=decode_type1008(); break;
	case 1009: ret=decode_type1009(); break; /* not supported */
	case 1010: ret=decode_type1010(); break;
	case 1011: ret=decode_type1011(); break; /* not supported */
	case 1012: ret=decode_type1012(); break;
	case 1013: ret=decode_type1013(); break; /* not supported */
	case 1019: ret=decode_type1019(); break;
	case 1020: ret=decode_type1020(); break;
	case 1021: ret=decode_type1021(); break; /* not supported */
	case 1022: ret=decode_type1022(); break; /* not supported */
	case 1023: ret=decode_type1023(); break; /* not supported */
	case 1024: ret=decode_type1024(); break; /* not supported */
	case 1025: ret=decode_type1025(); break; /* not supported */
	case 1026: ret=decode_type1026(); break; /* not supported */
	case 1027: ret=decode_type1027(); break; /* not supported */
	case 1030: ret=decode_type1030(); break; /* not supported */
	case 1031: ret=decode_type1031(); break; /* not supported */
	case 1032: ret=decode_type1032(); break; /* not supported */
	case 1033: ret=decode_type1033(); break;
	case 1034: ret=decode_type1034(); break; /* not supported */
	case 1035: ret=decode_type1035(); break; /* not supported */
	case 1037: ret=decode_type1037(); break; /* not supported */
	case 1038: ret=decode_type1038(); break; /* not supported */
	case 1039: ret=decode_type1039(); break; /* not supported */
	case 1044: ret=decode_type1044(); break;
	case 1045: ret=decode_type1045(); break;
	case 1046: ret=decode_type1046(); break; /* extension for IGS MGEX */
	case 1042: ret=decode_type1042(); break; /* beidou ephemeris (tentative mt) */
	case   63: ret=decode_type63();   break; /* beidou ephemeris (rtcm draft) */
	case 1057: ret=decode_ssr1(SYS_GPS); break;
	case 1058: ret=decode_ssr2(SYS_GPS); break;
	case 1059: ret=decode_ssr3(SYS_GPS); break;
	case 1060: ret=decode_ssr4(SYS_GPS); break;
	case 1061: ret=decode_ssr5(SYS_GPS); break;
	case 1062: ret=decode_ssr6(SYS_GPS); break;
	case 1063: ret=decode_ssr1(SYS_GLO); break;
	case 1064: ret=decode_ssr2(SYS_GLO); break;
	case 1065: ret=decode_ssr3(SYS_GLO); break;
	case 1066: ret=decode_ssr4(SYS_GLO); break;
	case 1067: ret=decode_ssr5(SYS_GLO); break;
	case 1068: ret=decode_ssr6(SYS_GLO); break;
	case 1071: ret=decode_msm0(SYS_GPS); break; /* not supported */
	case 1072: ret=decode_msm0(SYS_GPS); break; /* not supported */
	case 1073: ret=decode_msm0(SYS_GPS); break; /* not supported */
	case 1074: ret=decode_msm4(SYS_GPS); break;
	case 1075: ret=decode_msm5(SYS_GPS); break;
	case 1076: ret=decode_msm6(SYS_GPS); break;
	case 1077: ret=decode_msm7(SYS_GPS); break;
	case 1081: ret=decode_msm0(SYS_GLO); break; /* not supported */
	case 1082: ret=decode_msm0(SYS_GLO); break; /* not supported */
	case 1083: ret=decode_msm0(SYS_GLO); break; /* not supported */
	case 1084: ret=decode_msm4(SYS_GLO); break;
	case 1085: ret=decode_msm5(SYS_GLO); break;
	case 1086: ret=decode_msm6(SYS_GLO); break;
	case 1087: ret=decode_msm7(SYS_GLO); break;
	case 1091: ret=decode_msm0(SYS_GAL); break; /* not supported */
	case 1092: ret=decode_msm0(SYS_GAL); break; /* not supported */
	case 1093: ret=decode_msm0(SYS_GAL); break; /* not supported */
	case 1094: ret=decode_msm4(SYS_GAL); break;
	case 1095: ret=decode_msm5(SYS_GAL); break;
	case 1096: ret=decode_msm6(SYS_GAL); break;
	case 1097: ret=decode_msm7(SYS_GAL); break;
	case 1101: ret=decode_msm0(SYS_SBS); break; /* not supported */
	case 1102: ret=decode_msm0(SYS_SBS); break; /* not supported */
	case 1103: ret=decode_msm0(SYS_SBS); break; /* not supported */
	case 1104: ret=decode_msm4(SYS_SBS); break;
	case 1105: ret=decode_msm5(SYS_SBS); break;
	case 1106: ret=decode_msm6(SYS_SBS); break;
	case 1107: ret=decode_msm7(SYS_SBS); break;
	case 1111: ret=decode_msm0(SYS_QZS); break; /* not supported */
	case 1112: ret=decode_msm0(SYS_QZS); break; /* not supported */
	case 1113: ret=decode_msm0(SYS_QZS); break; /* not supported */
	case 1114: ret=decode_msm4(SYS_QZS); break;
	case 1115: ret=decode_msm5(SYS_QZS); break;
	case 1116: ret=decode_msm6(SYS_QZS); break;
	case 1117: ret=decode_msm7(SYS_QZS); break;
	case 1121: ret=decode_msm0(SYS_CMP); break; /* not supported */
	case 1122: ret=decode_msm0(SYS_CMP); break; /* not supported */
	case 1123: ret=decode_msm0(SYS_CMP); break; /* not supported */
	case 1124: ret=decode_msm4(SYS_CMP); break;
	case 1125: ret=decode_msm5(SYS_CMP); break;
	case 1126: ret=decode_msm6(SYS_CMP); break;
	case 1127: ret=decode_msm7(SYS_CMP); break;
	case 1230: ret=decode_type1230();    break; /* not supported */
	case 1240: ret=decode_ssr1(SYS_GAL); break;
	case 1241: ret=decode_ssr2(SYS_GAL); break;
	case 1242: ret=decode_ssr3(SYS_GAL); break;
	case 1243: ret=decode_ssr4(SYS_GAL); break;
	case 1244: ret=decode_ssr5(SYS_GAL); break;
	case 1245: ret=decode_ssr6(SYS_GAL); break;
	case 1246: ret=decode_ssr1(SYS_QZS); break;
	case 1247: ret=decode_ssr2(SYS_QZS); break;
	case 1248: ret=decode_ssr3(SYS_QZS); break;
	case 1249: ret=decode_ssr4(SYS_QZS); break;
	case 1250: ret=decode_ssr5(SYS_QZS); break;
	case 1251: ret=decode_ssr6(SYS_QZS); break;
	case 1252: ret=decode_ssr1(SYS_SBS); break;
	case 1253: ret=decode_ssr2(SYS_SBS); break;
	case 1254: ret=decode_ssr3(SYS_SBS); break;
	case 1255: ret=decode_ssr4(SYS_SBS); break;
	case 1256: ret=decode_ssr5(SYS_SBS); break;
	case 1257: ret=decode_ssr6(SYS_SBS); break;
	case 1258: ret=decode_ssr1(SYS_CMP); break;
	case 1259: ret=decode_ssr2(SYS_CMP); break;
	case 1260: ret=decode_ssr3(SYS_CMP); break;
	case 1261: ret=decode_ssr4(SYS_CMP); break;
	case 1262: ret=decode_ssr5(SYS_CMP); break;
	case 1263: ret=decode_ssr6(SYS_CMP); break;
	case 2065: ret=decode_ssr7(SYS_GPS); break; /* tentative */
	case 2066: ret=decode_ssr7(SYS_GLO); break; /* tentative */
	case 2067: ret=decode_ssr7(SYS_GAL); break; /* tentative */
	case 2068: ret=decode_ssr7(SYS_QZS); break; /* tentative */
	case 2070: ret=decode_ssr7(SYS_CMP); break; /* tentative */
	}
	if (ret>=0) {
		type-=1000;
		if (1<=type&&type<= 299) nmsg3[type]++; /* 1001-1299 */
		else if (1000<=type&&type<=1099) nmsg3[type-700]++; /* 2000-2099 */
		else nmsg3[0]++;
	}
	return ret;
}
/* input rtcm 3 message from stream ------------------------------------------------------------------
* fetch next rtcm 3 message and input a message from byte stream
* args   : rtcm_t *rtcm IO   rtcm control struct
*          unsigned char data I stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 5: input station pos/ant parameters,
*                  10: input ssr messages)
* notes  : before firstly calling the function, time in rtcm control struct has
*          to be set to the approximate time within 1/2 week in order to resolve
*          ambiguity of time in rtcm messages.
*
*          to specify input options, set this->opt to the following option
*          strings separated by spaces.
*
*          -EPHALL  : input all ephemerides
*          -STA=nnn : input only message with STAID=nnn
*          -GLss    : select signal ss for GPS MSM (ss=1C,1P,...)
*          -RLss    : select signal ss for GLO MSM (ss=1C,1P,...)
*          -ELss    : select signal ss for GAL MSM (ss=1C,1B,...)
*          -JLss    : select signal ss for QZS MSM (ss=1C,2C,...)
*          -CLss    : select signal ss for BDS MSM (ss=2I,7I,...)
*
*          supported RTCM 3 messages
*                  (ref [2][3][4][5][6][7][8][9][10][11][12][13][14][15])
*
*            TYPE       GPS     GLOASS    GALILEO    QZSS     BEIDOU     SBAS
*         ----------------------------------------------------------------------
*          OBS C-L1  : 1001~     1009~       -         -         -         -
*              F-L1  : 1002      1010        -         -         -         -
*              C-L12 : 1003~     1011~       -         -         -         -
*              F-L12 : 1004      1012        -         -         -         -
*
*          NAV       : 1019      1020      1045*     1044*     1042(63)  1043
*                        -         -       1046*       -         -         -
*
*          MSM 1     : 1071~     1081~     1091~     1111*~    1121*~    1101*~
*              2     : 1072~     1082~     1092~     1112*~    1122*~    1102*~
*              3     : 1073~     1083~     1093~     1113*~    1123*~    1103*~
*              4     : 1074      1084      1094      1114*     1124*     1104*
*              5     : 1075      1085      1095      1115*     1125*     1105*
*              6     : 1076      1086      1096      1116*     1126*     1106*
*              7     : 1077      1087      1097      1117*     1127*     1107*
*
*          SSR OBT   : 1057      1063      1240*     1246*     1258*       -
*              CLK   : 1058      1064      1241*     1247*     1259*       -
*              BIAS  : 1059      1065      1242*     1248*     1260*       -
*              OBTCLK: 1060      1066      1243*     1249*     1261*       -
*              URA   : 1061      1067      1244*     1250*     1262*       -
*              HRCLK : 1062      1068      1245*     1251*     1263*       -
*
*          ANT INFO  : 1005 1006 1007 1008 1033
*         ----------------------------------------------------------------------
*                                                    (* draft, ~ only encode)
*
*          for MSM observation data with multiple signals for a frequency,
*          a signal is selected according to internal priority. to select
*          a specified signal, use the input options.
*
*          rtcm3 message format:
*            +----------+--------+-----------+--------------------+----------+
*            | preamble | 000000 |  length   |    data message    |  parity  |
*            +----------+--------+-----------+--------------------+----------+
*            |<-- 8 --->|<- 6 -->|<-- 10 --->|<--- length x 8 --->|<-- 24 -->|
*
*-------------------------------------------------------------------------------------------------- */
int rtcm_3::decode(unsigned char data){
	/* synchronize frame */
	if (nbyte==0) {
		if (data!=RTCM3PREAMB) return 0;
		buff[nbyte++]=data;
		return 0;
	}
	buff[nbyte++]=data;

	if (nbyte==3) {
		len=getbitu(buff,14,10)+3; /* length without parity */
	}
	if (nbyte<3||nbyte<len+3) return 0;
	nbyte=0;

	/* check parity */
	if (rtk_crc24q(buff,len)!=getbitu(buff,len*8,24)) {
		return 0;
	}
	/* decode rtcm3 message */
	return decode_rtcm3();
}