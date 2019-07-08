#include "superstar2.h"

#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
#define SS2SOH      0x01        /* ss2 start of header */
#define ID_SS2LLH   20          /* ss2 message ID#20 navigation data (user) */
#define ID_SS2ECEF  21          /* ss2 message ID#21 navigation data (ecef) */
#define ID_SS2EPH   22          /* ss2 message ID#22 ephemeris data */
#define ID_SS2RAW   23          /* ss2 message ID#23 measurement block */
#define ID_SS2SBAS  67          /* ss2 message ID#67 sbas data */

static const char rcsid[]="$Id: ss2.c,v 1.2 2008/07/14 00:05:05 TTAKA Exp $";

/* get/set fields (little-endian) --------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
static unsigned short U2(unsigned char *p) { unsigned short u; memcpy(&u,p,2); return u; }
static unsigned int   U4(unsigned char *p) { unsigned int   u; memcpy(&u,p,4); return u; }
static double         R8(unsigned char *p) { double         r; memcpy(&r,p,8); return r; }

/* input superstar 2 raw message from stream ---------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
ss2::ss2(){
}
ss2::~ss2(){
}

/* checksum ------------------------------------------------------------------*/
int ss2::chksum(){
	int i;
	unsigned short sum=0;

	for (i=0; i<len-2; i++) sum+=buff[i];
	return (sum>>8)==buff[len-1]&&(sum&0xFF)==buff[len-2];
}
/* adjust week ------------------------------------------------------------ */
int ss2::adjweek(double sec){
	double tow;
	int week;

	if (time.time==0) return 0;
	tow=time.time2gpst(&week);
	if (sec<tow-302400.0) sec+=604800.0;
	else if (sec>tow+302400.0) sec-=604800.0;
	time.gpst2time(week,sec);
	return 1;
}
/* decode id#20 navigation data (user) ------------------------------------ */
int ss2::decode_ss2llh(){
	unsigned char *p=buff+4;

	if (len!=77) {
		return -1;
	}
	time.ep[3]=U1(p); time.ep[4]=U1(p+ 1); time.ep[5]=R8(p+ 2);
	time.ep[2]=U1(p+10); time.ep[1]=U1(p+11); time.ep[0]=U2(p+12);
	time.epoch2time(time.ep)->utc2gpst();
	return 0;
}
/* decode id#21 navigation data (ecef) ------------------------------------ */
int ss2::decode_ss2ecef(){
	unsigned char *p=buff+4;

	if (len!=85) {
		return -1;
	}
	time.gpst2time(U2(p+8),R8(p));
	return 0;
}
/* decode id#23 measurement block ----------------------------------------- */
int ss2::decode_ss2meas(){
	const double freqif=1.405396825E6,tslew=1.75E-7;
	double tow,slew,code,icp,d;
	int i,j,n,prn,sat,nobs;
	unsigned char *p=buff+4;
	unsigned int sc;

	nobs=U1(p+2);
	if (17+nobs*11!=len) {
		return -1;
	}
	tow=floor(R8(p+3)*1000.0+0.5)/1000.0; /* rounded by 1ms */
	if (!adjweek(tow)) {
		return -1;
	}
	/* time slew defined as uchar (ref [1]) but minus value appears in some f/w */
	slew=*(char *)(p)*tslew;

	icpc+=4.5803-freqif*slew-FREQ1*(slew-1E-6); /* phase correction */

	for (i=n=0,p+=11; i<nobs&&n<MAXOBS; i++,p+=11) {
		prn=(p[0]&0x1F)+1;
		if (!(sat=satno(p[0]&0x20 ? SYS_SBS : SYS_GPS,prn))) {
			continue;
		}
		obs.data[n].time=time;
		obs.data[n].sat=sat;
		code=(tow-floor(tow))-(double)(U4(p+2))/2095104000.0;
		obs.data[n].P[0]=CLIGHT*(code+(code<0.0 ? 1.0 : 0.0));
		icp=(double)(U4(p+6)>>2)/1024.0+off[sat-1]; /* unwrap */
		if (fabs(icp-icpp[sat-1])>524288.0) {
			d=icp>icpp[sat-1] ? -1048576.0 : 1048576.0;
			off[sat-1]+=d; icp+=d;
		}
		icpp[sat-1]=icp;
		obs.data[n].L[0]=icp+icpc;
		obs.data[n].D[0]=0.0;
		obs.data[n].SNR[0]=(unsigned char)(floor(U1(p+1)+0.5));
		sc=U1(p+10);
		obs.data[n].LLI[0]=(int)((unsigned char)sc-(unsigned char)lockt[sat-1][0])>0;
		obs.data[n].LLI[0]|=U1(p+6)&1 ? 2 : 0;
		obs.data[n].code[0]=CODE_L1C;
		lockt[sat-1][0]=sc;

		for (j=1; j<NFREQ; j++) {
			obs.data[n].L[j]=obs.data[n].P[j]=0.0;
			obs.data[n].D[j]=0.0;
			obs.data[n].SNR[j]=obs.data[n].LLI[j]=0;
			obs.data[n].code[j]=CODE_NONE;
		}
		n++;
	}
	obs.n=n;
	return 1;
}
/* decode id#22 ephemeris data -------------------------------------------- */
int ss2::decode_ss2eph(){
	decode_frame dec;
	unsigned int tow;
	int i,j,prn,sat;
	unsigned char *p=buff+4,bbuff[90]={ 0 };

	if (len!=79) {
		return -1;
	}
	prn=(U4(p)&0x1F)+1;
	if (!(sat=satno(SYS_GPS,prn))) {
		return -1;
	}
	if (time.time==0) {
		return -1;
	}
	tow=(unsigned int)(time.time2gpst(NULL)/6.0);
	for (i=0; i<3; i++) {
		bbuff[30*i+3]=(unsigned char)(tow>>9); /* add tow + subframe id */
		bbuff[30*i+4]=(unsigned char)(tow>>1);
		bbuff[30*i+5]=(unsigned char)(((tow&1)<<7)+((i+1)<<2));
		for (j=0; j<24; j++) bbuff[30*i+6+j]=p[1+24*i+j];
	}
	if (dec.decode(bbuff)!=1||
		dec.decode(bbuff+30)!=2||
		dec.decode(bbuff+60)!=3) {
		return -1;
	}
	if (dec.eph.iode==nav.eph[sat-1].iode) return 0; /* unchanged */
	dec.eph.sat=sat;
	dec.eph.ttr=time;
	nav.eph[sat-1]=dec.eph;
	ephsat=sat;
	return 2;
}
/* decode id#67 sbas data ------------------------------------------------- */
int ss2::decode_ss2sbas(){
	gtime_t ttt;
	int i,prn;
	unsigned char *p=buff+4;

	if (len!=54) {
		return -1;
	}
	prn=U4(p+12);
	if (prn<MINPRNSBS||MAXPRNSBS<prn) return 0;
	sbsmsg.week=U4(p);
	sbsmsg.tow=(int)R8(p+4);
	ttt.gpst2time(sbsmsg.week,sbsmsg.tow);
	sbsmsg.prn=prn;
	for (i=0; i<29; i++) sbsmsg.msg[i]=p[16+i];
	return 3;
}
/* sync code -------------------------------------------------------------------------------------- */
int ss2::sync_ss2(unsigned char data){
	buff[0]=buff[1]; buff[1]=buff[2]; buff[2]=data;
	return buff[0]==SS2SOH&&(buff[1]^buff[2])==0xFF;
}
/* decode superstar 2 raw message ----------------------------------------- */
int ss2::decode_ss2(){
	unsigned char *p=buff;
	int type=U1(p+1);
	string str;

	if (!chksum()) {
		return -1;
	}
	if (outtype) {
		msgtype="SS2 "+int2str(2," ",type,str)+" ("+int2str(4," ",len,str)+"):";
	}
	switch (type) {
		case ID_SS2LLH: return decode_ss2llh();
		case ID_SS2ECEF: return decode_ss2ecef();
		case ID_SS2RAW: return decode_ss2meas();
		case ID_SS2EPH: return decode_ss2eph();
		case ID_SS2SBAS: return decode_ss2sbas();
	}
	return 0;
}

/* input ublox raw message from stream ------------------------------------------------------------ */
int ss2::decode(unsigned char data){
	/* synchronize frame */
	if (nbyte==0) {
		if (!sync_ss2(data)) return 0;
		nbyte=3;
		return 0;
	}
	buff[nbyte++]=data;

	if (nbyte==4) {
		if ((len=U1(buff+3)+6)>MAXRAWLEN) {
			nbyte=0;
			return -1;
		}
	}
	if (nbyte<4||nbyte<len) return 0;
	nbyte=0;

	/* decode superstar 2 raw message */
	return decode_ss2();
}