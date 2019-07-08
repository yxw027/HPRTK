#include "nvs.h"

#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
#define NVSSYNC     0x10        /* nvs message sync code 1 */
#define NVSENDMSG   0x03        /* nvs message sync code 1 */
#define NVSCFG      0x06        /* nvs message cfg-??? */

#define ID_XF5RAW   0xf5        /* nvs msg id: raw measurement data */
#define ID_X4AIONO  0x4a        /* nvs msg id: gps ionospheric data */
#define ID_X4BTIME  0x4b        /* nvs msg id: GPS/GLONASS/UTC timescale data */
#define ID_XF7EPH   0xf7        /* nvs msg id: subframe buffer */
#define ID_XE5BIT   0xe5        /* nvs msg id: bit information */

#define ID_XD7ADVANCED 0xd7     /* */
#define ID_X02RATEPVT  0x02     /* */
#define ID_XF4RATERAW  0xf4     /* */
#define ID_XD7SMOOTH   0xd7     /* */
#define ID_XD5BIT      0xd5     /* */

static const char rcsid[]="$Id: nvs.c,v 1.0 2012/01/30 00:05:05 MBAVA Exp $";

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))
static unsigned short U2(unsigned char *p) { unsigned short u; memcpy(&u,p,2); return u; }
static unsigned int   U4(unsigned char *p) { unsigned int   u; memcpy(&u,p,4); return u; }
static short          I2(unsigned char *p) { short          i; memcpy(&i,p,2); return i; }
static int            I4(unsigned char *p) { int            i; memcpy(&i,p,4); return i; }
static float          R4(unsigned char *p) { float          r; memcpy(&r,p,4); return r; }
static double         R8(unsigned char *p) { double         r; memcpy(&r,p,8); return r; }

/* ura values (ref [3] 20.3.3.3.1.1) -----------------------------------------*/
static const double ura_eph[]={
	2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
	3072.0,6144.0,0.0
};
/* ura value (m) to ura index ------------------------------------------------*/
static int uraindex(double value)
{
	int i;
	for (i=0; i<15; i++) if (ura_eph[i]>=value) break;
	return i;
}


/* input NVS raw message from stream -----------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
nvs::nvs(){
}
nvs::~nvs(){
}

/* adjust daily rollover of time ------------------------------------------ */
gtime_t nvs::adjday(double ttt){
	double tod_p;
	gtime_t adjt;

	time.time2epoch();
	tod_p=time.ep[3]*3600.0+time.ep[4]*60.0+time.ep[5];
	if (ttt<tod_p-43200.0) ttt+=86400.0;
	else if (ttt>tod_p+43200.0) ttt-=86400.0;

	adjt.ep[0]=time.ep[0]; adjt.ep[1]=time.ep[1]; adjt.ep[2]=time.ep[2];
	adjt.ep[3]=adjt.ep[4]=adjt.ep[5]=0.0;
	return *adjt.epoch2time(adjt.ep)->timeadd(ttt);
}
/* decode ephemeris ------------------------------------------------------- */
int nvs::decode_gpsephem(int sat){
	eph_t eph;
	unsigned char *puiTmp = (buff)+2;
	unsigned short week;
	double toc;

	eph.crs    = R4(&puiTmp[2]);
	eph.deln   = R4(&puiTmp[6]) * 1e+3;
	eph.M0     = R8(&puiTmp[10]);
	eph.cuc    = R4(&puiTmp[18]);
	eph.e      = R8(&puiTmp[22]);
	eph.cus    = R4(&puiTmp[30]);
	eph.A      = pow(R8(&puiTmp[34]),2);
	eph.toes   = R8(&puiTmp[42]) * 1e-3;
	eph.cic    = R4(&puiTmp[50]);
	eph.OMG0   = R8(&puiTmp[54]);
	eph.cis    = R4(&puiTmp[62]);
	eph.i0     = R8(&puiTmp[66]);
	eph.crc    = R4(&puiTmp[74]);
	eph.omg    = R8(&puiTmp[78]);
	eph.OMGd   = R8(&puiTmp[86]) * 1e+3;
	eph.idot   = R8(&puiTmp[94]) * 1e+3;
	eph.tgd[0] = R4(&puiTmp[102]) * 1e-3;
	toc        = R8(&puiTmp[106]) * 1e-3;
	eph.f2     = R4(&puiTmp[114]) * 1e+3;
	eph.f1     = R4(&puiTmp[118]);
	eph.f0     = R4(&puiTmp[122]) * 1e-3;
	eph.sva    = uraindex(I2(&puiTmp[126]));
	eph.iode   = I2(&puiTmp[128]);
	eph.iodc   = I2(&puiTmp[130]);
	eph.code   = I2(&puiTmp[132]);
	eph.flag   = I2(&puiTmp[134]);
	week       = I2(&puiTmp[136]);
	eph.fit    = 0;

	if (week>=4096) {
		return -1;
	}
	eph.week=adjgpsweek(week);
	eph.toe.gpst2time(eph.week,eph.toes);
	eph.toc.gpst2time(eph.week,toc);
	eph.ttr=time;

	if (opt.find("-EPHALL")==string::npos) {
		if (eph.iode==nav.eph[sat-1].iode) return 0; /* unchanged */
	}
	eph.sat=sat;
	nav.eph[sat-1]=eph;
	ephsat=sat;
	return 2;
}
/* decode gloephem -------------------------------------------------------- */
int nvs::decode_gloephem(int sat){
	geph_t geph;
	unsigned char *p=(buff)+2;
	int prn,tk,tb;

	if (len>=93) {
		prn        =I1(p+ 1);
		geph.frq   =I1(p+ 2);
		geph.pos[0]=R8(p+ 3);
		geph.pos[1]=R8(p+11);
		geph.pos[2]=R8(p+19);
		geph.vel[0]=R8(p+27) * 1e+3;
		geph.vel[1]=R8(p+35) * 1e+3;
		geph.vel[2]=R8(p+43) * 1e+3;
		geph.acc[0]=R8(p+51) * 1e+6;
		geph.acc[1]=R8(p+59) * 1e+6;
		geph.acc[2]=R8(p+67) * 1e+6;
		tb = R8(p+75) * 1e-3;
		tk = tb;
		geph.gamn  =R4(p+83);
		geph.taun  =R4(p+87) * 1e-3;
		geph.age   =I2(p+91);
	}
	else {
		return -1;
	}
	if (!(geph.sat=satno(SYS_GLO,prn))) {
		return -1;
	}
	if (time.time==0) return 0;

	geph.iode=(tb/900)&0x7F;
	geph.toe=adjday(tb-10800.0); geph.toe.utc2gpst();
	geph.tof=adjday(tk-10800.0); geph.tof.utc2gpst();
#if 0
	/* check illegal ephemeris by toe */
	tt=time.timediff(geph.toe);
	if (fabs(tt)>3600.0) {
		return 0;
	}
#endif
#if 0
	/* check illegal ephemeris by frequency number consistency */
	if (nav.geph[prn-MINPRNGLO].toe.time&&
		geph.frq!=nav.geph[prn-MINPRNGLO].frq) {
		return -1;
	}
	if (opt.find("-EPHALL")==string::npos) {
		if (fabs(geph.toe.timediff(nav.geph[prn-MINPRNGLO].toe))<1.0&&
			geph.svh==nav.geph[prn-MINPRNGLO].svh) return 0;
	}
#endif
	nav.geph[prn-1]=geph;
	ephsat=geph.sat;

	return 2;
}

/* decode NVS xf5-raw: raw measurement data ------------------------------- */
int nvs::decode_xf5raw(){
	gtime_t ttt;
	double tadj=0.0,toff=0.0,tn;
	int dTowInt;
	double dTowUTC,dTowGPS,dTowFrac,L1,P1,D1;
	double gpsutcTimescale;
	unsigned char rcvTimeScaleCorr,sys,carrNo;
	int i,j,prn,sat,n=0,nsat,week;
	unsigned char *p=buff+2;
	char fflag;
	size_t qqq;

	/* time tag adjustment option (-TADJ) */
	if ((qqq=opt.find("-tadj"))!=string::npos) {
		str2double(opt.substr(qqq+5),tadj);
	}
	dTowUTC =R8(p);
	week = U2(p+8);
	gpsutcTimescale = R8(p+10);
	/* glonassutcTimescale = R8(p+18); */
	rcvTimeScaleCorr = I1(p+26);

	/* check gps week range */
	if (week>=4096) {
		return -1;
	}
	week=adjgpsweek(week);

	if ((len - 31)%30) {

		/* Message length is not correct: there could be an error in the stream */
		return -1;
	}
	nsat = (len - 31)/30;

	dTowGPS = dTowUTC + gpsutcTimescale;

	/* Tweak pseudoranges to allow Rinex to represent the NVS time of measure */
	dTowInt  = 10.0*floor((dTowGPS/10.0)+0.5);
	dTowFrac = dTowGPS - (double)dTowInt;
	ttt.gpst2time(week,dTowInt*0.001);

	/* time tag adjustment */
	if (tadj>0.0) {
		tn=ttt.time2gpst(&week)/tadj;
		toff=(tn-floor(tn+0.5))*tadj;
		ttt.timeadd(-toff);
	}
	/* check time tag jump and output warning */
	if (time.time&&fabs(ttt.timediff(time))>86400.0) {
		ttt.time2str(3);
	}
	if (fabs(ttt.timediff(time))<=1e-3) {
		ttt.time2str(3);
		return 0;
	}
	for (i=0,p+=27; (i<nsat) && (n<MAXOBS); i++,p+=30) {
		obs.data[n].time  = ttt;
		sys = (U1(p)==1) ? SYS_GLO : ((U1(p)==2) ? SYS_GPS : ((U1(p)==4) ? SYS_SBS : SYS_NONE));
		prn = U1(p+1);
		if (sys == SYS_SBS) prn += 120; /* Correct this */
		if (!(sat=satno(sys,prn))) {
			continue;
		}
		carrNo = I1(p+2);
		L1 = R8(p+ 4);
		P1 = R8(p+12);
		D1 = R8(p+20);

		/* check range error */
		if (L1<-1E10||L1>1E10||P1<-1E10||P1>1E10||D1<-1E5||D1>1E5) {
			continue;
		}
		obs.data[n].SNR[0]=(unsigned char)(I1(p+3)*4.0+0.5);
		if (sys==SYS_GLO) {
			obs.data[n].L[0]  =  L1 - toff*(FREQ1_GLO+DFRQ1_GLO*carrNo);
		}
		else {
			obs.data[n].L[0]  =  L1 - toff*FREQ1;
		}
		obs.data[n].P[0]    = (P1-dTowFrac)*CLIGHT*0.001 - toff*CLIGHT; /* in ms, needs to be converted */
		obs.data[n].D[0]    =  (float)D1;

		/* set LLI if meas flag 4 (carrier phase present) off -> on */
		fflag=U1(p+28);
		obs.data[n].LLI[0]=(fflag&0x08)&&!(halfc[sat-1][0]&0x08) ? 1 : 0;
		halfc[sat-1][0]=fflag;

#if 0
		if (obs.data[n].SNR[0] > 160) {
			ttt.time2str(3);
		}
#endif
		obs.data[n].code[0] = CODE_L1C;
		obs.data[n].sat = sat;

		for (j=1; j<NFREQ+NEXOBS; j++) {
			obs.data[n].L[j]=obs.data[n].P[j]=0.0;
			obs.data[n].D[j]=0.0;
			obs.data[n].SNR[j]=obs.data[n].LLI[j]=0;
			obs.data[n].code[j]=CODE_NONE;
		}
		n++;
	}
	time=ttt;
	obs.n=n;
	return 1;
}
/* decode NVS epehemerides in clear --------------------------------------- */
int nvs::decode_xf7eph(){
	int prn,sat,sys;
	unsigned char *p=buff;

	if ((len)<93) {
		return -1;
	}
	sys = (U1(p+2)==1) ? SYS_GPS : ((U1(p+2)==2) ? SYS_GLO : SYS_NONE);
	prn = U1(p+3);
	if (!(sat=satno(sys==1 ? SYS_GPS : SYS_GLO,prn))) {
		return -1;
	}
	if (sys==SYS_GPS) {
		return decode_gpsephem(sat);
	}
	else if (sys==SYS_GLO) {
		return decode_gloephem(sat);
	}
	return 0;
}
/* decode NVS rxm-sfrb: subframe buffer ----------------------------------- */
int nvs::decode_xe5bit(){
	int prn;
	int iBlkStartIdx,iExpLen,iIdx;
	unsigned int words[10];
	unsigned char uiDataBlocks,uiDataType;
	unsigned char *p=buff;

	p += 2;         /* Discard preamble and message identifier */
	uiDataBlocks = U1(p);

	if (uiDataBlocks>=16) {
		return -1;
	}
	iBlkStartIdx = 1;
	for (iIdx = 0; iIdx < uiDataBlocks; iIdx++) {
		iExpLen = (iBlkStartIdx+10);
		if ((len) < iExpLen) {
			return -1;
		}
		uiDataType = U1(p+iBlkStartIdx+1);

		switch (uiDataType) {
		case 1: /* Glonass */
			iBlkStartIdx += 19;
			break;
		case 2: /* GPS */
			iBlkStartIdx += 47;
			break;
		case 4: /* SBAS */
			prn = U1(p+(iBlkStartIdx+2)) + 120;

			/* sat = satno(SYS_SBS, prn); */
			/* sys = satsys(sat,&prn); */
			memset(words,0,10*sizeof(unsigned int));
			for (iIdx=0,iBlkStartIdx+=7; iIdx<10; iIdx++,iBlkStartIdx+=4) {
				words[iIdx]=U4(p+iBlkStartIdx);
			}
			words[7] >>= 6;
			return sbsdecodemsg(prn,words) ? 3 : 0;
		default:
			return -1;
		}
	}
	return 0;
}
/* decode NVS x4aiono ----------------------------------------------------- */
int nvs::decode_x4aiono(){
	unsigned char *p=buff+2;

	nav.ion_gps[0] = R4(p);
	nav.ion_gps[1] = R4(p+ 4);
	nav.ion_gps[2] = R4(p+ 8);
	nav.ion_gps[3] = R4(p+12);
	nav.ion_gps[4] = R4(p+16);
	nav.ion_gps[5] = R4(p+20);
	nav.ion_gps[6] = R4(p+24);
	nav.ion_gps[7] = R4(p+28);

	return 9;
}
/* decode NVS x4btime ----------------------------------------------------- */
int nvs::decode_x4btime(){
	unsigned char *p=buff+2;

	nav.utc_gps[1] = R8(p);
	nav.utc_gps[0] = R8(p+ 8);
	nav.utc_gps[2] = I4(p+16);
	nav.utc_gps[3] = I2(p+20);
	nav.leaps = I1(p+22);

	return 9;
}

/* decode NVS raw message ------------------------------------------------- */
int nvs::decode_nvs(){
	int type=U1(buff+1);
	string str;
		
	msgtype="NVS: type="+int2str(2," ",type,str)+" len="+int2str(3," ",len,str);

	switch (type) {
	case ID_XF5RAW:  return decode_xf5raw();
	case ID_XF7EPH:  return decode_xf7eph();
	case ID_XE5BIT:  return decode_xe5bit();
	case ID_X4AIONO: return decode_x4aiono();
	case ID_X4BTIME: return decode_x4btime();
	default: break;
	}
	return 0;
}

/* input NVS raw message from stream -------------------------------------------------------------- */
int nvs::decode(unsigned char data){
	/* synchronize frame */
	if ((nbyte==0) && (data==NVSSYNC)) {

		/* Search a 0x10 */
		buff[0] = data;
		nbyte=1;
		return 0;
	}
	if ((nbyte==1) && (data != NVSSYNC) && (data != NVSENDMSG)) {

		/* Discard double 0x10 and 0x10 0x03 at beginning of frame */
		buff[1]=data;
		nbyte=2;
		flag=0;
		return 0;
	}
	/* This is all done to discard a double 0x10 */
	if (data==NVSSYNC) flag = (flag +1) % 2;
	if ((data!=NVSSYNC) || (flag)) {

		/* Store the new byte */
		buff[(nbyte++)] = data;
	}
	/* Detect ending sequence */
	if ((data==NVSENDMSG) && (flag)) {
		len   = nbyte;
		nbyte = 0;

		/* Decode NVS raw message */
		return decode_nvs();
	}
	if (nbyte == MAXRAWLEN) {
		nbyte=0;
		return -1;
	}
	return 0;
}