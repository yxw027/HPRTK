#include "javad.h"

#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
static const char rcsid[]="$Id:$";

#define PREAMB_CNAV 0x8B

#define ISTXT(c)    ('0'<=(c)&&(c)<='~')
#define ISHEX(c)    (('0'<=(c)&&(c)<='9')||('A'<=(c)&&(c)<='F'))
#define ROT_LEFT(val) (((val)<<2)|((val)>>6))

/* extract field (little-endian) ------------------------------------------------------------------ */
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))
static unsigned short U2(unsigned char *p) { unsigned short u; memcpy(&u,p,2); return u; }
static unsigned int   U4(unsigned char *p) { unsigned int   u; memcpy(&u,p,4); return u; }
static short          I2(unsigned char *p) { short          i; memcpy(&i,p,2); return i; }
static int            I4(unsigned char *p) { int            i; memcpy(&i,p,4); return i; }

static float R4(unsigned char *p)
{
	float value;
	unsigned char *q=(unsigned char *)&value;
	int i;
	if (U4(p)==0x7FC00000) return 0.0f; /* quiet nan */
	for (i=0; i<4; i++) *q++=*p++;
	return value;
}
static double R8(unsigned char *p)
{
	double value;
	unsigned char *q=(unsigned char *)&value;
	int i;
	if (U4(p+4)==0x7FF80000&&U4(p)==0) return 0.0; /* quiet nan */
	for (i=0; i<8; i++) *q++=*p++;
	return value;
}

/* input javad raw message from stream ---------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
javad::javad(){
}
javad::~javad(){
}
/* decode message length -------------------------------------------------------------------------- */
int javad::decodelen(const unsigned char *bb){
	unsigned int len;
	if (!ISHEX(bb[0])||!ISHEX(bb[1])||!ISHEX(bb[2])) return 0;
	if (sscanf((char *)bb,"%3X",&len)==1) return (int)len;
	return 0;
}
/* test measurement data -------------------------------------------------------------------------- */
int javad::is_meas(char sig){
	return sig=='c'||sig=='C'||sig=='1'||sig=='2'||sig=='3'||sig=='5'||sig=='l';
}
/* convert signal to frequency and obs type ------------------------------------------------------- */
int javad::tofreq(char sig,int sys,int &type){
	const unsigned char types[6][6]={ /* ref [5] table 3-7 */
									  /*  c/C       1        2        3        5        l  */
		{ CODE_L1C,CODE_L1W,CODE_L2W,CODE_L2X,CODE_L5X,CODE_L1X }, /* GPS */
		{ CODE_L1C,CODE_L1Z,CODE_L6X,CODE_L2X,CODE_L5X,CODE_L1X }, /* QZS */
		{ CODE_L1C,0       ,0       ,0       ,CODE_L5X,0 }, /* SBS */
		{ CODE_L1X,CODE_L8X,CODE_L7X,CODE_L6X,CODE_L5X,0 }, /* GAL */
		{ CODE_L1C,CODE_L1P,CODE_L2P,CODE_L2C,CODE_L3X,0 }, /* GLO */
		{ CODE_L1I,0       ,0       ,0       ,CODE_L7I,0 }  /* CMP */
	};
	const int freqs[6][6]={
		{ 1,1,2,2,3,1 },{ 1,1,4,2,3,1 },{ 1,0,0,0,3,0 },     /* GPS,QZS,SBS */
		{ 1,6,5,4,3,0 },{ 1,1,2,2,3,0 },{ 1,0,0,0,2,0 }      /* GAL,GLO,CMP */
	};
	int i,j;

	switch (sig) {
	case 'c':
	case 'C': i=0; break;
	case '1': i=1; break;
	case '2': i=2; break;
	case '3': i=3; break;
	case '5': i=4; break;
	case 'l': i=5; break;
	default: return -1;
	}
	switch (sys) {
	case SYS_GPS: j=0; break;
	case SYS_QZS: j=1; break;
	case SYS_SBS: j=2; break;
	case SYS_GAL: j=3; break;
	case SYS_GLO: j=4; break;
	case SYS_CMP: j=5; break;
	default: return -1;
	}
	type=types[j][i];

	/* 0:L1,1:L2,2:L5,3:L6,4:L7,5:L8,-1:error */
	return freqs[j][i]<=NFREQ ? freqs[j][i]-1 : -1;
}
/* check code priority and return obs position ---------------------------------------------------- */
int javad::checkpri(int sys,int code,int freq){
	int nex=NEXOBS; /* number of extended obs data */

	if (sys==SYS_GPS) {
		if (opt.find("-GL1W")!=string::npos&&freq==0) return code==CODE_L1W ? 0 : -1;
		if (opt.find("-GL1X")!=string::npos&&freq==0) return code==CODE_L1X ? 0 : -1;
		if (opt.find("-GL2X")!=string::npos&&freq==1) return code==CODE_L2X ? 1 : -1;
		if (code==CODE_L1W) return nex<1 ? -1 : NFREQ;
		if (code==CODE_L2X) return nex<2 ? -1 : NFREQ+1;
		if (code==CODE_L1X) return nex<3 ? -1 : NFREQ+2;
	}
	else if (sys==SYS_GLO) {
		if (opt.find("-RL1C")!=string::npos&&freq==0) return code==CODE_L1C ? 0 : -1;
		if (opt.find("-RL2C")!=string::npos&&freq==1) return code==CODE_L2C ? 1 : -1;
		if (code==CODE_L1C) return nex<1 ? -1 : NFREQ;
		if (code==CODE_L2C) return nex<2 ? -1 : NFREQ+1;
	}
	else if (sys==SYS_QZS) {
		if (opt.find("-JL1Z")!=string::npos&&freq==0) return code==CODE_L1Z ? 0 : -1;
		if (opt.find("-JL1X")!=string::npos&&freq==0) return code==CODE_L1X ? 0 : -1;
		if (code==CODE_L1Z) return nex<1 ? -1 : NFREQ;
		if (code==CODE_L1X) return nex<2 ? -1 : NFREQ+1;
	}
	return freq<NFREQ ? freq : -1;
}
/* glonass carrier frequency ---------------------------------------------------------------------- */
double javad::freq_glo(int freq,int num){
	switch (freq) {
	case 0: return FREQ1_GLO+DFRQ1_GLO*freqn[num];
	case 1: return FREQ2_GLO+DFRQ2_GLO*freqn[num];
	}
	return 0.0;
}
/* checksum --------------------------------------------------------------------------------------- */
int javad::checksum(){
	unsigned char cs=0;
	int i;
	for (i=0; i<len-1; i++) {
		cs=ROT_LEFT(cs)^buff[i];
	}
	cs=ROT_LEFT(cs);
	return cs==buff[len-1];
}
/* adjust weekly rollover of gps time ------------------------------------------------------------- */
gtime_t javad::adjweek(gtime_t ttt,double tow){
	double tow_p;
	int week;
	gtime_t adjt;

	tow_p=ttt.time2gpst(&week);
	if (tow<tow_p-302400.0) tow+=604800.0;
	else if (tow>tow_p+302400.0) tow-=604800.0;
	return *adjt.gpst2time(week,tow);
}
/* adjust daily rollover of time ------------------------------------------------------------------ */
gtime_t javad::adjday(gtime_t ttt,double ttod){
	double tod_p;
	gtime_t adjt;

	ttt.time2epoch();
	tod_p=ttt.ep[3]*3600.0+ttt.ep[4]*60.0+ttt.ep[5];
	if (ttod<tod_p-43200.0) ttod+=86400.0;
	else if (ttod>tod_p+43200.0) ttod-=86400.0;

	adjt.ep[0]=ttt.ep[0];adjt.ep[1]=ttt.ep[1];adjt.ep[2]=ttt.ep[2];
	adjt.ep[3]=adjt.ep[4]=adjt.ep[5]=0.0;
	return *adjt.epoch2time(adjt.ep)->timeadd(ttod);
}
/* set time tag ----------------------------------------------------------------------------------- */
int javad::settag(int num){

	if (obuf.data[num].time.time!=0&&fabs(obuf.data[num].time.timediff(time))>5E-4) {
		obuf.data[num].time.time2str(4); time.time2str(4);
		return 0;
	}
	obuf.data[num].time=time;
	return 1;
}
/* flush observation data buffer ------------------------------------------------------------------ */
int javad::flushobuf(){
	gtime_t ttt0;
	int i,j,n=0;

	/* copy observation data buffer */
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		if (!satsys(obuf.data[i].sat,NULL)) continue;
		if (obuf.data[i].time.time==0) continue;
		obs.data[n++]=obuf.data[i];
	}
	obs.n=n;

	/* clear observation data buffer */
	for (i=0; i<MAXOBS; i++) {
		obuf.data[i].time=ttt0;
		for (j=0; j<NFREQ+NEXOBS; j++) {
			obuf.data[i].L[j]=obuf.data[i].P[j]=0.0;
			obuf.data[i].D[j]=0.0;
			obuf.data[i].SNR[j]=obuf.data[i].LLI[j]=0;
			obuf.data[i].code[j]=CODE_NONE;
		}
	}
	for (i=0; i<MAXSAT; i++) prCA[i]=dpCA[i]=0.0;
	return n>0 ? 1 : 0;
}
/* sync javad message ----------------------------------------------------------------------------- */
int javad::sync_javad(unsigned char data){
	unsigned char p=buff[0];

	buff[0]=buff[1]; buff[1]=buff[2]; buff[2]=buff[3]; buff[3]=buff[4];
	buff[4]=data;

	/* sync message header {\r|\n}IIHHH (II:id,HHH: hex length) */
	return (p=='\r'||p=='\n')&&ISTXT(buff[0])&&ISTXT(buff[1])&&
		ISHEX(buff[2])&&ISHEX(buff[3])&&ISHEX(buff[4]);
}
/* clear buffer ----------------------------------------------------------------------------------- */
void javad::clearbuff(){
	int i;
	for (i=0; i<5; i++) buff[i]=0;
	len=nbyte=0;
}

/* decode [~~] receiver time ---------------------------------------------------------------------- */
int javad::decode_RT(){
	gtime_t ttt;
	unsigned char *p=buff+5;

	if (!checksum()) {
		return -1;
	}
	if (len<10) {
		return -1;
	}
	tod=U4(p);

	if (time.time==0) return 0;

	/* update receiver time */
	ttt=time;
	if (tbase>=1) ttt.gpst2utc(); /* gpst->utc */
	ttt=adjday(ttt,tod*0.001);
	if (tbase>=1) ttt.utc2gpst(); /* utc->gpst */
	time=ttt;

	if (outtype) {
		msgtype+=ttt.time2str(3);
	}
	/* flush observation data buffer */
	return flushobuf();
}
/* decode [::] epoch time ------------------------------------------------------------------------- */
int javad::decode_ET(){
	unsigned char *p=buff+5;

	if (!checksum()) {
		return -1;
	}
	if (len<10) {
		return -1;
	}
	if (tod!=(int)U4(p)) {
		return -1;
	}
	tod=-1; /* end of epoch */

				 /* flush observation data buffer */
	return flushobuf();
}
/* decode [RD] receiver date ---------------------------------------------------------------------- */
int javad::decode_RD(){
	char msg[100];
	unsigned char *p=buff+5;

	if (!checksum()) {
		return -1;
	}
	if (len<11) {
		return -1;
	}
	time.ep[0]=U2(p); p+=2;
	time.ep[1]=U1(p); p+=1;
	time.ep[2]=U1(p); p+=1;
	tbase=U1(p);

	if (outtype) {
		sprintf(msg, " %04.0f/%02.0f/%02.0f base=%d",time.ep[0],time.ep[1],time.ep[2],tbase);
		msgtype+=msg;
	}
	if (tod<0) {
		return 0;
	}
	time.epoch2time(time.ep)->timeadd(tod*0.001);
	if (tbase>=1) time.utc2gpst(); /* utc->gpst */

	return 0;
}
/* decode [SI] satellite indices ------------------------------------------------------------------ */
int javad::decode_SI(){
	int i,usi,sat;
	string str;
	unsigned char *p=buff+5;

	if (!checksum()) {
		return -1;
	}
	obuf.n=len-6;

	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		usi=U1(p); p+=1;

		if (usi<=  0) sat=0;                      /* ref [5] table 3-6 */
		else if (usi<= 37) sat=satno(SYS_GPS,usi);     /*   1- 37: GPS */
		else if (usi<= 70) sat=255;                    /*  38- 70: GLONASS */
		else if (usi<=119) sat=satno(SYS_GAL,usi-70);  /*  71-119: GALILEO */
		else if (usi<=142) sat=satno(SYS_SBS,usi);     /* 120-142: SBAS */
		else if (usi<=192) sat=0;
		else if (usi<=197) sat=satno(SYS_QZS,usi);     /* 193-197: QZSS */
		else if (usi<=210) sat=0;
		else if (usi<=240) sat=satno(SYS_CMP,usi-210); /* 211-240: BeiDou */
		else               sat=0;

		obuf.data[i].time=time;
		obuf.data[i].sat=sat;

		/* glonass fcn (frequency channel number) */
		if (sat==255) freqn[i]=usi-45;
	}

	if (outtype) {
		msgtype+=" nsat="+int2str(2," ",obuf.n,str);
	}
	return 0;
}
/* decode [NN] glonass satellite system numbers --------------------------------------------------- */
int javad::decode_NN(){
	unsigned char *p=buff+5;
	string str;
	int i,n,ns,slot,sat,index[MAXOBS];

	if (!checksum()) {
		return -1;
	}
	for (i=n=0; i<obuf.n&&i<MAXOBS; i++) {
		if (obuf.data[i].sat==255) index[n++]=i;
	}
	ns=len-6;

	for (i=0; i<ns&&i<n; i++) {
		slot=U1(p); p+=1;
		sat=satno(SYS_GLO,slot);
		obuf.data[index[i]].sat=sat;
	}
	if (outtype) {
		msgtype+=" nsat="+int2str(2," ",obuf.n,str);
	}
	return 0;
}
/* decode [GA] gps almanac ------------------------------------------------------------------------ */
int javad::decode_GA(){
	return 0;
}
/* decode [NA] glonass almanac -------------------------------------------------------------------- */
int javad::decode_NA(){
	return 0;
}
/* decode [EA] galileo almanac -------------------------------------------------------------------- */
int javad::decode_EA(){
	return 0;
}
/* decode [WA] waas almanac ----------------------------------------------------------------------- */
int javad::decode_WA(){
	return 0;
}
/* decode [QA] qzss almanac ----------------------------------------------------------------------- */
int javad::decode_QA(){
	return 0;
}
/* decode gps/galileo/qzss ephemeris -------------------------------------------------------------- */
int javad::decode_eph(int sys){
	eph_t eph;
	double toc,sqrtA,tt;
	char msg[100];
	int prn,tow,fflag,week;
	unsigned char *p=buff+5;

	prn       =U1(p);        p+=1;
	tow       =U4(p);        p+=4;
	fflag      =U1(p);        p+=1;
	eph.iodc  =I2(p);        p+=2;
	toc       =I4(p);        p+=4;
	eph.sva   =I1(p);        p+=1;
	eph.svh   =U1(p);        p+=1;
	week      =I2(p);        p+=2;
	eph.tgd[0]=R4(p);        p+=4;
	eph.f2    =R4(p);        p+=4;
	eph.f1    =R4(p);        p+=4;
	eph.f0    =R4(p);        p+=4;
	eph.toes  =I4(p);        p+=4;
	eph.iode  =I2(p);        p+=2;
	sqrtA     =R8(p);        p+=8;
	eph.e     =R8(p);        p+=8;
	eph.M0    =R8(p)*SC2RAD; p+=8;
	eph.OMG0  =R8(p)*SC2RAD; p+=8;
	eph.i0    =R8(p)*SC2RAD; p+=8;
	eph.omg   =R8(p)*SC2RAD; p+=8;
	eph.deln  =R4(p)*SC2RAD; p+=4;
	eph.OMGd  =R4(p)*SC2RAD; p+=4;
	eph.idot  =R4(p)*SC2RAD; p+=4;
	eph.crc   =R4(p);        p+=4;
	eph.crs   =R4(p);        p+=4;
	eph.cuc   =R4(p);        p+=4;
	eph.cus   =R4(p);        p+=4;
	eph.cic   =R4(p);        p+=4;
	eph.cis   =R4(p);        p+=4;
	eph.A     =sqrtA*sqrtA;

	if (outtype) {
		sprintf(msg, " prn=%3d iode=%3d iodc=%3d toes=%6.0f",prn,eph.iode,
			eph.iodc,eph.toes);
		msgtype+=msg;
	}
	if (sys==SYS_GPS||sys==SYS_QZS) {
		if (!(eph.sat=satno(sys,prn))) {
			return -1;
		}
		eph.flag=(fflag>>1)&1;
		eph.code=(fflag>>2)&3;
		eph.fit =fflag&1;
		eph.week=adjgpsweek(week);
		eph.toe.gpst2time(eph.week,eph.toes);

		/* for week-handover problem */
		tt=eph.toe.timediff(time);
		if (tt<-302400.0) eph.week++;
		else if (tt> 302400.0) eph.week--;
		eph.toe.gpst2time(eph.week,eph.toes);

		eph.toc.gpst2time(eph.week,toc);
		eph.ttr=adjweek(eph.toe,tow);
	}
	else if (sys==SYS_GAL) {
		if (!(eph.sat=satno(sys,prn))) {
			return -1;
		}
		eph.tgd[1]=R4(p); p+=4;    /* BGD: E1-E5A (s) */
		eph.tgd[2]=R4(p); p+=4+13; /* BGD: E1-E5B (s) */
		eph.code  =U1(p);          /* navtype: 0:E1B(INAV),1:E5A(FNAV) */
								   /*          3:GIOVE E1B,4:GIOVE E5A */

								   /* gst week -> gps week */
		eph.week=week+1024;
		eph.toe.gpst2time(eph.week,eph.toes);

		/* for week-handover problem */
		tt=eph.toe.timediff(time);
		if (tt<-302400.0) eph.week++;
		else if (tt> 302400.0) eph.week--;
		eph.toe.gpst2time(eph.week,eph.toes);

		eph.toc.gpst2time(eph.week,toc);
		eph.ttr=adjweek(eph.toe,tow);
	}
	else if (sys==SYS_CMP) {
		if (!(eph.sat=satno(sys,prn))) {
			return -1;
		}
		eph.tgd[1]=R4(p); p+=4;    /* TGD2 (s) */
		eph.code  =U1(p);          /* type of nav data */

		eph.week=week;
		eph.toe.bdt2time(week,eph.toes); /* bdt -> gpst */
		eph.toc.bdt2time(week,toc);      /* bdt -> gpst */
		eph.ttr=adjweek(eph.toe,tow);
	}
	else return 0;

	if (opt.find("-EPHALL")==string::npos) {
		if (nav.eph[eph.sat-1].iode==eph.iode&&
			nav.eph[eph.sat-1].iodc==eph.iodc) return 0; /* unchanged */
	}
	nav.eph[eph.sat-1]=eph;
	ephsat=eph.sat;
	return 2;
}
/* decode [GE] gps ephemeris ---------------------------------------------------------------------- */
int javad::decode_GE(){
	if (!checksum()) {
		return -1;
	}
	if (len<128) {
		return -1;
	}
	return decode_eph(SYS_GPS);
}
/* decode [NE] glonass ephemeris ------------------------------------------------------------------ */
int javad::decode_NE(){
	geph_t geph;
	double tt;
	char msg[100];
	int prn,tk,tb;
	unsigned char *p=buff+5;

	if (!checksum()) {
		return -1;
	}
	if (len>=85) { /* firmware v 2.6.0 [2] */
		prn        =U1(p);     p+=1;
		geph.frq   =I1(p);     p+=1+2;
		tk         =I4(p);     p+=4;
		tb         =I4(p);     p+=4;
		geph.svh   =U1(p)&0x7; p+=1;
		geph.age   =U1(p);     p+=1+1;
		geph.pos[0]=R8(p)*1E3; p+=8;
		geph.pos[1]=R8(p)*1E3; p+=8;
		geph.pos[2]=R8(p)*1E3; p+=8;
		geph.vel[0]=R4(p)*1E3; p+=4;
		geph.vel[1]=R4(p)*1E3; p+=4;
		geph.vel[2]=R4(p)*1E3; p+=4;
		geph.acc[0]=R4(p)*1E3; p+=4;
		geph.acc[1]=R4(p)*1E3; p+=4;
		geph.acc[2]=R4(p)*1E3; p+=4+8;
		geph.taun  =R4(p);     p+=4;
		geph.gamn  =R4(p);     p+=4;
	}
	else {
		return -1;
	}
	if (len>=93) { /* firmware v 3.2.0 [1] */
		geph.dtaun =R4(p); p+=4;
		geph.sva   =U1(p);
	}
	if (outtype) {
		sprintf(msg, " prn=%2d frq=%2d tk=%6d tb=%4d",prn,geph.frq,tk,tb);
		msgtype+=msg;
	}
	if (!(geph.sat=satno(SYS_GLO,prn))) {
		return 0;
	}
	if (time.time==0) return 0;
	geph.iode=(tb/900)&0x7F;
	geph.toe=adjday(time,tb-10800.0); geph.toe.utc2gpst();
	geph.tof=adjday(time,tk-10800.0); geph.tof.utc2gpst();

	/* check illegal ephemeris by toe */
	tt=time.timediff(geph.toe);
	if (fabs(tt)>3600.0) {
		return 0;
	}
	/* check illegal ephemeris by frequency number consistency */
	if (nav.geph[prn-MINPRNGLO].toe.time&&
		geph.frq!=nav.geph[prn-MINPRNGLO].frq) {
		return -1;
	}
	if (opt.find("-EPHALL")==string::npos) {
		if (fabs(geph.toe.timediff(nav.geph[prn-MINPRNGLO].toe))<1.0&&
			geph.svh==nav.geph[prn-MINPRNGLO].svh) return 0; /* unchanged */
	}
	nav.geph[prn-1]=geph;
	ephsat=geph.sat;
	return 2;
}
/* decode [EN] galileo ephemeris ------------------------------------------------------------------ */
int javad::decode_EN(){
	if (!checksum()) {
		return -1;
	}
	if (len<150) {
		return -1;
	}
	return decode_eph(SYS_GAL);
}
/* decode [WE] sbas ephemeris --------------------------------------------------------------------- */
int javad::decode_WE(){
	seph_t seph;
	unsigned int ttod,tow;
	char msg[50];
	int i,prn,week;
	unsigned char *p=buff+5;

	if (!checksum()) {
		return -1;
	}
	if (len<44) {
		return -1;
	}
	prn     =U1(p); p+=1+1+1;
	seph.sva=U1(p); p+=1;
	ttod     =U4(p); p+=4;
	for (i=0; i<3; i++) { seph.pos[i]=R8(p); p+=8; }
	for (i=0; i<3; i++) { seph.vel[i]=R4(p); p+=4; }
	for (i=0; i<3; i++) { seph.acc[i]=R4(p); p+=4; }
	seph.af0 =R4(p); p+=4;
	seph.af1 =R4(p); p+=4;
	tow      =U4(p); p+=4;
	week     =U2(p);

	if (outtype) {
		sprintf(msg, " prn=%3d tod=%6d",prn,ttod);
		msgtype+=msg;
	}
	if (!(seph.sat=satno(SYS_SBS,prn))) {
		return -1;
	}
	seph.tof.gpst2time(adjgpsweek(week),tow);
	seph.t0=adjday(seph.tof,ttod);

	if (opt.find("-EPHALL")==string::npos) {
		if (fabs(seph.t0.timediff(nav.seph[prn-MINPRNSBS].t0))<1.0&&
			seph.sva==nav.seph[prn-MINPRNSBS].sva) return 0; /* unchanged */
	}
	nav.seph[prn-MINPRNSBS]=seph;
	ephsat=seph.sat;
	return 2;
}
/* decode [QE] qzss ephemeris --------------------------------------------------------------------- */
int javad::decode_QE(){
	if (!checksum()) {
		return -1;
	}
	if (len<128) {
		return -1;
	}
	return decode_eph(SYS_QZS);
}
/* decode [CN] beidou ephemeris ------------------------------------------------------------------- */
int javad::decode_CN(){
	if (!checksum()) {
		return -1;
	}
	if (len<133) {
		return -1;
	}
	return decode_eph(SYS_CMP);
}
/* decode [UO] gps utc time parameters ------------------------------------------------------------ */
int javad::decode_UO(){
	unsigned char *p=buff+5;

	if (!checksum()) {
		return -1;
	}
	if (len<29) {
		return -1;
	}
	nav.utc_gps[0]=R8(p); p+=8;
	nav.utc_gps[1]=R4(p); p+=4;
	nav.utc_gps[2]=U4(p); p+=4;
	nav.utc_gps[3]=adjgpsweek((int)U2(p)); p+=2;
	nav.leaps     =I1(p);
	return 9;
}
/* decode [NU] glonass utc and gps time parameters ------------------------------------------------ */
int javad::decode_NU(){
	return 0;
}
/* decode [EU] galileo utc and gps time parameters ------------------------------------------------ */
int javad::decode_EU(){
	return 0;
}
/* decode [WU] waas utc time parameters ----------------------------------------------------------- */
int javad::decode_WU(){
	return 0;
}
/* decode [QU] qzss utc and gps time parameters --------------------------------------------------- */
int javad::decode_QU(){
	return 0;
}
/* decode [IO] ionospheric parameters ------------------------------------------------------------- */
int javad::decode_IO(){
	int i;
	unsigned char *p=buff+5;

	if (!checksum()) {
		return -1;
	}
	if (len<44) {
		return -1;
	}
	p+=4+2;
	for (i=0; i<8; i++) {
		nav.ion_gps[i]=R4(p); p+=4;
	}
	return 9;
}
/* decode L1 NAV data ----------------------------------------------------------------------------- */
int javad::decode_L1nav(unsigned char *buff,int len,int sat){
	decode_frame dec;
	unsigned char *frm,*p;
	unsigned int word;
	int i,j,sys,week,id=(U4((unsigned char*)buff+4)>>8)&7;

	if (id<1||5<id) {
		return 0;
	}
	frm=subfrm[sat-1];

	for (i=0,p=frm+(id-1)*30; i<10; i++) {
		word=U4((unsigned char*)buff+i*4)>>6;
		for (j=16; j>=0; j-=8) {
			*p++=(word>>j)&0xFF;
		}
	}
	if (id==3) { /* ephemeris */
		dec.eph.sat=sat;
		if (dec.decode(frm)!=1||
			dec.decode(frm+30)!=2||
			dec.decode(frm+60)!=3) {
			return 0;
		}
		if (opt.find("-EPHALL")==string::npos) {
			if (dec.eph.iode==nav.eph[sat-1].iode&&
				dec.eph.iodc==nav.eph[sat-1].iodc) return 0; /* unchanged */
		}
		nav.eph[sat-1]=dec.eph;
		ephsat=sat;
		return 2;
	}
	if (id==4) { /* almanac or ion/utc parameters */
		if (dec.decode(frm+90)!=4) {
			return 0;
		}
		if (norm(dec.ion,8)==0.0||norm(dec.utc,4)==0.0||time.time==0) {
			return 0;
		}
		sys=satsys(sat,NULL);
		time.time2gpst(&week);
		dec.utc[3]+=floor((week-dec.utc[3])/256.0+0.5)*256.0;

		if (sys==SYS_GPS) {
			for (i=0; i<8; i++) nav.ion_gps[i]=dec.ion[i];
			for (i=0; i<4; i++) nav.utc_gps[i]=dec.utc[i];
			nav.leaps=dec.leaps;
			return 9;
		}
		if (sys==SYS_QZS) {
			for (i=0; i<8; i++) nav.ion_qzs[i]=dec.ion[i];
			for (i=0; i<4; i++) nav.utc_qzs[i]=dec.utc[i];
			nav.leaps=dec.leaps;
			return 9;
		}
	}
	return 0;
}
/* decode raw L2C CNAV data ----------------------------------------------------------------------- */
/* DO NOTHING !!! --------------------------------------------------------------------------------- */
int javad::decode_L2nav(unsigned char *buff,int len,int sat){
	unsigned char msg[1024]={ 0 };
	int i,j,preamb,prn,msgid,tow,alert;

	for (i=0; i<len; i++) for (j=0; j<4; j++) {
		msg[3-j+i*4]=buff[j+i*4];
	}
	i=0;
	preamb=getbitu(msg,i,8); i+= 8;
	prn   =getbitu(msg,i,6); i+= 6;
	msgid =getbitu(msg,i,6); i+= 6;
	tow   =getbitu(msg,i,17); i+=17;
	alert =getbitu(msg,i,1); i+= 1;

	if (preamb!=PREAMB_CNAV) {
		return -1;
	}

	return 0;
}
/* decode raw L5 CNAV data ------------------------------------------------------------------------ */
/* DO NOTHING !!! --------------------------------------------------------------------------------- */
int javad::decode_L5nav(unsigned char *buff,int len,int sat){
	unsigned char msg[1024]={ 0 };
	int i,j,preamb,prn,msgid,tow,alert;

	for (i=0; i<len; i++) for (j=0; j<4; j++) {
		msg[3-j+i*4]=buff[j+i*4];
	}
	i=0;
	preamb=getbitu(msg,i,8); i+= 8;
	prn   =getbitu(msg,i,6); i+= 6;
	msgid =getbitu(msg,i,6); i+= 6;
	tow   =getbitu(msg,i,17); i+=17;
	alert =getbitu(msg,i,1); i+= 1;

	if (preamb!=PREAMB_CNAV) {
		return -1;
	}

	return 0;
}
/* decode raw L1C CNAV2 data ---------------------------------------------------------------------- */
int javad::decode_L1Cnav(unsigned char *buff,int len,int sat){
	return 0;
}
/* decode [*D] raw navigation data ---------------------------------------------------------------- */
int javad::decode_nD(int sys){
	int i,n,siz,sat,prn,stat=0;
	unsigned char *p=buff+5;

	if (!checksum()) {
		return -1;
	}
	siz=U1(p); p+=1;
	n=(len-7)/siz;

	if (n<=0) {
		return -1;
	}
	for (i=0; i<n; i++,p+=siz) {
		prn=U1(p);
		if (!(sat=satno(sys,prn))) {
			continue;
		}
		stat=decode_L1nav(p+2,0,sat);
	}
	return stat;
}
/* decode [*d] raw navigation data ---------------------------------------------------------------- */
int javad::decode_nd(int sys){
	unsigned char *p=buff+5;
	char msg[100];
	int sat,prn,ttt,type,lll;

	if (!checksum()) {
		return -1;
	}

	prn =U1(p); p+=1;
	ttt=U4(p); p+=4;
	type=U1(p); p+=1;
	lll =U1(p); p+=1;
	if (len!=13+lll*4) {
		return -1;
	}
	if (outtype) {
		sprintf(msg, " prn=%3d time=%7d type=%d",prn,ttt,type);
		msgtype+=msg;
	}
	if (!(sat=satno(sys,prn))) {
		return 0;
	}

	switch (type) {
		case 0: return decode_L1nav(p,lll,sat); /* L1  NAV */
		case 1: return decode_L2nav(p,lll,sat); /* L2C CNAV */
		case 2: return decode_L5nav(p,lll,sat); /* L5  CNAV */
		case 3: return decode_L1Cnav(p,lll,sat); /* L1C CNAV2 */
		case 4: break;
	}
	return 0;
}
/* decode [LD] glonass raw navigation data -------------------------------------------------------- */
int javad::decode_LD(){
	return 0;
}
/* decode [lD] glonass raw navigation data -------------------------------------------------------- */
int javad::decode_lD(){
	geph_t geph;
	unsigned char *p=buff+5;
	char msg[100];
	int i,sat,prn,frq,ttt,type,lll,id;

	if (!checksum()) {
		return -1;
	}

	prn =U1(p); p+=1;
	frq =I1(p); p+=1;
	ttt=U4(p); p+=4;
	type=U1(p); p+=1;
	lll =U1(p); p+=1;

	if (len!=14+lll*4) {
		return -1;
	}
	if (outtype) {
		sprintf(msg, " prn=%2d frq=%2d time=%7d type=%d",prn,frq,ttt,type);
		msgtype += msg;
	}
	if (!(sat=satno(SYS_GLO,prn))) {
		return 0;
	}
	if (type!=0) {
		return 0;
	}
	if ((id=(U4(p)>>20)&0xF)<1) return 0;

	/* get 77 bit (25x3+2) in frame without hamming and time mark */
	for (i=0; i<4; i++) {
		setbitu(subfrm[sat-1]+(id-1)*10,i*25,i<3 ? 25 : 2,
			U4(p+4*i)>>(i<3 ? 0 : 23));
	}
	if (id!=4) return 0;

	/* decode glonass ephemeris strings */
	geph.tof=time;
	if (!decode_glostr(sat,geph)||geph.sat!=sat) return -1;
	geph.frq=frq;

	if (opt.find("-EPHALL")==string::npos) {
		if (geph.iode==nav.geph[prn-1].iode) return 0; /* unchanged */
	}
	nav.geph[prn-1]=geph;
	ephsat=sat;
	return 2;
}
/* decode [WD] waas raw navigation data ----------------------------------------------------------- */
int javad::decode_WD(){
	int i,prn,tow,tow_p,week;
	char msg[40];
	unsigned char *p=buff+5;

	if (!checksum()) {
		return -1;
	}
	if (len<45) {
		return -1;
	}

	prn=U1(p); p+=1;
	tow=U4(p); p+=4+2;

	if (outtype) {
		sprintf(msg, " prn=%3d tow=%6d",prn,tow);
		msgtype += msg;
	}
	if ((prn<MINPRNSBS||MAXPRNSBS<prn)&&(prn<MINPRNQZS||MAXPRNQZS<prn)) {
		return 0;
	}
	sbsmsg.prn=prn;
	sbsmsg.tow=tow;

	if (time.time==0) {
		sbsmsg.week=0;
	}
	else {
		tow_p=(int)time.time2gpst(&week);
		if (tow<tow_p-302400.0) week++;
		else if (tow>tow_p+302400.0) week--;
		sbsmsg.week=week;
	}
	for (i=0; i<29; i++) sbsmsg.msg[i]=*p++;
	sbsmsg.msg[28]&=0xC0;
	return 3;
}
/* decode [R*] pseudoranges ----------------------------------------------------------------------- */
int javad::decode_Rx(char code){
	double pr,prm;
	int i,j,freq,type,sat,sys;
	unsigned char *p=buff+5;

	if (!is_meas(code)||tod<0||obuf.n==0) return 0;

	if (!checksum()) {
		return -1;
	}
	if (len!=obuf.n*8+6) {
		return -1;
	}
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		pr=R8(p); p+=8; if (pr==0.0) continue;

		sat=obuf.data[i].sat;
		if (!(sys=satsys(sat,NULL))) continue;

		prm=pr*CLIGHT;

		if (code=='C') prCA[sat-1]=prm;

		if ((freq=tofreq(code,sys,type))<0) continue;

		if ((j=checkpri(sys,type,freq))>=0) {
			if (!settag(i)) continue;
			obuf.data[i].P[j]=prm;
			obuf.data[i].code[j]=type;
		}
	}
	return 0;
}
/* decode [r*] short pseudoranges ----------------------------------------------------------------- */
int javad::decode_rx(char code){
	double prm;
	int i,j,pr,freq,type,sat,sys;
	unsigned char *p=buff+5;

	if (!is_meas(code)||tod<0||obuf.n==0) return 0;

	if (!checksum()) {
		return -1;
	}
	if (len!=obuf.n*4+6) {
		return -1;
	}
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		pr=I4(p); p+=4;
		sat=obuf.data[i].sat;
		if (!(sys=satsys(sat,NULL))) continue;

		if (pr==0x7FFFFFFF) {
			continue;
		}
		if (sys==SYS_SBS) prm=(pr*1E-11+0.115)*CLIGHT;
		else if (sys==SYS_QZS) prm=(pr*2E-11+0.125)*CLIGHT; /* [3] */
		else if (sys==SYS_CMP) prm=(pr*2E-11+0.105)*CLIGHT; /* [4] */
		else if (sys==SYS_GAL) prm=(pr*1E-11+0.090)*CLIGHT; /* [3] */
		else                   prm=(pr*1E-11+0.075)*CLIGHT;

		if (code=='c') prCA[sat-1]=prm;

		if ((freq=tofreq(code,sys,type))<0) continue;

		if ((j=checkpri(sys,type,freq))>=0) {
			if (!settag(i)) continue;
			obuf.data[i].P[j]=prm;
			obuf.data[i].code[j]=type;
		}
	}
	return 0;
}
/* decode [*R] relative pseudoranges -------------------------------------------------------------- */
int javad::decode_xR(char code){
	float pr;
	int i,j,freq,type,sat,sys;
	unsigned char *p=buff+5;

	if (!is_meas(code)||tod<0||obuf.n==0) return 0;

	if (!checksum()) {
		return -1;
	}
	if (len!=obuf.n*4+6) {
		return -1;
	}
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		pr=R4(p); p+=4; if (pr==0.0) continue;

		sat=obuf.data[i].sat;
		if (!(sys=satsys(sat,NULL))||prCA[sat-1]==0.0) continue;

		if ((freq=tofreq(code,sys,type))<0) continue;

		if ((j=checkpri(sys,type,freq))>=0) {
			if (!settag(i)) continue;
			obuf.data[i].P[j]=pr*CLIGHT+prCA[sat-1];
			obuf.data[i].code[j]=type;
		}
	}
	return 0;
}
/* decode [*r] short relative pseudoranges -------------------------------------------------------- */
int javad::decode_xr(char code){
	double prm;
	short pr;
	int i,j,freq,type,sat,sys;
	unsigned char *p=buff+5;

	if (!is_meas(code)||tod<0||obuf.n==0) return 0;

	if (!checksum()) {
		return -1;
	}
	if (len!=obuf.n*2+6) {
		return -1;
	}
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		pr=I2(p); p+=2; if (pr==(short)0x7FFF) continue;

		sat=obuf.data[i].sat;
		if (!(sys=satsys(sat,NULL))||prCA[sat-1]==0.0) continue;

		prm=(pr*1E-11+2E-7)*CLIGHT+prCA[sat-1];

		if ((freq=tofreq(code,sys,type))<0) continue;

		if ((j=checkpri(sys,type,freq))>=0) {
			if (!settag(i)) continue;
			obuf.data[i].P[j]=prm;
			obuf.data[i].code[j]=type;
		}
	}
	return 0;
}
/* decode [P*] carrier phases --------------------------------------------------------------------- */
int javad::decode_Px(char code){
	double cp;
	int i,j,freq,type,sys;
	unsigned char *p=buff+5;

	if (!is_meas(code)||tod<0||obuf.n==0) return 0;

	if (!checksum()) {
		return -1;
	}
	if (len!=obuf.n*8+6) {
		return -1;
	}
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		cp=R8(p); p+=8; if (cp==0.0) continue;

		if (!(sys=satsys(obuf.data[i].sat,NULL))) continue;

		if ((freq=tofreq(code,sys,type))<0) continue;

		if ((j=checkpri(sys,type,freq))>=0) {
			if (!settag(i)) continue;
			obuf.data[i].L[j]=cp;
			obuf.data[i].code[j]=type;
		}
	}
	return 0;
}
/* decode [p*] short carrier phases --------------------------------------------------------------- */
int javad::decode_px(char code){
	unsigned int cp;
	int i,j,freq,type,sys;
	unsigned char *p=buff+5;

	if (!is_meas(code)||tod<0||obuf.n==0) return 0;

	if (!checksum()) {
		return -1;
	}
	if (len!=obuf.n*4+6) {
		return -1;
	}
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		cp=U4(p); p+=4; if (cp==0xFFFFFFFF) continue;

		if (!(sys=satsys(obuf.data[i].sat,NULL))) continue;

		if ((freq=tofreq(code,sys,type))<0) continue;

		if ((j=checkpri(sys,type,freq))>=0) {
			if (!settag(i)) continue;
			obuf.data[i].L[j]=cp/1024.0;
			obuf.data[i].code[j]=type;
		}
	}
	return 0;
}
/* decode [*P] short relative carrier phases ------------------------------------------------------ */
int javad::decode_xP(char code){
	double cp,rcp,fn;
	int i,j,freq,type,sat,sys;
	unsigned char *p=buff+5;

	if (!is_meas(code)||tod<0||obuf.n==0) return 0;

	if (!checksum()) {
		return -1;
	}
	if (len!=obuf.n*4+6) {
		return -1;
	}
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		rcp=R4(p); p+=4; if (rcp==0.0) continue;

		sat=obuf.data[i].sat;
		if (!(sys=satsys(sat,NULL))||prCA[sat-1]==0.0) continue;

		if ((freq=tofreq(code,sys,type))<0) continue;

		if ((j=checkpri(sys,type,freq))>=0) {
			if (!settag(i)) continue;

			fn=sys==SYS_GLO ? freq_glo(freq,freqn[i]) : CLIGHT/WaveLengths[freq];
			cp=(rcp+prCA[sat-1]/CLIGHT)*fn;

			obuf.data[i].L[j]=cp;
			obuf.data[i].code[j]=type;
		}
	}
	return 0;
}
/* decode [*p] short relative carrier phases ------------------------------------------------------ */
int javad::decode_xp(char code){
	double cp,fn;
	int i,j,rcp,freq,type,sat,sys;
	unsigned char *p=buff+5;

	if (!is_meas(code)||tod<0||obuf.n==0) return 0;

	if (!checksum()) {
		return -1;
	}
	if (len!=obuf.n*4+6) {
		return -1;
	}
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		rcp=I4(p); p+=4; if (rcp==0x7FFFFFFF) continue;

		sat=obuf.data[i].sat;
		if (!(sys=satsys(sat,NULL))||prCA[sat-1]==0.0) continue;

		if ((freq=tofreq(code,sys,type))<0) continue;

		if ((j=checkpri(sys,type,freq))>=0) {
			if (!settag(i)) continue;

			fn=sys==SYS_GLO ? freq_glo(freq,freqn[i]) : CLIGHT/WaveLengths[freq];
			cp=(rcp*P2_40+prCA[sat-1]/CLIGHT)*fn;

			obuf.data[i].L[j]=cp;
			obuf.data[i].code[j]=type;
		}
	}
	return 0;
}
/* decode [D*] doppler ---------------------------------------------------------------------------- */
int javad::decode_Dx(char code){
	double dop;
	int i,j,dp,freq,type,sat,sys;
	unsigned char *p=buff+5;

	if (!is_meas(code)||tod<0||obuf.n==0) return 0;

	if (!checksum()) {
		return -1;
	}
	if (len!=obuf.n*4+6) {
		return -1;
	}
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		dp=I4(p); p+=4; if (dp==0x7FFFFFFF) continue;

		sat=obuf.data[i].sat;
		if (!(sys=satsys(sat,NULL))) continue;

		dop=-dp*1E-4;

		if (code=='C') dpCA[sat-1]=dop;

		if ((freq=tofreq(code,sys,type))<0) continue;

		if ((j=checkpri(sys,type,freq))>=0) {
			if (!settag(i)) continue;
			obuf.data[i].D[j]=(float)dop;
		}
	}
	return 0;
}
/* decode [*d] short relative doppler ------------------------------------------------------------- */
int javad::decode_xd(char code){
	double dop,f1,fn;
	short rdp;
	int i,j,freq,type,sat,sys;
	unsigned char *p=buff+5;

	if (!is_meas(code)||tod<0||obuf.n==0) return 0;

	if (!checksum()) {
		return -1;
	}
	if (len!=obuf.n*2+6) {
		return -1;
	}
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		rdp=I2(p); p+=2; if (rdp==(short)0x7FFF) continue;

		sat=obuf.data[i].sat;
		if (!(sys=satsys(sat,NULL))||dpCA[sat-1]==0.0) continue;

		if ((freq=tofreq(code,sys,type))<0) continue;

		if ((j=checkpri(sys,type,freq))>=0) {
			if (!settag(i)) continue;
			f1=sys==SYS_GLO ? freq_glo(0,freqn[i]) : CLIGHT/WaveLengths[0];
			fn=sys==SYS_GLO ? freq_glo(freq,freqn[i]) : CLIGHT/WaveLengths[freq];
			dop=(-rdp+dpCA[sat-1]*1E4)*fn/f1*1E-4;

			obuf.data[i].D[j]=(float)dop;
		}
	}
	return 0;
}
/* decode [E*] carrier to noise ratio ------------------------------------------------------------- */
int javad::decode_Ex(char code){
	unsigned char cnr;
	int i,j,freq,type,sys;
	unsigned char *p=buff+5;

	if (!is_meas(code)||tod<0||obuf.n==0) return 0;

	if (!checksum()) {
		return -1;
	}
	if (len!=obuf.n+6) {
		return -1;
	}
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		cnr=U1(p); p+=1; if (cnr==255) continue;

		if (!(sys=satsys(obuf.data[i].sat,NULL))) continue;

		if ((freq=tofreq(code,sys,type))<0) continue;

		if ((j=checkpri(sys,type,freq))>=0) {
			if (!settag(i)) continue;
			obuf.data[i].SNR[j]=(unsigned char)(cnr*4.0+0.5);
		}
	}
	return 0;
}
/* decode [*E] carrier to noise ratio x 4 --------------------------------------------------------- */
int javad::decode_xE(char code){
	unsigned char cnr;
	int i,j,freq,type,sys;
	unsigned char *p=buff+5;

	if (!is_meas(code)||tod<0||obuf.n==0) return 0;

	if (!checksum()) {
		return -1;
	}
	if (len!=obuf.n+6) {
		return -1;
	}
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		cnr=U1(p); p+=1; if (cnr==255) continue;

		if (!(sys=satsys(obuf.data[i].sat,NULL))) continue;

		if ((freq=tofreq(code,sys,type))<0) continue;

		if ((j=checkpri(sys,type,freq))>=0) {
			if (!settag(i)) continue;
			obuf.data[i].SNR[j]=cnr;
		}
	}
	return 0;
}
/* decode [F*] signal lock loop flags ------------------------------------------------------------- */
int javad::decode_Fx(char code){
	unsigned short flags;
	int i,j,freq,type,sat,sys;
	unsigned char *p=buff+5;

	if (!is_meas(code)||tod<0||obuf.n==0) return 0;

	if (!checksum()) {
		return -1;
	}
	if (len!=obuf.n*2+6) {
		return -1;
	}
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		flags=U2(p); p+=1; if (flags==0xFFFF) continue;

		sat=obuf.data[i].sat;
		if (!(sys=satsys(sat,NULL))) continue;

		if ((freq=tofreq(code,sys,type))<0) continue;

		if ((j=checkpri(sys,type,freq))>=0) {
			if (!settag(i)) continue;
#if 0
			if (flags&0x20) { /* loss-of-lock potential */
				obuf.data[i].LLI[j]|=1;
			}
			if (!(flags&0x40)||!(flags&0x100)) { /* integral indicator */
				obuf.data[i].LLI[j]|=2;
			}
#endif
		}
	}
	return 0;
}
/* decode [TC] CA/L1 continuous tracking time ----------------------------------------------------- */
int javad::decode_TC(){
	unsigned short tt,tt_p;
	int i,sat;
	unsigned char *p=buff+5;

	if (obuf.n==0) return 0;

	if (!checksum()) {
		return -1;
	}
	if (len!=obuf.n*2+6) {
		return -1;
	}
	for (i=0; i<obuf.n&&i<MAXOBS; i++) {
		tt=U2(p); p+=2; if (tt==0xFFFF) continue;

		if (!settag(i)) continue;

		sat=obuf.data[i].sat;
		tt_p=(unsigned short)lockt[sat-1][0];

		/* loss-of-lock detected by lock-time counter */
		if (tt==0||tt<tt_p) {
			obuf.data[i].LLI[0]|=1;
		}
		lockt[sat-1][0]=tt;
	}
	return 0;
}

/* decode javad raw message ----------------------------------------------------------------------- */
int javad::decode_javad(){
	char *p=(char *)buff;
	char msg[40];

	if (outtype) {
		sprintf(msg, "JAVAD %2.2s (%4d)",p,len);
		msgtype=msg;
	}
	if (!strncmp(p,"~~",2)) return decode_RT(); /* receiver time */

	if (opt.find("-NOET")!=string::npos) {
		if (!strncmp(p,"::",2)) return decode_ET(); /* epoch time */
	}
	if (!strncmp(p,"RD",2)) return decode_RD(); /* receiver date */
	if (!strncmp(p,"SI",2)) return decode_SI(); /* satellite indices */
	if (!strncmp(p,"NN",2)) return decode_NN(); /* glonass slot numbers */

	if (!strncmp(p,"GA",2)) return decode_GA(); /* gps almanac */
	if (!strncmp(p,"NA",2)) return decode_NA(); /* glonass almanac */
	if (!strncmp(p,"EA",2)) return decode_EA(); /* galileo almanac */
	if (!strncmp(p,"WA",2)) return decode_WA(); /* sbas almanac */
	if (!strncmp(p,"QA",2)) return decode_QA(); /* qzss almanac (ext) */

	if (!strncmp(p,"GE",2)) return decode_GE(); /* gps ephemeris */
	if (!strncmp(p,"NE",2)) return decode_NE(); /* glonass ephemeris */
	if (!strncmp(p,"EN",2)) return decode_EN(); /* galileo ephemeris */
	if (!strncmp(p,"WE",2)) return decode_WE(); /* waas ephemeris */
	if (!strncmp(p,"QE",2)) return decode_QE(); /* qzss ephemeris (ext) */
	if (!strncmp(p,"CN",2)) return decode_CN(); /* beidou ephemeris (ext) */

	if (!strncmp(p,"UO",2)) return decode_UO(); /* gps utc time parameters */
	if (!strncmp(p,"NU",2)) return decode_NU(); /* glonass utc and gps time par */
	if (!strncmp(p,"EU",2)) return decode_EU(); /* galileo utc and gps time par */
	if (!strncmp(p,"WU",2)) return decode_WU(); /* waas utc time parameters */
	if (!strncmp(p,"QU",2)) return decode_QU(); /* qzss utc and gps time par */
	if (!strncmp(p,"IO",2)) return decode_IO(); /* ionospheric parameters */

	if (!strncmp(p,"GD",2)) return decode_nD(SYS_GPS); /* raw navigation data */
	if (!strncmp(p,"QD",2)) return decode_nD(SYS_QZS); /* raw navigation data */
	if (!strncmp(p,"gd",2)) return decode_nd(SYS_GPS); /* raw navigation data */
	if (!strncmp(p,"qd",2)) return decode_nd(SYS_QZS); /* raw navigation data */
	if (!strncmp(p,"ED",2)) return decode_nd(SYS_GAL); /* raw navigation data */
	if (!strncmp(p,"cd",2)) return decode_nd(SYS_CMP); /* raw navigation data */
	if (!strncmp(p,"LD",2)) return decode_LD(); /* glonass raw navigation data */
	if (!strncmp(p,"lD",2)) return decode_lD(); /* glonass raw navigation data */
	if (!strncmp(p,"WD",2)) return decode_WD(); /* sbas raw navigation data */

	if (!strncmp(p,"TC",2)) return decode_TC(); /* CA/L1 continuous track time */

	if (p[0]=='R') return decode_Rx(p[1]); /* pseudoranges */
	if (p[0]=='r') return decode_rx(p[1]); /* short pseudoranges */
	if (p[1]=='R') return decode_xR(p[0]); /* relative pseudoranges */
	if (p[1]=='r') return decode_xr(p[0]); /* short relative pseudoranges */
	if (p[0]=='P') return decode_Px(p[1]); /* carrier phases */
	if (p[0]=='p') return decode_px(p[1]); /* short carrier phases */
	if (p[1]=='P') return decode_xP(p[0]); /* relative carrier phases */
	if (p[1]=='p') return decode_xp(p[0]); /* relative carrier phases */
	if (p[0]=='D') return decode_Dx(p[1]); /* doppler */
	if (p[1]=='d') return decode_xd(p[0]); /* short relative doppler */
	if (p[0]=='E') return decode_Ex(p[1]); /* carrier to noise ratio */
	if (p[1]=='E') return decode_xE(p[0]); /* carrier to noise ratio x 4 */
	if (p[0]=='F') return decode_Fx(p[1]); /* signal lock loop flags */

	return 0;
}

/* input javad raw message from stream ---------------------------------------------------------------
* fetch next javad raw data and input a mesasge from stream
* args   : raw_t *raw   IO     receiver raw data control struct
*          unsigned char data I stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : to specify input options, set opt to the following option
*          strings separated by spaces.
*
*          -EPHALL : input all ephemerides
*          -GL1W   : select 1W for GPS L1 (default 1C)
*          -GL1X   : select 1X for GPS L1 (default 1C)
*          -GL2X   : select 2X for GPS L2 (default 2W)
*          -RL1C   : select 1C for GLO L1 (default 1P)
*          -RL2C   : select 2C for GLO L2 (default 2P)
*          -JL1Z   : select 1Z for QZS L1 (default 1C)
*          -JL1X   : select 1X for QZS L1 (default 1C)
*          -NOET   : discard epoch time message ET (::)
*
*-------------------------------------------------------------------------------------------------- */
int javad::decode(unsigned char data){
	int llen,stat;

	/* synchronize message */
	if (nbyte==0) {
		if (!sync_javad(data)) return 0;
		if (!(llen=decodelen(buff+2))||llen>MAXRAWLEN-5) {
			clearbuff();
			return -1;
		}
		len=llen+5;
		nbyte=5;
		return 0;
	}
	buff[nbyte++]=data;

	if (nbyte<len) return 0;

	/* decode javad raw message */
	stat=decode_javad();

	clearbuff();
	return stat;
}