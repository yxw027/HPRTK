#include "skytraq.h"

#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
#define STQSYNC1    0xA0        /* skytraq binary sync code 1 */
#define STQSYNC2    0xA1        /* skytraq binary sync code 2 */

#define ID_STQTIME  0xDC        /* skytraq message id: measurement epoch */
#define ID_STQRAW   0xDD        /* skytraq message id: raw measurement */
#define ID_STQGPS   0xE0        /* skytraq message id: gps/qzs subframe */
#define ID_STQGLO   0xE1        /* skytraq message id: glonass string */
#define ID_STQGLOE  0x5C        /* skytraq message id: glonass ephemeris */
#define ID_STQBDSD1 0xE2        /* skytraq message id: beidou d1 subframe */
#define ID_STQBDSD2 0xE3        /* skytraq message id: beidou d2 subframe */

#define ID_RESTART  0x01        /* skytraq message id: system restart */
#define ID_CFGSERI  0x05        /* skytraq message id: configure serial port */
#define ID_CFGFMT   0x09        /* skytraq message id: configure message format */
#define ID_CFGRATE  0x12        /* skytraq message id: configure message rate */
#define ID_CFGBIN   0x1E        /* skytraq message id: configure binary message */
#define ID_GETGLOEPH 0x5B       /* skytraq message id: get glonass ephemeris */

static const char rcsid[]="$Id:$";

/* extract field (big-endian) ------------------------------------------------*/
#define U1(p)       (*((unsigned char *)(p)))
#define I1(p)       (*((char *)(p)))

static unsigned short U2(unsigned char *p)
{
	unsigned short value;
	unsigned char *q=(unsigned char *)&value+1;
	int i;
	for (i=0; i<2; i++) *q--=*p++;
	return value;
}
static unsigned int U4(unsigned char *p)
{
	unsigned int value;
	unsigned char *q=(unsigned char *)&value+3;
	int i;
	for (i=0; i<4; i++) *q--=*p++;
	return value;
}
static float R4(unsigned char *p)
{
	float value;
	unsigned char *q=(unsigned char *)&value+3;
	int i;
	for (i=0; i<4; i++) *q--=*p++;
	return value;
}
static double R8(unsigned char *p)
{
	double value;
	unsigned char *q=(unsigned char *)&value+7;
	int i;
	for (i=0; i<8; i++) *q--=*p++;
	return value;
}

/* input skytraq raw message from stream -------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
skyq::skyq(){
}
skyq::~skyq(){
}
/* checksum --------------------------------------------------------------------------------------- */
unsigned char skyq::checksum(unsigned char *buff,int len){
	unsigned char cs=0;
	int i;

	for (i=4; i<len-3; i++) {
		cs^=buff[i];
	}
	return cs;
}
/* 8-bit week -> full week ------------------------------------------------------------------------ */
void skyq::adj_utcweek(double *utc){
	int week;

	if (utc[3]>=256.0) return;
	time.time2gpst(&week);
	utc[3]+=week/256*256;
	if (utc[3]<week-128) utc[3]+=256.0;
	else if (utc[3]>week+128) utc[3]-=256.0;
}
/* save subframe ---------------------------------------------------------- */
int skyq::save_subfrm(int sat){
	unsigned char *p=buff+7,*q;
	int i,id;

	/* check navigation subframe preamble */
	if (p[0]!=0x8B) {
		return 0;
	}
	id=(p[5]>>2)&0x7;

	/* check subframe id */
	if (id<1||5<id) {
		return 0;
	}
	q=subfrm[sat-1]+(id-1)*30;

	for (i=0; i<30; i++) q[i]=p[i];

	return id;
}
/* decode ephemeris ------------------------------------------------------- */
int skyq::decode_ephem(int sat){
	decode_frame dec;

	if (dec.decode(subfrm[sat-1])!=1||
		dec.decode(subfrm[sat-1]+30)!=2||
		dec.decode(subfrm[sat-1]+60)!=3) return 0;

	if (opt.find("-EPHALL")==string::npos) {
		if (dec.eph.iode==nav.eph[sat-1].iode&&
			dec.eph.iodc==nav.eph[sat-1].iodc) return 0; /* unchanged */
	}
	dec.eph.sat=sat;
	nav.eph[sat-1]=dec.eph;
	ephsat=sat;
	return 2;
}
/* decode almanac and ion/utc --------------------------------------------- */
int skyq::decode_alm1(int sat){
	decode_frame dec;
	int sys=satsys(sat,NULL);

	if (sys==SYS_GPS) {
		dec.decode(subfrm[sat-1]+90);
		nav.alm.assign(dec.alm.begin(),dec.alm.end());
		vecarr(dec.ion.begin(),nav.ion_gps,8);
		vecarr(dec.utc.begin(),nav.utc_gps,4);
		nav.leaps=dec.leaps;
		adj_utcweek(nav.utc_gps);
	}
	else if (sys==SYS_QZS) {
		dec.decode(subfrm[sat-1]+90);
		nav.alm.assign(dec.alm.begin(),dec.alm.end());
		vecarr(dec.ion.begin(),nav.ion_qzs,8);
		vecarr(dec.utc.begin(),nav.utc_qzs,4);
		nav.leaps=dec.leaps;
		adj_utcweek(nav.utc_qzs);
	}
	return 9;
}
/* decode almanac --------------------------------------------------------- */
int skyq::decode_alm2(int sat){
	decode_frame dec;
	int sys=satsys(sat,NULL);

	if (sys==SYS_GPS) {
		dec.decode(subfrm[sat-1]+120);
		nav.alm.assign(dec.alm.begin(),dec.alm.end());
	}
	else if (sys==SYS_QZS) {
		dec.decode(subfrm[sat-1]+120);
		nav.alm.assign(dec.alm.begin(),dec.alm.end());
		vecarr(dec.ion.begin(),nav.ion_qzs,8);
		vecarr(dec.utc.begin(),nav.utc_qzs,4);
		nav.leaps=dec.leaps;
		adj_utcweek(nav.utc_qzs);
	}
	return  0;
}

/* decode skytraq measurement epoch (0xDC) -------------------------------- */
int skyq::decode_stqtime(){
	unsigned char *p=buff+4;
	double tow;
	int week;

	iod=U1(p+1);
	week    =U2(p+2);
	tow     =U4(p+4)*0.001;
	time.gpst2time(week,tow);
	return 0;
}
/* decode skytraq raw measurement (0xDD) ---------------------------------- */
int skyq::decode_stqraw(){
	unsigned char *p=buff+4,ind;
	double pr1,cp1;
	int i,j,iiod,prn,sys,sat,n=0,nsat;

	iiod=U1(p+1);
	if (iiod!=iod) {
		return -1;
	}
	nsat=U1(p+2);
	if (len<8+23*nsat) {
		return -1;
	}
	for (i=0,p+=3; i<nsat&&i<MAXOBS; i++,p+=23) {
		prn=U1(p);

		if (MINPRNGPS<=prn&&prn<=MAXPRNGPS) {
			sys=SYS_GPS;
		}
		else if (MINPRNGLO<=prn-64&&prn-64<=MAXPRNGLO) {
			sys=SYS_GLO;
			prn-=64;
		}
		else if (MINPRNQZS<=prn&&prn<=MAXPRNQZS) {
			sys=SYS_QZS;
		}
		else if (MINPRNCMP<=prn-200&&prn-200<=MAXPRNCMP) {
			sys=SYS_CMP;
			prn-=200;
		}
		else {
			continue;
		}
		if (!(sat=satno(sys,prn))) {
			continue;
		}
		ind=U1(p+22);
		pr1=!(ind&1) ? 0.0 : R8(p+ 2);
		cp1=!(ind&4) ? 0.0 : R8(p+10);
		cp1-=floor((cp1+1E9)/2E9)*2E9; /* -10^9 < cp1 < 10^9 */

		obs.data[n].P[0]=pr1;
		obs.data[n].L[0]=cp1;
		obs.data[n].D[0]=!(ind&2) ? 0.0 : R4(p+18);
		obs.data[n].SNR[0]=U1(p+1)*4;
		obs.data[n].LLI[0]=0;
		obs.data[n].code[0]=sys==SYS_CMP ? CODE_L1I : CODE_L1C;

		lockt[sat-1][0]=ind&8 ? 1 : 0; /* cycle slip */

		if (obs.data[n].L[0]!=0.0) {
			obs.data[n].LLI[0]=(unsigned char)lockt[sat-1][0];
			lockt[sat-1][0]=0;
		}
		/* receiver dependent options */
		if (opt.find("-INVCP")!=string::npos) {
			obs.data[n].L[0]*=-1.0;
		}
		obs.data[n].time=time;
		obs.data[n].sat =sat;

		for (j=1; j<NFREQ+NEXOBS; j++) {
			obs.data[n].L[j]=obs.data[n].P[j]=0.0;
			obs.data[n].D[j]=0.0;
			obs.data[n].SNR[j]=obs.data[n].LLI[j]=0;
			obs.data[n].code[j]=CODE_NONE;
		}
		n++;
	}
	obs.n=n;
	return n>0 ? 1 : 0;
}
/* decode gps/qzss subframe (0xE0) ---------------------------------------- */
int skyq::decode_stqgps(){
	int prn,sat,id;
	unsigned char *p=buff+4;

	if (len<40) {
		return -1;
	}
	prn=U1(p+1);
	if (!(sat=satno(MINPRNQZS<=prn&&prn<=MAXPRNQZS ? SYS_QZS : SYS_GPS,prn))) {
		return -1;
	}
	id=save_subfrm(sat);
	if (id==3) return decode_ephem(sat);
	if (id==4) return decode_alm1(sat);
	if (id==5) return decode_alm2(sat);
	return 0;
}
/* decode glonass string (0xE1) ------------------------------------------- */
int skyq::decode_stqglo(){
	geph_t geph;
	int i,prn,sat,m;
	unsigned char *p=buff+4;

	if (len<19) {
		return -1;
	}
	prn=U1(p+1)-64;
	if (!(sat=satno(SYS_GLO,prn))) {
		return -1;
	}
	m=U1(p+2); /* string number */
	if (m<1||4<m) {
		return -1;
	}
	setbitu(subfrm[sat-1]+(m-1)*10,1,4,m);
	for (i=0; i<9; i++) {
		setbitu(subfrm[sat-1]+(m-1)*10,5+i*8,8,p[3+i]);
	}
	if (m!=4) return 0;

	/* decode glonass ephemeris strings */
	geph.tof=time;
	if (!decode_glostr(sat,geph)||geph.sat!=sat) return 0;

	/* freq channel number by stqgloe (0x5C) message */
	geph.frq=nav.geph[prn-1].frq;

	if (opt.find("-EPHALL")==string::npos) {
		if (geph.iode==nav.geph[prn-1].iode) return 0; /* unchanged */
	}
	nav.geph[prn-1]=geph;
	ephsat=sat;
	return 2;
}
/* decode glonass string (requested) (0x5C) ------------------------------- */
int skyq::decode_stqgloe(){
	int prn,sat;
	unsigned char *p=buff+4;

	if (len<50) {
		return -1;
	}
	prn=U1(p+1);
	if (!(sat=satno(SYS_GLO,prn))) {
		return -1;
	}
	/* only set frequency channel number */
	nav.geph[prn-1].frq=I1(p+2);

	return 0;
}
/* decode beidou subframe (0xE2,0xE3) ------------------------------------- */
int skyq::decode_stqbds(){
	eph_t eph;
	unsigned int word;
	int i,j=0,id,pgn,prn,sat;
	unsigned char *p=buff+4;

	if (len<38) {
		return -1;
	}
	prn=U1(p+1)-200;
	if (!(sat=satno(SYS_CMP,prn))) {
		return -1;
	}
	id=U1(p+2); /* subframe id */
	if (id<1||5<id) {
		return -1;
	}
	if (prn>=5) { /* IGSO/MEO */
		word=getbitu(p+3,j,26)<<4; j+=26;
		setbitu(subfrm[sat-1]+(id-1)*38,0,30,word);

		for (i=1; i<10; i++) {
			word=getbitu(p+3,j,22)<<8; j+=22;
			setbitu(subfrm[sat-1]+(id-1)*38,i*30,30,word);
		}
		if (id!=3) return 0;

		/* decode beidou D1 ephemeris */
		if (!decode_bds_d1(sat,eph)) return 0;
	}
	else { /* GEO */
		if (id!=1) return 0;

		pgn=getbitu(p+3,26+12,4); /* page number */
		if (pgn<1||10<pgn) {
			return -1;
		}
		word=getbitu(p+3,j,26)<<4; j+=26;
		setbitu(subfrm[sat-1]+(pgn-1)*38,0,30,word);

		for (i=1; i<10; i++) {
			word=getbitu(p+3,j,22)<<8; j+=22;
			setbitu(subfrm[sat-1]+(pgn-1)*38,i*30,30,word);
		}
		if (pgn!=10) return 0;

		/* decode beidou D2 ephemeris */
		if (!decode_bds_d2(sat,eph)) return 0;
	}
	if (opt.find("-EPHALL")==string::npos) {
		if (eph.toe.timediff(nav.eph[sat-1].toe)==0.0) return 0; /* unchanged */
	}
	eph.sat=sat;
	nav.eph[sat-1]=eph;
	ephsat=sat;
	return 2;
}

/* decode skytraq message ------------------------------------------------- */
int skyq::decode_stq(){
	int type=U1(buff+4);
	unsigned char cs,*p=buff+len-3;
	char msg[40];

	/* checksum */
	cs=checksum(buff,len);

	if (cs!=*p||*(p+1)!=0x0D||*(p+2)!=0x0A) {
		return -1;
	}
	if (outtype) {
		sprintf(msg, "SKYTRAQ 0x%02x (%4d):",type,len);
		msgtype=msg;
	}
	switch (type) {
		case ID_STQTIME: return decode_stqtime();
		case ID_STQRAW: return decode_stqraw();
		case ID_STQGPS: return decode_stqgps();
		case ID_STQGLO: return decode_stqglo();
		case ID_STQGLOE: return decode_stqgloe();
		case ID_STQBDSD1: return decode_stqbds();
		case ID_STQBDSD2: return decode_stqbds();
	}
	return 0;
}
/* sync code -------------------------------------------------------------- */
int skyq::sync_stq(unsigned char data){
	buff[0]=buff[1]; buff[1]=data;
	return buff[0]==STQSYNC1&&buff[1]==STQSYNC2;
}

/* input superstar 2 raw message from stream ------------------------------------------------------ */
int skyq::decode(unsigned char data){
	/* synchronize frame */
	if (nbyte==0) {
		if (!sync_stq(data)) return 0;
		nbyte=2;
		return 0;
	}
	buff[nbyte++]=data;

	if (nbyte==4) {
		if ((len=U2(buff+2)+7)>MAXRAWLEN) {
			nbyte=0;
			return -1;
		}
	}
	if (nbyte<4||nbyte<len) return 0;
	nbyte=0;

	/* decode skytraq raw message */
	return decode_stq();
}

