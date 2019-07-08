#include "gw10.h"

#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
#define GW10SYNC    0x8B        /* gw10 sync code */

#define ID_GW10RAW  0x08        /* gw10 msg id: raw obs data */
#define ID_GW10GPS  0x02        /* gw10 msg id: gps message */
#define ID_GW10SBS  0x03        /* gw10 msg id: sbas message */
#define ID_GW10DGPS 0x06        /* gw10 msg id: dgps message */
#define ID_GW10REF  0x07        /* gw10 msg id: dgps ref info */
#define ID_GW10SOL  0x20        /* gw10 msg id: solution */
#define ID_GW10SATH 0x22        /* gw10 msg id: satellite health */
#define ID_GW10SATO 0x23        /* gw10 msg id: satellite orbit */
#define ID_GW10EPH  0x24        /* gw10 msg id: ephemeris */
#define ID_GW10ALM  0x25        /* gw10 msg id: almanac */
#define ID_GW10ION  0x26        /* gw10 msg id: ion/utc correction */
#define ID_GW10REPH 0x27        /* gw10 msg id: raw ephemeris */

#define LEN_GW10RAW 379         /* gw10 msg length: raw obs data */
#define LEN_GW10GPS 48          /* gw10 msg length: gps message */
#define LEN_GW10SBS 40          /* gw10 msg length: sbas message */
#define LEN_GW10DGPS 21         /* gw10 msg length: dgps message */
#define LEN_GW10REF 22          /* gw10 msg length: dgps ref info */
#define LEN_GW10SOL 227         /* gw10 msg length: solution */
#define LEN_GW10SATH 17         /* gw10 msg length: satellite health */
#define LEN_GW10SATO 67         /* gw10 msg length: satellite orbit */
#define LEN_GW10EPH 68          /* gw10 msg length: ephemeris */
#define LEN_GW10ALM 39          /* gw10 msg length: almanac */
#define LEN_GW10ION 32          /* gw10 msg length: ion/utc correction */
#define LEN_GW10REPH 98         /* gw10 msg length: raw ephemeris */

#define OFFWEEK     1024        /* week offset for ephemeris */

/* extract field (big-endian) ------------------------------------------------*/
#define U1(p)       (*((unsigned char *)(p)))
#define I1(p)       (*((char *)(p)))

static const char rcsid[]="$Id:$";

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
static double R8(unsigned char *p)
{
	double value;
	unsigned char *q=(unsigned char *)&value+7;
	int i;
	for (i=0; i<8; i++) *q--=*p++;
	return value;
}
/* message length ------------------------------------------------------------*/
static int msglen(unsigned char id)
{
	switch (id) {
	case ID_GW10RAW: return LEN_GW10RAW;
	case ID_GW10GPS: return LEN_GW10GPS;
	case ID_GW10SBS: return LEN_GW10SBS;
	case ID_GW10DGPS: return LEN_GW10DGPS;
	case ID_GW10REF: return LEN_GW10REF;
	case ID_GW10SOL: return LEN_GW10SOL;
	case ID_GW10SATH: return LEN_GW10SATH;
	case ID_GW10SATO: return LEN_GW10SATO;
	case ID_GW10EPH: return LEN_GW10EPH;
	case ID_GW10ALM: return LEN_GW10ALM;
	case ID_GW10ION: return LEN_GW10ION;
	case ID_GW10REPH: return LEN_GW10REPH;
	}
	return 0;
}

/* input gw10 raw message ----------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
gw10::gw10(){
}
gw10::~gw10(){
}
/* compute checksum ------------------------------------------------------- */
int gw10::chksum(){
	unsigned char cs=0;
	int i;
	for (i=1; i<len-1; i++) cs+=buff[i];
	return buff[len-1]==cs;
}
/* adjust weekly rollover of gps time ------------------------------------- */
int gw10::adjweek(double tow){
	double tow_p;
	int week;

	if (time.time==0) return 0;
	tow_p=time.time2gpst(&week);
	if (tow<tow_p-302400.0) tow+=604800.0;
	else if (tow>tow_p+302400.0) tow-=604800.0;
	time.gpst2time(week,tow);
	return 1;
}
/* bcd to number ---------------------------------------------------------- */
int gw10::bcd2num(unsigned char bcd){
	return (bcd>>4)*10+(bcd&0xF);
}
/* check partity ---------------------------------------------------------- */
int gw10::check_parity(unsigned int word,unsigned char *data){
	const unsigned int hamming[]={
		0xBB1F3480,0x5D8F9A40,0xAEC7CD00,0x5763E680,0x6BB1F340,0x8B7A89C0
	};
	unsigned int parity=0,w;
	int i;

	for (i=0; i<6; i++) {
		parity<<=1;
		for (w=(word&hamming[i])>>6; w; w>>=1) parity^=w&1;
	}
	if (parity!=(word&0x3F)) return 0;

	for (i=0; i<3; i++) data[i]=(unsigned char)(word>>(22-i*8));
	return 1;
}

/* decode raw obs data ---------------------------------------------------- */
int gw10::decode_gw10raw(){
	double tow,tows,toff,pr,cp;
	int i,j,n,prn,flg,sat,snr;
	unsigned char *p=buff+2;

	tow=R8(p);
	tows=floor(tow*1000.0+0.5)/1000.0; /* round by 10ms */
	toff=CLIGHT*(tows-tow);            /* time tag offset (m) */
	if (!adjweek(tows)) {
		return 0;
	}
	for (i=n=0,p+=8; i<16&&n<MAXOBS; i++,p+=23) {
		if (U1(p+1)!=1) continue;
		prn=U1(p);
		if (!(sat=satno(prn<=MAXPRNGPS ? SYS_GPS : SYS_SBS,prn))) {
			continue;
		}
		pr =R8(p+ 2)-toff;
		snr=U2(p+16);
		cp =-(int)(U4(p+18))/256.0-toff/WaveLengths[0];
		flg=U1(p+22);
		if (flg&0x3) {
			continue;
		}
		obs.data[n].time=time;
		obs.data[n].sat =sat;
		obs.data[n].P[0]=pr;
		obs.data[n].L[0]=(flg&0x80) ? 0.0 : ((flg&0x40) ? cp-0.5 : cp);
		obs.data[n].D[0]=0.0;
		obs.data[n].SNR[0]=(unsigned char)(snr*4.0+0.5);
		obs.data[n].LLI[0]=(flg&0x80) ? 1 : 0;
		obs.data[n].code[0]=CODE_L1C;

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
/* decode gps message ----------------------------------------------------- */
int gw10::decode_gw10gps(){
	decode_frame dec;
	double tow;
	unsigned int bbuff=0;
	int i,prn,sat,id;
	unsigned char *p=buff+2,frm[30];

	tow=U4(p)/1000.0; p+=4;
	prn=U1(p);        p+=1;
	if (!(sat=satno(SYS_GPS,prn))) {
		return -1;
	}
	for (i=0; i<10; i++) {
		bbuff=(bbuff<<30)|U4(p); p+=4;

		/* check parity of word */
		if (!check_parity(bbuff,frm+i*3)) {
			return -1;
		}
	}
	id=getbitu(frm,43,3); /* subframe id */

	if (id<1||5<id) {
		return -1;
	}
	for (i=0; i<30; i++) subfrm[sat-1][i+(id-1)*30]=frm[i];

	if (id==3) { /* decode ephemeris */
		if (dec.decode(subfrm[sat-1])!=1||
			dec.decode(subfrm[sat-1]+30)!=2||
			dec.decode(subfrm[sat-1]+60)!=3) {
			return 0;
		}
		if (opt.find("-EPHALL")==string::npos) {
			if (dec.eph.iode==nav.eph[sat-1].iode) return 0; /* unchanged */
		}
		dec.eph.sat=sat;
		nav.eph[sat-1]=dec.eph;
		ephsat=sat;
		return 2;
	}
	else if (id==4) { /* decode ion-utc parameters */
		if (dec.decode(frm)!=4) {
			return 0;
		}
		if (norm(dec.ion.begin(),8)>0.0&&norm(dec.utc.begin(),4)>0.0&&dec.leaps!=0) {
			for (i=0; i<8; i++) nav.ion_gps[i]=dec.ion[i];
			for (i=0; i<4; i++) nav.utc_gps[i]=dec.utc[i];
			nav.leaps=dec.leaps;
			return 9;
		}
	}
	return 0;
}
/* decode waas messages --------------------------------------------------- */
int gw10::decode_gw10sbs(){
	double tow;
	int i,prn;
	unsigned char *p=buff+2;

	tow=U4(p)/1000.0;
	prn=U1(p+4);
	if (prn<MINPRNSBS||MAXPRNSBS<prn) {
		return -1;
	}
	sbsmsg.prn=prn;
	sbsmsg.tow=(int)tow;
	tow=time.time2gpst(&sbsmsg.week);
	if (sbsmsg.tow<tow-302400.0) sbsmsg.week++;
	else if (sbsmsg.tow>tow+302400.0) sbsmsg.week--;

	for (i=0; i<29; i++) {
		sbsmsg.msg[i]=*(p+5+i);
	}
	sbsmsg.msg[28]&=0xC0;
	return 3;
}
/* decode raw ephemereris ------------------------------------------------- */
int gw10::decode_gw10reph(){
	decode_frame dec;
	double tow;
	int i,week,prn,sat;
	unsigned char *p=buff+2,bbuff[90];

	prn=U1(p);
	if (!(sat=satno(SYS_GPS,prn))) {
		return -1;
	}
	for (i=0; i<90; i++) {
		bbuff[i]=*(p+1+i);
	}
	if (dec.decode(bbuff)!=1||
		dec.decode(bbuff+ 30)!=2||
		dec.decode(bbuff+ 60)!=3) {
		return -1;
	}
	/* set time if no time avaliable */
	if (time.time==0) {
		tow=getbitu(bbuff,24,17)*6.0;
		week=getbitu(bbuff,48,10)+OFFWEEK;
		time.gpst2time(week,tow)->timeadd(24.0);
	}
	if (opt.find("-EPHALL")==string::npos) {
		if (dec.eph.iode==nav.eph[sat-1].iode) return 0; /* unchanged */
	}
	dec.eph.sat=sat;
	nav.eph[sat-1]=dec.eph;
	ephsat=sat;
	return 2;
}
/* decode solution -------------------------------------------------------- */
int gw10::decode_gw10sol(){
	gtime_t ttt;
	double ep[6]={ 0 },sec;
	unsigned char *p=buff+6;

	if (U2(p+42)&0xC00) { /* time valid? */
		return 0;
	}
	sec=U4(p+27)/16384.0;
	sec=floor(sec*1000.0+0.5)/1000.0;
	ep[2]=bcd2num(p[31]);
	ep[1]=bcd2num(p[32]);
	ep[0]=bcd2num(p[33])*100+bcd2num(p[34]);
	ttt.epoch2time(ep)->timeadd(sec)->utc2gpst();

	/* set time if no time available */
	if (time.time==0) {
		time=ttt;
	}
	return 0;
}

/* decode gw10 raw message ------------------------------------------------ */
int gw10::decode_gw10(){
	int type=U1(buff+1);
	char msg[40];

	if (outtype) {
		sprintf(msg, "GW10 0x%02X (%4d):",type,len);
		msgtype=msg;
	}
	switch (type) {
		case ID_GW10RAW: return decode_gw10raw();
		case ID_GW10GPS: return decode_gw10gps();
		case ID_GW10SBS: return decode_gw10sbs();
		case ID_GW10REPH: return decode_gw10reph();
		case ID_GW10SOL: return decode_gw10sol();
	}
	return 0;
}

/* input gw10 raw message ------------------------------------------------- */
int gw10::decode(unsigned char data){
	int stat;

	buff[nbyte++]=data;

	/* synchronize frame */
	if (buff[0]!=GW10SYNC) {
		nbyte=0;
		return 0;
	}
	if (nbyte>=2&&!(len=msglen(buff[1]))) {
		nbyte=0;
		return 0;
	}
	if (nbyte<2||nbyte<len) return 0;

	if (!chksum()) {
		buff[0]=0;
		nbyte=0;
		return -1;
	}
	/* decode gw10 raw message */
	stat=decode_gw10();

	buff[0]=0;
	nbyte=0;

	return stat;
}