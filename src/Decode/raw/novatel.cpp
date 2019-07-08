#include "novatel.h"

#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
#define OEM4SYNC1   0xAA        /* oem4 message start sync code 1 */
#define OEM4SYNC2   0x44        /* oem4 message start sync code 2 */
#define OEM4SYNC3   0x12        /* oem4 message start sync code 3 */
#define OEM3SYNC1   0xAA        /* oem3 message start sync code 1 */
#define OEM3SYNC2   0x44        /* oem3 message start sync code 2 */
#define OEM3SYNC3   0x11        /* oem3 message start sync code 3 */

#define OEM4HLEN    28          /* oem4 message header length (bytes) */
#define OEM3HLEN    12          /* oem3 message header length (bytes) */

#define ID_ALMANAC  73          /* message id: oem4 decoded almanac */
#define ID_GLOALMANAC 718       /* message id: oem4 glonass decoded almanac */
#define ID_GLOEPHEMERIS 723     /* message id: oem4 glonass ephemeris */
#define ID_IONUTC   8           /* message id: oem4 iono and utc data */
#define ID_RANGE    43          /* message id: oem4 range measurement */
#define ID_RANGECMP 140         /* message id: oem4 range compressed */
#define ID_RAWALM   74          /* message id: oem4 raw almanac */
#define ID_RAWEPHEM 41          /* message id: oem4 raw ephemeris */
#define ID_RAWWAASFRAME 287     /* message id: oem4 raw waas frame */

#define ID_QZSSIONUTC 1347      /* message id: oem6 qzss ion/utc parameters */
#define ID_QZSSRAWEPHEM 1330    /* message id: oem6 qzss raw ephemeris */
#define ID_QZSSRAWSUBFRAME 1331 /* message id: oem6 qzss raw subframe */
#define ID_RAWSBASFRAME 973     /* message id: oem6 raw sbas frame */
#define ID_GALEPHEMERIS 1122    /* message id: oem6 decoded galileo ephemeris */
#define ID_GALALMANAC 1120      /* message id: oem6 decoded galileo almanac */
#define ID_GALCLOCK 1121        /* message id: oem6 galileo clockinformation */
#define ID_GALIONO  1127        /* message id: oem6 decoded galileo iono corrections */
#define ID_GALFNAVRAWPAGE 1413  /* message id: oem6 raw galileo f/nav paga data */
#define ID_GALINAVRAWWORD 1414  /* message id: oem6 raw galileo i/nav word data */
#define ID_RAWCNAVFRAME 1066    /* message id: oem6 raw cnav frame data */
#define ID_BDSEPHEMERIS 1696    /* message id: oem6 decoded bds ephemeris */

#define ID_ALMB     18          /* message id: oem3 decoded almanac */
#define ID_IONB     16          /* message id: oem3 iono parameters */
#define ID_UTCB     17          /* message id: oem3 utc parameters */
#define ID_FRMB     54          /* message id: oem3 framed raw navigation data */
#define ID_RALB     15          /* message id: oem3 raw almanac */
#define ID_RASB     66          /* message id: oem3 raw almanac set */
#define ID_REPB     14          /* message id: oem3 raw ephemeris */
#define ID_RGEB     32          /* message id: oem3 range measurement */
#define ID_RGED     65          /* message id: oem3 range compressed */

#define WL1         0.1902936727984
#define WL2         0.2442102134246
#define MAXVAL      8388608.0

#define OFF_FRQNO   -7          /* F/W ver.3.620 */

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))
static unsigned short U2(unsigned char *p) { unsigned short u; memcpy(&u,p,2); return u; }
static unsigned int   U4(unsigned char *p) { unsigned int   u; memcpy(&u,p,4); return u; }
static int            I4(unsigned char *p) { int            i; memcpy(&i,p,4); return i; }
static float          R4(unsigned char *p) { float          r; memcpy(&r,p,4); return r; }
static double         R8(unsigned char *p) { double         r; memcpy(&r,p,8); return r; }

/* INPUT NovAtel data */
/* input omex raw data from stream ------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
/* input oem4 raw data from stream -------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
oem_t::oem_t(){
}
oem_t::~oem_t(){
}
/* Novatel functions ------------------------------------------------------------------------------ */
/* Base functions */
/* extend sign ------------------------------------------------------------------------------------ */
int oem_t::exsign(unsigned int v,int bits){
	return (int)(v&(1<<(bits-1)) ? v|(~0u<<bits) : v);
}
/* checksum --------------------------------------------------------------------------------------- */
unsigned char oem_t::chksum()
{
	unsigned char sum=0;
	int i;
	for (i=0; i<len; i++) sum^=buff[i];
	return sum;
}
/* adjust weekly rollover of gps time ------------------------------------------------------------- */
gtime_t oem_t::adjweek(gtime_t &time,double tow)
{
	double tow_p;
	int week;
	tow_p=time.time2gpst(&week);
	if (tow<tow_p-302400.0) tow+=604800.0;
	else if (tow>tow_p+302400.0) tow-=604800.0;
	time.gpst2time(week,tow);
	return time;
}
/* get observation data index --------------------------------------------------------------------- */
int oem_t::obsindex(int sat){
	int i,j;

	if (obs.n>=MAXOBS) return -1;
	for (i=0; i<obs.n; i++) {
		if (obs.data[i].sat==sat) return i;
	}
	obs.data[i].time=time;
	obs.data[i].sat=sat;
	for (j=0; j<NFREQ+NEXOBS; j++) {
		obs.data[i].L[j]=obs.data[i].P[j]=0.0;
		obs.data[i].D[j]=0.0;
		obs.data[i].SNR[j]=obs.data[i].LLI[j]=0;
		obs.data[i].code[j]=CODE_NONE;
	}
	obs.n++;
	return i;
}
/* ura value (m) to ura index --------------------------------------------------------------------- */
int oem_t::uraindex(double value)
{
	static const double ura_eph[]={
		2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
		3072.0,6144.0,0.0
	};
	int i;
	for (i=0; i<15; i++) if (ura_eph[i]>=value) break;
	return i;
}
/* DECODE oem3 functions */
/* DECODE oem4 functions */
/* decode rgeb ------------------------------------------------------------------------------------ */
int oem_t::decode_rgeb(){
	unsigned char *p=buff+OEM3HLEN;
	double tow,psr,adr,tt,locktt,dop,snr;
	int i,week,nobs,prn,sat,stat,sys,parity,lli,index,freq;

	week=adjgpsweek(U4(p));
	tow =R8(p+ 4);
	nobs=U4(p+12);
	time.gpst2time(week,tow);

	if (len!=OEM3HLEN+20+nobs*44) {
		return -1;
	}
	for (i=0,p+=20; i<nobs; i++,p+=44) {
		prn   =U4(p);
		psr   =R8(p+ 4);
		adr   =R8(p+16);
		dop   =R4(p+28);
		snr   =R4(p+32);
		locktt=R4(p+36);     /* lock time (s) */
		stat  =I4(p+40);     /* tracking status */
		freq  =(stat>>20)&1; /* L1:0,L2:1 */
		sys   =(stat>>15)&7; /* satellite sys (0:GPS,1:GLONASS,2:WAAS) */
		parity=(stat>>10)&1; /* parity known */
		if (!(sat=satno(sys==1 ? SYS_GLO : (sys==2 ? SYS_SBS : SYS_GPS),prn))) {
			continue;
		}
		tt=time.timediff(tobs);
		if (tobs.time!=0) {
			lli=locktt-lockt[sat-1][freq]+0.05<tt||
				parity!=halfc[sat-1][freq];
		}
		else {
			lli=0;
		}
		if (!parity) lli|=2;
		lockt[sat-1][freq]=locktt;
		halfc[sat-1][freq]=parity;

		if (fabs(obs.data[0].time.timediff(time))>1E-9) {
			obs.n=0;
		}
		if ((index=obsindex(sat))>=0) {
			obs.data[index].L[freq]=-adr; /* flip sign */
			obs.data[index].P[freq]=psr;
			obs.data[index].D[freq]=(float)dop;
			obs.data[index].SNR[freq]=
				0.0<=snr&&snr<255.0 ? (unsigned char)(snr*4.0+0.5) : 0;
			obs.data[index].LLI[freq]=(unsigned char)lli;
			obs.data[index].code[freq]=freq==0 ? CODE_L1C : CODE_L2P;
		}
	}
	tobs=time;
	return 1;
}
/* decode rged ---------------------------------------------------------------------------------- */
int oem_t::decode_rged(){
	unsigned int word;
	unsigned char *p=buff+OEM3HLEN;
	double tow,psrh,psrl,psr,adr,adr_rolls,tt,locktt,dop;
	int i,week,nobs,prn,sat,stat,sys,parity,lli,index,freq,snr;

	nobs=U2(p);
	week=adjgpsweek(U2(p+2));
	tow =U4(p+4)/100.0;
	time.gpst2time(week,tow);
	if (len!=OEM3HLEN+12+nobs*20) {
		return -1;
	}
	for (i=0,p+=12; i<nobs; i++,p+=20) {
		word  =U4(p);
		prn   =word&0x3F;
		snr   =((word>>6)&0x1F)+20;
		locktt=(word>>11)/32.0;
		adr   =-I4(p+4)/256.0;
		word  =U4(p+8);
		psrh  =word&0xF;
		dop   =exsign(word>>4,28)/256.0;
		psrl  =U4(p+12);
		stat  =U4(p+16)>>8;
		freq  =(stat>>20)&1; /* L1:0,L2:1 */
		sys   =(stat>>15)&7; /* satellite sys (0:GPS,1:GLONASS,2:WAAS) */
		parity=(stat>>10)&1; /* parity known */
		if (!(sat=satno(sys==1 ? SYS_GLO : (sys==2 ? SYS_SBS : SYS_GPS),prn))) {
			continue;
		}
		tt=time.timediff(tobs);
		psr=(psrh*4294967296.0+psrl)/128.0;
		adr_rolls=floor((psr/(freq==0 ? WL1 : WL2)-adr)/MAXVAL+0.5);
		adr=adr+MAXVAL*adr_rolls;

		if (tobs.time!=0) {
			lli=locktt-lockt[sat-1][freq]+0.05<tt||
				parity!=halfc[sat-1][freq];
		}
		else {
			lli=0;
		}
		if (!parity) lli|=2;
		lockt[sat-1][freq]=locktt;
		halfc[sat-1][freq]=parity;

		if (fabs(obs.data[0].time.timediff(time))>1E-9) {
			obs.n=0;
		}
		if ((index=obsindex(sat))>=0) {
			obs.data[index].L[freq]=adr;
			obs.data[index].P[freq]=psr;
			obs.data[index].D[freq]=(float)dop;
			obs.data[index].SNR[freq]=(unsigned char)(snr*4.0+0.5);
			obs.data[index].LLI[freq]=(unsigned char)lli;
			obs.data[index].code[freq]=freq==0 ? CODE_L1C : CODE_L2P;
		}
	}
	tobs=time;
	return 1;
}
/* decode repb ---------------------------------------------------------------------------------- */
int oem_t::decode_repb(){
	unsigned char *p=buff+OEM3HLEN;
	decode_frame dec=decode_frame();
	int prn,sat;

	if (len!=OEM3HLEN+96) {
		return -1;
	}
	prn=U4(p);
	if (!(sat=satno(SYS_GPS,prn))) {
		return -1;
	}
	if (dec.decode(p+ 4)!=1||
		dec.decode(p+34)!=2||
		dec.decode(p+64)!=3) {
		return -1;
	}
	if (opt.find("-EPHALL")==string::npos) {
		if (dec.eph.iode==nav.eph[sat-1].iode) return 0; /* unchanged */
	}
	dec.eph.sat=sat;
	nav.eph[sat-1]=dec.eph;
	ephsat=sat;
	return 2;
}
/* decode frmb --------------------------------------------------------------------------------- */
int oem_t::decode_frmb(){
	unsigned char *p=buff+OEM3HLEN;
	double tow;
	int i,week,prn,nbit;

	week=adjgpsweek(U4(p));
	tow =R8(p+ 4);
	prn =U4(p+12);
	nbit=U4(p+20);
	time.gpst2time(week,tow);
	if (nbit!=250) return 0;
	if (prn<MINPRNSBS||MAXPRNSBS<prn) {
		return -1;
	}
	sbsmsg.week=week;
	sbsmsg.tow=(int)tow;
	sbsmsg.prn=prn;
	for (i=0; i<29; i++) sbsmsg.msg[i]=p[24+i];
	return 3;
}
/* decode ionb ---------------------------------------------------------------------------------- */
int oem_t::decode_ionb(){
	unsigned char *p=buff+OEM3HLEN;
	int i;

	if (len!=64+OEM3HLEN) {
		return -1;
	}
	for (i=0; i<8; i++) nav.ion_gps[i]=R8(p+i*8);
	return 9;
}
/* decode utcb ---------------------------------------------------------------------------------- */
int oem_t::decode_utcb(){
	unsigned char *p=buff+OEM3HLEN;

	if (len!=40+OEM3HLEN) {
		return -1;
	}
	nav.utc_gps[0]=R8(p);
	nav.utc_gps[1]=R8(p+ 8);
	nav.utc_gps[2]=U4(p+16);
	nav.utc_gps[3]=adjgpsweek(U4(p+20));
	nav.leaps =I4(p+28);
	return 9;
}
int oem_t::decode(unsigned char data){
	return 0;
}

/* input oem4 raw data from stream -------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
oem4::oem4(){
}
oem4::~oem4(){
}
/* DECODE oem4 functions */
/* decode oem4 tracking status --------------------------------------------------------------------
* deocode oem4 tracking status
* args   : unsigned int stat I  tracking status field
*          int    *sys   O      system (SYS_???)
*          int    *code  O      signal code (CODE_L??)
*          int    *track O      tracking state
*                         (oem4/5)
*                         0=L1 idle                   8=L2 idle
*                         1=L1 sky search             9=L2 p-code align
*                         2=L1 wide freq pull-in     10=L2 search
*                         3=L1 narrow freq pull-in   11=L2 pll
*                         4=L1 pll                   12=L2 steering
*                         5=L1 reacq
*                         6=L1 steering
*                         7=L1 fll
*                         (oem6)
*                         0=idle                      7=freq-lock loop
*                         2=wide freq band pull-in    9=channel alignment
*                         3=narrow freq band pull-in 10=code search
*                         4=phase lock loop          11=aided phase lock loop
*          int    *plock O      phase-lock flag   (0=not locked, 1=locked)
*          int    *clock O      code-lock flag    (0=not locked, 1=locked)
*          int    *parity O     parity known flag (0=not known,  1=known)
*          int    *halfc O      phase measurement (0=half-cycle not added,
*                                                  1=added)
* return : signal frequency (0:L1,1:L2,2:L5,3:L6,4:L7,5:L8,-1:error)
* notes  : refer [1][3]
*-------------------------------------------------------------------------------------------------- */
int oem4::decode_trackstat(unsigned int stat,int &sys,int &code,int &track,
	int &plock,int &clock,int &parity,int &halfc)
{
	int satsys,sigtype,freq=0;

	track =stat&0x1F;
	plock =(stat>>10)&1;
	parity=(stat>>11)&1;
	clock =(stat>>12)&1;
	satsys =(stat>>16)&7;
	halfc =(stat>>28)&1;
	sigtype=(stat>>21)&0x1F;

	switch (satsys) {
	case 0: sys=SYS_GPS; break;
	case 1: sys=SYS_GLO; break;
	case 2: sys=SYS_SBS; break;
	case 3: sys=SYS_GAL; break; /* OEM6 */
	case 4: sys=SYS_CMP; break; /* OEM6 F/W 6.400 */
	case 5: sys=SYS_QZS; break; /* OEM6 */
	default:
		return -1;
	}
	if (sys==SYS_GPS||sys==SYS_QZS) {
		switch (sigtype) {
		case  0: freq=0; code=CODE_L1C; break; /* L1C/A */
		case  5: freq=0; code=CODE_L1P; break; /* L1P */
		case  9: freq=1; code=CODE_L2D; break; /* L2Pcodeless */
		case 14: freq=2; code=CODE_L5Q; break; /* L5Q (OEM6) */
		case 17: freq=1; code=CODE_L2X; break; /* L2C(M+L) */
		default: freq=-1; break;
		}
	}
	else if (sys==SYS_GLO) {
		switch (sigtype) {
		case  0: freq=0; code=CODE_L1C; break; /* L1C/A */
		case  1: freq=1; code=CODE_L2C; break; /* L2C/A (OEM6) */
		case  5: freq=1; code=CODE_L2P; break; /* L2P */
		default: freq=-1; break;
		}
	}
	else if (sys==SYS_GAL) {
		switch (sigtype) {
		case  1: freq=0; code=CODE_L1B; break; /* E1B  (OEM6) */
		case  2: freq=0; code=CODE_L1C; break; /* E1C  (OEM6) */
		case 12: freq=2; code=CODE_L5Q; break; /* E5aQ (OEM6) */
		case 17: freq=4; code=CODE_L7Q; break; /* E5bQ (OEM6) */
		case 20: freq=5; code=CODE_L8Q; break; /* AltBOCQ (OEM6) */
		default: freq=-1; break;
		}
	}
	else if (sys==SYS_CMP) {
		switch (sigtype) {
		case  0: freq=0; code=CODE_L1I; break; /* B1 with D1 (OEM6) */
		case  1: freq=1; code=CODE_L7I; break; /* B2 with D1 (OEM6) */
		case  4: freq=0; code=CODE_L1I; break; /* B1 with D2 (OEM6) */
		case  5: freq=1; code=CODE_L7I; break; /* B2 with D2 (OEM6) */
		default: freq=-1; break;
		}
	}
	else if (sys==SYS_SBS) {
		switch (sigtype) {
		case  0: freq=0; code=CODE_L1C; break; /* L1C/A */
		case  6: freq=2; code=CODE_L5I; break; /* L5I (OEM6) */
		default: freq=-1; break;
		}
	}
	if (freq<0) return -1;

	return freq;
}
/* check code priority and return obs position -------------------------------------------------- */
int oem4::checkpri(string opt,int sys,int code,int freq){
	int nex=NEXOBS; /* number of extended obs data */

	if (sys==SYS_GPS) {
		if (opt.find("-GL1P")!=string::npos&&freq==0) return code==CODE_L1P ? 0 : -1;
		if (opt.find("-GL1X")!=string::npos&&freq==1) return code==CODE_L2X ? 1 : -1;
		if (code==CODE_L1P) return nex<1 ? -1 : NFREQ;
		if (code==CODE_L2X) return nex<2 ? -1 : NFREQ+1;
	}
	else if (sys==SYS_GLO) {
		if (opt.find("-RL2C")!=string::npos&&freq==1) return code==CODE_L2C ? 1 : -1;
		if (code==CODE_L2C) return nex<1 ? -1 : NFREQ;
	}
	else if (sys==SYS_GAL) {
		if (opt.find("-EL1B")!=string::npos&&freq==0) return code==CODE_L1B ? 0 : -1;
		if (code==CODE_L1B) return nex<1 ? -1 : NFREQ;
		if (code==CODE_L7Q) return nex<2 ? -1 : NFREQ+1;
		if (code==CODE_L8Q) return nex<3 ? -1 : NFREQ+2;
	}
	return freq<NFREQ ? freq : -1;
}
/* decode rangecmpb ------------------------------------------------------------------------------- */
int oem4::decode_rangecmpb(){
	double psr,adr,adr_rolls,locktt,tt,dop,snr,wavelen;
	int i,index,nobs,prn,sat,sys,code,freq,pos;
	int track,plock,clock,parity,halfcc,lli;
	string str;
	unsigned char *p=buff+OEM4HLEN;

	nobs=U4(p);

	if (outtype) {

		msgtype+=" nobs="+int2str(2," ",nobs,str);
	}
	if (len<OEM4HLEN+4+nobs*24) {
		return -1;
	}
	for (i=0,p+=4; i<nobs; i++,p+=24) {

		/* decode tracking status */
		if ((freq=decode_trackstat(U4(p),sys,code,track,plock,clock,
			parity,halfcc))<0) continue;

		/* obs position */
		if ((pos=checkpri(opt,sys,code,freq))<0) continue;

		prn=U1(p+17);
		if (sys==SYS_GLO) prn-=37;

		if (!(sat=satno(sys,prn))) {
			continue;
		}
		if (sys==SYS_GLO&&!parity) continue; /* invalid if GLO parity unknown */

		dop=exsign(U4(p+4)&0xFFFFFFF,28)/256.0;
		psr=(U4(p+7)>>4)/128.0+U1(p+11)*2097152.0;

		if ((wavelen=satwavelen(sat,freq,&nav))<=0.0) {
			if (sys==SYS_GLO) wavelen=CLIGHT/(freq==0 ? FREQ1_GLO : FREQ2_GLO);
			else wavelen=WaveLengths[freq];
		}
		adr=I4(p+12)/256.0;
		adr_rolls=(psr/wavelen+adr)/MAXVAL;
		adr=-adr+MAXVAL*floor(adr_rolls+(adr_rolls<=0 ? -0.5 : 0.5));

		locktt=(U4(p+18)&0x1FFFFF)/32.0; /* lock time */

		tt=time.timediff(tobs);
		if (tobs.time!=0) {
			lli=(locktt<65535.968&&locktt-lockt[sat-1][pos]+0.05<=tt)||
				halfcc!=halfc[sat-1][pos];
		}
		else {
			lli=0;
		}
		if (!parity) lli|=2;
		lockt[sat-1][pos]=locktt;
		halfc[sat-1][pos]=halfcc;

		snr=((U2(p+20)&0x3FF)>>5)+20.0;
		if (!clock) psr=0.0;     /* code unlock */
		if (!plock) adr=dop=0.0; /* phase unlock */

		if (fabs(obs.data[0].time.timediff(time))>1E-9) {
			obs.n=0;
		}
		if ((index=obsindex(sat))>=0) {
			obs.data[index].L[pos]=adr;
			obs.data[index].P[pos]=psr;
			obs.data[index].D[pos]=(float)dop;
			obs.data[index].SNR[pos]=
				0.0<=snr&&snr<255.0 ? (unsigned char)(snr*4.0+0.5) : 0;
			obs.data[index].LLI[pos]=(unsigned char)lli;
			obs.data[index].code[pos]=code;
#if 0
			/* L2C phase shift correction (L2C->L2P) */
			if (code==CODE_L2X) {
				obs.data[index].L[pos]+=0.25;
			}
#endif
		}
	}
	tobs=time;
	return 1;
}
/* decode rangeb ---------------------------------------------------------------------------------- */
int oem4::decode_rangeb(){
	double psr,adr,dop,snr,locktt,tt;
	string str;
	int i,index,nobs,prn,sat,sys,code,freq,pos;
	int track,plock,clock,parity,halfcc,lli,gfrq;
	unsigned char *p=buff+OEM4HLEN;

	nobs=U4(p);

	if (outtype) {
		msgtype+=" nobs="+int2str(2," ",nobs,str);
	}
	if (len<OEM4HLEN+4+nobs*44) {
		return -1;
	}
	for (i=0,p+=4; i<nobs; i++,p+=44) {

		/* decode tracking status */
		if ((freq=decode_trackstat(U4(p+40),sys,code,track,plock,clock,
			parity,halfcc))<0) continue;

		/* obs position */
		if ((pos=checkpri(opt,sys,code,freq))<0) continue;

		prn=U2(p);
		if (sys==SYS_GLO) prn-=37;

		if (!(sat=satno(sys,prn))) {
			continue;
		}
		if (sys==SYS_GLO&&!parity) continue; /* invalid if GLO parity unknown */

		gfrq =U2(p+ 2);
		psr  =R8(p+ 4);
		adr  =R8(p+16);
		dop  =R4(p+28);
		snr  =R4(p+32);
		locktt=R4(p+36);

		/* set glonass frequency channel number */
		if (sys==SYS_GLO&&nav.geph[prn-1].sat!=sat) {
			nav.geph[prn-1].frq=gfrq-7;
		}
		tt=time.timediff(tobs);
		if (tobs.time!=0) {
			lli=locktt-lockt[sat-1][pos]+0.05<=tt||
				halfcc!=halfc[sat-1][pos];
		}
		else {
			lli=0;
		}
		if (!parity) lli|=2;
		lockt[sat-1][pos]=locktt;
		halfc[sat-1][pos]=halfcc;
		if (!clock) psr=0.0;     /* code unlock */
		if (!plock) adr=dop=0.0; /* phase unlock */

		if (fabs(obs.data[0].time.timediff(time))>1E-9) {
			obs.n=0;
		}
		if ((index=obsindex(sat))>=0) {
			obs.data[index].L[pos]=-adr;
			obs.data[index].P[pos]=psr;
			obs.data[index].D[pos]=(float)dop;
			obs.data[index].SNR[pos]=
				0.0<=snr&&snr<255.0 ? (unsigned char)(snr*4.0+0.5) : 0;
			obs.data[index].LLI[pos]=(unsigned char)lli;
			obs.data[index].code[pos]=code;
#if 0
			/* L2C phase shift correction */
			if (code==CODE_L2X) {
				obs.data[index].L[pos]+=0.25;
			}
#endif
		}
	}
	tobs=time;
	return 1;
}
/* decode rawephemb ------------------------------------------------------------------------------- */
int oem4::decode_rawephemb(){
	unsigned char *p=buff+OEM4HLEN;
	decode_frame dec=decode_frame();
	int prn,sat;

	if (len<OEM4HLEN+102) {
		return -1;
	}
	prn=U4(p);
	if (!(sat=satno(SYS_GPS,prn))) {
		return -1;
	}
	if (dec.decode(p+ 12)!=1||
		dec.decode(p+ 42)!=2||
		dec.decode(p+ 72)!=3) {
		return -1;
	}
	if (opt.find("-EPHALL")==string::npos) {
		if (dec.eph.iode==nav.eph[sat-1].iode) return 0; /* unchanged */
	}
	dec.eph.sat=sat;
	nav.eph[sat-1]=dec.eph;
	ephsat=sat;
	return 2;
}
/* decode rawwaasframeb --------------------------------------------------------------------------- */
int oem4::decode_rawwaasframeb(){
	unsigned char *p=buff+OEM4HLEN;
	int i,prn;

	if (len<OEM4HLEN+48) {
		return -1;
	}
	prn=U4(p+4);

	if (MINPRNQZS_S<=prn&&prn<=MAXPRNQZS_S) {
		prn+=10; /* QZSS SAIF PRN -> QZSS PRN */
	}
	else if (prn<MINPRNSBS||MAXPRNSBS<prn) return 0;

	sbsmsg.tow=(int)time.time2gpst(&sbsmsg.week);
	sbsmsg.prn=prn;
	for (i=0,p+=12; i<29; i++,p++) sbsmsg.msg[i]=*p;
	return 3;
}
/* decode rawsbasframeb --------------------------------------------------------------------------- */
int oem4::decode_rawsbasframeb(){
	/* format same as rawwaasframeb */
	return this->decode_rawwaasframeb();
}
/* decode ionutcb --------------------------------------------------------------------------------- */
int oem4::decode_ionutcb(){
	unsigned char *p=buff+OEM4HLEN;
	int i;

	if (len<OEM4HLEN+108) {
		return -1;
	}
	for (i=0; i<8; i++) nav.ion_gps[i]=R8(p+i*8);
	nav.utc_gps[0]=R8(p+72);
	nav.utc_gps[1]=R8(p+80);
	nav.utc_gps[2]=U4(p+68);
	nav.utc_gps[3]=U4(p+64);
	nav.leaps =I4(p+96);
	return 9;
}
/* decode gloephemerisb --------------------------------------------------------------------------- */
int oem4::decode_gloephemerisb(){
	unsigned char *p=buff+OEM4HLEN;
	geph_t geph=geph_t();
	string str;
	double tow,tof,toff;
	int prn,sat,week;

	if (len<OEM4HLEN+144) {
		return -1;
	}
	prn        =U2(p)-37;

	if (outtype) {
		msgtype+=" prn="+int2str(3," ",prn,str);
	}
	if (!(sat=satno(SYS_GLO,prn))) {
		return -1;
	}
	geph.frq   =U2(p+  2)+OFF_FRQNO;
	week       =U2(p+  6);
	tow        =floor(U4(p+8)/1000.0+0.5); /* rounded to integer sec */
	toff       =U4(p+ 12);
	geph.iode  =U4(p+ 20)&0x7F;
	geph.svh   =U4(p+ 24);
	geph.pos[0]=R8(p+ 28);
	geph.pos[1]=R8(p+ 36);
	geph.pos[2]=R8(p+ 44);
	geph.vel[0]=R8(p+ 52);
	geph.vel[1]=R8(p+ 60);
	geph.vel[2]=R8(p+ 68);
	geph.acc[0]=R8(p+ 76);
	geph.acc[1]=R8(p+ 84);
	geph.acc[2]=R8(p+ 92);
	geph.taun  =R8(p+100);
	geph.gamn  =R8(p+116);
	tof        =U4(p+124)-toff; /* glonasst->gpst */
	geph.age   =U4(p+136);
	geph.toe.gpst2time(week,tow);
	tof+=floor(tow/86400.0)*86400;
	if (tof<tow-43200.0) tof+=86400.0;
	else if (tof>tow+43200.0) tof-=86400.0;
	geph.tof.gpst2time(week,tof);

	if (opt.find("-EPHALL")==string::npos) {
		if (fabs(geph.toe.timediff(nav.geph[prn-1].toe))<1.0&&
			geph.svh==nav.geph[prn-1].svh) return 0; /* unchanged */
	}
	geph.sat=sat;
	nav.geph[prn-1]=geph;
	ephsat=sat;
	return 2;
}
/* decode qzss rawephemb -------------------------------------------------------------------------- */
int oem4::decode_qzssrawephemb(){
	unsigned char *p=buff+OEM4HLEN,*q;
	decode_frame dec=decode_frame();
	string str;
	int i,prn,id,sat;

	if (len<OEM4HLEN+44) {
		return -1;
	}
	prn=U4(p);
	id =U4(p+4);

	if (outtype) {
		msgtype+=" prn="+int2str(3," ",prn,str)+" id="+to_string(id);
	}
	if (!(sat=satno(SYS_QZS,prn))) {
		return -1;
	}
	if (id<1||3<id) return 0;

	q=subfrm[sat-1]+(id-1)*30;
	for (i=0; i<30; i++) *q++=p[8+i];

	if (id<3) return 0;
	if (dec.decode(subfrm[sat-1])!=1||
		dec.decode(subfrm[sat-1]+30)!=2||
		dec.decode(subfrm[sat-1]+60)!=3) {
		return 0;
	}
	if (opt.find("-EPHALL")==string::npos) {
		if (dec.eph.iodc==nav.eph[sat-1].iodc&&
			dec.eph.iode==nav.eph[sat-1].iode) return 0; /* unchanged */
	}
	dec.eph.sat=sat;
	nav.eph[sat-1]=dec.eph;
	ephsat=sat;
	return 2;
}
/* decode qzss rawsubframeb ----------------------------------------------------------------------- */
int oem4::decode_qzssrawsubframeb(){
	unsigned char *p=buff+OEM4HLEN;
	decode_frame dec=decode_frame();
	string str;
	int prn,sat;

	if (len<OEM4HLEN+44) {
		return -1;
	}
	prn=U4(p);

	if (outtype) {
		msgtype+=" prn="+int2str(3," ",prn,str);
	}
	if (!(sat=satno(SYS_QZS,prn))) {
		return -1;
	}
	if (dec.decode(p+12)!=1||
		dec.decode(p+42)!=2||
		dec.decode(p+72)!=3) {
		return 0;
	}
	if (opt.find("-EPHALL")==string::npos) {
		if (dec.eph.iodc==nav.eph[sat-1].iodc&&
			dec.eph.iode==nav.eph[sat-1].iode) return 0; /* unchanged */
	}
	dec.eph.sat=sat;
	nav.eph[sat-1]=dec.eph;
	ephsat=sat;
	return 2;
}
/* decode qzssionutcb ----------------------------------------------------------------------------- */
int oem4::decode_qzssionutcb(){
	unsigned char *p=buff+OEM4HLEN;
	int i;

	if (len<OEM4HLEN+108) {
		return -1;
	}
	for (i=0; i<8; i++) nav.ion_qzs[i]=R8(p+i*8);
	nav.utc_qzs[0]=R8(p+72);
	nav.utc_qzs[1]=R8(p+80);
	nav.utc_qzs[2]=U4(p+68);
	nav.utc_qzs[3]=U4(p+64);
	nav.leaps =I4(p+96);
	return 9;
}
/* decode galephemerisb --------------------------------------------------------------------------- */
int oem4::decode_galephemerisb(){
	eph_t eph=eph_t();
	unsigned char *p=buff+OEM4HLEN;
	double tow,sqrtA,af0_fnav,af1_fnav,af2_fnav,af0_inav,af1_inav,af2_inav,tt;
	string str;
	int prn,rcv_fnav,rcv_inav,svh_e1b,svh_e5a,svh_e5b,dvs_e1b,dvs_e5a,dvs_e5b;
	int toc_fnav,toc_inav,week,sel_nav=0;

	if (len<OEM4HLEN+220) {
		return -1;
	}
	prn       =U4(p);   p+=4;
	rcv_fnav  =U4(p)&1; p+=4;
	rcv_inav  =U4(p)&1; p+=4;
	svh_e1b   =U1(p)&3; p+=1;
	svh_e5a   =U1(p)&3; p+=1;
	svh_e5b   =U1(p)&3; p+=1;
	dvs_e1b   =U1(p)&1; p+=1;
	dvs_e5a   =U1(p)&1; p+=1;
	dvs_e5b   =U1(p)&1; p+=1;
	eph.sva   =U1(p);   p+=1+1; /* SISA */
	eph.iode  =U4(p);   p+=4;   /* IODNav */
	eph.toes  =U4(p);   p+=4;
	sqrtA     =R8(p);   p+=8;
	eph.deln  =R8(p);   p+=8;
	eph.M0    =R8(p);   p+=8;
	eph.e     =R8(p);   p+=8;
	eph.omg   =R8(p);   p+=8;
	eph.cuc   =R8(p);   p+=8;
	eph.cus   =R8(p);   p+=8;
	eph.crc   =R8(p);   p+=8;
	eph.crs   =R8(p);   p+=8;
	eph.cic   =R8(p);   p+=8;
	eph.cis   =R8(p);   p+=8;
	eph.i0    =R8(p);   p+=8;
	eph.idot  =R8(p);   p+=8;
	eph.OMG0  =R8(p);   p+=8;
	eph.OMGd  =R8(p);   p+=8;
	toc_fnav  =U4(p);   p+=4;
	af0_fnav  =R8(p);   p+=8;
	af1_fnav  =R8(p);   p+=8;
	af2_fnav  =R8(p);   p+=8;
	toc_inav  =U4(p);   p+=4;
	af0_inav  =R8(p);   p+=8;
	af1_inav  =R8(p);   p+=8;
	af2_inav  =R8(p);   p+=8;
	eph.tgd[0]=R8(p);   p+=8; /* BGD: E5A-E1 (s) */
	eph.tgd[1]=R8(p);         /* BGD: E5B-E1 (s) */
	eph.iodc  =eph.iode;
	eph.svh   =(svh_e5b<<7)|(dvs_e5b<<6)|(svh_e5a<<4)|(dvs_e5a<<3)|
		(svh_e1b<<1)|dvs_e1b;

	/* ephemeris selection (0:INAV,1:FNAV) */
	if (opt.find("-GALINAV")!=string::npos) sel_nav=0;
	else if (opt.find("-GALFNAV")!=string::npos) sel_nav=1;
	else if (!rcv_inav&&rcv_fnav) sel_nav=1;

	eph.A     =sqrtA*sqrtA;
	eph.f0    =sel_nav ? af0_fnav : af0_inav;
	eph.f1    =sel_nav ? af1_fnav : af1_inav;
	eph.f2    =sel_nav ? af2_fnav : af2_inav;
	eph.code  =sel_nav ? 2 : 1; /* data source 1:I/NAV E1B,2:F/NAV E5a-I */

	if (outtype) {
		msgtype+=" prn="+int2str(3," ",prn,str)+" iod="+int2str(3," ",eph.iode,str)+
			" toes="+doul2str(6,0," ",eph.toes,str);
	}
	if (!(eph.sat=satno(SYS_GAL,prn))) {
		return -1;
	}
	tow=time.time2gpst(&week);
	eph.week=week; /* gps week */
	eph.toe.gpst2time(eph.week,eph.toes);

	/* for week-handover problem */
	tt=eph.toe.timediff(time);
	if (tt<-302400.0) eph.week++;
	else if (tt> 302400.0) eph.week--;
	eph.toe.gpst2time(eph.week,eph.toes);
	eph.toc=adjweek(eph.toe,sel_nav ? toc_fnav : toc_inav);
	eph.ttr=adjweek(eph.toe,tow);

	if (opt.find("-EPHALL")==string::npos) {
		if (nav.eph[eph.sat-1].iode==eph.iode&&
			nav.eph[eph.sat-1].code==eph.code) return 0; /* unchanged */
	}
	nav.eph[eph.sat-1]=eph;
	ephsat=eph.sat;
	return 2;
}
/* decode galalmanacb ----------------------------------------------------------------------------- */
int oem4::decode_galalmanacb(){
	alm_t alm=alm_t();
	unsigned char *p=buff+OEM4HLEN;
	double dsqrtA,sqrtA=sqrt(29601297.0);
	int prn,rcv_fnav,rcv_inav,svh_e1b,svh_e5a,svh_e5b,ioda;

	if (len<OEM4HLEN+100) {
		return -1;
	}
	prn     =U4(p);   p+=4;
	rcv_fnav=U4(p)&1; p+=4;
	rcv_inav=U4(p)&1; p+=4;
	svh_e1b =U1(p)&3; p+=1;
	svh_e5a =U1(p)&3; p+=1;
	svh_e5b =U1(p)&3; p+=1+1;
	ioda    =U4(p);   p+=4;
	alm.week=U4(p);   p+=4; /* gst week */
	alm.toas=U4(p);   p+=4;
	alm.e   =R8(p);   p+=8;
	alm.OMGd=R8(p);   p+=8;
	alm.OMG0=R8(p);   p+=8;
	alm.omg =R8(p);   p+=8;
	alm.M0  =R8(p);   p+=8;
	alm.f0  =R8(p);   p+=8;
	alm.f1  =R8(p);   p+=8;
	dsqrtA  =R8(p);   p+=8;
	alm.i0  =(R8(p)+56.0)*D2R;
	alm.svh =(svh_e5b<<7)|(svh_e5a<<4)|(svh_e1b<<1);
	alm.A   =(sqrtA+dsqrtA)*(sqrtA+dsqrtA);

	if (!(alm.sat=satno(SYS_GAL,prn))) {
		return -1;
	}
	alm.toa.gst2time(alm.week,alm.toas);
	nav.alm[alm.sat-1]=alm;
	return 0;
}
/* decode galclockb ------------------------------------------------------------------------------- */
int oem4::decode_galclockb(){
	unsigned char *p=buff+OEM4HLEN;
	double a0,a1,a0g,a1g;
	int leaps,tot,wnt,wnlsf,dn,dtlsf,t0g,wn0g;

	if (len<OEM4HLEN+64) {
		return -1;
	}
	a0   =R8(p); p+=8;
	a1   =R8(p); p+=8;
	leaps=I4(p); p+=4;
	tot  =U4(p); p+=4;
	wnt  =U4(p); p+=4;
	wnlsf=U4(p); p+=4;
	dn   =U4(p); p+=4;
	dtlsf=U4(p); p+=4;
	a0g  =R8(p); p+=8;
	a1g  =R8(p); p+=8;
	t0g  =U4(p); p+=4;
	wn0g =U4(p);

	nav.utc_gal[0]=a0;
	nav.utc_gal[1]=a1;
	nav.utc_gal[2]=tot; /* utc reference tow (s) */
	nav.utc_gal[3]=wnt; /* utc reference week */
	return 9;
}
/* decode galionob -------------------------------------------------------------------------------- */
int oem4::decode_galionob(){
	unsigned char *p=buff+OEM4HLEN;
	double ai[3];
	int i,sf[5];

	if (len<OEM4HLEN+29) {
		return -1;
	}
	ai[0]=R8(p); p+=8;
	ai[1]=R8(p); p+=8;
	ai[2]=R8(p); p+=8;
	sf[0]=U1(p); p+=1;
	sf[1]=U1(p); p+=1;
	sf[2]=U1(p); p+=1;
	sf[3]=U1(p); p+=1;
	sf[4]=U1(p);

	for (i=0; i<3; i++) nav.ion_gal[i]=ai[i];
	return 9;
}
/* decode galfnavrawpageb ------------------------------------------------------------------------- */
int oem4::decode_galfnavrawpageb(){
	unsigned char *p=buff+OEM4HLEN;
	unsigned char buff[27];
	int i,sigch,satid,page;

	if (len<OEM4HLEN+35) {
		return -1;
	}
	sigch=U4(p); p+=4;
	satid=U4(p); p+=4;
	for (i=0; i<27; i++) {
		buff[i]=U1(p); p+=1;
	}
	page=getbitu(buff,0,6);

	return 0;
}
/* decode galinavrawwordb ----------------------------------------------------------------------- */
int oem4::decode_galinavrawwordb(){
	unsigned char *p=buff+OEM4HLEN;
	unsigned char buff[16];
	gtime_t ttt=time;
	string sig;
	int i,sigch,satid,sigtype,type,week=0,tow=0;

	if (len<OEM4HLEN+28) {
		return -1;
	}
	sigch  =U4(p); p+=4;
	satid  =U4(p); p+=4;
	sigtype=U4(p); p+=4;

	switch (sigtype) {
		case 10433: sig="E1 "; break;
		case 10466: sig="E5A"; break;
		case 10499: sig="E5B"; break;
		default: sig="???"; break;
	}
	for (i=0; i<16; i++) {
		buff[i]=U1(p); p+=1;
	}
	type=getbitu(buff,0,6);
	if (type==0&&getbitu(buff,6,2)==2) {
		week=getbitu(buff,96,12); /* gst week */
		tow =getbitu(buff,108,20);
		ttt.gst2time(week,tow);
	}

	return 0;
}
/* decode rawcnavframeb ------------------------------------------------------------------------- */
int oem4::decode_rawcnavframeb(){
	unsigned char *p=buff+OEM4HLEN;
	unsigned char buff[38];
	int i,sigch,prn,frmid;

	if (len<OEM4HLEN+50) {
		return -1;
	}
	sigch=U4(p); p+=4;
	prn  =U4(p); p+=4;
	frmid=U4(p); p+=4;

	for (i=0; i<38; i++) {
		buff[i]=U1(p); p+=1;
	}

	return 0;
}
/* decode bdsephemerisb ------------------------------------------------------------------------- */
int oem4::decode_bdsephemerisb(){
	eph_t eph=eph_t();
	unsigned char *p=buff+OEM4HLEN;
	double ura,sqrtA;
	string str;
	int prn,toc;

	if (len<OEM4HLEN+196) {
		return -1;
	}
	prn       =U4(p);   p+=4;
	eph.week  =U4(p);   p+=4;
	ura       =R8(p);   p+=8;
	eph.svh   =U4(p)&1; p+=4;
	eph.tgd[0]=R8(p);   p+=8; /* TGD1 for B1 (s) */
	eph.tgd[1]=R8(p);   p+=8; /* TGD2 for B2 (s) */
	eph.iodc  =U4(p);   p+=4; /* AODC */
	toc       =U4(p);   p+=4;
	eph.f0    =R8(p);   p+=8;
	eph.f1    =R8(p);   p+=8;
	eph.f2    =R8(p);   p+=8;
	eph.iode  =U4(p);   p+=4; /* AODE */
	eph.toes  =U4(p);   p+=4;
	sqrtA     =R8(p);   p+=8;
	eph.e     =R8(p);   p+=8;
	eph.omg   =R8(p);   p+=8;
	eph.deln  =R8(p);   p+=8;
	eph.M0    =R8(p);   p+=8;
	eph.OMG0  =R8(p);   p+=8;
	eph.OMGd  =R8(p);   p+=8;
	eph.i0    =R8(p);   p+=8;
	eph.idot  =R8(p);   p+=8;
	eph.cuc   =R8(p);   p+=8;
	eph.cus   =R8(p);   p+=8;
	eph.crc   =R8(p);   p+=8;
	eph.crs   =R8(p);   p+=8;
	eph.cic   =R8(p);   p+=8;
	eph.cis   =R8(p);
	eph.A     =sqrtA*sqrtA;
	eph.sva   =uraindex(ura);

	if (outtype) {
		msgtype+=" prn="+int2str(3," ",prn,str)+" iod="+int2str(3," ",eph.iode,str)+
			" toes="+doul2str(6,0," ",eph.toes,str);
	}
	if (!(eph.sat=satno(SYS_CMP,prn))) {
		return -1;
	}
	eph.toe.bdt2time(eph.week,eph.toes)->bdt2gpst(); /* bdt -> gpst */
	eph.toc.bdt2time(eph.week,toc)->bdt2gpst();      /* bdt -> gpst */
	eph.ttr=time;

	if (opt.find("-EPHALL")==string::npos) {
		if (nav.eph[eph.sat-1].toe.timediff(eph.toe)==0.0) return 0; /* unchanged */
	}
	nav.eph[eph.sat-1]=eph;
	ephsat=eph.sat;
	return 2;
}
/* crc-32 parity -------------------------------------------------------------------------------- */
unsigned int oem4::rtk_crc32(){
	unsigned int crc=0;
	int i,j;

	for (i=0; i<len; i++) {
		crc^=buff[i];
		for (j=0; j<8; j++) {
			if (crc&1) crc=(crc>>1)^POLYCRC32; else crc>>=1;
		}
	}
	return crc;
}
/* sync_oem4 header ------------------------------------------------------------------------------- */
int oem4::sync_oem4(unsigned char data){
	buff[0]=buff[1]; buff[1]=buff[2]; buff[2]=data;
	return buff[0]==OEM4SYNC1&&buff[1]==OEM4SYNC2&&buff[2]==OEM4SYNC3;
}
/* decode oem4 message -------------------------------------------------------*/
int oem4::decode_oem4()
{
	double tow;
	int msg,week,type=U2(buff+4);
	string str;

	/* check crc32 */
	if (rtk_crc32()!=U4(buff+len)) {
		return -1;
	}
	msg =(U1(buff+6)>>4)&0x3;
	if (!(week=U2(buff+14))) {
		return -1;
	}
	week=adjgpsweek(week);
	tow =U4(buff+16)*0.001;
	time.gpst2time(week,tow);

	if (outtype) {
		msgtype="OEM4"+int2str(4," ",type,str)+"("+int2str(4," ",len,str)+"): msg="+
			to_string(msg)+time.time2str(1);
	}
	if (msg!=0) return 0; /* message type: 0=binary,1=ascii */

	switch (type) {
	case ID_RANGECMP		: return decode_rangecmpb();
	case ID_RANGE			: return decode_rangeb();
	case ID_RAWEPHEM		: return decode_rawephemb();
	case ID_RAWWAASFRAME	: return decode_rawwaasframeb();
	case ID_RAWSBASFRAME	: return decode_rawsbasframeb();
	case ID_IONUTC			: return decode_ionutcb();
	case ID_GLOEPHEMERIS	: return decode_gloephemerisb();
	case ID_QZSSRAWEPHEM	: return decode_qzssrawephemb();
	case ID_QZSSRAWSUBFRAME	: return decode_qzssrawsubframeb();
	case ID_QZSSIONUTC		: return decode_qzssionutcb();
	case ID_GALEPHEMERIS	: return decode_galephemerisb();
	case ID_GALALMANAC		: return decode_galalmanacb();
	case ID_GALCLOCK		: return decode_galclockb();
	case ID_GALIONO			: return decode_galionob();
	case ID_GALFNAVRAWPAGE	: return decode_galfnavrawpageb();
	case ID_GALINAVRAWWORD	: return decode_galinavrawwordb();
	case ID_RAWCNAVFRAME	: return decode_rawcnavframeb();
	case ID_BDSEPHEMERIS	: return decode_bdsephemerisb();
	}
	return 0;
}
/* raw data */
/* input oem4 raw data from stream ----------------------------------------------------------------
* fetch next novatel oem4/oem3 raw data and input a mesasge from stream
* args   : raw_t *raw   IO     receiver raw data control struct
*          unsigned char data I stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : to specify input options for oem4, set opt to the following
*          option strings separated by spaces.
*
*          -EPHALL : input all ephemerides
*          -GL1P   : select 1P for GPS L1 (default 1C)
*          -GL2X   : select 2X for GPS L2 (default 2W)
*          -RL2C   : select 2C for GLO L2 (default 2P)
*          -EL2C   : select 2C for GAL L2 (default 2C)
*          -GALINAV: use I/NAV for GAL ephemeris
*          -GALFNAV: use F/NAV for GAL ephemeris
*
*-------------------------------------------------------------------------------------------------- */
int oem4::decode(unsigned char data){
	/* synchronize frame */
	if (nbyte==0) {
		if (sync_oem4(data)) nbyte=3;
		return 0;
	}
	buff[nbyte++]=data;

	if (nbyte==10&&(len=U2(buff+8)+OEM4HLEN)>MAXRAWLEN-4) {
		nbyte=0;
		return -1;
	}
	if (nbyte<10||nbyte<len+4) return 0;
	nbyte=0;

	/* decode oem4 message */
	return decode_oem4();
}

/* input oem3 raw data from stream -------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
oem3::oem3(){
}
oem3::~oem3(){
}
/* input oem3 raw data from stream -------------------------------------------------------------    */
/* sync_oem3 header ------------------------------------------------------------------------------- */
int oem3::sync_oem3(unsigned char data){
	buff[0]=buff[1]; buff[1]=buff[2]; buff[2]=data;
	return buff[0]==OEM3SYNC1&&buff[1]==OEM3SYNC2&&buff[2]==OEM3SYNC3;
}

/* decode oem3 message -------------------------------------------------------------------------- */
int oem3::decode_oem3(){
	int type=U4(buff+4);
	string str;

	/* checksum */
	if (chksum()) {
		return -1;
	}
	if (outtype) {
		msgtype="OEM3 "+int2str(4," ",type,str)+" ("+int2str(4," ",len,str)+"):";
	}
	switch (type) {
		case ID_RGEB: return decode_rgeb();
		case ID_RGED: return decode_rged();
		case ID_REPB: return decode_repb();
		case ID_FRMB: return decode_frmb();
		case ID_IONB: return decode_ionb();
		case ID_UTCB: return decode_utcb();
	}
	return 0;
}
int oem3::decode(unsigned char data){/* synchronize frame */
	if (nbyte==0) {
		if (sync_oem3(data)) nbyte=3;
		return 0;
	}
	buff[nbyte++]=data;

	if (nbyte==12&&(len=U4(buff+8))>MAXRAWLEN) {
		nbyte=0;
		return -1;
	}
	if (nbyte<12||nbyte<len) return 0;
	nbyte=0;

	/* decode oem3 message */
	return decode_oem3();
}
