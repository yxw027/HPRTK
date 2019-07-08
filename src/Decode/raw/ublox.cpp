#include "ublox.h"

#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
#define UBXSYNC1    0xB5        /* ubx message sync code 1 */
#define UBXSYNC2    0x62        /* ubx message sync code 2 */
#define UBXCFG      0x06        /* ubx message cfg-??? */

#define ID_NAVSOL   0x0106      /* ubx message id: nav solution info */
#define ID_NAVTIME  0x0120      /* ubx message id: nav time gps */
#define ID_RXMRAW   0x0210      /* ubx message id: raw measurement data */
#define ID_RXMSFRB  0x0211      /* ubx message id: subframe buffer */
#define ID_RXMSFRBX 0x0213      /* ubx message id: raw subframe data */
#define ID_RXMRAWX  0x0215      /* ubx message id: multi-gnss raw meas data */
#define ID_TRKD5    0x030A      /* ubx message id: trace mesurement data */
#define ID_TRKMEAS  0x0310      /* ubx message id: trace mesurement data */
#define ID_TRKSFRBX 0x030F      /* ubx message id: trace subframe buffer */

#define P2_10       0.0009765625 /* 2^-10 */

#define CPSTD_VALID 5           /* std-dev threshold of carrier-phase valid */

#define ROUND(x)    (int)floor((x)+0.5)

static const char rcsid[]="$Id: ublox.c,v 1.2 2008/07/14 00:05:05 TTAKA Exp $";

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))
static unsigned short U2(unsigned char *p) { unsigned short u; memcpy(&u,p,2); return u; }
static unsigned int   U4(unsigned char *p) { unsigned int   u; memcpy(&u,p,4); return u; }
static int            I4(unsigned char *p) { int            u; memcpy(&u,p,4); return u; }
static float          R4(unsigned char *p) { float          r; memcpy(&r,p,4); return r; }
static double         R8(unsigned char *p) { double         r; memcpy(&r,p,8); return r; }

static double         I8(unsigned char *p) { return I4(p+4)*4294967296.0+U4(p); }

/* input ublox raw message from stream ---------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
ublox::ublox(){
}
ublox::~ublox(){
}
/* Implementation functions */
/* checksum --------------------------------------------------------------------------------------- */
int ublox::checksum()
{
	unsigned char cka=0,ckb=0;
	int i;

	for (i=2; i<len-2; i++) {
		cka+=buff[i]; ckb+=cka;
	}
	return cka==buff[len-2]&&ckb==buff[len-1];
}
void ublox::setcs(){
	unsigned char cka=0,ckb=0;
	int i;

	for (i=2; i<len-2; i++) {
		cka+=buff[i]; ckb+=cka;
	}
	buff[len-2]=cka;
	buff[len-1]=ckb;
}
/* ubx gnss indicator (ref [2] 25) ---------------------------------------------------------------- */
int ublox::ubx_sys(int ind){
	switch (ind) {
	case 0: return SYS_GPS;
	case 1: return SYS_SBS;
	case 2: return SYS_GAL;
	case 3: return SYS_CMP;
	case 5: return SYS_QZS;
	case 6: return SYS_GLO;
	}
	return 0;
}
/* 8-bit week -> full week ------------------------------------------------------------------------ */
void ublox::adj_utcweek(double *utc){
	int week;

	if (utc[3]>=256.0) return;
	time.time2gpst(&week);
	utc[3]+=week/256*256;
	if (utc[3]<week-128) utc[3]+=256.0;
	else if (utc[3]>week+128) utc[3]-=256.0;
}
/* save subframe ---------------------------------------------------------------------------------- */
int ublox::save_subfrm(int sat){
	unsigned char *p=buff+6,*q;
	int i,j,n,id=(U4(p+6)>>2)&0x7;

	if (id<1||5<id) return 0;

	q=subfrm[sat-1]+(id-1)*30;

	for (i=n=0,p+=2; i<10; i++,p+=4) {
		for (j=23; j>=0; j--) {
			*q=(*q<<1)+((U4(p)>>j)&1); if (++n%8==0) q++;
		}
	}
	return id;
}
/* decode ephemeris ------------------------------------------------------------------------------- */
int ublox::decode_ephem(int sat){
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
/* decode almanac and ion/utc --------------------------------------------------------------------- */
int ublox::decode_alm1(int sat){
	decode_frame dec;
	int sys=satsys(sat,NULL);

	if (sys==SYS_GPS) {
		dec.decode(subfrm[sat-1]+90);
		/* assign dec to nav */
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
		adj_utcweek(nav.utc_gps);
	}
	return 9;
}
/* decode almanac --------------------------------------------------------------------------------- */
int ublox::decode_alm2(int sat){
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

/* decode gps and qzss navigation data ------------------------------------------------------------ */
int ublox::decode_nav(int sat,int offside){
	unsigned int words[10];
	int i,id;
	unsigned char *p=buff+6+offside;

	if (len<48+offside) {
		return -1;
	}
	for (i=0; i<10; i++,p+=4) words[i]=U4(p)>>6; /* 24 bits without parity */

	id=(words[1]>>2)&7;
	if (id<1||5<id) {
		return -1;
	}
	for (i=0; i<10; i++) {
		setbitu(subfrm[sat-1]+(id-1)*30,i*24,24,words[i]);
	}
	if (id==3) return decode_ephem(sat);
	if (id==4) return decode_alm1(sat);
	if (id==5) return decode_alm2(sat);
	return 0;
}
/* decode galileo navigation data ----------------------------------------------------------------- */
int ublox::decode_enav(int sat,int offside){
	eph_t eph;
	unsigned char *p=buff+6+offside,bbuff[32],crc_buff[26]={ 0 };
	int i,j,k,part1,page1,part2,page2,type;

	if (len<44+offside) {
		return -1;
	}
	for (i=k=0; i<8; i++,p+=4) for (j=0; j<4; j++) {
		bbuff[k++]=p[3-j];
	}
	part1=getbitu(bbuff,0,1);
	page1=getbitu(bbuff,1,1);
	part2=getbitu(bbuff+16,0,1);
	page2=getbitu(bbuff+16,1,1);

	/* skip alert page */
	if (page1==1||page2==1) return 0;

	/* test even-odd parts */
	if (part1!=0||part2!=1) {
		return -1;
	}
	/* test crc (4(pad) + 114 + 82 bits) */
	for (i=0,j= 4; i<15; i++,j+=8) setbitu(crc_buff,j,8,getbitu(bbuff,i*8,8));
	for (i=0,j=118; i<11; i++,j+=8) setbitu(crc_buff,j,8,getbitu(bbuff+16,i*8,8));
	if (rtk_crc24q(crc_buff,25)!=getbitu(bbuff+16,82,24)) {
		return -1;
	}
	type=getbitu(bbuff,2,6); /* word type */

							/* skip word except for ephemeris, iono, utc parameters */
	if (type>6) return 0;

	/* clear word 0-6 flags */
	if (type==2) subfrm[sat-1][112]=0;

	/* save page data (112 + 16 bits) to frame buffer */
	k=type*16;
	for (i=0,j=2; i<14; i++,j+=8) subfrm[sat-1][k++]=getbitu(bbuff,j,8);
	for (i=0,j=2; i< 2; i++,j+=8) subfrm[sat-1][k++]=getbitu(bbuff+16,j,8);

	/* test word 0-6 flags */
	subfrm[sat-1][112]|=(1<<type);
	if (subfrm[sat-1][112]!=0x7F) return 0;

	/* decode galileo inav ephemeris */
	if (!decode_gal_inav(sat,eph)) {
		return 0;
	}
	/* test svid consistency */
	if (eph.sat!=sat) {
		return -1;
	}
	if (opt.find("-EPHALL")==string::npos) {
		if (eph.iode==nav.eph[sat-1].iode&& /* unchanged */
			eph.toe.timediff(nav.eph[sat-1].toe)==0.0&&
			eph.toc.timediff(nav.eph[sat-1].toc)==0.0) return 0;
	}
	eph.sat=sat;
	nav.eph[sat-1]=eph;
	ephsat=sat;
	return 2;
}
/* decode beidou navigation data ------------------------------------------------------------------ */
int ublox::decode_cnav(int sat,int offside){
	eph_t eph;
	unsigned int words[10];
	int i,id,pgn,prn;
	unsigned char *p=buff+6+offside;

	if (len<48+offside) {
		return -1;
	}
	for (i=0; i<10; i++,p+=4) words[i]=U4(p)&0x3FFFFFFF; /* 30 bits */

	satsys(sat,&prn);
	id=(words[0]>>12)&0x07; /* subframe id (3bit) */
	if (id<1||5<id) {
		return -1;
	}
	if (prn>=5) { /* IGSO/MEO */

		for (i=0; i<10; i++) {
			setbitu(subfrm[sat-1]+(id-1)*38,i*30,30,words[i]);
		}
		if (id!=3) return 0;

		/* decode beidou D1 ephemeris */
		if (!decode_bds_d1(sat,eph)) return 0;
	}
	else { /* GEO */
		if (id!=1) return 0;

		/* subframe 1 */
		pgn=(words[1]>>14)&0x0F; /* page number (4bit) */
		if (pgn<1||10<pgn) {
			return -1;
		}
		for (i=0; i<10; i++) {
			setbitu(subfrm[sat-1]+(pgn-1)*38,i*30,30,words[i]);
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
/* decode glonass navigation data ----------------------------------------------------------------- */
int ublox::decode_gnav(int sat,int offside,int frq){
	geph_t geph;
	int i,j,k,m,prn;
	unsigned char *p=buff+6+offside,buff[64],*fid;

	satsys(sat,&prn);

	if (len<24+offside) {
		return -1;
	}
	for (i=k=0; i<4; i++,p+=4) for (j=0; j<4; j++) {
		buff[k++]=p[3-j];
	}
	/* test hamming of glonass string */
	if (!test_glostr(buff)) {
		return -1;
	}
	m=getbitu(buff,1,4);
	if (m<1||15<m) {
		return -1;
	}
	/* flush frame buffer if frame-id changed */
	fid=subfrm[sat-1]+150;
	if (fid[0]!=buff[12]||fid[1]!=buff[13]) {
		for (i=0; i<4; i++) memset(subfrm[sat-1]+i*10,0,10);
		memcpy(fid,buff+12,2); /* save frame-id */
	}
	memcpy(subfrm[sat-1]+(m-1)*10,buff,10);

	if (m!=4) return 0;

	/* decode glonass ephemeris strings */
	geph.tof=time;
	if (!decode_glostr(sat,geph)||geph.sat!=sat) return 0;
	geph.frq=frq-7;

	if (opt.find("-EPHALL")==string::npos) {
		if (geph.iode==nav.geph[prn-1].iode) return 0; /* unchanged */
	}
	nav.geph[prn-1]=geph;
	ephsat=sat;
	return 2;
}
/* decode sbas navigation data -------------------------------------------------------------------- */
int ublox::decode_snav(int sat,int offside){
	int i,j,k,prn,tow,week;
	unsigned char *p=buff+6+offside,bbuff[64];

	if (len<40+offside) {
		return -1;
	}
	tow=(int)time.timeadd(-1.0)->time2gpst(&week);
	satsys(sat,&prn);
	sbsmsg.prn=prn;
	sbsmsg.tow=tow;
	sbsmsg.week=week;
	for (i=k=0; i<8; i++,p+=4) for (j=0; j<4; j++) {
		bbuff[k++]=p[3-j];
	}
	memcpy(sbsmsg.msg,bbuff,29);
	sbsmsg.msg[28]&=0xC0;
	return 3;
}

/* decode ubx-rxm-raw: raw measurement data ------------------------------------------------------- */
int ublox::decode_rxmraw(){
	gtime_t ttime;
	double tow,tt,tadj=0.0,toff=0.0,tn;
	int i,j,prn,sat,n=0,nsat,week;
	unsigned char *p=buff+6;
	string str;

	if (outtype) {
		msgtype="UBX RXM-RAW   ("+int2str(4," ",len,str)+"): nsat="+to_string(U1(p+6));
	}
	/* time tag adjustment option (-TADJ) */
	if ((opt.find("-TADJ=")!=string::npos)) {
		opt+="-TADJ="+to_string(tadj);
	}
	nsat=U1(p+6);
	if (len<12+24*nsat) {
		return -1;
	}
	tow =U4(p);
	week=U2(p+4);
	ttime.gpst2time(week,tow*0.001);

	if (week==0) {
		return -1;
	}
	/* time tag adjustment */
	if (tadj>0.0) {
		tn=ttime.time2gpst(&week)/tadj;
		toff=(tn-floor(tn+0.5))*tadj;
		ttime.timeadd(-toff);
	}
	tt=ttime.timediff(time);

	for (i=0,p+=8; i<nsat&&i<MAXOBS; i++,p+=24) {
		obs.data[n].time=ttime;
		obs.data[n].L[0]  =R8(p)-toff*FREQ1;
		obs.data[n].P[0]  =R8(p+ 8)-toff*CLIGHT;
		obs.data[n].D[0]  =R4(p+16);
		prn                    =U1(p+20);
		obs.data[n].SNR[0]=(unsigned char)(I1(p+22)*4.0+0.5);
		obs.data[n].LLI[0]=U1(p+23);
		obs.data[n].code[0]=CODE_L1C;

		/* phase polarity flip option (-INVCP) */
		if (opt.find("-INVCP")!=string::npos) {
			obs.data[n].L[0]=-obs.data[n].L[0];
		}
		if (!(sat=satno(MINPRNSBS<=prn ? SYS_SBS : SYS_GPS,prn))) {
			continue;
		}
		obs.data[n].sat=sat;

		if (obs.data[n].LLI[0]&1) lockt[sat-1][0]=0.0;
		else if (tt<1.0||10.0<tt) lockt[sat-1][0]=0.0;
		else lockt[sat-1][0]+=tt;

		for (j=1; j<NFREQ+NEXOBS; j++) {
			obs.data[n].L[j]=obs.data[n].P[j]=0.0;
			obs.data[n].D[j]=0.0;
			obs.data[n].SNR[j]=obs.data[n].LLI[j]=0;
			obs.data[n].code[j]=CODE_NONE;
		}
		n++;
	}
	time=ttime;
	obs.n=n;
	return 1;
}
/* decode ubx-rxm-rawx: multi-gnss raw measurement data (ref [3]) --------------------------------- */
int ublox::decode_rxmrawx(){
	gtime_t ttime;
	double tow,cp1,pr1,tadj=0.0,toff=0.0,freq,tn;
	int i,j,sys,prn,sat,n=0,nsat,week,tstat,locktt,slip,halfv,halfcc,fcn,cpstd;
	int std_slip=0;
	string str;
	unsigned char *p=buff+6;

	nsat=U1(p+11);
	if (len<24+32*nsat) {
		return -1;
	}
	tow=R8(p);
	week=U2(p+8);
	ttime.gpst2time(week,tow);

	if (week==0) {
		return -1;
	}
	if (outtype) {
		msgtype="UBX RXM-RAWX  ("+int2str(4," ",len,str)+"): time="+ttime.time2str(2)
			+" nsat="+to_string(U1(p+11));
			
	}
	/* time tag adjustment option (-TADJ) */
	if ((opt.find("-TADJ="))!=string::npos) {
		opt+="-TADJ="+to_string(tadj);
	}
	/* slip theshold of std-dev of carreir-phase (-STD_SLIP) */
	if ((opt.find("-STD_SLIP="))!=string::npos) {
		opt+="-STD_SLIP="+to_string(std_slip);
	}
	/* time tag adjustment */
	if (tadj>0.0) {
		tn=ttime.time2gpst(&week)/tadj;
		toff=(tn-floor(tn+0.5))*tadj;
		ttime.timeadd(-toff);
	}
	for (i=0,p+=16; i<nsat&&i<MAXOBS; i++,p+=32) {

		if (!(sys=ubx_sys(U1(p+20)))) {
			continue;
		}
		prn=U1(p+21)+(sys==SYS_QZS ? 192 : 0);
		if (!(sat=satno(sys,prn))) {
			continue;
		}
		cpstd=U1(p+28)&15; /* carrier-phase std-dev */
		tstat=U1(p+30); /* tracking status */
		pr1=tstat&1 ? R8(p) : 0.0;
		cp1=tstat&2 ? R8(p+8) : 0.0;
		if (cp1==-0.5||cpstd>CPSTD_VALID) cp1=0.0; /* invalid phase */
		obs.data[n].sat=sat;
		obs.data[n].time=ttime;
		obs.data[n].P[0]=pr1;
		obs.data[n].L[0]=cp1;

		/* offset by time tag adjustment */
		if (toff!=0.0) {
			fcn=(int)U1(p+23)-7;
			freq=sys==SYS_CMP ? ( prn<18? FREQ1_CMP:FREQ1_CMP_3 ):
				(sys==SYS_GLO ? FREQ1_GLO+DFRQ1_GLO*fcn : FREQ1);
			obs.data[n].P[0]-=toff*CLIGHT;
			obs.data[n].L[0]-=toff*freq;
		}
		obs.data[n].D[0]=R4(p+16);
		obs.data[n].SNR[0]=U1(p+26)*4;
		obs.data[n].LLI[0]=0;
		obs.data[n].code[0]=
			sys==SYS_CMP ? CODE_L1I : (sys==SYS_GAL ? CODE_L1X : CODE_L1C);

		locktt=U2(p+24);    /* lock time count (ms) */
		slip=locktt==0||locktt<lockt[sat-1][0] ? 1 : 0;
		if (std_slip>0) {
			slip|=(cpstd>=std_slip) ? 1 : 0; /* slip by std-dev of cp */
		}
		halfv=tstat&4 ? 1 : 0; /* half cycle valid */
		halfcc=tstat&8 ? 1 : 0; /* half cycle subtracted from phase */

		if (cp1!=0.0) { /* carrier-phase valid */

						/* LLI: bit1=loss-of-lock,bit2=half-cycle-invalid */
			obs.data[n].LLI[0]|=slip;
			obs.data[n].LLI[0]|=halfcc!=halfc[sat-1][0] ? 1 : 0;
			obs.data[n].LLI[0]|=halfv ? 0 : 2;
			lockt[sat-1][0]=locktt;
			halfc[sat-1][0]=halfcc;
		}
		for (j=1; j<NFREQ+NEXOBS; j++) {
			obs.data[n].L[j]=obs.data[n].P[j]=0.0;
			obs.data[n].D[j]=0.0;
			obs.data[n].SNR[j]=obs.data[n].LLI[j]=0;
			obs.data[n].code[j]=CODE_NONE;
		}
		n++;
	}
	time=ttime;
	obs.n=n;
	return 1;
}
/* decode ubx-rxm-sfrb: subframe buffer ----------------------------------------------------------- */
int ublox::decode_rxmsfrb(){
	unsigned int words[10];
	int i,prn,sat,sys,id;
	unsigned char *p=buff+6;

	if (outtype) {
		sprintf((char *)msgtype.c_str(), "UBX RXM-SFRB  (%4d): prn=%2d",len,U1(p+1));
	}
	if (len<42) {
		return -1;
	}
	prn=U1(p+1);
	if (!(sat=satno(MINPRNSBS<=prn ? SYS_SBS : SYS_GPS,prn))) {
		return -1;
	}
	sys=satsys(sat,&prn);

	if (sys==SYS_GPS) {
		id=save_subfrm(sat);
		if (id==3) return decode_ephem(sat);
		if (id==4) return decode_alm1(sat);
		if (id==5) return decode_alm2(sat);
		return 0;
	}
	else if (sys==SYS_SBS) {
		for (i=0,p+=2; i<10; i++,p+=4) words[i]=U4(p);
		return sbsdecodemsg(prn,words) ? 3 : 0;
	}
	return 0;
}
/* decode ubx-rxm-sfrbx: raw subframe data (ref [3]) ---------------------------------------------- */
int ublox::decode_rxmsfrbx(){
	int prn,sat,sys;
	unsigned char *p=buff+6;
	string str;

	if (outtype) {
		msgtype="UBX RXM-SFRBX ("+int2str(4," ",len,str)+"): sys="+to_string(U1(p))+
			" prn="+int2str(3," ",U1(p+1),str);
	}
	if (!(sys=ubx_sys(U1(p)))) {
		return -1;
	}
	prn=U1(p+1)+(sys==SYS_QZS ? 192 : 0);
	if (!(sat=satno(sys,prn))) {
		return -1;
	}
	switch (sys) {
	case SYS_GPS: return decode_nav(sat,8);
	case SYS_QZS: return decode_nav(sat,8);
	case SYS_GAL: return decode_enav(sat,8);
	case SYS_CMP: return decode_cnav(sat,8);
	case SYS_GLO: return decode_gnav(sat,8,U1(p+3));
	case SYS_SBS: return decode_snav(sat,8);
	}
	return 0;
}
/* decode ubx-nav-sol: navigation solution -------------------------------------------------------- */
int ublox::decode_navsol(){
	int itow,ftow,week;
	unsigned char *p=buff+6;
	string str;

	if (outtype) {
		msgtype="UBX NAV-SOL   ("+int2str(4," ",len,str)+"):";
	}
	itow=U4(p);
	ftow=I4(p+4);
	week=U2(p+8);
	if ((U1(p+11)&0x0C)==0x0C) {
		time.gpst2time(week,itow*1E-3+ftow*1E-9);
	}
	return 0;
}
/* decode ubx-nav-timegps: gps time solution ------------------------------------------------------ */
int ublox::decode_navtime(){
	int itow,ftow,week;
	unsigned char *p=buff+6;
	string str;

	if (outtype) {
		msgtype="UBX NAV-TIME  ("+int2str(4," ",len,str)+"):";
	}
	itow=U4(p);
	ftow=I4(p+4);
	week=U2(p+8);
	if ((U1(p+11)&0x03)==0x03) {
		time.gpst2time(week,itow*1E-3+ftow*1E-9);
	}
	return 0;
}
/* decode ubx-trk-meas: trace measurement data ---------------------------------------------------- */
int ublox::decode_trkmeas(){
	static double adrs[MAXSAT]={ 0 };
	gtime_t ttt;
	double ts,tr=-1.0,t,tau,utc_gpst,snr,adr,dop;
	int i,j,n=0,nch,sys,prn,sat,qi,frq,fflag,lock1,lock2,week;
	unsigned char *p=buff+6;
	string str;

	if (outtype) {
		msgtype="UBX TRK-MEAS  ("+int2str(4," ",len,str)+"):";
	}
	if (!time.time) return 0;

	/* number of channels */
	nch=U1(p+2);

	if (len<112+nch*56) {
		return -1;
	}
	/* time-tag = max(transmission time + 0.08) rounded by 100 ms */
	for (i=0,p=buff+110; i<nch; i++,p+=56) {
		if (U1(p+1)<4||ubx_sys(U1(p+4))!=SYS_GPS) continue;
		if ((t=I8(p+24)*P2_32/1000.0)>tr) tr=t;
	}
	if (tr<0.0) return 0;

	tr=ROUND((tr+0.08)/0.1)*0.1;

	/* adjust week handover */
	t=time.time2gpst(&week);
	if (tr<t-302400.0) week--;
	else if (tr>t+302400.0) week++;
	ttt.gpst2time(week,tr);

	utc_gpst=ttt.gpst2utc()->timediff(ttt);

	for (i=0,p=buff+110; i<nch; i++,p+=56) {

		/* quality indicator (0:idle,1:search,2:aquired,3:unusable, */
		/*                    4:code lock,5,6,7:code/carrier lock) */
		qi=U1(p+1);
		if (qi<4||7<qi) continue;

		/* system and satellite number */
		if (!(sys=ubx_sys(U1(p+4)))) {
			continue;
		}
		prn=U1(p+5)+(sys==SYS_QZS ? 192 : 0);
		if (!(sat=satno(sys,prn))) {
			continue;
		}
		/* transmission time */
		ts=I8(p+24)*P2_32/1000.0;
		if (sys==SYS_CMP) ts+=14.0;             /* bdt  -> gpst */
		else if (sys==SYS_GLO) ts-=10800.0+utc_gpst; /* glot -> gpst */

													 /* signal travel time */
		tau=tr-ts;
		if (tau<-302400.0) tau+=604800.0;
		else if (tau> 302400.0) tau-=604800.0;

		frq  =U1(p+ 7)-7; /* frequency */
		fflag =U1(p+ 8);   /* tracking status */
		lock1=U1(p+16);   /* code lock count */
		lock2=U1(p+17);   /* phase lock count */
		snr  =U2(p+20)/256.0;
		adr  =I8(p+32)*P2_32+(fflag&0x40 ? 0.5 : 0.0);
		dop  =I4(p+40)*P2_10*10.0;

		/* set slip flag */
		if (lock2==0||lock2<lockt[sat-1][0]) lockt[sat-1][1]=1.0;
		lockt[sat-1][0]=lock2;

		adrs[sat-1]=adr;

		/* check phase lock */
		if (!(fflag&0x20)) continue;

		obs.data[n].time=ttt;
		obs.data[n].sat=sat;
		obs.data[n].P[0]=tau*CLIGHT;
		obs.data[n].L[0]=-adr;
		obs.data[n].D[0]=(float)dop;
		obs.data[n].SNR[0]=(unsigned char)(snr*4.0);
		obs.data[n].code[0]=sys==SYS_CMP ? CODE_L1I : CODE_L1C;
		obs.data[n].LLI[0]=lockt[sat-1][1]>0.0 ? 1 : 0;
		if (sys==SYS_SBS) { /* half-cycle valid */
			obs.data[n].LLI[0]|=lock2>142 ? 0 : 2;
		}
		else {
			obs.data[n].LLI[0]|=fflag&0x80 ? 0 : 2;
		}
		lockt[sat-1][1]=0.0;

		for (j=1; j<NFREQ+NEXOBS; j++) {
			obs.data[n].L[j]=obs.data[n].P[j]=0.0;
			obs.data[n].D[j]=0.0;
			obs.data[n].SNR[j]=obs.data[n].LLI[j]=0;
			obs.data[n].code[j]=CODE_NONE;
		}
		n++;
	}
	if (n<=0) return 0;
	time=ttt;
	obs.n=n;
	return 1;
}
/* decode ubx-trkd5: trace measurement data ------------------------------------------------------- */
int ublox::decode_trkd5(){
	static double adrs[MAXSAT]={ 0 };
	gtime_t ttt;
	double ts,tr=-1.0,t,tau,adr,dop,snr,utc_gpst;
	int i,j,n=0,type,offside,llen,sys,prn,sat,qi,frq,fflag,week;
	unsigned char *p=buff+6;
	string str;

	if (outtype) {
		msgtype="UBX TRK-D5    ("+int2str(4," ",len,str)+"):";
	} 
	if (!time.time) return 0;

	utc_gpst=time.gpst2utc()->timediff(time);

	switch ((type=U1(p))) {
	case 3: offside=86; llen=56; break;
	case 6: offside=86; llen=64; break; /* u-blox 7 */
	default: offside=78; llen=56; break;
	}
	for (i=0,p=buff+offside; p-buff<len-2; i++,p+=llen) {
		if (U1(p+41)<4) continue;
		t=I8(p)*P2_32/1000.0;
		if (ubx_sys(U1(p+56))==SYS_GLO) t-=10800.0+utc_gpst;
		if (t>tr) tr=t;
	}
	if (tr<0.0) return 0;

	tr=ROUND((tr+0.08)/0.1)*0.1;

	/* adjust week handover */
	t=time.time2gpst(&week);
	if (tr<t-302400.0) week--;
	else if (tr>t+302400.0) week++;
	ttt.gpst2time(week,tr);

	for (i=0,p=buff+offside; p-buff<len-2; i++,p+=llen) {

		/* quality indicator */
		qi =U1(p+41)&7;
		if (qi<4||7<qi) continue;

		if (type==6) {
			if (!(sys=ubx_sys(U1(p+56)))) {
				continue;
			}
			prn=U1(p+57)+(sys==SYS_QZS ? 192 : 0);
			frq=U1(p+59)-7;
		}
		else {
			prn=U1(p+34);
			sys=prn<MINPRNSBS ? SYS_GPS : SYS_SBS;
		}
		if (!(sat=satno(sys,prn))) {
			continue;
		}
		/* transmission time */
		ts=I8(p)*P2_32/1000.0;
		if (sys==SYS_GLO) ts-=10800.0+utc_gpst; /* glot -> gpst */

												/* signal travel time */
		tau=tr-ts;
		if (tau<-302400.0) tau+=604800.0;
		else if (tau> 302400.0) tau-=604800.0;

		fflag=U1(p+54);   /* tracking status */
		adr=qi<6 ? 0.0 : I8(p+8)*P2_32+(fflag&0x01 ? 0.5 : 0.0);
		dop=I4(p+16)*P2_10/4.0;
		snr=U2(p+32)/256.0;

		if (snr<=10.0) lockt[sat-1][1]=1.0;

		adrs[sat-1]=adr;

		/* check phase lock */
		if (!(fflag&0x08)) continue;

		obs.data[n].time=ttt;
		obs.data[n].sat=sat;
		obs.data[n].P[0]=tau*CLIGHT;
		obs.data[n].L[0]=-adr;
		obs.data[n].D[0]=(float)dop;
		obs.data[n].SNR[0]=(unsigned char)(snr*4.0);
		obs.data[n].code[0]=sys==SYS_CMP ? CODE_L1I : CODE_L1C;
		obs.data[n].LLI[0]=lockt[sat-1][1]>0.0 ? 1 : 0;
		lockt[sat-1][1]=0.0;

		for (j=1; j<NFREQ+NEXOBS; j++) {
			obs.data[n].L[j]=obs.data[n].P[j]=0.0;
			obs.data[n].D[j]=0.0;
			obs.data[n].SNR[j]=obs.data[n].LLI[j]=0;
			obs.data[n].code[j]=CODE_NONE;
		}
		n++;
	}
	if (n<=0) return 0;
	time=ttt;
	obs.n=n;
	return 1;
}
/* decode ubx-trk-sfrbx: subframe buffer extension ------------------------------------------------ */
int ublox::decode_trksfrbx(){
	int prn,sat,sys;
	unsigned char *p=buff+6;
	string str;

	if (outtype) {
		msgtype="UBX TRK-SFRBX ("+int2str(4," ",len,str)+"): sys="+to_string(U1(p+1))+
			" prn="+int2str(3," ",U1(p+2),str);
	}
	if (!(sys=ubx_sys(U1(p+1)))) {
		return -1;
	}
	prn=U1(p+2)+(sys==SYS_QZS ? 192 : 0);
	if (!(sat=satno(sys,prn))) {
		return -1;
	}
	switch (sys) {
	case SYS_GPS: return decode_nav(sat,13);
	case SYS_QZS: return decode_nav(sat,13);
	case SYS_GAL: return decode_enav(sat,13);
	case SYS_CMP: return decode_cnav(sat,13);
	case SYS_GLO: return decode_gnav(sat,13,U1(p+4));
	case SYS_SBS: return decode_snav(sat,13);
	}
	return 0;
}
/* decode ublox raw message ----------------------------------------------------------------------- */
int ublox::decode_ubx()
{
	int type=(U1(buff+2)<<8)+U1(buff+3);
	char message[40];

	/* checksum */
	if (!checksum()) {
		return -1;
	}
	switch (type) {
		case ID_RXMRAW: return decode_rxmraw();
		case ID_RXMRAWX: return decode_rxmrawx();
		case ID_RXMSFRB: return decode_rxmsfrb();
		case ID_RXMSFRBX: return decode_rxmsfrbx();
		case ID_NAVSOL: return decode_navsol();
		case ID_NAVTIME: return decode_navtime();
		case ID_TRKMEAS: return decode_trkmeas();
		case ID_TRKD5: return decode_trkd5();
		case ID_TRKSFRBX: return decode_trksfrbx();
	}
	if (outtype) {
		sprintf(message, "UBX 0x%02X 0x%02X (%4d)",type>>8,type&0xF,
			len);
		msgtype=message;
	}
	return 0;
}
/* sync code -------------------------------------------------------------------------------------- */
int ublox::sync_ubx(unsigned char data)
{
	buff[0]=buff[1]; buff[1]=data;
	return buff[0]==UBXSYNC1&&buff[1]==UBXSYNC2;
}
/* input ublox raw message from stream ------------------------------------------------------------ */
int ublox::decode(unsigned char data){
	/* synchronize frame */
	if (nbyte==0) {
		if (!sync_ubx(data)) return 0;
		nbyte=2;
		return 0;
	}
	buff[nbyte++]=data;

	if (nbyte==6) {
		if ((len=U2(buff+4)+8)>MAXRAWLEN) {
			nbyte=0;
			return -1;
		}
	}
	if (nbyte<6||nbyte<len) return 0;
	nbyte=0;

	/* decode ublox raw message */
	return decode_ubx();
}