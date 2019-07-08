#include "Decode/raw.h"
#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
const char rcsid[]="$Id:$";

#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define P2_66       1.355252715606881E-20 /* 2^-66 for BeiDou ephemeris */

/* get two component bits ----------------------------------------------------*/
unsigned int getbitu2(const unsigned char *buff,int p1,int l1,int p2,
	int l2)
{
	return (getbitu(buff,p1,l1)<<l2)+getbitu(buff,p2,l2);
}
int getbits2(const unsigned char *buff,int p1,int l1,int p2,int l2)
{
	if (getbitu(buff,p1,1))
		return (int)((getbits(buff,p1,l1)<<l2)+getbitu(buff,p2,l2));
	else
		return (int)getbitu2(buff,p1,l1,p2,l2);
}
/* get three component bits --------------------------------------------------*/
unsigned int getbitu3(const unsigned char *buff,int p1,int l1,int p2,
	int l2,int p3,int l3)
{
	return (getbitu(buff,p1,l1)<<(l2+l3))+(getbitu(buff,p2,l2)<<l3)+
		getbitu(buff,p3,l3);
}
int getbits3(const unsigned char *buff,int p1,int l1,int p2,int l2,
	int p3,int l3)
{
	if (getbitu(buff,p1,1))
		return (int)((getbits(buff,p1,l1)<<(l2+l3))+
		(getbitu(buff,p2,l2)<<l3)+getbitu(buff,p3,l3));
	else
		return (int)getbitu3(buff,p1,l1,p2,l2,p3,l3);
}
/* merge two components ------------------------------------------------------*/
unsigned int merge_two_u(unsigned int a,unsigned int b,int n)
{
	return (a<<n)+b;
}
int merge_two_s(int a,unsigned int b,int n)
{
	return (int)((a<<n)+b);
}
/* get sign-magnitude bits ---------------------------------------------------*/
double getbitg(const unsigned char *buff,int pos,int len)
{
	double value=getbitu(buff,pos+1,len-1);
	return getbitu(buff,pos,1) ? -value : value;
}

/* Class of decode navigation data for ------------------------------------------------------------ */
/* decode gps/qzss navigation data frame -------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
decode_frame::decode_frame(){
	alm.assign(MAXSAT,alm_t());
	ion.assign(8,0.0);
	utc.assign(4,0.0);
	leaps=0.0;
}
decode_frame::~decode_frame(){
	alm.clear(); ion.clear();
	utc.clear();
}
/* Base functions --------------------------------------------------------------------------------- */
/* decode gps/qzss almanac ------------------------------------------------------------------------ */
void decode_frame::decode_almanac(const unsigned char *bbuff,int sat){
	gtime_t toa;
	double deltai,sqrtA,tt;
	int i=50,f0;

	if (alm.empty()||alm[sat-1].week==0) return;

	alm[sat-1].sat =sat;
	alm[sat-1].e   =getbits(bbuff,i,16)*P2_21;			i+=16;
	alm[sat-1].toas=getbitu(bbuff,i,8)*4096.0;			i+= 8;
	deltai         =getbits(bbuff,i,16)*P2_19*SC2RAD;	i+=16;
	alm[sat-1].OMGd=getbits(bbuff,i,16)*P2_38*SC2RAD;	i+=16;
	alm[sat-1].svh =getbitu(bbuff,i,8);					i+= 8;
	sqrtA          =getbitu(bbuff,i,24)*P2_11;			i+=24;
	alm[sat-1].OMG0=getbits(bbuff,i,24)*P2_23*SC2RAD;	i+=24;
	alm[sat-1].omg =getbits(bbuff,i,24)*P2_23*SC2RAD;	i+=24;
	alm[sat-1].M0  =getbits(bbuff,i,24)*P2_23*SC2RAD;	i+=24;
	f0             =getbitu(bbuff,i,8);					i+= 8;
	alm[sat-1].f1  =getbits(bbuff,i,11)*P2_38;			i+=11;
	alm[sat-1].f0  =getbits(bbuff,i,3)*P2_17+f0*P2_20;
	alm[sat-1].A   =sqrtA*sqrtA;
	alm[sat-1].i0  =0.3*SC2RAD+deltai;

	toa.gpst2time(alm[sat-1].week,alm[sat-1].toas);
	tt=toa.timediff(alm[sat-1].toa);
	if (tt<302400.0) alm[sat-1].week--;
	else if (tt>302400.0) alm[sat-1].week++;
	alm[sat-1].toa.gpst2time(alm[sat-1].week,alm[sat-1].toas);
}
/* decode gps navigation data subframe 4 ---------------------------------------------------------- */
void decode_frame::decode_gps_subfrm4(const unsigned char *bbuff){
	int i,sat,svid=getbitu(bbuff,50,6);

	if (25<=svid&&svid<=32) { /* page 2,3,4,5,7,8,9,10 */

							  /* decode almanac */
		sat=getbitu(bbuff,50,6);
		if (1<=sat&&sat<=32) decode_almanac(bbuff,sat);
	}
	else if (svid==63) { /* page 25 */

						 /* decode as and sv config */
		i=56;
		for (sat=1; sat<=32; sat++) {
			alm[sat-1].svconf=getbitu(bbuff,i,4); i+=4;
		}
		/* decode sv health */
		i=186;
		for (sat=25; sat<=32; sat++) {
			if (!alm.empty()) alm[sat-1].svh   =getbitu(bbuff,i,6); i+=6;
		}
	}
	else if (svid==56) { /* page 18 */

						 /* decode ion/utc parameters */
		if (!ion.empty()) {
			i=56;
			ion[0]=getbits(bbuff,i,8)*P2_30;     i+= 8;
			ion[1]=getbits(bbuff,i,8)*P2_27;     i+= 8;
			ion[2]=getbits(bbuff,i,8)*P2_24;     i+= 8;
			ion[3]=getbits(bbuff,i,8)*P2_24;     i+= 8;
			ion[4]=getbits(bbuff,i,8)*pow(2,11); i+= 8;
			ion[5]=getbits(bbuff,i,8)*pow(2,14); i+= 8;
			ion[6]=getbits(bbuff,i,8)*pow(2,16); i+= 8;
			ion[7]=getbits(bbuff,i,8)*pow(2,16);
		}
		if (!utc.empty()) {
			i=120;
			utc[1]=getbits(bbuff,i,24)*P2_50;     i+=24;
			utc[0]=getbits(bbuff,i,32)*P2_30;     i+=32;
			utc[2]=getbits(bbuff,i,8)*pow(2,12); i+= 8;
			utc[3]=getbitu(bbuff,i,8);
		}
		/* leaps */
		{
			i=192;
			leaps=getbits(bbuff,i,8);
		}
	}
}
/* decode qzss navigation data subframe 4/5 ----------------------------------------------- */
void decode_frame::decode_qzs_subfrm45(const unsigned char *bbuff){
	int i,j,sat,toas,week,svid=getbitu(bbuff,50,6);

	if (1<=svid&&svid<=5) { /* qzss almanac */

		if (!(sat=satno(SYS_QZS,192+svid))) return;
		decode_almanac(bbuff,sat);
	}
	else if (svid==51) { /* qzss health */

		if (!alm.empty()) {
			i=56;
			toas=getbitu(bbuff,i,8)*4096; i+=8;
			week=getbitu(bbuff,i,8);      i+=8;
			week=adjgpsweek(week);

			for (j=0; j<5; j++) {
				if (!(sat=satno(SYS_QZS,193+j))) continue;
				alm[sat-1].toas=toas;
				alm[sat-1].week=week;
				alm[sat-1].toa.gpst2time(week,toas);
				alm[sat-1].svh=getbitu(bbuff,i,6); i+=6;
			}
		}
	}
	else if (svid==56) { /* ion/utc parameters */

		if (!ion.empty()) {
			i=56;
			ion[0]=getbits(bbuff,i,8)*P2_30;     i+= 8;
			ion[1]=getbits(bbuff,i,8)*P2_27;     i+= 8;
			ion[2]=getbits(bbuff,i,8)*P2_24;     i+= 8;
			ion[3]=getbits(bbuff,i,8)*P2_24;     i+= 8;
			ion[4]=getbits(bbuff,i,8)*pow(2,11); i+= 8;
			ion[5]=getbits(bbuff,i,8)*pow(2,14); i+= 8;
			ion[6]=getbits(bbuff,i,8)*pow(2,16); i+= 8;
			ion[7]=getbits(bbuff,i,8)*pow(2,16);
		}
		if (!utc.empty()) {
			i=120;
			utc[1]=getbits(bbuff,i,24)*P2_50;     i+=24;
			utc[0]=getbits(bbuff,i,32)*P2_30;     i+=32;
			utc[2]=getbits(bbuff,i,8)*pow(2,12); i+= 8;
			utc[3]=getbitu(bbuff,i,8);
		}
	}
}
/* decode gps navigation data subframe 5 -------------------------------------------------- */
void decode_frame::decode_gps_subfrm5(const unsigned char *bbuff){
	double toas;
	int i,sat,week,svid=getbitu(bbuff,50,6);

	if (1<=svid&&svid<=24) { /* page 1-24 */

							 /* decode almanac */
		sat=getbitu(bbuff,50,6);
		if (1<=sat&&sat<=32) decode_almanac(bbuff,sat);
	}
	else if (svid==51) { /* page 25 */

		if (!alm.empty()) {
			i=56;
			toas=getbitu(bbuff,i,8)*4096; i+=8;
			week=getbitu(bbuff,i,8);      i+=8;
			week=adjgpsweek(week);

			/* decode sv health */
			for (sat=1; sat<=24; sat++) {
				alm[sat-1].svh=getbitu(bbuff,i,6); i+=6;
			}
			for (sat=1; sat<=32; sat++) {
				alm[sat-1].toas=toas;
				alm[sat-1].week=week;
				alm[sat-1].toa.gpst2time(week,toas);
			}
		}
	}
}
/* decode gps/qzss navigation data subframe 1 ----------------------------------------------------- */
int decode_frame::decode_subfrm1(const unsigned char *bbuff){
	double tow,toc;
	int i=48,week,iodc0,iodc1,tgd;

	tow			=getbitu(bbuff,24,17)*6.0;           /* transmission time */
	week		=getbitu(bbuff,i,10);		i+=10;
	eph.code	=getbitu(bbuff,i,2);			i+= 2;
	eph.sva		=getbitu(bbuff,i,4);			i+= 4;   /* ura index */
	eph.svh		=getbitu(bbuff,i,6);			i+= 6;
	iodc0		=getbitu(bbuff,i,2);			i+= 2;
	eph.flag	=getbitu(bbuff,i,1);			i+= 1+87;
	tgd			=getbits(bbuff,i,8);			i+= 8;
	iodc1		=getbitu(bbuff,i,8);			i+= 8;
	toc			=getbitu(bbuff,i,16)*16.0;	i+=16;
	eph.f2		=getbits(bbuff,i,8)*P2_55;	i+= 8;
	eph.f1		=getbits(bbuff,i,16)*P2_43;	i+=16;
	eph.f0		=getbits(bbuff,i,22)*P2_31;

	eph.tgd[0]=tgd==-128 ? 0.0 : tgd*P2_31; /* ref [4] */
	eph.iodc=(iodc0<<8)+iodc1;
	eph.week=adjgpsweek(week); /* week of tow */
	eph.ttr.gpst2time(eph.week,tow);
	eph.toc.gpst2time(eph.week,toc);

	return 1;
}
/* decode gps/qzss navigation data subframe 2 ----------------------------------------------------- */
int decode_frame::decode_subfrm2(const unsigned char *bbuff){
	double sqrtA;
	int i=48;

	eph.iode=getbitu(bbuff,i,8);					i+= 8;
	eph.crs =getbits(bbuff,i,16)*P2_5;			i+=16;
	eph.deln=getbits(bbuff,i,16)*P2_43*SC2RAD;	i+=16;
	eph.M0  =getbits(bbuff,i,32)*P2_31*SC2RAD;	i+=32;
	eph.cuc =getbits(bbuff,i,16)*P2_29;			i+=16;
	eph.e   =getbitu(bbuff,i,32)*P2_33;			i+=32;
	eph.cus =getbits(bbuff,i,16)*P2_29;			i+=16;
	sqrtA    =getbitu(bbuff,i,32)*P2_19;			i+=32;
	eph.toes=getbitu(bbuff,i,16)*16.0;			i+=16;
	eph.fit =getbitu(bbuff,i,1) ? 0.0 : 4.0; /* 0:4hr,1:>4hr */

	eph.A=sqrtA*sqrtA;

	return 2;
}
/* decode gps/qzss navigation data subframe 3 ----------------------------------------------------- */
int decode_frame::decode_subfrm3(const unsigned char *bbuff){
	double tow,toc;
	int i=48,iode;

	eph.cic =getbits(bbuff,i,16)*P2_29;        i+=16;
	eph.OMG0=getbits(bbuff,i,32)*P2_31*SC2RAD; i+=32;
	eph.cis =getbits(bbuff,i,16)*P2_29;        i+=16;
	eph.i0  =getbits(bbuff,i,32)*P2_31*SC2RAD; i+=32;
	eph.crc =getbits(bbuff,i,16)*P2_5;         i+=16;
	eph.omg =getbits(bbuff,i,32)*P2_31*SC2RAD; i+=32;
	eph.OMGd=getbits(bbuff,i,24)*P2_43*SC2RAD; i+=24;
	iode     =getbitu(bbuff,i,8);              i+= 8;
	eph.idot=getbits(bbuff,i,14)*P2_43*SC2RAD;

	/* check iode and iodc consistency */
	if (iode!=eph.iode||iode!=(eph.iodc&0xFF)) return 0;

	/* adjustment for week handover */
	tow=eph.ttr.time2gpst(&eph.week);
	toc=eph.toc.time2gpst(NULL);
	if (eph.toes<tow-302400.0) { eph.week++; tow-=604800.0; }
	else if (eph.toes>tow+302400.0) { eph.week--; tow+=604800.0; }
	eph.toe.gpst2time(eph.week,eph.toes);
	eph.toc.gpst2time(eph.week,toc);
	eph.ttr.gpst2time(eph.week,tow);

	return 3;
}
/* decode gps/qzss navigation data subframe 4 ----------------------------------------------------- */
int decode_frame::decode_subfrm4(const unsigned char *bbuff){
	int dataid=getbitu(bbuff,48,2);

	if (dataid==1) { /* gps */
		decode_gps_subfrm4(bbuff);
	}
	else if (dataid==3) { /* qzss */
		decode_qzs_subfrm45(bbuff);
	}
	return 4;
}
/* decode gps/qzss navigation data subframe 5 ----------------------------------------------------- */
int decode_frame::decode_subfrm5(const unsigned char *bbuff){
	int dataid=getbitu(bbuff,48,2);

	if (dataid==1) { /* gps */
		decode_gps_subfrm5(bbuff);
	}
	else if (dataid==3) { /* qzss */
		decode_qzs_subfrm45(bbuff);
	}
	return 5;
}
int decode_frame::decode(const unsigned char *bbuff){
	int id=getbitu(bbuff,43,3); /* subframe id */

	switch (id) {
		case 1: return this->decode_subfrm1(bbuff);
		case 2: return this->decode_subfrm2(bbuff);
		case 3: return this->decode_subfrm3(bbuff);
		case 4: return this->decode_subfrm4(bbuff);
		case 5: return this->decode_subfrm5(bbuff);
	}
	return 0;
}

/* receiver raw data control type --------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
/* Constructors ----------------------------------------------------------------------------------- */
raw_t::raw_t(){
	const double lam_glo[NFREQ]={ CLIGHT/FREQ1_GLO,CLIGHT/FREQ2_GLO };
	/* num */
	nbyte=len=iod=tbase=flag=outtype=format=icpc=0;
	tod=-1;
	for (int i=0; i<MAXRAWLEN; i++) buff[i]=0;
	opt="\0";
	/* class */
	tobs=gtime_t();
	obuf.data.assign(MAXOBS,obsd_t());
	nav.seph.assign(NSATSBS*2,seph_t());
	nav.alm.assign(MAXSAT,alm_t());
	nav.na=MAXSAT; nav.ns=NSATSBS*2;
	for (int i=0; i<MAXSAT; i++) for (int j=0; j<NFREQ; j++){
		int sys;
		if (!(sys=satsys(i+1,NULL))) continue;
		nav.lam[i][j]=sys==SYS_GLO ? lam_glo[j] : WaveLengths[j];
	}

	for (int i=0; i<MAXSAT; i++){
		icpp[i]=off[i]=prCA[i]=dpCA[i]=0.0;
		for (int j=0; j<NFREQ+NEXOBS; j++) lockt[i][j]=0.0;
	}
}
raw_t::~raw_t(){
	half_cyc.clear();
}
/* implementation functions ----------------------------------------------------------------------- */
/* common decode functions */
/* decode sbas message ---------------------------------------------------- */
int raw_t::sbsdecodemsg(int prn, unsigned int words[10]){
	int i,j;
	unsigned char f[29];
	double tow;

	if (time.time==0) return 0;
	tow=time.time2gpst(&sbsmsg.week);
	sbsmsg.tow=(int)(tow+DTTOL);
	sbsmsg.prn=prn;
	for (i=0; i<7; i++) for (j=0; j<4; j++) {
		sbsmsg.msg[i*4+j]=(unsigned char)(words[i]>>((3-j)*8));
	}
	sbsmsg.msg[28]=(unsigned char)(words[7]>>18)&0xC0;
	for (i=28; i>0; i--) f[i]=(sbsmsg.msg[i]>>6)+(sbsmsg.msg[i-1]<<2);
	f[0]=sbsmsg.msg[0]>>6;

	return rtk_crc24q(f,29)==(words[7]&0xFFFFFF); /* check crc */
}
/* decode Galileo I/NAV ephemeris --------------------------------------------------------------------
* decode Galileo I/NAV (ref [5] 4.3)
* args   : int sat I    *p = subfrm[sat-1]   Galileo I/NAV subframe bits
*						p[ 0-15]: I/NAV word type 0 (128 bit)
*						p[16-31]: I/NAV word type 1
*						p[32-47]: I/NAV word type 2
*						p[48-63]: I/NAV word type 3
*						p[64-79]: I/NAV word type 4
*						p[80-95]: I/NAV word type 5
*          eph_t    *eph    IO  ephemeris structure
* return : status (1:ok,0:error)
*-------------------------------------------------------------------------------------------------- */
int raw_t::decode_gal_inav(const int sat,eph_t &eph){
	double tow,toc,tt,sqrtA;
	int i,time_f,week,svid,e5b_hs,e1b_hs,e5b_dvs,e1b_dvs,type[6],iod_nav[4];
	unsigned char *p=subfrm[sat-1];
	gtime_t ttt;

	i=0; /* word type 0 */
	type[0]    =getbitu(p,i,6);              i+= 6;
	time_f     =getbitu(p,i,2);              i+= 2+88;
	week       =getbitu(p,i,12);              i+=12;
	tow        =getbitu(p,i,20);

	i=128; /* word type 1 */
	type[1]    =getbitu(p,i,6);              i+= 6;
	iod_nav[0] =getbitu(p,i,10);              i+=10;
	eph.toes  =getbitu(p,i,14)*60.0;         i+=14;
	eph.M0    =getbits(p,i,32)*P2_31*SC2RAD; i+=32;
	eph.e     =getbitu(p,i,32)*P2_33;        i+=32;
	sqrtA      =getbitu(p,i,32)*P2_19;

	i=128*2; /* word type 2 */
	type[2]    =getbitu(p,i,6);              i+= 6;
	iod_nav[1] =getbitu(p,i,10);              i+=10;
	eph.OMG0  =getbits(p,i,32)*P2_31*SC2RAD; i+=32;
	eph.i0    =getbits(p,i,32)*P2_31*SC2RAD; i+=32;
	eph.omg   =getbits(p,i,32)*P2_31*SC2RAD; i+=32;
	eph.idot  =getbits(p,i,14)*P2_43*SC2RAD;

	i=128*3; /* word type 3 */
	type[3]    =getbitu(p,i,6);              i+= 6;
	iod_nav[2] =getbitu(p,i,10);              i+=10;
	eph.OMGd  =getbits(p,i,24)*P2_43*SC2RAD; i+=24;
	eph.deln  =getbits(p,i,16)*P2_43*SC2RAD; i+=16;
	eph.cuc   =getbits(p,i,16)*P2_29;        i+=16;
	eph.cus   =getbits(p,i,16)*P2_29;        i+=16;
	eph.crc   =getbits(p,i,16)*P2_5;         i+=16;
	eph.crs   =getbits(p,i,16)*P2_5;         i+=16;
	eph.sva   =getbitu(p,i,8);

	i=128*4; /* word type 4 */
	type[4]    =getbitu(p,i,6);              i+= 6;
	iod_nav[3] =getbitu(p,i,10);              i+=10;
	svid       =getbitu(p,i,6);              i+= 6;
	eph.cic   =getbits(p,i,16)*P2_29;        i+=16;
	eph.cis   =getbits(p,i,16)*P2_29;        i+=16;
	toc        =getbitu(p,i,14)*60.0;         i+=14;
	eph.f0    =getbits(p,i,31)*P2_34;        i+=31;
	eph.f1    =getbits(p,i,21)*P2_46;        i+=21;
	eph.f2    =getbits(p,i,6)*P2_59;

	i=128*5; /* word type 5 */
	type[5]    =getbitu(p,i,6);              i+= 6+41;
	eph.tgd[0]=getbits(p,i,10)*P2_32;        i+=10; /* BGD E5a/E1 */
	eph.tgd[1]=getbits(p,i,10)*P2_32;        i+=10; /* BGD E5b/E1 */
	e5b_hs     =getbitu(p,i,2);              i+= 2;
	e1b_hs     =getbitu(p,i,2);              i+= 2;
	e5b_dvs    =getbitu(p,i,1);              i+= 1;
	e1b_dvs    =getbitu(p,i,1);

	/* test word types */
	if (type[0]!=0||type[1]!=1||type[2]!=2||type[3]!=3||type[4]!=4) {
		return 0;
	}
	/* test word type 0 time field */
	if (time_f!=2) {
		return 0;
	}
	/* test consistency of iod_nav */
	if (iod_nav[0]!=iod_nav[1]||iod_nav[0]!=iod_nav[2]||iod_nav[0]!=iod_nav[3]) {
		return 0;
	}
	if (!(eph.sat=satno(SYS_GAL,svid))) {
		return 0;
	}
	eph.A=sqrtA*sqrtA;
	eph.iode=eph.iodc=iod_nav[0];
	eph.svh=(e5b_hs<<7)|(e5b_dvs<<6)|(e1b_hs<<1)|e1b_dvs;
	eph.ttr.gst2time(week,tow);
	tt=ttt.gst2time(week,eph.toes)->timediff(eph.ttr); /* week complient to toe */
	if (tt> 302400.0) week--;
	else if (tt<-302400.0) week++;
	eph.toe.gst2time(week,eph.toes);
	eph.toc.gst2time(week,toc);
	eph.week=week+1024; /* gal-week = gst-week + 1024 */
	eph.code=1;         /* data source = I/NAV E1B */

	return 1;
}
/* decode BeiDou D1 ephemeris ------------------------------------------------------------------------
* decode BeiDou D1 ephemeris (IGSO/MEO satellites) (ref [3] 5.2)
* args   : int sat I    *p = subfrm[sat-1]   beidou D1 subframe bits
*							p[ 0- 37]: subframe 1 (300 bits)
*							p[38- 75]: subframe 2
*							p[76-113]: subframe 3
*          eph_t    &eph    IO  ephemeris structure
* return : status (1:ok,0:error)
*-------------------------------------------------------------------------------------------------- */
int raw_t::decode_bds_d1(const int sat,eph_t &eph){
	double toc_bds,sqrtA;
	unsigned int toe1,toe2,sow1,sow2,sow3;
	int i,frn1,frn2,frn3;
	unsigned char *p=subfrm[sat-1];

	i=8*38*0; /* subframe 1 */
	frn1       =getbitu(p,i+ 15,3);
	sow1       =getbitu2(p,i+ 18,8,i+30,12);
	eph.svh   =getbitu(p,i+ 42,1); /* SatH1 */
	eph.iodc  =getbitu(p,i+ 43,5); /* AODC */
	eph.sva   =getbitu(p,i+ 48,4);
	eph.week  =getbitu(p,i+ 60,13); /* week in BDT */
	toc_bds    =getbitu2(p,i+ 73,9,i+ 90,8)*8.0;
	eph.tgd[0]=getbits(p,i+ 98,10)*0.1*1E-9;
	eph.tgd[1]=getbits2(p,i+108,4,i+120,6)*0.1*1E-9;
	eph.f2    =getbits(p,i+214,11)*P2_66;
	eph.f0    =getbits2(p,i+225,7,i+240,17)*P2_33;
	eph.f1    =getbits2(p,i+257,5,i+270,17)*P2_50;
	eph.iode  =getbitu(p,i+287,5); /* AODE */

	i=8*38*1; /* subframe 2 */
	frn2       =getbitu(p,i+ 15,3);
	sow2       =getbitu2(p,i+ 18,8,i+30,12);
	eph.deln  =getbits2(p,i+ 42,10,i+ 60,6)*P2_43*SC2RAD;
	eph.cuc   =getbits2(p,i+ 66,16,i+ 90,2)*P2_31;
	eph.M0    =getbits2(p,i+ 92,20,i+120,12)*P2_31*SC2RAD;
	eph.e     =getbitu2(p,i+132,10,i+150,22)*P2_33;
	eph.cus   =getbits(p,i+180,18)*P2_31;
	eph.crc   =getbits2(p,i+198,4,i+210,14)*P2_6;
	eph.crs   =getbits2(p,i+224,8,i+240,10)*P2_6;
	sqrtA      =getbitu2(p,i+250,12,i+270,20)*P2_19;
	toe1       =getbitu(p,i+290,2); /* TOE 2-MSB */
	eph.A     =sqrtA*sqrtA;

	i=8*38*2; /* subframe 3 */
	frn3       =getbitu(p,i+ 15,3);
	sow3       =getbitu2(p,i+ 18,8,i+30,12);
	toe2       =getbitu2(p,i+ 42,10,i+ 60,5); /* TOE 5-LSB */
	eph.i0    =getbits2(p,i+ 65,17,i+ 90,15)*P2_31*SC2RAD;
	eph.cic   =getbits2(p,i+105,7,i+120,11)*P2_31;
	eph.OMGd  =getbits2(p,i+131,11,i+150,13)*P2_43*SC2RAD;
	eph.cis   =getbits2(p,i+163,9,i+180,9)*P2_31;
	eph.idot  =getbits2(p,i+189,13,i+210,1)*P2_43*SC2RAD;
	eph.OMG0  =getbits2(p,i+211,21,i+240,11)*P2_31*SC2RAD;
	eph.omg   =getbits2(p,i+251,11,i+270,21)*P2_31*SC2RAD;
	eph.toes  =merge_two_u(toe1,toe2,15)*8.0;

	/* check consistency of subframe numbers, sows and toe/toc */
	if (frn1!=1||frn2!=2||frn3!=3) {
		return 0;
	}
	if (sow2!=sow1+6||sow3!=sow2+6) {
		return 0;
	}
	if (toc_bds!=eph.toes) {
		return 0;
	}
	eph.ttr.bdt2time(eph.week,sow1)->bdt2gpst();      /* bdt -> gpst */
	if (eph.toes>sow1+302400.0) eph.week++;
	else if (eph.toes<sow1-302400.0) eph.week--;
	eph.toe.bdt2time(eph.week,eph.toes)->bdt2gpst(); /* bdt -> gpst */
	eph.toc.bdt2time(eph.week,toc_bds)->bdt2gpst();   /* bdt -> gpst */
	return 1;
}
/* decode BeiDou D2 ephemeris ------------------------------------------------------------------------ 
* decode BeiDou D2 ephemeris (GEO satellites) (ref [3] 5.3)
* args   : int sat I    *p = subfrm[sat-1]   beidou D1 subframe bits
*							p[  0- 37]: page 1 (300 bits)
*							p[ 38- 75]: page 2
*							...
*							p[342-379]: page 10
*          eph_t    &eph    IO  ephemeris structure
* return : status (1:ok,0:error)
*-------------------------------------------------------------------------------------------------- */
int raw_t::decode_bds_d2(const int sat,eph_t &eph){
	double toc_bds,sqrtA;
	unsigned int f1p4,cucp5,ep6,cicp7,i0p8,OMGdp9,omgp10;
	unsigned int sow1,sow3,sow4,sow5,sow6,sow7,sow8,sow9,sow10;
	int i,f1p3,cucp4,ep5,cicp6,i0p7,OMGdp8,omgp9;
	int pgn1,pgn3,pgn4,pgn5,pgn6,pgn7,pgn8,pgn9,pgn10;
	unsigned char *p = subfrm[sat-1];

	i=8*38*0; /* page 1 */
	pgn1       =getbitu(p,i+ 42,4);
	sow1       =getbitu2(p,i+ 18,8,i+ 30,12);
	eph.svh   =getbitu(p,i+ 46,1); /* SatH1 */
	eph.iodc  =getbitu(p,i+ 47,5); /* AODC */
	eph.sva   =getbitu(p,i+ 60,4);
	eph.week  =getbitu(p,i+ 64,13); /* week in BDT */
	toc_bds    =getbitu2(p,i+ 77,5,i+ 90,12)*8.0;
	eph.tgd[0]=getbits(p,i+102,10)*0.1*1E-9;
	eph.tgd[1]=getbits(p,i+120,10)*0.1*1E-9;

	i=8*38*2; /* page 3 */
	pgn3       =getbitu(p,i+ 42,4);
	sow3       =getbitu2(p,i+ 18,8,i+ 30,12);
	eph.f0    =getbits2(p,i+100,12,i+120,12)*P2_33;
	f1p3       =getbits(p,i+132,4);

	i=8*38*3; /* page 4 */
	pgn4       =getbitu(p,i+ 42,4);
	sow4       =getbitu2(p,i+ 18,8,i+ 30,12);
	f1p4       =getbitu2(p,i+ 46,6,i+ 60,12);
	eph.f2    =getbits2(p,i+ 72,10,i+ 90,1)*P2_66;
	eph.iode  =getbitu(p,i+ 91,5); /* AODE */
	eph.deln  =getbits(p,i+ 96,16)*P2_43*SC2RAD;
	cucp4      =getbits(p,i+120,14);

	i=8*38*4; /* page 5 */
	pgn5       =getbitu(p,i+ 42,4);
	sow5       =getbitu2(p,i+ 18,8,i+ 30,12);
	cucp5      =getbitu(p,i+ 46,4);
	eph.M0    =getbits3(p,i+ 50,2,i+ 60,22,i+ 90,8)*P2_31*SC2RAD;
	eph.cus   =getbits2(p,i+ 98,14,i+120,4)*P2_31;
	ep5        =getbits(p,i+124,10);

	i=8*38*5; /* page 6 */
	pgn6       =getbitu(p,i+ 42,4);
	sow6       =getbitu2(p,i+ 18,8,i+ 30,12);
	ep6        =getbitu2(p,i+ 46,6,i+ 60,16);
	sqrtA      =getbitu3(p,i+ 76,6,i+ 90,22,i+120,4)*P2_19;
	cicp6      =getbits(p,i+124,10);
	eph.A     =sqrtA*sqrtA;

	i=8*38*6; /* page 7 */
	pgn7       =getbitu(p,i+ 42,4);
	sow7       =getbitu2(p,i+ 18,8,i+ 30,12);
	cicp7      =getbitu2(p,i+ 46,6,i+ 60,2);
	eph.cis   =getbits(p,i+ 62,18)*P2_31;
	eph.toes  =getbitu2(p,i+ 80,2,i+ 90,15)*8.0;
	i0p7       =getbits2(p,i+105,7,i+120,14);

	i=8*38*7; /* page 8 */
	pgn8       =getbitu(p,i+ 42,4);
	sow8       =getbitu2(p,i+ 18,8,i+ 30,12);
	i0p8       =getbitu2(p,i+ 46,6,i+ 60,5);
	eph.crc   =getbits2(p,i+ 65,17,i+ 90,1)*P2_6;
	eph.crs   =getbits(p,i+ 91,18)*P2_6;
	OMGdp8     =getbits2(p,i+109,3,i+120,16);

	i=8*38*8; /* page 9 */
	pgn9       =getbitu(p,i+ 42,4);
	sow9       =getbitu2(p,i+ 18,8,i+ 30,12);
	OMGdp9     =getbitu(p,i+ 46,5);
	eph.OMG0  =getbits3(p,i+ 51,1,i+ 60,22,i+ 90,9)*P2_31*SC2RAD;
	omgp9      =getbits2(p,i+ 99,13,i+120,14);

	i=8*38*9; /* page 10 */
	pgn10      =getbitu(p,i+ 42,4);
	sow10      =getbitu2(p,i+ 18,8,i+ 30,12);
	omgp10     =getbitu(p,i+ 46,5);
	eph.idot  =getbits2(p,i+ 51,1,i+ 60,13)*P2_43*SC2RAD;

	/* check consistency of page numbers, sows and toe/toc */
	if (pgn1!=1||pgn3!=3||pgn4!=4||pgn5!=5||pgn6!=6||pgn7!=7||pgn8!=8||pgn9!=9||
		pgn10!=10) {
		return 0;
	}
	if (sow3!=sow1+6||sow4!=sow3+3||sow5!=sow4+3||sow6!=sow5+3||
		sow7!=sow6+3||sow8!=sow7+3||sow9!=sow8+3||sow10!=sow9+3) {
		return 0;
	}
	if (toc_bds!=eph.toes) {
		return 0;
	}
	eph.f1  =merge_two_s(f1p3,f1p4,18)*P2_50;
	eph.cuc =merge_two_s(cucp4,cucp5,4)*P2_31;
	eph.e   =merge_two_s(ep5,ep6,22)*P2_33;
	eph.cic =merge_two_s(cicp6,cicp7,8)*P2_31;
	eph.i0  =merge_two_s(i0p7,i0p8,11)*P2_31*SC2RAD;
	eph.OMGd=merge_two_s(OMGdp8,OMGdp9,5)*P2_43*SC2RAD;
	eph.omg =merge_two_s(omgp9,omgp10,5)*P2_31*SC2RAD;

	eph.ttr.bdt2time(eph.week,sow1)->bdt2gpst();      /* bdt -> gpst */
	if (eph.toes>sow1+302400.0) eph.week++;
	else if (eph.toes<sow1-302400.0) eph.week--;
	eph.toe.bdt2time(eph.week,eph.toes)->bdt2gpst(); /* bdt -> gpst */
	eph.toc.bdt2time(eph.week,toc_bds)->bdt2gpst();   /* bdt -> gpst */
	return 1;
}
/* test hamming code of glonass ephemeris string -----------------------------------------------------
* test hamming code of glonass ephemeris string (ref [2] 4.7)
* args   : unsigned char *buff I glonass navigation data string bits in frame
*                                with hamming
*                                  buff[ 0]: string bit 85-78
*                                  buff[ 1]: string bit 77-70
*                                  ...
*                                  buff[10]: string bit  5- 1 (0 padded)
* return : status (1:ok,0:error)
*-------------------------------------------------------------------------------------------------- */
int raw_t::test_glostr(const unsigned char *bbuff){
	const unsigned char xor_8bit[256]={ /* xor of 8 bits */
		0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
		1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
		1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
		0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
		1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
		0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
		0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
		1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0
	};
	const unsigned char mask_hamming[][12]={ /* mask of hamming codes */
		{ 0x55,0x55,0x5A,0xAA,0xAA,0xAA,0xB5,0x55,0x6A,0xD8,0x08 },
		{ 0x66,0x66,0x6C,0xCC,0xCC,0xCC,0xD9,0x99,0xB3,0x68,0x10 },
		{ 0x87,0x87,0x8F,0x0F,0x0F,0x0F,0x1E,0x1E,0x3C,0x70,0x20 },
		{ 0x07,0xF8,0x0F,0xF0,0x0F,0xF0,0x1F,0xE0,0x3F,0x80,0x40 },
		{ 0xF8,0x00,0x0F,0xFF,0xF0,0x00,0x1F,0xFF,0xC0,0x00,0x80 },
		{ 0x00,0x00,0x0F,0xFF,0xFF,0xFF,0xE0,0x00,0x00,0x01,0x00 },
		{ 0xFF,0xFF,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00 },
		{ 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF8 }
	};
	unsigned char cs;
	int i,j,n=0;

	for (i=0; i<8; i++) {
		for (j=0,cs=0; j<11; j++) {
			cs^=xor_8bit[bbuff[j]&mask_hamming[i][j]];
		}
		if (cs) n++;
	}
	return n==0||(n==2&&cs);
}
/* decode glonass ephemeris strings ------------------------------------------------------------------
* decode glonass ephemeris string (ref [2])
* args   : int sat I    *p = subfrm[sat-1]   glonass navigation data string bits in frames
*											(without hamming and time mark)
*									p[ 0- 9]: string #1 (77 bits)
*									p[10-19]: string #2
*									p[20-29]: string #3
*									p[30-39]: string #4
*          geph_t &geph  IO     glonass ephemeris message
* return : status (1:ok,0:error)
* notes  : geph.tof should be set to frame time witin 1/2 day before calling
*          geph.frq is set to 0
*-------------------------------------------------------------------------------------------------- */
int raw_t::decode_glostr(const int sat,geph_t &geph){
	double tow,ttod,tof,toe;
	int P,P1,P2,P3,P4,tk_h,tk_m,tk_s,tb,ln,NT,slot,M,week;
	int i=1,frn1,frn2,frn3,frn4;
	unsigned char *p = subfrm[sat-1];

	/* frame 1 */
	frn1        =getbitu(p,i,4);           i+= 4+2;
	P1          =getbitu(p,i,2);           i+= 2;
	tk_h        =getbitu(p,i,5);           i+= 5;
	tk_m        =getbitu(p,i,6);           i+= 6;
	tk_s        =getbitu(p,i,1)*30;        i+= 1;
	geph.vel[0]=getbitg(p,i,24)*P2_20*1E3; i+=24;
	geph.acc[0]=getbitg(p,i,5)*P2_30*1E3; i+= 5;
	geph.pos[0]=getbitg(p,i,27)*P2_11*1E3; i+=27+4;

	/* frame 2 */
	frn2        =getbitu(p,i,4);           i+= 4;
	geph.svh   =getbitu(p,i,3);           i+= 3;
	P2          =getbitu(p,i,1);           i+= 1;
	tb          =getbitu(p,i,7);           i+= 7+5;
	geph.vel[1]=getbitg(p,i,24)*P2_20*1E3; i+=24;
	geph.acc[1]=getbitg(p,i,5)*P2_30*1E3; i+= 5;
	geph.pos[1]=getbitg(p,i,27)*P2_11*1E3; i+=27+4;

	/* frame 3 */
	frn3        =getbitu(p,i,4);           i+= 4;
	P3          =getbitu(p,i,1);           i+= 1;
	geph.gamn  =getbitg(p,i,11)*P2_40;     i+=11+1;
	P           =getbitu(p,i,2);           i+= 2;
	ln          =getbitu(p,i,1);           i+= 1;
	geph.vel[2]=getbitg(p,i,24)*P2_20*1E3; i+=24;
	geph.acc[2]=getbitg(p,i,5)*P2_30*1E3; i+= 5;
	geph.pos[2]=getbitg(p,i,27)*P2_11*1E3; i+=27+4;

	/* frame 4 */
	frn4        =getbitu(p,i,4);           i+= 4;
	geph.taun  =getbitg(p,i,22)*P2_30;     i+=22;
	geph.dtaun =getbitg(p,i,5)*P2_30;     i+= 5;
	geph.age   =getbitu(p,i,5);           i+= 5+14;
	P4          =getbitu(p,i,1);           i+= 1;
	geph.sva   =getbitu(p,i,4);           i+= 4+3;
	NT          =getbitu(p,i,11);           i+=11;
	slot        =getbitu(p,i,5);           i+= 5;
	M           =getbitu(p,i,2);

	if (frn1!=1||frn2!=2||frn3!=3||frn4!=4) {
		return 0;
	}
	if (!(geph.sat=satno(SYS_GLO,slot))) {
		return 0;
	}
	geph.frq=0;
	geph.iode=tb;
	tow=geph.tof.gpst2utc()->time2gpst(&week);
	ttod=fmod(tow,86400.0); tow-=ttod;
	tof=tk_h*3600.0+tk_m*60.0+tk_s-10800.0; /* lt->utc */
	if (tof<ttod-43200.0) tof+=86400.0;
	else if (tof>ttod+43200.0) tof-=86400.0;
	geph.tof.gpst2time(week,tow+tof)->utc2gpst();
	toe=tb*900.0-10800.0; /* lt->utc */
	if (toe<ttod-43200.0) toe+=86400.0;
	else if (toe>ttod+43200.0) toe-=86400.0;
	geph.toe.gpst2time(week,tow+toe)->utc2gpst(); /* utc->gpst */
	return 1;
}


/* input receiver raw data from stream ------------------------------------------------------------ */
int raw_t::decode(unsigned char data){
	return 0;
}