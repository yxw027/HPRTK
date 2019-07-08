#include "binex.h"

#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
const char rcsid[]="$Id:$";

#define BNXSYNC1    0xC2    /* binex sync (little-endian,regular-crc) */
#define BNXSYNC2    0xE2    /* binex sync (big-endian   ,regular-crc) */
#define BNXSYNC3    0xC8    /* binex sync (little-endian,enhanced-crc) */
#define BNXSYNC4    0xE8    /* binex sync (big-endian   ,enhanced-crc) */

#define BNXSYNC1R   0xD2    /* binex sync (little-endian,regular-crc,rev) */
#define BNXSYNC2R   0xF2    /* binex sync (big-endian   ,regular-crc,rev) */
#define BNXSYNC3R   0xD8    /* binex sync (little-endian,enhanced-crc,rev) */
#define BNXSYNC4R   0xF8    /* binex sync (big-endian   ,enhanced-crc,rev) */

#define MIN(x,y)    ((x)<(y)?(x):(y))

/* ura table -------------------------------------------------------------------------------------- */
const double ura_eph[]={
	2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
	3072.0,6144.0,0.0
};
/* get fields (big-endian) ------------------------------------------------------------------------ */
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))

unsigned short U2(unsigned char *p)
{
	unsigned short value;
	unsigned char *q=(unsigned char *)&value+1;
	int i;
	for (i=0; i<2; i++) *q--=*p++;
	return value;
}
unsigned int U4(unsigned char *p)
{
	unsigned int value;
	unsigned char *q=(unsigned char *)&value+3;
	int i;
	for (i=0; i<4; i++) *q--=*p++;
	return value;
}
int I4(unsigned char *p)
{
	return (int)U4(p);
}
float R4(unsigned char *p)
{
	float value;
	unsigned char *q=(unsigned char *)&value+3;
	int i;
	for (i=0; i<4; i++) *q--=*p++;
	return value;
}
double R8(unsigned char *p)
{
	double value;
	unsigned char *q=(unsigned char *)&value+7;
	int i;
	for (i=0; i<8; i++) *q--=*p++;
	return value;
}

/* input binex message from stream -------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
binex::binex(){
}
binex::~binex(){
}

/* get binex 1-4 byte unsigned integer (big endian) ----------------------------------------------- */
int binex::getbnxi(unsigned char *p,unsigned int &val){
	int i;

	for (val=0,i=0; i<3; i++) {
		val=(val<<7)+(p[i]&0x7F);
		if (!(p[i]&0x80)) return i+1;
	}
	val=(val<<8)+p[i];
	return 4;
}
/* checksum 8 parity (buff + 1)-------------------------------------------------------------------- */
unsigned char binex::csum8(int len){
	unsigned char cs=0;
	int i;

	for (i=0; i<len; i++) {
		cs^=buff[i+1];
	}
	return cs;
}
/* adjust weekly rollover of gps time ------------------------------------------------------------- */
gtime_t binex::adjweek(gtime_t ttt,double tow){
	double tow_p;
	int week;
	gtime_t adjw;

	tow_p=ttt.time2gpst(&week);
	if (tow<tow_p-302400.0) tow+=604800.0;
	else if (tow>tow_p+302400.0) tow-=604800.0;
	return *adjw.gpst2time(week,tow);
}
/* adjust daily rollover of time ------------------------------------------------------------------ */
gtime_t binex::adjday(double ttod){
	double tod_p;
	gtime_t adjd;

	time.time2epoch();
	tod_p=time.ep[3]*3600.0+time.ep[4]*60.0+time.ep[5];
	if (ttod<tod_p-43200.0) ttod+=86400.0;
	else if (ttod>tod_p+43200.0) ttod-=86400.0;

	adjd.ep[0]=time.ep[0]; adjd.ep[1]=time.ep[1]; adjd.ep[2]=time.ep[2];
	adjd.ep[3]=adjd.ep[4]=adjd.ep[5]=0.0;
	return *adjd.epoch2time(adjd.ep)->timeadd(ttod);
}
/* ura value (m) to ura index --------------------------------------------------------------------- */
int binex::uraindex(double value){
	int i;
	for (i=0; i<15; i++) if (ura_eph[i]>=value) break;
	return i;
}
/* beidou signed 10 bit tgd -> sec ---------------------------------------------------------------- */
double binex::bds_tgd(int tgd){
	tgd&=0x3FF;
	return (tgd&0x200) ? -1E-10*((~tgd)&0x1FF) : 1E-10*(tgd&0x1FF);
}
/* synchronize binex message ---------------------------------------------------------------------- */
int binex::sync_bnx(unsigned char data){
	buff[0]=buff[1]; buff[1]=data;

	return buff[0]==BNXSYNC2&&
		(buff[1]==0x00||buff[1]==0x01||buff[1]==0x02||buff[1]==0x03||
			buff[1]==0x7D||buff[1]==0x7E||buff[1]==0x7F);
}

/* decode binex mesaage 0x00-00: comment ---------------------------------------------------------- */
int binex::decode_bnx_00_00(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-01: program or software package -------------------------------------- */
int binex::decode_bnx_00_01(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-02: program operator ------------------------------------------------- */
int binex::decode_bnx_00_02(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-03: reserved --------------------------------------------------------- */
int binex::decode_bnx_00_03(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-04: site name/description -------------------------------------------- */
int binex::decode_bnx_00_04(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-05: site number ------------------------------------------------------ */
int binex::decode_bnx_00_05(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-06: monumnent name --------------------------------------------------- */
int binex::decode_bnx_00_06(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-07: monumnent number ------------------------------------------------- */
int binex::decode_bnx_00_07(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-08: marker name ------------------------------------------------------ */
int binex::decode_bnx_00_08(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-09: marker number ---------------------------------------------------- */
int binex::decode_bnx_00_09(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-0a: reference point name --------------------------------------------- */
int binex::decode_bnx_00_0a(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-0b: reference point number ------------------------------------------- */
int binex::decode_bnx_00_0b(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-0c: date esttablished ------------------------------------------------ */
int binex::decode_bnx_00_0c(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-0d: reserved --------------------------------------------------------- */
int binex::decode_bnx_00_0d(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-0e: reserved --------------------------------------------------------- */
int binex::decode_bnx_00_0e(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-0f: 4-character id --------------------------------------------------- */
int binex::decode_bnx_00_0f(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-10: project name ----------------------------------------------------- */
int binex::decode_bnx_00_10(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-11: principal investigator for this project -------------------------- */
int binex::decode_bnx_00_11(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-12: pi's agency/institution ------------------------------------------ */
int binex::decode_bnx_00_12(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-13: pi's contact information ----------------------------------------- */
int binex::decode_bnx_00_13(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-14: site operator ---------------------------------------------------- */
int binex::decode_bnx_00_14(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-15: site operator's agency/institution ------------------------------- */
int binex::decode_bnx_00_15(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-16: site operator's contact information ------------------------------ */
int binex::decode_bnx_00_16(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-17: antenna type ----------------------------------------------------- */
int binex::decode_bnx_00_17(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-18: antenna number --------------------------------------------------- */
int binex::decode_bnx_00_18(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-19: receiver type ---------------------------------------------------- */
int binex::decode_bnx_00_19(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-1a: receiver number -------------------------------------------------- */
int binex::decode_bnx_00_1a(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-1b: receiver firmware version ---------------------------------------- */
int binex::decode_bnx_00_1b(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-1c: antenna mount description ---------------------------------------- */
int binex::decode_bnx_00_1c(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-1d: antenna xyz position --------------------------------------------- */
int binex::decode_bnx_00_1d(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-1e: antenna geographic position -------------------------------------- */
int binex::decode_bnx_00_1e(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-1f: antenna offset from reference point ------------------------------ */
int binex::decode_bnx_00_1f(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-20: antenna radome type ---------------------------------------------- */
int binex::decode_bnx_00_20(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-21: antenna radome number -------------------------------------------- */
int binex::decode_bnx_00_21(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-22: geocode ---------------------------------------------------------- */
int binex::decode_bnx_00_22(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x00-7f: notes/additional information ------------------------------------- */
int binex::decode_bnx_00_7f(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x01-00: coded (raw bytes) gnss ephemeris --------------------------------- */
int binex::decode_bnx_01_00(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x02: generalized gnss data ----------------------------------------------- */
int binex::decode_bnx_02(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x03: generalized ancillary site data ------------------------------------- */
int binex::decode_bnx_03(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x7d: receiver internal state prototyping --------------------------------- */
int binex::decode_bnx_7d(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x7e: ancillary site data prototyping ------------------------------------- */
int binex::decode_bnx_7e(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x7f-00: jpl fiducial site ------------------------------------------------ */
int binex::decode_bnx_7f_00(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x7f-01: ucar cosmic ------------------------------------------------------ */
int binex::decode_bnx_7f_01(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x7f-02: trimble 4700 ----------------------------------------------------- */
int binex::decode_bnx_7f_02(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x7f-03: trimble netrs ---------------------------------------------------- */
int binex::decode_bnx_7f_03(unsigned char *buff,int len){ return 0; }
/* decode binex mesaage 0x7f-04: trimble netrs ---------------------------------------------------- */
int binex::decode_bnx_7f_04(unsigned char *buff,int len){ return 0; }

/* decode binex mesaage 0x00: site/monument/marker/ref point/setup metadata------------------------ */
int binex::decode_bnx_00(unsigned char *bbb,int len){
	const double gpst0[]={ 1980,1,6,0,0,0 };
	char msg[100];
	unsigned char *p=bbb;
	unsigned int min,qsec,src,fid;
	int n=6;

	min =U4(p); p+=4;
	qsec=U1(p); p+=1;
	src =U1(p); p+=1;
	n+=getbnxi(p,fid);
	time.epoch2time(gpst0)->timeadd(min*60.0+qsec*0.25);

	if (outtype) {
		sprintf(msg, " fid=%02X time=%s src=%d",fid,time.time2str(0).c_str(),src);
		msgtype+=msg;
	}
	switch (fid) {
		case 0x00: return decode_bnx_00_00(bbb+n,len-n);
		case 0x01: return decode_bnx_00_01(bbb+n,len-n);
		case 0x02: return decode_bnx_00_02(bbb+n,len-n);
		case 0x03: return decode_bnx_00_03(bbb+n,len-n);
		case 0x04: return decode_bnx_00_04(bbb+n,len-n);
		case 0x05: return decode_bnx_00_05(bbb+n,len-n);
		case 0x06: return decode_bnx_00_06(bbb+n,len-n);
		case 0x07: return decode_bnx_00_07(bbb+n,len-n);
		case 0x08: return decode_bnx_00_08(bbb+n,len-n);
		case 0x09: return decode_bnx_00_09(bbb+n,len-n);
		case 0x0A: return decode_bnx_00_0a(bbb+n,len-n);
		case 0x0B: return decode_bnx_00_0b(bbb+n,len-n);
		case 0x0C: return decode_bnx_00_0c(bbb+n,len-n);
		case 0x0D: return decode_bnx_00_0d(bbb+n,len-n);
		case 0x0E: return decode_bnx_00_0e(bbb+n,len-n);
		case 0x0F: return decode_bnx_00_0f(bbb+n,len-n);
		case 0x10: return decode_bnx_00_10(bbb+n,len-n);
		case 0x11: return decode_bnx_00_11(bbb+n,len-n);
		case 0x12: return decode_bnx_00_12(bbb+n,len-n);
		case 0x13: return decode_bnx_00_13(bbb+n,len-n);
		case 0x14: return decode_bnx_00_14(bbb+n,len-n);
		case 0x15: return decode_bnx_00_15(bbb+n,len-n);
		case 0x16: return decode_bnx_00_16(bbb+n,len-n);
		case 0x17: return decode_bnx_00_17(bbb+n,len-n);
		case 0x18: return decode_bnx_00_18(bbb+n,len-n);
		case 0x19: return decode_bnx_00_19(bbb+n,len-n);
		case 0x1A: return decode_bnx_00_1a(bbb+n,len-n);
		case 0x1B: return decode_bnx_00_1b(bbb+n,len-n);
		case 0x1C: return decode_bnx_00_1c(bbb+n,len-n);
		case 0x1D: return decode_bnx_00_1d(bbb+n,len-n);
		case 0x1E: return decode_bnx_00_1e(bbb+n,len-n);
		case 0x1F: return decode_bnx_00_1f(bbb+n,len-n);
		case 0x20: return decode_bnx_00_20(bbb+n,len-n);
		case 0x21: return decode_bnx_00_21(bbb+n,len-n);
		case 0x22: return decode_bnx_00_22(bbb+n,len-n);
		case 0x7F: return decode_bnx_00_7f(bbb+n,len-n);
	}
	return 0;
}
/* decode binex mesaage 0x01-01: decoded gps ephmemeris ------------------------------------------- */
int binex::decode_bnx_01_01(unsigned char *bbb,int len){
	eph_t eph;
	unsigned char *p=bbb;
	double tow,ura,sqrtA;
	int prn,fflag;

	if (len>=127) {
		prn       =U1(p)+1;      p+=1;
		eph.week  =U2(p);        p+=2;
		tow       =I4(p);        p+=4;
		eph.toes  =I4(p);        p+=4;
		eph.tgd[0]=R4(p);        p+=4;
		eph.iodc  =I4(p);        p+=4;
		eph.f2    =R4(p);        p+=4;
		eph.f1    =R4(p);        p+=4;
		eph.f0    =R4(p);        p+=4;
		eph.iode  =I4(p);        p+=4;
		eph.deln  =R4(p)*SC2RAD; p+=4;
		eph.M0    =R8(p);        p+=8;
		eph.e     =R8(p);        p+=8;
		sqrtA     =R8(p);        p+=8;
		eph.cic   =R4(p);        p+=4;
		eph.crc   =R4(p);        p+=4;
		eph.cis   =R4(p);        p+=4;
		eph.crs   =R4(p);        p+=4;
		eph.cuc   =R4(p);        p+=4;
		eph.cus   =R4(p);        p+=4;
		eph.OMG0  =R8(p);        p+=8;
		eph.omg   =R8(p);        p+=8;
		eph.i0    =R8(p);        p+=8;
		eph.OMGd  =R4(p)*SC2RAD; p+=4;
		eph.idot  =R4(p)*SC2RAD; p+=4;
		ura       =R4(p)*0.1;    p+=4;
		eph.svh   =U2(p);        p+=2;
		fflag      =U2(p);
	}
	else {
		return -1;
	}
	if (!(eph.sat=satno(SYS_GPS,prn))) {
		return -1;
	}
	eph.A=sqrtA*sqrtA;
	eph.toe.gpst2time(eph.week,eph.toes);
	eph.toc.gpst2time(eph.week,eph.toes);
	eph.ttr=adjweek(eph.toe,tow);
	eph.fit=fflag&0xFF;
	eph.flag=(fflag>>8)&0x01;
	eph.code=(fflag>>9)&0x03;
	eph.sva=uraindex(ura);

	if (opt.find("-EPHALL")==string::npos) {
		if (nav.eph[eph.sat-1].iode==eph.iode&&
			nav.eph[eph.sat-1].iodc==eph.iodc) return 0; /* unchanged */
	}
	nav.eph[eph.sat-1]=eph;
	ephsat=eph.sat;
	return 2;
}
/* decode binex mesaage 0x01-02: decoded glonass ephmemeris --------------------------------------- */
int binex::decode_bnx_01_02(unsigned char *bbb,int len){
	geph_t geph;
	unsigned char *p=bbb;
	double ttod,tof,tau_gps;
	int prn,day,lleap;

	if (len>=119) {
		prn        =U1(p)+1;   p+=1;
		day        =U2(p);     p+=2;
		ttod        =U4(p);     p+=4;
		geph.taun  =-R8(p);    p+=8;
		geph.gamn  =R8(p);     p+=8;
		tof        =U4(p);     p+=4;
		geph.pos[0]=R8(p)*1E3; p+=8;
		geph.vel[0]=R8(p)*1E3; p+=8;
		geph.acc[0]=R8(p)*1E3; p+=8;
		geph.pos[1]=R8(p)*1E3; p+=8;
		geph.vel[1]=R8(p)*1E3; p+=8;
		geph.acc[1]=R8(p)*1E3; p+=8;
		geph.pos[2]=R8(p)*1E3; p+=8;
		geph.vel[2]=R8(p)*1E3; p+=8;
		geph.acc[2]=R8(p)*1E3; p+=8;
		geph.svh   =U1(p)&0x1; p+=1;
		geph.frq   =I1(p);     p+=1;
		geph.age   =U1(p);     p+=1;
		lleap       =U1(p);     p+=1;
		tau_gps    =R8(p);     p+=8;
		geph.dtaun =R8(p);
	}
	else {
		return -1;
	}
	if (!(geph.sat=satno(SYS_GLO,prn))) {
		return -1;
	}
	if (time.time==0) return 0;
	geph.toe=adjday(ttod-10800.0); geph.toe.utc2gpst();
	geph.tof=adjday(tof-10800.0); geph.tof.utc2gpst();
	geph.iode=(int)(fmod(ttod,86400.0)/900.0+0.5);

	if (opt.find("-EPHALL")==string::npos) {
		if (fabs(geph.toe.timediff(nav.geph[prn-MINPRNGLO].toe))<1.0&&
			geph.svh==nav.geph[prn-MINPRNGLO].svh) return 0; /* unchanged */
	}
	nav.geph[prn-1]=geph;
	ephsat=geph.sat;
	return 2;
}
/* decode binex mesaage 0x01-03: decoded sbas ephmemeris ------------------------------------------ */
int binex::decode_bnx_01_03(unsigned char *bbb,int len){
	seph_t seph;
	unsigned char *p=bbb;
	double tow,ttod,tof;
	int prn,week,iodn;

	if (len>=98) {
		prn        =U1(p);     p+=1;
		week       =U2(p);     p+=2;
		tow        =U4(p);     p+=4;
		seph.af0   =R8(p);     p+=8;
		ttod        =R4(p);     p+=4;
		tof        =U4(p);     p+=4;
		seph.pos[0]=R8(p)*1E3; p+=8;
		seph.vel[0]=R8(p)*1E3; p+=8;
		seph.acc[0]=R8(p)*1E3; p+=8;
		seph.pos[1]=R8(p)*1E3; p+=8;
		seph.vel[1]=R8(p)*1E3; p+=8;
		seph.acc[1]=R8(p)*1E3; p+=8;
		seph.pos[2]=R8(p)*1E3; p+=8;
		seph.vel[2]=R8(p)*1E3; p+=8;
		seph.acc[2]=R8(p)*1E3; p+=8;
		seph.svh   =U1(p);     p+=1;
		seph.sva   =U1(p);     p+=1;
		iodn       =U1(p);
	}
	else {
		return -1;
	}
	if (!(seph.sat=satno(SYS_SBS,prn))) {
		return -1;
	}
	seph.t0.gpst2time(week,tow);
	seph.tof=adjweek(seph.t0,tof);

	if (opt.find("-EPHALL")==string::npos) {
		if (fabs(seph.t0.timediff(nav.seph[prn-MINPRNSBS].t0))<1.0&&
			seph.sva==nav.seph[prn-MINPRNSBS].sva) return 0; /* unchanged */
	}
	nav.seph[prn-MINPRNSBS]=seph;
	ephsat=seph.sat;
	return 2;
}
/* decode binex mesaage 0x01-04: decoded galileo ephmemeris --------------------------------------- */
int binex::decode_bnx_01_04(unsigned char *bbb,int len){
	eph_t eph;
	unsigned char *p=bbb;
	double tow,ura,sqrtA;
	int prn;

	if (len>=127) {
		prn       =U1(p)+1;      p+=1;
		eph.week  =U2(p);        p+=2;
		tow       =I4(p);        p+=4;
		eph.toes  =I4(p);        p+=4;
		eph.tgd[0]=R4(p);        p+=4; /* BGD E5a/E1 */
		eph.tgd[1]=R4(p);        p+=4; /* BGD E5b/E1 */
		eph.iode  =I4(p);        p+=4; /* IODnav */
		eph.f2    =R4(p);        p+=4;
		eph.f1    =R4(p);        p+=4;
		eph.f0    =R4(p);        p+=4;
		eph.deln  =R4(p)*SC2RAD; p+=4;
		eph.M0    =R8(p);        p+=8;
		eph.e     =R8(p);        p+=8;
		sqrtA     =R8(p);        p+=8;
		eph.cic   =R4(p);        p+=4;
		eph.crc   =R4(p);        p+=4;
		eph.cis   =R4(p);        p+=4;
		eph.crs   =R4(p);        p+=4;
		eph.cuc   =R4(p);        p+=4;
		eph.cus   =R4(p);        p+=4;
		eph.OMG0  =R8(p);        p+=8;
		eph.omg   =R8(p);        p+=8;
		eph.i0    =R8(p);        p+=8;
		eph.OMGd  =R4(p)*SC2RAD; p+=4;
		eph.idot  =R4(p)*SC2RAD; p+=4;
		ura       =R4(p)*0.1;    p+=4;
		eph.svh   =U2(p);        p+=2;
		eph.code  =U2(p);              /* data source */
	}
	else {
		return -1;
	}
	if (!(eph.sat=satno(SYS_GAL,prn))) {
		return -1;
	}
	eph.A=sqrtA*sqrtA;
	eph.iode=eph.iodc;
	eph.toe.gpst2time(eph.week,eph.toes);
	eph.toc.gpst2time(eph.week,eph.toes);
	eph.ttr=adjweek(eph.toe,tow);
	eph.sva=uraindex(ura);

	if (opt.find("-EPHALL")==string::npos) {
		if (nav.eph[eph.sat-1].iode==eph.iode&&
			nav.eph[eph.sat-1].iodc==eph.iodc) return 0; /* unchanged */
	}
	nav.eph[eph.sat-1]=eph;
	ephsat=eph.sat;
	return 2;
}
/* decode binex mesaage 0x01-05: decoded beidou-2/compass ephmemeris ------------------------------ */
int binex::decode_bnx_01_05(unsigned char *bbb,int len){
	eph_t eph;
	unsigned char *p=bbb;
	double tow,toc,sqrtA;
	int prn,flag1,flag2;

	if (len>=117) {
		prn       =U1(p);        p+=1;
		eph.week  =U2(p);        p+=2;
		tow       =I4(p);        p+=4;
		toc       =I4(p);        p+=4;
		eph.toes  =I4(p);        p+=4;
		eph.f2    =R4(p);        p+=4;
		eph.f1    =R4(p);        p+=4;
		eph.f0    =R4(p);        p+=4;
		eph.deln  =R4(p)*SC2RAD; p+=4;
		eph.M0    =R8(p);        p+=8;
		eph.e     =R8(p);        p+=8;
		sqrtA     =R8(p);        p+=8;
		eph.cic   =R4(p);        p+=4;
		eph.crc   =R4(p);        p+=4;
		eph.cis   =R4(p);        p+=4;
		eph.crs   =R4(p);        p+=4;
		eph.cuc   =R4(p);        p+=4;
		eph.cus   =R4(p);        p+=4;
		eph.OMG0  =R8(p);        p+=8;
		eph.omg   =R8(p);        p+=8;
		eph.i0    =R8(p);        p+=8;
		eph.OMGd  =R4(p)*SC2RAD; p+=4;
		eph.idot  =R4(p)*SC2RAD; p+=4;
		flag1     =U2(p);        p+=2;
		flag2     =U4(p);
	}
	else {
		return -1;
	}
	if (!(eph.sat=satno(SYS_CMP,prn))) {
		return 0;
	}
	eph.A=sqrtA*sqrtA;
	eph.toe.gpst2time(eph.week+1356,eph.toes+14.0); /* bdt -> gpst */
	eph.toc.gpst2time(eph.week+1356,eph.toes+14.0); /* bdt -> gpst */
	eph.ttr=adjweek(eph.toe,tow+14.0); /* bdt -> gpst */
	eph.iodc=(flag1>>1)&0x1F;
	eph.iode=(flag1>>6)&0x1F;
	eph.svh=flag1&0x01;
	eph.sva=flag2&0x0F; /* ura index */
	eph.tgd[0]=bds_tgd(flag2>> 4); /* TGD1 (s) */
	eph.tgd[1]=bds_tgd(flag2>>14); /* TGD2 (s) */
	eph.flag=(flag1>>11)&0x07; /* nav type (0:unknown,1:IGSO/MEO,2:GEO) */
	eph.code=(flag2>>25)&0x7F;
	/* message source (0:unknown,1:B1I,2:B1Q,3:B2I,4:B2Q,5:B3I,6:B3Q)*/

	if (opt.find("-EPHALL")==string::npos) {
		if (nav.eph[eph.sat-1].iode==eph.iode&&
			nav.eph[eph.sat-1].iodc==eph.iodc) return 0; /* unchanged */
	}
	nav.eph[eph.sat-1]=eph;
	ephsat=eph.sat;
	return 2;
}
/* decode binex mesaage 0x01-06: decoded qzss ephmemeris ------------------------------------------ */
int binex::decode_bnx_01_06(unsigned char *bbb,int len){
	eph_t eph;
	unsigned char *p=bbb;
	double tow,ura,sqrtA;
	int prn,fflag;

	if (len>=127) {
		prn       =U1(p);        p+=1;
		eph.week  =U2(p);        p+=2;
		tow       =I4(p);        p+=4;
		eph.toes  =I4(p);        p+=4;
		eph.tgd[0]=R4(p);        p+=4;
		eph.iodc  =I4(p);        p+=4;
		eph.f2    =R4(p);        p+=4;
		eph.f1    =R4(p);        p+=4;
		eph.f0    =R4(p);        p+=4;
		eph.iode  =I4(p);        p+=4;
		eph.deln  =R4(p)*SC2RAD; p+=4;
		eph.M0    =R8(p);        p+=8;
		eph.e     =R8(p);        p+=8;
		sqrtA     =R8(p);        p+=8;
		eph.cic   =R4(p);        p+=4;
		eph.crc   =R4(p);        p+=4;
		eph.cis   =R4(p);        p+=4;
		eph.crs   =R4(p);        p+=4;
		eph.cuc   =R4(p);        p+=4;
		eph.cus   =R4(p);        p+=4;
		eph.OMG0  =R8(p);        p+=8;
		eph.omg   =R8(p);        p+=8;
		eph.i0    =R8(p);        p+=8;
		eph.OMGd  =R4(p)*SC2RAD; p+=4;
		eph.idot  =R4(p)*SC2RAD; p+=4;
		ura       =R4(p)*0.1;    p+=4;
		eph.svh   =U2(p);        p+=2;
		fflag      =U2(p);
	}
	else {
		return -1;
	}
	if (!(eph.sat=satno(SYS_QZS,prn))) {
		return 0;
	}
	eph.A=sqrtA*sqrtA;
	eph.toe.gpst2time(eph.week,eph.toes);
	eph.toc.gpst2time(eph.week,eph.toes);
	eph.ttr=adjweek(eph.toe,tow);
	eph.fit=(fflag&0x01) ? 0.0 : 2.0; /* 0:2hr,1:>2hr */
	eph.sva=uraindex(ura);
	eph.code=2; /* codes on L2 channel */

	if (opt.find("-EPHALL")==string::npos) {
		if (nav.eph[eph.sat-1].iode==eph.iode&&
			nav.eph[eph.sat-1].iodc==eph.iodc) return 0; /* unchanged */
	}
	nav.eph[eph.sat-1]=eph;
	ephsat=eph.sat;
	return 2;
}
/* decode binex mesaage 0x01: gnss navigaion informtion ------------------------------------------- */
int binex::decode_bnx_01(unsigned char *bbb,int len){
	char msg[60];
	int srec=U1(bbb),prn=U1(bbb+1);

	if (outtype) {
		prn=srec==0x01||srec==0x02||srec==0x04 ? prn+1 : (srec==0x00 ? 0 : prn);
		sprintf(msg, " subrec=%02X prn=%d",srec,prn);
		msgtype+=msg;
	}
	switch (srec) {
	case 0x00: return decode_bnx_01_00(bbb+1,len-1);
	case 0x01: return decode_bnx_01_01(bbb+1,len-1);
	case 0x02: return decode_bnx_01_02(bbb+1,len-1);
	case 0x03: return decode_bnx_01_03(bbb+1,len-1);
	case 0x04: return decode_bnx_01_04(bbb+1,len-1);
	case 0x05: return decode_bnx_01_05(bbb+1,len-1);
	case 0x06: return decode_bnx_01_06(bbb+1,len-1);
	}
	return 0;
}
/* decode binex mesaage 0x7f-05: trimble netr8 obs data ------------------------------------------- */
unsigned char * binex::decode_bnx_7f_05_obs(unsigned char *bbb,
	int sat,int nobs,obsd_t &data){
	const unsigned char codes_gps[32]={
		CODE_L1C ,CODE_L1C ,CODE_L1P ,CODE_L1W ,CODE_L1Y ,CODE_L1M , /*  0- 5 */
		CODE_L1X ,CODE_L1N ,CODE_NONE,CODE_NONE,CODE_L2W ,CODE_L2C , /*  6-11 */
		CODE_L2D ,CODE_L2S ,CODE_L2L ,CODE_L2X ,CODE_L2P ,CODE_L2W , /* 12-17 */
		CODE_L2Y ,CODE_L2M ,CODE_L2N ,CODE_NONE,CODE_NONE,CODE_L5X , /* 18-23 */
		CODE_L5I ,CODE_L5Q ,CODE_L5X                                 /* 24-26 */
	};
	const unsigned char codes_glo[32]={
		CODE_L1C ,CODE_L1C ,CODE_L1P ,CODE_NONE,CODE_NONE,CODE_NONE, /*  0- 5 */
		CODE_NONE,CODE_NONE,CODE_NONE,CODE_NONE,CODE_L2C ,CODE_L2C , /*  6-11 */
		CODE_L2P ,CODE_L3X ,CODE_L3I ,CODE_L3Q ,CODE_L3X             /* 12-16 */
	};
	const unsigned char codes_gal[32]={
		CODE_L1C ,CODE_L1A ,CODE_L1B ,CODE_L1C ,CODE_L1X ,CODE_L1Z , /*  0- 5 */
		CODE_L5X ,CODE_L5I ,CODE_L5Q ,CODE_L5X ,CODE_L7X ,CODE_L7I , /*  6-11 */
		CODE_L7Q ,CODE_L7X ,CODE_L8X ,CODE_L8I ,CODE_L8Q ,CODE_L8X , /* 12-17 */
		CODE_L6X ,CODE_L6A ,CODE_L6B ,CODE_L6C ,CODE_L6X ,CODE_L6Z , /* 18-23 */
	};
	const unsigned char codes_sbs[32]={
		CODE_L1C ,CODE_L1C ,CODE_NONE,CODE_NONE,CODE_NONE,CODE_NONE, /*  0- 5 */
		CODE_L5X ,CODE_L5I ,CODE_L5Q ,CODE_L5X                       /*  6- 9 */
	};
	const unsigned char codes_cmp[32]={
		CODE_L1X ,CODE_L1I ,CODE_L1Q ,CODE_L1X ,CODE_L7X ,CODE_L7I , /*  0- 5 */
		CODE_L7Q ,CODE_L7X ,CODE_L6X ,CODE_L6I ,CODE_L6Q ,CODE_L6X , /*  6-11 */
		CODE_L1X ,CODE_L1S ,CODE_L1L ,CODE_L1X                       /* 12-15 */
	};
	const unsigned char codes_qzs[32]={
		CODE_L1C ,CODE_L1C ,CODE_L1S ,CODE_L1L ,CODE_L1X ,CODE_NONE, /*  0- 5 */
		CODE_NONE,CODE_L2X ,CODE_L2S ,CODE_L2L ,CODE_L2X ,CODE_NONE, /*  6-11 */
		CODE_NONE,CODE_L5X ,CODE_L5I ,CODE_L5Q ,CODE_L5X ,CODE_NONE, /* 12-17 */
		CODE_NONE,CODE_L6X ,CODE_L6S ,CODE_L6L ,CODE_L6X ,CODE_NONE, /* 18-23 */
		CODE_NONE,CODE_NONE,CODE_NONE,CODE_NONE,CODE_NONE,CODE_NONE, /* 24-29 */
		CODE_L1Z                                                     /* 30-30 */
	};
	const unsigned char *codes=NULL;
	double range[8],phase[8],cnr[8],dopp[8]={ 0 },acc,wl;
	unsigned char *p=bbb;
	unsigned char fflag,flags[4];
	int i,j,k,sys,fcn=-10,code[8],slip[8],pri[8],freq[8],slipcnt[8]={ 0 },mask[8]={ 0 };

	sys=satsys(sat,NULL);

	switch (sys) {
	case SYS_GPS: codes=codes_gps; break;
	case SYS_GLO: codes=codes_glo; break;
	case SYS_GAL: codes=codes_gal; break;
	case SYS_QZS: codes=codes_qzs; break;
	case SYS_SBS: codes=codes_sbs; break;
	case SYS_CMP: codes=codes_cmp; break;
	}
	for (i=0; i<nobs; i++) {

		fflag  =getbitu(p,0,1);
		slip[i]=getbitu(p,2,1);
		code[i]=getbitu(p,3,5); p++;

		for (j=0; j<4; j++) flags[j]=0;

		for (j=0; fflag&&j<4; j++) {
			fflag=U1(p++);
			flags[fflag&0x03]=fflag&0x7F;
			fflag&=0x80;
		}
		if (flags[2]) {
			fcn=getbits(flags+2,2,4);
		}
		acc=(flags[0]&0x20) ? 0.0001 : 0.00002; /* phase accuracy */

		cnr[i]=U1(p++)*0.4;

		if (i==0) {
			cnr[i]+=getbits(p,0,2)*0.1;
			range[i]=getbitu(p,2,32)*0.064+getbitu(p,34,6)*0.001; p+=5;
		}
		else if (flags[0]&0x40) {
			cnr[i]+=getbits(p,0,2)*0.1;
			range[i]=range[0]+getbits(p,4,20)*0.001; p+=3;
		}
		else {
			range[i]=range[0]+getbits(p,0,16)*0.001; p+=2;
		}
		if (flags[0]&0x40) {
			phase[i]=range[i]+getbits(p,0,24)*acc; p+=3;
		}
		else {
			cnr[i]+=getbits(p,0,2)*0.1;
			phase[i]=range[i]+getbits(p,2,22)*acc; p+=3;
		}
		if (flags[0]&0x04) {
			dopp[i]=getbits(p,0,24)/256.0; p+=3;
		}
		if (flags[0]&0x08) {
			if (flags[0]&0x10) {
				slipcnt[i]=U2(p); p+=2;
			}
			else {
				slipcnt[i]=U1(p); p+=1;
			}
		}
	}
	if (!codes) {
		data.sat=0;
		return p;
	}
	data.time=time;
	data.sat=sat;

	/* get code priority */
	for (i=0; i<nobs; i++) {
		code2obs(codes[code[i]&0x3F],freq+i);
		pri[i]=getcodepri(sys,codes[code[i]&0x3F],opt);

		/* frequency index for beidou */
		if (sys==SYS_CMP) {
			if (freq[i]==5) freq[i]=2; /* B2 */
			else if (freq[i]==4) freq[i]=3; /* B3 */
		}
	}
	for (i=0; i<NFREQ; i++) {
		for (j=0,k=-1; j<nobs; j++) {
			if (freq[j]==i+1&&(k<0||pri[j]>pri[k])) k=j;
		}
		if (k<0) {
			data.P[i]=data.L[i]=0.0;
			data.D[i]=0.0f;
			data.SNR[i]=data.LLI[i]=0;
			data.code[i]=CODE_NONE;
		}
		else {
			wl=satwavelen(sat,i,&nav);
			if (sys==SYS_GLO&&fcn>=-7&&freq[k]<=2) {
				wl=CLIGHT/(freq[k]==1 ? FREQ1_GLO+DFRQ1_GLO*fcn :
					FREQ2_GLO+DFRQ2_GLO*fcn);
			}
			data.P[i]=range[k];
			data.L[i]=wl<=0.0 ? 0.0 : phase[k]/wl;
			data.D[i]=dopp[k];
			data.SNR[i]=(unsigned char)(cnr[k]/0.25+0.5);
			data.code[i]=codes[code[k]&0x3F];
			data.LLI[i]=slip[k] ? 1 : 0;
			mask[k]=1;
		}
	}
	for (; i<NFREQ+NEXOBS; i++) {
		for (k=0; k<nobs; k++) {
			if (!mask[k]) break;
		}
		if (k>=nobs) {
			data.P[i]=data.L[i]=0.0;
			data.D[i]=0.0f;
			data.SNR[i]=data.LLI[i]=0;
			data.code[i]=CODE_NONE;
		}
		else {
			wl=satwavelen(sat,freq[k]-1,&nav);
			if (sys==SYS_GLO&&fcn>=-7&&freq[k]<=2) {
				wl=CLIGHT/(freq[k]==1 ? FREQ1_GLO+DFRQ1_GLO*fcn :
					FREQ2_GLO+DFRQ2_GLO*fcn);
			}
			data.P[i]=range[k];
			data.L[i]=wl<=0.0 ? 0.0 : phase[k]/wl;
			data.D[i]=dopp[k];
			data.SNR[i]=(unsigned char)(cnr[k]/0.25+0.5);
			data.code[i]=codes[code[k]&0x3F];
			data.LLI[i]=slip[k] ? 1 : 0;
			mask[k]=1;
		}
	}
	return p;
}
/* decode binex mesaage 0x7f-05: trimble netr8 ---------------------------------------------------- */
int binex::decode_bnx_7f_05(unsigned char *bbb,int len){
	obsd_t data;
	double clkoff=0.0,toff[16]={ 0 };
	char msg[100];
	unsigned char *p=bbb;
	unsigned int fflag;
	int i,nsat,nobs,prn,sys,sat,clkrst=0,rsys=0,nsys=0,tsys[16]={ 0 };

	obs.n=0;
	fflag=U1(p++);
	nsat=(int)(fflag&0x3F)+1;

	if (fflag&0x80) { /* rxclkoff */
		clkrst=getbitu(p,0,2);
		clkoff=getbits(p,2,22)*1E-9; p+=3;
	}
	if (fflag&0x40) { /* systime */
		nsys=getbitu(p,0,4);
		rsys=getbitu(p,4,4); p++;
		for (i=0; i<nsys; i++) {
			toff[i]=getbits(p,0,24)*1E-9;
			tsys[i]=getbitu(p,28,4); p+=4;
		}
	}
	for (i=0; i<nsat; i++) {
		prn =U1(p++);
		nobs=getbitu(p,1,3);
		sys =getbitu(p,4,4); p++;

		switch (sys) {
			case 0: sat=satno(SYS_GPS,prn); break;
			case 1: sat=satno(SYS_GLO,prn); break;
			case 2: sat=satno(SYS_SBS,prn); break;
			case 3: sat=satno(SYS_GAL,prn); break;
			case 4: sat=satno(SYS_CMP,prn); break;
			case 5: sat=satno(SYS_QZS,prn); break;
			default: sat=0; break;
		}
		/* decode binex mesaage 0x7F-05 obs data */
		if (!(p=decode_bnx_7f_05_obs(p,sat,nobs,data))) return -1;

		if ((int)(p-bbb)>len) {
			return -1;
		}
		/* save obs data to obs buffer */
		if (data.sat&&obs.n<MAXOBS) {
			obs.data[obs.n++]=data;
		}
	}
	if (outtype) {
		sprintf(msg, " nsat=%2d",nsat);
		msgtype+=msg;
	}
	return obs.n>0 ? 1 : 0;
}
/* decode binex mesaage 0x7f: gnss data prototyping ----------------------------------------------- */
int binex::decode_bnx_7f(unsigned char *bbb,int len){
	const double gpst0[]={ 1980,1,6,0,0,0 };
	char msg[60];
	unsigned char *p=bbb;
	unsigned int srec,min,msec;

	srec=U1(p); p+=1; /* subrecord id */
	min =U4(p); p+=4;
	msec=U2(p); p+=2;
	time.epoch2time(gpst0)->timeadd(min*60.0+msec*0.001);

	if (outtype) {
		sprintf(msg, " subrec=%02X time%s",srec,time.time2str(3).c_str());
		msgtype+=msg;
	}
	switch (srec) {
	case 0x00: return decode_bnx_7f_00(bbb+7,len-7);
	case 0x01: return decode_bnx_7f_01(bbb+7,len-7);
	case 0x02: return decode_bnx_7f_02(bbb+7,len-7);
	case 0x03: return decode_bnx_7f_03(bbb+7,len-7);
	case 0x04: return decode_bnx_7f_04(bbb+7,len-7);
	case 0x05: return decode_bnx_7f_05(bbb+7,len-7);
	}
	return 0;
}

/* decode binex mesaage --------------------------------------------------------------------------- */
int binex::decode_bnx(){
	unsigned int lll,cs1,cs2;
	int rec,len_h;
	char msg[100];

	rec=buff[1]; /* record id */

	/* record and header length */
	len_h=getbnxi(buff+2,lll);

	/* check parity */
	if (len-1<128) {
		cs1=U1(buff+len);
		cs2=csum8(len-1);
	}
	else {
		cs1=U2(buff+len);
		cs2=rtk_crc16(buff+1,len-1);
	}
	if (cs1!=cs2) {
		return -1;
	}
	if (outtype) {
		sprintf(msg, "BINEX 0x%02X (%4d)",rec,len);
		msgtype=msg;
	}
	/* decode binex message record */
	switch (rec) {
		case 0x00: return decode_bnx_00(buff+2+len_h,lll);
		case 0x01: return decode_bnx_01(buff+2+len_h,lll);
		case 0x02: return decode_bnx_02(buff+2+len_h,lll);
		case 0x03: return decode_bnx_03(buff+2+len_h,lll);
		case 0x7d: return decode_bnx_7d(buff+2+len_h,lll);
		case 0x7e: return decode_bnx_7e(buff+2+len_h,lll);
		case 0x7f: return decode_bnx_7f(buff+2+len_h,lll);
	}
	return 0;
}

/* input binex message from stream ---------------------------------------------------------------- */
int binex::decode(unsigned char data){
	unsigned int lll;
	int len_h,len_c;

	/* synchronize binex message */
	if (nbyte==0) {
		if (!sync_bnx(data)) return 0;
		nbyte=2;
		return 0;
	}
	buff[nbyte++]=data;
	if (nbyte<4) return 0;

	len_h=getbnxi(buff+2,lll);

	len=lll+len_h+2; /* length without crc */

	if (len-1>4096) {
		nbyte=0;
		return -1;
	}
	len_c=len-1<128 ? 1 : 2;

	if (nbyte<(int)(len+len_c)) return 0;
	nbyte=0;

	/* decode binex message */
	return decode_bnx();
}