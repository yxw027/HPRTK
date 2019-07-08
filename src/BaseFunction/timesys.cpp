/* Time function source file ---------------------------------------------------------------------- */

#include "BaseFunction/timesys.h"

#include "hprtk_lib.h"
#include "BaseFunction/basefunction.h"

/* const */
static const double gpst0[] = { 1980,1, 6,0,0,0 }; /* gps time reference */
static const double gst0[] = { 1999,8,22,0,0,0 }; /* galileo system time reference */
static const double bdt0[] = { 2006,1, 1,0,0,0 }; /* beidou time reference */

static double leaps[][MAXLEAPS + 1] = { /* leap seconds (y,m,d,h,m,s,utc-gpst) */
	{ 2017,1,1,0,0,0,-18 },
	{ 2015,7,1,0,0,0,-17 },
	{ 2012,7,1,0,0,0,-16 },
	{ 2009,1,1,0,0,0,-15 },
	{ 2006,1,1,0,0,0,-14 },
	{ 1999,1,1,0,0,0,-13 },
	{ 1997,7,1,0,0,0,-12 },
	{ 1996,1,1,0,0,0,-11 },
	{ 1994,7,1,0,0,0,-10 },
	{ 1993,7,1,0,0,0, -9 },
	{ 1992,7,1,0,0,0, -8 },
	{ 1991,1,1,0,0,0, -7 },
	{ 1990,1,1,0,0,0, -6 },
	{ 1988,1,1,0,0,0, -5 },
	{ 1985,7,1,0,0,0, -4 },
	{ 1983,7,1,0,0,0, -3 },
	{ 1982,7,1,0,0,0, -2 },
	{ 1981,7,1,0,0,0, -1 },
	{ 0   ,0,0,0,0,0,  0 }
};

/* difference with other time ----------------------------------------------------- */
gtime_t::gtime_t(){
	time=0; sec=0.0;
	sys=0; sep="\0";
	for (int i=0;i<6;i++) ep[i]=0.0;
	doy=0;
}
/* initialize with epoch array ---------------------------------------------------- */
gtime_t::gtime_t(const double *epoch){
	epoch2time(epoch);
}
gtime_t::~gtime_t() {
}

/* string to time --------------------------------------------------------------------
* convert substring in string to gtime_t struct
* args   : char   *s        I   string ("... yyyy mm dd hh mm ss ...")
----------------------------------------------------------------------------------- */
int gtime_t::str2time(string s){

	if (sscanf(s.c_str(),"%lf %lf %lf %lf %lf %lf",ep,ep+1,ep+2,ep+3,ep+4,ep+5)<6)
		return -1;
	if (ep[0]<100) ep[0]+=2000;
	if (ep[0]<=1990||ep[1]==0||ep[2]==0) return -1;

	epoch2time(ep);

	sep=s;

	return 0;
}

/* ep time to string ---------------------------------------------------------------------------------
string sep yyyy/mm/dd hh:mm:ss.ssss...
--------------------------------------------------------------------------------------------------- */
string gtime_t::time2str(int n){

	if (n<0) n=0; else if (n>12) n=12;
	string str;
	if (1.0-sec<0.5/pow(10.0,n)) { time++; sec=0.0; };
	time2epoch();
	sep=int2str(4,"0",(int)ep[0],str)+"/"+int2str(2,"0",(int)ep[1],str)+"/"+
		int2str(2,"0",(int)ep[2],str)+" "+int2str(2,"0",(int)ep[3],str)+":"+
		int2str(2,"0",(int)ep[4],str)+":"+doul2str(2+n+1,n,"0",ep[5],str);
	return sep;
}

/* calender day/time (ep) to time ----------------------------------------------------------------- */
gtime_t *gtime_t::epoch2time(const double *inep){
	const int doy[]={ 1,32,60,91,121,152,182,213,244,274,305,335 };

	int days, dsec, year=int(inep[0]), mon=(int)inep[1], day=(int)inep[2];

	if (year<1970||2099<year||mon<1||12<mon) {
		time=0;sec=0; return this;
	}

	/* leap year if year%4==0 in 1901-2099 */
	days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3 ? 1 : 0);
	dsec=(int)floor(inep[5]);
	time=(time_t)days*86400+(time_t)inep[3]*3600+(time_t)inep[4]*60+dsec;
	sec=inep[5]-dsec;

	return this;
}

/* time to calender day/time (ep) --------------------------------------------------------------------
 ep={yyyy,mm,dd,hh,mm,ss.ssss...}
--------------------------------------------------------------------------------------------------- */
void gtime_t::time2epoch(){
	const int mday[]={ /* # of days in a month */
		31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
		31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
	};
	int days, dsec, mon, day;

	/* leap year if year%4==0 in 1901-2099 */
	days=(int)(time/86400);
	dsec=(int)(time-(time_t)days*86400);
	for (day=days%1461, mon=0; mon<48; mon++) {
		if (day>=mday[mon]) day-=mday[mon]; else break;
	}
	ep[0]=1970+days/1461*4+mon/12; ep[1]=mon%12+1; ep[2]=day+1;
	ep[3]=dsec/3600; ep[4]=dsec%3600/60; ep[5]=dsec%60+sec;
}

/* gps week time to gtime_t --------------------------------------------------------------------------
* args   : int    week      I   week number in gps time
*          double sss       I   time of week in gps time (s)
--------------------------------------------------------------------------------------------------- */
gtime_t *gtime_t::gpst2time(int week, double sss){
	epoch2time(gpst0);

	if (sss<-1E9||1E9<sss) sss=0.0;
	time+=(time_t)86400*7*week+(int)sss;
	sec=sss-(int)sss;

	return this;
}

/* gtime_t to gps week time --------------------------------------------------------------------------
*  args  : int    *week     IO  week number in gps time (NULL: no output)
--------------------------------------------------------------------------------------------------- */
double gtime_t::time2gpst(int *week) const{
	gtime_t t0; 
	time_t sss;

	t0.epoch2time(gpst0);
	sss=time-t0.time;

	int w=(int)(sss/(86400*7));
	if (week) *week=w;

	return (double)(sss-(double)w*86400*7)+sec;
}

/* galileo week time to gtime_t ----------------------------------------------------------------------
* args   : int    week      I   week number in gst
*          double sec       I   time of week in gst (s)
--------------------------------------------------------------------------------------------------- */
gtime_t *gtime_t::gst2time(int week, double sss){
	epoch2time(gst0);

	if (sss<-1E9||1E9<sss) sss=0.0;
	time+=(time_t)86400*7*week+(int)sss;
	sec=sss-(int)sss;

	return this;
}

/* gtime_t to galileo week time ----------------------------------------------------------------------
*  args  : int    *week     IO  week number in gst (NULL: no output)
--------------------------------------------------------------------------------------------------- */
double gtime_t::time2gst(int *week){
	gtime_t t0;
	time_t sss;

	t0.epoch2time(gst0);
	sss=time-t0.time;
	
	int w=(int)(sss/(86400*7));
	if (week) *week=w;

	return (double)(sss-(double)w*86400*7)+sec;
}

/* Beidou week time to gtime_t -----------------------------------------------------------------------
* args   : int    week      I   week number in bdt
*          double sss       I   time of week in bdt (s)
--------------------------------------------------------------------------------------------------- */
gtime_t *gtime_t::bdt2time(int week, double sss){
	epoch2time(bdt0);

	if (sss<-1E9||1E9<sss) sss=0.0;
	time+=(time_t)86400*7*week+(int)sss;
	sec=sss-(int)sss;

	return this;
}

/* gtime_t to Beidou week time -----------------------------------------------------------------------
* args   : int    *week     IO  week number in bdt (NULL: no output)
--------------------------------------------------------------------------------------------------- */
double gtime_t::time2bdt(int *week){
	gtime_t t0;
	time_t sss;

	t0.epoch2time(bdt0);
	sss=time-t0.time;

	int w=(int)(sss/(86400*7));
	if (week) *week=w;

	return (double)(sss-(double)w*86400*7)+sec;
}

/* add dsec(s) to gtime_t ------------------------------------------------------------------------- */
gtime_t *gtime_t::timeadd(double dsec){
	double tt;
	sec+=dsec; tt=floor(sec); time+=(int)tt; sec-=tt;
	return this;
}

/* difference with gtime_t t2 --------------------------------------------------------------------- */
double gtime_t::timediff(const gtime_t t2)	const
{
	return difftime(time,t2.time)+sec-t2.sec;
}

/* get current time in utc ---------------------------------------------------------------------------
* get current time in utc
* args   : none
* return : current time in utc
*-------------------------------------------------------------------------------------------------- */
static double timeoffset_=0.0;        /* time offset (s) */

gtime_t *gtime_t::timeget(){
#ifdef WIN32
	SYSTEMTIME ts;

	GetSystemTime(&ts); /* utc */
	ep[0]=ts.wYear; ep[1]=ts.wMonth;  ep[2]=ts.wDay;
	ep[3]=ts.wHour; ep[4]=ts.wMinute; ep[5]=ts.wSecond+ts.wMilliseconds*1E-3;
#else
	struct timeval tv;
	struct tm *tt;

	if (!gettimeofday(&tv,NULL)&&(tt=gmtime(&tv.tv_sec))) {
		ep[0]=tt->tm_year+1900; ep[1]=tt->tm_mon+1; ep[2]=tt->tm_mday;
		ep[3]=tt->tm_hour; ep[4]=tt->tm_min; ep[5]=tt->tm_sec+tv.tv_usec*1E-6;
	}
#endif
	epoch2time(ep);

#ifdef CPUTIME_IN_GPST /* cputime operated in gpst */
	gpst2utc();
#endif
	return timeadd(timeoffset_);
}
/* set current time in utc ---------------------------------------------------------------------------
* set current time in utc
* args   : gtime_t          I   current time in utc
* return : none
* notes  : just set time offset between cpu time and current time
*          the time offset is reflected to only timeget()
*          not reentrant
*-------------------------------------------------------------------------------------------------- */
void gtime_t::timeset(){
	gtime_t t0;
	timeoffset_+=timediff(*t0.timeget());
}

/* gpstime to utc ------------------------------------------------------------------------------------
* convert gpstime to utc considering leap seconds
* return : time expressed in utc
* notes  : ignore slight time offset under 100 ns
*-------------------------------------------------------------------------------------------------- */
gtime_t *gtime_t::gpst2utc(){
	gtime_t tu,t0;
	int i;

	for (i=0; leaps[i][0]>0; i++) {
		tu=*this;
		tu.timeadd(leaps[i][6]);
		if (tu.timediff(*t0.epoch2time(leaps[i]))>=0.0) { *this=tu; return this; }
	}
	return this;
}

/* utc to gpstime ------------------------------------------------------------------------------------
* convert utc to gpstime considering leap seconds
* return : time expressed in gpstime
* notes  : ignore slight time offset under 100 ns
*-------------------------------------------------------------------------------------------------- */
gtime_t *gtime_t::utc2gpst(){
	int i;
	gtime_t t0;

	for (i=0;leaps[i][0]>0;i++) {
		if (timediff(*t0.epoch2time(leaps[i]))>=0.0)
			return timeadd(-leaps[i][6]);
	}
	return this;
}

/* gpstime to bdt ------------------------------------------------------------------------------------
* convert gpstime to bdt (beidou navigation satellite system time)
* return : time expressed in bdt
* notes  : ref [8] 3.3, 2006/1/1 00:00 BDT = 2006/1/1 00:00 UTC
*          no leap seconds in BDT
*          ignore slight time offset under 100 ns
*-------------------------------------------------------------------------------------------------- */
gtime_t *gtime_t::gpst2bdt(){
	return timeadd(-14.0);
}
/* bdt to gpstime ------------------------------------------------------------------------------------
* convert bdt (beidou navigation satellite system time) to gpstime
* return : time expressed in gpstime
* notes  : see gpst2bdt()
*-------------------------------------------------------------------------------------------------- */
gtime_t *gtime_t::bdt2gpst(){
	return timeadd(14.0);
}

/* time to day and sec ---------------------------------------------------------------------------- */
double gtime_t::time2sec(gtime_t &day){
	double sss;
	double ep0[6]={0};
	int i;
	time2epoch();

	sss=ep[3]*3600.0+ep[4]*60.0+ep[5];
	for (i=0;i<3;i++) ep0[i]=ep[i];
	day.epoch2time(ep0);
	return sss;
}

/* utc to gmst ---------------------------------------------------------------------------------------
* convert utc to gmst (Greenwich mean sidereal time)
* args   : gtime_t t        I   time expressed in utc
*          double ut1_utc   I   UT1-UTC (s)
* return : gmst (rad)
*-------------------------------------------------------------------------------------------------- */
double gtime_t::utc2gmst(double ut1_utc){
	const double ep2000[]={ 2000,1,1,12,0,0 };
	gtime_t tut,tut0,t2000;
	double ut,t1,t2,t3,gmst0,gmst;

	tut=*this; tut.timeadd(ut1_utc);
	ut=tut.time2sec(tut0);
	t1=tut0.timediff(*t2000.epoch2time(ep2000))/86400.0/36525.0;
	t2=t1*t1; t3=t2*t1;
	gmst0=24110.54841+8640184.812866*t1+0.093104*t2-6.2E-6*t3;
	gmst=gmst0+1.002737909350795*ut;

	return fmod(gmst,86400.0)*PI/43200.0; /* 0 <= gmst <= 2*PI */
}

/* day of year to time ---------------------------------------------------------------------------- */
int gtime_t::doy2time(int Year,int Doy) {
	const int doys[]={ 1,32,60,91,121,152,182,213,244,274,305,335 };

	ep[0]=Year; ep[3]=ep[4]=ep[5]=0; doy=Doy;
	/* get month and day */
	for (int i=0; i<12; i++) { 
		if (i==11||Doy<(doys[i+1]+(i>0&&Year%4==0?1:0))) {
			ep[1]=i+1;
			ep[2]=Doy-doys[i]-(i>1&&Year%4==0?1:0)+1;
			break;
		}
	}
	epoch2time(ep);
	return 1;
}

/* gtime_t to day of year ----------------------------------------------------------------------------
* convert time to day of year
* return : day of year (days)
*-------------------------------------------------------------------------------------------------- */
double gtime_t::time2doy(){
	double ep0[6]={0};
	gtime_t t0;

	time2epoch();
	ep0[0]=ep[0]; ep0[1]=ep0[2]=1.0; ep0[3]=ep0[4]=ep0[5]=0.0;
	return this->timediff(*t0.epoch2time(ep0))/86400.0+1.0;
}

/* read leap seconds table */
int gtime_t::read_leaps(const string file){
	int i,n;

	/* read leap seconds table by text or usno */
	if (!(n=read_leaps_text(file))&&!(n=read_leaps_usno(file))) {
		return 0;
	}

	for (i=0; i<7; i++) leaps[n][i]=0.0;

	return 1;
}
/* adjust time considering week handover ---------------------------------- */
gtime_t *gtime_t::adjweek(gtime_t t0) {
	double dt=timediff(t0);
	if (dt < -302400.0) return timeadd( 604800.0);
	if (dt >  302400.0) return timeadd(-604800.0);
	return this;
}
/* adjust time considering week handover ---------------------------------- */
gtime_t *gtime_t::adjday(gtime_t t0) {
	double dt=timediff(t0);
	if (dt < -43200.0) return timeadd( 86400.0);
	if (dt >  43200.0) return timeadd(-86400.0);
	return this;
}
/* screen by time ------------------------------------------------------------------------------------
* screening by time start, time end, and time interval
* args   : 
*		   gtime_t ts    I      time start (ts.time==0:no screening by ts)
*          gtime_t te    I      time end   (te.time==0:no screening by te)
*          double  tint  I      time interval (s) (0.0:no screen by tint)
* return : 1:on condition, 0:not on condition
*-------------------------------------------------------------------------------------------------- */
int gtime_t::screent(gtime_t ts, gtime_t te, double tint){
    return (tint<=0.0||fmod(time2gpst(NULL)+DTTOL,tint)<=DTTOL*2.0)&&
           (ts.time==0||timediff(ts)>=-DTTOL)&&
           (te.time==0||timediff(te)<  DTTOL);
}
/* read leap seconds table by text -------------------------------------------------------------------
* format : yyyy mm dd hh mm ss ls
--------------------------------------------------------------------------------------------------- */
int gtime_t::read_leaps_text(const string file){
	ifstream inf;
	string buff;
	int i,n=0,ls,fd;

	inf.open(file,ios::in);
	if (!inf.is_open()) return 0;

	while (getline(inf,buff)&&n<MAXLEAPS){
		if((fd=buff.find('#'))!=string::npos) buff.replace(fd,1,'\0');
		if (str2double(buff.substr(0,4),ep[0])==0) continue; /* year */
		for (i=0;i<5;i++)                                  /* mm dd hh mm ss */
			if(str2double(buff.substr(5+3*i,2),ep[i+1])==0) continue;
		if (str2int(buff.substr(20,2),ls)==0) continue;  /* leap second */
		for (i=0;i<6;i++) leaps[n][i]=ep[i];
		leaps[n++][6]=ls;
	}

	inf.close();
	return n;
}
/* read leap seconds table by usno ---------------------------------------------------------------- */
int gtime_t::read_leaps_usno(const string file){
	static const string months[]={
		"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"
	};
	ifstream inf;
	string buff,month;
	int i,j,y,m,d,n=0;
	double tai_utc,ls[MAXLEAPS][7]={ {0.0} };

	inf.open(file,ios::in);
	if (!inf.is_open()) return 0;

	while (getline(inf,buff)&&n<MAXLEAPS){
		if(str2int(buff.substr(0,4),y)==0) continue; /* year */
		/* month */
		for (m=0;m<12;m++) if (buff.find(months[m],5)!=string::npos) break;
		if (m>12) continue;
		if (str2int(buff.substr(9,2),d)==0) continue;     /* day */
		/* leap second */
		if ((j=buff.find("TAI-UTC="))==string::npos) continue;
		else str2double(buff.substr(j+8),tai_utc);
		ls[n][0]=y; ls[n][1]=m; ls[n][2]=d; ls[n++][6]=19.0-tai_utc;
	}
	for (i=0;i<n;i++) for (j=0;j<7;j++) leaps[i][j]=ls[n-i-1][j];

	inf.close();
	return n;
}
/* next download time ----------------------------------------------------------------------------- */
gtime_t *gtime_t::nextdltime(const int *topts,int stat){
	double tow;
	int week,tint;

	/* current time (gpst) */
	timeget()->utc2gpst();
	tow=time2gpst(&week);

	/* next retry time */
	if (stat==0&&topts[3]>0) {
		tow=(floor((tow-topts[2])/topts[3])+1.0)*topts[3]+topts[2];
		return gpst2time(week,tow);
	}

	/* next interval time */
	tint=topts[1]<=0 ? 3600 : topts[1];
	tow=(floor((tow-topts[2])/tint)+1.0)*tint+topts[2];
	gpst2time(week,tow);

	return this;
}

/* copy gtime_t ----------------------------------------------------------- */
gtime_t *gtime_t::copy_gtime(gtime_t t0) {
	time=t0.time;
	sec=t0.sec;
	sep=t0.sep;
	sys=t0.sys;
	for (int i=0; i<6; i++) ep[i]=t0.ep[i];

	return this;
}