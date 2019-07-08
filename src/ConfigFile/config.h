/* Class of Reading Positioning Configure file */
#ifndef CONFIG_H
#define CONFIG_H

#include "hprtk_lib.h"
#include "BaseFunction/timesys.h"
#include "GNSS/DataClass/data.h"
#include "RtkStream/stream.h"

/* struct: option type ---------------------------------------------------------------------------- */
typedef struct {						
	const string name;					/* option name */
	int format;							/* option format (0:int,1:double,2:string,3:enum) */
	void *var;							/* pointer to option variable */
	const string comment;				/* option comment/enum labels/unit */
} opt_t;

/* SNR mask type ---------------------------------------------------------------------------------- */
typedef struct {
	int ena[2];							/* enable flag {rover,base} */
	double mask[NFREQ][9];				/* mask (dBHz) at 5,10,...85 deg */
}snrmask_t;

/* post-processing option type -------------------------------------------------------------------- */
class pstopt_t {
	/* Constructor */
	public:
		pstopt_t();
		~pstopt_t();
	/* Components */  
	public:
		int predict;					/* flag of use predicted data */
		double time_inter;				/* time interval (s) */
		gtime_t time_start,time_end;	/* start and end time */
		string rover_obs;				/* rover observation file */
		string base_obs;				/* base observation file */
		string nav;						/* navigation file */
		string prseph;					/* precise ephemeris file */
		string satclk;					/* satellite clock file */
		int outflag[2];					/* out-put file open flag */
		string output[2];					/* out-put file */
};

/* rtk-processing option type --------------------------------------------------------------------- */
class rtkopt_t{
	/* Constructor */
	public:
		rtkopt_t();
		~rtkopt_t();
	/* Components */  
	public:
		int strtype[8];					/* stream types */
		string strpath[8];				/* stream paths */
		int strfmt[3];					/* stream formats */

		int svrcycle;					/* server cycle (ms) */
		int timeout;					/* timeout time (ms) */
		int reconnect;					/* reconnect interval (ms) */
		int nmeacycle;					/* nmea request cycle (ms) */
		int fswapmargin;				/* file swap margin (s) */
		int buffsize;					/* input buffer size (bytes) */
		int navmsgsel;					/* navigation message select */
		int nmeareq;					/* nmea request type (0:off,1:lat/lon,2:single) */
		double nmeapos[3];				/* nmea position (lat/lon) (deg) */
		string proxyaddr;				/* proxy address (1024) */

		/* unset options */
		string cmds[3];					/* stream start commands (256) */
		string rropts[3];				/* receiver and rtcm options (256) */				
		stream_t *monitor;				/* monitor stream */
};
/* processing options type ------------------------------------------------------------------------ */
class prcopt_t{
	/* Constructor */
	public:
		prcopt_t();
		~prcopt_t();
	/* Implementation functions */
	public:
	/* Components */
	public:
		int mode;						/* positioning mode (PMODE_???) */
		int soltype;					/* solution type (0:forward,1:backward,2:combined) */
		int nf;							/* number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) */
		int navsys;						/* navigation system */
		double elmin;					/* elevation mask angle (rad) */
		snrmask_t snrmask;				/* SNR mask */
		int sateph;						/* satellite ephemeris/clock (EPHOPT_???) */

		int modear;						/* AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,
										 * 4:ppp-ar) */
		int order;						/* order of polynomial fitting (L) to detect cycle slip (3-5) */
		int slipmode;					/* slip-detect mode (0:obs+1:poly+2:geo-free+4:TurboEdit) */
		double slip_std;				/* m, std variance of repaired cycle slip (1-10) */
		double ion_gf;					/* ionosphere change rate of geometry-free phase (m/s) */
		double sampling;				/* sec, observation time sampling */
		int restime;					/* sec, min unsolved time interval to reset ambiguity */
		int glomodear;					/* GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
		int bdsmodear;					/* BeiDou AR mode (0:off,1:on) */
		int iniamb;						/* set initial ambiguity if lock count < iniar */
		int maxariter;					/* max iteration to resolve ambiguity */	
		
		int sppiono;					/* SPP ionosphere option (no estimate option) */
		int ionoopt;					/* ionosphere option (IONOOPT_???) */
		int iondeg_n;					/* POLY ionosphere model: lat degrees */
		int iondeg_m;					/* POLY ionosphere model: lon degrees */
		int ion_nm;						/* POLY ionosphere model: max-sum of n+m */

		int spptrop;					/* SPP troposphere option (no estimate option) */
		int tropopt;					/* troposphere option (TROPOPT_???) */

		int dynamics;					/* dynamics model (0:none,1:velociy) */
		int tidecorr;					/* earth tide correction (0:off,1:solid,2:solid+otl+pole) */
		int adjustfunc;					/* adjustment function */					
		int niter;						/* number of filter iteration */
		int codesmooth;					/* code smoothing window size (0:none) */
		int intpref;					/* interpolate reference obs (for post mission) */
		int sbascorr;					/* SBAS correction options */
		int sbassatsel;					/* SBAS satellite selection (0:all) */
		int rovpos;						/* rover position for fixed mode */
		int refpos;						/* base position for relative mode */
										/* (0:pos in prcopt,  1:average of single pos, */
										/*  2:read from file, 3:rinex header, 4:rtcm pos) */
		double eratio[NFREQ];			/* code/phase error ratio */
		double err[4];					/* measurement error factor */
										/* [0]:phase error */
										/* [1]:elevation dependent phase error */
										/* [2]:baseline-length dependent phase error m/10km */
										/* [3]:doppler frequency (hz) */
		double std[3];					/* initial-state std [0]bias,[1]iono [2]trop */
		double stdrate[5];				/* growth rate of std [0]iono,[1]trop 
										 * [2]velh [3]velv */
		double sclkstab;				/* satellite clock stability (sec/sec) */
		double thresar[8];				/* AR validation threshold */
		double elmaskhold;				/* elevation mask to hold ambiguity (deg) */
		double maxtdiff;				/* max difference of time (sec) */
		double maxres;					/* max residual of code in SPP */
		double baseline[2];				/* baseline length constraint {const,sigma} (m) */

		string name[2];					/* rover and base name */
		double ru[3];					/* rover position for fixed mode {x,y,z} (ecef) (m) */
		double rb[3];					/* base position for relative mode {x,y,z} (ecef) (m) */
		string anttype[2];				/* antenna types {rover,base} */
		double antdel[2][3];			/* antenna delta {{rov_e,rov_n,rov_u},{ref_e,ref_n,ref_u}} */
		pcv_t pcvr[2];					/* receiver antenna parameters {rov,base} */
		unsigned char exsats[MAXSAT];	/* excluded satellites (1:excluded,2:included) */
		int  maxaveep;					/* max averaging epoches */
		int  initrst;					/* initialize by restart */
		string rnxopt[2];				/* rinex options {rover,base} */
		int  posopt[3];					/* positioning options */
		int  syncsol;					/* solution sync mode (0:off,1:on) */
		int freqopt;					/* disable L2-AR */
		string pppopt;					/* ppp option */
};

/* solution options type -------------------------------------------------------------------------- */
class solopt_t {  
	/* Constructors */
	public:
		solopt_t();
		~solopt_t();
	/* implementation functions */
	public:
		/* solution option to field separator ------------------------------------- */
		string opt2sep();
		/* write solution header to output stream --------------------------------- */
		int outsolheads(unsigned char *buff);
	/* Componentss */
	public:
		int posf;						/* solution format (SOLF_???) */
		int times;						/* time system (TIMES_???) */
		int timef;						/* time format (0:sssss.s,1:yyyy/mm/dd hh:mm:ss.s) */
		int timeu;						/* time digits under decimal point */
		int degf;						/* latitude/longitude format (0:ddd.ddd,1:ddd mm ss) */
		int outhead;					/* output header (0:no,1:yes) */
		int outopt;						/* output processing options (0:no,1:yes) */
		int origin;						/* origin of enu solution (0:base,1:original rover position) */
		int datum;						/* datum (0:WGS84,1:CGCS2000) */
		int height;						/* height (0:ellipsoidal,1:geodetic) */
		int geoid;						/* geoid model (0:EGM96,1:JGD2000) */
		int solstatic;					/* solution of static mode (0:all,1:single) */
		int sstat;						/* solution statistics level (0:off,1:states,2:residuals) */
		int trace;						/* debug trace level (0:off,1-5:debug) */
		double nmeaintv[2];				/* nmea output interval (s) (<0:no,0:all) */
										/* nmeaintv[0]:gprmc,gpgga,nmeaintv[1]:gpgsv */
		string sep;						/* field separator */
		string prog;					/* program name */
};
/* file options type ------------------------------------------------------------------------------ */
class filopt_t{
	/* Constructor */
	public:
		filopt_t();
		~filopt_t();
	/* Components */
	public:
		string satantp;					/* satellite antenna parameters file */
		string rcvantp;					/* receiver antenna parameters file */
		string stapos;					/* station positions file */
		string geoid;					/* external geoid data file */
		string iono;					/* ionosphere data file */
		string dcb;						/* dcb data file */
		string erp;						/* erp data file */
		string blq;						/* ocean loading tide blq file */
		string tempdir;					/* ftp/http temporaly directory */
		string geexe;					/* google earth exec file */
		string solstat;					/* solution statistics file */
		string test;					/* debug test file */
};
/* RINEX options type ----------------------------------------------------------------------------- */
class rnxopt_t{
	/* Constructor */
	public:
		rnxopt_t();
		~rnxopt_t();
	/* Components */
	public:
		gtime_t ts,te;					/* time start/end */
		double tint;					/* time interval (s) */
		double tunit;					/* time unit for multiple-session (s) */
		double rnxver;					/* RINEX version */
		int navsys;						/* navigation system */
		int obstype;					/* observation type */
		int freqtype;					/* frequency type */
		char mask[7][64];				/* code mask {GPS,GLO,GAL,QZS,SBS,CMP,IRN} */
		char staid[32];					/* station id for rinex file name */
		char prog[32];					/* program */
		char runby[32];					/* run-by */
		char marker[64];				/* marker name */
		char markerno[32];				/* marker number */
		char markertype[32];			/* marker type (ver.3) */
		char name[2][32];				/* observer/agency */
		char rec[3][32];				/* receiver #/type/vers */
		char ant[3][32];				/* antenna #/type */
		double apppos[3];				/* approx position x/y/z */
		double antdel[3];				/* antenna delta h/e/n */
		string comment[MAXCOMMENT];		/* comments */
		char rcvopt[256];				/* receiver dependent options */
		unsigned char exsats[MAXSAT];	/* excluded satellites */
		int scanobs;					/* scan obs types */
		int outiono;					/* output iono correction */
		int outtime;					/* output time system correction */
		int outleaps;					/* output leap seconds */
		int autopos;					/* auto approx position */
		int halfcyc;					/* half cycle correction */
		gtime_t tstart;					/* first obs time */
		gtime_t tend;					/* last obs time */
		gtime_t trtcm;					/* approx log start time for rtcm */
		string tobs[7][MAXOBSTYPE];		/* obs types {GPS,GLO,GAL,QZS,SBS,CMP,IRN} */
		int nobs[7];					/* number of obs types {GPS,GLO,GAL,QZS,SBS,CMP,IRN} */
};
/* all options ------------------------------------------------------------------------------------ */
class option_t{
	/* Constructor */
	public:
		option_t();
		~option_t();
	/* implementation functions */
	protected:
		/* reset system options to default ---------------------------------------- */
		void resetsysopts();
		/* string option to enum (int) -------------------------------------------- */
		int str2enum(const string str, const string comment, int *val);
		/* enum (int) to string option -------------------------------------------- */
		int enum2str(string &str,const string comment,int val);
		/* discard space characters at tail --------------------------------------- */
		void chop(string &str);
		/* search option ---------------------------------------------------------- */
		opt_t *searchopt(const string name,const opt_t *opts);
		/* string to option value ------------------------------------------------- */
		int str2opt(opt_t *opt,const string str);
		/* load options ----------------------------------------------------------- */
		int loadopts(const string file,opt_t *opts);
		/* system options buffer to options --------------------------------------- */
		void buff2sysopts();
		/* get system options ----------------------------------------------------- */
		void getsysopts();
	public:
		/* read post options ------------------------------------------------------ */
		int readpostopt(const string file);
		/* read rtk options ------------------------------------------------------- */
		int readrtkopt(const string file);
	/* Components */
	public:
		pstopt_t pstopt;
		rtkopt_t rtkopt;
		prcopt_t prcopt;
		solopt_t solopt[2];
		filopt_t filopt;
		rnxopt_t rnxopt;
};
#endif