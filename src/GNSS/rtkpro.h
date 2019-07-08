#ifndef RTKPRO_H
#define RTKPRO_H

#include "hprtk_lib.h"

#include "Decode/decode.h"
#include "ConfigFile/config.h"
#include "RtkStream/stream.h"
#include "GNSS/DataClass/data.h"

#include "GNSS/EphModel/satellite.h"
#include "GNSS/AntModel/antenna.h"
#include "GNSS/TidModel/tide.h"
#include "GNSS/IonModel/ionosphere.h"
#include "GNSS/TroModel/troposphere.h"
#include "GNSS/AdjModel/adjustment.h"
#include "GNSS/ParModel/parameter.h"
#include "GNSS/AmbModel/ambiguity.h"

/* solution type ---------------------------------------------------------------------------------- */
class sol_t{
	/* Constructor */
	public:
		sol_t();
		sol_t(const rtk_t *rtk);
		~sol_t();
		/* Implementation functions */
	protected:
		/* dynamic covariance to ecef covariance ---------------------------------- */
		void dyc2ecef();
		/* ecef solution ---------------------------------------------------------- */
		void ecef(const solopt_t *opt);
		/* ecef position to LLH --------------------------------------------------- */
		void llh(const solopt_t *opt);
		/* ecef position to ENU --------------------------------------------------- */
		void enu(const solopt_t *opt,rtk_t *rtk);
		/* ecef position to EMEA -------------------------------------------------- */
		void nmea(const solopt_t *opt);
	public:
		/* solution position ------------------------------------------------------ */
		string forposvel(const solopt_t *opt, rtk_t *rtk);
		/* solution time ---------------------------------------------------------- */
		string fortime(const solopt_t *opt);
	/* Components */
	public:
		gtime_t time;					/* time (GPST) */
		unsigned char type;				/* type (0:xyz-ecef,1:enu-baseline) */
		unsigned char stat;				/* solution status (SOLQ_???) */
		unsigned int ns;				/* number of valid satellites */
		float age;						/* age of differential (s) */
		float ratio;					/* AR ratio factor for valiation */
		float thres;					/* AR ratio threshold for valiation */

		/* parameters (only for position mode higher than SPP) */
		int NF,NL;						/* number of used frequency and observation */
		int ND,NI,NT,NG,NC,NA;			/* number of each kind of parameters */
		vector<double> 				/* parameter vectors */
			xdyc,xion,xtro,xglo,xclk,xamb;
										/* dynamic parameters */
										/* ionosphere parameters */
										/* troposphere parameters */
										/* GLO receiver differenced IFB rate (only for relative) */
										/* receiver clock parameters (only for ppp) */
										/* ambiguity parameters */
		vector<double>
			vdyc,vion,vtro,vglo,vclk,vamb;
										/* covariance of parameters */



		/* formated solution (data and string) */
		gtime_t soltime;				/* solution time */
		string  strtime;

		double posvel[6];				/* position and velocity (m|m/s or deg|deg/s) */
		double posvar[9];				/* position covariance (m^2 or deg^2) */
		double ecefvar[9];
		double lat[3];					/* latitude (ddd mm ss) */
		double lon[3];					/* longitude (ddd mm ss) */
		string strpv;
};

/* satellite status type -------------------------------------------------------------------------- */
class ssat_t{
	/* Constructor */
	public:
		ssat_t();
		~ssat_t();
	/* Implemetaion functions */
	protected:
		/* set coefficients matrix for estimate of gf-slip ------------------------ */
		void coe_gfslip(const int flag,const double *lam,
			vector<double> &An,vector<double> &Rn);
		/* estimate cycle slip of gf ---------------------------------------------- */
		void est_gfslip(const double DN12,const double DN15,const int f12, const int f15,
		const double *lam);
		/* detect cycle slip functions -------------------------------------------- */
		/* detect by LLI ---------------------------------------------------------- */
		void detslip_LLI(const obsd_t *rov,const obsd_t *bas,const prcopt_t *opt);
		/* detect by polynomial fitting ------------------------------------------- */
		void detslip_poly(const prcopt_t *opt,const double *lam);
		/* detect by geometry-free combination ------------------------------------ */
		void detslip_gf(const prcopt_t *opt,const double *lam);
		/* detect by Melbourne-Wubbena -------------------------------------------- */
		void detslip_MW(const prcopt_t *opt);
	public:
		/* initialize vectors with order of polynomial fitting -------------------- */
		void init_vector(prcopt_t *opt);
		/* update vectors for current status -------------------------------------- */
		void update_vector(const obsd_t *rov,const obsd_t *bas,
			const double *lam,const prcopt_t *opt);
		/* reset flag according to unsolved time interval ------------------------- */
		void test_reset(const prcopt_t *opt);
		/* reset_ambiguity -------------------------------------------------------- */
		void reset_amb(const prcopt_t *opt,const int freq);
		/* detect cycle slip ------------------------------------------------------ */
		void detect_slip(const obsd_t *rov,const obsd_t *bas,
			const prcopt_t *opt,const double *lam);
		/* repair cycle slip (not used) ------------------------------------------- */
		void repair_slip(const double *lam,const prcopt_t *opt);
		/* update ambiguity parameters -------------------------------------------- */
		void update_amb(const double *lam,const prcopt_t *opt);

	/* Components */
	public:
		string errmsg;
		/* state parameters */
		unsigned char sys;				/* navigation system */
		unsigned int sat;				/* satellite number */
		string id;						/* satellite id */
		unsigned char vs;				/* valid satellite flag single */
		double azel[2];					/* azimuth/elevation angles {az,el} (rad) */
		double resp[NFREQ];				/* residuals of pseudorange (m) */
		double resc[NFREQ];				/* residuals of carrier-phase (m) */
		unsigned char vsat[NFREQ];		/* valid satellite flag */
		unsigned char snr[NFREQ];		/* signal strength (0.25 dBHz) */

		/* observation/solution parameters vector (size = prcopt_t->order) */
		double           max_sumdt;		/* max sum_dt */
		double           sum_dt;		/* sum of dtime */
		int              polyodr;		/* polymonial fitting order (size of vector) */
		vector<gtime_t> obstime;		/* observtaion time */
		vector<double>  dtime;			/* time difference */
		vector<double>  L[NFREQ];		/* phase observation (cycle) */
		vector<double>  P[NFREQ];		/* code observation (m) */
		vector<double>  D[NFREQ];		/* doppler frequency */
		double gf12[2],gf15[2];			/* geometry-free phase L1-L2/L1-L5 (m) (0:last,1:new) */
		double lc12[2],pc12[2];			/* ionosphere-free phase/code L1-L2 (m) (0:last,1:new) */
		double mw12[2],mw15[2];			/* Melbourne-Wubbena L1-L2/L1-L5 (n) (0:average,1:new) */
		double nl12[2];					/* not used narrow-lane L1+L2 (n) (0:average,1:new) */

		/* ionosphere parameters */
		double ion_delay;				/* vertical ionosphere delay of GPS L1 (m) */
		double ion_var;					/* variance of ion_var */
		vector<double> d_ion[NFREQ];	/* ionosphere delay of each frequency */

		/* ambiguity parameters */
		vector<unsigned char> slip[NFREQ];
						
										/* cycle-slip flag vector
										* freq(f+1) slip flag
										* (bit7-6: rov LLI,   bit5-4: bas LLI,
										* bit2   : repaired
										* bit1   : half slip, bit0  : slip) */
		double        dslip[2][NFREQ];	/* estimated slip value to be repaired
										 * [0] estimated by polynomial fitting (L:1,2,5)
										 * [1] estimated by gf combination (L:1,2,5) */
		gtime_t       ambtime[NFREQ];	/* time of ambiguity solution */
		gtime_t       ambfirst[NFREQ];	/* time of first ambiguity solution */
		int           fix[NFREQ];		/* ambiguity fix flag (1:initial,2:fix,3:hold) */
		double        amb[NFREQ];		/* ambiguity solution vector */
		double        ambvar[NFREQ];	/* ambiguity solution variance vector */
		double        amb_ave[NFREQ];	/* average ambiguity vector since first lock */
		double        d_ave[NFREQ+2];	/* current correction of average amb (L1,2,3 and LC,mw12) */
		double        lcamb,lcvar;		/* LC ambiguity, variance */
		double        lcamb_ave,plc;	/* average LC ambiguity, weight */
		double        phw;				/* phase windup (cycle) */
		int           sol_flag[NFREQ];	/* slove ambiguity flag */
		int           reset[NFREQ];		/* reset ambiguity flag */
		double        lock[NFREQ];		/* lock time of phase */
		int           lock_con[NFREQ];	/* lock count of phase */
		int           lock_LC;			/* lock count of LC phase */
		int           MW12_con,MW15_con;	/* lock count of MW combination */
		unsigned char half[NFREQ];		/* half-cycle valid flag */
		unsigned int  slipc[NFREQ];		/* cycle-slip counter */	

		/* state file */
		fstream *state_file;			/* amb test file */
};

/* RTK control/result type ------------------------------------------------------------------------ */
class rtk_t{
	/* Constructor */
	public:
		rtk_t();
		virtual ~rtk_t();
	/* Implementation functions */
	protected:
	public:
		/* initialize rtk control ------------------------------------------------- */
		void rtkinit();
		/* rtk-position function (virtual) ---------------------------------------- */
		virtual int basepos();
		/* rtk-position function (virtual) ---------------------------------------- */
		virtual int rtkpos();
	/* Components */
	public:
		vector<sol_t>  sol;			/* RTK solution */
		vector<sol_t> b_sol;			/* RTK base solution */
		double rb[6];					/* base position/velocity (ecef) (m|m/s) */
		double tt;						/* time difference between current and previous (s) */
		int nfix;						/* number of continuous fixes of ambiguity */
		vector<ssat_t> ssat;			/* satellite status */
		int neb;						/* bytes in error message buffer */
		string errbuf;					/* error message buffer */
		prcopt_t *opt;					/* processing options */
		string msg;						/* error message */
		/* position classes */
		obs_t *obsr,*obsb;				/* observation data (rover, base) */
		nav_t *nav;						/* navigation data */
		sol_t *solp;					/* sol_t pointer */
		obs_t *obsp;					/* obs_t pointer */

		/* parameters */
		int NF,numF;					/* number of used frequency and computed frequency */
		int ND,NT,NG,NC,NI;				/* number of each kind of parameters */
		int NX;							/* number of all parameters except ION and ambiguity */
		int NXI;						/* number of all parameters except ambiguity */
		int N_ALL;						/* number of all parameters with all satellite ambiguity */
		vector<double> Rx_ALL;			/* covariance of all parameters */
		gtime_t ambtime;				/* time of last ambiguity solution */


		/* function classes */
		parafunc_t parafunc;			/* pararmeter number function */
		satantenna_t satantfunc;		/* satellite antenna functions (for satfuc) */
		recantenna_t recantfunc;		/* receiver antenna functions */
		satellite_t *satfunc;			/* satellite function class point */
		tidecorr_t tidefunc;			/* tidal displacement correction functions */
		ioncorr_t *sppionf;				/* SPP ionosphere delay functions (no estimate) */
		trocorr_t *spptrof;				/* SPP troposphere delay functions (no estimate) */
		ioncorr_t *ionfunc;				/* ionosphere delay functions */
		trocorr_t *trofunc;				/* troposphere delay functions */
		adjfunc_t *adjfunc;				/* adjustment functions */

		/* state file */
		fstream state_file;				/* amb out put file */
};

/* RTK server type -------------------------------------------------------------------------------- */
class rtksvr_t{
	/* Constructor */
	public:
		rtksvr_t();
		~rtksvr_t();
	/* Implementation functions */
	protected:
		/* initialzie function ---------------------------------------------------- */
		/* set sat\rcv antenna information ---------------------------------------- */
		void setpcv(inatx_t *atx,int satrcv);
		/* new rtk according to opt ----------------------------------------------- */
		void inirtk(prcopt_t *Prcopt,filopt_t *Filopt);
		/* initialize stream environment ------------------------------------------ */
		void strinitcom();
		/* initialize decode format ----------------------------------------------- */
		void inidecode();
		/* initialize stream type ------------------------------------------------- */
		void inistream();	

		/* process function ------------------------------------------------------- */
		/* sync input streams (if type=STR_FILE) ---------------------------------- */
		void strsync();
		/* write solution header to output stream --------------------------------- */
		void writesolhead();
		/* update navigation data ------------------------------------------------- */
		void updatenav();
		/* update glonass frequency channel number in raw data struct ------------- */
		void updatefcn();
		/* write solution to each out-stream (stream[3:4])------------------------- */
		void writesolstr(int index);
	public:
		/* initialize observation pointer obsr/obsb (*rtk) ------------------------ */
		int iniobs();
		/* lock/unlock rtk server ------------------------------------------------- */
		void rtksvrlock();
		void rtksvrunlock();
		/* write solution to output stream ---------------------------------------- */
		void writesol();
		/* input message from stream ---------------------------------------------- */
		/* update rtk server struct ----------------------------------------------- */
		void updatesvr(int ret,int index);
		/* decode receiver raw/rtcm data ------------------------------------------ */
		int decoderaw(int index);
		/* initialize rtksvr ------------------------------------------------------ */
		int rtksvrini(option_t *option);
		/* start rtksvr ----------------------------------------------------------- */
		int rtksvrstart();
		/* stop rtksvr ------------------------------------------------------------ */
		void rtksvrstop(char **cmds);

	/* Components */
	public:
		int state;						/* server state (0:stop,1:running) */
		int sampling;					/* observation sampling time */
		int cyctime;					/* processing cycle (ms) */
		int nmeacycle;					/* NMEA request cycle (ms) (0:no req) */
		int nmeareq;					/* NMEA request (0:no,1:nmeapos,2:single sol) */
		double nmeapos[3];				/* NMEA request position (ecef) (m) */
		int buffsize;					/* input buffer size (bytes) */
		int format[3];					/* input format {rov,base,corr} */
		solopt_t solopt[2];				/* output solution options {sol1,sol2} */
		int navsel;						/* ephemeris select (0:all,1:rover,2:base,3:corr) */
		int nsbs;						/* number of sbas message */
		int nsol;						/* number of solution buffer */
		rtk_t *rtk;						/* RTK control/result struct */
		int nb[3];						/* bytes in input buffers {rov,base} */
		int nsb[2];						/* bytes in soulution buffers */
		int npb[3];						/* bytes in input peek buffers */
		unsigned char *buff[3];			/* input buffers {rov,base,corr} */
		unsigned char *sbuf[2];			/* output buffers {sol1,sol2} */
		unsigned char *pbuf[3];			/* peek buffers {rov,base,corr} */
		unsigned int nmsg[3][10];		/* input message counts */
		decode_data *data[3];			/* un-decoded data (raw,rtcm2,rtcm3) for {rov,base,corr} */
		gtime_t ftime[3];				/* download time {rov,base,corr} */
		string files[3];				/* download paths {rov,base,corr} */
		obs_t obs[3];					/* observation data {rov,base,corr} (give to rtk) */
		nav_t *nav;						/* navigation data */
		sbsmsg_t sbsmsg[MAXSBSMSG];		/* SBAS message buffer */
		int strtype[8];					/* stream types {rov,base,corr,sol1,sol2,logr,logb,logc} */
		stream_t *stream[8];			/* streams {rov,base,corr,sol1,sol2,logr,logb,logc} */
		stream_t *moni;					/* monitor stream */
		unsigned int tick;				/* start tick */
		thread_t thread;				/* server thread */
		int cputime;					/* CPU time (ms) for a processing cycle */
		int prcout;						/* missing observation data count */
		int nave;						/* number of averaging base pos */
		double rb_ave[3];				/* averaging base pos */
		string cmds_periodic[3];		/* periodic commands */
		lock_t lock;					/* lock flag */
		int fobs[3];					/* observation buff number */
		string errmsg;					/* error message */
};

#endif