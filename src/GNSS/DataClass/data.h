/* Data Classes File */
/*
 * 2017-7-25 class of observation data
 *
 *
 *
 *
 * 
 */
#ifndef DATA_H
#define DATA_H

#include "hprtk_lib.h"
#include "BaseFunction/timesys.h"

/* Data Structures -------------------------------------------------------------------------------- */
/* IGP band type ------------------------------------------------------------ */
typedef struct {        
	short x;							/* longitude/latitude (deg) */
	const short *y;						/* latitudes/longitudes (deg) */
	unsigned char bits;					/* IGP mask start bit */
	unsigned char bite;					/* IGP mask end bit */
} sbsigpband_t;

/* class of one epoch observation data ------------------------------------------------------------- */
class obsd_t {
	/* Constructors */
	public:
		obsd_t();
		~obsd_t();
	/* implementation functions */
	public:
		/* reset the whole data --------------------------------------------------- */
		void reset();
		/* reset satellite data --------------------------------------------------- */
		void satreset();
		/* update signal time use pseudorange-------------------------------------- */
		int sigtime_opsr();
		/* update signal time use satellite clock bias ---------------------------- */
		void sigtime_sclk();
	/* Components */
	public:
		unsigned char rcv;				/* receiver number */
		unsigned int sat;				/* satellite number */
		int prn,sys;					/* satellite prn number */
		gtime_t time;					/* receiver sampling time (GPST) */
		gtime_t sigtime;				/* satellite send signal time (GPST) */
		unsigned char SNR[NFREQ+NEXOBS]; /* signal strength (0.25BHz) */
		unsigned char LLI[NFREQ+NEXOBS]; /* loss of lock indicator */
		unsigned char code[NFREQ+NEXOBS];/* code indicator (CODE_???) */
		double L[NFREQ+NEXOBS];			/* observation data carrier-phase (cycle) */
		double P[NFREQ+NEXOBS];			/* observation data pseudorange (m) */
		float  D[NFREQ+NEXOBS];			/* observation data doppler frequency (Hz) */
		double res[NFREQ+NEXOBS];		/* observation residual */
		double ovar[NFREQ+NEXOBS];		/* observation variance */
		double dist;					/* distance between sat and rec */
		double sigvec[3];				/* line-of-sight vector */
		double dcbvar;					/* code bias error variance */
		int used;						/* observation used flag */
		int exc;						/* observation excluded flag */
		double posvel[6];				/* satellite position and velocity (ecef) */
		double dts[2];					/* satellite clock bias and drift */
		double azel[2];					/* satellite azimuth and elevation angles */
		double svar;					/* satellite position and clock variance */	
		int svh;						/* satellite health flag */
		double ionmap,dion,ionvar;		/* ionosphere delay mapping function, 
										 * correction and variance (GPS L1, m) */
		double dtro,trovar;				/* troposphere delay correction and variance */
		double dant[3];					/* antenna phase center correction */
		string errmsg;					/* error message */
};

/* station informtations -------------------------------------------------------------------------- */
class sta_t{        
  /* Constructors */
	public:
		sta_t();
		~sta_t();	
	/* Components */
	public:
		string name; 					/* marker name */
		string marker; 					/* marker number */
		string antdes; 					/* antenna descriptor */
		string antsno; 					/* antenna serial number */
		string rectype; 				/* receiver type descriptor */
		string recver; 					/* receiver firmware version */
		string recsno; 					/* receiver serial number */
		int antsetup;					/* antenna setup id */
		int itrf;						/* ITRF realization year */
		int deltype;					/* antenna delta type (0:enu,1:xyz) */
		double pos[3];					/* station position (ecef) (m) */
		double del[3];					/* antenna position delta (e/n/u or x/y/z) (m) */
		double hgt;						/* antenna height (m) */
};

/* class of station's information and observation chains ------------------------------------------ */
class obs_t {
	/* Constructors */
	public:
		obs_t();
		~obs_t();
	/* Implementaion functions */
	public:
		void reset();
	/* Components */
	public:
		int n;							/* observation number */
		unsigned char rcv;				/* receiver number */
		sta_t sta;						/* station parameter type */
		vector<obsd_t> data;			/* observation vector */
		string errmsg;					/* error massage */
		int used;
};

/* CMP/GPS/QZS/GAL broadcast ephemeris type ------------------------------------------------------- */
class eph_t{        		
	/* Constructors */
	public:
		eph_t();
		~eph_t();
	/* Components */
	public:
		/* parameters refer to the antenna phase center */
		int sat;						/* satellite number */
		int iode,iodc;					/* IODE,IODC */
		int sva;						/* SV accuracy (URA index) */
		int svh;						/* SV health (0:ok) */
		int week;						/* GPS/QZS: gps week, GAL: galileo week, BDS: BeiDou week */
		int code;						/* GPS/QZS: code on L2, GAL/CMP: data sources */
		int flag;						/* GPS/QZS: L2 P data flag, CMP: nav type */
		gtime_t toe,toc,ttr;			/* Toe,Toc,T_trans */
										/* SV orbit parameters */
		double A,e,i0,OMG0,omg,M0,deln,OMGd,idot;
		double crc,crs,cuc,cus,cic,cis;
		double toes;					/* Toe (s) in week */
		double fit;						/* fit interval (h) */
		double f0,f1,f2;				/* SV clock parameters (af0,af1,af2) */
		double tgd[4];					/* group delay parameters */
										/* GPS/QZS:tgd[0]=TGD */
										/* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 */
										/* CMP    :tgd[0]=BGD1,tgd[1]=BGD2 */
		double Adot,ndot;				/* Adot,ndot for CNAV */
};

/* GLONASS broadcast ephemeris type --------------------------------------------------------------- */
class geph_t{        
	/* Constructors */
	public:
		geph_t();
		~geph_t();
	/* Components */
	public:
		int sat;						/* satellite number */
		int iode;						/* IODE (0-6 bit of tb field) */
		int frq;						/* satellite frequency number */
		int svh,sva,age;				/* satellite health, accuracy, age of operation */
		gtime_t toe;					/* epoch of epherides (gpst) */
		gtime_t tof;					/* message frame time (gpst) */
		double pos[3];					/* satellite position (ecef) (m) */
		double vel[3];					/* satellite velocity (ecef) (m/s) */
		double acc[3];					/* satellite acceleration (ecef) (m/s^2) */
		double taun,gamn;				/* SV clock bias (s)/relative freq bias */
		double dtaun;					/* delay between L1 and L2 (s) */
};

/* SBAS ephemeris type ---------------------------------------------------------------------------- */
class seph_t{        
	/* Constructors */
	public:
		seph_t();
		~seph_t();
	/* Components */
	public:
		int sat;						/* satellite number */
		gtime_t t0;						/* reference epoch time (GPST) */
		gtime_t tof;					/* time of message frame (GPST) */
		int sva;						/* SV accuracy (URA index) */
		int svh;						/* SV health (0:ok) */
		double pos[3];					/* satellite position (m) (ecef) */
		double vel[3];					/* satellite velocity (m/s) (ecef) */
		double acc[3];					/* satellite acceleration (m/s^2) (ecef) */
		double af0,af1;					/* satellite clock-offset/drift (s,s/s) */
};

/* precise ephemeris type ------------------------------------------------------------------------- */
class peph_t{        
	/* Constructors */
	public:
		peph_t();
		~peph_t();
	/* Components */
	public:
		gtime_t time;					/* time (GPST) */
		double pos[MAXSAT][4];			/* satellite position/clock (ecef) (m|s) */
		float  std[MAXSAT][4];			/* satellite position/clock std (m|s) */
		double vel[MAXSAT][4];			/* satellite velocity/clk-rate (m/s|s/s) */
		float  vst[MAXSAT][4];			/* satellite velocity/clk-rate std (m/s|s/s) */
		float  cov[MAXSAT][3];			/* satellite position covariance (m^2) */
		float  vco[MAXSAT][3];			/* satellite velocity covariance (m^2) */
};

/* precise clock type ----------------------------------------------------------------------------- */
class pclk_t{        
	/* Constructors */
	public:
		pclk_t();
		~pclk_t();
	/* Components */
	public:
		gtime_t time;					/* time (GPST) */
		double clk[MAXSAT];			/* satellite clock (s) */
		float  std[MAXSAT];			/* satellite clock std (s) */
};

/* almanac type ----------------------------------------------------------------------------------- */
class alm_t{        
	/* Constructors */
	public:
		alm_t();
		~alm_t();
	/* Components */
	public:
		int sat;						/* satellite number */
		int svh;						/* sv health (0:ok) */
		int svconf;						/* as and sv config */
		int week;						/* GPS/QZS: gps week, GAL: galileo week */
		gtime_t toa;					/* Toa */
										/* SV orbit parameters */
		double A,e,i0,OMG0,omg,M0,OMGd;
		double toas;					/* Toa (s) in week */
		double f0,f1;					/* SV clock parameters (af0,af1) */
};

/* TEC grid type ---------------------------------------------------------------------------------- */
class tec_t{        
	/* Constructors */
	public:
		tec_t();
		~tec_t();
	/* Components */
	public:
		gtime_t time;					/* epoch time (GPST) */
		int ndata[3];					/* TEC grid data size {nlat,nlon,nhgt} */
		double rb;						/* earth radius (km) */
		double lats[3];					/* latitude start/interval (deg) */
		double lons[3];					/* longitude start/interval (deg) */
		double hgts[3];					/* heights start/interval (km) */
		vector<double> data;			/* TEC grid data (tecu) */
		vector<float> rms;				/* RMS values (tecu) */
};

/* satellite fcb data type ------------------------------------------------------------------------ */
class fcbd_t{        
	/* Constructors */
	public:
		fcbd_t();
		~fcbd_t();
	/* Components */
	public:
		gtime_t ts,te;					/* start/end time (GPST) */
		double bias[MAXSAT][3];			/* fcb value   (cyc) */
		double std [MAXSAT][3];			/* fcb std-dev (cyc) */
};

/* earth rotation parameter data type ------------------------------------------------------------- */
class erpd_t{        
  /* Constructors */
	public:
		erpd_t();
		~erpd_t();
	/* Components */
	public:
		double mjd;						/* mjd (days) */
		double xp,yp;					/* pole offset (rad) */
		double xpr,ypr;					/* pole offset rate (rad/day) */
		double ut1_utc;					/* ut1-utc (s) */
		double lod;						/* length of day (s/day) */
};

/* earth rotation parameter type ------------------------------------------------------------------ */
class erp_t{        
	/* Constructors */
	public:
		erp_t();
		~erp_t();
	/* Components */
	public:
		int n,nmax;						/* number and max number of data */
		vector<erpd_t> data;			/* earth rotation parameter data */
};

/* antenna parameter type ------------------------------------------------------------------------- */
class pcv_t{        
	/* Constructors */
	public:
		pcv_t();
		~pcv_t();
	/* Components */
	public:
		int sat;						/* satellite number (0:receiver) */
		string type;  					/* antenna type */
		string code;  					/* serial number or satellite code */
		gtime_t ts,te;					/* valid time start and end */
		double off[NFREQ][ 3];			/* phase center offset e/n/u or x/y/z (m) */
		double var[NFREQ][19];			/* phase center variation (m) */
										/* el=90,85,...,0 or nadir=0,1,2,3,... (deg) */
};
/* SBAS fast correction type ---------------------------------------------------------------------- */
class sbsfcorr_t{
	/* Constructors */
	public:
		sbsfcorr_t();
		~sbsfcorr_t();
	/* Components */
	public:
		gtime_t t0;						/* time of applicability (TOF) */
		double prc;						/* pseudorange correction (PRC) (m) */
		double rrc;						/* range-rate correction (RRC) (m/s) */
		double dt;						/* range-rate correction delta-time (s) */
		int iodf;						/* IODF (issue of date fast corr) */
		short udre;						/* UDRE+1 */
		short ai;						/* degradation factor indicator */
};

/* SBAS long term satellite error correction type ------------------------------------------------- */
class sbslcorr_t{        
	/* Constructors */
	public:
		sbslcorr_t();
		~sbslcorr_t();
	/* Components */
	public:
		gtime_t t0;						/* correction time */
		int iode;						/* IODE (issue of date ephemeris) */
		double dpos[3];					/* delta position (m) (ecef) */
		double dvel[3];					/* delta velocity (m/s) (ecef) */
		double daf0,daf1;				 /* delta clock-offset/drift (s,s/s) */
};

/* SBAS satellite correction type ----------------------------------------------------------------- */
class sbssatp_t{        
	/* Constructors */
	public:
		sbssatp_t();
		~sbssatp_t();
	/* Components */
	public:
		int sat;						/* satellite number */
		sbsfcorr_t fcorr;				/* fast correction */
		sbslcorr_t lcorr;				/* long term correction */
};

/* SBAS satellite corrections type ---------------------------------------------------------------- */
class sbssat_t{        
	/* Constructors */
	public:
		sbssat_t();
		~sbssat_t();
	/* Implementation functions */
	protected:
		/* long term correction --------------------------------------------------- */
		int sbslongcorr(obsd_t *data,double &dclk) const;
		/* fast correction -------------------------------------------------------- */
		int sbsfastcorr(obsd_t *data,double &prc) const;
	public:
		/* sbas satellite ephemeris and clock correction -------------------------- */
		int sbssatcorr(obsd_t *data) const;
	/* Components */
	public:
		int iodp;						/* IODP (issue of date mask) */
		int nsat;						/* number of satellites */
		int tlat;						/* system latency (s) */
		sbssatp_t sat[MAXSAT];			/* satellite correction */
};

/* SBAS ionospheric correction type --------------------------------------------------------------- */
class sbsigp_t{
	/* Constructors */
	public:
		sbsigp_t();
		~sbsigp_t();
	/* Components */
	public:
		gtime_t t0;						/* correction time */
		short lat,lon;					/* latitude/longitude (deg) */
		short give;						/* GIVI+1 */
		float delay;					/* vertical delay estimate (m) */
};

/* SBAS ionospheric corrections type -------------------------------------------------------------- */
class sbsion_t{        
	/* Constructors */
	public:
		sbsion_t();
		~sbsion_t();
	/* Components */
	public:
		int iodi;						/* IODI (issue of date ionos corr) */
		int nigp;						/* number of igps */
		sbsigp_t igp[MAXNIGP];			/* ionospheric correction */
};

/* DGPS/GNSS correction type ---------------------------------------------------------------------- */
class dgps_t{        
	/* Constructors */
	public:
		dgps_t();
		~dgps_t();
	/* Components */
	public:
		gtime_t t0;						/* correction time */
		double prc;						/* pseudorange correction (PRC) (m) */
		double rrc;						/* range rate correction (RRC) (m/s) */
		int iod;						/* issue of data (IOD) */
		double udre;					/* UDRE */
};

/* SSR correction type ---------------------------------------------------------------------------- */
class ssr_t{        
	/* Constructors */
	public:
		ssr_t();
		~ssr_t();
	/* Components */
	public:
		gtime_t t0[6];					/* epoch time (GPST) {eph,clk,hrclk,ura,bias,pbias} */
		double udi[6];					/* SSR update interval (s) */
		int iod[6];						/* iod ssr {eph,clk,hrclk,ura,bias,pbias} */
		int iode;						/* issue of data */
		int iodcrc;						/* issue of data crc for beidou/sbas */
		int ura;						/* URA indicator */
		int refd;						/* sat ref datum (0:ITRF,1:regional) */
		double deph [3];				/* delta orbit {radial,along,cross} (m) */
		double ddeph[3];				/* dot delta orbit {radial,along,cross} (m/s) */
		double dclk [3];				/* delta clock {c0,c1,c2} (m,m/s,m/s^2) */
		double hrclk;					/* high-rate clock corection (m) */
		float  cbias[MAXCODE];			/* code biases (m) */
		double pbias[MAXCODE];			/* phase biases (m) */
		float  stdpb[MAXCODE];			/* std-dev of phase biases (m) */
		double yaw_ang,yaw_rate;		/* yaw angle and yaw rate (deg,deg/s) */
		unsigned char update;			/* update flag (0:no update,1:update) */
};

/* QZSS LEX ephemeris type ------------------------------------------------------------------------ */
class lexeph_t{        
	/* Constructors */
	public:
		lexeph_t();
		~lexeph_t();
	/* Components */
	public:
		gtime_t toe;					/* epoch time (GPST) */
		gtime_t tof;					/* message frame time (GPST) */
		int sat;						/* satellite number */
		unsigned char health;			/* signal health (L1,L2,L1C,L5,LEX) */
		unsigned char ura;				/* URA index */
		double pos[3];					/* satellite position (m) */
		double vel[3];					/* satellite velocity (m/s) */
		double acc[3];					/* satellite acceleration (m/s2) */
		double jerk[3];					/* satellite jerk (m/s3) */
		double af0,af1;					/* satellite clock bias and drift (s,s/s) */
		double tgd;						/* TGD */
		double isc[8];					/* ISC */
};

/* QZSS LEX ionosphere correction type ------------------------------------------------------------ */
class lexion_t{        
	/* Constructors */
	public:
		lexion_t();
		~lexion_t();
	/* Components */
	public:
		gtime_t t0;						/* epoch time (GPST) */
		double tspan;					/* valid time span (s) */
		double pos0[2];					/* reference position {lat,lon} (rad) */
		double coef[3][2];				/* coefficients lat x lon (3 x 2) */
};

/* stec data type --------------------------------------------------------------------------------- */
class stec_t{        
	/* Constructors */
	public:
		stec_t();
		~stec_t();
	/* Components */
	public:
		gtime_t time;					/* time (GPST) */
		unsigned int sat;				/* satellite number */
		double ion;						/* slant ionos delay (m) */
		float std;						/* std-dev (m) */
		float azel[2];					/* azimuth/elevation (rad) */
		unsigned char flag;				/* fix flag */
};

/* trop data type --------------------------------------------------------------------------------- */
class trop_t{        
	/* Constructors */
	public:
		trop_t();
		~trop_t();
	/* Components */
	public:
		gtime_t time;					/* time (GPST) */
		double trp[3];					/* zenith tropos delay/gradient (m) */
		float std[3];					/* std-dev (m) */
};

/* ppp corrections type --------------------------------------------------------------------------- */
class pppcorr_t{        
	/* Constructors */
	public:
		pppcorr_t();
		~pppcorr_t();
	/* Components */
	public:
		int nsta;						/* number of stations */
		string stas[MAXSTA];			/* station names */
		double rr[MAXSTA][3];			/* station ecef positions (m) */
		int ns[MAXSTA],nsmax[MAXSTA];	/* number of stec data */
		int nt[MAXSTA],ntmax[MAXSTA];	/* number of trop data */
		stec_t *stec[MAXSTA];			/* stec data */
		trop_t *trop[MAXSTA];			/* trop data */
};

/* class of navigation data ----------------------------------------------------------------------- -*/
class nav_t{        		
	/* Constructors */
	public:
		nav_t();
		~nav_t();
	/* Components */
	public:
		int n,nmax;						/* number of broadcast ephemeris */
		int ng,ngmax;					/* number of glonass ephemeris */
		int ns,nsmax;					/* number of sbas ephemeris */
		int ne,nemax;					/* number of precise ephemeris */
		int nc,ncmax;					/* number of precise clock */
		int na,namax;					/* number of almanac data */
		int nt,ntmax;					/* number of tec grid data */
		int nf,nfmax;					/* number of satellite fcb data */
		vector<eph_t> eph;				/* GPS/QZS/GAL ephemeris */
		vector<geph_t> geph;			/* GLONASS ephemeris */
		vector<seph_t> seph;			/* SBAS ephemeris */
		vector<peph_t> peph;			/* precise ephemeris */
		vector<pclk_t> pclk;			/* precise clock */
		vector<alm_t> alm;				/* almanac data */
		vector<tec_t> tec;				/* tec grid data */
		vector<fcbd_t> fcb;			/* satellite fcb data */
		erp_t  erp;						/* earth rotation parameters */
		double ocean_par[2][6*11];		/* ocean tide loading parameters {rov,base} */
		double utc_gps[4];				/* GPS delta-UTC parameters {A0,A1,T,W} */
		double utc_glo[4];				/* GLONASS UTC GPS time parameters */
		double utc_gal[4];				/* Galileo UTC GPS time parameters */
		double utc_qzs[4];				/* QZS UTC GPS time parameters */
		double utc_cmp[4];				/* BeiDou UTC parameters */
		double utc_irn[4];				/* IRNSS UTC parameters */
		double utc_sbs[4];				/* SBAS UTC parameters */
		double ion_gps[8];				/* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
		double ion_gal[4];				/* Galileo iono model parameters {ai0,ai1,ai2,0} */
		double ion_qzs[8];				/* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
		double ion_cmp[8];				/* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
		double ion_irn[8];				/* IRNSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
		int leaps;						/* leap seconds (s) */
		double lam[MAXSAT][NFREQ];		/* carrier wave lengths (m) */
		double cbias[MAXSAT][3];		/* satellite dcb (0:p1-p2,1:p1-c1,2:p2-c2) (m) */
		double rbias[MAXRCV][2][3];		/* receiver dcb (0:p1-p2,1:p1-c1,2:p2-c2) (m) */
		double wlbias[MAXSAT];			/* wide-lane bias (cycle) */
		double glo_cpbias[4];			/* glonass code-phase bias {1C,1P,2C,2P} (m) */
		char glo_fcn[MAXPRNGLO+1];		/* glonass frequency channel number + 8 */
		pcv_t pcvs[MAXSAT];				/* satellite antenna pcv */
		sbssat_t sbssat;				/* SBAS satellite corrections */
		sbsion_t sbsion[MAXBAND+1];		/* SBAS ionosphere corrections */
		dgps_t dgps[MAXSAT];			/* DGPS corrections */
		ssr_t ssr[MAXSAT];				/* SSR corrections */
		lexeph_t lexeph[MAXSAT];		/* LEX ephemeris */
		lexion_t lexion;				/* LEX ionosphere correction */
		pppcorr_t pppcorr;				/* ppp corrections */
};

/* SBAS message type ------------------------------------------------------------------------------ */
class sbsmsg_t{
	/* Constructors */
public:
	sbsmsg_t();
	~sbsmsg_t();
	/* Implementation functions */
protected:
	/* decode half long term correction (vel code=0) --------------------------- */
	int decode_longcorr0(int p,sbssat_t *sbssat);
	/* decode half long term correction (vel code=1) --------------------------- */
	int decode_longcorr1(int p,sbssat_t *sbssat);
	/* decode half long term correction ---------------------------------------- */
	int decode_longcorrh(int p,sbssat_t *sbssat);
	/* decode type 1: prn masks ------------------------------------------------ */
	int decode_sbstype1(sbssat_t *sbssat);
	/* decode type 2-5,0: fast corrections ------------------------------------- */
	int decode_sbstype2(sbssat_t *sbssat);
	/* decode type 6: integrity info ------------------------------------------- */
	int decode_sbstype6(sbssat_t *sbssat);
	/* decode type 7: fast correction degradation factor ----------------------- */
	int decode_sbstype7(sbssat_t *sbssat);
	/* decode type 9: geo navigation message ----------------------------------- */
	int decode_sbstype9(nav_t *nav);
	/* decode type 18: ionospheric grid point masks ---------------------------- */
	int decode_sbstype18(sbsion_t *sbsion);
	/* decode type 24: mixed fast/long term correction ------------------------- */
	int decode_sbstype24(sbssat_t *sbssat);
	/* decode type 25: long term satellite error correction -------------------- */
	int decode_sbstype25(sbssat_t *sbssat);
	/* decode type 26: ionospheric deley corrections --------------------------- */
	int decode_sbstype26(sbsion_t *sbsion);
public:
	/* update sbas corrections to nav ------------------------------------------ */
	int sbsupdatecorr(nav_t *nav);
	/* Components */
public:
	int week,tow;					/* receiption time */
	int prn;						/* SBAS satellite PRN number */
	unsigned char msg[29];			/* SBAS message (226bit) padded by 0 */
};
/* SBAS messages type ----------------------------------------------------------------------------- */
class sbs_t{
	/* Constructors */
public:
	sbs_t();
	~sbs_t();
	/* Components */
public:
	int n,nmax;						/* number of SBAS messages/allocated */
	vector<sbsmsg_t> msgs;			/* SBAS messages */
};

/* QZSS LEX message type -------------------------------------------------------------------------- */
class lexmsg_t{
	/* Constructors */
public:
	lexmsg_t();
	~lexmsg_t();
	/* Implementation functions */
protected:
	/* decode tof and toe field (ref [1] 5.7.2.2.1.1) ----------------------------*/
	int decode_lextof(int i,gtime_t &tof,gtime_t &toe);
	/* decode signal health field (ref [1] 5.7.2.2.1.1) --------------------------*/
	int decode_lexhealth(int i,gtime_t tof,nav_t *nav);
	/* decode ephemeris and sv clock field (ref [1] 5.7.2.2.1.2) -----------------*/
	int decode_lexeph(int i,gtime_t toe,nav_t *nav);
	/* decode ionosphere correction field (ref [1] 5.7.2.2.1.3) ------------------*/
	int decode_lexion(int i,gtime_t tof,nav_t *nav);
	/* convert lex type 12 to rtcm ssr message -----------------------------------*/
	int lex2rtcm(int i,unsigned char *buff);
	/* decode type 10: ephemeris data and clock (ref [1] 5.7.2.2.1,1) ------- */
	int decode_lextype10(nav_t *nav,gtime_t tof);
	/* decode type 11: ephemeris data and clock (ref [1] 5.7.2.2.1,1) ------- */
	int decode_lextype11(nav_t *nav,gtime_t tof);
	/* decode type 12: madoca orbit and clock correction -------------------- */
	int decode_lextype12(nav_t *nav,gtime_t tof);
	/* decode type 20: gsi experiment message (ref [1] 5.7.2.2.2) ----------- */
	int decode_lextype20(nav_t *nav,gtime_t tof);
public:
	/* update lex corrections ----------------------------------------------- */
	int lexupdatecorr(nav_t *nav,gtime_t &tof);
	/* Components */
public:
	int prn;						/* satellite PRN number */
	int type;						/* message type */
	int alert;						/* alert flag */
	unsigned char stat;				/* signal tracking status */
	unsigned char snr;				/* signal C/N0 (0.25 dBHz) */
	unsigned int ttt;				/* tracking time (ms) */
	unsigned char msg[212];			/* LEX message data part 1695 bits */
};
/* QZSS LEX messages type ------------------------------------------------------------------------- */
class lex_t{/* Constructors */
public:
	lex_t();
	~lex_t();
	/* Components */
public:
	int n,nmax;						/* number of LEX messages and allocated */
	vector<lexmsg_t> msgs;			/* LEX messages */
};
#endif
