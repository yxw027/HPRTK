#ifndef RTCM_H
#define RTCM_H
#include "Decode/decode.h"

typedef struct {                    /* multi-signal-message header type */
	unsigned char iod;              /* issue of data station */
	unsigned char time_s;           /* cumulative session transmitting time */
	unsigned char clk_str;          /* clock steering indicator */
	unsigned char clk_ext;          /* external clock indicator */
	unsigned char smooth;           /* divergence free smoothing indicator */
	unsigned char tint_s;           /* soothing interval */
	unsigned char nsat,nsig;        /* number of satellites/signals */
	unsigned int  sats[64];         /* satellites */
	unsigned char sigs[32];         /* signals */
	unsigned char cellmask[64];     /* cell mask */
} msm_h_t;

/* RTCM control struct type ----------------------------------------------------------------------- */
class rtcm_t : public decode_data{
	/* Constructor */
	public:
		rtcm_t();
		virtual ~rtcm_t();
	/* Implementation functions */
	protected:
		/* get observation data index --------------------------------------------- */
		int obsindex(gtime_t time,int sat);
	public:
		virtual int decode(unsigned char data);
	/* Components */
	public:
		int staid;						/* station id */
		int stah;						/* station health */
		int seqno;						/* sequence number for rtcm 2 or iods msm */
		int outtype;					/* output message type */
		gtime_t time_s;					/* message start time */
		sta_t sta;						/* station parameters */
		string msg;						/* special message (128) */
		string msgtype;					/* last message type (256) */
		string msmtype[6];				/* msm signal types (128) */
		int obsflag;					/* obs data complete flag (1:ok,0:not complete) */
		double cp[MAXSAT][NFREQ+NEXOBS]; /* carrier-phase measurement */
		unsigned short lock[MAXSAT][NFREQ+NEXOBS]; /* lock time */
		unsigned short loss[MAXSAT][NFREQ+NEXOBS]; /* loss of lock count */
		gtime_t lltime[MAXSAT][NFREQ+NEXOBS]; /* last lock time */
		int nbyte;						/* number of bytes in message buffer */
		int nbit;						/* number of bits in word buffer */
		int len;						/* message length (bytes) */
		unsigned char buff[1200];		/* message buffer (1200) */
		unsigned int word;				/* word buffer for rtcm 2 */
		unsigned int nmsg2[100];		/* message count of RTCM 2 (1-99:1-99,0:other) */
		unsigned int nmsg3[400];		/* message count of RTCM 3
										(1-299:1001-1299,300-399:2000-2099,0:ohter) */
};

/* RTCM 2 control struct type --------------------------------------------------------------------- */
class rtcm_2 : public rtcm_t{
	/* Constructor */
	public:
		rtcm_2();
		~rtcm_2();
	/* Implementation functions */
	/* RTCM version 2 ------------------------------------------------------------- */
	protected:
		/* decode type 1/9: differential gps correction/partial correction set ---- */
		int decode_type1();
		/* decode type 3: reference station parameter ----------------------------- */
		int decode_type3();
		/* decode type 14: gps time of week --------------------------------------- */
		int decode_type14();
		/* decode type 16: gps special message ------------------------------------ */
		int decode_type16();
		/* decode type 17: gps ephemerides ---------------------------------------- */
		int decode_type17();
		/* decode type 18: rtk uncorrected carrier-phase -------------------------- */
		int decode_type18();
		/* decode type 19: rtk uncorrected pseudorange ---------------------------- */
		int decode_type19();
		/* decode type 22: extended reference station parameter ------------------- */
		int decode_type22();
		/* decode type 23: antenna type definition record ------------------------- */
		int decode_type23();
		/* decode type 24: antenna reference point (arp) -------------------------- */
		int decode_type24();
		/* decode type 31: differential glonass correction ------------------------ */
		int decode_type31();
		/* decode type 32: differential glonass reference station parameters ------ */
		int decode_type32();
		/* decode type 34: glonass partial differential correction set ------------ */
		int decode_type34();
		/* decode type 36: glonass special message -------------------------------- */
		int decode_type36();
		/* decode type 37: gnss system time offset -------------------------------- */
		int decode_type37();
		/* decode type 59: proprietary message ------------------------------------ */
		int decode_type59();
		/* adjust hourly rollover of rtcm 2 time ---------------------------------- */
		void adjhour(double zcnt);
		/* decode rtcm ver.2 message ---------------------------------------------- */
		int decode_rtcm2();
		/* input rtcm 2 message from stream --------------------------------------- */
	public:
		virtual int decode(unsigned char data);
};

/* RTCM 3 control struct type --------------------------------------------------------------------- */
class rtcm_3 : public rtcm_t{
	/* Constructor */
	public:
		rtcm_3();
		~rtcm_3();
	/* Implementation functions */
	/* RTCM version 3 ------------------------------------------------------------- */
	protected:
		/* get sign-magnitude bits ------------------------------------------------ */
		double getbitg(int pos,int len);
		/* adjust daily rollover of glonass time ---------------------------------- */
		void adjday_glot(double tod);
		/* adjust weekly rollover of gps time ------------------------------------- */
		void adjweek(double tow);
		/* adjust weekly rollover of bdt time ------------------------------------- */
		int adjbdtweek(int week,double sec);
		/* adjust carrier-phase rollover ------------------------------------------ */
		double adjcp(int sat,int freq,double cccp);
		/* loss-of-lock indicator ------------------------------------------------- */
		int lossoflock(int sat,int freq,int lllock);
		/* s/n ratio -------------------------------------------------------------- */
		unsigned char snratio(double snr);
		/* test station id consistency -------------------------------------------- */
		int test_staid(int staid);
		/* get signed 38bit field ------------------------------------------------- */
		double getbits_38(const unsigned char *buff,int pos);


		/* decode type 1001-1004 message header ----------------------------------- */
		int decode_head1001(int &sync);
		/* decode type 1001: L1-only gps rtk observation -------------------------- */
		int decode_type1001();
		/* decode type 1002: extended L1-only gps rtk observables ----------------- */
		int decode_type1002();
		/* decode type 1003: L1&L2 gps rtk observables ---------------------------- */
		int decode_type1003();
		/* decode type 1004: extended L1&L2 gps rtk observables ------------------- */
		int decode_type1004();
		/* decode type 1005: stationary rtk reference station arp ----------------- */
		int decode_type1005();
		/* decode type 1006: stationary rtk reference station arp with height ----- */
		int decode_type1006();
		/* decode type 1007: antenna descriptor ----------------------------------- */
		int decode_type1007();
		/* decode type 1008: antenna descriptor & serial number ------------------- */
		int decode_type1008();
		/* decode type 1009-1012 message header ----------------------------------- */
		int decode_head1009(int &sync);
		/* decode type 1009: L1-only glonass rtk observables ---------------------- */
		int decode_type1009();
		/* decode type 1010: extended L1-only glonass rtk observables ------------- */
		int decode_type1010();
		/* decode type 1011: L1&L2 glonass rtk observables ------------------------ */
		int decode_type1011();
		/* decode type 1012: extended L1&L2 glonass rtk observables --------------- */
		int decode_type1012();
		/* decode type 1013: system parameters ------------------------------------ */
		int decode_type1013();
		/* decode type 1019: gps ephemerides -------------------------------------- */
		int decode_type1019();
		/* decode type 1020: glonass ephemerides ---------------------------------- */
		int decode_type1020();
		/* decode type 1021: helmert/abridged molodenski -------------------------- */
		int decode_type1021();
		/* decode type 1022: moledenski-badekas transfromation -------------------- */
		int decode_type1022();
		/* decode type 1023: residual, ellipoidal grid representation ------------- */
		int decode_type1023();
		/* decode type 1024: residual, plane grid representation ------------------ */
		int decode_type1024();
		/* decode type 1025: projection (types except LCC2SP,OM) ------------------ */
		int decode_type1025();
		/* decode type 1026: projection (LCC2SP - lambert conic conformal (2sp)) -- */
		int decode_type1026();
		/* decode type 1027: projection (type OM - oblique mercator) -------------- */
		int decode_type1027();
		/* decode type 1030: network rtk residual --------------------------------- */
		int decode_type1030();
		/* decode type 1031: glonass network rtk residual ------------------------- */
		int decode_type1031();
		/* decode type 1032: physical reference station position information ------ */
		int decode_type1032();
		/* decode type 1033: receiver and antenna descriptor ---------------------- */
		int decode_type1033();
		/* decode type 1034: gps network fkp gradient ----------------------------- */
		int decode_type1034();
		/* decode type 1035: glonass network fkp gradient ------------------------- */
		int decode_type1035();
		/* decode type 1037: glonass network rtk ionospheric correction difference  */
		int decode_type1037();
		/* decode type 1038: glonass network rtk geometic correction difference --- */
		int decode_type1038();
		/* decode type 1039: glonass network rtk combined correction difference --- */
		int decode_type1039();
		/* decode type 1044: qzss ephemerides (ref [15]) -------------------------- */
		int decode_type1044();
		/* decode type 1045: galileo satellite ephemerides (ref [15]) ------------- */
		int decode_type1045();
		/* decode type 1046: galileo satellite ephemerides (extension for IGS MGEX) */
		int decode_type1046();
		/* decode type 1047: beidou ephemerides (tentative mt and format) --------- */
		int decode_type1042();
		/* decode type 63: beidou ephemerides (rtcm draft) ------------------------ */
		int decode_type63();
		/* decode ssr 1,4 message header ------------------------------------------ */
		int decode_ssr1_head(int sys,int &sync,int &iod,double &udint,int &refd,int &hsize);
		/* decode ssr 2,3,5,6 message header -------------------------------------- */
		int decode_ssr2_head(int sys,int &sync,int &iod,double &udint,int &hsize);
		/* decode ssr 7 message header -------------------------------------------- */
		int decode_ssr7_head(int sys,int &sync,int &iod,double &udint,int &dispe,int &mw,int &hsize);
		/* decode ssr 1: orbit corrections ---------------------------------------- */
		int decode_ssr1(int sys);
		/* decode ssr 2: clock corrections ---------------------------------------- */
		int decode_ssr2(int sys);
		/* decode ssr 3: satellite code biases ------------------------------------ */
		int decode_ssr3(int sys);
		/* decode ssr 4: combined orbit and clock corrections --------------------- */
		int decode_ssr4(int sys);
		/* decode ssr 5: ura ------------------------------------------------------ */
		int decode_ssr5(int sys);
		/* decode ssr 6: high rate clock correction ------------------------------- */
		int decode_ssr6(int sys);
		/* decode ssr 7: phase bias ----------------------------------------------- */
		int decode_ssr7(int sys);
		/* get signal index ------------------------------------------------------- */
		void sigindex(int sys,const unsigned char *code,const int *freq,int n,int *ind);
		/* save obs data in msm message ------------------------------------------- */
		void save_msm_obs(int sys,msm_h_t *h,const double *r,const double *pr,const double *ccp,
			const double *rr,const double *rrf,const double *cnr,const int *llock,const int *ex,
			const int *half);
		/* decode type msm message header ----------------------------------------- */
		int decode_msm_head(int sys,int &sync,int &iod,msm_h_t *h,int &hsize);
		/* decode unsupported msm message ----------------------------------------- */
		int decode_msm0(int sys);
		/* decode msm 4: full pseudorange and phaserange plus cnr ----------------- */
		int decode_msm4(int sys);
		/* decode msm 5: full pseudorange, phaserange, phaserangerate and cnr ----- */
		int decode_msm5(int sys);
		/* decode msm 6: full pseudorange and phaserange plus cnr (high-res) ------ */
		int decode_msm6(int sys);
		/* decode msm 7: full pseudorange, phaserange, phaserangerate and cnr (h-res)*/
		int decode_msm7(int sys);
		/* decode type 1230: glonass L1 and L2 code-phase biases ------------------ */
		int decode_type1230();
		/* decode rtcm ver.3 message ---------------------------------------------- */
		int decode_rtcm3();
	public:
		/* input rtcm 3 message from stream --------------------------------------- */
		virtual int decode(unsigned char data);
};

#endif
