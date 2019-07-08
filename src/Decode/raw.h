#ifndef RAW_H
#define RAW_H
#include "Decode/decode.h"

/* Constant */
/* half-cycle correction list type ---------------------------------------------------------------- */
typedef struct{
	unsigned int sat;					/* satellite number */
	unsigned char freq;					/* frequency number (0:L1,1:L2,2:L5) */
	unsigned char valid;				/* half-cycle valid flag */
	char corr;							/* half-cycle corrected (x 0.5 cyc) */
	gtime_t ts,te;						/* time start, time end */
} half_cyc_t;

/* Class of decode navigation data for ------------------------------------------------------------ */
/* decode gps/qzss navigation data frame ---------------------------------------------------------- */
class decode_frame{
	/* Constructor */
	public:
		decode_frame();
		~decode_frame();
	/* Implementation functions */
	public:
		/* Base functions */
		/* decode gps/qzss almanac ------------------------------------------------ */
		void decode_almanac(const unsigned char *bbuff,int sat);
		/* decode gps navigation data subframe 4 ---------------------------------- */
		void decode_gps_subfrm4(const unsigned char *bbuff);
		/* decode qzss navigation data subframe 4/5 ------------------------------- */
		void decode_qzs_subfrm45(const unsigned char *bbuff);
		/* decode gps navigation data subframe 5 ---------------------------------- */
		void decode_gps_subfrm5(const unsigned char *bbuff);

		/* decode gps/qzss navigation data subframe 1 ----------------------------- */
		int decode_subfrm1(const unsigned char *bbuff);
		/* decode gps/qzss navigation data subframe 2 ----------------------------- */
		int decode_subfrm2(const unsigned char *bbuff);
		/* decode gps/qzss navigation data subframe 3 ----------------------------- */
		int decode_subfrm3(const unsigned char *bbuff);
		/* decode gps/qzss navigation data subframe 4 ----------------------------- */
		int decode_subfrm4(const unsigned char *bbuff);
		/* decode gps/qzss navigation data subframe 5 ----------------------------- */
		int decode_subfrm5(const unsigned char *bbuff);
		/* decode gps/qzss navigation data frame ---------------------------------- */
		int decode(const unsigned char *bbuff);
		/* Components */
	public:
		eph_t eph;
		vector<alm_t> alm;
		vector<double> ion;
		vector<double> utc;
		int leaps;
};

/* receiver raw data control type ----------------------------------------------------------------- */
class raw_t : public decode_data{
	/* Constructor */
	public:
		raw_t();
		virtual ~raw_t();

	/* implementation functions */
	protected:
		/* common decode functions */
		/* decode sbas message ---------------------------------------------------- */
		int sbsdecodemsg(int prn, unsigned int words[10]);
		/* decode Galileo I/NAV ephemeris ----------------------------------------- */
		int decode_gal_inav(const int sat,eph_t &eph);
		/* decode BeiDou D1 ephemeris --------------------------------------------- */
		int decode_bds_d1(const int sat,eph_t &eph);
		/* decode BeiDou D2 ephemeris --------------------------------------------- */
		int decode_bds_d2(const int sat,eph_t &eph);
		/* test hamming code of glonass ephemeris string -------------------------- */
		int test_glostr(const unsigned char *bbuff);
		/* decode glonass ephemeris strings --------------------------------------- */
		int decode_glostr(const int sat,geph_t &geph);

		/* INPUT raw data */
	public:
		/* input receiver raw data from stream ------------------------------------ */
		virtual int decode(unsigned char data);
	/* Components */
	public:
		gtime_t tobs;					/* observation data time */
		obs_t obuf;						/* observation data buffer */
		string msgtype;					/* last message type (256) */
		unsigned char subfrm[MAXSAT][380];/* subframe buffer (380) */
		double lockt[MAXSAT][NFREQ+NEXOBS]; /* lock time (s) */
		double icpp[MAXSAT],off[MAXSAT],icpc; /* carrier params for ss2 */
		double prCA[MAXSAT],dpCA[MAXSAT]; /* L1/CA pseudrange/doppler for javad */
		unsigned char halfc[MAXSAT][NFREQ+NEXOBS]; /* half-cycle add flag */
		char freqn[MAXOBS];				/* frequency number for javad */
		int nbyte;						/* number of bytes in message buffer */
		int len;						/* message length (bytes) */
		int iod;						/* issue of data */
		int tod;						/* time of day (ms) */
		int tbase;						/* time base (0:gpst,1:utc(usno),2:glonass,3:utc(su) */
		int flag;						/* general purpose flag */
		int outtype;					/* output message type */
		unsigned char buff[MAXRAWLEN];	/* message buffer (4096) */
		vector<half_cyc_t> half_cyc;	/* half-cycle correction list */

		int	dataindex;					/* index of { rov, base, corr } */
};

#endif
