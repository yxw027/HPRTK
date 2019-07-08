/*------------------------------------------------------------------------------
* binex.c : binex dependent functions
*
*          Copyright (C) 2013-2016 by T.TAKASU, All rights reserved.
*
* reference :
*     [1] UNAVCO, BINEX: Binary exchange format
*         (http://binex.unavco.org/binex.html)
*
* version : $Revision:$ $Date:$
*--------------------------------------------------------------------------- */
#ifndef BINEX_H
#define BINEX_H

#include "Decode/raw.h"

/* input binex message from stream ---------------------------------------------------------------- */
class binex : public raw_t{
	/* Constructor */
	public:
		binex();
		~binex();
	/* Implementation functions */
	protected:
		/* get binex 1-4 byte unsigned integer (big endian) ----------------------- */
		int getbnxi(unsigned char *p,unsigned int &val);
		/* checksum 8 parity (buff + 1)-------------------------------------------- */
		unsigned char csum8(int len);
		/* adjust weekly rollover of gps time ------------------------------------- */
		gtime_t adjweek(gtime_t ttt,double tow);
		/* adjust daily rollover of time ------------------------------------------ */
		gtime_t adjday(double ttod);
		/* ura value (m) to ura index --------------------------------------------- */
		int uraindex(double value);
		/* beidou signed 10 bit tgd -> sec ---------------------------------------- */
		double bds_tgd(int tgd);
		/* synchronize binex message ---------------------------------------------- */
		int sync_bnx(unsigned char data);

		/* decode binex mesaage 0x00-00: comment ---------------------------------- */
		int decode_bnx_00_00(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-01: program or software package -------------- */
		int decode_bnx_00_01(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-02: program operator ------------------------- */
		int decode_bnx_00_02(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-03: reserved --------------------------------- */
		int decode_bnx_00_03(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-04: site name/description -------------------- */
		int decode_bnx_00_04(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-05: site number ------------------------------ */
		int decode_bnx_00_05(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-06: monumnent name --------------------------- */
		int decode_bnx_00_06(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-07: monumnent number ------------------------- */
		int decode_bnx_00_07(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-08: marker name ------------------------------ */
		int decode_bnx_00_08(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-09: marker number ---------------------------- */
		int decode_bnx_00_09(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-0a: reference point name --------------------- */
		int decode_bnx_00_0a(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-0b: reference point number ------------------- */
		int decode_bnx_00_0b(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-0c: date esttablished ------------------------ */
		int decode_bnx_00_0c(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-0d: reserved --------------------------------- */
		int decode_bnx_00_0d(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-0e: reserved --------------------------------- */
		int decode_bnx_00_0e(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-0f: 4-character id --------------------------- */
		int decode_bnx_00_0f(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-10: project name ----------------------------- */
		int decode_bnx_00_10(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-11: principal investigator for this project -- */
		int decode_bnx_00_11(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-12: pi's agency/institution ------------------ */
		int decode_bnx_00_12(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-13: pi's contact information ----------------- */
		int decode_bnx_00_13(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-14: site operator ---------------------------- */
		int decode_bnx_00_14(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-15: site operator's agency/institution ------- */
		int decode_bnx_00_15(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-16: site operator's contact information ------ */
		int decode_bnx_00_16(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-17: antenna type ----------------------------- */
		int decode_bnx_00_17(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-18: antenna number --------------------------- */
		int decode_bnx_00_18(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-19: receiver type ---------------------------- */
		int decode_bnx_00_19(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-1a: receiver number -------------------------- */
		int decode_bnx_00_1a(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-1b: receiver firmware version ---------------- */
		int decode_bnx_00_1b(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-1c: antenna mount description ---------------- */
		int decode_bnx_00_1c(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-1d: antenna xyz position --------------------- */
		int decode_bnx_00_1d(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-1e: antenna geographic position -------------- */
		int decode_bnx_00_1e(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-1f: antenna offset from reference point ------ */
		int decode_bnx_00_1f(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-20: antenna radome type ---------------------- */
		int decode_bnx_00_20(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-21: antenna radome number -------------------- */
		int decode_bnx_00_21(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-22: geocode ---------------------------------- */
		int decode_bnx_00_22(unsigned char *bbb,int len);
		/* decode binex mesaage 0x00-7f: notes/additional information ------------- */
		int decode_bnx_00_7f(unsigned char *bbb,int len);
		/* decode binex mesaage 0x01-00: coded (raw bytes) gnss ephemeris --------- */
		int decode_bnx_01_00(unsigned char *bbb,int len);
		/* decode binex mesaage 0x02: generalized gnss data ----------------------- */
		int decode_bnx_02(unsigned char *bbb,int len);
		/* decode binex mesaage 0x03: generalized ancillary site data ------------- */
		int decode_bnx_03(unsigned char *bbb,int len);
		/* decode binex mesaage 0x7d: receiver internal state prototyping --------- */
		int decode_bnx_7d(unsigned char *bbb,int len);
		/* decode binex mesaage 0x7e: ancillary site data prototyping ------------- */
		int decode_bnx_7e(unsigned char *bbb,int len);
		/* decode binex mesaage 0x7f-00: jpl fiducial site ------------------------ */
		int decode_bnx_7f_00(unsigned char *bbb,int len);
		/* decode binex mesaage 0x7f-01: ucar cosmic ------------------------------ */
		int decode_bnx_7f_01(unsigned char *bbb,int len);
		/* decode binex mesaage 0x7f-02: trimble 4700 ----------------------------- */
		int decode_bnx_7f_02(unsigned char *bbb,int len);
		/* decode binex mesaage 0x7f-03: trimble netrs ---------------------------- */
		int decode_bnx_7f_03(unsigned char *bbb,int len);
		/* decode binex mesaage 0x7f-04: trimble netrs ---------------------------- */
		int decode_bnx_7f_04(unsigned char *bbb,int len);

		/* decode binex mesaage 0x00: site/monument/marker/ref point/setup metadata */
		int decode_bnx_00(unsigned char *bbb,int len);
		/* decode binex mesaage 0x01-01: decoded gps ephmemeris ------------------- */
		int decode_bnx_01_01(unsigned char *bbb,int len);
		/* decode binex mesaage 0x01-02: decoded glonass ephmemeris --------------- */
		int decode_bnx_01_02(unsigned char *bbb,int len);
		/* decode binex mesaage 0x01-03: decoded sbas ephmemeris ------------------ */
		int decode_bnx_01_03(unsigned char *bbb,int len);
		/* decode binex mesaage 0x01-04: decoded galileo ephmemeris --------------- */
		int decode_bnx_01_04(unsigned char *bbb,int len);
		/* decode binex mesaage 0x01-05: decoded beidou-2/compass ephmemeris ------ */
		int decode_bnx_01_05(unsigned char *bbb,int len);
		/* decode binex mesaage 0x01-06: decoded qzss ephmemeris ------------------ */
		int decode_bnx_01_06(unsigned char *bbb,int len);
		/* decode binex mesaage 0x01: gnss navigaion informtion ------------------- */
		int decode_bnx_01(unsigned char *bbb,int len);
		/* decode binex mesaage 0x7f-05: trimble netr8 obs data ------------------- */
		unsigned char *decode_bnx_7f_05_obs(unsigned char *bbb,
			int sat,int nobs,obsd_t &data);
		/* decode binex mesaage 0x7f-05: trimble netr8 ---------------------------- */
		int decode_bnx_7f_05(unsigned char *bbb,int len);
		/* decode binex mesaage 0x7f: gnss data prototyping ----------------------- */
		int decode_bnx_7f(unsigned char *bbb,int len);

		/* decode binex mesaage --------------------------------------------------- */
		int decode_bnx();

	public:
		/* input binex message from stream ---------------------------------------- */
		virtual int decode(unsigned char data);
};

#endif