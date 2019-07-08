/*------------------------------------------------------------------------------
* septentrio.c : Septentrio Binary Format decoder (All Septentrio receivers)
*
*          Copyright (C) 2013 by Fabrizio Tappero.
*          Copyright (C) 2015 by Jens Reimann
*
* reference :
*     [1] Septentrio, SBF Reference Guide, Version 130722r38600, 07/2013
*
* note: - QZSS and Compass/Beidou is deactivated. The code is not tested. Use -DTESTING to activate.
*
* version : $Revision: 1.4 $ $Date: 2016/01/29 15:05:00 $
*-----------------------------------------------------------------------------*/
#ifndef SEPTENTRIO_H
#define SEPTENTRIO_H

#include "Decode/raw.h"

/* input sbf raw data from stream ----------------------------------------------------------------- */
class sbf : public raw_t {
	/* Constructor */
	public:
		sbf();
		~sbf();
	/* Implementation functions */
	protected:
		/* SBF checksum calculation ----------------------------------------------- */
		unsigned short sbf_checksum(unsigned char *bbb,int lll);
		/* 8-bit week -> full week ------------------------------------------------ */
		void adj_utcweek(gtime_t ttt,double *utc);
		/* adjust daily rollover of time ------------------------------------------ */
		gtime_t adjday(gtime_t ttt,double tddd);
		/* return frequency value in Hz from signal type name --------------------- */
		double getSigFreq(int _signType,int freqNo);
		/* adjust weekly rollover of gps time ------------------------------------- */
		gtime_t adjweek(gtime_t ttt,double tow);
		/* return the Septentrio signal type -------------------------------------- */
		int getSignalCode(int signType);
		/* return the signal type ------------------------------------------------- */
		int getFreqNo(int signType);
		/* sync to the beginning of a block --------------------------------------- */
		int sync_sbf(unsigned char data);

		/* decode SBF measurements message (observables) -------------------------- */
		int decode_measepoch();
		/* decode SBF nav message for GPS (navigation data) ----------------------- */
		int decode_gpsnav();
		/* decode SBF gpsion ------------------------------------------------------ */
		int decode_gpsion();
		/* decode SBF gpsutc ------------------------------------------------------ */
		int decode_gpsutc();
		/* decode SBF gpsalm ------------------------------------------------------ */
		int decode_gpsalm();
		/* decode SBF raw nav message (raw navigation data) ----------------------- */
		int decode_rawnav(int sys);
		/* decode SBF nav message for sbas (navigation data) ---------------------- */
		int decode_sbasnav();
		/* decode SBF raw nav message (raw navigation data) ----------------------- */
		int decode_georaw();


		/* decode SBF raw message ------------------------------------------------- */
		int decode_sbf();

	public:
		/* input sbf raw data from stream ----------------------------------------- */
		virtual int decode(unsigned char data);
};

#endif
