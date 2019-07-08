/*------------------------------------------------------------------------------
* skytraq.c : skytraq receiver dependent functions
*
*          Copyright (C) 2009-2014 by T.TAKASU, All rights reserved.
*
* reference :
*     [1] Skytraq, Application Note AN0023 Binary Message of SkyTraq Venus 6
*         GPS Receiver, ver 1.4.8, August 21, 2008
*     [2] Skytraq, Application Note AN0024 Raw Measurement Binary Message
*         Extension of SkyTraq Venus 6 GPS Receiver, ver 0.5, October 9, 2009
*     [3] Skytraq, Application Note AN0024G2 Binary Message of SkyTraq Venus 7
*         GLONASS/GPS Receiver (Raw Measurement F/W), ver 1.4.26, April 26, 2012
*     [4] Skytraq, Application Note AN0030 Binary Message of Raw Measurement
*         Data Extension of SkyTraq Venus 8 GNSS Receiver, ver.1.4.29,
*         April 3, 2014
*     [5] Skytraq, Application Note AN0030 Binary Message of Raw Measurement
*         Data Extension of SkyTraq Venus 8 GNSS Receiver, ver.1.4.31,
*         August 12, 2014
*
* notes   :
*     The byte order of S1315F raw message is big-endian inconsistent to [1].
*
* version : $Revision:$
*-----------------------------------------------------------------------------*/

#ifndef SKYTRAQ_H
#define SKYTRAQ_H

#include "Decode/raw.h"

/* input skytraq raw message from stream ---------------------------------------------------------- */
class skyq : public raw_t{
	/* Constructor */
	public:
		skyq();
		~skyq();
	/* Implementation functions */
	protected:
		/* checksum --------------------------------------------------------------- */
		unsigned char checksum(unsigned char *buff, int len);
		/* 8-bit week -> full week ------------------------------------------------ */
		void adj_utcweek(double *utc);
		/* save subframe ---------------------------------------------------------- */
		int save_subfrm(int sat);
		/* decode ephemeris ------------------------------------------------------- */
		int decode_ephem(int sat);
		/* decode almanac and ion/utc --------------------------------------------- */
		int decode_alm1(int sat);
		/* decode almanac --------------------------------------------------------- */
		int decode_alm2(int sat);

		/* decode skytraq measurement epoch (0xDC) -------------------------------- */
		int decode_stqtime();
		/* decode skytraq raw measurement (0xDD) ---------------------------------- */
		int decode_stqraw();
		/* decode gps/qzss subframe (0xE0) ---------------------------------------- */
		int decode_stqgps();
		/* decode glonass string (0xE1) ------------------------------------------- */
		int decode_stqglo();
		/* decode glonass string (requested) (0x5C) ------------------------------- */
		int decode_stqgloe();
		/* decode beidou subframe (0xE2,0xE3) ------------------------------------- */
		int decode_stqbds();

		/* decode skytraq message ------------------------------------------------- */
		int decode_stq();
		/* sync code -------------------------------------------------------------- */
		int sync_stq(unsigned char data);
	public:
		/* input superstar 2 raw message from stream ------------------------------ */
		virtual int decode(unsigned char data);
};

#endif