/*------------------------------------------------------------------------------
* crescent.c : hemisphere crescent/eclipse receiver dependent functions
*
*          Copyright (C) 2007-2014 by T.TAKASU, All rights reserved.
*
* reference :
*     [1] Hemisphere GPS, Grescent Integrator's Manual, December, 2005
*     [2] Hemisphere GPS, GPS Technical Reference, Part No. 875-0175-000,
*         Rev.D1, 2008
*     [3] Hemisphere GPS, Hemisphere GPS Technical Reference, 2014
*
* version : $Revision: 1.2 $ $Date: 2008/07/14 00:05:05 $
*-----------------------------------------------------------------------------*/
#ifndef CRESCENT_H
#define CRESCENT_H

#include "Decode/raw.h"

/* input cresent raw message ---------------------------------------------------------------------- */
class cres : public raw_t{
	/* Constructor */
	public:
		cres();
		~cres();
	/* Implementation functions */
	protected:
		/* checksum --------------------------------------------------------------- */
		int chksum(int len);
		/* decode bin 1 postion/velocity ------------------------------------------ */
		int decode_crespos();
		/* decode bin 96 raw phase and code --------------------------------------- */
		int decode_cresraw();
		/* decode bin 76 dual-freq raw phase and code ----------------------------- */
		int decode_cresraw2();
		/* decode bin 95 ephemeris ------------------------------------------------ */
		int decode_creseph();
		/* decode bin 94 ion/utc parameters --------------------------------------- */
		int decode_cresionutc();
		/* decode bin 80 waas messages -------------------------------------------- */
		int decode_creswaas();
		/* decode bin 66 glonass L1/L2 code and carrier phase --------------------- */
		int decode_cresgloraw();
		/* decode bin 65 glonass ephemeris ---------------------------------------- */
		int decode_cresgloeph();
		/* sync code -------------------------------------------------------------- */
		int sync_cres(unsigned char data);
		/* decode crescent raw message -------------------------------------------- */
		int decode_cres();
	public:
		/* input cresent raw message ---------------------------------------------- */
		virtual int decode(unsigned char data);
};

#endif