/*------------------------------------------------------------------------------
* ss2.c : superstar II receiver dependent functions
*
*          Copyright (C) 2007-2013 by T.TAKASU, All rights reserved.
*
* reference:
*     [1] NovAtel, OM-20000086 Superstar II Firmware Reference Manuall, 2005
*
* version : $Revision: 1.2 $ $Date: 2008/07/14 00:05:05 $
*-----------------------------------------------------------------------------*/

#ifndef SUPERSTAR2_H
#define SUPERSTAR2_H

#include "Decode/raw.h"

/* input superstar 2 raw message from stream ------------------------------------------------------ */
class ss2 : public raw_t{
	/* Constructor */
	public:
		ss2();
		~ss2();
	/* Implementation functions */
	protected:
		/* checksum --------------------------------------------------------------- */
		int chksum();
		/* adjust week ------------------------------------------------------------ */
		int adjweek(double sec);
		/* decode id#20 navigation data (user) ------------------------------------ */
		int decode_ss2llh();
		/* decode id#21 navigation data (ecef) ------------------------------------ */
		int decode_ss2ecef();
		/* decode id#23 measurement block ----------------------------------------- */
		int decode_ss2meas();
		/* decode id#22 ephemeris data -------------------------------------------- */
		int decode_ss2eph();
		/* decode id#67 sbas data ------------------------------------------------- */
		int decode_ss2sbas();
		/* sync code -------------------------------------------------------------- */
		int sync_ss2(unsigned char data);
		/* decode superstar 2 raw message ----------------------------------------- */
		int decode_ss2();
	public:
		/* input superstar 2 raw message from stream ------------------------------ */
		virtual int decode(unsigned char data);
};

#endif

