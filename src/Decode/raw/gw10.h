/*------------------------------------------------------------------------------
* gw10.c : furuno GW-10 receiver functions
*
*          Copyright (C) 2011-2012 by T.TAKASU, All rights reserved.
*
* reference :
*     [1] Furuno, SBAS/GPS receiver type GW-10 III manual, July 2004
*
* version : $Revision:$ $Date:$
*-----------------------------------------------------------------------------*/

#ifndef GW10_H
#define GW10_H

#include "Decode/raw.h"

/* input gw10 raw message ------------------------------------------------------------------------- */
class gw10 : public raw_t{
	/* Constructor */
	public:
		gw10();
		~gw10();
	/* Implementation functions */
	protected:
		/* compute checksum ------------------------------------------------------- */
		int chksum();
		/* adjust weekly rollover of gps time ------------------------------------- */
		int adjweek(double tow);
		/* bcd to number ---------------------------------------------------------- */
		int bcd2num(unsigned char bcd);
		/* check partity ---------------------------------------------------------- */
		int check_parity(unsigned int word,unsigned char *data);

		/* decode raw obs data ---------------------------------------------------- */
		int decode_gw10raw();
		/* decode gps message ----------------------------------------------------- */
		int decode_gw10gps();
		/* decode waas messages --------------------------------------------------- */
		int decode_gw10sbs();
		/* decode raw ephemereris ------------------------------------------------- */
		int decode_gw10reph();
		/* decode solution -------------------------------------------------------- */
		int decode_gw10sol();

		/* decode gw10 raw message ------------------------------------------------ */
		int decode_gw10();
	public:
		/* input gw10 raw message ------------------------------------------------- */
		virtual int decode(unsigned char data);
};

#endif