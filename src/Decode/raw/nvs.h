/*------------------------------------------------------------------------------
* nvs.c : NVS receiver dependent functions
*
*    Copyright (C) 2012-2016 by M.BAVARO and T.TAKASU, All rights reserved.
*    Copyright (C) 2014 by T.TAKASU, All rights reserved.
*
*     [1] Description of BINR messages which is used by RC program for RINEX
*         files accumulation, NVS
*     [2] NAVIS Navis Standard Interface Protocol BINR, NVS
*
* version : $Revision:$ $Date:$
*-----------------------------------------------------------------------------*/
#ifndef NVS_H
#define NVS_H

#include "Decode/raw.h"

/* input NVS raw message from stream -------------------------------------------------------------- */
class nvs : public raw_t{
	/* Constructor */
	public:
		nvs();
		~nvs();
	/* Implementation functions */
	protected:
		/* adjust daily rollover of time ------------------------------------------ */
		gtime_t adjday(double ttt);
		/* decode ephemeris ------------------------------------------------------- */
		int decode_gpsephem(int sat);
		/* decode gloephem -------------------------------------------------------- */
		int decode_gloephem(int sat);

		/* decode NVS xf5-raw: raw measurement data ------------------------------- */
		int decode_xf5raw();
		/* decode NVS epehemerides in clear --------------------------------------- */
		int decode_xf7eph();
		/* decode NVS rxm-sfrb: subframe buffer ----------------------------------- */
		int decode_xe5bit();
		/* decode NVS x4aiono ----------------------------------------------------- */
		int decode_x4aiono();
		/* decode NVS x4btime ----------------------------------------------------- */
		int decode_x4btime();

		/* decode NVS raw message ------------------------------------------------- */
		int decode_nvs();

	public:
		/* input NVS raw message from stream -------------------------------------- */
		virtual int decode(unsigned char data);
};

#endif