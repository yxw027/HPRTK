/*------------------------------------------------------------------------------
* notvatel.c : NovAtel OEM6/OEM5/OEM4/OEM3 receiver functions
*
*          Copyright (C) 2007-2014 by T.TAKASU, All rights reserved.
*
* reference :
*     [1] NovAtel, OM-20000094 Rev6 OEMV Family Firmware Reference Manual, 2008
*     [2] NovAtel, OM-20000053 Rev2 MiLLennium GPSCard Software Versions 4.503
*         and 4.52 Command Descriptions Manual, 2001
*     [3] NovAtel, OM-20000129 Rev2 OEM6 Family Firmware Reference Manual, 2011
*     [4] NovAtel, OM-20000127 Rev1 OEMStar Firmware Reference Manual, 2009
*     [5] NovAtel, OM-20000129 Rev6 OEM6 Family Firmware Reference Manual, 2014
*
* version : $Revision: 1.2 $ $Date: 2008/07/14 00:05:05 $ ------------------- */
#ifndef NOVATEL_H
#define NOVATEL_H

#include "Decode/raw.h"

/* INPUT NovAtel data */
/* input omex raw data from stream ---------------------------------------------------------------- */
class oem_t : public raw_t{
	/* Constructor */
	public:
		oem_t();
		~oem_t();
	/* Implementation functions */
	protected:
		/* Novatel oem4 oem3 functions */
		/* extend sign ------------------------------------------------------------ */
		int exsign(unsigned int v,int bits);
		/* checksum --------------------------------------------------------------- */
		unsigned char chksum();
		/* adjust weekly rollover of gps time ------------------------------------- */
		gtime_t adjweek(gtime_t &time,double tow);
		/* get observation data index --------------------------------------------- */
		int obsindex(int sat);
		/* ura value (m) to ura index --------------------------------------------- */
		int uraindex(double value);
		/* decode rgeb ------------------------------------------------------------ */
		int decode_rgeb();
		/* decode rged ------------------------------------------------------------ */
		int decode_rged();
		/* decode repb ------------------------------------------------------------ */
		int decode_repb();
		/* decode frmb ------------------------------------------------------------ */
		int decode_frmb();
		/* decode ionb ------------------------------------------------------------ */
		int decode_ionb();
		/* decode utcb ------------------------------------------------------------ */
		int decode_utcb();
	public:
		/* input oem4 raw data from stream ---------------------------------------- */
		virtual int decode(unsigned char data);
	/* Components */
};

/* input oem4 raw data from stream ---------------------------------------------------------------- */
class oem4 : public oem_t{
	/* Constructor */
	public:
		oem4();
		~oem4();
	/* Implementation functions */
	protected:
		/* DECODE oem4 functions */
		/* decode oem4 tracking status -------------------------------------------- */
		int decode_trackstat(unsigned int stat,int &sys,int &code,int &track,
			int &plock,int &clock,int &parity,int &halfc);
		/* check code priority and return obs position ---------------------------- */
		int checkpri(string opt,int sys,int code,int freq);
		/* decode rangecmpb ------------------------------------------------------- */
		int decode_rangecmpb();
		/* decode rangeb ---------------------------------------------------------- */
		int decode_rangeb();
		/* decode rawephemb ------------------------------------------------------- */
		int decode_rawephemb();
		/* decode rawwaasframeb --------------------------------------------------- */
		int decode_rawwaasframeb();
		/* decode rawsbasframeb --------------------------------------------------- */
		int decode_rawsbasframeb();
		/* decode ionutcb --------------------------------------------------------- */
		int decode_ionutcb();
		/* decode gloephemerisb --------------------------------------------------- */
		int decode_gloephemerisb();
		/* decode qzss rawephemb -------------------------------------------------- */
		int decode_qzssrawephemb();
		/* decode qzss rawsubframeb ----------------------------------------------- */
		int decode_qzssrawsubframeb();
		/* decode qzssionutcb ----------------------------------------------------- */
		int decode_qzssionutcb();
		/* decode galephemerisb --------------------------------------------------- */
		int decode_galephemerisb();
		/* decode galalmanacb ----------------------------------------------------- */
		int decode_galalmanacb();
		/* decode galclockb ------------------------------------------------------- */
		int decode_galclockb();
		/* decode galionob -------------------------------------------------------- */
		int decode_galionob();
		/* decode galfnavrawpageb ------------------------------------------------- */
		int decode_galfnavrawpageb();
		/* decode galinavrawwordb ------------------------------------------------- */
		int decode_galinavrawwordb();
		/* decode rawcnavframeb --------------------------------------------------- */
		int decode_rawcnavframeb();
		/* decode bdsephemerisb --------------------------------------------------- */
		int decode_bdsephemerisb();
		/* crc-32 parity ---------------------------------------------------------- */
		unsigned int rtk_crc32();
		/* sync_oem4 header ------------------------------------------------------- */
		int sync_oem4(unsigned char data);
		/* decode oem4 message ---------------------------------------------------- */
		int decode_oem4();
	public:
		virtual int decode(unsigned char data);
	/* Components */
};

/* INPUT raw data */
/* input oem3 raw data from stream ---------------------------------------------------------------- */
class oem3 : public oem_t{
	/* Constructor */
	public:
		oem3();
		~oem3();
	/* Implementation functions */
	protected:
		/* sync_oem3 header ------------------------------------------------------- */
		int sync_oem3(unsigned char data);
		/* decode oem3 message ---------------------------------------------------- */
		int decode_oem3();
	public:
		/* input oem4 raw data from stream ---------------------------------------- */
		virtual int decode(unsigned char data);
	/* Components */
};

#endif