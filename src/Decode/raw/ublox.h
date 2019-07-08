/*------------------------------------------------------------------------------
* ublox.c : ublox receiver dependent functions
*
*          Copyright (C) 2007-2016 by T.TAKASU, All rights reserved.
*          Copyright (C) 2014 by T.SUZUKI, All rights reserved.
*
* reference :
*     [1] ublox-AG, GPS.G3-X-03002-D, ANTARIS Positioning Engine NMEA and UBX
*         Protocol Specification, Version 5.00, 2003
*     [2] ublox-AG, UBX-13003221-R03, u-blox M8 Receiver Description including
*         Protocol Specification V5, Dec 20, 2013
*     [3] ublox-AG, UBX-13003221-R07, u-blox M8 Receiver Description including
*         Protocol Specification V15.00-17.00, Nov 3, 2014
*     [4] ublox-AG, UBX-13003221-R09, u-blox 8 /u-blox M8 Receiver Description
*         including Protocol Specification V15.00-18.00, January, 2016
*
* version : $Revision: 1.2 $ $Date: 2008/07/14 00:05:05 $ ------------------- */

#ifndef UBLOX_H
#define UBLOX_H

#include "Decode/raw.h"

/* input ublox raw message from stream ------------------------------------------------------------ */
class ublox : public raw_t{
	/* Constructor */
	public:
		ublox();
		~ublox();
	/* Implementation functions */
	protected:
		/* checksum --------------------------------------------------------------- */
		int checksum();
		void setcs();
		/* ubx gnss indicator (ref [2] 25) ---------------------------------------- */
		int ubx_sys(int ind);
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

		/* decode navigation data */
		/* decode gps and qzss navigation data ------------------------------------ */
		int decode_nav(int sat,int offside);
		/* decode galileo navigation data ----------------------------------------- */
		int decode_enav(int sat,int offside);
		/* decode beidou navigation data ------------------------------------------ */
		int decode_cnav(int sat,int offside);
		/* decode glonass navigation data ----------------------------------------- */
		int decode_gnav(int sat,int offside,int frq);
		/* decode sbas navigation data -------------------------------------------- */
		int decode_snav(int sat,int offside);

		/* decode ubx-rxm-raw: raw measurement data ------------------------------- */
		int decode_rxmraw();
		/* decode ubx-rxm-rawx: multi-gnss raw measurement data (ref [3]) --------- */
		int decode_rxmrawx();
		/* decode ubx-rxm-sfrb: subframe buffer ----------------------------------- */
		int decode_rxmsfrb();
		/* decode ubx-rxm-sfrbx: raw subframe data (ref [3]) ---------------------- */
		int decode_rxmsfrbx();
		/* decode ubx-nav-sol: navigation solution -------------------------------- */
		int decode_navsol();
		/* decode ubx-nav-timegps: gps time solution ------------------------------ */
		int decode_navtime();
		/* decode ubx-trk-meas: trace measurement data ---------------------------- */
		int decode_trkmeas();
		/* decode ubx-trkd5: trace measurement data ------------------------------- */
		int decode_trkd5();
		/* decode ubx-trk-sfrbx: subframe buffer extension ------------------------ */
		int decode_trksfrbx();
		
		/* decode ublox raw message ----------------------------------------------- */
		int decode_ubx();
		/* sync code -------------------------------------------------------------- */
		int sync_ubx(unsigned char data);
	public:
		/* input ublox raw message from stream ------------------------------------ */
		virtual int decode(unsigned char data);
	/* Components */
};

#endif