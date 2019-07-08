/*------------------------------------------------------------------------------
* javad.c : javad receiver dependent functions
*
*          Copyright (C) 2011-2014 by T.TAKASU, All rights reserved.
*
* reference :
*     [1] Javad GNSS, GREIS GNSS Receiver External Interface Specification,
*         Reflects Firmware Version 3.2.0, July 22, 2010
*     [2] Javad navigation systemms, GPS Receiver Interface Language (GRIL)
*         Reference Guide Rev 2.2, Reflects Firmware Version 2.6.0
*     [3] Javad GNSS, User visible changes in the firmware vesion 3.4.0 since
*         version 3.3.x (NEWS_3_4_0.txt)
*     [4] Javad GNSS, GREIS GNSS Receiver External Interface Specification,
*         Reflects Firmware Version 3.4.6, October 9, 2012
*     [5] Javad GNSS, GREIS GNSS Receiver External Interface Specification,
*         Reflects Firmware Version 3.5.4, January 30, 2014
*
* version : $Revision:$ $Date:$
*-----------------------------------------------------------------------------*/
#ifndef JAVAD_H
#define JAVAD_H

#include "Decode/raw.h"

/* input javad raw message from stream ------------------------------------------------------------ */
class javad : public raw_t{
	/* Constructor */
	public:
		javad();
		~javad();
	/* Implementation functions */
	protected:
		/* decode message length -------------------------------------------------- */
		int decodelen(const unsigned char *bb);
		/* test measurement data -------------------------------------------------- */
		int is_meas(char sig);
		/* convert signal to frequency and obs type ------------------------------- */
		int tofreq(char sig,int sys,int &type);
		/* check code priority and return obs position ---------------------------- */
		int checkpri(int sys,int code,int freq);
		/* glonass carrier frequency ---------------------------------------------- */
		double freq_glo(int freq,int freqn);
		/* checksum --------------------------------------------------------------- */
		int checksum();
		/* adjust weekly rollover of gps time ------------------------------------- */
		gtime_t adjweek(gtime_t ttt,double tow);
		/* adjust daily rollover of time ------------------------------------------ */
		gtime_t adjday(gtime_t ttt,double ttod);
		/* set time tag ----------------------------------------------------------- */
		int settag(int num);
		/* flush observation data buffer ------------------------------------------ */
		int flushobuf();
		/* sync javad message ----------------------------------------------------- */
		int sync_javad(unsigned char data);
		/* clear buffer ----------------------------------------------------------- */
		void clearbuff();

		/* decode [~~] receiver time -------------------------------------------------*/
		int decode_RT();
		/* decode [::] epoch time ----------------------------------------------------*/
		int decode_ET();
		/* decode [RD] receiver date -------------------------------------------------*/
		int decode_RD();
		/* decode [SI] satellite indices ---------------------------------------------*/
		int decode_SI();
		/* decode [NN] glonass satellite system numbers ------------------------------*/
		int decode_NN();
		/* decode [GA] gps almanac ---------------------------------------------------*/
		int decode_GA();
		/* decode [NA] glonass almanac -----------------------------------------------*/
		int decode_NA();
		/* decode [EA] galileo almanac -----------------------------------------------*/
		int decode_EA();
		/* decode [WA] waas almanac --------------------------------------------------*/
		int decode_WA();
		/* decode [QA] qzss almanac --------------------------------------------------*/
		int decode_QA();
		/* decode gps/galileo/qzss ephemeris -------------------------------------- */
		int decode_eph(int sys);
		/* decode [GE] gps ephemeris -------------------------------------------------*/
		int decode_GE();
		/* decode [NE] glonass ephemeris ---------------------------------------------*/
		int decode_NE();
		/* decode [EN] galileo ephemeris ---------------------------------------------*/
		int decode_EN();
		/* decode [WE] sbas ephemeris ------------------------------------------------*/
		int decode_WE();
		/* decode [QE] qzss ephemeris ------------------------------------------------*/
		int decode_QE();
		/* decode [CN] beidou ephemeris ----------------------------------------------*/
		int decode_CN();
		/* decode [UO] gps utc time parameters ---------------------------------------*/
		int decode_UO();
		/* decode [NU] glonass utc and gps time parameters ---------------------------*/
		int decode_NU();
		/* decode [EU] galileo utc and gps time parameters ---------------------------*/
		int decode_EU();
		/* decode [WU] waas utc time parameters --------------------------------------*/
		int decode_WU();
		/* decode [QU] qzss utc and gps time parameters ------------------------------*/
		int decode_QU();
		/* decode [IO] ionospheric parameters ----------------------------------------*/
		int decode_IO();
		/* decode L1 NAV data --------------------------------------------------------*/
		int decode_L1nav(unsigned char *buff,int len,int sat);
		/* decode raw L2C CNAV data --------------------------------------------------*/
		int decode_L2nav(unsigned char *buff,int len,int sat);
		/* decode raw L5 CNAV data ---------------------------------------------------*/
		int decode_L5nav(unsigned char *buff,int len,int sat);
		/* decode raw L1C CNAV2 data -------------------------------------------------*/
		int decode_L1Cnav(unsigned char *buff,int len,int sat);
		/* decode [*D] raw navigation data -------------------------------------------*/
		int decode_nD(int sys);
		/* decode [*d] raw navigation data -------------------------------------------*/
		int decode_nd(int sys);
		/* decode [LD] glonass raw navigation data -----------------------------------*/
		int decode_LD();
		/* decode [lD] glonass raw navigation data -----------------------------------*/
		int decode_lD();
		/* decode [WD] waas raw navigation data --------------------------------------*/
		int decode_WD();
		/* decode [R*] pseudoranges --------------------------------------------------*/
		int decode_Rx(char code);
		/* decode [r*] short pseudoranges --------------------------------------------*/
		int decode_rx(char code);
		/* decode [*R] relative pseudoranges -----------------------------------------*/
		int decode_xR(char code);
		/* decode [*r] short relative pseudoranges -----------------------------------*/
		int decode_xr(char code);
		/* decode [P*] carrier phases ------------------------------------------------*/
		int decode_Px(char code);
		/* decode [p*] short carrier phases ------------------------------------------*/
		int decode_px(char code);
		/* decode [*P] short relative carrier phases ---------------------------------*/
		int decode_xP(char code);
		/* decode [*p] short relative carrier phases ---------------------------------*/
		int decode_xp(char code);
		/* decode [D*] doppler -------------------------------------------------------*/
		int decode_Dx(char code);
		/* decode [*d] short relative doppler ----------------------------------------*/
		int decode_xd(char code);
		/* decode [E*] carrier to noise ratio ----------------------------------------*/
		int decode_Ex(char code);
		/* decode [*E] carrier to noise ratio x 4 ------------------------------------*/
		int decode_xE(char code);
		/* decode [F*] signal lock loop flags ----------------------------------------*/
		int decode_Fx(char code);
		/* decode [TC] CA/L1 continuous tracking time --------------------------------*/
		int decode_TC();

		/* decode javad raw message ----------------------------------------------- */
		int decode_javad();

	public:
		/* input javad raw message from stream ------------------------------------ */
		virtual int decode(unsigned char data);
};

#endif
