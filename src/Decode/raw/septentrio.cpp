#include "septentrio.h"

#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */

static const char rcsid[]="$Id: Septentrio SBF,v 1.1 2016/02/11 FT $";

static unsigned char locktime[255][32];

/* SBF definitions Version 2.9.1 */
#define SBF_SYNC1   0x24        /* SBF message header sync field 1 (correspond to $) */
#define SBF_SYNC2   0x40        /* SBF message header sync field 2 (correspont to @)*/

#define ID_MEASEPOCH       4027 /* SBF message id: range measurememts */
#define ID_MEASEPOCHEXTRA  4000 /* SBF message id: range measurememts extra info */
#define ID_MEASEPOCH_END   5922 /* SBF message id: end of SBF range measurememts */

#define ID_GPSRAWCA     4017    /* SBF message id: GPS raw navigation page or frame */
#define ID_GPSRAWL2C    4018    /* SBF message id: GPS raw navigation page or frame */
#define ID_GPSRAWL5     4019    /* SBF message id: GPS raw navigation page or frame */
#define ID_GEORAWL1     4020    /* SBF message id: SBAS raw navigation page or frame */
#define ID_GEORAWL5     4021    /* SBF message id: SBAS raw navigation page or frame */
#define ID_GALRAWFNAV   4022    /* SBF message id: Galileo raw navigation page or frame */
#define ID_GALRAWINAV   4023    /* SBF message id: Galileo raw navigation page or frame */
#define ID_GALRAWCNAV   4024    /* SBF message id: Galileo raw navigation page or frame */
#define ID_GLORAWCA     4026    /* SBF message id: GLONASS raw navigation page or frame */
#define ID_CMPRAW       4047    /* SBF message id: Compass raw navigation page or frame */
#define ID_QZSSL1CA     4066    /* SBF message id: QZSS raw navigation page or frame */
#define ID_QZSSL2C      4067    /* SBF message id: QZSS raw navigation page or frame */
#define ID_QZSSL5       4068    /* SBF message id: QZSS raw navigation page or frame */
#define ID_IRNSSRAW     4093    /* SBF message id: IRNSS raw navigation page or frame */

#define ID_GEONAV                 5896 /* SBF message id:  SBAS navigation message */
#define ID_GEOALM                 5897 /* SBF message id:  SBAS satellite almanac */
#define ID_GEOSERVICELEVEL        5917 /* SBF message id:  SBAS Service Message */
#define ID_GEONETWORKTIME         5918 /* SBF message id:  SBAS Network Time/UTC offset parameters */
#define ID_GEOMT00                5925 /* SBF message id:  SBAS: Don't use for safety application */
#define ID_GEOPRNMASK             5926 /* SBF message id:  PRN Mask assignments */
#define ID_GEOFASTCORR            5927 /* SBF message id:  Fast Corrections */
#define ID_GEOINTEGRITY           5928 /* SBF message id:  Integrity information */
#define ID_GEOFASTCORRDEGR        5929 /* SBF message id:  fast correction degradation factor */
#define ID_GEODEGRFACTORS         5930 /* SBF message id:  Degration factors */
#define ID_GEOIGPMASK             5931 /* SBF message id:  Ionospheric grid point mask */
#define ID_GEOLONGTERMCOR         5932 /* SBF message id:  Long term satellite error corrections */
#define ID_GEOIONODELAY           5933 /* SBF message id:  Inospheric delay correction */
#define ID_GEOCLOCKEPHCOVMATRIX   5934 /* SBF message id:  Clock-Ephemeris Covariance Matrix l*/


#define ID_GPSNAV   5891        /* SBF message id: GPS navigation data */
#define ID_GPSALM   5892        /* SBF message id: GPS almanac */
#define ID_GPSION   5893        /* SBF message id: GPS ionosphere data, Klobuchar coefficients */
#define ID_GPSUTC   5894        /* SBF message id: GPS UTC data */

#define ID_GLONAV   4004        /* SBF message id: GLONASS navigation data */
#define ID_GLOALM   4005        /* SBF message id: GLONASS almanac */
#define ID_GLOTIME  4036        /* SBF message id: GLONASS time data */

#define ID_GALNAV   4002        /* SBF message id: Galileo navigation data */
#define ID_GALALM   4003        /* SBF message id: Galileo almanac */
#define ID_GALION   4030        /* SBF message id: Galileo ionosphere data, Klobuchar coefficients */
#define ID_GALUTC   4031        /* SBF message id: Galileo UTC data */

#define ID_CMPNAV   4081        /* SBF message id: Compass navigation data */
#define ID_QZSSNAV  4095        /* SBF message id: QZSS navigation data */

#define ID_GALGSTGPS  4032      /* SBF message id: Galileo GPS time offset */

#define ID_PVTCART    4006      /* SBF message id: Rx Position Velocity and 
								Time data in Cartesian coordinates in m */
#define ID_PVTGEOD    4007      /* SBF message id: Rx Position Velocity and 
								Time data in Geodetic coordinates */
#define ID_DOP        4001      /* SBF message id: Dilution of Precision data */
#define ID_PVTSATCART 4008      /* SBF message id: Satellite Position Velocity and Time data */

#define ID_ENDOFPVT     5921    /* SBF message id: End of any PVT block */

#define ID_RXTIME       5914    /* SBF message id: Receiver time data */

#define ID_DIFFCORRIN   5919    /* SBF message id: incoming RTCM2 or RTCM3 or CMR message */

#define ID_BASESTATION  5949    /* SBF message id: Base station position */

#define ID_CHNSTATUS   4013     /* SBF message id: Status of the receiver channels */
#define ID_RXSTATUS    4014     /* SBF message id: Status of the receiver */
#define ID_RXSETUP     5902     /* SBF message id: Status of the receiver */
#define ID_COMMENT     5936     /* SBF message id: Status of the receiver */

#define ID_SATVISIBILITY  4012  /* SBF message id: Ssatellites visibility */
#define ID_BBSMPS         4040  /* SBF message id: series of successive Rx baseband samples */

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))
static unsigned short U2(unsigned char *p) { unsigned short u; memcpy(&u,p,2); return u; }
static unsigned int   U4(unsigned char *p) { unsigned int   u; memcpy(&u,p,4); return u; }
static float          R4(unsigned char *p) { float          r; memcpy(&r,p,4); return r; }
static double         R8(unsigned char *p) { double         r; memcpy(&r,p,8); return r; }
static signed int     I4(unsigned char *p) { signed int     u; memcpy(&u,p,4); return u; }
static short          I2(unsigned char *p) { short          i; memcpy(&i,p,2); return i; }

/* checksum lookup table -----------------------------------------------------*/
static const unsigned int CRC_16CCIT_LookUp[256] ={
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };

/* input sbf raw data from stream --------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
sbf::sbf(){
}
sbf::~sbf(){
}
/* SBF checksum calculation ----------------------------------------------- */
unsigned short sbf::sbf_checksum(unsigned char *bbb,int lll){
	int i;
	unsigned short crc = 0;
	for (i=0; i<lll; i++) {
		crc = (crc << 8) ^ CRC_16CCIT_LookUp[(crc >> 8) ^ bbb[i]];
	}
	return crc;
}
/* 8-bit week -> full week ------------------------------------------------ */
void sbf::adj_utcweek(gtime_t ttt,double *utc){
	int week;

	if (utc[3]>=256.0) return;
	ttt.time2gpst(&week);
	utc[3]+=week/256*256;
	if (utc[3]<week-128) utc[3]+=256.0;
	else if (utc[3]>week+128) utc[3]-=256.0;
}
/* adjust daily rollover of time ------------------------------------------ */
gtime_t sbf::adjday(gtime_t ttt,double tddd){
	double tod_p;
	gtime_t adjd;

	ttt.time2epoch();
	tod_p=ttt.ep[3]*3600.0+ttt.ep[4]*60.0+ttt.ep[5];
	if (tddd<tod_p-43200.0) tddd+=86400.0;
	else if (tddd>tod_p+43200.0) tddd-=86400.0;

	adjd.ep[0]=ttt.ep[0]; adjd.ep[1]=ttt.ep[1]; adjd.ep[2]=ttt.ep[2];
	adjd.ep[3]=adjd.ep[4]=adjd.ep[5]=0.0;
	return *adjd.epoch2time(adjd.ep)->timeadd(tddd);
}
/* return frequency value in Hz from signal type name --------------------- */
double sbf::getSigFreq(int _signType,int freqNo){
	switch (_signType){	
		case 0:		return FREQ1;							/* GPSL1CA */
		case 1:		return FREQ1;							/* GPSL1PY */	
		case 2:		return FREQ2;							/* GPSL2PY */
		case 3:		return FREQ2;							/* GPSL2C  */
		case 4:		return FREQ5;							/* GPSL5   */
		case 6:		return FREQ1;							/* QZSL1C  */
		case 7:		return FREQ2;							/* QZSL2C  */
		case 8:		return FREQ1_GLO+(freqNo*9./16.)*1e6;	/* GLOL1CA */
		case 9:		return FREQ1_GLO+(freqNo*9./16.)*1e6;	/* GLOL1P  */
		case 10:	return FREQ2_GLO+(freqNo*7./16.)*1e6;	/* GLOL2P  */
		case 11:	return FREQ2_GLO+(freqNo*7./16.)*1e6;	/* GLOL2CA */
		case 12:	return 1.202025*1e9;					/* GLOL3X  */
		case 15:	return FREQ5;							/* IRNSSL5  */
		case 16:	return FREQ1;							/* GALL1A  */
		case 17:	return FREQ1;							/* GALL1BC */
		case 18:	return FREQ6;							/* GALE6A  */
		case 19:	return FREQ6;							/* GALE6BC */
		case 20:	return FREQ5;							/* GALE5a  */
		case 21:	return FREQ7;							/* GALE5b  */
		case 22:	return FREQ8;							/* GALE5   */
		case 24:	return FREQ1;							/* GEOL1   */
		case 25:	return FREQ5;							/* GEOL5   */
		case 26:	return FREQ5;							/* QZSL5   */
		case 28:	return FREQ1_CMP;						/* CMPL1   */
		case 29:	return FREQ2_CMP;						/* CMPE5B  */
		case 30:	return FREQ3_CMP;						/* CMPB3   */
	}
	return FREQ1;
}
/* adjust weekly rollover of gps time ------------------------------------- */
gtime_t sbf::adjweek(gtime_t ttt,double tow){
	double tow_p;
	int week;
	gtime_t adjw;

	tow_p=ttt.time2gpst(&week);
	if (tow<tow_p-302400.0) tow+=604800.0;
	else if (tow>tow_p+302400.0) tow-=604800.0;
	return *adjw.gpst2time(week,tow);
}
/* return the Septentrio signal type -------------------------------------- */
int sbf::getSignalCode(int signType){

	switch (signType){
		case 0:		return CODE_L1C;						/* GPSL1CA */
		case 1:		return CODE_L1W;						/* GPSL1PY */
		case 2:		return CODE_L2W;						/* GPSL2PY */
		case 3:		return CODE_L2L;						/* GPSL2C  */
		case 4:		return CODE_L5Q;						/* GPSL5   */
		case 6:		return CODE_L1C;						/* QZSL1C  */
		case 7:		return CODE_L2L;						/* QZSL2C  */
		case 8:		return CODE_L1C;						/* GLOL1CA */
		case 9:		return CODE_L1P;						/* GLOL1P  */
		case 10:	return CODE_L2P;						/* GLOL2P  */
		case 11:	return CODE_L2C;						/* GLOL2CA */
		case 12:	return CODE_L3Q;						/* GLOL3X  */
		case 15:	return CODE_L5A;						/* IRNSSL5  */
		case 16:	return CODE_L1A;						/* GALL1A  */
		case 17:	return CODE_L1C;						/* GALL1BC */
		case 18:	return CODE_L6A;						/* GALE6A  */
		case 19:	return CODE_L6C;						/* GALE6BC */
		case 20:	return CODE_L5Q;						/* GALE5a  */
		case 21:	return CODE_L7Q;						/* GALE5b  */
		case 22:	return CODE_L8Q;						/* GALE5   */
		case 24:	return CODE_L1C;						/* GEOL1   */
		case 25:	return CODE_L5I;						/* GEOL5   */
		case 26:	return CODE_L5Q;						/* QZSL5   */
		case 28:	return CODE_L2I;						/* CMPL1   */
		case 29:	return CODE_L7I;						/* CMPE5B  */
		case 30:	return CODE_L6I;						/* CMPB3   */
	}
	return CODE_L1C;
}
/* return the signal type ------------------------------------------------- */
int sbf::getFreqNo(int signType){
	switch (signType){
		case 0:		return 0;						/* GPSL1CA */
		case 1:		return 0;						/* GPSL1PY */
		case 2:		return 1;						/* GPSL2PY */
		case 3:		return 1;						/* GPSL2C  */
		case 4:		return 2;						/* GPSL5   */
		case 6:		return 0;						/* QZSL1C  */
		case 7:		return 1;						/* QZSL2C  */
		case 8:		return 0;						/* GLOL1CA */
		case 9:		return 0;						/* GLOL1P  */
		case 10:	return 1;						/* GLOL2P  */
		case 11:	return 2;						/* GLOL2CA */
		case 12:	return 3;						/* GLOL3X  */
		case 15:	return 2;						/* IRNSSL5  */
		case 16:	return 0;						/* GALL1A  */
		case 17:	return 0;						/* GALL1BC */
		case 18:	return 1;						/* GALE6A  */
		case 19:	return 1;						/* GALE6BC */
		case 20:	return 1;						/* GALE5a  */
		case 21:	return 1;						/* GALE5b  */
		case 22:	return 2;						/* GALE5   */
		case 24:	return 0;						/* GEOL1   */
		case 25:	return 2;						/* GEOL5   */
		case 26:	return 2;						/* QZSL5   */
		case 28:	return 0;						/* CMPL1   */
		case 29:	return 2;						/* CMPE5B  */
		case 30:	return 1;						/* CMPB3   */
	}
	return 0;
}
/* sync to the beginning of a block --------------------------------------- */
int sbf::sync_sbf(unsigned char data){
	buff[0]=buff[1]; buff[1]=data;
	return buff[0]== SBF_SYNC1 && buff[1]==SBF_SYNC2;
}

/* decode SBF measurements message (observables) -------------------------- */
/*
* this is the most importan block in the SBF format. It it contains all code
* pseudoranges and carrier phase measurements of all received satellites.
* This block is made of one Type1 sub-block per santellite followed by, if any,
* a certain number of Type2 sub-blocks. SB2Num defines how many Type2
* sub-blocks there are inside its Type1 sub-block.
* Type1 subplock contains code pseudorange and carrier phase range of the first
* decoded sygnal defined by signType1, this is typically L1 signal.
* Any following Type2 sub-block (if there are any) contains signType2 signal
* information, typically L2 signal. Inside Type2 sub-blocks, information is
* expressed as difference from the data in signType1 sub-block. This makes the
* format a little more compact.
*
*/
int sbf::decode_measepoch(){
	gtime_t ttt;
	double tow,psr,adr,dopplerType1;
	double SNR_DBHZ,SNR2_DBHZ,freqType1,freqType2,alpha;
	int16_t i,ii,j,prn,sat,n=0,nsat,week,code,h;
	uint8_t *p=(buff)+8;							/* jump to TOW location */
	int SB1length,SB2length;
	uint8_t signType1,signType2;
	uint32_t codeLSB,SB2Num,sys;
	uint8_t codeMSB,CommonFlags;
	int pri;

	/* signals for type2 sub-block */
	int32_t CodeOffsetMSB,DopplerOffsetMSB,CarrierMSB;
	uint16_t DopplerOffsetLSB,CodeOffsetLSB,CarrierLSB;
	double PRtype2,Ltype2,dopplerType2;

	uint16_t LockTime,LockTime2;
	uint8_t ObsInfo,ObsInfo2,offsetMSB;
	double SB1_WaveLength,SB1_Code = 0.0;
	short SB1_FreqNr;

	/* Get time information */
	tow =U4(p);										/*       TOW in ms */
	week=U2(p+4);									/* number of weeks */

	/* Tweak pseudoranges to allow Rinex to represent the time of measure */

	ttt.gpst2time(week,tow*0.001);

	/* number of type1 sub-blocks also equal to number of satellites */
	nsat   = U1(p+6);

	/* additional block information */
	SB1length=U1(p+7);								/* Type1 sub-block length */
	SB2length=U1(p+8);								/* Type2 sub-block length */

	CommonFlags = U1(p+9);

	if ((CommonFlags & 0x80)==0x80) return 0;		/* data is ccrambled and not valid */

	/* set the pointer from TOW to the beginning of type1 sub-block */
	p = p + 12;

	for (i=0; i<nsat&&i<MAXOBS; i++) {

		/* decode type1 sub-block */
		signType1 = U1(p+1) & 0x1f;					/* type of signal, bit[0-4] */
		prn = U1(p+2);								/* satellite number         */
		obs.data[n].time  = ttt;					/* not sure what ref. it is */
		codeMSB = (U1(p+3) & 0x0f);					/* code phase MSB. bit[0-3] */
		codeLSB = U4(p+4);							/* code phase LSB           */

		/* code pseudorange in m */
		psr = (codeMSB*4294967296.0+codeLSB)*0.001;

		/*  Doppler in Hz */
		dopplerType1  = 0.0001*I4(p+8);

		/* signal to noise ratio in dBHz */
		if ((signType1==1) || (signType1==2)){
			SNR_DBHZ=((double)U1(p+15))*0.25;
		}
		else SNR_DBHZ=(((double)U1(p+15))*0.25)+10;

		/* Compute the carrier phase measurement (a little complicated) */
		LockTime = U2(p+16);						/* Duration of contin. carrier phase */
		ObsInfo = U1(p+18);							/* some extra info                   */
		SB2Num = U1(p+19);							/* number of type2 sub-blocks        */
		SB1_Code = psr;								/* code phase (from before)          */

		/* FreqNr */
		SB1_FreqNr = ((ObsInfo >> 3) & 0x1f) - 8;

		SB1_WaveLength=CLIGHT/getSigFreq(signType1,SB1_FreqNr);

		/* final carrier phase calculation */
		adr = (SB1_Code/SB1_WaveLength)+(I1(p+14)*65536.0+U2(p+12))*0.001;
		if ((I2(p+14)==-128)&&(U2(p+12)==0)) {
			adr=0;
		}

		/* debug */

		/* from the signal tiype get the type of RTKLIB signal code*/
		code = getSignalCode(signType1);

		/* NOT SURE IF THIS IS CORRECT, MAYBE ITS SHOULD APPLY TO L2 AS WELL */
		/* phase polarity flip option (-INVCP) */
		if (opt.find("-INVCP")!=string::npos) {
			adr = -adr;
		}

		/* work out sat number and type of GNSS system */
		if ((prn>=1)&&(prn<=37)){
			sys = SYS_GPS;                      /* navigation system: GPS     */
			sat = prn;
		}
		else if ((prn>=38)&&(prn<=61)){
			sys = SYS_GLO;                      /* navigation system: GLONASS */
			sat = prn - 37;
		}
		else if ((prn>=63)&&(prn<=68)){
			sys = SYS_GLO;                      /* navigation system: GLONASS */
			sat = prn - 38;
		}
		else if ((prn>=71)&&(prn<=102)){
			sys = SYS_GAL;                      /* navigation system: Galileo */
			sat = prn - 70;
		}
		else if ((prn>=120)&&(prn<=140)){
			sys = SYS_SBS;                      /* navigation system: SBAS    */
			sat = prn;
		}
		else if ((prn>=141)&&(prn<=177)){
			sys = SYS_CMP;                      /* navigation system: BeiDou  */
			sat = prn - 140;
		}
		else if ((prn>=181)&&(prn<=187)){
			sys = SYS_QZS;                      /* navigation system: QZSS    */
			sat = prn - 180;
		}
		else if ((prn>=191)&&(prn<=197)){
			sys = SYS_IRN;                      /* navigation system: IRNSS  */
			sat = prn - 190;
		}
		else if ((prn>=198)&&(prn<=215)){
			sys = SYS_SBS;                      /* navigation system: SBAS, */
			sat = prn - 157;
		}
		else{
			sys = SYS_NONE;                     /* navigation system: none    */
			sat = 0;
		}

		/* store satellite number */
		sat = satno(sys,sat);
		if (sat == 0)
		{
			p = p + SB1length;						/* skip data */
			p = p + SB2length*SB2Num;
			continue;
		};

		obs.data[n].sat=sat;

		/* start new observation period */
		if (fabs(obs.data[0].time.timediff(time))>1E-9) {
			obs.n=0;
		}

		/* store type1 signal information is RTKLIB signal structure
		*/
		/* Set all channels to 0 */
		for (j=0; j<NFREQ+NEXOBS; j++) {
			obs.data[n].L[j]=0.0;
			obs.data[n].P[j]=0.0;
			obs.data[n].D[j]=(float)0.0;
			obs.data[n].SNR[j]=(unsigned char)0;
			obs.data[n].LLI[j]=(unsigned char)0;
			obs.data[n].code[j]=CODE_NONE;
		}
		/* detect which signals is stored in Type1 sub-block */
#if 1
		h=getFreqNo(signType1);
#else
		freqType1 = getSigFreq(signType1,8);
		if (freqType1 == FREQ1) h = 0;
		else if (freqType1 == FREQ2) h = 1;
		else if (freqType1 == FREQ5) h = 2;
		else if (freqType1 == FREQ6) h = 3;
		else if (freqType1 == FREQ7) h = 4;
		else if (freqType1 == FREQ8) h = 5;
		else                         h = 0;
#endif
		/* store signal info */
		if (h<=NFREQ+NEXOBS) {
			obs.data[n].L[h]    = adr;
			obs.data[n].P[h]    = psr;
			obs.data[n].D[h]    = (float)dopplerType1;
			obs.data[n].SNR[h]  = (unsigned char)(SNR_DBHZ*4.0);
			obs.data[n].code[h] = code;

			/* lock to signal indication */
			if ((ObsInfo&0x4)==0x4) obs.data[n].LLI[h]|=0x2; /* half-cycle ambiguity */
			if (LockTime!=65535){
				/* limit locktime to sizeof(unsigned char) */
				LockTime = LockTime>254 ? 254 : LockTime; 
				if (locktime[sat][signType1]>LockTime) obs.data[n].LLI[h]|=0x1;
				lockt[sat][h]       = (unsigned char)LockTime;
				locktime[sat][signType1] = LockTime;
			};
		}

		/* decode all Type2 sub-blocks (if there is any) */
		p = p + SB1length;							/* get to the begin of Type2 block */

		for (ii=0; ii<(int)SB2Num; ii++)
		{
			signType2 = U1(p) & 0x1f;				/* type of signal, bit[0-4] */

			/* Duration of continuous carrier phase */
			LockTime2 = U1(p+1);

			/* Signal to noise ratio in dbHz */
			if ((signType2==1) || (signType2==2)){
				SNR2_DBHZ=((double)U1(p+2))*0.25;
			}
			else SNR2_DBHZ=(((double)U1(p+2))*0.25)+10;

			offsetMSB = U1(p+3);
			CodeOffsetMSB=((offsetMSB&0x04)==0x04) ? 
				offsetMSB| ~((int32_t)0x03) : offsetMSB&0x03;			/* bit[0-2] */
			DopplerOffsetMSB=((offsetMSB&0x80)==0x80) ? 
				(offsetMSB>>3)| ~((int32_t)0x1f) : (offsetMSB>>3)&0x1f;/* bit[3-7] */

			CarrierMSB = I1(p+4);

			ObsInfo2 = U1(p+5);						/* minor informations */

			freqType1 = getSigFreq(signType1,SB1_FreqNr);
			freqType2 = getSigFreq(signType2,SB1_FreqNr);

			/* pseudrange in meters */
			CodeOffsetLSB = U2(p+6);
			PRtype2 = psr + (CodeOffsetMSB*65536.+CodeOffsetLSB)*0.001;
			if ((CodeOffsetMSB==-4)&&(CodeOffsetLSB==0)) {
				PRtype2=0;
			}

			/* carrier phase in cycles */
			CarrierLSB = U2(p+8);
			Ltype2=(PRtype2/CLIGHT*freqType2)+(CarrierMSB*65536.+CarrierLSB)*0.001;
			if ((CarrierMSB==-128)&&(CarrierLSB==0)) {
				Ltype2=0;
			}

			/* Doppler in Hz */
			DopplerOffsetLSB = U2(p+10);
			alpha = (freqType2/freqType1);
			if ((DopplerOffsetMSB==-16) && (DopplerOffsetLSB==0)) dopplerType2=0;
			else
				dopplerType2 = dopplerType1*alpha +\
				(DopplerOffsetMSB*65536.+DopplerOffsetLSB)*1E-4;

			/* store Type2 signal info in rtklib structure */
#if 1
			h=getFreqNo(signType2);
#else
			freqType2 = getSigFreq(signType2,8);
			if (freqType2 == FREQ1) h = 0;
			else if (freqType2 == FREQ2) h = 1;
			else if (freqType2 == FREQ5) h = 2;
			else if (freqType2 == FREQ6) h = 3;
			else if (freqType2 == FREQ7) h = 4;
			else if (freqType2 == FREQ8) h = 5;
			else                         h = 0;
#endif
			pri=getcodepri(sys,getSignalCode(signType2),opt);	/* get signal priority */
			/* store signal info */
			if ((h<=NFREQ+NEXOBS)&&
				(pri>getcodepri(sys,obs.data[n].code[h],opt))) {
				obs.data[n].L[h]    = Ltype2;
				obs.data[n].P[h]    = PRtype2;
				obs.data[n].D[h]    = (float)dopplerType2;
				obs.data[n].SNR[h]  = (unsigned char)(SNR2_DBHZ*4.0);
				obs.data[n].code[h] = getSignalCode(signType2);

				/* lock to signal indication */
				if ((ObsInfo2&0x4)==0x4) obs.data[n].LLI[h]|=0x2; /* half-cycle ambiguity */
				if (LockTime2!=255) {
					if (locktime[sat][signType2]>LockTime2) obs.data[n].LLI[h]|=0x1;
					lockt[sat][h]       = (unsigned char)LockTime2;
					locktime[sat][signType2] = (unsigned char)LockTime2;
				};
			}

			/* get to the beginning of next Type 2 block */
			p = p + SB2length;
		}

		/* Receiver channel goes up */
		n++;
	}
	time=ttt;
	obs.n=n;
	return 1;
}
/* decode SBF nav message for GPS (navigation data) ----------------------- */
int sbf::decode_gpsnav(){
	uint8_t *puiTmp = (buff)+6;
	eph_t eph;
	double toc;
	uint8_t prn,sat;
	uint16_t week;

	if ((len)<120) {
		return -1;
	}

	prn = U1(puiTmp+8);
	sat = satno(SYS_GPS,prn);

	if (sat == 0) return -1;

	if (!((prn>=1)&&(prn<=37))){
		return -1;
	}

	eph.crs    = R4(puiTmp +  42);
	eph.deln   = R4(puiTmp +  46) * PI;
	eph.M0     = R8(puiTmp +  50) * PI;
	eph.cuc    = R4(puiTmp +  58);
	eph.e      = R8(puiTmp +  62);
	eph.cus    = R4(puiTmp +  70);
	eph.A      = pow(R8(puiTmp +  74),2);
	eph.toes   = U4(puiTmp +  82);
	eph.cic    = R4(puiTmp +  86);
	eph.OMG0   = R8(puiTmp +  90) * PI;
	eph.cis    = R4(puiTmp +  98);
	eph.i0     = R8(puiTmp + 102) * PI;
	eph.crc    = R4(puiTmp + 110);
	eph.omg    = R8(puiTmp + 114) * PI;
	eph.OMGd   = R4(puiTmp + 122) * PI;
	eph.idot   = R4(puiTmp + 126) * PI;
	eph.tgd[0] = R4(puiTmp +  22);
	toc        = U4(puiTmp +  26);
	eph.f2     = R4(puiTmp +  30);
	eph.f1     = R4(puiTmp +  34);
	eph.f0     = R4(puiTmp +  38);
	eph.sva    = U1(puiTmp +  13); /* URA */
	eph.iodc   = U2(puiTmp +  16);
	eph.iode   = U1(puiTmp +  18);
	eph.code   = U1(puiTmp +  12);
	eph.flag   = U1(puiTmp +  15);
	eph.fit    = U1(puiTmp +  20) ? 0 : 4;
	week       = U2(puiTmp +  10); /* WN */

	if (week>=4096) {
		return -1;
	}

	eph.week=adjgpsweek(week);
	eph.toe.gpst2time(eph.week,eph.toes);
	eph.toc.gpst2time(eph.week,toc);
	eph.ttr=time;

	if (opt.find("-EPHALL")==string::npos) {
		if ((eph.iode==nav.eph[sat-1].iode) &&
			(eph.iodc==nav.eph[sat-1].iodc)) return 0;
	}

	eph.sat=sat;
	nav.eph[sat-1]=eph;
	ephsat=sat;
	return 2;
}
/* decode SBF gpsion ------------------------------------------------------ */
int sbf::decode_gpsion(){
	uint8_t *p=(buff)+8;            /* points at TOW location */

	if (len<48){
		return -1;
	}

	nav.ion_gps[0] = R4(p + 8);
	nav.ion_gps[1] = R4(p + 12);
	nav.ion_gps[2] = R4(p + 16);
	nav.ion_gps[3] = R4(p + 20);
	nav.ion_gps[4] = R4(p + 24);
	nav.ion_gps[5] = R4(p + 28);
	nav.ion_gps[6] = R4(p + 32);
	nav.ion_gps[7] = R4(p + 36);

	return 9;
}
/* decode SBF gpsutc ------------------------------------------------------ */
int sbf::decode_gpsutc(){
	uint8_t *p=(buff)+8;                 /* points at TOW location */

	if (len<37){
		return -1;
	}

	/* GPS delta-UTC parameters */
	nav.utc_gps[1] = R4(p + 8);                                  /*   A1 */
	nav.utc_gps[0] = R8(p + 12);                                 /*   A0 */
	nav.utc_gps[2] = U4(p + 20);                                 /*  tot */
	/* nav.utc_gps[3] = U1(p + 24); */                           /*  WNt */
	nav.utc_gps[3] = adjgpsweek(U2(p + 4));                      /*   WN */
	nav.leaps      = I1(p + 25);                                 /* Dtls */

	/*NOTE. it is kind of strange that I have to use U1(p+4) and not U1(p+24)
			in fact if I take U1(p+24) I do not seem to ge the correct W in
			the header of RINEX nav file, line DELTA-UTC: A0,A1,T,W    */
	return 9;
}
/* decode SBF gpsalm ------------------------------------------------------ */
int sbf::decode_gpsalm(){
	uint8_t *p=(buff)+8;                 /* points at TOW location */
	int sat;

	if (len<60){
		return -1;
	}
	sat=satno(SYS_GPS,U1(p + 6));
	if (sat == -1) return 0;

	nav.alm[sat].sat   = sat;
	nav.alm[sat].e     = R4(p + 8);
	nav.alm[sat].toas  = U4(p + 12);
	nav.alm[sat].i0    = R4(p + 16);
	nav.alm[sat].OMGd  = R4(p + 20);
	nav.alm[sat].A     = pow(R4(p + 24),2);
	nav.alm[sat].OMG0  = R4(p + 28);
	nav.alm[sat].omg   = R4(p + 32);
	nav.alm[sat].M0    = R4(p + 36);
	nav.alm[sat].f1    = R4(p + 40);
	nav.alm[sat].f0    = R4(p + 44);
	nav.alm[sat].week  = U1(p + 48);
	nav.alm[sat].svconf= U1(p + 49);
	nav.alm[sat].svh   = U1(p + 50);
	nav.alm[sat].toa.gpst2time(nav.alm[sat].week,nav.alm[sat].toas);

	return 9;
}
/* decode SBF raw nav message (raw navigation data) ----------------------- */
int sbf::decode_rawnav(int sys){/* NOTE. This function works quite well but it somestimes fails in line:
     * if (resp>5 || resp<=0){
     * To debug the problem an understanding of the whole RTK code is needed
     */

	uint8_t *p=(buff)+6,id;
	decode_frame dec;
	int sat,prn;
	uint8_t _buf[30]={ 0 };
	int i=0,ii=0;

	if (len<58) {
		return -1;
	}

	/* get GPS satellite number */
	prn=U1(p+8);
	if (sys==SYS_QZS) prn-=180;

	sat=satno(sys,prn);
	if (sat == 0) return -1;

	/* clean up subframe from Septentrio. This is a little bit of work because
	* Septentrio Rx add some parity bits to this message.
	* We have to throw away the reserved bits as well as the parity bits.
	*/

	/*   | 2bits |         24bits        |  6bits  |       <- SBF 32-bit word
	------------------------------------------
	| byte1 | bite2 | byte3 |                 <- sat nav message
	*/

	for (i=0; i<40; i+=4){
		_buf[ii]   =((U4(p+14+i)>>22) & 0x000000FF);     /* take first byte  */
		_buf[1+ii] =((U4(p+14+i)>>14) & 0x000000FF);     /* take second byte */
		_buf[2+ii] =((U4(p+14+i)>>6) & 0x000000FF);     /* take third byte  */
		ii = ii+3;
	}

	/* Now that we have a classic subframe we call the generic function */
	id=getbitu(_buf,43,3); /* get subframe id */
	if ((id < 1) || (id > 5)) return -1;

	memcpy(subfrm[sat-1]+(id-1)*30,_buf,30);

	if (dec.decode(subfrm[sat-1])==1&&
		dec.decode(subfrm[sat-1]+30)==2&&
		dec.decode(subfrm[sat-1]+60)==3) {

		if (opt.find("-EPHALL")==string::npos) {
			if ((dec.eph.iode==nav.eph[sat-1].iode)&&
				(dec.eph.iodc==nav.eph[sat-1].iodc)) return 0;
		}
		dec.eph.sat=sat;
		nav.eph[sat-1]=dec.eph;
		ephsat=sat;
		return 2;
	}
	if (id==4) {
		if (sys==SYS_GPS) {
			dec.decode(subfrm[sat-1]+90);
			nav.alm.assign(dec.alm.begin(),dec.alm.end());
			vecarr(dec.ion.begin(),nav.ion_gps,8);
			vecarr(dec.utc.begin(),nav.utc_gps,4);
			nav.leaps=dec.leaps;
			adj_utcweek(time,nav.utc_gps);
		}
		else if (sys==SYS_QZS) {
			dec.decode(subfrm[sat-1]+90);
			nav.alm.assign(dec.alm.begin(),dec.alm.end());
			vecarr(dec.ion.begin(),nav.ion_qzs,8);
			vecarr(dec.utc.begin(),nav.utc_qzs,4);
			nav.leaps=dec.leaps;
			adj_utcweek(time,nav.utc_qzs);
		}
		return 9;
	};
	if (id==5) {
		if (sys==SYS_GPS) {
			dec.decode(subfrm[sat-1]+120);
			nav.alm.assign(dec.alm.begin(),dec.alm.end());
		}
		else if (sys==SYS_QZS) {
			dec.decode(subfrm[sat-1]+120);
			nav.alm.assign(dec.alm.begin(),dec.alm.end());
			vecarr(dec.ion.begin(),nav.ion_qzs,8);
			vecarr(dec.utc.begin(),nav.utc_qzs,4);
			nav.leaps=dec.leaps;
			adj_utcweek(time,nav.utc_qzs);
		}
		return 9;
	};
	return 0;
}
/* decode SBF nav message for sbas (navigation data) ---------------------- */
int sbf::decode_sbasnav(){
	uint8_t *puiTmp = (buff)+6;
	seph_t eph;
	int prn,sat;
	uint16_t week;
	uint32_t ttt,tow;

	if ((len)<104) {
		return -1;
	}
	prn = U1(puiTmp+8);
	sat = satno(SYS_SBS,prn);

	if (!((prn>=120)&&(prn<=140))){
		return -1;
	}

	if (sat == 0) return -1;

	week       = U2(puiTmp +   6);
	tow        = U4(puiTmp +  2)/1000;
	ttt        = U4(puiTmp +  14);
	eph.t0     = adjday(eph.tof,ttt);
	eph.sva    = U2(puiTmp +  12);
	eph.svh    = eph.sva==15 ? 1 : 0;
	eph.pos[0] = R8(puiTmp +  18);
	eph.pos[1] = R8(puiTmp +  26);
	eph.pos[2] = R8(puiTmp +  34);
	eph.vel[0] = R8(puiTmp +  42);
	eph.vel[1] = R8(puiTmp +  50);
	eph.vel[2] = R8(puiTmp +  58);
	eph.acc[0] = R8(puiTmp +  66);
	eph.acc[1] = R8(puiTmp +  74);
	eph.acc[2] = R8(puiTmp +  82);
	eph.af0    = R4(puiTmp +  90);
	eph.af1    = R4(puiTmp +  94);
	eph.tof.gpst2time(adjgpsweek(week),tow);

	/* debug */

	if (opt.find("-EPHALL")==string::npos) {
		if (fabs(eph.t0.timediff(nav.seph[prn-120].t0))<1.0&&
			eph.sva==nav.seph[prn-120].sva)
			return 0;
	}

	eph.sat=sat;
	nav.seph[prn-120]=eph;
	ephsat=eph.sat;
	return 2;
}
/* decode SBF raw nav message (raw navigation data) ----------------------- */
int sbf::decode_georaw(){
	uint8_t *p=(buff)+6;
	uint8_t bbb[8*4];
	int prn;
	int i=0;
	uint32_t tmp,crc;

	if (len<52) {
		return -1;
	}
	/* CRC test failed */
	if (U1(p+9)!=1){
		return -1;
	}

	/* get GPS satellite number */
	prn=U1(p+8);

	/* copy data */
	for (i=0; i<8; i++)
	{
		tmp = U4(p+14+i*4);
		bbb[4*i]=(tmp>>24) & 0xff;
		bbb[4*i+1]=(tmp>>16) & 0xff;
		bbb[4*i+2]=(tmp>>8) & 0xff;
		bbb[4*i+3]=(tmp>>0) & 0xff;
	}

	sbsmsg.prn=prn;
	sbsmsg.tow=U4(p+2)/1000;
	sbsmsg.week=U2(p+6);
	time.gpst2time(sbsmsg.week,sbsmsg.tow);

	crc=(bbb[31])+(bbb[30]<<8)+(bbb[29]<<16);
	if (crc!=rtk_crc24q(bbb,29)) return 0;

	for (i=0; i<29; i++) sbsmsg.msg[i]=bbb[i];
	sbsmsg.msg[28]&=0xC0;
	return 3;
}

/* decode SBF raw message ------------------------------------------------- */
int sbf::decode_sbf(){
	unsigned short crc;
	char msg[100];

	/* read the SBF block ID and revision */
	int type = U2(buff+4) & 0x1fff << 0;
	int revision = U2(buff+4) >> 13;
	(void)revision;

	/* read the SBF block CRC */
	crc = U2(buff+2);

	/* checksum skipping first 4 bytes */
	if (sbf_checksum(buff+4,len-4) !=  crc){
		return -1;
	}

	if (outtype) {
		sprintf(msg, "SBF 0x%04X (%4d):",type,len);
		msgtype=msg;
	}

	switch (type) {
	case ID_MEASEPOCH:      return decode_measepoch();

	case ID_GPSNAV:         return decode_gpsnav();
	case ID_GPSION:         return decode_gpsion();
	case ID_GPSUTC:         return decode_gpsutc();
	case ID_GPSALM:         return decode_gpsalm();
	case ID_GPSRAWCA:
	case ID_GPSRAWL2C:
	case ID_GPSRAWL5:       return decode_rawnav(SYS_GPS);

	case ID_GEONAV:         return decode_sbasnav();
	case ID_GEORAWL1:
	case ID_GEORAWL5:       return decode_georaw();

#ifdef ENAGLO
	case ID_GLONAV:         return decode_glonav();
	case ID_GLORAWCA:       return decode_glorawcanav();
	case ID_GLOTIME:        return decode_gloutc();
#endif

#ifdef ENAGAL
	case ID_GALNAV:         return decode_galnav();
	case ID_GALION:         return decode_galion();
	case ID_GALUTC:         return decode_galutc();
	case ID_GALALM:         return decode_galalm();
	case ID_GALRAWINAV:     return decode_galrawinav();
#endif

#ifdef TESTING /* not tested */
#ifdef ENAQZS
	case ID_QZSSL1CA:
	case ID_QZSSL2C:
	case ID_QZSSL5:         return decode_rawnav(,SYS_QZS); l
	case ID_QZSS_NAV:       return decode_qzssnav();
#endif

#ifdef ENACMP
	case ID_CMPRAW:         return decode_cmpraw();
	case ID_CMPNAV:         return decode_cmpnav();
#endif
#endif

#if 0 /* not yet supported */
	case ID_GEOMT00:        return decode_sbsfast();
	case ID_GEOPRNMASK:     return decode_sbsprnmask();
	case ID_GEOFASTCORR:    return decode_sbsfast();
	case ID_GEOINTEGRITY:   return decode_sbsintegriy();
	case ID_GEOFASTCORRDEGR:return decode_sbsfastcorrdegr();
	case ID_GEOIGPMASK:     return decode_sbsigpmask();
	case ID_GEOLONGTERMCOR: return decode_sbslongcorrh();
	case ID_GEOIONODELAY:   return decode_sbsionodelay();
#endif

#if 0 /* unused */
	case ID_GALRAWFNAV:     return decode_galrawfnav(); /* not yet supported */
	case ID_GLOALM:         return decode_glosalm(); /* not yet supported */

	case ID_PVTGEOD:        return decode_pvtgeod();
	case ID_RXSETUP:        return decode_rxsetup();
	case ID_COMMENT:        return decode_comment();
#endif
	default:				return 0;
		/* there are many more SBF blocks to be extracted */
	}
	return 0;
}

/* input sbf raw data from stream ----------------------------------------- */
int sbf::decode(unsigned char data){
	if (nbyte==0) {
		if (sync_sbf(data)) nbyte=2;
		return 0;
	}
	buff[nbyte++]=data;

	if (nbyte<8) return 0;

	if ((len=U2(buff+6))>MAXRAWLEN) {
		nbyte=0;
		return -1;
	}
	if (nbyte<len) return 0;
	nbyte=0;

	return decode_sbf();
}