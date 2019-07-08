#include "trimble17.h"

#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
/* General purpose flag bits masks: */
#define M_BIT0 (1 << 0)
#define M_BIT1 (1 << 1)
#define M_BIT2 (1 << 2)
#define M_BIT3 (1 << 3)
#define M_BIT4 (1 << 4)
#define M_BIT5 (1 << 5)
#define M_BIT6 (1 << 6)
#define M_BIT7 (1 << 7)
#define M_BIT8 (1 << 8)
#define M_BIT9 (1 << 9)
#define M_BIT10 (1 << 10)
#define M_BIT11 (1 << 11)
#define M_BIT12 (1 << 12)
#define M_BIT13 (1 << 13)
#define M_BIT14 (1 << 14)
#define M_BIT15 (1 << 15)

/* Constant definitions: */
#define STX           2         /* Start of packet character */
#define ETX           3         /* End of packet character */
#define GENOUT        0x40      /* General Serial Output Format (GSOF) */
#define RETSVDATA     0x55      /* Satellite information reports */
#define RAWDATA       0x57      /* Position or real-time survey data report */
#define MBUFF_LENGTH  8192      /* Message buffer length */
#define PBUFF_LENGTH  (4+255+2) /* Packet buffer length */

/* Record Interpretation Flags bit masks: */
#define M_CONCISE     M_BIT0    /* Concise format */
#define M_ENHANCED    M_BIT1    /* Enhanced record with real-time flags and IODE information */

/* rt17.flag bit definitions: */
#define M_WEEK_OPTION M_BIT0    /* GPS week number set by WEEK=n option */
#define M_WEEK_EPH    M_BIT1    /* GPS week number set by ephemeris week */
#define M_WEEK_TIME   M_BIT2    /* GPS week set by computer time */
#define M_WEEK_SCAN   M_BIT3    /* WEEK=n option already looked for, no need to do it again */

/* Data conversion macros: */
#define I1(p) (*((char*)(p)))           /* One byte signed integer */
#define U1(p) (*((unsigned char*)(p)))  /* One byte unsigned integer */
#define I2(p) ReadI2(p)                 /* Two byte signed integer */
#define U2(p) ReadU2(p)                 /* Two byte unsigned integer */
#define I4(p) ReadI4(p)                 /* Four byte signed integer */
#define U4(p) ReadU4(p)                 /* Four byte unsigned integer */
#define R4(p) ReadR4(p)                 /* IEEE S_FLOAT floating point number */
#define R8(p) ReadR8(p)                 /* IEEE T_FLOAT floating point number */

/* Internal structure definitions. */
typedef union { unsigned short u2; unsigned char c[2]; } ENDIAN_TEST;

/* global literals: */
const char rcsid[]="$Id:$";

/* GENOUT 0x40 message types: */
const char *GSOFTable[] ={
	/* 00 */ NULL,
	/* 01 */ "Position Time",
	/* 02 */ "Latitude Longitude Height",
	/* 03 */ "ECEF Position",
	/* 04 */ "Local Datum LLH Position",
	/* 05 */ "Local Zone ENU Position",
	/* 06 */ "ECEF Delta",
	/* 07 */ "Tangent Plane Delta",
	/* 08 */ "Velocity Data",
	/* 09 */ "DOP Information",
	/* 10 */ "Clock Information",
	/* 11 */ "Position VCV Information",
	/* 12 */ "Position Sigma Information",
	/* 13 */ "SV Brief Information",
	/* 14 */ "SV Detailed Information",
	/* 15 */ "Receiver Serial Number",
	/* 16 */ "Current Time UTC",
	/* 17 */ NULL,
	/* 18 */ NULL,
	/* 19 */ NULL,
	/* 20 */ NULL,
	/* 21 */ NULL,
	/* 22 */ NULL,
	/* 23 */ NULL,
	/* 24 */ NULL,
	/* 25 */ NULL,
	/* 26 */ "Position Time UTC",
	/* 27 */ "Attitude Information",
	/* 28 */ NULL,
	/* 29 */ NULL,
	/* 30 */ NULL,
	/* 31 */ NULL,
	/* 32 */ NULL,
	/* 33 */ "All SV Brief Information",
	/* 34 */ "All SV Detailed Information",
	/* 35 */ "Received Base Information",
	/* 36 */ NULL,
	/* 37 */ "Battery and Memory Information",
	/* 38 */ NULL,
	/* 39 */ NULL,
	/* 40 */ "L-Band Status Information",
	/* 41 */ "Base Position and Quality Indicator"
};

/* RAWDATA 0x57 message types: */
const char *RawdataTable[] ={
	/* 00 */ "Real-time GPS Survey Data, type 17",
	/* 01 */ "Position Record, type 11",
	/* 02 */ "Event Mark",
	/* 03 */ NULL,
	/* 04 */ NULL,
	/* 05 */ NULL,
	/* 06 */ "Real-time GNSS Survey Data, type 27",
	/* 07 */ "Enhanced Position Record, type 29"
};

/* RETSVDATA 0x55 message types: */
const char *RetsvdataTable[] ={
	/* 00 */ "SV Flags",
	/* 01 */ "GPS Ephemeris",
	/* 02 */ "GPS Almanac",
	/* 03 */ "ION / UTC Data",
	/* 04 */ "Disable Satellite, depreciated",
	/* 05 */ "Enable Satellite, depreciated",
	/* 06 */ NULL,
	/* 07 */ "Extended GPS Almanac",
	/* 08 */ "GLONASS Almanac",
	/* 09 */ "GLONASS Ephemeris",
	/* 10 */ NULL,
	/* 11 */ "Galileo Ephemeris",
	/* 12 */ "Galileo Almanac",
	/* 13 */ NULL,
	/* 14 */ "QZSS Ephemeris",
	/* 15 */ NULL,
	/* 16 */ "QZSS Almanac",
	/* 17 */ NULL,
	/* 18 */ NULL,
	/* 19 */ NULL,
	/* 20 */ "SV Flags",
	/* 21 */ "BeiDou Ephemeris",
	/* 22 */ "BeiDou Almanac"
};

/* ReadU2 - Fetch & convert an unsigned twe byte integer (unsigned short) */
unsigned short ReadU2(unsigned char *p){
	ENDIAN_TEST et;
	union U2 { unsigned short u2; unsigned char c[2]; } u;

	memcpy(&u.u2,p,sizeof(u.u2));

	et.u2 = 0; et.c[0] = 1;
	if (et.u2 == 1){
		unsigned char t;
		t = u.c[0]; u.c[0] = u.c[1]; u.c[1] = t;
	}
	return u.u2;
}

/* ReadU4 - Fetch & convert a four byte unsigned integer (unsigned int) */
unsigned int ReadU4(unsigned char *p){
	ENDIAN_TEST et;
	union U4 { unsigned int u4; unsigned char c[4]; } u;

	memcpy(&u.u4,p,sizeof(u.u4));

	et.u2 = 0; et.c[0] = 1;
	if (et.u2 == 1){
		unsigned char t;
		t = u.c[0]; u.c[0] = u.c[3]; u.c[3] = t;
		t = u.c[1]; u.c[1] = u.c[2]; u.c[2] = t;
	}
	return u.u4;
}

/* ReadI2 - Fetch & convert a signed two byte integer (short) */
short ReadI2(unsigned char *p)
{
	union I2 { short i2; unsigned char c[2]; } u;
	ENDIAN_TEST et;

	memcpy(&u.i2,p,sizeof(u.i2));

	et.u2 = 0; et.c[0] = 1;
	if (et.u2 == 1){
		unsigned char t;
		t = u.c[0]; u.c[0] = u.c[1]; u.c[1] = t;
	}
	return u.i2;
}

/* ReadI4 - Fetch & convert a four byte signed integer (int) */
int ReadI4(unsigned char *p){
	union I4 { int i4; unsigned char c[4]; } u;
	ENDIAN_TEST et;

	memcpy(&u.i4,p,sizeof(u.i4));

	et.u2 = 0; et.c[0] = 1;
	if (et.u2 == 1){
		unsigned char t;
		t = u.c[0]; u.c[0] = u.c[3]; u.c[3] = t;
		t = u.c[1]; u.c[1] = u.c[2]; u.c[2] = t;
	}
	return u.i4;
}

/* ReadR4 - Fetch & convert an IEEE S_FLOAT (float) */
float ReadR4(unsigned char *p){
	union R4 { float f; unsigned int u4; } u;
	u.u4 = U4(p);
	return u.f;
}

/* ReadR8 - Fetch & convert an IEEE T_FLOAT (double) */
double ReadR8(unsigned char *p){
	ENDIAN_TEST et;
	union R8 { double d; unsigned char c[8]; } u;

	memcpy(&u.d,p,sizeof(u.d));

	et.u2 = 0; et.c[0] = 1;
	if (et.u2 == 1){
		unsigned char t;
		t = u.c[0]; u.c[0] = u.c[7]; u.c[7] = t;
		t = u.c[1]; u.c[1] = u.c[6]; u.c[6] = t;
		t = u.c[2]; u.c[2] = u.c[5]; u.c[5] = t;
		t = u.c[3]; u.c[3] = u.c[4]; u.c[4] = t;
	}
	return u.d;
}

/* input_rt17 - Read an RT-17 mesasge from a raw data stream -----------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
rt17::rt17(){
}
rt17::~rt17(){
}

/* Get GPS week number ---------------------------------------------------- */
int rt17::GetWeek(double ttt){
	int www = 0;

	if (Flags & M_WEEK_OPTION){
		if ((ttt && Tow) && (ttt < Tow)) Week++;

		if (ttt != 0.0)
			Tow = ttt;
	}
	else if (!(Flags & M_WEEK_SCAN)){
		size_t ppp = opt.find("-WEEK=");

		Flags |= M_WEEK_SCAN;

		if (ppp!=string::npos){
			if (www>0){
				opt+=to_string(www);
				Week = www;
				Flags |= M_WEEK_OPTION;
			}
		}
	}

	www = Week;

	if (!www && !(Flags & (M_WEEK_OPTION|M_WEEK_EPH))){
		if ((time.time == 0) && (time.sec == 0.0)) time.timeget();
		time.time2gpst(&www);

		if (ttt != 0.0) time.gpst2time(www,ttt);
		Week = www;
		Flags |= M_WEEK_TIME;
	}

	return www;
}
/* Set GPS week number ---------------------------------------------------- */
void rt17::SetWeek(int www,double ttt){
	if (!(Flags & M_WEEK_OPTION)){
		if (!Week)
			Week = www;
	}

	/* Also update the time if we can */
	if (www && (ttt != 0.0))
		time.gpst2time(www,ttt);
}
/* Synchronize the raw data stream to the start of a series of RT-17 packets ---------------------- */
int rt17::SyncPacket(unsigned char Data){
	unsigned char Type;

	PacketBuffer[0] = PacketBuffer[1];
	PacketBuffer[1] = PacketBuffer[2];
	PacketBuffer[2] = PacketBuffer[3];
	PacketBuffer[3] = Data;

	Type = PacketBuffer[2];

	/*
	| Byte 0 must be an STX character.
	| Byte 1 = status byte which we always ignore (for now).
	| Byte 2 = packet type which must be GENOUT (0x40) RAWDATA (0x57) or RETSVDATA (0x55) (for now).
	| Byte 3 = data length which must be non-zero for any packet we're interested in.
	*/
	return ((PacketBuffer[0] == STX) && 
		(Data != 0) && 
		((Type == GENOUT) || (Type == RAWDATA) || (Type == RETSVDATA)));
}
/* ClearPacketBuffer - Clear the packet buffer ---------------------------------------------------- */
void rt17::ClearPacketBuffer(){
	int i;

	for (i = 0; i < 4; i++)
		PacketBuffer[i] = 0;

	PacketLength = PacketBytes = 0;
}
/* ClearMessageBuffer - Clear the raw data stream buffer ------------------ */
void rt17::ClearMessageBuffer(){
	int i;

	for (i = 0; i < 4; i++)
		MessageBuffer[i] = 0;

	MessageLength = MessageBytes = 0;
	Reply = 0;
}
/* Check the packet checksum -------------------------------------------------------------------------
The checksum is computed as the modulo 256 (unsigned 8-bit integer) sum
| of the packet contents starting with the status byte, including the
| packet type byte, length byte, data bytes and ending with the last byte
| of the data bytes. It does not include the STX leader, the ETX trailer
| nor the checksum byte.
* ------------------------------------------------------------------------------------------------- */
int rt17::CheckPacketChecksum(){
	unsigned char Checksum = 0;
	unsigned char *p = &PacketBuffer[1];        /* Starting with status */
	unsigned int Length = PacketBuffer[3] + 3; /* status, type, length, data */

	/* Compute the packet checksum */
	while (Length > 0){
		Checksum += *p++;
		Length--;
	}
	/*
	| Make sure our computed checksum matches the one at the end of the packet.
	| (Note that the above loop by design very conveniently left *p pointing
	|  to the checksum byte at the end of the packet.)
	*/
	return (Checksum == *p);
}

/* Decode a GPS Ephemeris record ---------------------------------------------------------------------
| See ICD-GPS-200C.PDF for documentation of the GPS satellite ephemeris.
| See reference #1 above for documentation of the RETSVDATA GPS Ephemeris. ------------------------ */
int rt17::DecodeGPSEphemeris(){
	unsigned char *p = PacketBuffer;
	int prn,sat,toc,ttt;
	unsigned int Flags,toe;
	double sqrtA;
	eph_t eph;

	if (PacketLength < 182){
		return -1;
	}

	prn = U1(p+5);
	if (!(sat=satno(SYS_GPS,prn))){
		return -1;
	}

	eph.week  = U2(p+6);    /* 006-007: Ephemeris Week number (weeks) */
	eph.iodc  = U2(p+8);    /* 008-009: IODC */
							/* Reserved byte */     /* 010-010: RESERVED */
	eph.iode  = U1(p+11);   /* 011-011: IODE */
	ttt       = I4(p+12);   /* 012-015: TOW */
	toc       = I4(p+16);   /* 016-019: TOC (seconds) */
	toe       = U4(p+20);   /* 020-023: TOE (seconds) */
	eph.tgd[0]= R8(p+24);   /* 024-031: TGD (seconds) */
	eph.f2    = R8(p+32);   /* 032-029: AF2 (seconds/seconds^2) */
	eph.f1    = R8(p+40);   /* 040-047: AF1 (seconds/seconds) */
	eph.f0    = R8(p+48);   /* 048-055: AF0 (seconds) */
	eph.crs   = R8(p+56);   /* 056-063: CRS (meters) */
	eph.deln  = R8(p+64);   /* 064-071: DELTA N (semi-circles/second) */
	eph.M0    = R8(p+72);   /* 072-079: M SUB 0 (semi-circles) */
	eph.cuc   = R8(p+80);   /* 080-087: CUC (semi-circles) */
	eph.e     = R8(p+88);   /* 088-095: ECCENTRICITY (dimensionless) */
	eph.cus   = R8(p+96);   /* 096-103: CUS (semi-circles) */
	sqrtA     = R8(p+104);  /* 104-111: SQRT A (meters ^ 0.5) */
	eph.cic   = R8(p+112);  /* 112-119: CIC (semi-circles) */
	eph.OMG0  = R8(p+120);  /* 120-127: OMEGA SUB 0 (semi-circles) */
	eph.cis   = R8(p+128);  /* 128-135: CIS (semi-circlces) */
	eph.i0    = R8(p+136);  /* 136-143: I SUB 0 (semi-circles) */
	eph.crc   = R8(p+144);  /* 144-151: CRC (meters) */
	eph.omg   = R8(p+152);  /* 152-159: OMEGA (semi-circles?) */
	eph.OMGd  = R8(p+160);  /* 160-167: OMEGA DOT (semi-circles/second) */
	eph.idot  = R8(p+168);  /* 168-175: I DOT (semi-circles/second) */
	Flags     = U4(p+176);  /* 176-179: FLAGS */

	/*
	| Multiply these by PI to make ICD specified semi-circle units into radian units.
	*/
	eph.deln *= SC2RAD;
	eph.i0   *= SC2RAD;
	eph.idot *= SC2RAD;
	eph.M0   *= SC2RAD;
	eph.omg  *= SC2RAD;
	eph.OMG0 *= SC2RAD;
	eph.OMGd *= SC2RAD;

	/*
	| As specifically directed to do so by Reference #1, multiply these by PI.
	| to make semi-circle units into radian units, which is what ICD-GPS-200C
	| calls.
	*/
	eph.cic *= SC2RAD;
	eph.cis *= SC2RAD;
	eph.cuc *= SC2RAD;
	eph.cus *= SC2RAD;

	/*
	| Select the correct curve fit interval as per ICD-GPS-200 sections
	| 20.3.3.4.3.1 and 20.3.4.4 using IODC, fit flag and Table 20-XII.
	*/
	/* Subframe 2, word 10, bit 17 (fit flag) */
	if (Flags & M_BIT10)  {
		if ((eph.iodc >= 240) && (eph.iodc <= 247))
			eph.fit = 8;
		else if (((eph.iodc >= 248) && (eph.iodc <= 255)) || (eph.iodc == 496))
			eph.fit = 14;
		else if ((eph.iodc >= 497) && (eph.iodc <= 503))
			eph.fit = 26;
		else if ((eph.iodc >= 504) && (eph.iodc <= 510))
			eph.fit = 50;
		else if ((eph.iodc == 511) || ((eph.iodc >= 752) && (eph.iodc <= 756)))
			eph.fit = 74;
		else if ((eph.iodc >= 757) && (eph.iodc <= 763))
			eph.fit = 98;
		else if (((eph.iodc >= 764) && (eph.iodc <= 767)) || ((eph.iodc >= 1008) && (eph.iodc <= 1010)))
			eph.fit = 122;
		else if ((eph.iodc >= 1011) && (eph.iodc <= 1020))
			eph.fit = 146;
		else
			eph.fit = 6;
	}
	else
		eph.fit = 4;

	eph.flag  = (Flags & M_BIT0);   /* Subframe 1, word 4, bit 1, Data flag for L2 P-code */
	eph.code  = (Flags >> 1) & 3;   /* Subframe 1, word 3, bits 11-12, Codes on L2 channel */
	eph.svh   = (Flags >> 4) & 127; /* Subframe 1, word 3, bits 17-22, SV health from ephemeris */
	eph.sva   = (Flags >> 11) & 15; /* Subframe 1, word 3, bits 13-16, User Range Accuracy index */
	eph.A     = sqrtA * sqrtA;
	eph.toes  = toe;
	eph.toc.gpst2time(eph.week,toc);
	eph.toe.gpst2time(eph.week,toe);
	eph.ttr.gpst2time(eph.week,ttt);

	if (!(Flags & M_WEEK_OPTION)){
		if (!Week || (Flags & M_WEEK_TIME) || (eph.week > Week)){
			Flags &= ~M_WEEK_TIME;
			Flags |= M_WEEK_EPH;
			Week = eph.week;
		}
	}
	if (opt.find("-EPHALL")==string::npos){
		if (eph.iode == nav.eph[sat-1].iode)
			return 0; /* unchanged */
	}
	eph.sat = sat;
	nav.eph[sat-1] = eph;
	ephsat = sat;
	return 2;
}
/* Decode an ION / UTC data record ---------------------------------------------------------------- */
int rt17::DecodeIONAndUTCData(){
	int www;
	unsigned char *p = PacketBuffer;

	if (PacketLength < 129){
		return -1;
	}

	/* ION / UTC data does not have the current GPS week number. Punt! */
	www = GetWeek(0.0);

	nav.ion_gps[0] = R8(p+6);  /* 006-013: ALPHA 0 (seconds) */
	nav.ion_gps[1] = R8(p+14); /* 014-021: ALPHA 1 (seconds/semi-circle) */
	nav.ion_gps[2] = R8(p+22); /* 022-029: ALPHA 2 (seconds/semi-circle)^2 */
	nav.ion_gps[3] = R8(p+30); /* 030-037: ALPHA 3 (seconds/semi-circle)^3 */
	nav.ion_gps[4] = R8(p+38); /* 038-045: BETA 0  (seconds) */
	nav.ion_gps[5] = R8(p+46); /* 046-053: BETA 1  (seconds/semi-circle) */
	nav.ion_gps[6] = R8(p+54); /* 054-061: BETA 2  (seconds/semi-circle)^2 */
	nav.ion_gps[7] = R8(p+62); /* 062-069: BETA 3  (seconds/semi-circle)^3 */
	nav.utc_gps[0] = R8(p+70); /* 070-077: ASUB0   (seconds)*/
	nav.utc_gps[1] = R8(p+78); /* 078-085: ASUB1   (seconds/seconds) */
	nav.utc_gps[2] = R8(p+86); /* 086-093: TSUB0T */
	nav.utc_gps[3] = www;
	nav.leaps =(int)R8(p+94); /* 094-101: DELTATLS (seconds) */
	/* Unused by R8 */   /* 102-109: DELTATLSF */
	/* Unused by R8 */   /* 110-117: IONTIME */
	/* Unused by U1 */   /* 118-118: WNSUBT */
	/* Unused by U1 */   /* 119-119: WNSUBLSF */
	/* Unused by U1 */   /* 120-120: DN */
	/* Reserved six bytes */    /* 121-126: RESERVED */

	return 9;
}
/* Decode a GLONASS Ephemeris record -------------------------------------- */
int rt17::DecodeGLONASSEphemeris(){ return 0; }
/* Decode a Galileo Ephemeris record -------------------------------------- */
int rt17::DecodeGalileoEphemeris(){ return 0; }
/* Decode a QZSS Ephemeris record ----------------------------------------- */
int rt17::DecodeQZSSEphemeris(){ return 0; }
/* Decode a Beidou Ephemeris record --------------------------------------- */
int rt17::DecodeBeidouEphemeris(){ return 0; }
/* Reassemble message by removing packet headers, trailers and page framing */
void rt17::UnwrapRawdata(unsigned int &rif){
	unsigned char *p_in = MessageBuffer;
	unsigned char *p_out = p_in;
	unsigned int InputLength,InputLengthTotal = MessageLength;
	unsigned int OutputLength,OutputLengthTotal = 0;

	rif = p_in[7];

	while (InputLengthTotal > 0){
		if ((unsigned int)p_in[7] != rif)

		InputLength = p_in[3] + 6;
		OutputLength = p_in[3] - 4;
		memmove(p_out,p_in + 8,OutputLength);
		p_in += InputLength;
		p_out += OutputLength;
		OutputLengthTotal += OutputLength;
		InputLengthTotal -= InputLength;
	}
	MessageBytes = MessageLength = OutputLengthTotal;
}
/* Decode Real-Time survey data (record type 17) -------------------------- */
int rt17::DecodeType17(unsigned int rif){
	unsigned char *p = MessageBuffer;
	double ClockOffset,ttt;
	int Flags1,Flags2,FlagStatus,i,n,nsat,prn,www;
	gtime_t TT;

	ttt = R8(p) * 0.001; p += 8;         /* Receive time within the current GPS week. */
	ClockOffset = R8(p) * 0.001; p += 8; /* Clock offset value. 0.0 = not known */

#if 0
	ttt += ClockOffset;
#endif

	/* The observation data does not have the current GPS week number. Punt! */
	www = GetWeek(ttt);
	TT.gpst2time(www,ttt);

	nsat = U1(p); p++; /* Number of SV data blocks in the record */

	for (i = n = 0; (i < nsat) && (i < MAXOBS); i++){
		obs.data[n].reset();
		obs.data[n].time = TT;

		if (rif & M_CONCISE){
			/* Satellite number (1-32). */
			prn = U1(p);
			p++;

			/* These indicate what data is loaded, is valid, etc */
			Flags1 = U1(p);
			p++;
			Flags2 = U1(p);
			p++;

			/* These are not needed */
			p++;    /* I1 Satellite Elevation Angle (degrees) */
			p += 2; /* I2 Satellite Azimuth (degrees) */

					/* L1 data valid */
			if (Flags1 & M_BIT6) {
				/* Measure of L1 signal strength (dB * 4) */
				obs.data[n].SNR[0] = U1(p);
				p++;

				/* Full L1 C/A code or P-code pseudorange (meters) */
				obs.data[n].P[0] = R8(p);
				p += 8;

				/*  L1 Continuous Phase (cycles) */
				if (Flags1 & M_BIT4) /* L1 phase valid */
					obs.data[n].L[0] = -R8(p);
				p += 8;

				/* L1 Doppler (Hz) */
				obs.data[n].D[0] = R4(p);
				p += 4;
			}

			/* L2 data loaded */
			if (Flags1 & M_BIT0)  {
				/* Measure of L2 signal strength (dB * 4) */
				obs.data[n].SNR[1] = U1(p);
				p++;

				/* L2 Continuous Phase (cycles) */
				if (Flags1 & M_BIT5)
					obs.data[n].L[1] = -R8(p);
				p += 8;

				/* L2 P-Code or L2 Encrypted Code */
				if (Flags1 & M_BIT5) /* L2 range valid */
					obs.data[n].P[1] = obs.data[n].P[0] + R4(p);
				p += 4;
			}

			/*
			| We can't use the IODE flags in this context.
			| We already have slip flags and don't need slip counters.
			*/
			if (rif & M_ENHANCED){
				p++; /* U1 IODE, Issue of Data Ephemeris */
				p++; /* U1 L1 cycle slip roll-over counter */
				p++; /* U1 L2 cycle slip roll-over counter */
			}
		}
		else /* Expanded Format */{
			/* Satellite number (1-32) */
			prn = U1(p);
			p++;

			/* These indicate what data is loaded, is valid, etc */
			Flags1 = U1(p);
			p++;
			Flags2 = U1(p);
			p++;

			/* Indicates whether FLAGS1 bit 6 and FLAGS2 are valid */
			FlagStatus = U1(p);
			p++;

			/* These are not needed */
			p += 2; /* I2 Satellite Elevation Angle (degrees) */
			p += 2; /* I2 Satellite Azimuth (degrees) */

					/*
					| FLAG STATUS bit 0 set   = Bit 6 of FLAGS1 and bit 0-7 of FLAGS2 are valid.
					| FLAG STATUS bit 0 clear = Bit 6 of FLAGS1 and bit 0-7 of FLAGS2 are UNDEFINED.
					|
					| According to reference #1 above, this bit should ALWAYS be set
					| for RAWDATA. If this bit is not set, then we're lost and cannot
					| process this message any further.
					*/
			if (!(FlagStatus & M_BIT0)) /* Flags invalid */
				return 0;

			/* L1 data valid */
			if (Flags1 & M_BIT6) {
				/* Measure of satellite signal strength (dB) */
				obs.data[n].SNR[0] = R8(p) * 4.0;
				p += 8;

				/* Full L1 C/A code or P-code pseudorange (meters) */
				obs.data[n].P[0] = R8(p);
				p += 8;

				/* L1 Continuous Phase (cycles) */
				if (Flags1 & M_BIT4) /* L1 phase valid */
					obs.data[n].L[0] = -R8(p);
				p += 8;

				/* L1 Doppler (Hz) */
				obs.data[n].D[0] = R8(p);
				p += 8;

				/* Reserved 8 bytes */
				p += 8;
			}

			/* L2 data loaded */
			if (Flags1 & M_BIT0) {
				/* Measure of L2 signal strength (dB) */
				obs.data[n].SNR[1] = R8(p) * 4.0;
				p += 8;

				/* L2 Continuous Phase (cycles) */
				if (Flags1 & M_BIT5) /* L2 phase valid */
					obs.data[n].L[1] = -R8(p);
				p += 8;

				/* L2 P-Code or L2 Encrypted Code */
				if (Flags1 & M_BIT5) /* L2 pseudorange valid */
					obs.data[n].P[1] = obs.data[n].P[0] + R8(p);
				p += 8;
			}

			if (rif & M_ENHANCED){
				/*
				| We can't use the IODE flags in this context.
				| We already have slip flags and don't need slip counters.
				*/
				p++; /* U1 IODE, Issue of Data Ephemeris */
				p++; /* U1 L1 cycle slip roll-over counter */
				p++; /* U1 L2 cycle slip roll-over counter */
				p++; /* U1 Reserved byte */

					 /* L2 Doppler (Hz) */
				obs.data[n].D[1] = R8(p);
				p += 8;
			}
		}

		obs.data[n].code[0] = 
			(obs.data[n].P[0] == 0.0) ? CODE_NONE : (Flags2 & M_BIT0) ? CODE_L1P : CODE_L1C;
		obs.data[n].code[1] = 
			(obs.data[n].P[1] == 0.0) ? CODE_NONE : (Flags2 & M_BIT2) ? CODE_L2W : (Flags2 & M_BIT1) ? 
			CODE_L2P : CODE_L2C;

		if (Flags1 & M_BIT1)
			obs.data[n].LLI[0] |= 1;  /* L1 cycle slip */

		if (Flags1 & M_BIT2)
			obs.data[n].LLI[1] |= 1;  /* L2 cycle slip */
		if ((Flags2 & M_BIT2) && (obs.data[n].P[1] != 0.0))
			obs.data[n].LLI[1] |= 4; /* Tracking encrypted code */

		if (!(obs.data[n].sat = satno(SYS_GPS,prn))){
			continue;
		}

#if 0
		/* Apply clock offset to observables */
		if (ClockOffset != 0.0){
			obs.data[n].P[0] += ClockOffset * (CLIGHT/FREQ1);
			obs.data[n].P[1] += ClockOffset * (CLIGHT/FREQ2);
			obs.data[n].L[0] += ClockOffset * FREQ1;
			obs.data[n].L[1] += ClockOffset * FREQ2;
		}
#endif
		n++;
	}

	time = TT;
	obs.n = n;

	return (n > 0);
}
/* DecodeType29 - Decode Enhanced position (record type 29) --------------- */
int rt17::DecodeType29(){
	unsigned char *p = MessageBuffer;

	if (*p >= 7)
		SetWeek(I2(p+1),((double)I4(p+3)) * 0.001);
	return 0;
}
/* Decode a Position Time GSOF message ------------------------------------ */
int rt17::DecodeGSOF1(unsigned char *p){
	if (p[1] >= 6)
		SetWeek(I2(p+6),((double)I4(p+2)) * 0.001);
	return 0;
}
/* Decode an ECEF Position GSOF message ----------------------------------- */
int rt17::DecodeGSOF3(unsigned char *p){

	if (p[1] >= 24){
		sta.pos[0] = R8(p+2);
		sta.pos[1] = R8(p+10);
		sta.pos[2] = R8(p+18);
		sta.del[0] = 0.0;
		sta.del[1] = 0.0;
		sta.del[2] = 0.0;
		sta.hgt    = 0.0;
		sta.deltype = 0;  /* e/n/u */
	}

	return 5;
}
/* Decode a Receiver Serial Number GSOF message --------------------------- */
int rt17::DecodeGSOF15(unsigned char *p){
	if (p[1] >= 15)
		sta.recsno=to_string(U4(p+2));

	return 0;
}
/* Decode a Current Time GSOF message ------------------------------------- */
int rt17::DecodeGSOF16(unsigned char *p){
	if ( p[1]>=9 && (U1(p+10) & M_BIT0)) /* If week and milliseconds of week are valid */
		SetWeek(I2(p+6),((double)I4(p+2)) * 0.001);
	return 0;
}
/* Decode a Position Time UTC GSOF message -------------------------------- */
int rt17::DecodeGSOF26(unsigned char *p){
	if (p[1] >=  6)
		SetWeek(I2(p+6),((double)I4(p+2)) * 0.001);
	return 0;
}
/* Decode a Base Position and Quality Indicator GSOF message -------------- */
int rt17::DecodeGSOF41(unsigned char *p){
	if (p[1] >= 6)
		SetWeek(I2(p+6),((double)I4(p+2)) * 0.001);
	return 0;
}

/* Decode an SVDATA packet ------------------------------------------------ */
int rt17::DecodeRetsvdata(){
	int Ret = 0;
	string Subtype_s="\n";
	unsigned char Subtype = PacketBuffer[4];

	if (Subtype < (sizeof(RetsvdataTable) / sizeof(char*)))
		Subtype_s = RetsvdataTable[Subtype];

	if (Subtype_s.compare("\n")==0)
		Subtype_s = "Unknown";

	/* Process (or possibly ignore) the message */
	switch (Subtype){
		case 1:	Ret = DecodeGPSEphemeris();		break;
		case 3:	Ret = DecodeIONAndUTCData();	break;
		case 9:	Ret = DecodeGLONASSEphemeris();	break;
		case 11:Ret = DecodeGalileoEphemeris();	break;
		case 14:Ret = DecodeQZSSEphemeris();	break;
		case 21:Ret = DecodeBeidouEphemeris();	break;
		default:;
	}
	return Ret;
}
/* Decode an RAWDATA packet sequence -------------------------------------- */
int rt17::DecodeRawdata(){
	int Ret = 0;
	unsigned int rif;
	string RecordType_s = "\n";
	unsigned char RecordType = MessageBuffer[4];

	if (RecordType < (sizeof(RawdataTable) / sizeof(char*)))
		RecordType_s = RawdataTable[RecordType];

	if (RecordType_s.compare("\n")==0)
		RecordType_s = "Unknown";

	/*
	| Reassemble origional message by removing packet headers,
	| trailers and page framing.
	*/
	UnwrapRawdata(rif);

	/* Process (or possibly ignore) the message */
	switch (RecordType){
		case 0:Ret = DecodeType17(rif);	break;
		case 7:Ret = DecodeType29();	break;
		default:;
	}
	return Ret;
}
/* DecodeGSOF - Decode a General Serial Output Format (GSOF) message ------ */
int rt17::DecodeGSOF(){
	return 0;
}

/* input_rt17 - Read an RT-17 mesasge from a raw data stream -------------------------------------- */
int rt17::decode(unsigned char data){
	unsigned int Page,Pages,Reply;
	int Ret = 0;
	char msg[60];

	/* If no current packet */
	if (PacketBytes == 0){
		/* Find something that looks like a packet. */
		if (SyncPacket(data)){
			/* Found one. */
			PacketLength = 4 + PacketBuffer[3] + 2; /* 4 (header) + length + 2 (trailer) */
			PacketBytes = 4; /* We now have four bytes in the packet buffer */
		}
		/* Continue reading the rest of the packet from the stream */
		return 0;
	}

	/* Store the next byte of the packet */
	PacketBuffer[PacketBytes++] = data;
	/*
	| Keep storing bytes into the current packet
	| until we have what we think are all of them.
	*/
	if (PacketBytes < PacketLength)
		return 0;
	/*
	| At this point we think have an entire packet.
	| The prospective packet must end with an ETX.
	*/
	if (PacketBuffer[PacketLength-1] != ETX){
		ClearPacketBuffer();
		return 0;
	}
	/*
	| We do indeed have an entire packet.
	| Check the packet checksum.
	*/
	if (!CheckPacketChecksum()){
		ClearPacketBuffer();
		return 0;
	}
	if (outtype){
		sprintf(msg, "RT17 0x%02X (%4d)",PacketBuffer[2],PacketLength);
		msgtype=msg;
	}
	/* If this is a SVDATA packet, then process it immediately */
	if (PacketBuffer[2] == RETSVDATA){
		Ret = DecodeRetsvdata();
		ClearPacketBuffer();
		return Ret;
	}
	/* Accumulate a sequence of RAWDATA packets (pages) */
	if (PacketBuffer[2] == RAWDATA){
		Page  = PacketBuffer[5] >> 4;
		Pages = PacketBuffer[5] & 15;
		Reply = PacketBuffer[6];
		/*
		| If this is the first RAWDATA packet in a sequence of RAWDATA packets,
		| then make sure it's page one and not a packet somewhere in the middle.
		| If not page one, then skip it and continue reading from the stream
		| until we find one that starts at page one. Otherwise make sure it is
		| a part of the same requence of packets as the last one, that it's
		| page number is in sequence.
		*/
		if (MessageBytes == 0){
			if (Page != 1){
				ClearPacketBuffer();
				return 0;
			}
			Reply = PacketBuffer[6];
		}
		else if ((Reply != Reply) || (Page != (Page + 1))){
			ClearMessageBuffer();
			ClearPacketBuffer();
			return 0;
		}

		/* Check for message buffer overflow */
		if ((MessageBytes + PacketBytes) > MBUFF_LENGTH){
			ClearMessageBuffer();
			ClearPacketBuffer();
			return 0;
		}

		memcpy(MessageBuffer + MessageBytes,PacketBuffer,PacketBytes);
		MessageBytes += PacketBytes;
		MessageLength += PacketLength;
		ClearPacketBuffer();

		if (Page == Pages){
			Ret = DecodeRawdata();
			ClearMessageBuffer();
			return Ret;
		}
		Page = Page;
		return 0;
	}

	/* Accumulate a sequence of GENOUT (GSOF) packets (pages) */
	if (PacketBuffer[2] == GENOUT){
		Reply = PacketBuffer[4];
		Page  = PacketBuffer[5];
		Pages = PacketBuffer[6];
		/*
		| If this is the first GENOUT packet in a sequence of GENOUT packets,
		| then make sure it's page zero and not a packet somewhere in the middle.
		| If not page zero, then skip it and continue reading from the stream
		| until we find one that starts at page zero. Otherwise make sure it is
		| a part of the same requence of packets as the last one, that it's
		| page number is in sequence.
		*/
		if (MessageBytes == 0){
			if (Page != 0){
				ClearPacketBuffer();
				return 0;
			}
			Reply = PacketBuffer[4];
		}
		else if ((Reply != Reply) || (Page != (Page + 1))){
			ClearMessageBuffer();
			ClearPacketBuffer();
			return 0;
		}

		/* Check for message buffer overflow. */
		if ((MessageBytes + PacketBytes) > MBUFF_LENGTH){
			ClearMessageBuffer();
			ClearPacketBuffer();
			return 0;
		}

		memcpy(MessageBuffer + MessageBytes,PacketBuffer,PacketBytes);
		MessageBytes += PacketBytes;
		MessageLength += PacketLength;
		ClearPacketBuffer();

		if (Page == Pages){
			Ret = DecodeGSOF();
			ClearMessageBuffer();
			return Ret;
		}
		Page = Page;
		return 0;
	}
	/*
	| If we fall through to here, then the packet is not one that we support
	| (and hence we can't really even get here). Dump the packet on the floor
	| and continue reading from the stream.
	*/
	ClearPacketBuffer();
	return 0;
}