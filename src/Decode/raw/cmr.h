/*------------------------------------------------------------------------------
* cmr.c : CMR dependent functions
*
*          Copyright (C) 2016 Daniel A. Cook, All rights reserved.
*
* references:
*     [1] https://github.com/astrodanco/RTKLIB/tree/cmr/src/cmr.c
*
* version : $Revision:$ $Date:$
*-----------------------------------------------------------------------------*/
/*
| CMR protocol stream and file handler functions.
|
| Written in June 2016 by Daniel A. Cook, for inclusion into the RTKLIB library.
| Copyright (C) 2016 by Daniel A. Cook. All Rights Reserved.
|
| The Compact Measurement Record Format (CMR) is a de facto industry standard
| reference station data transmission protocol for RTK positioning. Despite
| the availability of practical alternatives such RTCM v3.1, CMR and CMR+
| still remain de facto industry standards, especially in the US market.
|
| Here we implement five public functions, one for reading CMR format streams
| and another for reading CMR format files, a third to supply rover observations
| to the CMR base observations referencing engine and a fourth to initialize
| CMR related memory storage and a fifth to free up CMR related memory storage.
|
| Although a Trimble proprietary protocol, CMR was documented in reference #1
| and CMR+ was documented in reference #2. These were then adopted by many
| other manufacurers in addition to Trimble. Leica extended the protocol to
| handle GLONASS observations and other manufacturers adopted Leica's extention,
| but there was apparently a disconnect with Trimble on how Trimble implemented
| the same extension making GLONASS compatibility between Trimble and non-
| Trimble receivers problematic whenever the CMR protocol is used. Note that
| so far as the author knows, Trimble never documented their own GLONASS
| extension to the protocol. It is not implemented here.
|
| RTCM3 should always be used whenever possible in lieu of CMR. Use CMR only
| when there is no other option.
|
| Receiver dependent options:
|
| -STA=nn - Set the base station ID to receive (0-31 for CMR).
|
| Notes:
|
| CMR message types 0, 1 and 2 are implemented as described by reference #1.
|
| CMR message type 3 (GLONASS) is implemented as described by reference #3,
| but I don't have suitable receivers with which to test and debug it. This
| message is the Leica (or rather non-Trimble since other manufacturers also
| output it) implementation of GLONASS base observations for CMR.
|
| CMR message type 4 is high speed CMR (5, 10, or 20 Hz). Support for this
| message is limited to utilizing the GPS L1 carrier phase observables
| contained therein.
|
| CMR+ message types 1, 2 and 3 are implemented. I have not yet been able
| to obtain reference #2 which presumably describes CMR+ in detail.
|
| Ag scrambled CMR+ (sCMR+) is not implemented.
|
| CMRW (message number 0x098) aka CMR-W is not implemented.
|
| CMRx (Compressed Measurement Record Extended) and sCMRx (scrambled CMRx)
| are patented (see the patents) therefore are absolutely positively not
| implemented!
|
| If you have a copy of reference #2, please be kind and send the author
| a copy. In fact if you have any technical information concerning any of
| the CMR family protocols the author would appreciate if you would share
| it with him.
|
| Both stream and file input of raw binary CMR base data are supported.
|
| Design notes:
|
| CMR messages  increment ????[]
| CMR+ messages increment ????[]
|
| Note that because the L1 pseudorange is transmitted modulo one light
| millisecond, the base to rover distance must be less than 300 KM.
| CMR will not work at all beyond that distance.
|
| Note that CMR observables are not standalone. Before being utilized as
| base observations they must first be "referenced" to closely matching
| (in time) rover L1 pseudorange observables. Thus they are of little to
| no practical use in the absence of matching rover observations.
|
| CMR does not stream any almanac or ephemeris data. Only observations
| and reference station parameters. Ephemeris data must be obtained from
| the rover or from other sources.
|
| The time in the CMR header is relative rather than absolute. It is also
| ambiguous. It is GPS time in milliseconds modulo four minutes (240,000
| milliseconds or 240 seconds). In a rover receiver this would be aligned
| based upon the rover's current GPS time. We do the same here. Absolute
| time comes from the rover observations. Note that with CMR message type
| 4 (high speed observations) the time is modulo four seconds (4000 milli-
| seconds). Also note that according to reference #3 the time in CMR type
| 3 messages is UTC time, not GPS or GLONASS time. We convert it to GPS time.
|
| This code was tested using CMR/CMR+ data output by the following receivers:
|
| 1. Trimble 4000SSI, firmware version 7.32
| 2. Trimble 5700, firmware version 2.32
| 3. Spectra Precision Epoch 25 Base, firmware version 2.32
|
| For testing purposes one would normally use the Trimble GPS Configurator utility
| to configure the test receiver, but that utility does not understand all receiver
| features such as CMR 5Hz and 10Hz output. The Trimble WINPAN utility was used
| to configure CMR 5Hz and 10Hz output.
|
| For the purposes of this explaination the following terms mean the following things:
|
| C     299,792,458     One light second in meters.
| CMS   C / 1000        One light millisecond in meters.
| L1F   1,574,420,000   L1 frequency in cycles per second.
| L2F   1,227,600,000   L2 frequency in cycles per second.
| L1W   C / L1F         L1 wavelength in meters.
| L2W   C / L2F         L2 wavelength in meters.
|
| L1P   L1 pseudorange.
| L2P   L2 pseudorange.
| L1L   L1 carrier phase.
| L2L   L2 carrier phase.
|
| BT    Base time in millseconds.
| BTS   Base time in seconds.
| BP1   Base L1 pseudorange in meters.
| BL1   Base L1 carrier phase in L1 cycles per second.
| BP2   Base L2 pseudorange in meters.
| BL2   Base L2 carrier phase in L2 cycles per second.
|
| CBS   Base time in milliseconds modulo 240,000.
| CP1   Base L1 pseudorange modulo CMS in 1/8 L1 cycles.
| CL1   Base L1 carrier phase delta in 1/256 L1 cycles.
| CP2   Base L2 pseudorange delta in centimeters.
| CL2   Base L2 carrier phase delta in 1/256 L2 cycles.
|
| RTS   Rover time in seconds.
| RP1   Rover L1 pseudorange in meters.
|
| UBS   Base time in seconds modulo 240.
| UP1   Base L1 pseudorange in meters modulo CMS.
| UL1   Base L1 carrier phase delta in L1 cycles.
| UP2   Base L2 pseudorange delta in meters.
| UL2   Base L2 carrier phase delta in L2 cycles.
|
| CMR type 0 messaages contain the following GPS observables for each satellite:
|
| 1. CBS    (actually just once in the message header)
| 2. CP1    8   * ((BP1 modulo CMS) / L1W)
| 3. CL1    256 * (BL1 - (BP1 / L1W))
| 4. CP2    100 * (BP2 - BP1)
| 5. CL2    256 * (BL2 - (BP1 / L2W))
|
| We temporarilly store them internally as "unreferenced" observables as follows:
|
| 1. UBS    CBS * 0.001     Milliseconds to seconds.
| 2. UP1    CP1 / 8 * L1W   1/8 L1 cycles to meters.
| 3. UL1    CL1 / 256       1/256 L1 cycles to cycles.
| 4. UP2    CP2 / 100       Centimeters to meters.
| 5. UL2    CL2 / 256       1/256 L2 cycles to cycles.
|
| These above "unreferenced" observables are output by RTKCONV and CONVBIN
| because they have no rover data to reference.
|
| Before use as base observables the above unreferenced observables must be
| referenced to closely corresponding (in time) rover observables as follows:
|
| 1. BTS    UBT + (RTS modulo 240)
| 2. BP1    UP1 + (RP1 - (RP1 modulo CMS))
| 3. BL1    UL1 + (BP1 / L1W)
| 4. BP2    UP2 + BP1
| 5. BL2    UL2 + (BP1 / L2W)
|
| Time in CMR type 0 messages is GPS time.
|
| CMR type 3 messages contain all the same information as CMR type 0 messages
| except that the carrier phase frequencies and wavelengths are those as for
| GLONASS. For GLONASS those things need to be looked up for every satellite
| and every signal.
|
| According to reference #3 the time in CMR type 3 messages is UTC time, not
| GPS time and not GLONASS time. We convert it to GPS time.
|
| CMR type 4 messages contain the following GPS observables for each satellite:
|
| 1. CBT (Actually just once in the messsage header and it's modulo 4000 instead of 240000.)
| 2. CL1 (But it's a delta against the prior CMR type 3 BL1 for this satellte.)
|
| CMR type 4 messages are only received with time represending the intervals
| between the seconds and never on an exact second.
|
| By convention functions within this source file appear in alphabetical order.
| Public functions appear as a set first (there are only three of them), followed
| by private functions as a set. Because of this, forward definitions are required
| for the private functions. Please keep that in mind when making changes to this
| source file.
|
| References:
|
| 1. Talbot, N.C., (1996), Compact Data Transmission Standard for
|    High-Precision GPS, in: Proc. of the 9th International Technical
|    Meeting of the Satellite Division of The Institute of Navigation,
|    Kansas City, Missouri, USA, September, pp. 861-871.
|
| 2. Talbot, N.C., (1997), Improvements in the Compact Measurement
|    Record Format, Trimble User?s Conference, San Jose, California,
|    pp. 322-337
|
| 3. A GLONASS Observation Message Compatible With The Compact
|    Measurement Record Format, Leica Geosystems AG,
|    <http://w3.leica-geosystems.com/downloads123/zz/gps/general
|    /white-tech-paper/GLONASS_Observation_Message_Specification.pdf>
|
| 4. Trimble Serial Reference Specification, Version 4.82, Revision A,
|    December 2013. Though not being in any way specific to the BD9xx
|    family of receivers, a handy downloadable copy of this document
|    is contained in the "Trimble OEM BD9xx GNSS Receiver Family ICD"
|    document located at <http://www.trimble.com/OEM_ReceiverHelp/
|    v4.85/en/BinaryInterfaceControlDoc.pdf>
|
| 5. RTKLIB Version 2.4.2 Manual, April 29 2013
|    <http://www.rtklib.com/prog/manual_2.4.2.pdf>
|
| 6. RTKLIB source code located at <https://github.com/tomojitakasu/RTKLIB>
*/

#ifndef CMR_H
#define CMR_H

#include "Decode/raw.h"

typedef struct {                    /* Rover observations cache data record */
	gtime_t       Time;             /* Rover observation time */
	double        P;                /* Rover L1 pseudorange (meters) */
	unsigned char Valid;            /* TRUE = Valid, FALSE = Invalid */
} obsr_t;
typedef struct {                    /* Base observables data record */
	double        P[2];             /* L1/L2 pseudoranges (meters) */
	double        L[2];             /* L1/L2 carrier-phases (cycles) */
	unsigned int  Slot;             /* Slot number */
	unsigned int Sat;              /* Satellite number */
	unsigned char Code[2];          /* L1/L2 code indicators (CODE_???) */
	unsigned char SNR[2];           /* L1/L2 signal strengths */
	unsigned char Slip[2];          /* L1/L2 slip counts */
	unsigned char LLI[2];           /* L1/L2 loss of lock indicators */
} obsbd_t;
typedef struct {                    /* Antenna number to name table record */
	unsigned short Number;          /* Antenna number */
	string         Name;           /* Antenna name */
} ant_t;

typedef struct {                    /* Receiver number to name table record */
	unsigned short Number;          /* Receiver number */
	string         Name;           /* Receiver name */
} rcv_t;
typedef struct {                    /* Base observables header record */
	gtime_t       Time;             /* Base observables time */
	int           n;                /* Number of observables */
	unsigned char Type;             /* Observables type (0, 3, 4) */
	obsbd_t       Data[MAXOBS];     /* Base observables data records */
} obsb_t;

/* input receiver raw data from stream ------------------------------------------------------------ */
class cmr : public raw_t{
	/* Constructor */
	public:
		cmr();
		~cmr();
	/* Implementation functions */
	protected:
		/* Synchronize the CMR data stream to the start of a series of CMR messages */
		int SyncMessage(unsigned char Data);
		/* CheckMessageFlags - Check for a message -------------------------------- */
		int CheckMessageFlags();
		/* Check the message checksum --------------------------------------------- */
		int CheckMessageChecksum();
		/* CheckCmrFlags - Check the CMR type 1 and 2 flags ----------------------- */
		void CheckCmrFlags(unsigned char *p);
		/* SetStationInfo - Set miscellaneous base station information ------------ */
		void SetStationInfo(unsigned char *p);
		/* SetStationCoordinates - Set the station coordinates -------------------- */
		void SetStationCoordinates(unsigned char *p);
		/* SetStationDescription - Set the station description -------------------- */
		void SetStationDescription(unsigned char *p,size_t Length);
		/* StatusReport - Output once a minute base status ------------------------ */
		void StatusReport();
		/* Reference and output a single CMR base observation --------------------- */
		int ReferenceCmrObs(gtime_t Time,unsigned char Type,double P0,obsbd_t *b);
		/* CheckStation - Check the Station ID number ----------------------------- */
		int CheckStation(unsigned int StationID);
		/* Output a set of CMR base observations ---------------------------------- */
		int OutputCmrObs(obsb_t *Obs);
		
		/* Decode a CMR message --------------------------------------------------- */
		int DecodeCmr();
		/* Decode a CMR+ message -------------------------------------------------- */
		int DecodeCmrPlus();
		/* Decode a set of buffered CMR+ messages --------------------------------- */
		int DecodeBuffer();
		/* Decode CMR GPS Observables --------------------------------------------- */
		int DecodeCmrType0();
		/* Decode CMR ECEF Reference Station Coordinates -------------------------- */
		int DecodeCmrType1();
		/* Decode CMR Reference Station Description ------------------------------- */
		int DecodeCmrType2();
		/* Decode CMR GLONASS Observables ----------------------------------------- */
		int DecodeCmrType3();
		/* Decode CMR High Speed Observables -------------------------------------- */
		int DecodeCmrType4();

		/* Update the CMR rover observations table -------------------------------- */
		int update_cmr();
	public:
		/* input receiver raw data from stream ------------------------------------ */
		virtual int decode(unsigned char data);

	/* Components */
	public:
		/* CMR informations */
		unsigned char Buffer[512];			/* Buffer for building full CMR+ message from little parts 
											512=BUFFER_LENGTH */
		unsigned char MessageBuffer[2048];	/* Message buffer 2048=MESSAGEBUFFER_LENGTH */
		obsr_t		RoverObservables[MAXSAT];/* Rover observables table */
		obsbd_t       T4Data[MAXOBS];		/* Type 3 reference data for type 4 observables */
		unsigned int  Flags;				/* Miscellaneous internal flag bits */
		unsigned int  CurrentMessages;		/* Current  base messages active */
		unsigned int  PreviousMessages;		/* Previous base messages active */
		unsigned int  BufferBytes;			/* Number of bytes of data in CMR+ message buffer */
		unsigned int  MessageBytes;			/* Number of bytes in message buffer */
		unsigned int  MessageLength;		/* Message Length */
		int           Page;					/* Previous page number added to CMR+ message mini-buffer */
		unsigned int  StationID;			/* Station ID */
		unsigned char SlipC[MAXSAT][2];		/* Slip counts */
		unsigned char SlipV[MAXSAT][2];		/* Slip counts valid indicator */
};

#endif