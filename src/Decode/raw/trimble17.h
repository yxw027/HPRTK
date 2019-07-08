/*------------------------------------------------------------------------------
* rt17.c : Trimble RT-17 dependent functions
*
*          Copyright (C) 2016 Daniel A. Cook, All rights reserved.
*
* references:
*     [1] https://github.com/astrodanco/RTKLIB/tree/cmr/src/rcv/rt17.c
*     [2] Trimble, Trimble OEM BD9xx GNSS Receiver Family IDC, version 4.82
*         Revision A, December, 2013
*
* version : $Revision:$ $Date:$
*-----------------------------------------------------------------------------*/
/*
| Trimble real-time binary data stream and file handler functions.
|
| Written in July 2014 by Daniel A. Cook, for inclusion into the library.
| Copyright (C) 2014, 2016 by Daniel A. Cook. All Rights Reserved.
|
| Here we implement four public functions, one for reading Trimble real-time
| binary data streams and another for reading log files containng data in the
| same format, a third to initialize RT17 related memory storage and a fourth
| to free up RT17 related memory storage. This real-time streaming data format,
| sometimes called RT-17, was designed by Trimble for use by Trimble receivers.
| Trimble receivers with the "Real-Time Survey Data" or "Binary Outputs" options
| can output this data. The RT-17 moniker refers to the GPS observables data
| record (record type 0) itself. There is also a GNSS observables data record
| (record type 6), which is referred to as RT-27. We also wish to handle RT-27
| data records, but lack sufficient documentation to do so.
|
| Notes:
|
| To specify receiver dependent options, set raw->opt to the following case
| sensitive option strings separated by spaces.
|
| Receiver dependent options:
|
| -EPHALL : Input all ephemerides
| -WEEK=n : Explicitly set starting GPS week number
|
| Neither the Trimble RT-17 observables packets nor the ION / UTC data packets
| contain the GPS week number. By default the current computer "now" time is
| used to determine the GPS week number. This works well in real time, but
| can be problematic when later converting and/or post processing recorded
| data. When recording data, also enable either GSOF Position & Time (1) or
| GSOF Current Time (16) messages on the same serial port along with the
| RT-17 Real-Time Survey Data output. For best results enable the GSOF
| message(s) and get them streaming prior to enabling the RT-17 messages.
|
| If desired the -WEEK=n option can be specified when converting or post
| processing recorded data to explicitly set the starting GPS week number.
| This option overrides anything and everything, including the current
| computer "now" time and any GSOF or other messages read from the raw
| data stream or recorded file. Note that the GPS week number explicitly
| specified with the -WEEK=n option GPS is automatically incremented when
| and if a subsequent GPS week number rollover occurs in the raw data.
|
| In addition to enabling RT-17 Real-Time Survey Data output, it is very
| helpful to also enable GSOF Position & Time (1) or GSOF Current Time (16)
| messages on the same serial port. This allows the GPS week number to be
| determined without using the current computer "now" time or the -WEEK=n
| option. Although not as important for real-time streaming data use where
| the current computer "now" time can be used to determine the current GPS
| week number, it becomes more important when recording files for later
| conversion and/or post processing.  For best results enable the GSOF
| message(s) and get them streaming prior to enabling the RT-17 messages.
|
| Support is provided for the following Trimble RT-17 packet Types:
|
|         Raw Observation   Satellite        ION/UTC
| Format       Data         Ephemerides      Parameters       GSOF
| ------- ---------------   ---------------- ---------------- ------------
| Trimble 0x57 (RAWDATA)    0x55 (RETSVDATA) 0x55 (RETSVDATA) 1, 16,
| RT-17   Recordtype 0 & 7  Subtype  1       Subtype 3        26, 41
|
| When the -WEEK=n option is NOT used, the GPS week number is set from any
| RAWDATA record type 7 or GENOUT (GSOF) 1, 16, 26, 41 records encountered
| in the raw data stream. These messages are only used to obtain the GPS
| WEEK number and are not used for any other purpose.
|
| Support is not provided for the GPS L2C or L5 signals. Those would likely
| require Trimble RT-27 protocol support. Support for that and much more
| could easily be added by anyone with the required RAWDATA record type
| 6 documentation.
|
| For Trimble GPS receivers which are capable of RT-17 binary output, the
| receiver and/or receiver configuration software generally provide several
| RT-17 binary output options:
|
| 1. Compact format aka Concise format
|    (RECOMMENDED)
|
| This option causes the raw satellite data to be streamed in a more compact
| format. The compact format does not include L2 DOPPLER observables.
|
| 2. Expanded format
|
| This is usually the default format if compact format is not enabled. The
| only advantage of this format over compact format is that L2 DOPPLER
| observables are output when used in combination with the Real Time
| Enhancements option. Otherwise this format just consumes more bandwidth
| and/or file space than compact format while offering no other advantages.
|
| 3. Real Time Enhancements, aka Real-time Enhanced format, aka R-T FLAGS
|
| This option adds extra data to the raw satellite data output by the
| receiver. When used in combination with expanded format, L2 DOPPLER
| observables are output. L2 DOPPLER can be used.
|
| 3. Measurements
|    (REQUIRED)
|
| If your configuration has a measurements option, enable it. Measurements
| are the raw satellite data. If you don't see this option then it is
| implied and enabled by default.
|
| 4. Ephermeris AKA Stream Ephemeris
|    (HIGHLY RECOMMENDED)
|
| This option causes satellite ephemerides and UTC / ION data to be streamed
| along with the raw satellite data. Streamed ephemerides and UTC / ION data
| consume very little extra bandwidth in the stream and/or space in a file.
| In most situations with most applications you will need them as well.
|
| 5. Positions AKA Stream Positions
|    (NOT RECOMMENDED)
|
| Streamed postions are of no use. They will be ignored. We
| Computes positions from the raw satellite data. It has no use for the
| receiver's position solutions. Streamed positions also consume
| considerable bandwidth in the stream and/or space in a file.
|
| 6. Positions Only
|    (HIGHLY NOT RECOMMENDED)
|
| Enabling the positions only option causes only positions and nothing else
| to be output, including no raw satellite data and no ephemerides and no
| ION / UTC data.
|
| Design notes:
|
| This source code handles GPS L1/L2 only. RT-17 is GPS. RT27 is GNSS.
| If you have RT27 (RAWDATA 57h record type 6) documentation, please
| forward it to the author.
|
| An RT-17 real-time survey data message is a series of RAWDATA (57h,
| Real-time survey data report) and RETSVDATA (55h, Satellite information
| report) packets.
|
| Each assembled RAWDATA message in an RT-17 packet stream may contain
| any of the following: Compact Format raw satellite measurements, Expanded
| Format raw satellite measurements, a receiver computed position or an
| event mark. Receiver computed positions and event marks are of no
| interest, therefore we ignore them.
|
| Each RETSVDATA message in an RT-17 packet stream may contain any one
| of the following: SV flags indicating tracking, a GPS Ephemeris, a GPS
| Almanac, ION / UTC Data or an Extended GPS Almanac. Of these only
| the GPS Ephemeris and the ION / UTC Data are of interest.
| In practice only GPS Ephemeris and ION / UTC Data are transmitted.
| Some receivers can be set to transmit them at regular intervals
| rather than only when they change.
|
| Certain simplifying assumptions are made concerning the way in which
| RAWDATA and GENOUT packets are transmitted in the stream or stored
| into the file.
|
| Therefore it is assumed that:
|
| 1. RAWDATA and GENOUT packets are never interleaved or interspersed
|    with packets of other types.
|
| 2. The sequence of page frames in a RAWDATA message are transmitted in the
|    stream or stored into a file as packets in order from first to last.
|    RAWDATA page numbers are one based. That is, 1 of n, 2 of n, 3 of n,
|    ..., to 15 of 15 for a total of 15 possible pages. We check for this
|    ordering. RAWDATA messages can therefore reach almost 4K in total
|    length. We check for potential buffer overflows in the input_rt17()
|    function.
|
| 3. The Record Interpretation Flags (RIF) field is repeated within the
|    page frame of every page making up a single RAWDATA message. It is
|    assumed that this is redundant and that the actual value of the record
|    interpretation flags does not change from one page to the next within
|    a single RAWDATA message. We check for this too.
|
| 4. The sequence of pages in a GENOUT message are transmitted in the
|    stream or stored into a file as packets in order from first to last.
|    GENOUT page numbers are zero based. That is, 0 of n, 1 of n, 2 of n,
|    ..., to 255 of 255 for a total of 256 possible pages. We check for
|    this ordering. GENOUT messages can therefore reach almost 64K in
|    total length. Such a large GENOUT message could exceed our maximum
|    buffer size. We check for potential buffer overflows in the
|    input_rt17() function.
|
| This code was tested using RT-17 data output from the following receivers:
|
| 1. Trimble 4000SSI, firmware version 7.32
| 2. Trimble 5700, firmware version 2.32
| 3. Spectra Precision Epoch 25 Base, firmware version 2.32
|
| By convention functions within this source file appear in alphabetical
| order. Public functions appear as a set first (there are only two of
| them), followed by private functions as a set. Because of this, forward
| definitions are required for the private functions. Please keep that
| in mind when making changes to this source file.
|
| References:
|
| 1. Trimble Serial Reference Specification, Version 4.82, Revision A,
|    December 2013. Though not being in any way specific to the BD9xx
|    family of receivers, a handy downloadable copy of this document
|    is contained in the "Trimble OEM BD9xx GNSS Receiver Family ICD"
|    document located at <http://www.trimble.com/OEM_ReceiverHelp/
|    v4.85/en/BinaryInterfaceControlDoc.pdf>
|
| 2. Trimble General Serial Output Format (GSOF)
|    <http://www.trimble.com/OEM_ReceiverHelp/v4.85/en/GSOFmessages_GSOF.html>
|
| 3. ICD-GPS-200C, Interface Control Document, Revision C, 10 October 1993
|    <http://www.gps.gov/technical/icwg/ICD-GPS-200C.pdf>
|
| 4. IS-GPS-200H, Interface Specification, 24 September 2013
|    <http://www.gps.gov/technical/icwg/IS-GPS-200H.pdf>
|
| 5. RTKLIB Version 2.4.2 Manual, April 29 2013
|    <http://www.rtklib.com/prog/manual_2.4.2.pdf>
*/
#ifndef TRIMBLE17_H
#define TRIMBLE17_H

#include "Decode/raw.h"

/* input_rt17 - Read an RT-17 mesasge from a raw data stream -------------------------------------- */
class rt17 : public raw_t{
	/* Constructor */
	public:
		rt17();
		~rt17();
	/* Implementation functions */
	protected:
		/* Get GPS week number ---------------------------------------------------- */
		int GetWeek(double ttt);
		/* Set GPS week number ---------------------------------------------------- */
		void SetWeek(int www,double ttt);
		/* Synchronize the raw data stream to the start of a series of RT-17 packets */
		int SyncPacket(unsigned char Data);
		/* ClearPacketBuffer - Clear the packet buffer ---------------------------- */
		void ClearPacketBuffer();
		/* ClearMessageBuffer - Clear the raw data stream buffer ------------------ */
		void ClearMessageBuffer();
		/* Check the packet checksum ---------------------------------------------- */
		int CheckPacketChecksum();

		/* Decode a GPS Ephemeris record ------------------------------------------ */
		int DecodeGPSEphemeris();
		/* Decode an ION / UTC data record ---------------------------------------- */
		int DecodeIONAndUTCData();
		/* Decode a GLONASS Ephemeris record -------------------------------------- */
		int DecodeGLONASSEphemeris();
		/* Decode a Galileo Ephemeris record -------------------------------------- */
		int DecodeGalileoEphemeris();
		/* Decode a QZSS Ephemeris record ----------------------------------------- */
		int DecodeQZSSEphemeris();
		/* Decode a Beidou Ephemeris record --------------------------------------- */
		int DecodeBeidouEphemeris();
		/* Reassemble message by removing packet headers, trailers and page framing */
		void UnwrapRawdata(unsigned int &rif);
		/* Decode Real-Time survey data (record type 17) -------------------------- */
		int DecodeType17(unsigned int rif);
		/* DecodeType29 - Decode Enhanced position (record type 29) --------------- */
		int DecodeType29();
		/* Decode a Position Time GSOF message ------------------------------------ */
		int DecodeGSOF1(unsigned char *p);
		/* Decode an ECEF Position GSOF message ----------------------------------- */
		int DecodeGSOF3(unsigned char *p);
		/* Decode a Receiver Serial Number GSOF message --------------------------- */
		int DecodeGSOF15(unsigned char *p);
		/* Decode a Current Time GSOF message ------------------------------------- */
		int DecodeGSOF16(unsigned char *p);
		/* Decode a Position Time UTC GSOF message -------------------------------- */
		int DecodeGSOF26(unsigned char *p);
		/* Decode a Base Position and Quality Indicator GSOF message -------------- */
		int DecodeGSOF41(unsigned char *p);

		/* Decode an SVDATA packet ------------------------------------------------ */
		int DecodeRetsvdata();
		/* Decode an RAWDATA packet sequence -------------------------------------- */
		int DecodeRawdata();
		/* DecodeGSOF - Decode a General Serial Output Format (GSOF) message ------ */
		int DecodeGSOF();

	public:
		/* input_rt17 - Read an RT-17 mesasge from a raw data stream -------------- */
		virtual int decode(unsigned char data);

	/* Components */
	protected:
		unsigned char MessageBuffer[8192];/* Message buffer = MBUFF_LENGTH in trimble17.cpp */
		unsigned char PacketBuffer[261];/* Packet buffer = PBUFF_LENGTH in trimble17.cpp */
		double        Tow;              /* Receive time of week */
		unsigned int  Flags;            /* Miscellaneous internal flag bits */
		unsigned int  MessageBytes;     /* Number of bytes in message buffer */
		unsigned int  MessageLength;    /* Message length (bytes) */
		unsigned int  PacketBytes;      /* How many packet bytes have been read so far */
		unsigned int  PacketLength;     /* Total size of packet to be read */
		unsigned int  Page;             /* Last page number */
		unsigned int  Reply;            /* Current reply number */
		int           Week;             /* GPS week number */
};

#endif
