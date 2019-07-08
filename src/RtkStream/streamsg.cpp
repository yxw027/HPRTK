#include "RtkStream/stream.h"
/* ublox ------------------------------------------------------------------------------------------ */
#define FU1         1           /* ubx message field types */
#define FU2         2
#define FU4         3
#define FI1         4
#define FI2         5
#define FI4         6
#define FR4         7
#define FR8         8
#define FS32        9
#define UBXSYNC1    0xB5        /* ubx message sync code 1 */
#define UBXSYNC2    0x62        /* ubx message sync code 2 */
#define UBXCFG      0x06        /* ubx message cfg-??? */
/* set fields (little-endian) ----------------------------------------------------- */
static void setU1(unsigned char *p,unsigned char  u) { *p=u; }
static void setU2(unsigned char *p,unsigned short u) { memcpy(p,&u,2); }
static void setU4(unsigned char *p,unsigned int   u) { memcpy(p,&u,4); }
static void setI1(unsigned char *p,char           i) { *p=(unsigned char)i; }
static void setI2(unsigned char *p,short          i) { memcpy(p,&i,2); }
static void setI4(unsigned char *p,int            i) { memcpy(p,&i,4); }
static void setR4(unsigned char *p,float          r) { memcpy(p,&r,4); }
static void setR8(unsigned char *p,double         r) { memcpy(p,&r,8); }
static void setcs(unsigned char *buff,int len)
{
	unsigned char cka=0,ckb=0;
	int i;

	for (i=2; i<len-2; i++) {
		cka+=buff[i]; ckb+=cka;
	}
	buff[len-2]=cka;
	buff[len-1]=ckb;
}
/* generate ublox binary message -----------------------------------------------------
* generate ublox binary message from message string
* args   : char  *msg   IO     message string
*            "CFG-PRT   portid res0 res1 mode baudrate inmask outmask flags"
*            "CFG-USB   vendid prodid res1 res2 power flags vstr pstr serino"
*            "CFG-MSG   msgid rate0 rate1 rate2 rate3 rate4 rate5 rate6"
*            "CFG-NMEA  filter version numsv flags"
*            "CFG-RATE  meas nav time"
*            "CFG-CFG   clear_mask save_mask load_mask [dev_mask]"
*            "CFG-TP    interval length status time_ref res adelay rdelay udelay"
*            "CFG-NAV2  ..."
*            "CFG-DAT   maja flat dx dy dz rotx roty rotz scale"
*            "CFG-INF   protocolid res0 res1 res2 mask0 mask1 mask2 ... mask5"
*            "CFG-RST   navbbr reset res"
*            "CFG-RXM   gpsmode lpmode"
*            "CFG-ANT   flags pins"
*            "CFG-FXN   flags treacq tacq treacqoff tacqoff ton toff res basetow"
*            "CFG-SBAS  mode usage maxsbas res scanmode"
*            "CFG-LIC   key0 key1 key2 key3 key4 key5"
*            "CFG-TM    intid rate flags"
*            "CFG-TM2   ch res0 res1 rate flags"
*            "CFG-TMODE tmode posx posy posz posvar svinmindur svinvarlimit"
*            "CFG-EKF   ..."
*            "CFG-GNSS  ..."
*            "CFG-ITFM  conf conf2"
*            "CFG-LOGFILTER ver flag min_int time_thr speed_thr pos_thr"
*            "CFG-NAV5  ..."
*            "CFG-NAVX5 ..."
*            "CFG-ODO   ..."
*            "CFG-PM2   ..."
*            "CFG-PWR   ver rsv1 rsv2 rsv3 state"
*            "CFG-RINV  flag data ..."
*            "CFG-SMGR  ..."
*            "CFG-TMODE2 ..."
*            "CFG-TMODE3 ..."
*            "CFG-TPS   ..."
*            "CFG-TXSLOT ..."
*          unsigned char *buff O binary message
* return : length of binary message (0: error)
* note   : see reference [1][3] for details.
*          the following messages are not supported:
*             CFG-DOSC,CFG-ESRC
*---------------------------------------------------------------------------------- */
int gen_ubx(const char *msg,unsigned char *buff)
{
	const char *cmd[]={
		"PRT","USB","MSG","NMEA","RATE","CFG","TP","NAV2","DAT","INF",
		"RST","RXM","ANT","FXN","SBAS","LIC","TM","TM2","TMODE","EKF",
		"GNSS","ITFM","LOGFILTER","NAV5","NAVX5","ODO","PM2","PWR","RINV","SMGR",
		"TMODE2","TMODE3","TPS","TXSLOT",""
	};
	const unsigned char id[]={
		0x00,0x1B,0x01,0x17,0x08,0x09,0x07,0x1A,0x06,0x02,
		0x04,0x11,0x13,0x0E,0x16,0x80,0x10,0x19,0x1D,0x12,
		0x3E,0x39,0x47,0x24,0x23,0x1E,0x3B,0x57,0x34,0x62,
		0x36,0x71,0x31,0x53
	};
	const int prm[][32]={
		{ FU1,FU1,FU2,FU4,FU4,FU2,FU2,FU2,FU2 },    /* PRT */
		{ FU2,FU2,FU2,FU2,FU2,FU2,FS32,FS32,FS32 }, /* USB */
		{ FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1 },        /* MSG */
		{ FU1,FU1,FU1,FU1 },                        /* NMEA */
		{ FU2,FU2,FU2 },                            /* RATE */
		{ FU4,FU4,FU4,FU1 },                        /* CFG */
		{ FU4,FU4,FI1,FU1,FU2,FI2,FI2,FI4 },        /* TP */
		{ FU1,FU1,FU2,FU1,FU1,FU1,FU1,FI4,FU1,FU1,FU1,FU1,FU1,FU1,FU2,FU2,FU2,FU2,
		FU2,FU1,FU1,FU2,FU4,FU4 },                /* NAV2 */
		{ FR8,FR8,FR4,FR4,FR4,FR4,FR4,FR4,FR4 },    /* DAT */
		{ FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1 }, /* INF */
		{ FU2,FU1,FU1 },                            /* RST */
		{ FU1,FU1 },                                /* RXM */
		{ FU2,FU2 },                                /* ANT */
		{ FU4,FU4,FU4,FU4,FU4,FU4,FU4,FU4 },        /* FXN */
		{ FU1,FU1,FU1,FU1,FU4 },                    /* SBAS */
		{ FU2,FU2,FU2,FU2,FU2,FU2 },                /* LIC */
		{ FU4,FU4,FU4 },                            /* TM */
		{ FU1,FU1,FU2,FU4,FU4 },                    /* TM2 */
		{ FU4,FI4,FI4,FI4,FU4,FU4,FU4 },            /* TMODE */
		{ FU1,FU1,FU1,FU1,FU4,FU2,FU2,FU1,FU1,FU2 }, /* EKF */
		{ FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU4 },    /* GNSS */
		{ FU4,FU4 },                                /* ITFM */
		{ FU1,FU1,FU2,FU2,FU2,FU4 },                /* LOGFILTER */
		{ FU2,FU1,FU1,FI4,FU4,FI1,FU1,FU2,FU2,FU2,FU2,FU1,FU1,FU1,FU1,FU1,FU1,FU2,
		FU1,FU1,FU1,FU1,FU1,FU1 },                /* NAV5 */
		{ FU2,FU2,FU4,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU2,FU1,FU1,FU1,FU1,
		FU1,FU1,FU1,FU1,FU1,FU1,FU2 },            /* NAVX5 */
		{ FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1,FU1 },    /* ODO */
		{ FU1,FU1,FU1,FU1,FU4,FU4,FU4,FU4,FU2,FU2 }, /* PM2 */
		{ FU1,FU1,FU1,FU1,FU4 },                    /* PWR */
		{ FU1,FU1 },                                /* RINV */
		{ FU1,FU1,FU2,FU2,FU1,FU1,FU2,FU2,FU2,FU2,FU4 }, /* SMGR */
		{ FU1,FU1,FU2,FI4,FI4,FI4,FU4,FU4,FU4 },    /* TMODE2 */
		{ FU1,FU1,FU2,FI4,FI4,FI4,FU4,FU4,FU4 },    /* TMODE3 */
		{ FU1,FU1,FU1,FU1,FI2,FI2,FU4,FU4,FU4,FU4,FI4,FU4 }, /* TPS */
		{ FU1,FU1,FU1,FU1,FU4,FU4,FU4,FU4,FU4 }     /* TXSLOT */
	};
	unsigned char *q=buff;
	char mbuff[1024],*args[32],*p;
	int i,j,n,narg=0;

	strcpy(mbuff,msg);
	for (p=strtok(mbuff," "); p&&narg<32; p=strtok(NULL," ")) {
		args[narg++]=p;
	}

	if (narg<1||strncmp(args[0],"CFG-",4)) return 0;

	for (i=0; *cmd[i]; i++) {
		if (!strcmp(args[0]+4,cmd[i])) break;
	}
	if (!*cmd[i]) return 0;

	*q++=UBXSYNC1;
	*q++=UBXSYNC2;
	*q++=UBXCFG;
	*q++=id[i];
	q+=2;
	for (j=1; prm[i][j-1]||j<narg; j++) {
		switch (prm[i][j-1]) {
		case FU1: setU1(q,j<narg ? (unsigned char)atoi(args[j]) : 0); q+=1; break;
		case FU2: setU2(q,j<narg ? (unsigned short)atoi(args[j]) : 0); q+=2; break;
		case FU4: setU4(q,j<narg ? (unsigned int)atoi(args[j]) : 0); q+=4; break;
		case FI1: setI1(q,j<narg ? (char)atoi(args[j]) : 0); q+=1; break;
		case FI2: setI2(q,j<narg ? (short)atoi(args[j]) : 0); q+=2; break;
		case FI4: setI4(q,j<narg ? (int)atoi(args[j]) : 0); q+=4; break;
		case FR4: setR4(q,j<narg ? (float)atof(args[j]) : 0); q+=4; break;
		case FR8: setR8(q,j<narg ? (double)atof(args[j]) : 0); q+=8; break;
		case FS32: sprintf((char *)q, "%-32.32s",j<narg ? args[j] : ""); q+=32; break;
		default: setU1(q,j<narg ? (unsigned char)atoi(args[j]) : 0); q+=1; break;
		}
	}
	n=(int)(q-buff)+2;
	setU2(buff+4,(unsigned short)(n-8));
	setcs(buff,n);

	return n;
}
/* skytraq ---------------------------------------------------------------------------------------- */
#define STQSYNC1    0xA0        /* skytraq binary sync code 1 */
#define STQSYNC2    0xA1        /* skytraq binary sync code 2 */

#define ID_STQTIME  0xDC        /* skytraq message id: measurement epoch */
#define ID_STQRAW   0xDD        /* skytraq message id: raw measurement */
#define ID_STQGPS   0xE0        /* skytraq message id: gps/qzs subframe */
#define ID_STQGLO   0xE1        /* skytraq message id: glonass string */
#define ID_STQGLOE  0x5C        /* skytraq message id: glonass ephemeris */
#define ID_STQBDSD1 0xE2        /* skytraq message id: beidou d1 subframe */
#define ID_STQBDSD2 0xE3        /* skytraq message id: beidou d2 subframe */

#define ID_RESTART  0x01        /* skytraq message id: system restart */
#define ID_CFGSERI  0x05        /* skytraq message id: configure serial port */
#define ID_CFGFMT   0x09        /* skytraq message id: configure message format */
#define ID_CFGRATE  0x12        /* skytraq message id: configure message rate */
#define ID_CFGBIN   0x1E        /* skytraq message id: configure binary message */
#define ID_GETGLOEPH 0x5B       /* skytraq message id: get glonass ephemeris */
/* checksum ------------------------------------------------------------------*/
static unsigned char checksum(unsigned char *buff,int len)
{
	unsigned char cs=0;
	int i;

	for (i=4; i<len-3; i++) {
		cs^=buff[i];
	}
	return cs;
}
/* generate skytraq binary message ---------------------------------------------------
* generate skytraq binary message from message string
* args   : char  *msg   I      message string
*            "RESTART  [arg...]" system restart
*            "CFG-SERI [arg...]" configure serial port propperty
*            "CFG-FMT  [arg...]" configure output message format
*            "CFG-RATE [arg...]" configure binary measurement output rates
*            "CFG-BIN  [arg...]" configure general binary
*            "GET-GLOEPH [slot]" get glonass ephemeris for freq channel number
*          unsigned char *buff O binary message
* return : length of binary message (0: error)
* note   : see reference [1][2][3][4] for details.
*------------------------------------------------------------------------------------- */
int gen_stq(const char *msg,unsigned char *buff)
{
	const char *hz[]={ "1Hz","2Hz","4Hz","5Hz","10Hz","20Hz","" };
	unsigned char *q=buff;
	char mbuff[1024],*args[32],*p;
	int i,n,narg=0;

	strcpy(mbuff,msg);
	for (p=strtok(mbuff," "); p&&narg<32; p=strtok(NULL," ")) {
		args[narg++]=p;
	}

	*q++=STQSYNC1;
	*q++=STQSYNC2;
	if (!strcmp(args[0],"RESTART")) {
		*q++=0;
		*q++=15;
		*q++=ID_RESTART;
		*q++=narg>2 ? (unsigned char)atoi(args[1]) : 0;
		for (i=1; i<15; i++) *q++=0; /* set all 0 */
	}
	else if (!strcmp(args[0],"CFG-SERI")) {
		*q++=0;
		*q++=4;
		*q++=ID_CFGSERI;
		for (i=1; i<4; i++) *q++=narg>i+1 ? (unsigned char)atoi(args[i]) : 0;
	}
	else if (!strcmp(args[0],"CFG-FMT")) {
		*q++=0;
		*q++=3;
		*q++=ID_CFGFMT;
		for (i=1; i<3; i++) *q++=narg>i+1 ? (unsigned char)atoi(args[i]) : 0;
	}
	else if (!strcmp(args[0],"CFG-RATE")) {
		*q++=0;
		*q++=8;
		*q++=ID_CFGRATE;
		if (narg>2) {
			for (i=0; *hz[i]; i++) if (!strcmp(args[1],hz[i])) break;
			if (*hz[i]) *q++=i; else *q++=(unsigned char)atoi(args[1]);
		}
		else *q++=0;
		for (i=2; i<8; i++) *q++=narg>i+1 ? (unsigned char)atoi(args[i]) : 0;
	}
	else if (!strcmp(args[0],"CFG-BIN")) {
		*q++=0;
		*q++=8;
		*q++=ID_CFGBIN;
		if (narg>2) {
			for (i=0; *hz[i]; i++) if (!strcmp(args[1],hz[i])) break;
			if (*hz[i]) *q++=i; else *q++=(unsigned char)atoi(args[1]);
		}
		else *q++=0;
		for (i=2; i<8; i++) *q++=narg>i+1 ? (unsigned char)atoi(args[i]) : 0;
	}
	else if (!strcmp(args[0],"GET-GLOEPH")) {
		*q++=0;
		*q++=2;
		*q++=ID_GETGLOEPH;
		*q++=narg>=2 ? (unsigned char)atoi(args[1]) : 0;
	}
	else return 0;

	n=(int)(q-buff);
	*q++=checksum(buff,n+3);
	*q++=0x0D;
	*q=0x0A;
	return n+3;
}

/* NVS -------------------------------------------------------------------------------------------- */
#define NVSSYNC     0x10        /* nvs message sync code 1 */
#define NVSENDMSG   0x03        /* nvs message sync code 1 */
#define NVSCFG      0x06        /* nvs message cfg-??? */

#define ID_XF5RAW   0xf5        /* nvs msg id: raw measurement data */
#define ID_X4AIONO  0x4a        /* nvs msg id: gps ionospheric data */
#define ID_X4BTIME  0x4b        /* nvs msg id: GPS/GLONASS/UTC timescale data */
#define ID_XF7EPH   0xf7        /* nvs msg id: subframe buffer */
#define ID_XE5BIT   0xe5        /* nvs msg id: bit information */

#define ID_XD7ADVANCED 0xd7     /* */
#define ID_X02RATEPVT  0x02     /* */
#define ID_XF4RATERAW  0xf4     /* */
#define ID_XD7SMOOTH   0xd7     /* */
#define ID_XD5BIT      0xd5     /* */
/* generate NVS binary message ----------------------------------------------------
* generate NVS binary message from message string
* args   : char  *msg   I      message string
*            "RESTART  [arg...]" system reset
*            "CFG-SERI [arg...]" configure serial port property
*            "CFG-FMT  [arg...]" configure output message format
*            "CFG-RATE [arg...]" configure binary measurement output rates
*          unsigned char *buff O binary message
* return : length of binary message (0: error)
* note   : see reference [1][2] for details.
*---------------------------------------------------------------------------------- */
int gen_nvs(const char *msg,unsigned char *buff)
{
	unsigned char *q=buff;
	char mbuff[1024],*args[32],*p;
	unsigned int byte;
	int iRate,n,narg=0;
	unsigned char ui100Ms;

	strcpy(mbuff,msg);
	for (p=strtok(mbuff," "); p&&narg<32; p=strtok(NULL," ")) {
		args[narg++]=p;
	}

	*q++=NVSSYNC; /* DLE */

	if (!strcmp(args[0],"CFG-PVTRATE")) {
		*q++=ID_XD7ADVANCED;
		*q++=ID_X02RATEPVT;
		if (narg>1) {
			iRate = atoi(args[1]);
			*q++ = (unsigned char)iRate;
		}
	}
	else if (!strcmp(args[0],"CFG-RAWRATE")) {
		*q++=ID_XF4RATERAW;
		if (narg>1) {
			iRate = atoi(args[1]);
			switch (iRate) {
			case 2:  ui100Ms =  5; break;
			case 5:  ui100Ms =  2; break;
			case 10: ui100Ms =  1; break;
			default: ui100Ms = 10; break;
			}
			*q++ = ui100Ms;
		}
	}
	else if (!strcmp(args[0],"CFG-SMOOTH")) {
		*q++=ID_XD7SMOOTH;
		*q++ = 0x03;
		*q++ = 0x01;
		*q++ = 0x00;
	}
	else if (!strcmp(args[0],"CFG-BINR")) {
		for (n=1; (n<narg); n++) {
			if (sscanf(args[n],"%2x",&byte)) *q++=(unsigned char)byte;
		}
	}
	else return 0;

	n=(int)(q-buff);

	*q++=0x10; /* ETX */
	*q=0x03;   /* DLE */
	return n+2;
}

int gen_lexr(const char *msg,unsigned char *buff) { return 0; }

/* generate general hex message ----------------------------------------------*/
int gen_hex(const char *msg,unsigned char *buff)
{
	unsigned char *q=buff;
	char mbuff[1024]="",*args[256],*p;
	unsigned int byte;
	int i,narg=0;

	strncpy(mbuff,msg,1023);
	for (p=strtok(mbuff," "); p&&narg<256; p=strtok(NULL," ")) {
		args[narg++]=p;
	}

	for (i=0; i<narg; i++) {
		if (sscanf(args[i],"%x",&byte)) *q++=(unsigned char)byte;
	}
	return (int)(q-buff);
}