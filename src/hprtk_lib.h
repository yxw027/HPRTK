/* Base dependent library file */
#ifndef LIB_H
#define LIB_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdarg>
#include <cstdlib>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <vector>
#include <cctype>
#include <cstdio>						/* for file stream read */

#ifdef WIN32
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <afx.h>
#include <conio.h>
#include <winsock2.h>
#include <windows.h>
#include <Mmsystem.h>
#pragma comment (lib,"winmm.lib")
#else
#include <pthread.h>					/* for Unix System thread */ 
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#ifndef __USE_MISC
#define __USE_MISC
#endif
#ifndef CRTSCTS
#define CRTSCTS  020000000000
#endif
#include <errno.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#endif

#ifdef WIN_DLL
#define EXPORT __declspec(dllexport)	/* for Windows DLL */
#else
#define EXPORT
#endif

#ifdef WIN32
#define thread_t    HANDLE
#define lock_t      CRITICAL_SECTION
#define initlock(f) InitializeCriticalSection(f)
#define tolock(f)   EnterCriticalSection(f)
#define tounlock(f) LeaveCriticalSection(f)
#define FILEPATHSEP '\\'
#define thrdID      LPDWORD
#else
#define thread_t    pthread_t
#define lock_t      pthread_mutex_t
#define initlock(f) pthread_mutex_init(f,NULL)
#define lock(f)     pthread_mutex_lock(f)
#define unlock(f)   pthread_mutex_unlock(f)
#define FILEPATHSEP '/'
#endif

/* define name space */
using std::cout;
using std::cin;
using std::setw;
using std::setprecision;

using std::istringstream;
using std::ostringstream;
using std::ifstream;
using std::fstream;
using std::ios;

using std::string;
using std::to_string;

using std::vector;

/* algorithm */
using std::any_of;
using std::transform;
using std::binary_search;

#define PI          3.1415926535897932  /* pi */
#define D2R         (PI/180.0)          /* deg to rad */
#define R2D         (180.0/PI)          /* rad to deg */
#define CLIGHT      299792458.0         /* speed of light (m/s) */
#define SC2RAD      3.1415926535898     /* semi-circle to radian (IS-GPS) */
#define AU          149597870691.0      /* 1 AU (m) */
#define AS2R        (D2R/3600.0)        /* arc sec to radian */

#define OMGE        7.2921151467E-5     /* earth angular velocity (IS-GPS) (rad/s) */

#define WGS84       0					/* WGS84 datum system */
#define CGCS2000    1					/* CGCS2000 datum system (China) */
#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define EE_WGS84    (1.0/298.257223563) /* earth eccentricity (WGS84) */
#define RE_CGCS2000 6378137.0			/* earth semimajor axis (CGCS2000) (m) */
#define EE_CGCS2000 (1.0/298.257222101) /* earth eccentricity (CGCS2000) */

#define HION        350000.0            /* ionosphere height (m) */

#define MAXFREQ     7                   /* max NFREQ */

#define FREQ1       1.57542E9           /* L1/E1  frequency (Hz) */
#define FREQ2       1.22760E9           /* L2     frequency (Hz) */
#define FREQ5       1.17645E9           /* L5/E5a frequency (Hz) */
#define FREQ6       1.27875E9           /* E6/LEX frequency (Hz) */
#define FREQ7       1.20714E9           /* E5b    frequency (Hz) */
#define FREQ8       1.191795E9          /* E5a+b  frequency (Hz) */
#define FREQ9       2.492028E9          /* S      frequency (Hz) */
#define FREQ1_GLO   1.60200E9           /* GLONASS G1 base frequency (Hz) */
#define DFRQ1_GLO   0.56250E6           /* GLONASS G1 bias frequency (Hz/n) */
#define FREQ2_GLO   1.24600E9           /* GLONASS G2 base frequency (Hz) */
#define DFRQ2_GLO   0.43750E6           /* GLONASS G2 bias frequency (Hz/n) */
#define FREQC1_GLO  1.600995E9			/* GLONASS G1 CDMA frequency (Hz) */
#define FREQC2_GLO  1.24806E9			/* GLONASS G2 CDMA frequency (Hz) */
#define FREQ3_GLO   1.202025E9          /* GLONASS G3 frequency (Hz) */
#define FREQ1_CMP   1.561098E9          /* BeiDou B1 frequency (Hz) */
#define FREQ2_CMP   1.20714E9           /* BeiDou B2 frequency (Hz) */
#define FREQ3_CMP   1.26852E9           /* BeiDou B3 frequency (Hz) */
#define FREQ1_CMP_3 1.57542E9			/* BeiDou B1C frequency for BDS-3 satellite */
#define FREQ2_CMP_3 1.17645E9			/* BeiDou B2a frequency for BDS-3 satellite */

#define EFACT_GPS   1.0                 /* error factor: GPS */
#define EFACT_GLO   1.5                 /* error factor: GLONASS */
#define EFACT_GAL   1.0                 /* error factor: Galileo */
#define EFACT_QZS   1.0                 /* error factor: QZSS */
#define EFACT_CMP   1.0                 /* error factor: BeiDou */
#define EFACT_CMP_G 2.0                 /* error factor: BeiDou GEO */
#define EFACT_IRN   1.5                 /* error factor: IRNSS */
#define EFACT_SBS   3.0                 /* error factor: SBAS */

#define SYS_NONE    0x00                /* navigation system: none */
#define SYS_GPS     0x01                /* navigation system: GPS */
#define SYS_SBS     0x02                /* navigation system: SBAS */
#define SYS_GLO     0x04                /* navigation system: GLONASS */
#define SYS_GAL     0x08                /* navigation system: Galileo */
#define SYS_QZS     0x10                /* navigation system: QZSS */
#define SYS_CMP     0x20                /* navigation system: BeiDou */
#define SYS_IRN     0x40                /* navigation system: IRNS */
#define SYS_LEO     0x80                /* navigation system: LEO */
#define SYS_ALL     0xFF                /* navigation system: all */

#define TSYS_GPS    0                   /* time system: GPS time */
#define TSYS_UTC    1                   /* time system: UTC */
#define TSYS_GLO    2                   /* time system: GLONASS time */
#define TSYS_GAL    3                   /* time system: Galileo time */
#define TSYS_QZS    4                   /* time system: QZSS time */
#define TSYS_CMP    5                   /* time system: BeiDou time */
#define TSYS_IRN    6                   /* time system: IRNSS time */

#ifndef NFREQ
#define NFREQ       3                   /* number of carrier frequencies */
#endif
#define NFREQGLO    2                   /* number of carrier frequencies of GLONASS */

#ifndef NEXOBS
#define NEXOBS      0                   /* number of extended obs codes */
#endif

#define MINPRNGPS   1                   /* min satellite PRN number of GPS */
#define MAXPRNGPS   32                  /* max satellite PRN number of GPS */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites */
#define NSYSGPS     1

#define MINPRNGLO   301                  /* min satellite slot number of GLONASS */
#define MAXPRNGLO   330                  /* max satellite slot number of GLONASS */
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites */
#define NSYSGLO     1

//#else
//#define MINPRNGLO   0
//#define MAXPRNGLO   0
//#define NSATGLO     0
//#define NSYSGLO     0
//#endif

//#ifdef ENAGAL
#define MINPRNGAL   1                   /* min satellite PRN number of Galileo */
#define MAXPRNGAL   30                  /* max satellite PRN number of Galileo */
#define NSATGAL    (MAXPRNGAL-MINPRNGAL+1) /* number of Galileo satellites */
#define NSYSGAL     1
//#else
//#define MINPRNGAL   0
//#define MAXPRNGAL   0
//#define NSATGAL     0
//#define NSYSGAL     0
//#endif

#ifdef ENAQZS
#define MINPRNQZS   193                 /* min satellite PRN number of QZSS */
#define MAXPRNQZS   199                 /* max satellite PRN number of QZSS */
#define MINPRNQZS_S 183                 /* min satellite PRN number of QZSS SAIF */
#define MAXPRNQZS_S 189                 /* max satellite PRN number of QZSS SAIF */
#define NSATQZS     (MAXPRNQZS-MINPRNQZS+1) /* number of QZSS satellites */
#define NSYSQZS     1
#else
#define MINPRNQZS   0
#define MAXPRNQZS   0
#define MINPRNQZS_S 0
#define MAXPRNQZS_S 0
#define NSATQZS     0
#define NSYSQZS     0
#endif

//#ifdef ENACMP
#define MINPRNCMP   1                   /* min satellite sat number of BeiDou */
#define MAXPRNCMP   40                  /* max satellite sat number of BeiDou */
#define NSATCMP     (MAXPRNCMP-MINPRNCMP+1) /* number of BeiDou satellites */
#define NSYSCMP     1

//#else
//#define MINPRNCMP   0
//#define MAXPRNCMP   0
//#define NSATCMP     0
//#define NSYSCMP     0
//#endif

#ifdef ENAIRN
#define MINPRNIRN   1                   /* min satellite sat number of IRNSS */
#define MAXPRNIRN   7                   /* max satellite sat number of IRNSS */
#define NSATIRN     (MAXPRNIRN-MINPRNIRN+1) /* number of IRNSS satellites */
#define NSYSIRN     1
#else
#define MINPRNIRN   0
#define MAXPRNIRN   0
#define NSATIRN     0
#define NSYSIRN     0
#endif
#ifdef ENALEO
#define MINPRNLEO   1                   /* min satellite sat number of LEO */
#define MAXPRNLEO   10                  /* max satellite sat number of LEO */
#define NSATLEO     (MAXPRNLEO-MINPRNLEO+1) /* number of LEO satellites */
#define NSYSLEO     1
#else
#define MINPRNLEO   0
#define MAXPRNLEO   0
#define NSATLEO     0
#define NSYSLEO     0
#endif
#define NSYS        (NSYSGPS+NSYSGLO+NSYSGAL+NSYSQZS+NSYSCMP+NSYSIRN+NSYSLEO) /* number of systems */

#define MINPRNSBS   101                 /* min satellite PRN number of SBAS */
#define MAXPRNSBS   142                 /* max satellite PRN number of SBAS */
#define NSATSBS     (MAXPRNSBS-MINPRNSBS+1) /* number of SBAS satellites */

#define MAXSAT      (NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATIRN+NSATSBS+NSATLEO)
                                        /* max satellite number (1 to MAXSAT) */
#define MAXSTA      255

#ifndef MAXOBS
#define MAXOBS      64                  /* max number of obs in an epoch */
#endif
#define MAXRCV      64                  /* max receiver number (1 to MAXRCV) */
#define MAXOBSTYPE  64                  /* max number of obs type in RINEX */
#define DTTOL       0.005               /* tolerance of time difference (s) */
#define MAXDTOE     7200.0              /* max time difference to GPS Toe (s) */
#define MAXDTOE_QZS 7200.0              /* max time difference to QZSS Toe (s) */
#define MAXDTOE_GAL 10800.0             /* max time difference to Galileo Toe (s) */
#define MAXDTOE_CMP 21600.0             /* max time difference to BeiDou Toe (s) */
#define MAXDTOE_GLO 1800.0              /* max time difference to GLONASS Toe (s) */
#define MAXDTOE_SBS 360.0               /* max time difference to SBAS Toe (s) */
#define MAXDTOE_S   86400.0             /* max time difference to ephem toe (s) for other */
#define MAXGDOP     300.0               /* max GDOP */

#define INT_SWAP_TRAC 86400.0           /* swap interval of trace file (s) */
#define INT_SWAP_STAT 86400.0           /* swap interval of solution status file (s) */

#define MAXRNAMLEN  20
#define MAXNCON     10                  /* max number of input config file*/
#define MAXEXFILE   1024                /* max number of expanded files */
#define MAXSBSAGEF  30.0                /* max age of SBAS fast correction (s) */
#define MAXSBSAGEL  1800.0              /* max age of SBAS long term corr (s) */
#define MAXSBSURA   8                   /* max URA of SBAS satellite */
#define MAXBAND     10                  /* max SBAS band of IGP */
#define MAXNIGP     201                 /* max number of IGP in SBAS band */
#define MAXNGEO     4                   /* max number of GEO satellites */
#define MAXCOMMENT  10                  /* max number of RINEX comments */
#define MAXSTRPATH  1024                /* max length of stream path */
#define MAXSTRMSG   1024                /* max length of stream message */
#define MAXSTRRTK   8                   /* max number of stream in RTK server */
#define MAXSBSMSG   32                  /* max number of SBAS msg in RTK server */
#define MAXSOLMSG   8191                /* max length of solution message */
#define MAXRAWLEN   4096                /* max length of receiver raw message */
#define MAXERRMSG   4096                /* max length of error/warning message */
#define MAXANT      64                  /* max length of station name/antenna type */
#define MAXSOLBUF   3					/* max number of solution buffer */
#define MAXOBSBUF   128                 /* max number of observation data buffer */
#define MAXNRPOS    16                  /* max number of reference positions */
#define MAXLEAPS    64                  /* max number of leap seconds table */
#define MAXGISLAYER 32                  /* max number of GIS data layers */
#define MAXRCVCMD   4096                /* max length of receiver commands */

#define RNX2VER     2.10                /* RINEX ver.2 default output version */
#define RNX3VER     3.00                /* RINEX ver.3 default output version */

#define OBSTYPE_PR  0x01                /* observation type: pseudorange */
#define OBSTYPE_CP  0x02                /* observation type: carrier-phase */
#define OBSTYPE_DOP 0x04                /* observation type: doppler-freq */
#define OBSTYPE_SNR 0x08                /* observation type: SNR */
#define OBSTYPE_ALL 0xFF                /* observation type: all */

#define FREQTYPE_L1 0x01                /* frequency type: L1/E1 */
#define FREQTYPE_L2 0x02                /* frequency type: L2/B1 */
#define FREQTYPE_L5 0x04                /* frequency type: L5/E5a/L3 */
#define FREQTYPE_L6 0x08                /* frequency type: E6/LEX/B3 */
#define FREQTYPE_L7 0x10                /* frequency type: E5b/B2 */
#define FREQTYPE_L8 0x20                /* frequency type: E5(a+b) */
#define FREQTYPE_L9 0x40                /* frequency type: S */
#define FREQTYPE_ALL 0xFF               /* frequency type: all */

#define CODE_NONE   0                   /* obs code: none or unknown */
#define CODE_L1C    1                   /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define CODE_L1P    2                   /* obs code: L1P,G1P    (GPS,GLO) */
#define CODE_L1W    3                   /* obs code: L1 Z-track (GPS) */
#define CODE_L1Y    4                   /* obs code: L1Y        (GPS) */
#define CODE_L1M    5                   /* obs code: L1M        (GPS) */
#define CODE_L1N    6                   /* obs code: L1codeless (GPS) */
#define CODE_L1S    7                   /* obs code: L1C(D)     (GPS,QZS) */
#define CODE_L1L    8                   /* obs code: L1C(P)     (GPS,QZS) */
#define CODE_L1E    9                   /* (not used) */
#define CODE_L1A    10                  /* obs code: E1A        (GAL) */
#define CODE_L1B    11                  /* obs code: E1B        (GAL) */
#define CODE_L1X    12                  /* obs code: E1B+C,L1C(D+P) (GAL,QZS) */
#define CODE_L1Z    13                  /* obs code: E1A+B+C,L1SAIF (GAL,QZS) */
#define CODE_L2C    14                  /* obs code: L2C/A,G1C/A (GPS,GLO) */
#define CODE_L2D    15                  /* obs code: L2 L1C/A-(P2-P1) (GPS) */
#define CODE_L2S    16                  /* obs code: L2C(M)     (GPS,QZS) */
#define CODE_L2L    17                  /* obs code: L2C(L)     (GPS,QZS) */
#define CODE_L2X    18                  /* obs code: L2C(M+L),B1I+Q (GPS,QZS,CMP) */
#define CODE_L2P    19                  /* obs code: L2P,G2P    (GPS,GLO) */
#define CODE_L2W    20                  /* obs code: L2 Z-track (GPS) */
#define CODE_L2Y    21                  /* obs code: L2Y        (GPS) */
#define CODE_L2M    22                  /* obs code: L2M        (GPS) */
#define CODE_L2N    23                  /* obs code: L2codeless (GPS) */
#define CODE_L5I    24                  /* obs code: L5/E5aI    (GPS,GAL,QZS,SBS) */
#define CODE_L5Q    25                  /* obs code: L5/E5aQ    (GPS,GAL,QZS,SBS) */
#define CODE_L5X    26                  /* obs code: L5/E5aI+Q/L5B+C (GPS,GAL,QZS,IRN,SBS) */
#define CODE_L7I    27                  /* obs code: E5bI,B2I   (GAL,CMP) */
#define CODE_L7Q    28                  /* obs code: E5bQ,B2Q   (GAL,CMP) */
#define CODE_L7X    29                  /* obs code: E5bI+Q,B2I+Q (GAL,CMP) */
#define CODE_L6A    30                  /* obs code: E6A        (GAL) */
#define CODE_L6B    31                  /* obs code: E6B        (GAL) */
#define CODE_L6C    32                  /* obs code: E6C        (GAL) */
#define CODE_L6X    33                  /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,CMP) */
#define CODE_L6Z    34                  /* obs code: E6A+B+C    (GAL) */
#define CODE_L6S    35                  /* obs code: LEXS       (QZS) */
#define CODE_L6L    36                  /* obs code: LEXL       (QZS) */
#define CODE_L8I    37                  /* obs code: E5(a+b)I   (GAL) */
#define CODE_L8Q    38                  /* obs code: E5(a+b)Q   (GAL) */
#define CODE_L8X    39                  /* obs code: E5(a+b)I+Q (GAL) */
#define CODE_L2I    40                  /* obs code: B1I        (BDS) */
#define CODE_L2Q    41                  /* obs code: B1Q        (BDS) */
#define CODE_L6I    42                  /* obs code: B3I        (BDS) */
#define CODE_L6Q    43                  /* obs code: B3Q        (BDS) */
#define CODE_L3I    44                  /* obs code: G3I        (GLO) */
#define CODE_L3Q    45                  /* obs code: G3Q        (GLO) */
#define CODE_L3X    46                  /* obs code: G3I+Q      (GLO) */
#define CODE_L1I    47                  /* obs code: B1I        (BDS) */
#define CODE_L1Q    48                  /* obs code: B1Q        (BDS) */
#define CODE_L5A    49                  /* obs code: L5A SPS    (IRN) */
#define CODE_L5B    50                  /* obs code: L5B RS(D)  (IRN) */
#define CODE_L5C    51                  /* obs code: L5C RS(P)  (IRN) */
#define CODE_L9A    52                  /* obs code: SA SPS     (IRN) */
#define CODE_L9B    53                  /* obs code: SB RS(D)   (IRN) */
#define CODE_L9C    54                  /* obs code: SC RS(P)   (IRN) */
#define CODE_L9X    55                  /* obs code: SB+C       (IRN) */
#define MAXCODE     55                  /* max number of obs code */

#define PMODE_SINGLE     0              /* positioning mode: single */
#define PMODE_PPP_KINEMA 1              /* positioning mode: PPP-kinemaric */
#define PMODE_PPP_STATIC 2              /* positioning mode: PPP-static */
#define PMODE_PPP_FIXED  3              /* positioning mode: PPP-fixed */
#define PMODE_DGPS       4              /* positioning mode: DGPS/DGNSS */
#define PMODE_KINEMA     5              /* positioning mode: kinematic */
#define PMODE_STATIC     6              /* positioning mode: static */
#define PMODE_MOVEB      7              /* positioning mode: moving-base */
#define PMODE_FIXED      8              /* positioning mode: fixed */


#define ADJUST_LSA         0			/* adjustment mode: least square adjustment */
#define ADJUST_KALMAN      1			/* adjustment mode: kalman filter */
#define ADJUST_HELMERT     2			/* adjustment mode: kalman filter with helmert */

#define SOLF_LLH    0                   /* solution format: lat/lon/height */
#define SOLF_XYZ    1                   /* solution format: x/y/z-ecef */
#define SOLF_ENU    2                   /* solution format: e/n/u-baseline */
#define SOLF_NMEA   3                   /* solution format: NMEA-183 */
#define SOLF_STAT   4                   /* solution format: solution status */
#define SOLF_GSIF   5                   /* solution format: GSI F1/F2 */

#define SOLQ_NONE   0                   /* solution status: no solution */
#define SOLQ_FIX    1                   /* solution status: fix */
#define SOLQ_FLOAT  2                   /* solution status: float */
#define SOLQ_SBAS   3                   /* solution status: SBAS */
#define SOLQ_DGPS   4                   /* solution status: DGPS/DGNSS */
#define SOLQ_SINGLE 5                   /* solution status: single */
#define SOLQ_PPP    6                   /* solution status: PPP */
#define SOLQ_DR     7                   /* solution status: dead reconing */
#define MAXSOLQ     7                   /* max number of solution status */

#define TIMES_GPST  0                   /* time system: gps time */
#define TIMES_UTC   1                   /* time system: utc */
#define TIMES_BDT   2                   /* time system: bdt */

#define IONOOPT_OFF    0                /* ionosphere option: correction off */
#define IONOOPT_BRDC   1                /* ionosphere option: broadcast model */
#define IONOOPT_SBAS   2                /* ionosphere option: SBAS model */
#define IONOOPT_IFLC   3                /* ionosphere option: L1/L2 or L1/L5 iono-free LC */
#define IONOOPT_CONST  4                /* ionosphere option: constrained ION model (estimation) */
#define IONOOPT_TEC    5                /* ionosphere option: IONEX TEC model */
#define IONOOPT_QZS    6                /* ionosphere option: QZSS broadcast model */
#define IONOOPT_LEX    7                /* ionosphere option: QZSS LEX ionospehre */
#define IONOOPT_STEC   8                /* ionosphere option: SLANT TEC model */

#define TROPOPT_OFF 0                   /* troposphere option: correction off */
#define TROPOPT_SAAS 1                  /* troposphere option: Saastamoinen model */
#define TROPOPT_SBAS 2                  /* troposphere option: SBAS model */
#define TROPOPT_EST 3                   /* troposphere option: ZTD estimation */
#define TROPOPT_ESTG 4                  /* troposphere option: ZTD+grad estimation */
#define TROPOPT_ZTD 5                   /* troposphere option: ZTD correction */

#define EPHOPT_BRDC 0                   /* ephemeris option: broadcast ephemeris */
#define EPHOPT_PREC 1                   /* ephemeris option: precise ephemeris */
#define EPHOPT_SBAS 2                   /* ephemeris option: broadcast + SBAS */
#define EPHOPT_SSRAPC 3                 /* ephemeris option: broadcast + SSR_APC */
#define EPHOPT_SSRCOM 4                 /* ephemeris option: broadcast + SSR_COM */
#define EPHOPT_LEX  5                   /* ephemeris option: QZSS LEX ephemeris */

#define ARMODE_OFF  0                   /* AR mode: off */
#define ARMODE_CONT 1                   /* AR mode: continuous */
#define ARMODE_INST 2                   /* AR mode: instantaneous */
#define ARMODE_FIXHOLD 3                /* AR mode: fix and hold */
#define ARMODE_LCWN 4                   /* AR mode: LC with wide/narrow-lane */

#define SBSOPT_LCORR 1                  /* SBAS option: long term correction */
#define SBSOPT_FCORR 2                  /* SBAS option: fast correction */
#define SBSOPT_ICORR 4                  /* SBAS option: ionosphere correction */
#define SBSOPT_RANGE 8                  /* SBAS option: ranging */

#define POSOPT_POS   0                  /* pos option: LLH/XYZ */
#define POSOPT_SINGLE 1                 /* pos option: average of single pos */
#define POSOPT_FILE  2                  /* pos option: read from pos file */
#define POSOPT_RINEX 3                  /* pos option: rinex header pos */
#define POSOPT_RTCM  4                  /* pos option: rtcm station pos */
#define POSOPT_RAW   5                  /* pos option: raw station pos */

#define STR_NONE     0                  /* stream type: none */
#define STR_SERIAL   1                  /* stream type: serial */
#define STR_FILE     2                  /* stream type: file */
#define STR_TCPSVR   3                  /* stream type: TCP server */
#define STR_TCPCLI   4                  /* stream type: TCP client */
#define STR_NTRIPSVR 6                  /* stream type: NTRIP server */
#define STR_NTRIPCLI 7                  /* stream type: NTRIP client */
#define STR_FTP      8                  /* stream type: ftp */
#define STR_HTTP     9                  /* stream type: http */
#define STR_NTRIPC_S 10                 /* stream type: NTRIP caster server */
#define STR_NTRIPC_C 11                 /* stream type: NTRIP caster client */
#define STR_UDPSVR   12                 /* stream type: UDP server */
#define STR_UDPCLI   13                 /* stream type: UDP server */
#define STR_MEMBUF   14                 /* stream type: memory buffer */

#define STRFMT_RTCM2 0                  /* stream format: RTCM 2 */
#define STRFMT_RTCM3 1                  /* stream format: RTCM 3 */
#define STRFMT_OEM4  2                  /* stream format: NovAtel OEMV/4 */
#define STRFMT_OEM3  3                  /* stream format: NovAtel OEM3 */
#define STRFMT_UBX   4                  /* stream format: u-blox LEA-*T */
#define STRFMT_SS2   5                  /* stream format: NovAtel Superstar II */
#define STRFMT_CRES  6                  /* stream format: Hemisphere */
#define STRFMT_STQ   7                  /* stream format: SkyTraq S1315F */
#define STRFMT_GW10  8                  /* stream format: Furuno GW10 */
#define STRFMT_JAVAD 9                  /* stream format: JAVAD GRIL/GREIS */
#define STRFMT_NVS   10                 /* stream format: NVS NVC08C */
#define STRFMT_BINEX 11                 /* stream format: BINEX */
#define STRFMT_RT17  12                 /* stream format: Trimble RT17 */
#define STRFMT_SEPT  13                 /* stream format: Septentrio */
#define STRFMT_CMR   14                 /* stream format: CMR/CMR+ */
#define STRFMT_LEXR  15                 /* stream format: Furuno LPY-10000 */
#define STRFMT_RINEX 16                 /* stream format: RINEX */
#define STRFMT_SP3   17                 /* stream format: SP3 */
#define STRFMT_RNXCLK 18                /* stream format: RINEX CLK */
#define STRFMT_SBAS  19                 /* stream format: SBAS messages */
#define STRFMT_NMEA  20                 /* stream format: NMEA 0183 */
#ifndef EXTLEX
#define MAXRCVFMT    14                 /* max number of receiver format */
#else
#define MAXRCVFMT    15
#endif

#define STR_MODE_R  0x1                 /* stream mode: read */
#define STR_MODE_W  0x2                 /* stream mode: write */
#define STR_MODE_RW 0x3                 /* stream mode: read/write */

#define GEOID_EMBEDDED    0             /* geoid model: embedded geoid */
#define GEOID_EGM96_M150  1             /* geoid model: EGM96 15x15" */
#define GEOID_EGM2008_M25 2             /* geoid model: EGM2008 2.5x2.5" */
#define GEOID_EGM2008_M10 3             /* geoid model: EGM2008 1.0x1.0" */
#define GEOID_GSI2000_M15 4             /* geoid model: GSI geoid 2000 1.0x1.5" */
#define GEOID_RAF09       5             /* geoid model: IGN RAF09 for France 1.5"x2" */

#define COMMENTH    "%"                 /* comment line indicator for solution */
#define MSG_DISCONN "$_DISCONNECT\r\n"  /* disconnect message */
#define PRINT_LEN    4096				/* size of print buff */

#define DLOPT_FORCE   0x01              /* download option: force download existing */
#define DLOPT_KEEPCMP 0x02              /* download option: keep compressed file */
#define DLOPT_HOLDERR 0x04              /* download option: hold on error file */
#define DLOPT_HOLDLST 0x08              /* download option: hold on listing file */

#define P2_5        0.03125             /* 2^-5 */
#define P2_6        0.015625            /* 2^-6 */
#define P2_11       4.882812500000000E-04 /* 2^-11 */
#define P2_15       3.051757812500000E-05 /* 2^-15 */
#define P2_17       7.629394531250000E-06 /* 2^-17 */
#define P2_19       1.907348632812500E-06 /* 2^-19 */
#define P2_20       9.536743164062500E-07 /* 2^-20 */
#define P2_21       4.768371582031250E-07 /* 2^-21 */
#define P2_23       1.192092895507810E-07 /* 2^-23 */
#define P2_24       5.960464477539063E-08 /* 2^-24 */
#define P2_27       7.450580596923828E-09 /* 2^-27 */
#define P2_29       1.862645149230957E-09 /* 2^-29 */
#define P2_30       9.313225746154785E-10 /* 2^-30 */
#define P2_31       4.656612873077393E-10 /* 2^-31 */
#define P2_32       2.328306436538696E-10 /* 2^-32 */
#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_35       2.910383045673370E-11 /* 2^-35 */
#define P2_38       3.637978807091710E-12 /* 2^-38 */
#define P2_39       1.818989403545856E-12 /* 2^-39 */
#define P2_40       9.094947017729280E-13 /* 2^-40 */
#define P2_43       1.136868377216160E-13 /* 2^-43 */
#define P2_48       3.552713678800501E-15 /* 2^-48 */
#define P2_50       8.881784197001252E-16 /* 2^-50 */
#define P2_55       2.775557561562891E-17 /* 2^-55 */

/* Constant --------------------------------------------------------------------------------------- */
/* BeiDou satellites GEO/MEO/IGSO prn vector */
const int BDS_GEO[] ={1,3,4,5,2,17};
const int BDS_MEO[] ={11,12,14,20,21,22,23,24,25,26,27,28,29,30};
const int BDS_IGSO[]={6,7,8,9,10,13,15,16};

typedef void fatalfunc_t(const char *); /* fatal callback function type */

/* classes pre-statement -------------------------------------------------------------------------- */
/* time strcuture ----------------------------------------------------------------- */
class gtime_t;

/* stream type -------------------------------------------------------------------- */
class stream_t;							/* stream type */
class file_t;							/* stream :: file control type */
class tcpsvr_t;							/* stream :: tcp server type */
class tcpcli_t;							/* stream :: tcp cilent type */
class serial_t;							/* stream :: tcpsvr_t :: serial control type */
class ntrip_t;							/* stream :: tcpcli_t :: ntrip control type */
class ntripc_t;							/* stream :: tcpsvr_t :: ntrip caster control type */
class udp_t;							/* stream :: udp type */
class ftp_t;							/* stream :: ftp download control type */
class membuf_t;							/* stream :: memory buffer type */

/* processing classes ------------------------------------------------------------- */
class sol_t;							/* solution type */
class ssat_t;							/* satellite status type */
class rtk_t;							/* RTK control/result type */
class rtksvr_t;							/* RTK server type */
class postsvr_t;						/* post-processing server type */

/* decode data classes ------------------------------------------------------------ */
class decode_data;						/* decode data for kinds of formats */
class rtcm_t;							/* RTCM control struct type */
class rtcm_2;							/* RTCM 2 control struct type */
class rtcm_3;							/* RTCM 3 control struct type */
class raw_t;							/* receiver raw data control type */

/* read file classes -------------------------------------------------------------- */
class inrnx_t;							/* read rinxe file (.O .N .G .H .J .L .C) */
class inrnxO_t;							/* read rinex file (.O) */
class inrnxN_t;							/* read rinex file (.N,.P) */
class inerp_t;							/* read earth rotation parameters file (.ERP) */
class inblq_t;							/* read ocean-loading tide file (.BLQ) */
class inatx_t;							/* read antenna information file (.ATX) */
class ineph_t;							/* read precise ephemeris file */
class inionex_t;						/* read ionex tec grid file */

/* GPS data classes --------------------------------------------------------------- */
class obsd_t;							/* one epoch observation data */
class sta_t;							/* station informtations */
class obs_t;							/* station's information and observation chains */
class eph_t;							/* GPS/QZS/GAL broadcast ephemeris type */
class geph_t;							/* GLONASS broadcast ephemeris type */
class seph_t;							/* SBAS ephemeris type */
class peph_t;							/* precise ephemeris type */
class pclk_t;							/* precise clock type */
class alm_t;							/* almanac type */
class tec_t;							/* TEC grid type */
class fcbd_t;							/* satellite fcb data type */
class erpd_t;							/* earth rotation parameter data type */
class erp_t;							/* earth rotation parameter type */
class pcv_t;							/* antenna parameter type */
class sbsfcorr_t;						/* SBAS fast correction type */
class sbslcorr_t;						/* SBAS long term satellite error correction type */
class sbssatp_t;						/* SBAS satellite correction type */
class sbssat_t;							/* SBAS satellite corrections type */
class sbsigp_t;							/* SBAS ionospheric correction type */
class sbsion_t;							/* SBAS ionospheric corrections type */
class dgps_t;							/* DGPS/GNSS correction type */
class ssr_t;							/* SSR correction type */
class lexeph_t;							/* QZSS LEX ephemeris type */
class lexion_t;							/* QZSS LEX ionosphere correction type */
class stec_t;							/* stec data type */
class trop_t;							/* trop data type */
class pppcorr_t;						/* ppp corrections type */
class nav_t;							/* class of navigation data */
class sbsmsg_t;							/* SBAS message type */
class sbs_t;							/* SBAS messages type */
class lexmsg_t;							/* QZSS LEX message type */
class lex_t;							/* QZSS LEX messages type */

/* configure option classes ------------------------------------------------------- */
class pstopt_t;							/* post-processing option type */
class rtkopt_t;							/* rtk-processing option type */
class prcopt_t;							/* processing options type */
class solopt_t;							/* solution options type */
class filopt_t;							/* file options type */
class rnxopt_t;							/* RINEX options type */
class option_t;							/* all options */

/* adjustment function classes ---------------------------------------------------- */
class adjfunc_t;						/* parent adjustment functions */
class lsadj_t;							/* least square adjustment function */
class kalmanadj_t;						/* kalman filter adjustment function */
class helmert_t;						/* helmert components covariance estimate 
										   with kalman filter */

/* antenna function classes ------------------------------------------------------- */
class satantenna_t;						/* satellite antenna phase center offest */
class recantenna_t;						/* receiver antenna phase center offest */

/* ionosphere correction function classes ----------------------------------------- */
class ioncorr_t;						/* parent ionosphere functions (vertical delay of GPS L1) */
class LCion_t;							/* ionosphere free combination */
class broadion_t;						/* broadcast ionosphere correction */
class qzssion_t;						/* qzss broadcast ionosphere correction */
class sbasion_t;						/* sbas ionosphere correction */
class ionexion_t;						/* ionex ionosphere correction */
class lexioncor_t;						/* lex ionosphere correction */
class constion_t;						/* constrained ionosphere model correction (L1) */

/* troposphere correction function classes ---------------------------------------- */
class tromod_t;							/* troposphere model class */
class trocorr_t;						/* parent troposphere functions */
class saastro_t;						/* saastamoinen model troposphere correction */
class sbastro_t;						/* sbas model troposphere correction */
class estitro_t;						/* estimated model troposphere correction */

/* earth tide and ocean load function classes ------------------------------------- */
class tidecorr_t;						/* earth, ocean loading and pole tide function */

/* satellite function classes ----------------------------------------------------- */
class satellite_t;						/* parent class of satellite functions */
class broadcast_t;						/* broadcast ephemeris */
class broadsbas_t;						/* broadcast ephemeris with sbas correction */
class broadssrapc_t;					/* broadcast ephemeris with ssr_apc correction */
class broadssrcom_t;					/* broadcast ephemeris with ssr_com correction */
class preciseph_t;						/* precise ephemeris */
class qzsslex_t;						/* qzss lex ephemeris */

/* parameter function classes ----------------------------------------------------- */
class parafunc_t;						/* parameter function */

/* integar ambiguity strategy class ----------------------------------------------- */
class intamb_t;							/* integar ambiguity strategy parent class */
class lambda_t;							/* lambda/mlambda integer least-square estimation */

#endif