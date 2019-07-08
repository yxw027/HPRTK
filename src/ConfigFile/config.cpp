/* Class of Reading Positioning Configure file */
#include "ConfigFile/config.h"
#include "BaseFunction/basefunction.h"

/* static option value */
/* system options buffer -----------------------------------------------------*/
static prcopt_t prcopt_=prcopt_t();
static solopt_t solopt1_=solopt_t(),solopt2_=solopt_t();
static filopt_t filopt_=filopt_t();
static int antpostype_[2];
static double elmask_,elmaskhold_;
static double antpos_[2][3];
static string exsats_;
static string snrmask_[NFREQ];

/* constant */
/* system options table --------------------------------------------------------------------------- */
#define SWTOPT  "0:off,1:on"
#define MODOPT  "0:single,1:ppp-kin,2:ppp-sta,3:ppp-fix,4:dgps,5:kinematic,6:static,7:movingbase,8:fixed"
#define ADJOPT  "0:lsa,1:kalman,2:helmert"
#define FRQOPT  "1:l1,2:l1+l2,3:l1+l2+l5"
#define TYPOPT  "0:forward,1:backward,2:combined"
#define SPPION  "0:off,1:brdc,2:sbas,3:dual-freq,5:ionex-tec,6:qzs-brdc,7:qzs-lex"
#define IONOPT  "0:off,1:brdc,2:sbas,3:dual-freq,4:constrain,5:ionex-tec,6:qzs-brdc,7:qzs-lex,8:stec"
#define SPPTRO  "0:off,1:saas,2:sbas"
#define TRPOPT  "0:off,1:saas,2:sbas,3:est-ztd,4:est-ztdgrad,5:ztd"
#define EPHOPT  "0:brdc,1:precise,2:brdc+sbas,3:brdc+ssrapc,4:brdc+ssrcom"
#define NAVOPT  "1:gps+2:sbas+4:glo+8:gal+16:qzs+32:bds"
#define SLPOPT  "0:obs+1:poly+2:geo-free+4:Melbourne-Wubbena"
#define GAROPT  "0:off,1:on,2:auto"
#define SOLOPT  "0:llh,1:xyz,2:enu,3:nmea"
#define TSYOPT  "0:gpst,1:utc,2:jst"
#define TFTOPT  "0:tow,1:hms"
#define DFTOPT  "0:deg,1:dms"
#define DTMOPT  "0:WGS84,1:CGCS2000"
#define HGTOPT  "0:ellipsoidal,1:geodetic"
#define GEOOPT  "0:internal,1:egm96,2:egm08_2.5,3:egm08_1,4:gsi2000"
#define STAOPT  "0:all,1:single"
#define STSOPT  "0:off,1:state,2:residual"
#define ARMOPT  "0:off,1:continuous,2:instantaneous,3:fix-and-hold,4:LC_WN"
#define POSOPT  "0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm,6:raw"
#define TIDEOPT "0:off+1:solid+2:otl+4:pole"
#define PHWOPT  "0:off,1:on,2:precise"
EXPORT opt_t sysopts[]={
	{ "pos1-posmode",    3,  (void *)&prcopt_.mode,           MODOPT   },
	{ "pos1-frequency",  3,  (void *)&prcopt_.nf,             FRQOPT   },
	{ "pos1-soltype",    3,  (void *)&prcopt_.soltype,        TYPOPT   },
	{ "pos1-elmask",     1,  (void *)&elmask_,                "deg"    },
	{ "pos1-snrmask_r",  3,  (void *)&prcopt_.snrmask.ena[0], SWTOPT   },
	{ "pos1-snrmask_b",  3,  (void *)&prcopt_.snrmask.ena[1], SWTOPT   },
	{ "pos1-snrmask_L1", 2,  (void *)&snrmask_[0],            ""       },
	{ "pos1-snrmask_L2", 2,  (void *)&snrmask_[1],            ""       },
	{ "pos1-snrmask_L5", 2,  (void *)&snrmask_[2],            ""       },
	{ "pos1-dynamics",   3,  (void *)&prcopt_.dynamics,       SWTOPT   },
	{ "pos1-tidecorr",   0,  (void *)&prcopt_.tidecorr,       TIDEOPT  },
	{ "pos1-sppiono",    3,  (void *)&prcopt_.sppiono,        SPPION   },
	{ "pos1-ionoopt",    3,  (void *)&prcopt_.ionoopt,        IONOPT   },
	{ "pos1-iondeg_n",   0,  (void *)&prcopt_.iondeg_n,       ""       },
	{ "pos1-iondeg_m",   0,  (void *)&prcopt_.iondeg_m,       ""       },
	{ "pos1-ion_nm",     0,  (void *)&prcopt_.ion_nm,         ""       },
	{ "pos1-spptrop",    3,  (void *)&prcopt_.spptrop,        SPPTRO   },
	{ "pos1-tropopt",    3,  (void *)&prcopt_.tropopt,        TRPOPT   },
	{ "pos1-sateph",     3,  (void *)&prcopt_.sateph,         EPHOPT   },
	{ "pos1-posopt1",    3,  (void *)&prcopt_.posopt[0],      SWTOPT   },
	{ "pos1-posopt2",    3,  (void *)&prcopt_.posopt[1],      SWTOPT   },
	{ "pos1-posopt3",    3,  (void *)&prcopt_.posopt[2],      PHWOPT   },
	{ "pos1-exclsats",   2,  (void *)&exsats_,                "prn ..."},
	{ "pos1-navsys",     0,  (void *)&prcopt_.navsys,         NAVOPT   },

	{ "pos2-armode",     3,  (void *)&prcopt_.modear,         ARMOPT   },
	{ "pos2-order",      0,  (void *)&prcopt_.order,          ""       },
	{ "pos2-slipmode",   0,  (void *)&prcopt_.slipmode,       SLPOPT   },
	{ "pos2-slipstd",    1,  (void *)&prcopt_.slip_std,       "m"      },
	{ "pos2-ion_gf",     1,  (void *)&prcopt_.ion_gf,         "m/s"    },
	{ "pos2-sampling",   1,  (void *)&prcopt_.sampling,       "s"      },
	{ "pos2-restime",    0,  (void *)&prcopt_.restime,        "s"      },
	{ "pos2-gloarmode",  3,  (void *)&prcopt_.glomodear,      GAROPT   },
	{ "pos2-bdsarmode",  3,  (void *)&prcopt_.bdsmodear,      SWTOPT   },
	{ "pos2-arthres",    1,  (void *)&prcopt_.thresar[0],     ""       },
	{ "pos2-arthres1",   1,  (void *)&prcopt_.thresar[1],     ""       },
	{ "pos2-arthres2",   1,  (void *)&prcopt_.thresar[2],     ""       },
	{ "pos2-arthres3",   1,  (void *)&prcopt_.thresar[3],     ""       },
	{ "pos2-arthres4",   1,  (void *)&prcopt_.thresar[4],     ""       },
	{ "pos2-iniar",      0,  (void *)&prcopt_.iniamb,         "n"      },
	{ "pos2-maxariter",  0,  (void *)&prcopt_.maxariter,      ""       },
	{ "pos2-elmaskhold", 1,  (void *)&elmaskhold_,            "deg"    },
	{ "pos2-maxage",     1,  (void *)&prcopt_.maxtdiff,       "s"      },
	{ "pos2-syncsol",    3,  (void *)&prcopt_.syncsol,        SWTOPT   },
	{ "pos2-maxres",     1,  (void *)&prcopt_.maxres,         ""       },
	{ "pos2-adjust",     3,  (void *)&prcopt_.adjustfunc,     ADJOPT   },
	{ "pos2-niter",      0,  (void *)&prcopt_.niter,          ""       },
	{ "pos2-baselen",    1,  (void *)&prcopt_.baseline[0],    "m"      },
	{ "pos2-basesig",    1,  (void *)&prcopt_.baseline[1],    "m"      },

	{ "out1-solformat",  3,  (void *)&solopt1_.posf,          SOLOPT   },
	{ "out1-outhead",    3,  (void *)&solopt1_.outhead,       SWTOPT   },
	{ "out1-outopt",     3,  (void *)&solopt1_.outopt,        SWTOPT   },
	{ "out1-timesys",    3,  (void *)&solopt1_.times,         TSYOPT   },
	{ "out1-timeform",   3,  (void *)&solopt1_.timef,         TFTOPT   },
	{ "out1-timendec",   0,  (void *)&solopt1_.timeu,         ""       },
	{ "out1-degform",    3,  (void *)&solopt1_.degf,          DFTOPT   },
	{ "out1-fieldsep",   2,  (void *)&solopt1_.sep,           ""       },
	{ "out1-origin",     0,  (void *)&solopt1_.origin,        ""       },
	{ "out1-datum",      3,  (void *)&solopt1_.datum,         DTMOPT   },
	{ "out1-height",     3,  (void *)&solopt1_.height,        HGTOPT   },
	{ "out1-geoid",      3,  (void *)&solopt1_.geoid,         GEOOPT   },
	{ "out1-solstatic",  3,  (void *)&solopt1_.solstatic,     STAOPT   },
	{ "out1-nmeaintv1",  1,  (void *)&solopt1_.nmeaintv[0],   "s"      },
	{ "out1-nmeaintv2",  1,  (void *)&solopt1_.nmeaintv[1],   "s"      },
	{ "out1-outstat",    3,  (void *)&solopt1_.sstat,         STSOPT   },

	{ "out2-solformat",  3,  (void *)&solopt2_.posf,          SOLOPT   },
	{ "out2-outhead",    3,  (void *)&solopt2_.outhead,       SWTOPT   },
	{ "out2-outopt",     3,  (void *)&solopt2_.outopt,        SWTOPT   },
	{ "out2-timesys",    3,  (void *)&solopt2_.times,         TSYOPT   },
	{ "out2-timeform",   3,  (void *)&solopt2_.timef,         TFTOPT   },
	{ "out2-timendec",   0,  (void *)&solopt2_.timeu,         ""       },
	{ "out2-degform",    3,  (void *)&solopt2_.degf,          DFTOPT   },
	{ "out2-fieldsep",   2,  (void *)&solopt2_.sep,           ""       },
	{ "out2-origin",     0,  (void *)&solopt2_.origin,        ""       },
	{ "out2-datum",      3,  (void *)&solopt2_.datum,         DTMOPT   },
	{ "out2-height",     3,  (void *)&solopt2_.height,        HGTOPT   },
	{ "out2-geoid",      3,  (void *)&solopt2_.geoid,         GEOOPT   },
	{ "out2-solstatic",  3,  (void *)&solopt2_.solstatic,     STAOPT   },
	{ "out2-nmeaintv1",  1,  (void *)&solopt2_.nmeaintv[0],   "s"      },
	{ "out2-nmeaintv2",  1,  (void *)&solopt2_.nmeaintv[1],   "s"      },
	{ "out2-outstat",    3,  (void *)&solopt2_.sstat,         STSOPT   },

	{ "stats-eratio1",   1,  (void *)&prcopt_.eratio[0],      ""       },
	{ "stats-eratio2",   1,  (void *)&prcopt_.eratio[1],      ""       },
	{ "stats-eratio3",   1,  (void *)&prcopt_.eratio[2],      ""       },
	{ "stats-errphase",  1,  (void *)&prcopt_.err[0],         "m"      },
	{ "stats-errphaseel",1,  (void *)&prcopt_.err[1],         "m"      },
	{ "stats-errphasebl",1,  (void *)&prcopt_.err[2],         "m/10km" },
	{ "stats-errdoppler",1,  (void *)&prcopt_.err[3],         "Hz"     },
	{ "stats-stdambs",   1,  (void *)&prcopt_.std[0],         "m"      },
	{ "stats-stdiono",   1,  (void *)&prcopt_.std[1],         "m"      },
	{ "stats-stdtrop",   1,  (void *)&prcopt_.std[2],         "m"      },
	{ "stats-rationo",   1,  (void *)&prcopt_.stdrate[0],     "m"      },
	{ "stats-rattrop",   1,  (void *)&prcopt_.stdrate[1],     "m"      },
	{ "stats-ratvelh",   1,  (void *)&prcopt_.stdrate[2],     "m"      },
	{ "stats-ratvelv",   1,  (void *)&prcopt_.stdrate[3],     "m"      },
	{ "stats-clkstab",   1,  (void *)&prcopt_.sclkstab,       "s/s"    },

	{ "ant1-postype",    3,  (void *)&antpostype_[0],         POSOPT   },
	{ "ant1-name",       2,  (void *)&prcopt_.name[0],        ""       },
	{ "ant1-pos1",       1,  (void *)&antpos_[0][0],          "deg|m"  },
	{ "ant1-pos2",       1,  (void *)&antpos_[0][1],          "deg|m"  },
	{ "ant1-pos3",       1,  (void *)&antpos_[0][2],          "m|m"    },
	{ "ant1-anttype",    2,  (void *)&prcopt_.anttype[0],     ""       },
	{ "ant1-antdele",    1,  (void *)&prcopt_.antdel[0][0],   "m"      },
	{ "ant1-antdeln",    1,  (void *)&prcopt_.antdel[0][1],   "m"      },
	{ "ant1-antdelu",    1,  (void *)&prcopt_.antdel[0][2],   "m"      },
	{ "ant2-postype",    3,  (void *)&antpostype_[1],         POSOPT   },
	{ "ant2-name",       2,  (void *)&prcopt_.name[1],        ""       },
	{ "ant2-pos1",       1,  (void *)&antpos_[1][0],          "deg|m"  },
	{ "ant2-pos2",       1,  (void *)&antpos_[1][1],          "deg|m"  },
	{ "ant2-pos3",       1,  (void *)&antpos_[1][2],          "m|m"    },
	{ "ant2-anttype",    2,  (void *)&prcopt_.anttype[1],     ""       },
	{ "ant2-antdele",    1,  (void *)&prcopt_.antdel[1][0],   "m"      },
	{ "ant2-antdeln",    1,  (void *)&prcopt_.antdel[1][1],   "m"      },
	{ "ant2-antdelu",    1,  (void *)&prcopt_.antdel[1][2],   "m"      },
	{ "ant2-maxaveep",   0,  (void *)&prcopt_.maxaveep,       ""       },
	{ "ant2-initrst",    3,  (void *)&prcopt_.initrst,        SWTOPT   },

	{ "misc-timeinterp", 3,  (void *)&prcopt_.intpref,        SWTOPT   },
	{ "misc-sbasatsel",  0,  (void *)&prcopt_.sbassatsel,     "0:all"  },
	{ "misc-rnxopt1",    2,  (void *)&prcopt_.rnxopt[0],      ""       },
	{ "misc-rnxopt2",    2,  (void *)&prcopt_.rnxopt[1],      ""       },
	{ "misc-pppopt",     2,  (void *)&prcopt_.pppopt,         ""       },

	{ "file-satantfile", 2,  (void *)&filopt_.satantp,        ""       },
	{ "file-rcvantfile", 2,  (void *)&filopt_.rcvantp,        ""       },
	{ "file-staposfile", 2,  (void *)&filopt_.stapos,         ""       },
	{ "file-geoidfile",  2,  (void *)&filopt_.geoid,          ""       },
	{ "file-ionofile",   2,  (void *)&filopt_.iono,           ""       },
	{ "file-dcbfile",    2,  (void *)&filopt_.dcb,            ""       },
	{ "file-erpfile",    2,  (void *)&filopt_.erp,            ""       },
	{ "file-blqfile",    2,  (void *)&filopt_.blq,            ""       },
	{ "file-tempdir",    2,  (void *)&filopt_.tempdir,        ""       },
	{ "file-geexefile",  2,  (void *)&filopt_.geexe,          ""       },
	{ "file-solstatfile",2,  (void *)&filopt_.solstat,        ""       },
	{ "file-testfile",   2,  (void *)&filopt_.test,           ""       },

	{ "",0,NULL,"" } /* terminator */
};

/* rtkpro options table --------------------------------------------------------------------------- */
static rtkopt_t rtkopt_;
#define TIMOPT  "0:gpst,1:utc,2:jst,3:tow"
#define CONOPT  "0:dms,1:deg,2:xyz,3:enu,4:pyl"
#define FLGOPT  "0:off,1:std+2:age/ratio/ns"
#define ISTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,7:ntripcli,8:ftp,9:http"
#define OSTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripsvr"
#define FMTOPT  "0:rtcm2,1:rtcm3,2:oem4,3:oem3,4:ubx,5:ss2,6:hemis,7:skytraq,8:gw10,9:javad,10:nvs,11:binex,12:rt17,15:sp3"
#define NMEOPT  "0:off,1:latlon,2:single"
#define SOLOPT  "0:llh,1:xyz,2:enu,3:nmea"
#define MSGOPT  "0:all,1:rover,2:base,3:corr"
static opt_t rtkopts[]={
	{ "inpstr1-type",    3,  (void *)&rtkopt_.strtype[0],         ISTOPT },
	{ "inpstr2-type",    3,  (void *)&rtkopt_.strtype[1],         ISTOPT },
	{ "inpstr3-type",    3,  (void *)&rtkopt_.strtype[2],         ISTOPT },
	{ "inpstr1-path",    2,  (void *)&rtkopt_.strpath[0],         ""     },
	{ "inpstr2-path",    2,  (void *)&rtkopt_.strpath[1],         ""     },
	{ "inpstr3-path",    2,  (void *)&rtkopt_.strpath[2],         ""     },
	{ "inpstr1-format",  3,  (void *)&rtkopt_.strfmt[0],          FMTOPT },
	{ "inpstr2-format",  3,  (void *)&rtkopt_.strfmt[1],          FMTOPT },
	{ "inpstr3-format",  3,  (void *)&rtkopt_.strfmt[2],          FMTOPT },
	{ "inpstr2-nmeareq", 3,  (void *)&rtkopt_.nmeareq,            NMEOPT },
	{ "inpstr2-nmealat", 1,  (void *)&rtkopt_.nmeapos[0],         "deg"  },
	{ "inpstr2-nmealon", 1,  (void *)&rtkopt_.nmeapos[1],         "deg"  },
	{ "inpstr2-nmeahig", 1,  (void *)&rtkopt_.nmeapos[2],         "m"    },
	{ "outstr1-type",    3,  (void *)&rtkopt_.strtype[3],         OSTOPT },
	{ "outstr2-type",    3,  (void *)&rtkopt_.strtype[4],         OSTOPT },
	{ "outstr1-path",    2,  (void *)&rtkopt_.strpath[3],         ""     },
	{ "outstr2-path",    2,  (void *)&rtkopt_.strpath[4],         ""     },
	{ "logstr1-type",    3,  (void *)&rtkopt_.strtype[5],         OSTOPT },
	{ "logstr2-type",    3,  (void *)&rtkopt_.strtype[6],         OSTOPT },
	{ "logstr3-type",    3,  (void *)&rtkopt_.strtype[7],         OSTOPT },
	{ "logstr1-path",    2,  (void *)&rtkopt_.strpath[5],         ""     },
	{ "logstr2-path",    2,  (void *)&rtkopt_.strpath[6],         ""     },
	{ "logstr3-path",    2,  (void *)&rtkopt_.strpath[7],         ""     },

	{ "misc-svrcycle",   0,  (void *)&rtkopt_.svrcycle,           "ms"   },
	{ "misc-timeout",    0,  (void *)&rtkopt_.timeout,            "ms"   },
	{ "misc-reconnect",  0,  (void *)&rtkopt_.reconnect,          "ms"   },
	{ "misc-nmeacycle",  0,  (void *)&rtkopt_.nmeacycle,          "ms"   },
	{ "misc-buffsize",   0,  (void *)&rtkopt_.buffsize,           "bytes"},
	{ "misc-navmsgsel",  3,  (void *)&rtkopt_.navmsgsel,          MSGOPT },
	{ "misc-proxyaddr",  2,  (void *)&rtkopt_.proxyaddr,          ""     },
	{ "misc-fswapmargin",0,  (void *)&rtkopt_.fswapmargin,        "s"    },

	{ "",0,NULL,"" }
};

/* postpro options table -------------------------------------------------------------------------- */
static pstopt_t pstopt_;
#define OUTOPT  "0:off,1:on"
#define PREOPT  "1: only observed + 2: only predicted + 4: not combined"
static opt_t pstopts[]={
	{ "post-predict",    0,  (void *)&pstopt_.predict,             PREOPT},
	{ "post-timeinter",  1,  (void *)&pstopt_.time_inter,             "" },
	{ "post-timestart",  2,  (void *)&pstopt_.time_start.sep,         "" },
	{ "post-timeend",    2,  (void *)&pstopt_.time_end.sep,           "" },
	{ "post-roverobs",   2,  (void *)&pstopt_.rover_obs,              "" },
	{ "post-baseobs",    2,  (void *)&pstopt_.base_obs,               "" },
	{ "post-navigation", 2,  (void *)&pstopt_.nav,                    "" },
	{ "post-preciseeph", 2,  (void *)&pstopt_.prseph,                 "" },
	{ "post-satclock",   2,  (void *)&pstopt_.satclk,                 "" },
	{ "post-out1",       3,  (void *)&pstopt_.outflag[0],         OUTOPT },
	{ "post-out2",       3,  (void *)&pstopt_.outflag[1],         OUTOPT },
	{ "post-output1",    2,  (void *)&pstopt_.output[0],              "" },
	{ "post-output2",    2,  (void *)&pstopt_.output[1],              "" },

	{ "",0,NULL,"" }
};

/* post-processing option type -----------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
* defaults processing options --------------------------------------------------------------------- */
pstopt_t::pstopt_t() {
	predict=1;
	time_inter=30;
	time_start=time_end=gtime_t();
	outflag[0]=outflag[1]=0;
}
pstopt_t::~pstopt_t() {
}

/* rtk-processing option type ------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
* defaults processing options --------------------------------------------------------------------- */
rtkopt_t::rtkopt_t(){
	for (int i=0; i<7; i++){ strtype[i]=0; strpath[i]="\0"; }
	strfmt[0]=strfmt[1]=1; strfmt[2]=17;
	svrcycle=10; timeout=10000; reconnect=10000; nmeacycle=5000;
	fswapmargin=30; buffsize=32768; navmsgsel=0; nmeareq=0;
	nmeapos[0]=nmeapos[1]=nmeapos[2]=0;
	proxyaddr="\0";

	/* unset options */
	cmds[0]=cmds[1]=cmds[2]="\0";
	rropts[0]=rropts[1]=rropts[2]="\0";
	monitor=NULL;
}
rtkopt_t::~rtkopt_t(){
	if (monitor )delete monitor;
}

/* processing options type ---------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
* defaults processing options --------------------------------------------------------------------- */
prcopt_t::prcopt_t(){
	int i;
	mode=PMODE_SINGLE; soltype=0;
	nf=2; navsys=SYS_GPS;
	elmin=15.0*D2R; snrmask; 
	sateph=0; 
	modear=1; order=0; restime=3;
	sampling = 0.0;
	glomodear=1; bdsmodear=1; 
	iniamb=300; maxariter=1; 
	ionoopt =0; tropopt=0; 
	iondeg_n=iondeg_m=ion_nm=0;
	dynamics=0; tidecorr=0; 
	niter=1; codesmooth=0; 
	intpref=0; sbascorr=0; 
	sbassatsel=0; rovpos=0; 
	refpos=0; 
	for (i=0;i<NFREQ;i++) eratio[i]=100.0;
	err[0]=0.003; err[1]=0.003; err[2]=0.0; err[3]=1.0; 
	std[0]=30.0; std[1]=0.02; std[2]=0.3; 
	stdrate[0]=1E-3; stdrate[1]=1E-4; stdrate[2]=stdrate[3]=30.0;
	sclkstab=5E-12; 
	thresar[0]=3.0; thresar[1]=0.9999; thresar[2]=0.25; thresar[3]=0.1; thresar[4]=0.05;
	thresar[5]=0.0; thresar[6]=0.0; thresar[7]=0.0;
	elmaskhold=0.0; 
	ion_gf=0.05; maxtdiff=30.0;
	maxres=30.0;
	ru[0]=ru[1]=ru[2]=rb[0]=rb[1]=rb[2]=0.0;
	for (i=0; i<2; i++){
		baseline[i]=0.0; anttype[i]="\0"; pcvr[i]=pcv_t();
		antdel[i][0]=antdel[i][1]=antdel[i][3]=0.0; ;
	}
	posopt[0]=posopt[1]=posopt[2]=0;
	for (i=0;i<MAXSAT;i++) exsats[i]=0;
	maxaveep=0; initrst=0;
	rnxopt[0]=rnxopt[1]="\0";
	syncsol=0; 
	freqopt=0;
	pppopt="\0";
}
prcopt_t::~prcopt_t(){
}
/* Implementation functions ------------------------------------------------------- */

/* solution options type -----------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
* defaults solution output options ---------------------------------------------------------------- */
solopt_t::solopt_t(){
	posf=SOLF_LLH; times=TIMES_GPST; 
	timef=1; timeu=3; 
	degf=0; outhead=1; 
	outopt=0; datum=0; 
	origin=0;
	height=0; geoid=0; 
	solstatic=0; sstat=0; 
	trace=0; 
	nmeaintv[0]=0.0; nmeaintv[1]=0.0; 
	sep=""; prog=""; 
}
solopt_t::~solopt_t(){
}
/* implementation functions ------------------------------------------------------- */
/* solution option to field separator --------------------------------------------- */
string solopt_t::opt2sep(){
	if (sep.size()<=0) sep=" ";
	else if (!sep.compare("\\t")) sep="\t";
	return sep;
}
/* write solution header to output stream ----------------------------------------- */
int solopt_t::outsolheads(unsigned char *buff){
	const char *s1[]={ "WGS84","CGCS2000" },*s2[]={ "ellipsoidal","geodetic" };
	const char *s3[]={ "GPST","UTC ","JST " };
	char *p=(char *)buff;
	string sepat=sep;
	int t_decimal=timeu<0 ? 0 : (timeu>12 ? 13 : timeu+1);

	if (posf==SOLF_NMEA||posf==SOLF_STAT||posf==SOLF_GSIF) {
		return 0;
	}
	if (outhead) {
		p+=sprintf(p,"%s (",COMMENTH);
		if (posf==SOLF_XYZ) p+=sprintf(p,"x/y/z-ecef=WGS84");
		else if (posf==SOLF_ENU) p+=sprintf(p,"e/n/u-baseline=WGS84");
		else p+=sprintf(p,"lat/lon/height=%s/%s",s1[datum],s2[height]);
		p+=sprintf(p, ",Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp,ns=# of satellites)\n");
	}
	p+=sprintf(p, "%s  %-*s%s",COMMENTH,(timef ? 16 : 8)+t_decimal,s3[times],sepat.c_str());

	if (posf==SOLF_LLH) { /* lat/lon/hgt */
		if (degf) {
			p+=sprintf(p, "%16s%s%16s%s%10s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s\n",
				"latitude(d'\")",sepat.c_str(),"longitude(d'\")",sepat.c_str(),"height(m)",
				sepat.c_str(),"Q",sepat.c_str(),"ns",sepat.c_str(),"sdn(m)",sepat.c_str(),
				"sde(m)",sepat.c_str(),"sdu(m)",sepat.c_str(),"sdne(m)",sepat.c_str(),
				"sdeu(m)",sepat.c_str(),"sdue(m)");
		}
		else {
			p+=sprintf(p, "%14s%s%14s%s%10s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s\n",
				"latitude(deg)",sepat.c_str(),"longitude(deg)",sepat.c_str(),"height(m)",
				sepat.c_str(),"Q",sepat.c_str(),"ns",sepat.c_str(),"sdn(m)",sepat.c_str(),
				"sde(m)",sepat.c_str(),"sdu(m)",sepat.c_str(),"sdne(m)",sepat.c_str(),
				"sdeu(m)",sepat.c_str(),"sdun(m)");
		}
	}
	else if (posf==SOLF_XYZ) { /* x/y/z-ecef */
		p+=sprintf(p, "%14s%s%14s%s%14s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s\n",
			"x-ecef(m)",sepat.c_str(),"y-ecef(m)",sepat.c_str(),"z-ecef(m)",sepat.c_str(),"Q",
			sepat.c_str(),"ns",sepat.c_str(),"sdx(m)",sepat.c_str(),"sdy(m)",sepat.c_str(),
			"sdz(m)",sepat.c_str(),"sdxy(m)",sepat.c_str(),"sdyz(m)",sepat.c_str(),"sdzx(m)");
	}
	else if (posf==SOLF_ENU) { /* e/n/u-baseline */
		p+=sprintf(p, "%14s%s%14s%s%14s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s\n",
			"e-baseline(m)",sepat.c_str(),"n-baseline(m)",sepat.c_str(),"u-baseline(m)",
			sepat.c_str(),"Q",sepat.c_str(),"ns",sepat.c_str(),"sde(m)",sepat.c_str(),
			"sdn(m)",sepat.c_str(),"sdu(m)",sepat.c_str(),"sden(m)",sepat.c_str(),"sdnu(m)",
			sepat.c_str(),"sdue(m)");
	}
	return p-(char *)buff;
}

/* file options type ---------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
filopt_t::filopt_t(){
}
filopt_t::~filopt_t(){
}

/* RINEX options type --------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
rnxopt_t::rnxopt_t(){
}
rnxopt_t::~rnxopt_t(){
}

/* all options ---------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
option_t::option_t(){
	pstopt=pstopt_t();
	rtkopt=rtkopt_t();
	prcopt=prcopt_t();
	solopt[0]=solopt[1]=solopt_t();
	filopt=filopt_t();
	rnxopt=rnxopt_t();
}
option_t::~option_t(){
}
/* reset static system options to default ----------------------------------------- */
void option_t::resetsysopts(){
	int i,j;

	rtkopt_=rtkopt_t();
	prcopt_=prcopt_t();
	solopt1_=solopt_t();
	solopt2_=solopt_t();
	filopt_=filopt_t();
	for (i=0; i<2; i++) antpostype_[i]=0;
	elmask_=15.0;
	elmaskhold_=0.0;
	for (i=0; i<2; i++) for (j=0; j<3; j++) {
		antpos_[i][j]=0.0; snrmask_[j]="\0";
	}
	exsats_ ="\0";
}
/* string option to enum (int) ---------------------------------------------------- */
int option_t::str2enum(const string str,const string comment,int *val){
	size_t strp,nlen=0;
	if ((strp=comment.find(str))==string::npos) return 0;
	if (comment[strp-1]!=':') return 0;
	while (strp>=nlen+2) {
		if (comment[strp-2-nlen]>='0'&&comment[strp-2-nlen]<='9')
			nlen++;
		else break;
	}
	*val=stoi(comment.substr(strp-1-nlen,nlen));
	return 1;
}
/* enum (int) to string option ---------------------------------------------------- */
int option_t::enum2str(string &str,const string comment,int val){
	size_t strp1,strp2;
	string num=to_string(val)+':';
	if ((strp1=comment.find(num))==string::npos) { str=to_string(val); return 0; }
	if ((strp2=comment.find(',',strp1+num.length()))==string::npos&&
		(strp2=comment.find(')',strp1+num.length()))==string::npos){
		str=comment.substr(strp1+num.length());
		return 1;
	}
	else str=comment.substr(strp1+num.length(),strp2-strp1-num.length());
	return 1;
}
/* discard space characters at tail ----------------------------------------------- */
void option_t::chop(string &str){
	size_t strp;
	if ((strp=str.find('#'))!=string::npos) str.erase(strp); /* '#' means comment */
	while(str.length()>0&&!isgraph((int)str.back())) str.pop_back();
}
/* search option ---------------------------------------------------------------------
* search option record
* args   : string name		I  option name (const)
*          opt_t  *opts     I  options table
*                              (terminated with table[i].name="")
* return : option record (NULL: not found)
-----------------------------------------------------------------------------------*/
opt_t * option_t::searchopt(const string name,const opt_t *opts){
	for (int i=0; opts[i].name!=""; i++)
		if (opts[i].name.find(name)!=string::npos) return (opt_t *)(opts+i);
	return NULL;
}
/* string to option value --------------------------------------------------------- */
int option_t::str2opt(opt_t *opt, const string str){
	switch (opt->format){
		case 0: *(int	 *)opt->var=stoi(str); break;
		case 1: *(double *)opt->var=stod(str); break;
		case 2: *(string *)opt->var=str; break;
		case 3: return str2enum(str,opt->comment,(int *)opt->var); break;
		default: return 0;
	}
	return 1;
}
/* load options ----------------------------------------------------------------------
* load options from file
* args   : string   file    I  options file
*          opt_t  *opts     IO options table
*                              (terminated with table[i].name="")
* return : status (1:ok,0:error)
*---------------------------------------------------------------------------------- */
int option_t::loadopts(const string file, opt_t *opts){
	ifstream inf;
	opt_t *opt;
	string buff;
	int n=0;
	size_t strp;

	inf.open(file,ios::in);
	if (!inf.is_open()) return 0;

	while (getline(inf,buff)){
		n++;
		chop(buff);

		if (buff[0]=='\0') continue;

		if((strp=buff.find('='))==string::npos) continue;

		string name=buff.substr(0,strp),value=buff.substr(strp+1);
		chop(name);

		if (!(opt=searchopt(name,opts))) continue;

		if (!str2opt(opt,value)) continue;
	}
	
	inf.close();

	return 1;
}
/* system options buffer to options ----------------------------------------------- */
void option_t::buff2sysopts(){
	double pos[3]={0},*rr;
	string buff,id;
	int i,j,sat,p,pe,*ps;

	prcopt_.elmin     =elmask_    *D2R;
	prcopt_.elmaskhold=elmaskhold_*D2R;

	for (i=0; i<2; i++) {
		ps=i==0 ? &prcopt_.rovpos : &prcopt_.refpos;
		rr=i==0 ? prcopt_.ru : prcopt_.rb;

		if (antpostype_[i]==0) { /* lat/lon/hgt */
			*ps=0;
			pos[0]=antpos_[i][0]*D2R;
			pos[1]=antpos_[i][1]*D2R;
			pos[2]=antpos_[i][2];
			pos2ecef(pos,WGS84,rr);
		}
		else if (antpostype_[i]==1) { /* xyz-ecef */
			*ps=0;
			rr[0]=antpos_[i][0];
			rr[1]=antpos_[i][1];
			rr[2]=antpos_[i][2];
		}
		else *ps=antpostype_[i]-1;
	}
	/* excluded satellites */
	for (i=0; i<MAXSAT; i++) prcopt_.exsats[i]=0;
	if (exsats_[0]!='\0'){
		buff=exsats_;
		for (p=0,j=0; p<buff.length(); p++){
			pe=p;
			if ((p=buff.substr(p).find(' '))==string::npos)
				p=buff.length();
			else p+=pe;
			if(buff[pe]=='+') id=buff.substr(pe+1,p-pe-1); else id=buff.substr(pe,p-pe);
			if(!(sat=satid2no(id))) continue;
			prcopt_.exsats[sat-1]=buff[pe]=='+'?2:1;
		}
	}

	/* snrmask */
	for (i=0; i<NFREQ; i++){
		for (j=0; j<9; j++) prcopt_.snrmask.mask[i][j]=0.0;
		buff=snrmask_[i];
		for (p=0,j=0; p<buff.length();p++){
			pe=p;
			if ((p=buff.substr(p).find(','))==string::npos)
				p=buff.length(); 
			else p+=pe;
			prcopt_.snrmask.mask[i][j++]=stod(buff.substr(pe,p-pe));
		}
	}
	/* number of frequency (4:L1+L5) */
	if (prcopt_.nf==4) {
		prcopt_.nf=3;
		prcopt_.freqopt=1;
	}
}
/* get system options ----------------------------------------------------------------
* get system options
* return : none
* notes  : to load system options, use loadopts() before calling the function
*---------------------------------------------------------------------------------- */
void option_t::getsysopts(){
	
	buff2sysopts();

	pstopt=pstopt_;
	rtkopt=rtkopt_;
	prcopt=prcopt_;
	solopt[0]=solopt1_;
	solopt[1]=solopt2_;
	filopt=filopt_;

	solopt[0].opt2sep();
	solopt[1].opt2sep();
}
/* read post options -------------------------------------------------------------- */
int option_t::readpostopt(const string file) {
	/* read system options */
	if (!loadopts(file,sysopts)) return 0;
	/* read post options */
	if (!loadopts(file,pstopts)) return 0;

	getsysopts();

	pstopt.time_start.str2time(pstopt.time_start.sep);
	pstopt.time_end.str2time(pstopt.time_end.sep);
	return 1;
}
/* read rtk options --------------------------------------------------------------- */
int option_t::readrtkopt(const string file){
	
	/* read system options */
	if (!loadopts(file,sysopts)) return 0;
	/* read rtk options */
	if (!loadopts(file,rtkopts)) return 0;

	getsysopts();

	return 1;
}