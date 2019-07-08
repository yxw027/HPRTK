/* Data Classes File */
/*
 * 2017-7-25 class of observation data
 *
 *
 *
 *
 * 
 */
#include "GNSS/DataClass/data.h"
 /*class of one epoch observation data ------------------------------------------------------------ */
obsd_t::obsd_t(){
	sat=prn=sys=0;
	time=sigtime=gtime_t();
	for (int i=0; i<NFREQ+NEXOBS; i++)
		SNR[i]=LLI[i]=code[i]=L[i]=D[i]=P[i]=res[i]=ovar[i]=0.0;
	for (int i=0; i<3; i++){
		sigvec[i]=0.0;
		dant[i]=0.0;
		for (int j=0; j<2; j++) posvel[i*2+j]=0.0;
	}
	dts[0]=dts[1]=azel[0]=azel[1]=0.0;
	rcv=0;
	dist=dcbvar=used=exc=sat=svar=svh=ionmap=dion=ionvar=dtro=trovar=0;
}
obsd_t::~obsd_t(){
}
/* implementation functions ------------------------------------------------------- */
/* reset the whole data ----------------------------------------------------------- */
void obsd_t::reset(){
	int i;

	sat='\0';
	/* observation data */
	for (i=0; i<NFREQ+NEXOBS; i++){
		SNR[i]=LLI[i]=code[i]='\0';
		L[i]=P[i]=D[i]=0.0;
	}
	for (i=0;i<3;i++) res[i]=ovar[i]=dant[i]=0;
	used=0;
	/* satellite data */
	svar=svh=0;
	for (i=0;i<2;i++){
		azel[i]=dts[i]=0.0;
		for (int j=0;j<3;j++) posvel[i*3+j]=0.0;
	}
	/* correction data */
	ionmap=dion=ionvar=dtro=trovar=0.0;
}
/* reset satellite data ----------------------------------------------------------- */
void obsd_t::satreset(){
	/* satellite data */
	svar=svh=0;
	for (int i=0; i<2; i++){
		azel[i]=dts[i]=0.0;
		for (int j=0; j<3; j++) posvel[i*3+j]=0.0;
	}
}
/* updata signal time use pseudorange---------------------------------------------- */
int obsd_t::sigtime_opsr(){
	double pr;
	int i;

	sigtime = time;
	for (i=0, pr=0.0; i<NFREQ; i++) if ((pr=P[i])!=0.0) break;
	if (i<NFREQ){
		sigtime.timeadd(-pr/CLIGHT);
		return 1;
	}
	else { errmsg = "no pseudorange"; return 0; }
}
/* update signal time use satellite clock bias ------------------------------------ */
void obsd_t::sigtime_sclk(){
	sigtime.timeadd(dts[0]);
}

/* station informtations -------------------------------------------------------------------------- */
sta_t::sta_t(){
	antsetup=itrf=deltype=hgt=0;
	for (int i=0;i<3;i++) pos[i]=del[i]=0.0;
}
sta_t::~sta_t(){
}

/* class of station's information and observation chains ------------------------------------------ */
obs_t::obs_t(){
	n=used=0;
	rcv=0;
	sta=sta_t();
}
obs_t::~obs_t(){
	data.clear();
}
void obs_t::reset(){
	n=0;
	data.clear();
}

/* GPS/QZS/GAL broadcast ephemeris type ----------------------------------------------------------- */
eph_t::eph_t(){
	sat=iode=iodc=sva=svh=week=code=flag=0;
	toe=toc=ttr=gtime_t();
	A=e=i0=OMG0=omg=M0=deln=OMGd=idot=0;
	crc=crs=cuc=cus=cic=cis=0;
	toes=fit=f0=f1=f2=Adot=ndot=0;
	tgd[0]=tgd[1]=tgd[2]=tgd[3]=0;
}
eph_t::~eph_t(){
}

/* GLONASS broadcast ephemeris type --------------------------------------------------------------- */
geph_t::geph_t(){
	sat=iode=frq=svh=sva=age=0;
	toe=tof=gtime_t();
	for (int i=0;i<3;i++) pos[i]=vel[i]=acc[i]=0;
	taun=gamn=dtaun=0;
}
geph_t::~geph_t(){
}

/* SBAS ephemeris type ---------------------------------------------------------------------------- */
seph_t::seph_t(){
	sat=sva=svh=0;
	t0=tof=gtime_t();
	af0=af1=0;
	for (int i=0;i<3;i++) pos[i]=vel[i]=acc[i]=0;
}
seph_t::~seph_t(){
}

/* precise ephemeris type ------------------------------------------------------------------------- */
peph_t::peph_t(){
	time=gtime_t();
	for (int i=0; i<MAXSAT; i++){
		for (int j=0;j<4;j++)
			pos[i][j]=std[i][j]=vel[i][j]=vst[i][j]=0;
		for (int j=0;j<3;j++)
			cov[i][j]=vco[i][j]=0;
	}
}
peph_t::~peph_t(){
}

/* precise clock type ----------------------------------------------------------------------------- */
pclk_t::pclk_t(){
	time=gtime_t();
	for (int i=0;i<MAXSAT;i++)
		clk[i]=std[i]=0;
}
pclk_t::~pclk_t(){
}

/* almanac type ----------------------------------------------------------------------------------- */
alm_t::alm_t(){
	sat=svh=svconf=week=0;
	toa=gtime_t();
	A=e=i0=OMG0=omg=M0=OMGd=0;
	toas=f0=f1=0;
}
alm_t::~alm_t(){
}

/* TEC grid type ---------------------------------------------------------------------------------- */
tec_t::tec_t(){
	time=gtime_t();
	rb=0;
	for (int i=0;i<3;i++)
		lats[i]=lons[3]=hgts[i]=ndata[i]=0;
}
tec_t::~tec_t(){
	data.clear(); rms.clear();
}

/* satellite fcb data type ------------------------------------------------------------------------ */
fcbd_t::fcbd_t(){
	ts=te=gtime_t();
	for (int i=0;i<MAXSAT;i++)
		bias[i][0]=bias[i][1]=bias[i][2]=
		std[i][0]=std[i][1]=std[i][2]=0;
}
fcbd_t::~fcbd_t(){
}

/* earth rotation parameter data type ------------------------------------------------------------- */
erpd_t::erpd_t(){
	mjd=xp=yp=xpr=ypr=ut1_utc=lod=0;
}
erpd_t::~erpd_t(){
}

/* earth rotation parameter type ------------------------------------------------------------------ */
erp_t::erp_t(){
	n=nmax=0;
}
erp_t::~erp_t(){
	data.clear();
}

/* antenna parameter type ------------------------------------------------------------------------- */
pcv_t::pcv_t(){
	sat=0;
	ts=te=gtime_t();
	for (int i=0; i<NFREQ; i++){
		off[i][0]=off[i][1]=off[i][2]=0;
		for (int j=0;j<19;j++)
			var[i][j]=0;
	}
}
pcv_t::~pcv_t(){
}

/* SBAS fast correction type ---------------------------------------------------------------------- */
sbsfcorr_t::sbsfcorr_t(){
	t0=gtime_t();
	prc=rrc=dt=iodf=udre=ai=0;
}
sbsfcorr_t::~sbsfcorr_t(){
}

/* SBAS long term satellite error correction type ------------------------------------------------- */
sbslcorr_t::sbslcorr_t(){
	t0=gtime_t();
	iode=daf0=daf1=0;
	dpos[0]=dpos[1]=dpos[2]=dvel[0]=dvel[1]=dvel[2]=0;
}
sbslcorr_t::~sbslcorr_t(){
}

/* SBAS satellite correction type ----------------------------------------------------------------- */
sbssatp_t::sbssatp_t(){
	sat=0;
	fcorr=sbsfcorr_t();
	lcorr=sbslcorr_t();
}
sbssatp_t::~sbssatp_t(){
}

/* SBAS satellite corrections type ---------------------------------------------------------------- */
sbssat_t::sbssat_t(){
	iodp=0;
	nsat=0;
	tlat=0;
	for (int i=0;i<MAXSAT;i++)
		sat[i]=sbssatp_t();
}
sbssat_t::~sbssat_t(){
}

/* SBAS ionospheric correction type --------------------------------------------------------------- */
sbsigp_t::sbsigp_t(){
	t0=gtime_t();
	lat=lon=give=delay=0;
}
sbsigp_t::~sbsigp_t(){
}

/* SBAS ionospheric corrections type -------------------------------------------------------------- */
sbsion_t::sbsion_t(){
	iodi=0;
	nigp=0;
	for (int i=0;i<MAXNIGP;i++)
		igp[i]=sbsigp_t();
}
sbsion_t::~sbsion_t(){
}

/* DGPS/GNSS correction type ---------------------------------------------------------------------- */
dgps_t::dgps_t(){
	t0=gtime_t();
	prc=rrc=iod=udre=0;
}
dgps_t::~dgps_t(){
}

/* SSR correction type ---------------------------------------------------------------------------- */
ssr_t::ssr_t(){
	t0[0]=t0[1]=t0[2]=t0[3]=t0[4]=t0[5]=gtime_t();
	update=0;
	iode=iodcrc=ura=refd=hrclk=yaw_ang=yaw_rate=0;
	for (int i=0; i<3; i++){
		deph[i]=ddeph[i]=dclk[i]=0.0;
		for (int j=0; j<2; j++) udi[i*2+j]=iod[i*2+j]=0;
	}
	for (int i=0; i<MAXCODE; i++) cbias[i]=pbias[i]=stdpb[i]=0.0;
}
ssr_t::~ssr_t(){
}

/* QZSS LEX ephemeris type ------------------------------------------------------------------------ */
lexeph_t::lexeph_t(){
	toe=tof=gtime_t();
	af0=af1=tgd=sat=0;
	health=ura='\0';
	for (int i=0;i<3;i++)
		pos[i]=vel[i]=acc[i]=jerk[i]=0;
	for (int i=0;i<8;i++)
		isc[i]=0;
}
lexeph_t::~lexeph_t(){
}

/* QZSS LEX ionosphere correction type ------------------------------------------------------------ */
lexion_t::lexion_t(){
	t0=gtime_t();
	pos0[0]=pos0[1]=tspan=0;
	coef[0][0]=coef[1][0]=coef[2][0]=
		coef[0][1]=coef[1][1]=coef[2][1]=0;
}
lexion_t::~lexion_t(){
}

/* stec data type --------------------------------------------------------------------------------- */
stec_t::stec_t(){
	time=gtime_t();
	sat=flag='\0';
	ion=std=azel[0]=azel[1]=0;
}
stec_t::~stec_t(){
}

/* trop data type --------------------------------------------------------------------------------- */
trop_t::trop_t(){
	time=gtime_t();
	trp[0]=trp[1]=trp[2]=
		std[0]=std[1]=std[2]=0;
}
trop_t::~trop_t(){
}

/* ppp corrections type --------------------------------------------------------------------------- */
pppcorr_t::pppcorr_t(){
	nsta=0;
	for (int i=0; i<MAXSAT; i++){
		rr[i][0]=rr[i][1]=rr[i][2]=0;
		ns[i]=nsmax[i]=nt[i]=ntmax[i]=0;
		stec[i]=NULL;
		trop[i]=NULL;
	}
}
pppcorr_t::~pppcorr_t(){
	for (int i=0; i<MAXSAT; i++){
		stec[i]=NULL;
		trop[i]=NULL;
	}
}

/* class of navigation data ----------------------------------------------------------------------- */
nav_t::nav_t(){
	int i,j;
	/* number of ephemeris */
	n=ng=ns=ne=nc=na=nt=nf=0;
	nmax=ngmax=nsmax=nemax=ncmax=namax=ntmax=nfmax=0;

	for (i=0; i<4; i++){
		/* utc parameters */
		utc_gps[i]=utc_glo[i]=utc_gal[i]=utc_qzs[i]=utc_cmp[i]=
			utc_irn[i]=utc_sbs[i]=0.0;
		/* model parameters */
		ion_gps[i]=ion_gps[i+1]=0.0;
		ion_gal[i]=0.0;
		ion_qzs[i]=ion_qzs[i+1]=0.0;
		ion_cmp[i]=ion_cmp[i+1]=0.0;
		ion_irn[i]=ion_irn[i+1]=0.0;
		/* glonass code-phase bias */
		glo_cpbias[i]=0.0;
	}
	for (i=0; i<66; i++){
		ocean_par[0][i]=ocean_par[1][0]=0.0;
	}
	leaps=0;
	erp=erp_t();
	sbssat=sbssat_t();
	lexion=lexion_t();
	pppcorr=pppcorr_t();

	/* MAXSAT array */
	for (i=0; i<MAXSAT; i++){
		for (j=0;j<NFREQ;j++) lam[i][j]=0.0;
		cbias[i][0]=cbias[i][1]=cbias[i][2]=0.0;
		wlbias[i]=0.0;
		pcvs[i]=pcv_t();
		dgps[i]=dgps_t();
		ssr[i]=ssr_t();
		lexeph[i]=lexeph_t();
	}
	for (i=0;i<MAXRCV;i++)
		rbias[i][0][0]=rbias[i][0][1]=rbias[i][0][2]=
		rbias[i][1][0]=rbias[i][1][1]=rbias[i][1][2]=0.0;
	for (i=0;i<MAXPRNGLO+1;i++) glo_fcn[i]='\0';
	for (i=0;i<MAXBAND+1;i++) sbsion[i]=sbsion_t();
}
nav_t::~nav_t(){
	eph.clear(); geph.clear(); seph.clear(); peph.clear();
	pclk.clear(); alm.clear(); tec.clear(); fcb.clear();
}

/* SBAS message type ------------------------------------------------------------------------------ */
sbsmsg_t::sbsmsg_t(){
	week=tow=prn=0;
	for (int i=0;i<29;i++)
		msg[i]='\0';
}
sbsmsg_t::~sbsmsg_t(){
}

/* SBAS messages type ----------------------------------------------------------------------------- */
sbs_t::sbs_t(){
	n=nmax=0;
}
sbs_t::~sbs_t(){
	msgs.clear();
}

/* QZSS LEX message type -------------------------------------------------------------------------- */
lexmsg_t::lexmsg_t(){
	prn=type=alert=ttt=0;
	stat=snr='\0';
	for (int i=0;i<212;i++)
		msg[i]='\0';
}
lexmsg_t::~lexmsg_t(){
}
/* QZSS LEX messages type ------------------------------------------------------------------------- */
lex_t::lex_t(){
	n=nmax=0;
}
lex_t::~lex_t(){
	msgs.clear();
}