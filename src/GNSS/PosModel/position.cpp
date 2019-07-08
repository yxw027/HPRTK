#include "GNSS/PosModel/position.h"
#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
#define SQR(x)      ((x)*(x))
#define SQRT(x)     ((x)<=0.0?0.0:sqrt(x))

#define NX          (4+3)				/* SPP: # of estimated parameters */

#define MAXITR      10					/* max number of iteration for point pos */
#define ERR_CBIAS   0.3					/* code bias error std (m) */

/* position types --------------------------------------------------------------------------------- */
/* single rtk-position class -------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
single_t::single_t(){
	stat=numL=ns=resflag=niter=0;
	for (int i=0; i<4; i++) ncode[0][i]=ncode[1][i]=0;
}
single_t::~single_t(){
	Lobs.clear(); Acoe.clear(); Rvar.clear();
	Rvec.clear(); Xpar.clear(); Rx.clear();
}
/* Implementation functions ------------------------------------------------------- */
/* system code to system number (1-4) --------------------------------------------- */
int single_t::syscd2num(int sys) {
	switch (sys) {
		case SYS_GPS: return 0;
		case SYS_QZS: return 0;
		case SYS_SBS: return 0;
		case SYS_GLO: return 1;
		case SYS_GAL: return 2;
		case SYS_CMP: return 3;
	}
	return 0;
}
/* update solution vectors -----------------------------------------------------------
*  argv :  int      rovbas    I   receiver index (1:rover, 2:base) */
void single_t::init_sol(int rovbas){
	/* rover station */
	if (rovbas==1){
		sol.erase(sol.begin()); sol.push_back(sol_t(this));
		solp=&sol.back();
		obsp=obsr;
		obsp->errmsg="";
		ncd=ncode[0];
	}
	/* base station */
	else if (rovbas==2){
		b_sol.erase(b_sol.begin()); b_sol.push_back(sol_t(this));
		solp=&b_sol.back();
		obsp=obsb;
		obsp->errmsg="";
		ncd=ncode[1];
	}
}
/* reset satellite status --------------------------------------------------------- */
void single_t::resetsat(){
	for (int i=0; i<MAXSAT; i++){
		for (int j=0; j<NFREQ; j++) 
			ssat[i].vsat[j]=ssat[i].snr[j]=0;
	}
}
/* carrier-phase bias (fcb) correction -------------------------------------------- */
void single_t::corr_phase_bias(){
	double lam;
	int code;
	
	/* rover observation */
	for (int i=0;i<obsr->n;i++) for (int j=0;j<NFREQ;j++){
		if(!(code=obsr->data[i].code[j])) continue;
		if((lam=nav->lam[obsr->data[i].sat-1][j])==0.0) continue;
		/* correct phase bias (cyc) */
		obsr->data[i].L[j]=nav->ssr[obsr->data[i].sat-1].pbias[code-1]/lam;
	}
	/* base observation */

	for (int i=0; i<obsb->n; i++) for (int j=0; j<NFREQ; j++){
		if (!(code=obsb->data[i].code[j])) continue;
		if ((lam=nav->lam[obsb->data[i].sat-1][j])==0.0) continue;
		/* correct phase bias (cyc) */
		obsb->data[i].L[j]=nav->ssr[obsb->data[i].sat-1].pbias[code-1]/lam;
	}

}
/* test SNR mask ---------------------------------------------------------------------
* test SNR mask
* args   : int    base      I   rover or base-station (0:rover,1:base station)
*          int    freq      I   frequency (0:L1,1:L2,2:L3,...)
*          double el        I   elevation angle (rad)
*          double snr       I   C/N0 (dBHz)
*          snrmask_t *mask  I   SNR mask
* return : status (1:masked,0:unmasked)
* --------------------------------------------------------------------------------- */
int single_t::testsnr(int base,int freq,double el,double snr,
	const snrmask_t *mask){
	double minsnr,a;
	int i;

	if (!mask->ena[base]||freq<0||freq>=NFREQ) return 0;

	a=(el*R2D+5.0)/10.0;
	i=(int)floor(a); a-=i;
	if (i<1) minsnr=mask->mask[freq][0];
	else if (i>8) minsnr=mask->mask[freq][8];
	else minsnr=(1.0-a)*mask->mask[freq][i-1]+a*mask->mask[freq][i];

	return snr<minsnr;
}
/* get tgd parameter (m) ---------------------------------------------------------- */
double single_t::gettgd(int sat){
	int i;
	for (i=0; i<nav->n; i++) {
		if (nav->eph[i].sat!=sat) continue;
		return CLIGHT*nav->eph[i].tgd[0];
	}
	return 0.0;
}
/* pseudorange measurement error variance ----------------------------------------- */
double single_t::varerr(const obsd_t &obs,int sys){
	double sfact,varr;
	if (sys!=SYS_CMP) sfact=sys==SYS_GLO ? EFACT_GLO : (sys==SYS_SBS ? EFACT_SBS : EFACT_GPS);
	else sfact = binary_search(BDS_GEO,BDS_GEO+6,obs.prn)? EFACT_CMP_G : EFACT_CMP;
	varr=SQR(opt->eratio[0])*(SQR(opt->err[0])+SQR(opt->err[1]/sin(obs.azel[1])));
	if (opt->sppiono==IONOOPT_IFLC) varr*=SQR(3.0); /* iono-free = 0 */
	return SQR(sfact)*varr;
}
/* verify excluded satellite ------------------------------------------------------ */
int single_t::satexclude(obsd_t &obs){
	 /* ephemeris unavailable */
	if (obs.svh!=0) {
		obs.errmsg="unhealthy satellite! ";
		return 1;
	}
	/*if (obs.sys==SYS_CMP&&obs.prn>18) 
		return 1;*/

	/* exclude satellite according to option */
	if (opt->exsats[obs.sat-1]==1) return 1; /* excluded satellite */
	if (opt->exsats[obs.sat-1]==2) return 0; /* included satellite */
	if (!(obs.sys&opt->navsys)) return 1; /* unselected sat sys */

	if (obs.sys==SYS_QZS) obs.svh&=0xFE; /* mask QZSS LEX health */
	
	return 0;
}
/* exclude observation with large residual -------------------------------- */
void single_t::exc_largeres(){
	for (int i=0; i<obsp->n; i++){
		if (fabs(obsp->data[i].res[0])>opt->maxres) { 
			obsp->data[i].exc=1;
			obsp->data[i].errmsg="high residual!\n";
		}
	}
}
/* psendorange with code bias correction and combination -------------------------- */
double single_t::prange(obsd_t &obs,const int iter){
	const double *lam=nav->lam[obs.sat-1];
	double PC,P1,P2,P1_P2,P1_C1,P2_C2,gamma;
	int i=0,j=1;

	obs.dcbvar=0.0;

	/* L1-L2 for GPS/GLO/QZS, L1-L5 for GAL/SBS */
	if (NFREQ>=3&&(obs.sys&(SYS_GAL|SYS_SBS))) j=2;

	if (NFREQ<2||lam[i]==0.0||lam[j]==0.0) return 0.0;

	/* test snr mask */
	if (iter>0) {
		if (testsnr(0,i,obs.azel[1],obs.SNR[i]*0.25,&opt->snrmask)) {
			return 0.0;
		}
		if (opt->sppiono==IONOOPT_IFLC) {
			if (testsnr(0,j,obs.azel[1],obs.SNR[j]*0.25,&opt->snrmask)) return 0.0;
		}
	}
	gamma=SQR(lam[j])/SQR(lam[i]); /* f1^2/f2^2 */
	P1=obs.P[i];
	P2=obs.P[j];
	P1_P2=nav->cbias[obs.sat-1][0];
	P1_C1=nav->cbias[obs.sat-1][1];
	P2_C2=nav->cbias[obs.sat-1][2];

	/* if no P1-P2 DCB, use TGD instead */
	if (P1_P2==0.0&&(obs.sys&(SYS_GPS|SYS_GAL|SYS_QZS))) {
		P1_P2=(1.0-gamma)*gettgd(obs.sat);
	}
	if (opt->sppiono==IONOOPT_IFLC) { /* dual-frequency */

		if (P1==0.0||P2==0.0) return 0.0;
		if (obs.code[i]==CODE_L1C) P1+=P1_C1; /* C1->P1 */
		if (obs.code[j]==CODE_L2C) P2+=P2_C2; /* C2->P2 */

		/* iono-free combination */
		PC=(gamma*P1-P2)/(gamma-1.0);
	}
	else { /* single-frequency */

		if (P1==0.0) return 0.0;
		if (obs.code[i]==CODE_L1C) P1+=P1_C1; /* C1->P1 */
		PC=P1-P1_P2/(1.0-gamma);
	}
	if (opt->sateph==EPHOPT_SBAS) PC-=P1_C1; /* sbas clock based C1 */

	obs.dcbvar=SQR(ERR_CBIAS); /* code bias error std added to ovar */

	return PC;
}
/* compute observation\coefficient\covariance matrix ------------------------------ */
int single_t::codearr(const int iter){
	numL=0;		/* number of used observation (size of L) */
	double pos[3];	/* geodetic position (lat,lon,h) */

	/* reset each matrix (except X=[4+3,1]) */
	Acoe.clear(); Lobs.clear(); Rvec.clear();
	for (int i=0; i<4; i++) ncd[i]=0; //number of code observation of each system

	/* ecef (XYZ) to geodetic position */
	ecef2pos(Xpar.begin(),WGS84,pos);

	for (int i=ns=0; i<obsp->n&&i<MAXOBS; i++){
		int sys;		/* satellite system */
		double Pc;		/* psudorange with code bias correction */
		double dion=0,ionvar=0;
		double lam_L1;	/* f1 lamda */

		/* reset observation data */
		obsp->data[i].used=0;

		/* excluded satellite? */
		if (obsp->data[i].exc==1) continue;
		if (satexclude(obsp->data[i])) {
			obsp->data[i].errmsg="excluded satellite!\n";
			obsp->data[i].exc=1;  continue;
		}

		if (!(sys=satsys(obsp->data[i].sat,NULL))) { 
			obsp->data[i].errmsg="unknown satellite!\n";
			continue; 
		}

		 /* reject duplicated observation data */
		 if (i<obsp->n-1&&i<MAXOBS-1&&obsp->data[i].sat==obsp->data[i+1].sat) {
			 obsp->data[i].errmsg="duplicated observation data with next data!\n";
			 i++;
			 continue;
		 }
		 /* geometric distance/azimuth/elevation angle */
		 if (
			 (obsp->data[i].dist = geodist(obsp->data[i].posvel,Xpar.begin(),obsp->data[i].sigvec)) <= 0.0 
			 || (satazel(pos,obsp->data[i].sigvec,obsp->data[i].azel) < opt->elmin
				&& fabs(obsp->data[i].res[0])<1000)
			 ) {
			 obsp->data[i].errmsg="low elevation!\n";
			 continue;
		 }

		 /* pseudorange with code bias correction */
		 if ((Pc=prange(obsp->data[i],iter))==0.0) { 
			 obsp->data[i].errmsg="no avaliable pseudorange!\n";
			 continue;
		 }

		 /* ionospheric corrections */
		 if (!sppionf->correction(&obsp->data[i],nav,pos)) { 
			 obsp->data[i].errmsg="ionosphere correction error!\n";
			 continue; 
		 }

		 /* GPS-L1 -> L1/B1 */
		 if (opt->sppiono!=IONOOPT_IFLC&&(lam_L1=nav->lam[obsp->data[i].sat-1][0])>0.0) {
			 double Coe=SQR(lam_L1/WaveLengths[0])*obsp->data[i].ionmap;
			 dion=obsp->data[i].dion*Coe;
			 ionvar=obsp->data[i].ionvar*Coe*Coe;
		 }

		 /* tropospheric corrections */
		 if (!spptrof->correction(&obsp->data[i],nav,pos,0.7)) { 
			 obsp->data[i].errmsg="troposphere correction error!\n";
			 continue; 
		 }

		 /* antenna delta */
		 double antR[3]={0};
		 recantfunc.recantoff(opt,0,&obsp->data[i]);

		 /* pseudorange residual 
		  * Lpc = Pc - (dist + cdtr - cdts + dtro + dion) */
		 double Lpc= Pc - (obsp->data[i].dist + Xpar[3] - CLIGHT*obsp->data[i].dts[0] +
			 obsp->data[i].dtro + dion + obsp->data[i].dant[0]);
		 /* add Lpc to L */
		 Lobs.push_back(Lpc);

		 /* coefficient matrix */
		 for (int j=0; j<NX; j++) Acoe.push_back( j<3 ? -obsp->data[i].sigvec[j] : (j==3 ? 1.0 : 0.0) );
		 /* time system and receiver bias offset correction */
		 if      (sys==SYS_GLO) { Lobs[numL]-=Xpar[4]; Acoe[4+(numL)*NX]=1.0; ncd[1]++; }
		 else if (sys==SYS_GAL) { Lobs[numL]-=Xpar[5]; Acoe[5+(numL)*NX]=1.0; ncd[2]++; }
		 else if (sys==SYS_CMP) { Lobs[numL]-=Xpar[6]; Acoe[6+(numL)*NX]=1.0; ncd[3]++; }
		 else ncd[0]++;

		 /* error variance 
		  * Rpc = code + sat + dcb + tro + ion */
		 double Rpc=varerr(obsp->data[i],sys)+obsp->data[i].svar+
			 obsp->data[i].dcbvar+obsp->data[i].trovar+ionvar;
		 /* add Rpc to R */
		 Rvec.push_back(Rpc);

		 /* update obsp->data[i] */
		 obsp->data[i].used=1; obsp->data[i].res[0]=Lobs.back(); obsp->data[i].ovar[0]=Rpc;

		 numL++; ns++;
	}
	/* constraint to avoid rank-deficient 
	 * if no this system */
	for (int i=0; i<4; i++) {
		if (ncd[i]) continue;
		Lobs.push_back(0.0);
		for (int j=0; j<NX; j++) Acoe.push_back(j==i+3 ? 1.0 : 0.0);
		Rvec.push_back(0.01);
		numL++;
	}
	return numL;
}
/* write state information to state_file ------------------------------------------ */
void single_t::write_state() {
	/* set float format */
	state_file.setf(ios::fixed);
	state_file << setprecision(4);

	/* time */
	obsr->data[0].time.time2str(3);
	state_file << obsr->data[0].time.sep;

	/* BODY of state ------------------------- */
	/* satellite parameters (prn, az, el) */
	state_file << setprecision(3) << "\n";
	for (int i=0; i<obsp->n; i++) { 
		if (!obsp->data[i].used) continue;
		ssat_t *sss=&ssat[ obsp->data[i].sat-1 ];
		state_file << setw(4) << " " << sss->id << ":"; //prn
		state_file << setw(9) << obsp->data[i].azel[0]*R2D << 
			          setw(9) << obsp->data[i].azel[1]*R2D;//az el
		state_file << "\n";
	}
}
/* update satellite sate vector (ssat) -------------------------------------------- */
void single_t::update_ssat(){
	if (obsp!=obsb){
		for (int i=0; i<MAXSAT; i++) {
			ssat[i].vs=0;
			ssat[i].azel[0]=ssat[i].azel[1]=0.0;
			ssat[i].resp[0]=ssat[i].resc[0]=0.0;
			ssat[i].snr[0]=0;
		}
		for (int i=0; i<obsp->n; i++) {
			ssat[obsp->data[i].sat-1].azel[0]=obsp->data[i].azel[0];
			ssat[obsp->data[i].sat-1].azel[1]=obsp->data[i].azel[1];
			ssat[obsp->data[i].sat-1].snr[0]=obsp->data[i].SNR[0];
			if (!obsp->data[i].used) continue;
			ssat[obsp->data[i].sat-1].vs=1;
			ssat[obsp->data[i].sat-1].resp[0]=obsp->data[i].res[0];
		}
	}
}
/* update solution vector (sol) --------------------------------------------------- */
void single_t::update_sol(){
	solp->type=0;
	gtime_t sss=obsp->data[0].time;
	solp->time=*sss.timeadd(-Xpar[3]/CLIGHT);
	solp->NL=numL;
	solp->xclk[0]=Xpar[3]/CLIGHT; /* receiver clock bias (s) */
	solp->xclk[1]=Xpar[4]/CLIGHT; /* glo-gps time offset (s) */
	solp->xclk[2]=Xpar[5]/CLIGHT; /* gal-gps time offset (s) */
	solp->xclk[3]=Xpar[6]/CLIGHT; /* bds-gps time offset (s) */
	for (int j=0; j<3; j++) solp->xdyc[j]=Xpar[j];
	for (int j=0; j<3; j++) for (int k=0; k<3; k++)
		solp->vdyc[k+j*6]=Rx[k+j*NX];
	solp->ns=(unsigned char)ns;
	solp->age=solp->ratio=0.0;
	solp->stat=opt->sateph==EPHOPT_SBAS ? SOLQ_SBAS : SOLQ_SINGLE;

	/* update Rx_ALL */
	if (opt->mode==PMODE_SINGLE&&obsp==obsr)
		for (int i=0; i<3; i++) for (int j=0; j<3; j++) Rx_ALL[j+i*N_ALL]=Rx[j+i*NX];
}
/* growth variance of static rover position ------------------------------- */
double single_t::growth_rate_static() {
	return tt<3?SQR(0.01*tt):SQR(0.1);
}
/* estimate receiver position ----------------------------------------------------- */
int single_t::singlepos(){
	Rx.assign(NX*NX,0.0);
	Xpar.assign(NX,0.0);
	/* NX: x,y,z dtr(GPS), dtr*3(GLO,GLA,CMP) */
	for (int i=0; i<3; i++) { 
		Xpar[i]=solp->xdyc[i]; 
	}

	/* initialize observation (exc) */
	for (int i=0; i<obsp->n; i++) obsp->data[i].exc=0;

	/* compute receiver position and clock bias */
	resflag=0; int nnn=0;
	for (niter=0; niter<MAXITR; niter++){
		int lsqflag;
		nnn++;

		/* compute observation\coefficient\covariance matrix */
		codearr(niter);

		/* verify if lack of valid observation */
		if (numL<=NX){
			obsp->errmsg="lack of valid sats ns!";
			break;
		}

		/* update Rvec using Rvar */
		Rvar.assign(numL*numL,0.0);
		for (int j=0; j<numL; j++) Rvar[j*numL+j]=Rvec[j];

		/* adjustment with least square function */
		if ((lsqflag=adjfunc->lsq(Acoe,Lobs,Rvar,Xpar,Rx,numL,NX))==-1){
			obsp->errmsg="least square error!";
			break;
		}

		if (lsqflag>=1||niter>=MAXITR-1) {
			/* exclude observation with large residual */
			if (resflag==0) exc_largeres();
			/* out solution if already exlclude large residual */
			else if (resflag>0) {
				single_t::update_sol();
				obsp->used=ns;
				return 1;
			}
			/* restart estimate without large residual observation */
			resflag++; niter=0;
		}
	}

	Lobs.clear(); Acoe.clear(); Rvar.clear();
	Rvec.clear(); Xpar.clear(); Rx.clear();

	return 0;
}
/* single point position -------------------------------------------------------------
* recnum	: receiver number (1:rover,2:base)
* --------------------------------------------------------------------------------- */
int single_t::single(){

	solp->stat=SOLQ_NONE;

	if (obsp->n <= 0){ obsp->errmsg="no observation data\n"; return 0; }

	solp->time=obsp->data[0].time; obsp->errmsg = "\0";

	/* satellite positons, velocities and clocks */
	if (satfunc->satposclk(obsp,nav)<=4) { 
		obsp->errmsg="no enough available emphemeris!\n ";
		return 0; 
	}

	/* estimate receiver position */
	stat=singlepos();
	if (opt->mode==PMODE_SINGLE) write_state();
	if (stat) single_t::update_ssat();

	return stat;
}
/* base station single-position -------------------------------------------- */
int single_t::basepos(){
	init_sol(2);

	/* intialize base position */
	if (opt->refpos!=POSOPT_SINGLE&&opt->mode!=PMODE_MOVEB) 
		for (int i=0; i<3; i++) b_sol.back().xdyc[i]=opt->rb[i];

	return single();
}
/* single rtk-position function --------------------------------------------------- */
int single_t::rtkpos(){
	init_sol(1);

	return single();
}