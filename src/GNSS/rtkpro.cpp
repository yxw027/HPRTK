#include "GNSS/rtkpro.h"

#include "BaseFunction/basefunction.h"
#include "GNSS/PosModel/position.h"
#include "GNSS/ReadFile/readfile.h"

/* raw_t */
#include "Decode/rtcm.h"
#include "Decode/raw/binex.h"
#include "Decode/raw/cmr.h"
#include "Decode/raw/crescent.h"
#include "Decode/raw/gw10.h"
#include "Decode/raw/javad.h"
#include "Decode/raw/novatel.h"
#include "Decode/raw/nvs.h"
#include "Decode/raw/septentrio.h"
#include "Decode/raw/skytraq.h"
#include "Decode/raw/superstar2.h"
#include "Decode/raw/trimble17.h"
#include "Decode/raw/ublox.h"

/* constant --------------------------------------------------------------------------------------- */
#define SQR(x)     ((x)<0.0?-(x)*(x):(x)*(x))
#define SQRT(x)    ((x)<0.0?0.0:sqrt(x))
#define MAXION_DT  120.0						/* max ion-change time (s) */
#define NESTSLIP   999							/* flag of no estimated cycle slip */
/* for geometry-free combination cycle slip compute ------------------------------- */
adjfunc_t lsqf;
/* sqrt of covariance ------------------------------------------------------------- */
static double sqvar(double covar)
{
	return covar<0.0 ? -sqrt(-covar) : sqrt(covar);
}
/* solution type -------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
sol_t::sol_t(){
	NF=NL=NI=NT=NA=NG=0;
	ND=3;
	NC=4;
	time=soltime=gtime_t();
	for (int i=0;i<6;i++) posvel[i]=0.0;
	for (int i=0;i<9;i++) posvar[i]=0.0;
	for (int i=0;i<3;i++) lat[i]=lon[i]=0.0;
	age=ratio=thres=0.0;
	type=stat='\0';
	ns=0;
}
sol_t::sol_t(const rtk_t *rtk){
	NF=rtk->NF;
	ND=rtk->ND;;
	NT=rtk->NT;
	NG=rtk->NG;
	NC=rtk->NC;

	xdyc.assign(6,0.0); vdyc.assign(6*6,0.0);
	if (NT) { xtro.assign(NT,0.0); vtro.assign(NT*NT,0.0); }
	if (NG) { xglo.assign(NG,0.0); vglo.assign(NG*NG,0.0); }
	xclk.assign(NSYS,0.0); vclk.assign(NSYS*NSYS,0.0);

	NL=NA=0;

	time=soltime=gtime_t();
	for (int i=0; i<6; i++) posvel[i]=0.0;
	for (int i=0; i<9; i++) posvar[i]=0.0;
	for (int i=0; i<3; i++) lat[i]=lon[i]=0.0;
	age=ratio=thres=0.0;
	type=stat='\0';
	ns=0;
}
sol_t::~sol_t(){
	xdyc.clear(); xion.clear(); xtro.clear(); 
	xamb.clear(); xglo.clear(); xclk.clear();

	vdyc.clear(); vion.clear(); vtro.clear(); 
	vamb.clear(); vglo.clear(); vclk.clear();
}
/* Implementation functions ------------------------------------------------------- */
/* dynamic covariance to ecef covariance ---------------------------------- */
void sol_t::dyc2ecef(){
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			ecefvar[i+j*3]=vdyc[j+i*6];
}
/* ecef solution ------------------------------------------------------------------ */
void sol_t::ecef(const solopt_t *opt){

	string str;
	strpv="";
	/* position */
	for (int i=0; i<3; i++) strpv+=doul2str(14,4," ",xdyc[i],str)+opt->sep;
	/* solution state */
	strpv+=int2str(3," ",stat,str)+opt->sep+int2str(3," ",ns,str)+opt->sep;
	/* position variance */
	/* xx yy zz */
	for (int i=0; i<3; i++)
		strpv+=doul2str(8,4," ",SQRT(vdyc[i+i*6]),str)+opt->sep;
	/* xy yz zx */
	strpv+=doul2str(8,4," ",sqvar(vdyc[1]),str)+opt->sep;
	strpv+=doul2str(8,4," ",sqvar(vdyc[8]),str)+opt->sep;
	strpv+=doul2str(8,4," ",sqvar(vdyc[2]),str);
}
/* ecef position to LLH ----------------------------------------------------------- */
void sol_t::llh(const solopt_t *opt){
	/* compute llh and covariance to posvel and posvar */
	ecef2pos(xdyc.begin(),opt->datum,posvel);
	dyc2ecef();
	covenu(posvel,ecefvar,posvar);
	posvel[0]=posvel[0]*R2D; posvel[1]=posvel[1]*R2D;

	/* geodetic height */
	/*if (opt->height==1) posvel[2]-=;*/

	string str;
	strpv="";
	/* latitude and longitude ddd.ddd or ddd mm ss */
	if (opt->degf){
		deg2dms(posvel[0],lat,5); //latitude
		deg2dms(posvel[1],lon,5); //longitude
		//latitude
		strpv+=doul2str(4,0," ",lat[0],str)+opt->sep+
			   doul2str(2,0,"0",lat[1],str)+opt->sep+
			   doul2str(8,5,"0",lat[2],str)+opt->sep;
		//longitude
		strpv+=doul2str(4,0," ",lon[0],str)+opt->sep+
			   doul2str(2,0,"0",lon[1],str)+opt->sep+
			   doul2str(8,5,"0",lon[2],str)+opt->sep;
	}
	else 
		strpv+=doul2str(14,9," ",posvel[0],str)+opt->sep+
		       doul2str(14,9," ",posvel[1],str)+opt->sep;
	/* height */
	strpv+=doul2str(10,4," ",posvel[2],str)+opt->sep;
	/* solution state */
	strpv+=int2str(3," ",stat,str)+opt->sep+int2str(3," ",ns,str)+opt->sep;
	/* position covariance */
	strpv+=doul2str(8,4," ",sqvar(posvar[4]),str)+opt->sep;
	strpv+=doul2str(8,4," ",sqvar(posvar[0]),str)+opt->sep;
	strpv+=doul2str(8,4," ",sqvar(posvar[8]),str)+opt->sep;
	strpv+=doul2str(8,4," ",sqvar(posvar[1]),str)+opt->sep;
	strpv+=doul2str(8,4," ",sqvar(posvar[2]),str)+opt->sep;
	strpv+=doul2str(8,4," ",sqvar(posvar[5]),str);
	
}
/* ecef position to ENU ----------------------------------------------------------- */
void sol_t::enu(const solopt_t *opt,rtk_t *rtk){
	/* compute rover enu and covariance to posvel and posvar */
	/* pos represents base llh */
	double pos[3],rr[3];
	double *origin = opt->origin==0? rtk->opt->rb : rtk->opt->ru;
	for (int i=0; i<3; i++) rr[i]=xdyc[i]-origin[i];
	ecef2pos(origin,opt->datum,pos);
	dyc2ecef();
	covenu(pos,ecefvar,posvar);
	ecef2enu(pos,rr,posvel);

	/* write formated solution to strpv */
	string str;
	strpv="";
	/* position */
	for (int i=0; i<3; i++) strpv+=doul2str(14,4," ",posvel[i],str)+opt->sep;
	/* solution state */
	strpv+=int2str(3," ",stat,str)+opt->sep+int2str(3," ",ns,str)+opt->sep;
	/* position covariance */
	for (int i=0; i<3; i++) strpv+=doul2str(8,4," ",SQRT(posvar[4*i]),str)+opt->sep;
	strpv+=doul2str(8,4," ",sqvar(posvar[1]),str)+opt->sep;
	strpv+=doul2str(8,4," ",sqvar(posvar[5]),str)+opt->sep;
	strpv+=doul2str(8,4," ",sqvar(posvar[2]),str);
}
/* ecef position to EMEA ---------------------------------------------------------- */
void sol_t::nmea(const solopt_t *opt){
	string str;
	strpv="";
	/* position */
	for (int i=0; i<3; i++) strpv+=doul2str(14,4," ",xdyc[i],str)+opt->sep;
	/* solution state */
	strpv+=int2str(3," ",stat,str)+opt->sep+int2str(3," ",ns,str)+opt->sep;
	/* position variance */
	/* xx yy zz */
	for (int i=0; i<3; i++) 
		strpv+=doul2str(8,4," ",SQRT(vdyc[i*6+i]),str)+opt->sep;
	/* xy yz zx */
	strpv+=doul2str(8,4," ",sqvar(vdyc[1]),str)+opt->sep;
	strpv+=doul2str(8,4," ",sqvar(vdyc[8]),str)+opt->sep;
	strpv+=doul2str(8,4," ",sqvar(vdyc[2]),str);
}
/* solution position -------------------------------------------------------------- */
string sol_t::forposvel(const solopt_t *opt,rtk_t *rtk){
	switch (opt->posf){
		case SOLF_LLH:  llh(opt);     break;
		case SOLF_ENU:  enu(opt,rtk); break;
		case SOLF_NMEA: nmea(opt);    break;
		case SOLF_XYZ:
		default:        ecef(opt);
	}
	return strpv;
}
/* solution time ------------------------------------------------------------------ */
string sol_t::fortime(const solopt_t *opt){
	int timeu=opt->timeu<0 ? 0 : (opt->timeu>12 ? 12 : opt->timeu);
	soltime = time;
	if (opt->times>=TIMES_UTC) soltime.gpst2utc();
	if (opt->times==TIMES_BDT) soltime.gpst2bdt();
	if (opt->timef==1) strtime=soltime.time2str(timeu);
	else {
		int week;
		double gpst=soltime.time2gpst(&week);
		if (86400*7-gpst<0.5/pow(10.0,timeu)){
			week++;
			gpst=0.0;
		}
		string strb;
		strtime=int2str(4,"0",week,strb)+" "+
			doul2str(6+(timeu<=0?0:timeu+1),timeu," ",gpst,strb);
	}
	return strtime;
}

/* satellite status type -----------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
ssat_t::ssat_t(){
	for (int i=0; i<NFREQ; i++){
		resp[i]=resc[i]=0;
		vsat[i]=snr[i]='\0';
	}
	id = "";
	sat=sys=vs='\0';
	azel[0]=azel[1]=0.0;
	/* ionosphere */
	ion_delay=ion_var=0.0;
	/* ambiguity */
	polyodr=3;
	sum_dt=0.0; max_sumdt=0.0;
	ambtime[0]=ambtime[1]=ambtime[2]=gtime_t();
	ambfirst[0]=ambfirst[1]=ambfirst[2]=gtime_t();
	phw=0.0;
	for (int i=0; i<2; i++) 
		gf12[i]=gf15[i]=
		lc12[i]=pc12[i]=
		mw12[i]=mw15[i]=
		nl12[i]=0.0;
	for (int i=0;i<NFREQ;i++){
		slipc[i]=half[i]='\0';
		fix[i]=0;
		dslip[0][i]=dslip[1][i]=0.0;
		reset[i]=0;
		sol_flag[i]=0;
		lock[i]=lock_con[i]=slipc[i]=0;
		amb[i]=ambvar[i]=0.0;
		amb_ave[i]=0.0;
		d_ave[i]=0.0;
	}
	MW12_con=MW15_con=0;
	d_ave[NFREQ]=d_ave[NFREQ+1]=0.0;
	lock_LC=0;
	lcamb=lcvar=lcamb_ave=plc=0.0;
}
ssat_t::~ssat_t(){
	obstime.clear(); dtime.clear();
	for (int i=0; i<NFREQ; i++){
		L[i].clear(); D[i].clear(); P[i].clear(); slip[i].clear(); d_ion[i].clear();
	}
	
}
/* Implementation functions ------------------------------------------------------- */
/* set coefficients matrix for estimate of gf-slip -----------------------------------
* argv  : int flag       phase combination type {0:L12+L15,1:L12,2:L15}
* --------------------------------------------------------------------------------- */
void ssat_t::coe_gfslip(const int flag,const double *lam,
	vector<double> &An,vector<double> &Rn){
	/* L12+L15 */
	if (flag==0) {
		//An lam1,-lam2,0; lam1,0,0; 0,lam2,0; lam1,0,-lam5; 0,0,lam5  
		An.assign(5*3,0.0); Rn.assign(5*5,0.0);
		An[0]=An[3]=An[9]=lam[0],An[1]=-lam[1];An[7]=lam[1]; 
		An[10]=-lam[2]; An[14]=lam[2];
		Rn[0]=Rn[18]=0.01; Rn[6]=Rn[12]=Rn[24]=10.0;
		return ;
	}
	else {
		//An lam1,-lam2; lam1,0; lam2,0
		An.assign(3*2,0.0); Rn.assign(3*3,0.0);
		An[0]=An[2]=lam[0]; An[1]=-lam[flag]; An[4]=lam[flag];
		Rn[0]=0.01; Rn[4]=Rn[8]=10.0; 
	}
}
/* estimate cycle slip of gf ---------------------------------------------------------
*   Ln :  DN12 L1-P1(dt) L2-P2(dt) DN15 L5-P5(dt) 
*   Dn :  DN1,DN2,DN5
* --------------------------------------------------------------------------------- */
void ssat_t::est_gfslip(const double DN12,const double DN15,const int f12,const int f15,
	const double *lam){
	if (f12==0&&f15==0) return;
	int prev=polyodr-1,flag12=f12,flag15=f15;
	/* verify availability of P1 */
	if (P[0].back()==0.0||P[0][prev]==0.0) return;
	vector<double> Ln,An,Rn,Dn(3,0.0),Rx(9,0.0);
	/* Ln for L1L2 and verify avaliability of P2 */
	if (f12!=0&&P[1].back()!=0.0&&P[1][prev]!=0.0){
		Ln.push_back(DN12);
		Ln.push_back(
			single_time(amb_cmb(L[0].back()*lam[0],P[0].back()),
				amb_cmb(L[0][prev]*lam[0],P[0][prev]))
		);
		Ln.push_back(
			single_time(amb_cmb(L[1].back()*lam[1],P[1].back()),
				amb_cmb(L[1][prev]*lam[1],P[1][prev]))
		);
	}
	else flag12=0;
	/* Ln for L1L5 and verify availability of P5 */
	if (f15!=0&&P[2].back()!=0.0&&P[2][prev]!=0.0){
		Ln.push_back(DN15);
		if (f12==0)Ln.push_back(
			single_time(amb_cmb(L[0].back()*lam[0],P[0].back()),
				amb_cmb(L[0][prev],P[0][prev]))
		);
		Ln.push_back(
			single_time(amb_cmb(L[2].back()*lam[2],P[2].back()),
				amb_cmb(L[2][prev],P[2][prev]))
		);
	}
	else flag15=0;

	/* DN  and nx */
	if (flag12==0&&flag15==0) return ;
	if (flag12+flag15==2){
		coe_gfslip(0,lam,An,Rn);
		lsqf.lsq(An,Ln,Rn,Dn,Rx,5,3);
		for (int i=0; i<3; i++) dslip[1][i]=Dn[i];
	}
	else {
		int f2=flag15==0?1:2;
		coe_gfslip(f2,lam,An,Rn);
		lsqf.lsq(An,Ln,Rn,Dn,Rx,3,2);
		dslip[1][0]=Dn[0]; dslip[1][f2]=Dn[1];
	}
}
/* detect cycle slip functions ---------------------------------------------------- */
/* detect by LLI ------------------------------------------------------------------ */
void ssat_t::detslip_LLI(const obsd_t *rov,const obsd_t *bas,const prcopt_t *opt){
	unsigned char LLr[NFREQ]={ 0,0,0 },LLb[NFREQ]={ 0,0,0 };
	for (int i=0; i<opt->nf; i++){
		if (reset[i]) continue;
		LLr[i]=(rov ? rov->LLI[i] : 0)&3; LLb[i]=(bas ? bas->LLI[i] : 0)&3;
		half[i]=(LLr[i]&2)||(LLb[i]&2);
		/* slip: 6-7:rover,4-5:base,1:half,0:slip */
		slip[i].back()=(LLr[i]<<6)|(LLb[i]<<4)|(LLr[i]|LLb[i]);
		if (slip[i].back()&1)
			slip[i].back()|=1;
	}
}
/* detect by polynomial fitting --------------------------------------------------- */
void ssat_t::detslip_poly(const prcopt_t *opt,const double *lam){
	for (int i=0; i<opt->nf; i++){
		if (reset[i]) continue;
		int poly_f=1;
		/* test slip and D,L history */
		for (int j=0; j<polyodr+1; j++){
			if (L[i][j]==0.0||D[i][j]==0.0||(j>0&&j<polyodr&&slip[i][j]&1)){
				poly_f=0; continue; 
			}
		}
		
		if (sum_dt<=max_sumdt&&poly_f){
			/* polymonial fitting of L */
			vector<double> Ap((polyodr+1)*(polyodr*2),0.0),Lp(polyodr*2,0.0),Coe(polyodr+1,0.0);
			double sigma=0.0;
			for (int nl=0; nl<polyodr*2; nl++){
				Lp[nl]=nl<polyodr ? L[i][nl] : D[i][nl-polyodr];
				for (int nx=0; nx<polyodr+1; nx++){
					Ap[nx+nl*(polyodr+1)] = nl<polyodr ?
						pow(dtime[nx],nx) : (nx)*pow(dtime[nx],nx-1);
				}
			}
			/* compute Coe and sigma */
			if (polyest(Ap,Lp,Coe,polyodr*2,polyodr+1,sigma)==-1) return ;
			/* compute predicted pL to compare with L */
			vector<double> xdt,pL(1,0.0);
			for (int nx=0; nx<polyodr+1; nx++) xdt.push_back(pow(dtime.back(),nx));
			matmul_vec("NN",1,1,polyodr,1.0,Coe,xdt,0.0,pL);
			dslip[0][i]=L[i].back()-pL[0];
			if (fabs(dslip[0][i])>3*sigma) {
				slip[i].back()|=1;
			}
		}
	}
}
/* detect by geometry-free combination -----------------------------------------------
* if slip checked  gf[n]-gf[n-1] > ion_change_rate * dt + phase_noise 
* --------------------------------------------------------------------------------- */
void ssat_t::detslip_gf(const prcopt_t *opt,const double *lam){
	/* if dtime > 3min (ion change to much) return */
	if (dtime.back()>MAXION_DT||reset[0]||reset[1]||reset[2]) {
		return; 
	}
	double DN12=0.0,DN15=0.0;

	/* L1L5 */
	if (opt->nf>=3){
		if (gf15[1]!=0.0&&gf15[0]!=0.0){
			DN15 = gf15[1]-gf15[0];
			if (fabs(DN15)>opt->ion_gf*dtime.back()+3.0*opt->err[0])
				slip[0].back()|=1; slip[2].back()|=1;
		}
	}
	/* L1L2 */
	if (gf12[1]!=0.0&&gf12[0]!=0.0){
		DN12 = gf12[1]-gf12[0];
		if (fabs(DN12)>opt->ion_gf*obstime.back().timediff(ambtime[0])+3.0*opt->err[0]) {
			slip[0].back()|=1; slip[1].back()|=1;
		}
	}
}
/* detect by Melbourne-Wubbena ---------------------------------------------------- */
void ssat_t::detslip_MW(const prcopt_t *opt){
	/* sigma^2 of new MW and average MW */
	double s2=SQR(0.5),s2_ave;
	/* Melbourne-Wubbena ambiguity */
	if (MW12_con>0) {	/* mw12 */
		s2_ave=s2/MW12_con;
		if (fabs(mw12[1]-mw12[0])>4.0*sqrt(s2+s2_ave)) { 
			slip[0].back()|=1; slip[1].back()|=1; 
		}
	}
	if (MW15_con>0) {	/* mw15 */
		s2_ave=s2/MW15_con;
		if (fabs(mw15[1]-mw15[0])>3.0*sqrt(s2+s2_ave)) { 
			slip[0].back()|=1; slip[2].back()|=1; 
		}
	}

	return ;
}
/* public: ------------------------------------------------------------------------ */
/* initialize vectors with order of polynomial fitting ---------------------------- */
void ssat_t::init_vector(prcopt_t *opt){
	polyodr==opt->order>3 ? (opt->order<6 ? opt->order : 5) : 3;
	max_sumdt=opt->sampling*(polyodr-1)+opt->restime;

	obstime.assign(polyodr+1,gtime_t());
	dtime.assign(polyodr+1,0.0);
	
	for (int i=0; i<2; i++) 
		gf12[i]=gf15[i]=
		lc12[i]=pc12[i]=
		mw12[i]=mw15[i]=0.0;
		/*nl12[i]=0.0;*/
	for (int i=0; i<NFREQ; i++){
		L[i].assign(polyodr+1,0.0);
		D[i].assign(polyodr+1,0.0);
		P[i].assign(polyodr+1,0.0);
		slip[i].assign(polyodr+1,0);
	}
	for (int i=0; i<NFREQ; i++) d_ion[i].assign(20,0.0);
}
/* update vectors for current status ---------------------------------------------- */
void ssat_t::update_vector(const obsd_t *rov,const obsd_t *bas,
	const double *lam,const prcopt_t *opt){
	/* put in new element and delete the oldest element
	* for obstime,L,gf12,gf15,lc12,pc12,mw,phw,slip */
	/* time difference and sum */
	sum_dt-=dtime[1];
	dtime.erase(dtime.begin()); dtime[0]=0.0;
	dtime.push_back(rov->time.timediff(obstime.back()));
	sum_dt+=dtime.back();
	/* obstime */
	obstime.erase(obstime.begin()); obstime.push_back(rov->time);
	
	/* observation */
	for (int i=0; i<NFREQ; i++){
		/* L */
		L[i].erase(L[i].begin()); L[i].push_back(single_diff(rov,bas,i));
		/* D */
		D[i].erase(D[i].begin()); D[i].push_back(dopsingle_d(rov,bas,i));
		/* P */
		P[i].erase(P[i].begin()); P[i].push_back(single_diff(rov,bas,NFREQ+i));
		/* slip */
		slip[i].erase(slip[i].begin()); slip[i].push_back(0);
	}

	/* gf12, gf15 */
	gf12[0]=gf12[1]; gf12[1]=geometry_free(1,rov,bas,lam);
	gf15[0]=gf15[1]; gf15[1]=geometry_free(2,rov,bas,lam);
	/* lc12, pc12 */
	lc12[0]=lc12[1]; lc12[1]=iono_free(1,rov,bas,lam);
	pc12[0]=pc12[1]; pc12[1]=iono_free(4,rov,bas,lam);
	/* mw */
	mw12[1]=Mel_Wub(1,rov,bas,lam); mw15[1]=Mel_Wub(2,rov,bas,lam);
	/* narrow-lane */
	/*nl12[1]=Narrow(1,rov,bas,lam);*/
	for (int i=0; i<opt->nf; i++) {
		slip[i][0]=0;
		dslip[0][i]=dslip[1][i]=0.0;
	}
}
/* reset flag according to unsolved time interval and last ambiguity solution ----- */
void ssat_t::test_reset(const prcopt_t *opt){
	/* LC ambiguity */
	if (opt->ionoopt==IONOOPT_IFLC){
		reset[0]=reset[1]=0;
		if (opt->modear==ARMODE_INST||lcamb==0.0||
			(obstime.back().timediff(ambtime[0])>opt->restime)) {
			fix[0]=fix[1]=1;
			reset[0]=reset[1]=1; 
			lock[0]=lock[1]=0;
			lock_LC=lock_con[0]=lock_con[1]=0;
			ambfirst[0]=ambfirst[1]=obstime.back();
		}
		return ;
	}
	/* reset normal ambiguity if no amb or out of time */
	for (int i=0; i<opt->nf; i++){
		reset[i]=0;
		if (opt->modear==ARMODE_INST||amb[i]==0.0||
			(obstime.back().timediff(ambtime[i])>opt->restime)) {
			fix[i]=1;
			reset[i]=1; lock[i]=0; lock_con[i]=0;
			ambfirst[i]=obstime.back();
		}
	}
}
/* reset_ambiguity -------------------------------------------------------- */
void ssat_t::reset_amb(const prcopt_t *opt,const int freq) {
	if (opt->ionoopt==IONOOPT_IFLC) {
		for (int i=0; i<opt->nf; i++){
			fix[i]=1;
			reset[i]=1; lock[i]=0; lock_con[i]=0;
			ambfirst[i]=obstime.back();
		}
		lock_LC=0;
	}
	else {
		fix[freq]=1;
		reset[freq]=1; lock[freq]=0; lock_con[freq]=0;
		ambfirst[freq]=obstime.back();
	}
}
/* detect cycle slip -------------------------------------------------------------- */
void ssat_t::detect_slip(const obsd_t *rov,const obsd_t *bas,
	const prcopt_t *opt,const double *lam){
	/* detect by LLI */
	detslip_LLI(rov,bas,opt);
	/* detect by polynomial fitting of L */
	if (opt->slipmode&1) detslip_poly(opt,lam);
	/* detect by gf */
	if (opt->slipmode&2) detslip_gf(opt,lam);
	/* detect by TurboEdit (not used now) */
	if (opt->slipmode&4) detslip_MW(opt);
	for (int f=0; f<opt->nf; f++) if (slip[f].back()&1||half[f]&2) {
		if (f<2) lock_LC=0;
		reset[f]=1; lock[f]=0; lock_con[f]=0;
		ambfirst[f]=obstime.back();
	}
}
/* repair cycle slip (not used) --------------------------------------------------- */
void ssat_t::repair_slip(const double *lam,const prcopt_t *opt){
	double cyc_slip[NFREQ]={ 0.0 },var_slip[NFREQ]={ 0.0 };
	int n_rep[NFREQ]={ 0 };

	for (int i=0; i<opt->nf; i++){
		/* repair cycle slip if detected slip */
		if (slip[i].back()&1){
			slipc[i]++;
			/* average cycle slip between polynomial-fitting and gf combination */
			//poly
			if (dslip[0][i]!=0.0) { 
				cyc_slip[i]+=dslip[0][i]; n_rep[i]++; 
				var_slip[i]+=SQR(opt->slip_std*0.01);
			} 
			//gf
			if (dslip[1][i]!=0.0) { 
				cyc_slip[i]+=dslip[1][i]; n_rep[i]++;
				var_slip[i]+=2*SQR(opt->slip_std);
			}
			cyc_slip[i]/= n_rep[i]==0 ? 1 : n_rep[i];
			/* flag of no estimated cycle slip */
			if (n_rep[i]==0) n_rep[i]=NESTSLIP;
		}
	}
	/* LC ambiguity */
	if (opt->ionoopt==IONOOPT_IFLC){
		/* if cycle-slip detected */
		if (slip[0].back()&1||slip[1].back()&1){
			/* reset lock and ambtime */
			lock[0]=lock[1]=0;
			lock_LC=0;
			ambfirst[0]=ambfirst[1]=obstime.back();
			/* if ambiguity can be repaired */
			if (n_rep[0]!=NESTSLIP&&n_rep[1]!=NESTSLIP){
				fix[0]=fix[1]=1;
				double gamma=SQR(lam[0])/SQR(lam[1]);
				lcamb+=(gamma*lam[0]*cyc_slip[0]-lam[1]*cyc_slip[1])/(gamma-1.0);
				lcvar+=var_slip[0]+var_slip[1];
				slip[0].back()&=0xFF; slip[0].back()|=4; //repaired flag
				slip[1].back()&=0xFF; slip[1].back()|=4; //repaired flag
			}
			/* reset ambiguity parameters if can't repair cycle slip */
			else reset[0]=reset[1]=1; 
		}
	}
	/* normal ambiguity */
	else {
		for (int i=0; i<opt->nf; i++){
			/* if cycle-slip detected */
			if (slip[i].back()&1){
				/* reset lock and ambtime */
				lock[i]=0;
				lock_con[i]=0;
				ambfirst[i]=obstime.back();
				/* if ambiguity can be repaired */
				if (n_rep[i]!=NESTSLIP){
					fix[i]=1;
					amb[i]+=cyc_slip[i]; ambvar[i]+=var_slip[i]; 
					slip[i].back()&=0xFF; slip[i].back()|=4; //repaired flag
				}
				/* reset ambiguity parameters if can't repair cycle slip */
				else reset[i]=1;
			}
		}
	}
}
/* update ambiguity parameters -------------------------------------------- */
void ssat_t::update_amb(const double *lam,const prcopt_t *opt) {
	/* Melbourne-Wubbena ambiguity */
	if (opt->nf>=2) {	/* mw12 */
		if (reset[0]||reset[1]||mw12[0]==0.0) { d_ave[NFREQ]=mw12[0]=mw12[1]; MW12_con=1; }
		else {
			d_ave[NFREQ]=(mw12[1]-mw12[0])/(++MW12_con);
			mw12[0] += d_ave[NFREQ];
		}
	}
	if (opt->nf>2) {/* mw15 */
		if (reset[0]||reset[2]||mw15[0]==0.0) { mw15[0]=mw15[1]; MW15_con=1; }
		else mw15[0] += (mw15[1]-mw15[0])/(++MW15_con);
	}

	/* normal ambiguity */
	for (int i=0; i<opt->nf; i++) {
		if (P[i].back()==0.0||L[i].back()==0.0) {
			sol_flag[i]=0;
			continue;
		}
		sol_flag[i]=1;
		/* reset ambiguity */
		if (reset[i]) {
			fix[i]=1;
			// consider ionosphere effect for ambiguity
			d_ion[i].erase(d_ion[i].begin()); d_ion[i].push_back(0.0); //length for frequency i (m)
			if (opt->ionoopt!=IONOOPT_OFF&&P[0].back()!=0.0&&P[1].back()!=0.0) {
				d_ion[i].back()=2.0*SQR(lam[i]/lam[0])*(P[0].back()-P[1].back())/(SQR(lam[1]/lam[0])-1.0);
			}
			//normal ambiguity
			d_ave[i]=amb[i]=amb_ave[i]=L[i].back()-P[i].back()/lam[i]-d_ion[i].back()/lam[i]; 
			ambvar[i]=SQR(opt->std[0]);
		}
		/* update ambiguity and variance */
		else {
			d_ion[i].erase(d_ion[i].begin()); d_ion[i].push_back(0.0); //length for frequency i (m)
			/* average ambiguity */
			double ambi=0.0;
			// consider ionosphere effect for ambiguity
			if (opt->ionoopt!=IONOOPT_OFF&&P[0].back()!=0.0&&P[1].back()!=0.0) {
				d_ion[i].back()=2.0*SQR(lam[i]/lam[0])*(P[0].back()-P[1].back())/(SQR(lam[1]/lam[0])-1.0);
			}
			ambi=L[i].back()-P[i].back()/lam[i]-d_ion[i].back()/lam[i];
			d_ave[i]=(ambi-amb_ave[i])/(lock_con[i]+1);
			amb_ave[i]+=d_ave[i]; //normal ambiguity
			if (fix[i] == 1) {
				/* if initialization use average ambiguity */
				amb[i]=amb_ave[i];
				if (fix[i]==1) ambvar[i]=SQR(opt->std[0])/(lock_con[i]+1);
			}
			else if (fix[i]==2) ambvar[i]+=SQR(0.0003)*dtime.back();
			else if (fix[i]==3) ambvar[i]+=SQR(0.0003)*dtime.back();
		}
		/* update ambiguity lock indexes */
		ambtime[i]=obstime.back();
		lock[i]=ambtime[i].timediff(ambfirst[i]);
		lock_con[i]++;
	}

	/* LC ambiguity */
	if (opt->ionoopt==IONOOPT_IFLC) {
		/* if no osbervations */
		if (lc12[1]==0.0||pc12[1]==0.0) {
			sol_flag[0]=sol_flag[1]=0;
			return;
		}
		sol_flag[0]=sol_flag[1]=1;
		/* reset LC ambiguity */
		if (reset[0]||reset[1]) {
			fix[0]=fix[1]=1;
			d_ave[NFREQ+1]=lcamb=lcamb_ave=lc12[1]-pc12[1]; //LC ambiguity
			lcvar=3.0*SQR(opt->std[0]);
		}
		/* update ambiguity and variance */
		else {
			/* average ambiguity */
			double amb0=lc12[1]-pc12[1];
			d_ave[NFREQ+1]=(amb0-lcamb_ave)/(lock_LC+1);
			lcamb_ave+=d_ave[NFREQ+1]; //LC ambiguity
			if (fix[0] == 1) {
				/* if initialization use average ambiguity */
				lcamb=lcamb_ave;
				lcvar=3.0*SQR(opt->std[0])/(lock_LC+1);
			}
			else if (fix[0]==2) lcvar+=SQR(0.003)*dtime.back();
		}
		/* update LC ambiguity lock indexes */
		lock_LC++;
	}
}
/* ambiguity control type ----------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
//ambc_t::ambc_t(){
//	for (int i=0; i<4; i++){
//		epoch[i]=gtime_t();
//		LC[i]=LCv[i]=n[i]=0;
//	}
//	fixcnt=0;
//	for (int i=0;i<MAXSAT;i++) flags[i]='\0';
//}
//ambc_t::~ambc_t(){
//}

/* RTK control/result type ---------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
rtk_t::rtk_t(){
	/* num */
	neb=nfix=0;
	NF=ND=NI=NT=NG=NC=numF=NX=NXI=N_ALL=0;
	rb[0]=rb[1]=rb[2]=rb[3]=rb[4]=rb[5]=tt=0.0;
	/* class */
	opt=NULL;
	obsr=NULL; obsb=NULL;
	nav=NULL;
	/* string */
	msg = "\0";
	ssat.assign(MAXSAT,ssat_t());
	/* initialize function classes */
	satantfunc=satantenna_t();
	recantfunc=recantenna_t();
	parafunc=parafunc_t();
	satfunc=NULL;
	sppionf=NULL;
	spptrof=NULL;
	ionfunc=NULL;
	trofunc=NULL;
	adjfunc=NULL;
}
rtk_t::~rtk_t(){
	opt=NULL; nav=NULL;
	sol.clear(); b_sol.clear();
	state_file.close();
	
	if (sppionf) delete sppionf;
	if (spptrof) delete spptrof;
	if (satfunc) delete satfunc;
	if (ionfunc) delete ionfunc; 
	if (trofunc) delete trofunc;
	if (adjfunc) delete adjfunc;
}
/* Implementation functions ----------------------------------------------- */
/* initialize rtk control ------------------------------------------------- */
void rtk_t::rtkinit(){
	/* initialize parameters */
	NF=parafunc.N_Freqency(opt); 
	ND=parafunc.N_Dynamic(opt);
	NT=parafunc.N_Tro(opt);
	NG=parafunc.N_GLOIFB(opt);
	NC=parafunc.N_Clock(opt);
	NI=opt->ionoopt==IONOOPT_CONST? MAXSAT : 0;
	NX=ND+NT+NG+NC; NXI=N_ALL=NX+NI;
	if (opt->mode>PMODE_DGPS) {
		numF = opt->ionoopt==IONOOPT_IFLC ? 1 : NF;
		N_ALL+=numF*MAXSAT;
		Rx_ALL.assign(N_ALL*N_ALL,0.0);
	}
	/* initialize functions ---------------- */
	/* satellite ephemeris functions */
	switch (opt->sateph){
		case EPHOPT_BRDC:   satfunc=new broadcast_t;   break;
		case EPHOPT_PREC:   satfunc=new preciseph_t;   break;
		case EPHOPT_SBAS:   satfunc=new broadsbas_t;   break;
		case EPHOPT_SSRAPC: satfunc=new broadssrapc_t; break;
		case EPHOPT_SSRCOM: satfunc=new broadssrcom_t; break;
		case EPHOPT_LEX:    satfunc=new qzsslex_t;     break;
		default: satfunc=new satellite_t;
	}
	satfunc->satantfunc=&satantfunc;
	/* adjustment functions */
	switch (opt->adjustfunc){
		case ADJUST_LSA:     adjfunc=new lsadj_t;     break;
		case ADJUST_KALMAN:  adjfunc=new kalmanadj_t; break;
		case ADJUST_HELMERT: adjfunc=new helmert_t;   break;
		default: adjfunc=new lsadj_t;
	}

	/* SPP ionosphere function */
	switch (opt->sppiono){
		case IONOOPT_IFLC:  sppionf=new LCion_t;     break;
		case IONOOPT_BRDC:  sppionf=new broadion_t;  break;
		case IONOOPT_SBAS:  sppionf=new sbasion_t;   break;
		case IONOOPT_TEC:   sppionf=new ionexion_t;  break;
		case IONOOPT_QZS:   sppionf=new qzssion_t;   break;
		case IONOOPT_LEX:   sppionf=new lexioncor_t; break;
		default:sppionf=new ioncorr_t;
	}
	/* SPP troposhere function */
	switch (opt->spptrop){
		case TROPOPT_SAAS: spptrof=new saastro_t; break;
		case TROPOPT_SBAS: spptrof=new sbastro_t; break;
		default: spptrof=new trocorr_t;
	}
	/* ionosphere function */
	switch (opt->ionoopt){
		case IONOOPT_IFLC:  ionfunc=new LCion_t;     break;
		case IONOOPT_BRDC:  ionfunc=new broadion_t;  break;
		case IONOOPT_SBAS:  ionfunc=new sbasion_t;   break;
		case IONOOPT_TEC:   ionfunc=new ionexion_t;  break;
		case IONOOPT_QZS:   ionfunc=new qzssion_t;   break;
		case IONOOPT_LEX:   ionfunc=new lexioncor_t; break;
		case IONOOPT_CONST: ionfunc=new constion_t;  break;
		default:ionfunc=new ioncorr_t;
	}
	/* troposphere function */
	switch (opt->tropopt){
		case TROPOPT_SAAS: trofunc=new saastro_t; break;
		case TROPOPT_SBAS: trofunc=new sbastro_t; break;
		case TROPOPT_EST:
		case TROPOPT_ESTG: trofunc=new estitro_t; break;
		default: trofunc=new trocorr_t;
	}

	/* ambiguity parameters */
	nfix=neb=0;

	/* satellite status vectors [MAXSAT] */
	for (int i=0; i<MAXSAT; i++){
		ssat[i].init_vector(opt);
		ssat[i].sat=i+1;
		satno2id(i+1,ssat[i].id);
		ssat[i].sys=satsys(i+1,NULL);
		ssat[i].state_file=&state_file;
	}

	/* solution vectors [MAXSOLBUF] */
	sol.assign(MAXSOLBUF,sol_t(this));
	b_sol.assign(MAXSOLBUF,sol_t(this));
}
/* rtk-position function (virtual) ---------------------------------------- */
int rtk_t::basepos(){
	return 0;
}
/* rtk-position function -------------------------------------------------- */
int rtk_t::rtkpos(){
	return 0;
}



/* RTK server type -------------------------------------------------------------------------------- */
/* Constructor -------------------------------------------------------------------- */
rtksvr_t::rtksvr_t(){
	int i,j;
	/* num */
	tick=state=sampling=cyctime=nmeacycle=nmeareq=buffsize=navsel=nsbs=nsol=
		cputime=prcout=nave=nsb[0]=nsb[1]=0;
	for (i=0;i<3;i++){
		rb_ave[i]=nmeapos[i]=nb[i]=npb[i]=fobs[i]=0;
		for (j=0;j<10;j++) nmsg[i][j]=0;
	}
	for (i=0;i<MAXSTRRTK;i++) strtype[i]=0;

	/* classes */
	solopt[0]=solopt[1]=solopt_t();
	nav=new nav_t;
	nav->eph.assign(MAXSAT*2,eph_t()); 
	nav->geph.assign(NSATGLO*2,geph_t());
	nav->seph.assign(NSATSBS*2,seph_t());
	nav->n=MAXSAT*2; nav->ng=NSATGLO*2; nav->ns=NSATSBS*2;

	obs[0]=obs[1]=obs[2]=obs_t();

	for (i=0; i<MAXSTRRTK; i++) { stream[i]=NULL;}
	for (i=0; i<3; i++) { format[i]=0; buff[i]=pbuf[i]=NULL; data[i]=NULL;}
	sbuf[0]=sbuf[1]=NULL;
	rtk=NULL;
	moni=NULL;

	for (i=0;i<MAXSBSMSG;i++) sbsmsg[i]=sbsmsg_t();

	initlock(&lock);
}
rtksvr_t::~rtksvr_t() {
	if (nav) delete nav;
	for (int i=0; i<3; i++){
		if (buff[i]) delete [] buff[i];
		if (pbuf[i]) delete [] pbuf[i];
		if (data[i]) delete data[i];
	}
	for (int i=0; i<MAXSTRRTK; i++)
		if (stream[i]) delete stream[i];
	if (sbuf[0]) delete [] sbuf[0]; if (sbuf[1]) delete [] sbuf[1];
	if (rtk) delete rtk;
	moni=NULL;
}
/* Implementation functions ------------------------------------------------------- */
/* initialzie function ------------------------------------------------------------ */
/* set sat\rcv antenna information ---------------------------------------------------
* argv   :  int   satrcv   0:satellite,1:receiver
* --------------------------------------------------------------------------------- */
void rtksvr_t::setpcv(inatx_t *atx,int satrcv){
	/* update satellite antenna to rtk->nav */
	if (satrcv==0){
		for (int i=0; i<MAXSAT; i++){
			if (!(satsys(i+1,NULL)&rtk->opt->navsys)) continue;
			for (int np=0; np<atx->pcv.size(); np++){
				if (atx->pcv[np].sat!=i+1) continue;
				nav->pcvs[i]=atx->pcv[np];
				/* use satellite L2 offset if L5 offset does not exists */
				if (rtk->opt->nf>2&&norm(nav->pcvs[i].off[2],3)<=0.0){
					matcpy(nav->pcvs[i].off[2],nav->pcvs[i].off[1], 3,1);
					matcpy(nav->pcvs[i].var[2],nav->pcvs[i].var[1],19,1);
				}
				break;
			}
		}
	}
	/* update receiver antenna to rtk->opt */
	else {
		/* loop receiver */
		for (int i=0; i<2; i++) for (int np=0; np<atx->pcv.size(); np++){
			/* antenna tpye check */
			if (rtk->opt->anttype[i].compare(0,20,atx->pcv[np].type)==0){
				rtk->opt->pcvr[i]=atx->pcv[np];
				/* use satellite L2 offset if L5 offset does not exists */
				if (rtk->opt->nf>2&&norm(rtk->opt->pcvr[i].off[2],3)<=0.0){
					matcpy(rtk->opt->pcvr[i].off[2],rtk->opt->pcvr[i].off[1],3,1);
					matcpy(rtk->opt->pcvr[i].var[2],rtk->opt->pcvr[i].var[1],19,1);
				}
				break;
			}
		}
	}
}
/* new rtk according to opt ------------------------------------------------------- */
void rtksvr_t::inirtk(prcopt_t *Prcopt,filopt_t *Filopt){
	if (rtk) delete rtk;
	if (Prcopt->mode==PMODE_SINGLE) rtk=new single_t;
	else if (Prcopt->mode<PMODE_DGPS) rtk=new ppp_t;
	else  rtk=new relative_t;
	rtk->opt=Prcopt;
	/* set base station position */
	for (int i=0; i<6; i++) {
		rtk->rb[i]=i<3 ? Prcopt->rb[i] : 0.0;
	}
	/* initialize navigation pointer */
	rtk->nav=nav;
	/* read erp file */
	if (Filopt->erp.size()>10){
		inerp_t erp(Filopt->erp);
		erp.readerp(&nav->erp);
		rtk->tidefunc.init_erp(rtk->opt,nav);
	}
	/* read blq file */
	if (Filopt->blq.size()>10&&Prcopt->tidecorr&2) {
		inblq_t blq(Filopt->blq);
		blq.readblq(Prcopt->name[0],nav->ocean_par[0]);
		if (Prcopt->mode>=PMODE_DGPS) blq.readblq(Prcopt->name[1],nav->ocean_par[1]);
		/* tidal displacement functions */
		rtk->tidefunc.init_otl(rtk->opt,nav);
	}
	/* read sat\rec antenna information file */
	if (Filopt->satantp.size()>10){
		inatx_t sat(Filopt->satantp);
		sat.readatx();
		setpcv(&sat,0);
	}
	if (Filopt->rcvantp.size()>10&&Prcopt->posopt[1]){
		inatx_t rcv(Filopt->rcvantp);
		rcv.readatx();
		setpcv(&rcv,1);
	}
	/* open test file */
	if (Filopt->test.length()>5) rtk->state_file.open(Filopt->test,ios::out);
}
/* initialize stream environment -------------------------------------------------- */
void rtksvr_t::strinitcom(){
#ifdef WIN32
	WSADATA data;
	WSAStartup(MAKEWORD(2,0),&data);
#endif
}
/* initialize stream type --------------------------------------------------------- */
void rtksvr_t::inistream(){

	for (int i=0; i<MAXSTRRTK; i++){
		if (stream[i]) delete stream[i];
		switch (strtype[i]){
			case STR_SERIAL:   stream[i]=new serial_t; break;
			case STR_FILE:     stream[i]=new file_t;   break;
			case STR_TCPSVR:   stream[i]=new tcpsvr_t; break;
			case STR_TCPCLI:   stream[i]=new tcpcli_t; break;
			case STR_NTRIPSVR:
			case STR_NTRIPCLI: stream[i]=new ntrip_t;  break;
			case STR_FTP:
			case STR_HTTP:     stream[i]=new ftp_t;    break;
			case STR_NTRIPC_S:
			case STR_NTRIPC_C: stream[i]=new ntripc_t; break;
			case STR_UDPSVR:
			case STR_UDPCLI:   stream[i]=new udp_t;    break;
			case STR_MEMBUF:   stream[i]=new membuf_t; break;
			default: stream[i]=new stream_t;
		}
		stream[i]->Stype=strtype[i];
	}
}
/* initialize decode format ------------------------------------------------------- */
void rtksvr_t::inidecode(){

	for (int i=0; i<3; i++){
		if (data[i]) delete data[i];
		switch (format[i]){
			case STRFMT_RTCM2: data[i]=new rtcm_2; break;
			case STRFMT_RTCM3: data[i]=new rtcm_3; break;
			case STRFMT_OEM3:  data[i]=new oem3;   break;
			case STRFMT_OEM4:  data[i]=new oem4;   break;
			case STRFMT_UBX:   data[i]=new ublox;  break;
			case STRFMT_SS2:   data[i]=new ss2;    break;
			case STRFMT_CRES:  data[i]=new cres;   break;
			case STRFMT_STQ:   data[i]=new skyq;   break;
			case STRFMT_GW10:  data[i]=new gw10;   break;
			case STRFMT_JAVAD: data[i]=new javad;  break;
			case STRFMT_NVS:   data[i]=new nvs;    break;
			case STRFMT_BINEX: data[i]=new binex;  break;
			case STRFMT_RT17:  data[i]=new rt17;   break;
			case STRFMT_SEPT:  data[i]=new sbf;    break;
			case STRFMT_LEXR:  data[i]=new decode_data;       break;
			case STRFMT_CMR:   data[i]=new cmr; data[i]->Svr=this; break;
			default: data[i]=new decode_data;
		}
		data[i]->format=format[i];
	}
}
/* sync input streams (if type=STR_FILE) ------------------------------------------ */
void rtksvr_t::strsync(){
	if (stream[0]->Stype==STR_FILE&&stream[1]->Stype==STR_FILE)
		stream[0]->strsync(stream+1);
	if (stream[0]->Stype==STR_FILE&&stream[2]->Stype==STR_FILE)
		stream[0]->strsync(stream+2);
}
/* write solution header to output stream ----------------------------------------- */
void rtksvr_t::writesolhead(){
	unsigned char buff1[1024]={0};
	unsigned char buff2[1024]={0};
	int n;

	n=solopt[0].outsolheads(buff1);
	stream[3]->StreamWrite(buff1,n);
	n=solopt[1].outsolheads(buff2);
	stream[4]->StreamWrite(buff2,n);
}
/* update navigation data --------------------------------------------------------- */
void rtksvr_t::updatenav(){
	int i,j;
	for (i=0; i<MAXSAT; i++) for (j=0; j<NFREQ; j++) {
		nav->lam[i][j]=satwavelen(i+1,j,nav);
	}
}
/* update glonass frequency channel number in raw data struct --------------------- */
void rtksvr_t::updatefcn(){
	int i,j,sat,frq;

	for (i=0; i<MAXPRNGLO; i++) {
		sat=satno(SYS_GLO,i+1);

		for (j=0,frq=-999; j<3; j++) {
			if (data[j]->nav.geph[i].sat!=sat) continue;
			frq=data[j]->nav.geph[i].frq;
		}
		if (frq<-7||frq>6) continue;

		for (j=0; j<3; j++) {
			if (data[j]->nav.geph[i].sat==sat) continue;
			data[j]->nav.geph[i].sat=sat;
			data[j]->nav.geph[i].frq=frq;
		}
	}
}
/* write solution to each out-stream (stream[3:4])--------------------------------- */
void rtksvr_t::writesolstr(int index){
	unsigned char buff[MAXSOLMSG+1]={ 0 };
	char *p=(char *)buff;

	/* write solution to buff */
	/* [1] write solution time */
	string soltime=rtk->sol.back().fortime(solopt+index);
	p+=sprintf(p, "%s%s",soltime.c_str(),solopt[index].sep.c_str());

	/* [2] position solution */
	string strpv;
	if (
		rtk->sol.back().stat==SOLQ_NONE||
		solopt[index].posf==SOLF_ENU&&
			(solopt[index].origin==0&&norm(rtk->rb,3)<=0.0||
			solopt[index].origin!=0&&norm(rtk->opt->ru,3)<=0.0)
		)
		return;
	else {
		strpv=rtk->sol.back().forposvel(solopt+index,rtk);
	}
	p+=sprintf(p, "%s\n",strpv.c_str());

	/* write solution to stream[index+3] */
	stream[index+3]->StreamWrite(buff,p-(char *)buff);
}

/* initialize observation pointer obsr/obsb (*rtk) ------------------------ */
int rtksvr_t::iniobs(){
	rtk->obsr=&obs[0];
	rtk->obsb=&obs[1];

	/* test availability of rover observation */
	if (fobs[0]<=0) return 0;
	/* test availability of base observation and time synchronization */
	if (rtk->opt->mode>=PMODE_DGPS&&
		(fobs[1]<=0||fabs(rtk->obsr->data[0].time.timediff(rtk->obsb->data[0].time))>=1E-3))
		return 0;
	if (obs[0].data[0].time.timediff(rtk->sol.back().time)<sampling-1E-3){
		errmsg="duplicated observation!\n";
		return 0;
	}

	return fobs[0];
}

/* process function --------------------------------------------------------------- */
/* lock/unlock rtk server --------------------------------------------------------- */
void rtksvr_t::rtksvrlock(){
	tolock(&lock);
}
void rtksvr_t::rtksvrunlock(){
	tounlock(&lock);
}
/* write solution to output stream ------------------------------------------------ */
void rtksvr_t::writesol(){
	writesolstr(0);
	writesolstr(1);
}
/* input message from stream ------------------------------------------------------ */
/* update rtk server struct ------------------------------------------------------- */
void rtksvr_t::updatesvr(int ret,int index){
	eph_t *eph1,*eph2,*eph3;
	geph_t *geph1,*geph2,*geph3;
	gtime_t tof;
	double pos[3],del[3]={ 0 },dr[3];
	int i,n=0,prn,sbssat=rtk->opt->sbassatsel,sys,iode;

	/* observation data */
	if (ret==1) { 
		/* initialize obs[index] */
		obs[index].reset();
		if (obs[index].n<MAXOBS) {
			for (i=0; i<data[index]->obs.n; i++) {
				data[index]->obs.data[i].sys=
					satsys(data[index]->obs.data[i].sat,&data[index]->obs.data[i].prn);
				if (rtk->opt->exsats[data[index]->obs.data[i].sat-1]==1||
				!(data[index]->obs.data[i].sys&rtk->opt->navsys))
					continue;
				obs[index].data.push_back(data[index]->obs.data[i]);
				obs[index].data.back().rcv=index+1; //rev flag
			}
			obs[index].n=obs[index].data.size();
			/* arrange observation data */
			sortobs(obs[index]);
		}
		obs[index].rcv=index;
		nmsg[index][0]++;
	}
	/* ephemeris */
	else if (ret==2) { 
		if (satsys(data[index]->ephsat,&prn)!=SYS_GLO) {
			if (!navsel||navsel==index+1) {
				eph1=&data[index]->nav.eph[data[index]->ephsat-1];
				eph2=&nav->eph[data[index]->ephsat-1];
				eph3=&nav->eph[data[index]->ephsat-1+MAXSAT];
				if (eph2->ttr.time==0||
					(eph1->iode!=eph3->iode&&eph1->iode!=eph2->iode)||
					(eph1->toe.timediff(eph3->toe)!=0.0&&
						eph1->toe.timediff(eph2->toe)!=0.0)) {
					*eph3=*eph2;
					*eph2=*eph1;
					updatenav();
				}
			}
			nmsg[index][1]++;
		}
		else {
			if (!navsel||navsel==index+1) {
				geph1=&data[index]->nav.geph[prn-1];
				geph2=&nav->geph[prn-1];
				geph3=&nav->geph[prn-1+MAXPRNGLO];
				if (geph2->tof.time==0||
					(geph1->iode!=geph3->iode&&geph1->iode!=geph2->iode)) {
					*geph3=*geph2;
					*geph2=*geph1;
					updatenav();
					updatefcn();
				}
			}
			nmsg[index][6]++;
		}
	}
	/* sbas message */
	else if (ret==3) { 
		if (sbssat==data[index]->sbsmsg.prn||sbssat==0) {
			if (nsbs<MAXSBSMSG) {
				sbsmsg[nsbs++]=data[index]->sbsmsg;
			}
			else {
				for (i=0; i<MAXSBSMSG-1; i++) sbsmsg[i]=sbsmsg[i+1];
				sbsmsg[i]=data[index]->sbsmsg;
			}
			data[index]->sbsmsg.sbsupdatecorr(nav);
		}
		nmsg[index][3]++;
	}
	/* ion/utc parameters */
	else if (ret==9) { 
		if (navsel==0||navsel==index+1) {
			for (i=0; i<8; i++) nav->ion_gps[i]=data[index]->nav.ion_gps[i];
			for (i=0; i<4; i++) nav->utc_gps[i]=data[index]->nav.utc_gps[i];
			for (i=0; i<4; i++) nav->ion_gal[i]=data[index]->nav.ion_gal[i];
			for (i=0; i<4; i++) nav->utc_gal[i]=data[index]->nav.utc_gal[i];
			for (i=0; i<8; i++) nav->ion_qzs[i]=data[index]->nav.ion_qzs[i];
			for (i=0; i<4; i++) nav->utc_qzs[i]=data[index]->nav.utc_qzs[i];
			nav->leaps=data[index]->nav.leaps;
		}
		nmsg[index][2]++;
	}
	/* antenna postion parameters */
	else if (ret==5) { 
		if (index==1 && (rtk->opt->refpos==POSOPT_RTCM||rtk->opt->refpos==POSOPT_RAW)) {
			for (i=0; i<3; i++) {
				rtk->rb[i]=data[1]->sta.pos[i];
			}
			/* antenna delta */
			ecef2pos(rtk->rb,WGS84,pos);
			if (data[1]->sta.deltype) { /* xyz */
				del[2]=data[1]->sta.hgt;
				enu2ecef(pos,del,dr);
				for (i=0; i<3; i++) {
					rtk->rb[i]+=data[1]->sta.del[i]+dr[i];
				}
			}
			else { /* enu */
				enu2ecef(pos,data[1]->sta.del,dr);
				for (i=0; i<3; i++) {
					rtk->rb[i]+=dr[i];
				}
			}
		}
		nmsg[index][4]++;
	}
	/* dgps correction */
	else if (ret==7) { 
		nmsg[index][5]++;
	}
	/* ssr message */
	else if (ret==10) {
		for (i=0; i<MAXSAT; i++) {
			if (!data[index]->ssr[i].update) continue;

			/* check consistency between iods of orbit and clock */
			if (data[index]->ssr[i].iod[0]!=
				data[index]->ssr[i].iod[1]) continue;

			data[index]->ssr[i].update=0;

			iode=data[index]->ssr[i].iode;
			sys=satsys(i+1,&prn);

			/* check corresponding ephemeris exists */
			if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS) {
				if (nav->eph[i].iode!=iode&&
					nav->eph[i+MAXSAT].iode!=iode) {
					continue;
				}
			}
			else if (sys==SYS_GLO) {
				if (nav->geph[prn-1].iode!=iode&&
					nav->geph[prn-1+MAXPRNGLO].iode!=iode) {
					continue;
				}
			}
			nav->ssr[i]=data[index]->ssr[i];
		}
		nmsg[index][7]++;
	}
	/* lex message */
	else if (ret==31) { 
		data[index]->lexmsg.lexupdatecorr(nav,tof);
		nmsg[index][8]++;
	}
	/* error */
	else if (ret==-1) { 
		nmsg[index][9]++;
	}
}
/* decode receiver raw/rtcm data -------------------------------------------------- */
int rtksvr_t::decoderaw(int index){
	int i,ret;

	/* initialize */
	rtksvrlock();

	for (i=0; i<nb[index]; i++) {

		/* input rtcm/receiver raw data from stream */
		ret=data[index]->decode((unsigned char)buff[index][i]);

		/* update rtk server */
		if (ret>0) updatesvr(ret,index);

		/* observation data received */
		if (ret==1) {
			if (obs[index].n<=MAXOBS) fobs[index]=obs[index].n; 
			else { prcout++; fobs[index]=0; }
		}
	}
	nb[index]=0;

	rtksvrunlock();

	return fobs[index];
}

/* initialize rtksvr ------------------------------------------------------ */
int rtksvr_t::rtksvrini(option_t *option){
	gtime_t time;
	int i,j,rw;

	if (state) return 0;

	strinitcom();
	sampling=option->prcopt.sampling;
	cyctime=option->rtkopt.svrcycle>1 ? option->rtkopt.svrcycle : 1;
	nmeacycle=option->rtkopt.nmeacycle>1000 ? option->rtkopt.nmeacycle : 1000;
	nmeareq=option->rtkopt.nmeareq;
	for (i=0; i<3; i++) nmeapos[i]=option->rtkopt.nmeapos[i];
	buffsize=option->rtkopt.buffsize>4096 ? option->rtkopt.buffsize : 4096;
	for (i=0; i<3; i++) format[i]=option->rtkopt.strfmt[i];
	for (i=0; i<7; i++) strtype[i]=option->rtkopt.strtype[i];
	navsel=option->rtkopt.navmsgsel;
	nsbs=0;
	nsol=0;
	prcout=0;
	
	/* initialize rtk */
	inirtk(&option->prcopt,&option->filopt);
	rtk->rtkinit();

	if (option->prcopt.initrst){
		nave=0;
		rb_ave[0]=rb_ave[1]=rb_ave[2]=0.0;
	}

	/* initialize decode and stream format */
	inidecode();
	for (i=0; i<3; i++) { /* input/log streams */
		nb[i]=npb[i]=0;
		if (!(buff[i]=new unsigned char [buffsize])||
			!(pbuf[i]=new unsigned char [buffsize])) {
			return 0;
		}
		for (j=0; j<10; j++) nmsg[i][j]=0;

		/* set receiver and rtcm option */
		data[i]->opt=option->rtkopt.rropts[i];

		/* connect dgps corrections */
		data[i]->dgps=nav->dgps;
	}
	for (i=0; i<2; i++) { /* output peek buffer */
		if (!(sbuf[i]=new unsigned char [buffsize])) {
			return 0;
		}
	}
	/* set solution options */
	solopt[0]=option->solopt[0];
	solopt[1]=option->solopt[1];

	/* update navigation data */
	updatenav();

	/* set monitor stream */
	moni=option->rtkopt.monitor;

	/* initialize streams */
	inistream();
	/* open input streams */
	for (i=0; i<MAXSTRRTK; i++) {
		rw=i<3 ? STR_MODE_R : STR_MODE_W;
		if (option->rtkopt.strtype[i]!=STR_FILE) rw|=STR_MODE_W;
		if (!stream[i]->StreamOpen(option->rtkopt.strpath[i].c_str(),strtype[i],rw)) {
			for (i--; i>=0; i--) stream[i]->StreamClose();
			return 0;
		}
		/* set initial time for rtcm and raw */
		if (i<3) {
			time.timeget()->utc2gpst();
			data[i]->time=
				option->rtkopt.strtype[i]==STR_FILE ? stream[i]->strgettime() : time;
		}
	}
	/* sync input streams (if type=STR_FILE) */
	strsync();

	/* write start commands to input streams */
	for (i=0; i<3; i++) {
		if (option->rtkopt.cmds[i]!="\0") stream[i]->SendCmd(option->rtkopt.cmds[i].c_str());
	}
	/* write solution header to solution streams */
	writesolhead();

	//test
	cout <<"rtksvr initialization is ok!\n";

	return 1;
}

/* thread-start function ---------------------------------------------------------- */
#ifdef WIN32
static DWORD WINAPI rtksvrthread(void *arg)
#else
static void * rtksvrthread(void *arg)
#endif
{
	/* initailize arg to rtksvr_t */
	rtksvr_t *svr=(rtksvr_t *) arg;
	/* compute time and thread run time */
	double cpttime,runtime;
	/* thread-start time and last position-fall time */
	unsigned int startick,lastfall;
	/* position cycle */
	int cycle;
	/* solution time (utc) */
	gtime_t soltime;

	/* initialize svr */
	svr->state=1; svr->tick=tickget();
	lastfall=svr->tick-1000;

	for (cycle=0; svr->state; cycle++){
		startick=tickget();

		for (int i=0; i<3; i++){
			/* pointer to buff head and tail */
			unsigned char *bufhead=svr->buff[i]+svr->nb[i],
				*buftail=svr->buff[i]+svr->buffsize;
			int rbufn; //recevied buff number

			/* read receiver raw/rtcm data from input stream */
			if ((rbufn=svr->stream[i]->StreamRead(bufhead,buftail-bufhead))<=0) continue;

			/* write receiver raw/rtcm data to log stream */
			svr->stream[i+5]->StreamWrite(bufhead,rbufn);
			svr->nb[i]+=rbufn;

			/* save peek buffer */
			svr->rtksvrlock();
			rbufn=rbufn<svr->buffsize-svr->npb[i] ? rbufn : svr->buffsize-svr->npb[i];
			memcpy(svr->pbuf[i]+svr->npb[i],bufhead,rbufn);
			svr->npb[i]+=rbufn;
			svr->rtksvrunlock();
		}
		for (int i=0; i<3; i++){
			/* decode receiver raw/rtcm data */
			svr->decoderaw(i);
		}
		if (svr->iniobs()){

			/* SPP for base station */
			if (svr->rtk->opt->mode>=PMODE_DGPS&&svr->fobs[1]>0) {
				if ((svr->rtk->opt->maxaveep<=0||svr->nave<svr->rtk->opt->maxaveep)&&
					svr->rtk->basepos()){ //return solution to b_sol
					svr->nave++;
					for (int i=0; i<3; i++)
						svr->rb_ave[i]+=(svr->rtk->b_sol.back().xdyc[i]-svr->rb_ave[i])/svr->nave;
				}
				for (int i=0; i<3; i++) {
					if (svr->rtk->opt->refpos==POSOPT_SINGLE) svr->rtk->rb[i]=svr->rb_ave[i];
					else if (svr->rtk->opt->mode==PMODE_MOVEB) {
						svr->rtk->rb[i]=svr->rtk->b_sol.back().xdyc[i];
					}
				}
			}

			/* rtk positioning for rover */
			svr->rtksvrlock();
			svr->rtk->rtkpos();
			svr->rtksvrunlock();

			/* output solution if sol.stat */
			if (svr->rtk->sol.back().stat!=SOLQ_NONE){
				/* adjust difference between computer time and UTC time */
				cpttime=(int)(tickget()-startick)/1000.0+DTTOL;
				soltime=svr->rtk->sol.back().time;
				soltime.timeadd(cpttime)->gpst2utc()->timeset();

				/* write solution */
				svr->writesol();
			}
			/* send null solution if no solution (1hz) */
			else if(svr->rtk->sol.back().stat==SOLQ_NONE&&(int)(startick-lastfall)>=1000) {
				svr->writesol();
				lastfall=startick;
			}
		}
		
		if ((runtime=(int)(tickget()-startick))>0) svr->cputime=runtime;

		/* sleep until next cycle */
		sleepms(svr->cyctime-runtime);
	}
	/* close stream */
	for (int i=0; i<MAXSTRRTK; i++) svr->stream[i]->StreamClose();
#ifdef WIN32
	return 0;
#endif
}

/* start rtksvr ----------------------------------------------------------- */
int rtksvr_t::rtksvrstart(){
	#ifdef WIN32
	if (!(thread=CreateThread(NULL,0,rtksvrthread,this,0,NULL)))
	#else
	if (pthread_create(&thread,NULL,rtksvrthread,this))
	#endif
	{
		for (int i=0; i<MAXSTRRTK; i++) stream[i]->StreamClose();
		errmsg="thread create error\n";
		return 0;
	}
	return 1;
}

/* stop rtksvr ------------------------------------------------------------ */
void rtksvr_t::rtksvrstop(char **cmds){
	/* write stop commands to input streams */
	rtksvrlock();
	for (int i=0; i<3; i++) {
		if (cmds[i]) stream[i]->SendCmd(cmds[i]);
	}
	rtksvrunlock();

	/* stop rtk server */
	state=0;

	/* free rtk server thread */
#ifdef WIN32
	WaitForSingleObject(thread,10000);
	CloseHandle(thread);
#else
	pthread_join(thread,NULL);
#endif
}