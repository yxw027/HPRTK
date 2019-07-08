#include "GNSS/PosModel/position.h"
#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
#define SQR(x)      ((x)*(x))
#define SQRT(x)     ((x)<=0.0?0.0:sqrt(x))

#define INIT_ION	0.5					/* initial Enm of ion POLY model */
#define INIT_GRA	1E-5				/* initial tro gradient */
#define VAR_GRA     SQR(0.001)			/* initial variance of gradient (m^2) */
#define INIT_HWBIAS	0					/* initial glo frequency-dependent amb bias */
#define VAR_HWBIAS  SQR(0.1)			/* initial variance of h/w bias ((m/MHz)^2) */
#define RAT_HWBIAS	1E-7				/* growth rate of std h/w bias (m/MHz/sqrt(s)) */

#define VAR_POS     SQR(30.0)			/* initial variance of receiver pos (m^2) */
#define VAR_VEL     SQR(50.0)			/* initial variance of receiver vel ((m/s)^2) */

#define TTOL_MOVEB  (1.0+2*DTTOL)

const string SYS_LIST="GREC";				/* system flag GPS, GLONASS, Galileo, BeiDou */

/* relative position class ---------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
relative_t::relative_t() {
	nsol = 0;
	/* station geographic parameters (ini. in  updatepar()) */
	odt = 0.0;
	for (int i=0; i<3; i++)
		Rxyz[i] = Bxyz[i] = Rblh[i] = Bblh[i] = Rtide[i] = Btide[i] = 0.0;

	baseline = Rbl = 0.0;
	/* centre satellite parameters */
	comnum = cennum = 0;
	for (int i=0; i<4; i++) {
		for (int j = 0; j<NFREQ; j++) L_nsat[i][j] = 0;
		irfsat[i] = -1;
		rfsat[0][i] = rfsat[1][i] = 0;
		disRB1[i] = troRB1[i] = ionRB1[i] = gloRB1[i][0] = gloRB1[i][1] = 0.0;
		ambRB1[i][0] = ambRB1[i][1] = ambRB1[i][2] = 0.0;
		ant1[i][0] = ant1[i][1] = ant1[i][2] = 0.0;
	}
	disRB2 = troRB2 = ionRB2 = gloRB2 = ambRB2 = 0.0;
	ant2[0] = ant2[1] = ant2[2] = 0.0;

	rp = bp = NULL;
	for (int sys = 0; sys<4; sys++) {
		comsys[sys]='\0';
		ion_reset[sys]=0;
		for (int i=0; i<NFREQ*2; i++)
			nobs[sys][i] = varRB1[sys][i] = 0;
	}

	for (int i=0; i<NFREQ; i++) nreset[i]=reset_flag[i]=0;

	numX = 0;
	iT = iG = iI = iA = 0;
	nA = nI = 0;
	ifact1=ifact2=0.0;
	iniamb=60;
	fix_rate=0;
	fix_num=n_Damb = 0;
}
relative_t::~relative_t() {
	lam1 = lam2 = NULL;
	rp = bp = NULL;

	comsat.clear(); rovsat.clear(); bassat.clear();
	for (int sys = 0; sys<4; sys++) Airfsat[sys].clear();

	Asatnum.clear();

	Rxvec.clear();

	ambnum.clear();
	fix_flag.clear();
	fix_amb.clear(); sum_var.clear();
	fix_Xpar.clear(); fix_Rx.clear();
}
/* Implementation functions ------------------------------------------------------- */
/* test satellite system -------------------------------------------------- */
int relative_t::test_system(int Sysyem,int System_Num) {
	switch (Sysyem) {
		case SYS_GPS: return System_Num==0;
		case SYS_QZS: return System_Num==0;
		case SYS_SBS: return System_Num==0;
		case SYS_GLO: return System_Num==1;
		case SYS_GAL: return System_Num==2;
		case SYS_CMP: return System_Num==3;
	}
	return 0;
}
/* select common satellites of rover and base ------------------------------------- */
/* and get highest elevation satellite of rover */
int relative_t::selcomsat() {
	vector<int> nphase_sat;		/* phase number of common satellites */
	comsat.clear(); rovsat.clear(); bassat.clear(); nphase_sat.clear();
	for (int sys = 0; sys<4; sys++) for (int freq = 0; freq<NFREQ; freq++) 
		L_nsat[sys][freq] = 0;

	/* select common satellites */
	for (int i=0,j = 0; i<obsr->n&&j<obsb->n; i++,j++) {
		if (obsr->data[i].sat<obsb->data[j].sat) j--;
		else if (obsr->data[i].sat > obsb->data[j].sat) i--;
		else {
			int sys=syscd2num(obsr->data[i].sys);
			if (obsr->data[i].used&&obsb->data[j].used&&ncode[0][sys]>1&&ncode[1][sys]>1) {
				comsat.push_back(obsr->data[i].sat);
				rovsat.push_back(i);
				bassat.push_back(j);
			}
		}
	}
	comnum = comsat.size();

	if (comnum <= 0) return 0;

	init_censat(); //initialize centre satellite parameters
	if (opt->mode > PMODE_DGPS) for (int sys = 0; sys<4; sys++) { // loop 4 systems
		/* get phase observation number of common satellite */
		nphase_sat.assign(comnum,0);
		for (int i=0; i<comnum; i++) { //loop every satellite
			int L_ava[NFREQ] = { 0 };
			rp = &obsr->data[rovsat[i]]; bp = &obsb->data[bassat[i]];
			if (!test_system(rp->sys,sys)) continue; //test satellite system
			for (int freq = 0; freq<NF; freq++) { //loop frequency
				if (rp->L[freq]!=0.0&&rp->P[freq]!=0.0&&bp->L[freq]!=0.0&&bp->P[freq]!=0.0) {
					nphase_sat[i]++;
					L_ava[freq] = 1; // available phase obs. (L1,L2,...)
				}
			}
			if (opt->ionoopt==IONOOPT_IFLC && L_ava[0] && L_ava[1]) L_nsat[sys][0]++;
			if (opt->ionoopt!=IONOOPT_IFLC) {
				for (int freq = 0; freq<NF; freq++) if (L_ava[freq]) L_nsat[sys][freq]++;
			}
			if (irfsat[sys]<0 || nphase_sat[i]>nphase_sat[irfsat[sys]]) irfsat[sys] = i;
		}
		if (irfsat[sys]<0) continue; //continue if no satellite of this system
		/* get satellites of rover with highest elevation  */
		for (int i=0; i<comnum; i++) {
			if (nphase_sat[i]<nphase_sat[irfsat[sys]]) continue;
			if (obsr->data[rovsat[i]].azel[1] > obsr->data[rovsat[irfsat[sys]]].azel[1]) 
				irfsat[sys] = i;
		}
		rfsat[1][sys] = comsat[irfsat[sys]];
		if (ncode[sys]) cennum++;
	}
	else for (int sys=0; sys<4; sys++) {
		for (int i=0; i<comnum; i++) {
			if (!test_system(obsr->data[rovsat[i]].sys,sys)) continue; //test satellite system
			if (irfsat[sys]<0 || obsr->data[rovsat[i]].azel[1]>obsr->data[rovsat[irfsat[sys]]].azel[1])
				irfsat[sys] = i;
		}
		if (irfsat[sys]>=0) { rfsat[1][sys] = comsat[irfsat[sys]]; cennum++; }
	}

	return comnum;
}
/* initialize centre satellites parameters ---------------------------------------- */
void relative_t::init_censat() {
	cennum = 0;
	for (int i=0; i<4; i++) {
		irfsat[i] = -1;
		rfsat[1][i] = 0;
		disRB1[i] = troRB1[i] = ionRB1[i] = gloRB1[i][0] = gloRB1[i][1] = 0.0;
		ambRB1[i][0] = ambRB1[i][1] = ambRB1[i][2] = 0.0;
		ant1[i][0] = ant1[i][1] = ant1[i][2] = 0.0;
	}
}
/* initialize vector and matrix according common satellite ------------------------ */
void relative_t::init_arrmat() {
	Lobs.clear(); Acoe.clear();
	Rvec.clear(); Rvar.clear(); Rx.clear();
	fix_flag.clear();
	fix_amb.clear(); sum_var.clear();
	fix_Xpar.clear(); fix_Rx.clear();
	ambnum.assign(numF*comnum,-1);

	for (int sys=0; sys<4; sys++) comsys[sys]='\0';

	/* number of kinds of parameters */
	/* ND NT NG nI nA */
	if (opt->ionoopt==IONOOPT_CONST) nI=comnum; 
	nA = 0;
	numX = ND+NT+NG+nI+nA;

	/* set start index of kinds of parameters */
	iT=ND; iG=iT+NT; iI=iG+NG; iA=iI+nI;

	/* initialize parameter and its variance vector (no amb) */
	Xpar.assign(numX,0.0); Rxvec.assign(numX,0.0);

	/* DD ambiguity fixed rate */
	if (opt->modear==ARMODE_FIXHOLD||opt->modear==ARMODE_LCWN) fix_rate=fix_num=0;
}
/* initialize covariance matrix of all parameters */
void relative_t::init_RxALL(unsigned int sat, int freq) {
	for (int i=0; i<N_ALL; i++) 
		Rx_ALL[i+(NXI+(sat-1)*numF+freq)*N_ALL] = Rx_ALL[NXI+(sat-1)*numF+freq+i*N_ALL] = 0.0;
}
/* update parameters functions ---------------------------------------------------- */
/* update dynamic parameters ------------------------------------------------------ */
void relative_t::updatexyz() {
	/* fix mode */
	if (opt->mode==PMODE_FIXED) {
		for (int i=0; i<3; i++) {
			Xpar[i] = opt->ru[i];
			Rxvec[i] = 1E-8;
			return;
		}
	}

	/* initialize xyz using SPP result for first epoch */
	if (norm(sol[MAXSOLBUF-2].xdyc.begin(),3) <= 0.0) {
		for (int i=0; i<ND; i++) {
			Xpar[i] = sol.back().xdyc[i];
			Rxvec[i] =
				i<3 ? VAR_POS : VAR_VEL;
		}
		return;
	}

	/* static mode */
	if (opt->mode==PMODE_STATIC) {
		for (int i=0; i<ND; i++) {
			Xpar[i] = sol[MAXSOLBUF-2].xdyc[i];
			/*if (opt->ionoopt==IONOOPT_CONST) 
				Rxvec[i] = nsol<iniamb? VAR_POS : sol[MAXSOLBUF-2].vdyc[i+i*6]+SQR(0.01)*tt;*/
			Rxvec[i] = nsol<iniamb? VAR_POS : sol[MAXSOLBUF-2].vdyc[i+i*6];
		}
		return;
	}

	/* kinematic mode without velocity (only xyz) */
	if (!opt->dynamics) {
		for (int i=0; i<3; i++) {
			Xpar[i] = sol.back().xdyc[i];
			Rxvec[i] = VAR_POS;
		}
		return;
	}

	/* kinematic mode if variance too large */
	double var = 0.0;
	for (int i=0; i<3; i++) var += sol[MAXSOLBUF-2].vdyc[i + i*6];
	if (var/3.0 > VAR_POS) {
		/* reset xdyc using SPP result */
		for (int i=0; i<ND; i++) {
			Xpar[i] = sol.back().xdyc[i];
			Rxvec[i] =
				i<3 ? VAR_POS : VAR_VEL;
		}
		return;
	}

	/* normal kinematic mode */
	vector<double> F(6*6,0.0),Q,FQ(6*6,0.0),Rpos(6*6,0.0);
	double pos[3],Qv[9] = { 0.0 },Qve[9] = { 0.0 };
	/* transition matrix F and Q */
	eyemat(F.begin(),6);
	for (int i=0; i<3; i++)
		F[i + (i + 3)*6] = tt;
	Q.assign(sol[MAXSOLBUF-2].vdyc.begin(),sol[MAXSOLBUF-2].vdyc.end());
	Qv[0] = Qv[4] = SQR(opt->stdrate[3]*tt); Qv[8] = SQR(opt->stdrate[2]*tt);
	ecef2pos(sol.back().xdyc.begin(),WGS84,pos);
	covecef(pos,Qv,Qve);
	for (int i=0; i<3; i++) for (int j = 0; j<3; j++) Q[i + 3 + (j + 3)*ND] += Qve[i + j*3];
	/* compute Xpar and Rxvec */
	matmul_vec("NN",6,1,6,1.0,F,sol[MAXSOLBUF-2].xdyc,0.0,Xpar); //Xpar (0:5)
	matmul_vec("NN",6,6,6,1.0,F,Q,0.0,FQ); //FP
	matmul_vec("NT",6,6,6,1.0,FQ,F,0.0,Rpos); //FPF'=Rxvec (0:5 x 0:5)
	for (int i=0; i<6; i++) {
		Rxvec[i] = Rpos[i + i*6];
	}
}
/* update troposphere parameters -------------------------------------------------- */
void relative_t::updatetro() {
	int nTp = NT/2;
	// loop rover and base
	for (int i=0; i<2; i++) {
		/* initialize tro parameters for the first epoch */
		//if (sol[MAXSOLBUF-2].xtro[i*nTp]==0.0){
		//	const double zazel[] = { 0.0,PI/2.0 };
		//	obsd_t obs;
		//	double *pos = i==0 ? Rblh : Bblh;
		//	trofunc->saascorr(&obs,pos,zazel,0.0);
		//	Xpar[iT+i*nTp] = obs.dtro;
		//	Rxvec[iT+i*nTp]=SQR(opt->std[2]);
		//	/* estimate tro gradient */
		//	if (nTp==3) for (int j=1; j<3; j++){
		//		Xpar[iT+i*nTp+j] =INIT_GRA;
		//		Rxvec[iT+i*nTp+j]=VAR_GRA;
		//	}
		//}
		///* update tro parameters using last solution */
		//else {
		//	Xpar[iT+i*nTp] =sol[MAXSOLBUF-2].xtro[i*nTp];
		//	Rxvec[iT+i*nTp]=sol[MAXSOLBUF-2].vtro[i*nTp+i*nTp*NT]+SQR(opt->stdrate[1]*tt);
		//	/* estimate tro gradient */
		//	if (nTp==3) for (int j=1; j<3; j++){
		//		Xpar[iT+i*nTp+j] =sol[MAXSOLBUF-2].xtro[i*nTp+j];
		//		Rxvec[iT+i*nTp+j]=
		//			sol[MAXSOLBUF-2].vtro[i*nTp+j+(i*nTp+j)*NT]+SQR(opt->stdrate[1]*tt*0.1);
		//	}
		//}
		const double zazel[] = { 0.0,PI/2.0 };
		obsd_t obs;
		double *pos = i==0 ? Rblh : Bblh;
		trofunc->saascorr(&obs,pos,zazel,0.0);
		Xpar[iT + i*nTp] = obs.dtro;
		Rxvec[iT + i*nTp] = SQR(opt->std[2]);
		/* estimate tro gradient */
		if (nTp==3) for (int j = 1; j<3; j++) {
			Xpar[iT + i*nTp + j] = INIT_GRA;
			Rxvec[iT + i*nTp + j] = VAR_GRA;
		}
	}
}
/* update glonass differenced IFB rate between 2 receivers ------------------------ */
void relative_t::updateglo() {
	for (int i=0; i<NG; i++) {
		/* initialize glo parameters */
		if (sol[MAXSOLBUF-2].xglo[i]==0.0||nsol<=iniamb) {
			Xpar[iG + i] = INIT_HWBIAS;
			Rxvec[(iG + i)] = VAR_HWBIAS;
		}
		/* update glo parameters */
		else {
			Xpar[iG + i] = sol[MAXSOLBUF-2].xglo[i];
			Rxvec[iG + i] = sol[MAXSOLBUF-2].vglo[i + i*NG] + SQR(RAT_HWBIAS)*tt;
		}
	}
}
/* update ambiguity parameters ---------------------------------------------------- */
void relative_t::updateamb() {
	int ambum = 0;
	int sat;
	double *lam;
	ssat_t *sss;
	for (int f=0; f<NFREQ; f++) nreset[f]=0;
	for (int i=0; i<comnum; i++) {
		sat = comsat[i]; lam = nav->lam[sat - 1]; sss = &ssat[sat - 1];
		rp = &obsr->data[rovsat[i]]; bp = &obsb->data[bassat[i]];
		/* update vectors for current status */
		sss->update_vector(rp,bp,lam,opt);
		if (opt->mode > PMODE_DGPS) {
			/* reset flag according to unsolved time interval */
			sss->test_reset(opt);
			/* detect cycle slip */
			sss->detect_slip(rp,bp,opt,lam);
			for (int f=0; f<opt->nf; f++) if (sss->reset[f]) nreset[f]++;
		}
	}
	/* update ambiguity parameters */
	if (opt->mode > PMODE_DGPS) {
		for (int f=0; f<numF; f++) { //reset flag of each frequency
			reset_flag[f] = 2*nreset[f] > comnum;
		}
		for (int i=0; i<comnum; i++) {
			sat = comsat[i]; lam = nav->lam[sat - 1]; sss = &ssat[sat - 1];
			/* reset ambiguity if this frequency should be reset */
			for (int f=0; f<numF; f++) if (reset_flag[f]) sss->reset_amb(opt,f);
			/* update ambiguity parameters */
			sss->update_amb(lam,opt);
			/* if reset initialize covariance matrix of all parameters */
			for (int f=0; f<numF; f++) if (sss->fix[f]==1 || opt->mode!=PMODE_STATIC)
				init_RxALL(sss->sat,f);
		}
		int reset_pos=1;
		for (int f=0; f<numF; f++) reset_pos=reset_flag[f]&&reset_pos;
		if (reset_pos) nsol=0;
	}

	/* update ambiguity parameter to Xpar and Rxvec */
	if (opt->mode > PMODE_DGPS) {
		for (int sys = 0; sys<4; sys++) {
			if (irfsat[sys]<0) continue; //continue if no observation of this system
			for (int freq = 0; freq<numF; freq++) {
				if (L_nsat[sys][freq]<2) continue; //continue if satellite <2
				for (int i=0; i<comnum; i++) {
					sss = &ssat[comsat[i] - 1];
					if (!test_system(sss->sys,sys)) continue;
					/* if LC combination strategy */
					if (opt->ionoopt==IONOOPT_IFLC) {
						if (sss->sol_flag[0]) {
							Xpar.push_back(sss->lcamb);
							Rxvec.push_back(sss->lcvar);
							ambnum[i + freq*comnum] = nA;
							nA++;
						}
					}
					/* or other strategy */
					else if (sss->sol_flag[freq]) {
						Xpar.push_back(sss->amb[freq]);
						Rxvec.push_back(sss->ambvar[freq]);
						ambnum[i + freq*comnum] = nA;
						nA++;
					}
				}
			}
		}
	}
}
/* update ionosphere parameters --------------------------------------------------- */
void relative_t::updateion() {
	for (int i=0; i<comnum; i++) {
		ionfunc->correction(&obsr->data[ rovsat[i] ],nav,Rblh);
		ionfunc->correction(&obsb->data[ bassat[i] ],nav,Bblh);
		/*double dion=obsb->data[ bassat[i] ].dion - obsr->data[ rovsat[i] ].dion;*/
		if (nsol<iniamb) {
			Xpar[ iI+i ]=obsb->data[ bassat[i] ].dion-obsr->data[ rovsat[i] ].dion;
			Rxvec[ iI+i ]=3.0*(obsb->data[ bassat[i] ].ionvar+obsb->data[ bassat[i] ].ionvar);
			/*for (int j=0; j<N_ALL; j++)
				Rx_ALL[j+(NX+comsat[i]-1)*N_ALL] = Rx_ALL[(NX+comsat[i]-1)+j*N_ALL] = 0.0;*/
		}
		else {
			Xpar[ iI+i ]=ssat[comsat[i]-1].ion_delay;
			Rxvec[ iI+i ]=ssat[comsat[i]-1].ion_var + SQR(opt->stdrate[0])*tt;
		}
	}
}
/* update parameters covariance matrix -------------------------------------------- */
void relative_t::updatevar() {
	//NX
	if (nsol>=iniamb&&opt->mode==PMODE_STATIC) for (int i=0; i<NX; i++) for (int j=0; j<NX; j++)
		Rx[j+i*numX]= Rx_ALL[j+i*N_ALL];
	//nI
	if (nsol>=iniamb&&nI>0) for (int i=0; i<comnum; i++) {
		// nIi and nIj
		for (int j=0; j<comnum; j++) {
			Rx[iI+j + (iI+i)*numX] = Rx_ALL[ (NX+comsat[j]-1) + (NX+comsat[i]-1)*N_ALL ];
		}
		//nI and NX
		if (opt->mode==PMODE_STATIC) for (int j=0; j<NX; j++) {
			Rx[j + (iI+i)*numX]= Rx_ALL[ j + (NX+comsat[i]-1)*N_ALL ];
			Rx[(iI+i) + j*numX]= Rx_ALL[ (NX+comsat[i]-1) + j*N_ALL ];
		}	
	}
	//nA
	if (nsol>=iniamb&&nA>0) {
		int nambi=0,nambj=0;
		int sati,satj;

		for (int i=0; i<comnum; i++) for (int fi=0; fi<numF; fi++) if ((nambi=ambnum[i+fi*comnum])>=0&&!nreset[fi]){
			//ambi and ambj
			 for (int j=0; j<comnum; j++) for (int fj=0; fj<numF; fj++) if ((nambj=ambnum[j+fj*comnum])>=0&&!nreset[fj])
					Rx[iA+nambj + (iA+nambi)*numX]=
					Rx_ALL[NXI+(comsat[j]-1)*numF+fj+(NXI+(comsat[i]-1)*numF+fi)*N_ALL];
			//ambi and NX
			 if (opt->mode==PMODE_STATIC) for (int j=0; j<NX; j++) {
				Rx[j + (iA+nambi)*numX] = Rx_ALL[j+(NXI+(comsat[i]-1)*numF+fi)*N_ALL];
				Rx[(iA+nambi) + j*numX] = Rx_ALL[(NXI+(comsat[i]-1)*numF+fi)+j*N_ALL];
			}
			//ambi and nI
			if (nI>0) for (int j=0; j<comnum; j++) {
				Rx[(iI+j) + (iA+nambi)*numX]=
					Rx_ALL[ (NX+comsat[j]-1) + (NXI+(comsat[i]-1)*numF+fi)*N_ALL ];
				Rx[(iA+nambi) + (iI+j)*numX]=
					Rx_ALL[ (NXI+(comsat[i]-1)*numF+fi) + (NX+comsat[j]-1)*N_ALL ];
			}
		}
	}
	/* add a noise if detected reseted ambiguity */
	if (opt->modear==ARMODE_FIXHOLD) for (int f=0; f<numF; f++) if (nreset[f]) 
		for (int i=0; i<nA; i++) Rxvec[iA+i]+=SQR(0.3);
	/* update Rx using Rxvec */
	for (int i=0; i<numX; i++) 
		Rx[i + i*numX] = Rxvec[i];
}
/* update parameters from previous time to current time --------------------------- */
void relative_t::updatepar() {
	iniamb = opt->iniamb;

	/* reset vector and matrix */
	init_arrmat();

	/* update position/velocity */
	updatexyz();
	/* initialize geodetic position of rover,base and centre */
	for (int i=0; i<3; i++) { Rxyz[i] = Xpar[i]; Bxyz[i] = rb[i]; }
	ecef2pos(Rxyz,WGS84,Rblh); ecef2pos(Bxyz,WGS84,Bblh);

	/* update troposphere parameters */
	if (opt->tropopt >= TROPOPT_EST) updatetro();

	/* update glonass differenced IFB rate between 2 receivers */
	if (NG>0 && (opt->navsys&SYS_GLO)) updateglo();

	/* update ssat observation and ambiguity parameters (not for DGPS mode) */
	/*if (tt>opt->restime) nsol=0;*/
	updateamb();

	/* update ionosphere parameters */
	if (opt->ionoopt==IONOOPT_CONST) updateion();

	/* update parameters number and Rx */
	numX += nA;
	fix_flag.assign(nA,0);
	Rx.assign(numX*numX,0.0);
	updatevar();
}
/* time difference between rover and base ----------------------------------------- */
int relative_t::timediff_rb() {
	/* odt:  time difference between obs. of rover and base (s) */
	/* sol.back().age: time difference bewtween rover and base
		   = solution time difference if moving base-line
		   = odt                                                */
	odt = obsr->data[0].time.timediff(obsb->data[0].time);
	if (opt->mode==PMODE_MOVEB) {
		solp->age = solp->time.timediff(b_sol.back().time);
		if (sol.back().age > TTOL_MOVEB) {
			msg = "time sync between rover and base failed!";
			return 0;
		}
		for (int i=0; i<3; i++) rb[i] += rb[i + 3]*sol.back().age;
	}
	else {
		solp->age = odt;
		if (solp->age > opt->maxtdiff) {
			msg = "time difference between rover and base too large!";
			return 0;
		}
	}
	return 1;
}
/* satellite-single-differenced dynamic parameters -----------------------------------
* argv   :  int   satnum   number of common satellite
* --------------------------------------------------------------------------------- */
double relative_t::single_distanc(int satnum,int sys) {
	/* R:rover B:base */
	double disR,disB;
	rp = &obsr->data[rovsat[satnum]]; bp = &obsb->data[bassat[satnum]];
	vector<double> *Adyc = satnum==irfsat[sys] ? Airfsat + sys : &Asatnum;
	disR = geodist(rp->posvel,Rxyz,rp->sigvec);
	disB = geodist(bp->posvel,Bxyz,bp->sigvec);

	satazel(Rblh,rp->sigvec,rp->azel);
	satazel(Bblh,rp->sigvec,bp->azel);

	/* Acoe xyz vector */
	for (int i=0; i<3; i++) Adyc->at(i) = -rp->sigvec[i];

	double sddis = disR - disB;

	return sddis;
}
/* satellite-single-differenced troposphere parameters -------------------------------
* argv   :  int   satnum   number of common satellite
* --------------------------------------------------------------------------------- */
double relative_t::single_troppar(int satnum,int sys) {
	rp = &obsr->data[rovsat[satnum]]; bp = &obsb->data[bassat[satnum]];
	vector<double> *Atro = satnum==irfsat[sys] ? Airfsat + sys : &Asatnum;

	/* troposphere delay correction and Acoe tro vector */
	int numtro = NT/2;
	double sdtro;
	// rover and satnum
	if (opt->tropopt >= TROPOPT_EST)
		for (int i=0; i<numtro; i++) trofunc->model.Ptro[i] = Xpar[iT + i];
	trofunc->correction(rp,nav,Rblh,0.7);
	for (int i=0; i<numtro; i++) Atro->at(iT + i) = trofunc->model.Atro[i];
	sdtro = rp->dtro;
	// base and satnum
	if (opt->tropopt >= TROPOPT_EST)
		for (int i=0; i<numtro; i++) trofunc->model.Ptro[i] = Xpar[iT + numtro + i];
	trofunc->correction(bp,nav,Bblh,0.7);
	for (int i=0; i<numtro; i++) Atro->at(iT + numtro + i) -= trofunc->model.Atro[i];
	sdtro -= bp->dtro;

	return sdtro;
}
/* satellite-single-differenced GLO amb-difference parameters ------------------------
* argv   :  int   satnum   number of common satellite
*           int   fff      frequency number
* --------------------------------------------------------------------------------- */
double relative_t::single_gloambd(int satnum,int fff,int sys) {
	vector<double> *Aglo = satnum==irfsat[sys] ? Airfsat + sys : &Asatnum;
	double *lam = satnum==irfsat[sys] ? lam1 : lam2;

	/* glo freq-difference parameter and Acoe glo vector */
	double sdglo = 0.0;
	Aglo->at(iG + fff) = CLIGHT/lam[fff]/1E6;
	sdglo = Aglo->at(iG + fff)*Xpar[iG + fff];

	return sdglo;
}
/* satellite-single-differenced ionosphere parameters --------------------------------
* argv   :  int   satnum   number of common satellite
* --------------------------------------------------------------------------------- */
double relative_t::single_ionopar(int satnum,int sys) {
	rp = &obsr->data[rovsat[satnum]]; bp = &obsb->data[bassat[satnum]];
	vector<double> *Aion = satnum==irfsat[sys] ? Airfsat + sys : &Asatnum;

	double sdion=0.0;
	/* ionosphere correction with certain model */
	if (opt->ionoopt!=IONOOPT_CONST) {
		// rover and satnum
		ionfunc->correction(rp,nav,Rblh);
		sdion = rp->dion;
		// base and satnum
		ionfunc->correction(bp,nav,Bblh);
		sdion -= bp->dion;
	}

	/* estimate constrained DDion vertical delay of GPS L1 (m) */
	else { 
		Aion->at(iI+satnum)=(rp->ionmap+bp->ionmap)/2.0;
		sdion=Aion->at(iI+satnum)*Xpar[ iI+satnum ];
	}

	return sdion;
}
/* satellite-single-differenced ambiguity parameters ---------------------------------
* argv   :  int   satnum   number of common satellite
*           int   fff      frequency number
* --------------------------------------------------------------------------------- */
double relative_t::single_ambtpar(int satnum,int fff,int sys) {
	double *lam = satnum==irfsat[sys] ? lam1 : lam2;
	vector<double> *Aamb = satnum==irfsat[sys] ? Airfsat + sys : &Asatnum;

	double sdamb;
	int namb = ambnum[satnum + fff*comnum];
	/* LC ambiguity parameters and Acoe amb vector */
	if (opt->ionoopt==IONOOPT_IFLC) {
		Aamb->at(iA + namb) = 1.0;
		sdamb = Xpar[iA + namb];
	}

	/* freq ambiguity parameters and Acoe amb vector */
	else {
		Aamb->at(iA + namb) = lam[fff];
		sdamb = lam[fff]*Xpar[iA + namb];
	}

	return sdamb;
}
/* satellite-single-differenced antenna offest ---------------------------- */
void relative_t::single_antoffs(int satnum,int sys) {
	rp = &obsr->data[rovsat[satnum]]; bp = &obsb->data[bassat[satnum]];
	double *ant = satnum==irfsat[sys] ? ant1[sys] : ant2;

	recantfunc.recantoff(opt,0,rp);
	recantfunc.recantoff(opt,1,bp);

	ant[0]= rp->dant[0] - bp->dant[0]; 
	ant[1]= rp->dant[1] - bp->dant[1]; 
	ant[2]= rp->dant[2] - bp->dant[2];
}
/* satellite-single-differenced variance ---------------------------------- */
double relative_t::single_variance(int satnum,int freq) {
	int fff = freq % numF;
	rp = &obsr->data[rovsat[satnum]]; bp = &obsb->data[bassat[satnum]];
	int sys = ssat[comsat[satnum] - 1].sys,prn = obsr->data[rovsat[satnum]].prn;
	double perr = opt->err[1],blerr = baseline*opt->err[2]/1E4,sterr = CLIGHT*opt->sclkstab*odt;
	double pfact = freq<numF ? 1.0 : opt->eratio[fff];
	double sfact;
	if (sys!=SYS_CMP) sfact = sys==SYS_GLO ? EFACT_GLO : (sys==SYS_SBS ? EFACT_SBS : EFACT_GPS);
	else sfact = binary_search(BDS_GEO,BDS_GEO + 6,prn) ? EFACT_CMP_G : EFACT_CMP;
	double sin_elR = sin(rp->azel[1]),sin_elB = sin(bp->azel[1]);

	return (opt->ionoopt==IONOOPT_IFLC ? 3.0 : 1.0)*
		(SQR(pfact*sfact*perr)*(SQR(2.0)+SQR(1.0/sin_elR)+SQR(1.0/sin_elB))+SQR(blerr)) + SQR(sterr);
}
/* irfsat-single-differenced parameters and variance and lam1 ---------------------- */
void relative_t::irfsat_single() {
	for (int sys = 0; sys<4; sys++) {
		Airfsat[sys].assign(numX,0.0);
		if (irfsat[sys]<0) continue; //continue if no observation of this system
		/* irfsat ssat_t and irfsat lamda */
		ssat_t *shhh = &ssat[rfsat[1][sys] - 1];
		lam1 = nav->lam[rfsat[1][sys] - 1];

		// distance
		disRB1[sys] = single_distanc(irfsat[sys],sys);
		// troposphere
		troRB1[sys] = single_troppar(irfsat[sys],sys);
		// glo freq-difference
		if (NG>0 && satsys(rfsat[1][sys],NULL)==SYS_GLO)
			for (int i=0; i<numF&&i<NFREQGLO; i++) gloRB1[sys][i] = single_gloambd(irfsat[sys],i,sys);
		// ionosphere
		ionRB1[sys] = single_ionopar(irfsat[sys],sys);
		// ambiguity
		if (opt->mode > PMODE_DGPS) for (int freq = 0; freq<numF; freq++) {
			if (L_nsat[sys][freq]<2) continue; //continue if satellite <2
			ambRB1[sys][freq] = 0.0;
			/* if slove ambiguity */
			if (ambnum[irfsat[sys]+freq*comnum]>=0) 
				ambRB1[sys][freq] = single_ambtpar(irfsat[sys],freq,sys);
		}
		// rec. antenna
		single_antoffs(irfsat[sys],sys);

		/* irfsat observation variance */
		for (int freq = 0; freq<numF*2; freq++) {
			varRB1[sys][freq] = single_variance(irfsat[sys],freq);
			shhh->vsat[freq%numF] = 1;
		}
	}
}
/* double-differenced value ----------------------------------------------------------
* argv   :  int   satnum   number of common satellite
*           int   freq     frequency number
*           int   sys      satellite system (1:GPS/SBS/QZS,2:GLO,3:GAL,4:BDS)
* process:  compute double-differenced value and its Acoe vector
* --------------------------------------------------------------------------------- */
double relative_t::double_value(int satnum,int freq,int sys) {
	/* observation type */
	int fff = freq % numF;
	ifact1 = (freq<numF?-1.0:1.0)*SQR(lam1[fff]/WaveLengths[0]);
	ifact2 = (freq<numF?-1.0:1.0)*SQR(lam2[fff]/WaveLengths[0]);
	ssat_t *sss = &ssat[comsat[satnum] - 1];

	/* single difference of satnum */
	// distance
	disRB2 = single_distanc(satnum,sys);
	// troposphere
	troRB2 = single_troppar(satnum,sys);
	// glo freq-difference
	gloRB2 = 0.0;
	if (NG>0 && freq<numF&&satsys(comsat[satnum],NULL)==SYS_GLO
		&& fff<NFREQGLO)
		gloRB2 = single_gloambd(satnum,fff,sys);
	// ionosphere
	ionRB2 = single_ionopar(satnum,sys);
	// ambiguity
	ambRB2 = 0.0;
	if (freq<numF) {
		if (opt->modear==ARMODE_LCWN) ambRB2=LC_WN(satnum,sys);
		else {
			ambRB2 = single_ambtpar(satnum,fff,sys); 
			ambRB2 = ambRB1[sys][fff] - ambRB2;
		}	
	}

	// rec. antenna phase center offest
	single_antoffs(satnum,sys);
	double ant = 0.0;
	if (opt->ionoopt==IONOOPT_IFLC) {
		double gamma1=SQR(lam1[1])/SQR(lam1[0]); /* f1^2/f2^2 */
		double gamma2=SQR(lam2[1])/SQR(lam2[0]); /* f1^2/f2^2 */
		ant=(gamma1*ant1[sys][0]-ant1[sys][1])/(gamma1-1.0) - (gamma2*ant2[0]-ant2[1])/(gamma2-1.0);
	}
	else ant=ant1[sys][fff]-ant2[fff];

	/* return double-differenced parameters */
	return disRB1[sys] - disRB2 +
		ifact1*ionRB1[sys] - ifact2*ionRB2 +
		troRB1[sys] - troRB2 +
		gloRB1[sys][fff] - gloRB2 +
		ambRB2 + 
		ant;
}
/* update Rvec vector ------------------------------------------------------------- */
void relative_t::update_Rvec(int satnum,int freq,int sys) {
	/* distance variance */
	Rvec.push_back(varRB1[sys][freq]);
	Rvec.push_back(single_variance(satnum,freq));
}
/* update Lobs vector and lam2 ---------------------------------------------------- */
int relative_t::update_Lobs(int satnum,int freq,int sys) {
	double Lo = 0.0;
	/* frequency wavelength */
	lam2 = nav->lam[comsat[satnum] - 1];
	int fff = freq % numF;

	/* stanum sat */
	unsigned int sat = comsat[satnum];
	ssat_t *sss = &ssat[sat - 1],*shhh = &ssat[rfsat[1][sys] - 1];

	/* phase observation */
	if (freq<numF) {
		if (ambnum[irfsat[sys] + fff*comnum]>=0 && ambnum[satnum + fff*comnum]>=0) {
			/* LC combination */
			if (opt->ionoopt==IONOOPT_IFLC) {
				Lo = shhh->lc12[1] - sss->lc12[1];
			}
			/* Lfreq */
			else {
				Lo = shhh->L[fff].back()*lam1[fff] -
					sss->L[fff].back()*lam2[fff];
			}
		}
		else return 0;
	}
	/* code observation */
	else {
		double p1 = 0.0,p2 = 0.0;
		/* LC combination */
		if (opt->ionoopt==IONOOPT_IFLC) {
			p1 = shhh->pc12[1];
			p2 = sss->pc12[1];
		}
		/* Lfreq */
		else {
			p1 = shhh->P[fff].back();
			p2 = sss->P[fff].back();
		}
		Lo = p1==0.0 || p2==0.0 ? 0.0 : p1 - p2;
	}

	if (Lo==0.0)
		return 0;
	else Lobs.push_back(Lo);
	return 1;
}
/* update Acoe matrix ------------------------------------------------------------- */
void relative_t::update_Acoe(int satnum,int freq,int sys) {
	/* frequency index */
	int fff = freq % numF;

	/* update Acoe matrix */
	// dynamic pararmeters coefficients
	for (int nx=0; nx<ND; nx++)
		Ais[nx] = Airfsat[sys][nx] - Asatnum[nx];
	// troposphere parameters coefficients
	for (int nx = 0; nx<NT; nx++)
		Ais[iT + nx] = Airfsat[sys][iT + nx] - Asatnum[iT + nx];
	// glo freq-difference parameters coefficients
	if (NG>0 && freq<numF&&sys==1 && fff<NFREQGLO)
		Ais[iG + fff] = Airfsat[sys][iG + fff] - Asatnum[iG + fff];
	// ionosphere parameters coefficients
	if (opt->ionoopt==IONOOPT_CONST) {
		Ais[iI + irfsat[sys]] =  ifact1*Airfsat[sys][iI + irfsat[sys]];
		Ais[iI + satnum]      = -ifact2*Asatnum[iI + satnum];
	}
	// ambiguity parameters coefficients
	if (opt->mode > PMODE_DGPS&&freq<numF) {
		int ah = ambnum[irfsat[sys] + fff*comnum],as = ambnum[satnum + fff*comnum];
		if (ah>=0) Ais[iA + ah] = Airfsat[sys][iA + ah];
		if (as>=0) Ais[iA + as] = -Asatnum[iA + as];
	}
	Acoe.insert(Acoe.end(),Ais.begin(),Ais.end());
}
/* double-differenced LC ambiguity using wide-lane and N1 ----------------- */
double relative_t::LC_WN(int satnum,int sys) {
	ssat_t *sss=&ssat[comsat[satnum]-1],*hhh=&ssat[rfsat[1][sys]-1];
	double *lam=nav->lam[comsat[satnum]-1];
	/* float and integar DD wide-lane and narrow-lane (N1) value */
	double f_wide=hhh->mw12[0]-sss->mw12[0],f_narr=hhh->amb_ave[0]-sss->amb_ave[0];
	double fix_wide,fix_narr;

	/* final wide-lane and narrow-lane (N1) ambiguity */
	double coe_narr=1.0/(1.0/lam[0]+1.0/lam[1]),
		coe_wide=1.0/(lam[1]/lam[0]-1.0)/(1.0/lam[0]+1.0/lam[1]);
	int wide=round(f_wide);
	int flag_wide=fabs(wide-f_wide)<0.25&&
		(hhh->lock_LC>iniamb&&sss->lock_LC>iniamb);
	fix_wide = /*flag_wide? wide : */f_wide;
	fix_narr = f_narr;

	/* DD LC ambiguity */
	double DDLC=coe_wide*fix_wide + coe_narr*fix_narr;

	/* update Xpar,its covariance matrix and coefficient vector */
	int nsss=iA+ambnum[satnum],nhhh=iA+ambnum[irfsat[sys]];
	Asatnum[nsss]=1.0;
	/* fixed information */
	if (flag_wide) { 
		Xpar[nsss]=Xpar[nhhh]-DDLC;
		fix_flag[nsss-iA]=fix_flag[nhhh-iA]=1; fix_num++; 
		Rx[nsss+nsss*numX]=Rx[nhhh+nhhh*numX]=SQR(0.007);
	}

	return DDLC;
}
/* baseline-constraint equation for moving-baseling ------------------------------- */
void relative_t::base_line() {
	Lobs.push_back(opt->baseline[0] - baseline);
	Acoe.insert(Acoe.end(),numX,0.0);
	for (int i=0; i<3; i++)
		Acoe[i + numL*numX] = (Rxyz[i] - Bxyz[i])/baseline;
	Rbl = SQR(opt->baseline[1]);
	numL++;
}
/* update Rarr matrix ------------------------------------------------------------- */
void relative_t::update_Rvar() {
	/* initialize Rarr */
	Rvar.assign(numL*numL,0.0);

	for (int sys = 0,nt = 0; sys<4; sys++) {
		for (int i=opt->mode > PMODE_DGPS ? 0 : numF; i<numF*2; i++) {
			for (int j = 0; j<nobs[sys][i]; j++) {
				for (int k = 0; k<nobs[sys][i]; k++) {
					Rvar[nt + k + (nt + j)*numL] = j==k ?
						Rvec[2*(nt + j)] + Rvec[2*(nt + j) + 1] :
						Rvec[2*(nt + j)];
				}
			}
			nt += nobs[sys][i];
		}
	}

	/* baseline variance */
	if (opt->mode==PMODE_MOVEB && opt->baseline[0] >= 0.0)
		Rvar[numL*numL - 1] = Rbl;
}
/* double-differenced observation equation -------------------------------------------
* return : number of used observation (size of L)
* --------------------------------------------------------------------------------- */
int relative_t::double_diff() {
	ns = numL = n_Damb = 0;
	for (int i=0; i<NFREQ*2; i++) for (int j = 0; j<4; j++)
		nobs[j][i] = 0;
	int iobs = 0;
	gtime_t time = obsr->data[0].time;

	/* initialize rover and base position */
	if (norm(Xpar.begin(),3) <= 0.0 || norm(rb,3) <= 0.0) return 0;
	// tidal correction
	if (opt->tidecorr) {
		tidefunc.tidecorr(*time.gpst2utc(),0,Rxyz,Rtide);
		tidefunc.tidecorr(time,1,Bxyz,Btide);
		for (int i=0; i<3; i++) { Rxyz[i] += Rtide[i]; Bxyz[i] += Btide[i]; }
	}
	// geodetic position of rover,base and centre
	ecef2pos(Rxyz,WGS84,Rblh); ecef2pos(Bxyz,WGS84,Bblh);
	/* initialize base-line parameters */
	baseline = distance(Xpar.begin(),rb,3);

	/* single-difference for reference satellite (irfsat) */
	irfsat_single();

	/* loop of different satellites */
	for (int sys=0, nsys=0; sys<4; sys++) {
		if (irfsat[sys]<0) continue; //continue if no observation of this system
		lam1 = nav->lam[rfsat[1][sys] - 1];
		for (int freq = iobs = opt->mode > PMODE_DGPS ? 0 : numF; freq<numF*2; freq++) {
			if (freq<numF&&L_nsat[sys][freq]<2) continue; //continue if satellite <2
			for (int satnum = 0; satnum<comnum; satnum++) {
				/* continue if not this system or centre satellite */
				if (!test_system(ssat[comsat[satnum] - 1].sys,sys) || satnum==irfsat[sys])
					continue;

				/* update Lobs vector and lam2 */
				if (!update_Lobs(satnum,freq,sys))
					continue;

				/* initialize Lobs,Rvec,Acoe for new observation */
				/* Rvec.insert(Rvec.end(),2,0.0); initialize in update_Rvec */
				Asatnum.assign(numX,0.0);
				Ais.assign(numX,0.0);

				/* compute double-difference between rover/base and irfsat/satnum */
				double doul_dis = double_value(satnum,freq,sys);

				/* update Lobs.back() */
				Lobs.back() -= doul_dis;

				/* update Aceo matrix */
				update_Acoe(satnum,freq,sys);

				/* update Rvec vector */
				update_Rvec(satnum,freq,sys);

				/* satnum sat */
				unsigned int sat = comsat[satnum];
				ssat[sat-1].vsat[freq%numF] = 1;
				if (freq<numF)
					ssat[sat-1].resp[freq] = Lobs.back();
				else
					ssat[sat-1].resc[freq-numF] = Lobs.back();
				numL++;
				nobs[sys][iobs]++;
			}
			iobs++;
			if (freq<numF) n_Damb += L_nsat[sys][freq]-1;
		}
		comsys[nsys++]=SYS_LIST[sys];
	}

	/* add baseline equation if moving-baseline */
	if (opt->mode==PMODE_MOVEB && opt->baseline[0] >= 0.0) base_line();

	/* update Rvar using Rvec */
	update_Rvar();
	return 1;
}
/* double-differenced ambiguity to single ambiguity ----------------------- */
void relative_t::ddamb2single(vector<double> &Dx_coe,vector<double> &Damb_Fix,
	vector<double> &R_Damb) {

	int nLa = nA + n_Damb; //number of pseudo-observation of ambiguity
	vector<double> Acoe(nLa*nA,0.0),Lamb(nLa,0.0),RLa(nLa*nLa,0.0),Ramb(nA*nA,0.0),da(nA,0.0);

	/* Acoe */
	for (int i=0; i<nA; i++) Acoe[i + i*nA] = 1.0; //single amb. pseudo-obs. Acoe
	for (int i=0; i<n_Damb; i++) for (int j = 0; j<nA; j++)
		Acoe[j + (i + nA)*nA] = Dx_coe[j + iA + (i + iA)*numX]; //fixed DDamb. pseudo-obs. Acoe
	/* Lamb */
	for (int i=0; i<nA; i++) Lamb[i] = 0.0; //single amb. pseudo-obs. Lamb
	for (int i=0; i<n_Damb; i++) Lamb[i + nA] = -Damb_Fix[i]; //fixed DDamb. pseudo-obs. Lamb
	/* RLa */
	for (int i=0; i<nA; i++) for (int j = 0; j<nA; j++)
		RLa[j + i*nLa] = 100.0*Rx[j + iA + (i + iA)*numX]; //single amb. pseudo-obs. RLa
	for (int i=0; i<n_Damb; i++) for (int j = 0; j<n_Damb; j++)
		RLa[j + nA + (i + nA)*nLa] = 0.01*R_Damb[j + i*n_Damb]; //fixed DDamb. pseudo-obs. RLa

	/* compute least-square solution */
	lsadj_t lsq;
	lsq.lsq(Acoe,Lamb,RLa,da,Ramb,nLa,nA);

	for (int i=0; i<nA; i++) fix_Xpar[i + iA] += da[i];
	/* test */
	/*vector<double> DDD(n_Damb*nA,0.0),DDA(n_Damb,0.0);
	for (int i=0; i<n_Damb; i++) for (int j=0; j<nA; j++) DDD[j+i*nA]=Dx_coe[j+iA+(i+iA)*numX];
	matmul_vec("TN",n_Damb,1,nA,1.0,DDD,fix_Xpar+iA,0.0,DDA,0,iA,0);
	int test=0;
	test=1;*/
}
/* single to double-difference tansformation matrix (Dx_coe) -------------- */
int relative_t::single2doul(vector<double> &Dx_coe) {
	int ddanum = 0;
	/* initialize Dx_coe */
	for (int i=0; i<iA; i++) Dx_coe[i + i*numX] = 1.0;

	/* Dx_coe for ambiguity parameters */
	for (int sys = 0; sys<4; sys++) {
		if (irfsat[sys]<0) continue; //continue if no observation of this system
		for (int freq = 0; freq<numF; freq++)
			if (L_nsat[sys][freq] > 1) for (int satnum = 0; satnum<comnum; satnum++) {
				if (!test_system(ssat[comsat[satnum] - 1].sys,sys) ||
					ambnum[satnum + freq*comnum]<0 || satnum==irfsat[sys]) continue;
				//centre satellite amb
				Dx_coe[iA + ambnum[irfsat[sys] + freq*comnum] + (iA + ddanum)*numX] = 1.0;
				//other satellite amb
				Dx_coe[iA + ambnum[satnum + freq*comnum] + (iA + ddanum)*numX] = -1.0;
				ddanum++;
			}
	}

	return ddanum;
}
/* get fixed solution ----------------------------------------------------- */
int relative_t::get_fixsolution() {
	if (n_Damb <= 0) return 0;
	int Dnum = iA + n_Damb;

	/* solve double-differenced ambiguity vectors */
	/* Dx      : double-differenced format Xpar
	*Dx_Coe  : Xpar to Dx Coefficients
	*DR      : Dx_Coe*Rx
	*Damb    : float double-differenced ambiguity
	*fix_amb : fixed double-differenced ambigtuiy
	*Damb_Fix: Damb - fix_amb
	*R_xa    : covariance of ambiguity and other parameters
	*P_Damb  : inverse of R_Damb
	*d_Damb  : P_Damb*Damb_Fix
	*RR      : R_xa*P_Damb */
	vector<double> Dx_Coe(Dnum*numX,0.0),DR(numX*Dnum,0.0),R_Dx(Dnum*Dnum,0.0),Dx(Dnum,1),
		R_Damb(n_Damb*n_Damb,0.0),Damb(n_Damb,0.0),Damb_Fix(n_Damb,0.0),
		R_xa(iA*n_Damb,0.0),P_Damb,d_Damb(n_Damb,0.0),RR(n_Damb*iA,0.0);

	if (single2doul(Dx_Coe)!=n_Damb) return 0; //number of DD ambiguity

	/* initialize of fixed solution */
	fix_Xpar.assign(Xpar.begin(),Xpar.end()); fix_Rx.assign(Rx.begin(),Rx.end());
	fix_amb.assign(n_Damb*2,0.0);

	/* Dx,Damb,DR,R_Dx,R_Damb,R_xa */
	matmul_vec("TN",Dnum,1,numX,1.0,Dx_Coe,Xpar,0.0,Dx); //Dx
	matmul_vec("TN",Dnum,numX,numX,1.0,Dx_Coe,Rx,0.0,DR); //DR
	matmul_vec("NN",Dnum,Dnum,numX,1.0,DR,Dx_Coe,0.0,R_Dx); //R_Dx

	for (int i=0; i<n_Damb; i++) {
		Damb[i] = Dx[iA + i]; //Damb
		for (int j=0; j<n_Damb; j++) R_Damb[j + i*n_Damb] = R_Dx[j+iA + (i+iA)*Dnum]; //R_Damb
		for (int j=0; j<iA; j++) R_xa[j + i*iA] = R_Dx[j + (i+iA)*Dnum]; //R_xa
	}

	/* fixed DD ambiguity solution using lambda/mlambda */
	lambda_t lambda;
	int flag = lambda.int_amb(Damb,R_Damb,fix_amb,n_Damb,2,sum_var);

	/* if succeed */
	if (flag==0 && sum_var[1]/sum_var[0] >= opt->thresar[0]) {
		/* transform float to fixed solution (fix_Xpar = Xpar - R_xa*P_Damb*Damb_Fix) */
		for (int i=0; i<n_Damb; i++) Damb_Fix[i] = Damb[i] - fix_amb[i];
		P_Damb.assign(R_Damb.begin(),R_Damb.end());
		if (matinv(P_Damb,n_Damb)==0) {
			matmul_vec("NN",n_Damb,1,n_Damb,1.0,P_Damb,Damb_Fix,0.0,d_Damb);
			matmul_vec("NN",iA,1,n_Damb,-1.0,R_xa,d_Damb,1.0,fix_Xpar);
		}
		/* covariance of fixed solution (fix_Rx = Rx - R_xa*P_Damb*R_xa) */
		matmul_vec("NN",iA,n_Damb,n_Damb,1.0,R_xa,P_Damb,0.0,RR);
		matmul_vec("NT",iA,iA,n_Damb,-1.0,RR,R_xa,1.0,fix_Rx);

		/* double-differenced ambiguity to single ambiguity */
		ddamb2single(Dx_Coe,Damb_Fix,R_Damb);

		return 1;
	}
	return 0;
}
/* get fixed wide-lane ambiguity -------------------------------------------------- */
int relative_t::fix_wide_narr(vector<double> &DLC,vector<double> &R_DLC) {
	vector<double> f_wide(n_Damb,0.0),fix_wide(n_Damb,0.0);
	vector<double> f_narr(n_Damb,0.0),fix_narr(2*n_Damb,0.0),narr(n_Damb,0.0); //N1
	vector<int>    lock(n_Damb,0);
	vector<double> coe_LC(n_Damb,0.0),coe_wide(n_Damb,0.0);
	int ndd=0;

	/* fixed double-wide-lane ambiguity */
	for (int sys = 0; sys<4; sys++) {
		if (irfsat[sys]<0) continue; //continue if no observation of this system
		ssat_t *sss,*hhh=&ssat[rfsat[1][sys] - 1];
		if (L_nsat[sys][0] > 1) for (int satnum = 0; satnum<comnum; satnum++) {
			if (!test_system(ssat[comsat[satnum]-1].sys,sys) ||
				ambnum[satnum]<0 || satnum==irfsat[sys]) continue;
			sss=&ssat[comsat[satnum]-1];
			double *lami=nav->lam[comsat[satnum]-1],*lamh=nav->lam[rfsat[1][sys]-1];
			/* float wide-lane */
			f_wide[ndd] = hhh->mw12[0] - sss->mw12[0];
			f_narr[ndd] = hhh->amb_ave[0] - sss->amb_ave[0];
			/* coe_LC = 1/lam1+1/lam2,coe_wide = 1/(lam2/lam1-1) */
			coe_LC[ndd] = 1.0/lami[0] + 1.0/lami[1]; 
			coe_wide[ndd] = 1.0/(lami[1]/lami[0]-1.0);
			/* lock count */
			if (sss->lock_LC>iniamb&&hhh->lock_LC>iniamb) lock[ndd]=1;
			/* number of dd ambiguity */
			ndd++;
		}
	}
	/* fixed wide/narrow-lane ambiguity with nearest int value */
	for (int i=0; i<n_Damb; i++) { 
		int wide=round(f_wide[i]),narr=round(f_narr[i]);
		fix_wide[i] = fabs(wide-f_wide[i])<0.35&&lock[i]? wide : f_wide[i];
		fix_narr[i] = fabs(narr-f_narr[i])<0.35&&lock[i]? narr : f_narr[i];
	}

	/* fixed LC ambiguity */
	for (int i=0; i<n_Damb; i++) 
		fix_amb[i]=1.0/coe_LC[i]*fix_narr[i]+coe_wide[i]/coe_LC[i]*fix_wide[i];

	return 1;
}
/* get LC ambiguity (wide-narrow lane) fixed solution ----------------------------- */
int relative_t::get_LCfixed() {
	if (n_Damb <= 0) return 0;
	int Dnum = iA + n_Damb; //dd-parameters number
	vector<double> Dx_Coe(Dnum*numX,0.0),fix_LC(n_Damb,0.0);
	/*number of DD ambiguity */
	if (single2doul(Dx_Coe)!=n_Damb) return 0;

	/* solve double-differenced ambiguity vectors */
	/* Dx      : double-differenced format Xpar
	*Dx_Coe  : Xpar to Dx Coefficients
	*DR      : Dx_Coe*Rx
	*DLC     : float double-differenced LC ambiguity
	*amb_fix : fixed double-differenced LC ambigtuiy
	*DLC_Fix : DDLC - amb_fix
	*R_xa    : covariance of ambiguity and other parameters
	*P_DLC   : inverse of R_Damb
	*d_DLC   : P_Damb*Damb_Fix
	*RR      : R_xa*P_Damb */
	vector<double> DR(numX*Dnum,0.0),R_Dx(Dnum*Dnum,0.0),Dx(Dnum,1),
		R_DLC(n_Damb*n_Damb,0.0),DLC(n_Damb,0.0),DLC_Fix(n_Damb,0.0),
		R_xa(iA*n_Damb,0.0),P_DLC,d_DLC(n_Damb,0.0),RR(n_Damb*iA,0.0);

	/* initialize of fixed solution */
	fix_Xpar.assign(Xpar.begin(),Xpar.end()); fix_Rx.assign(Rx.begin(),Rx.end());
	fix_amb.assign(n_Damb,0.0);

	/* Dx,Damb,DR,R_Dx,R_Damb,R_xa */
	matmul_vec("TN",Dnum,1,numX,1.0,Dx_Coe,Xpar,0.0,Dx); //Dx
	matmul_vec("TN",Dnum,numX,numX,1.0,Dx_Coe,Rx,0.0,DR); //DR
	matmul_vec("NN",Dnum,Dnum,numX,1.0,DR,Dx_Coe,0.0,R_Dx); //R_Dx

	for (int i=0; i<n_Damb; i++) {
		DLC[i] = Dx[iA + i]; //Damb
		for (int j=0; j<n_Damb; j++) R_DLC[i+j*n_Damb] = R_Dx[i+iA+(j+iA)*Dnum]; //R_Damb
		for (int j=0; j<iA; j++) R_xa[j+i*iA] = R_Dx[j+(i+iA)*Dnum]; //R_xa
	}

	/* get fixed wide-narrow-lane ambiguity and update LC ambiguity */
	if (fix_wide_narr(DLC,R_DLC)) {
		/* update solution */
		/* transform float to fixed solution (fix_Xpar = Xpar - R_xa*P_Damb*Damb_Fix) */
		for (int i=0; i<n_Damb; i++) DLC_Fix[i] = DLC[i] - fix_amb[i];
		P_DLC.assign(R_DLC.begin(),R_DLC.end());
		if (matinv(P_DLC,n_Damb)==0) {
			matmul_vec("NN",n_Damb,1,n_Damb,1.0,P_DLC,DLC_Fix,0.0,d_DLC);
			matmul_vec("NN",iA,1,n_Damb,-1.0,R_xa,d_DLC,1.0,fix_Xpar);
		}
		/* covariance of fixed solution (fix_Rx = Rx - R_xa*P_Damb*R_xa) */
		matmul_vec("NN",iA,n_Damb,n_Damb,1.0,R_xa,P_DLC,0.0,RR);
		matmul_vec("NT",iA,iA,n_Damb,-1.0,RR,R_xa,1.0,fix_Rx);

		/* double-differenced ambiguity to single ambiguity */
		ddamb2single(Dx_Coe,DLC_Fix,R_DLC);

		return 1;
	}

	return 0;
}
/* write state information to state_file ------------------------------------------ */
void relative_t::write_state() {
	/* set float format */
	state_file.setf(ios::fixed);
	state_file << setprecision(4);

	/* solution number */
	state_file << setw(7) << nsol << ": ";
	/* time */
	obsr->data[0].time.time2str(3);
	state_file << obsr->data[0].time.sep << "   Centre Sat:";
	/* centre satellite */
	for (int sys = 0; sys<4; sys++)
		if (irfsat[sys] >= 0) state_file << " " << ssat[rfsat[1][sys] - 1].id;
	/* fixed ratio */
	state_file << "   fix_rate: ";
	if (opt->modear==ARMODE_LCWN) state_file << setw(8) << fix_rate;
	else if (opt->modear==ARMODE_FIXHOLD) state_file << setw(8) << (sum_var[1]/sum_var[0]);

	/* BODY of state ------------------------- */
	/* single solution */
	/*state_file << " single: ";
	for (int i=0; i<3; i++) state_file << setw(14) << sol.back().xdyc[i];*/
	/* tide correction */
	/*for (int i=0; i<3; i++) state_file << setw(9) << Rtide[i];*/
	/* ionosphere parameters */
	/* troposphere parameters */
	if (NT > 0) {
		state_file << "\n    Troposphere: ";
		for (int i=0; i<NT; i++) state_file << setw(8) << *(sPar + iT + i);
	}
	/* GLONASS differenced IFB rate paramters */
	if (NG > 0) {
		state_file << "\n    GLONASS IFB: ";
		for (int i=0; i<NG; i++) state_file << setw(10) << *(sPar + iG + i)*1E6;
	}
	/* helmert component covariance sigma */
	if (opt->adjustfunc==ADJUST_HELMERT) {
		state_file << "\n    HELMERT SGM: ";
		for (int i=0; i<adjfunc->nsys; i++) 
			state_file << "   " << comsys[i] << ":" << setw(8) << adjfunc->sgm2[i];
	}
	/* satellite parameters (prn,az,el,lock_con,ion,amb) */
	state_file << setprecision(3) << "\n";
	for (int i=0; i<comnum; i++) {
		rp = &obsr->data[rovsat[i]]; bp = &obsb->data[bassat[i]];
		ssat_t *sss = &ssat[comsat[i] - 1];
		state_file << setw(4) << " " << sss->id << ":"; //prn
		state_file << " A" << setw(8) << rp->azel[0]*R2D << "  E" << setw(7) << rp->azel[1]*R2D;//rover az el
		/* ionosphere delay of GPS L1 (m) */
		if (nI>0) { 
			state_file << "  ION:" << setw(9) << *(sPar + iI+i);
		}
		/* ambiguity */
		for (int freq = 0; freq<numF; freq++) {
			int namb = ambnum[i + freq*comnum];
			state_file << "   L" << freq + 1;
			/* LC ambiguity */
			if (opt->ionoopt==IONOOPT_IFLC) {
				state_file << setw(6) << sss->lock_LC;//lock_con
				state_file << setw(12) << sss->lcamb; //orignal ambiguity
				if (namb >= 0) state_file << setw(12) << *(sPar + iA + namb); // new ambiguity
				else state_file << setw(12) << 0.0; //amb
				state_file << setw(10) << sss->d_ave[NFREQ+1]; // d_mw12
				state_file << setw(10) << sss->lcamb_ave; // d_N1
			}
			/* normal ambiguity */
			else {
				state_file << setw(6) << sss->lock_con[freq]; //lock_con
				state_file << setw(12) << sss->amb[freq]; //orignal ambiguity
				if (namb >= 0) state_file << setw(12) << *(sPar + iA + namb); // new ambiguity
				else state_file << setw(12) << 0.0; //amb
				state_file << setw(12) << sss->amb_ave[freq]; // d_N1
			}
		}
		state_file << "\n";
	}
}
/* write cycle-jump information --------------------------------------------------- */
void relative_t::write_jump() {
	/* set float format */
	state_file.setf(ios::fixed);
	state_file << setprecision(4);

	/* time */
	obsr->data[0].time.time2str(3);
	state_file << obsr->data[0].time.sep;

	/* BODY of state ------------------------- */
	/* satellite parameters (prn,az,el,lock_con,ion,amb) */
	for (int i=0; i<comnum; i++) {
		state_file << setprecision(3);
		if (i>0) state_file <<"                       ";
		rp = &obsr->data[rovsat[i]]; bp = &obsb->data[bassat[i]];
		ssat_t *sss = &ssat[comsat[i] - 1];
		state_file << " " << sss->id << ":"; //prn
		/* ambiguity */
		for (int freq = 0; freq<numF; freq++) {
			int namb = ambnum[i + freq*comnum];
			state_file << " L" << freq + 1;
			/* LC ambiguity */
			if (opt->ionoopt==IONOOPT_IFLC) {
				state_file << setw(6) << sss->lock_LC;//lock_con
				state_file << setw(12) << sss->lcamb; //orignal ambiguity
				if (namb >= 0) state_file << setw(12) << *(sPar + iA + namb); // new ambiguity
				else state_file << setw(12) << 0.0; //amb
			}
			/* normal ambiguity */
			else {
				state_file << setw(6) << sss->lock_con[freq]; //lock_con
				state_file << setw(12) << sss->amb[freq]; //orignal ambiguity
				if (namb >= 0) state_file << setw(12) << *(sPar + iA + namb); // new ambiguity
				else state_file << setw(12) << 0.0; //amb
			}
		}
		/* cycle-jump infromation */
		state_file << setprecision(5) << "   gf12:" << setw(14) << sss->gf12[1];
		state_file << setprecision(3) << "   mw12:" << setw(12) << sss->mw12[1] 
			<< setw(12) << sss->mw12[0];
		state_file << "\n";
	}
}
/* update satellite sate vector (ssat) -------------------------------------------- */
void relative_t::update_ssat() {
	for (int i=0; i<MAXSAT; i++) {
		ssat[i].vs = 0;
		ssat[i].azel[0] = ssat[i].azel[1] = 0.0;
		ssat[i].resp[0] = ssat[i].resc[0] = 0.0;
		ssat[i].snr[0] = 0;
	}
	ssat_t *sss; int namb;
	for (int satnum = 0; satnum<comnum; satnum++) {
		rp = &obsr->data[rovsat[satnum]];
		sss = &ssat[comsat[satnum] - 1];
		sss->vs = 1;
		sss->azel[0] = rp->azel[0];  sss->azel[1] = rp->azel[1];
		for (int freq = 0; freq<numF; freq++) {
			sss->snr[freq] = rp->SNR[freq];
			/* ionosphere solution */
			if (nI>0) {
				sss->ion_delay=*(sPar + iI+satnum);
				sss->ion_var=*(sRx + iI+satnum + (iI+satnum)*numX);
			}
			/* ambiguity solution */
			if ((namb = ambnum[satnum + freq*comnum])>=0) {
				// LC ambiguity
				if (opt->ionoopt==IONOOPT_IFLC) {
					sss->lcamb = *(sPar+iA+namb);
					sss->lcvar = *(sRx + iA+namb + (iA+namb)*numX);
					if (sss->lock_LC>=iniamb) sss->fix[0]=sss->fix[1]=2;
				}
				// normal ambiguity
				else {
					sss->amb[freq] = *(sPar+iA+namb)/*Xpar[iA+namb]*/;
					sss->ambvar[freq] = *(sRx + iA+namb + (iA+namb)*numX);
					/*if (sss->ambvar[freq]<=0) iniamb=iniamb;*/
					if ((sss->lock_con[freq]<iniamb)) { sss->fix[freq]=1; }
					else if (stat!=SOLQ_FIX) sss->fix[freq]=2;
					if (stat==SOLQ_FIX) {
						if (nreset[freq]||sss->fix[freq]!=3) { 
							sss->ambvar[freq]=SQR(0.03);
							for (int i=0; i<numX; i++) //reset covariance matrix
								*(sRx + i + (iA+namb)*numX) = *(sRx + (iA+namb) + i*numX) = 0.0;
						}
						sss->fix[freq]=3;
					}
				}
			}
		}
	}
}
/* update solution vector (sol) --------------------------------------------------- */
void relative_t::update_sol() {
	solp->type = 0;
	solp->NL = numL;

	/* parameters solution */
	// dynamic parameters
	for (int i=0; i<ND; i++) {
		solp->xdyc[i] = *(sPar+i);
		for (int j = 0; j<ND; j++)
			solp->vdyc[j + i*6] = *(sRx + j+i*numX);
	}
	// ionosphere parameters
	solp->xion.clear(); solp->vion.clear();
	solp->NI=nI;
	for (int i=0; i<nI; i++) {
		solp->xion.push_back(*(sPar+iI+i));
		for (int j = 0; j<nI; j++)
			solp->vion.push_back(*(sRx + j+iI + (i+iI)*numX));
	}
	// troposphere parameters
	for (int i=0; i<NT; i++) {
		solp->xtro[i] = *(sPar+iT+i);
		for (int j = 0; j<NT; j++)
			solp->vtro[j + i*NT] = *(sRx + j+iT + (i+iT)*numX);
	}
	// GLO freq-diffrerence paramters
	for (int i=0; i<NG; i++) {
		solp->xglo[i] = *(sPar+iG+i);
		for (int j = 0; j<NG; j++)
			solp->vglo[j + i*NG] = *(sRx + j+iG + (i+iG)*numX);
	}
	// ambiguity parameters
	solp->NA = nA;
	solp->xamb.assign(sPar+iA,sPar+iA+nA);
	solp->vamb.assign(nA*nA,0.0);
	for (int i=0; i<nA; i++) for (int j = 0; j<nA; j++)
		solp->vamb[j + i*nA] = *(sRx + j+iA + (i+iA)*numX);

	solp->ns = (unsigned int)comnum;
	if (stat==SOLQ_FIX && opt->modear!=ARMODE_LCWN) solp->ratio=float(sum_var[1]/sum_var[0]);
	solp->stat = stat;

	/* update Rx_ALL */
	//NX
	for (int i=0; i<NX; i++) for (int j=0; j<NX; j++) Rx_ALL[j+i*N_ALL]=*(sRx+j+i*numX);
	//nI
	if (nI>0) for (int i=0; i<comnum; i++) {	//loop of satellite
		// nIi and nIj
		for (int j=0; j<comnum; j++) {
			Rx_ALL[ (NX+comsat[j]-1) + (NX+comsat[i]-1)*N_ALL ] = 
				*(sRx + iI+j + (iI+i)*numX);
		}
		//nI and NX
		for (int j=0; j<NX; j++) {
			Rx_ALL[ j + (NX+comsat[i]-1)*N_ALL ] = *(sRx + j + (iI+i)*numX);
			Rx_ALL[ (NX+comsat[i]-1) + j*N_ALL ] = *(sRx + (iI+i) + j*numX);
		}
	}
	//nA
	if (nA>0) {
		ambtime=solp->time; //ambiguity solution time
		int nambi=0,nambj=0;
		int sati,satj;
		
		for (int i=0; i<comnum; i++) for (int fi=0; fi<numF; fi++) if ((nambi=ambnum[i+fi*comnum])>=0) {
			//ambi and ambj
			for (int j=0; j<comnum; j++) for (int fj=0; fj<numF; fj++)
				if ((nambj=ambnum[j+fj*comnum])>=0)
					Rx_ALL[ (NXI+(comsat[j]-1)*numF+fj) + (NXI+(comsat[i]-1)*numF+fi)*N_ALL ] =
					*(sRx + iA+nambj + (iA+nambi)*numX);
			// ambi and NX
			for (int j=0; j<NX; j++) {
				Rx_ALL[ j + (NXI+(comsat[i]-1)*numF+fi)*N_ALL ] = *(sRx + j + (iA+nambi)*numX);
				Rx_ALL[ (NXI+(comsat[i]-1)*numF+fi) + j*N_ALL ] = *(sRx + (iA+nambi) + j*numX);
			}
			// ambi and nI
			if (nI>0) for (int j=0; j<comnum; j++) {
				Rx_ALL[ (NX+comsat[j]-1) + (NXI+(comsat[i]-1)*numF+fi)*N_ALL ] = 
					*(sRx + (iI+j) + (iA+nambi)*numX);
				Rx_ALL[ (NXI+(comsat[i]-1)*numF+fi) + (NX+comsat[j]-1)*N_ALL ] = 
					*(sRx + (iA+nambi) + (iI+j)*numX);
			}
		}
	}
}
/* relative position function ----------------------------------------------------- */
int relative_t::rtkpos() {
	/* carrier phase bias correction */
	if (opt->pppopt.find("-DIS_FCB")!=string::npos) corr_phase_bias();

	/* single solution of rover position */
	/* base position has already been set */
	init_sol(1);
	/* test */
	if (obsr->data[0].time.sep.compare("2017/12/01 07:53:38.000")==0)
		opt->niter=opt->niter;

	if (single()==SOLQ_NONE) return 0;

	/* base observation? */
	if (obsb->n <= 0) {
		msg = "no base observation!";
		return 0;
	}

	/* tt : time difference between current and previous (s) */
	if (sol[MAXSOLBUF-2].time.time!=0) tt = sol.back().time.timediff(sol[MAXSOLBUF-2].time);
	/* time difference between rover and base (s) */
	if (timediff_rb()==0) 
		return 0;

	/* select common satellite and test availability */
	if ((selcomsat()-2*cennum)*numF < NX) {
		msg = "no enough common satellite!";
		return 0;
	}

	/* reset satellites status */
	resetsat();

	/* test */
	if (nsol==iniamb-1)
		opt->niter=opt->niter;

	/* update parameters */
	updatepar();

	/* compute float solution */
	for (int i=0; i<opt->niter; i++) {
		double_diff();

		/* adjustment with least square function */
		if ((adjfunc->adjustment(Acoe,Lobs,Rvar,Xpar,Rx,numL,numX,nobs))==-1) {
			msg = "adjustment error!";
			stat = SOLQ_NONE;
			break;
		}
		sum_var.assign(2,0.0);
		stat=SOLQ_FLOAT;
		nsol++;
	}

	/* fix solution */
	if (stat!=SOLQ_NONE && opt->mode!=PMODE_DGPS) {
		/* normal ambiguity */
		if (opt->modear==ARMODE_FIXHOLD && get_fixsolution()) stat=SOLQ_FIX;
		/* LC ambiguity */
		else if (nsol>iniamb&&opt->modear==ARMODE_LCWN) stat=SOLQ_FIX;
		fix_rate=1.0*fix_num/(1.0*n_Damb);
	}

	/* update solution */
	if (stat!=SOLQ_NONE) {
		if (opt->mode==PMODE_DGPS) stat = SOLQ_DGPS;
		/* final solution iterator */
		sPar = (stat==SOLQ_FIX && opt->modear!=ARMODE_LCWN)? fix_Xpar.begin() : Xpar.begin();
		sRx = (stat==SOLQ_FIX && opt->modear!=ARMODE_LCWN)? fix_Rx.begin() : Rx.begin();

		/* update reference satellite to irfsat */
		for (int i=0; i<4; i++) rfsat[0][i]=rfsat[1][i];

		/* write state information to state_file */
		if (state_file.is_open()) write_state();
			/*write_jump();*/

		/* write satellite state */
		update_ssat();
		/* write solution */
		update_sol();
	}

	return 1;
}