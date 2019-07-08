#ifndef POSITION_H
#define POSITION_H

#include "GNSS/DataClass/data.h"
#include "GNSS/rtkpro.h"

/* position types --------------------------------------------------------------------------------- */
/* single rtk-position class ---------------------------------------------------------------------- */
class single_t : public rtk_t{
	/* Constructors */
	public:
		single_t();
		virtual ~single_t();
	/* Implementation functions */
	protected:
		/* system code to system number (1-4) ------------------------------------- */
		int syscd2num(int sys);
		/* initialize solution vectors -------------------------------------------- */
		void init_sol(int rovbas);
		/* reset satellite status ------------------------------------------------- */
		void resetsat();
		/* carrier-phase bias (fcb) correction ------------------------------------ */
		void corr_phase_bias();
		/* test SNR mask ---------------------------------------------------------- */
		int testsnr(int base,int freq,double el,double snr,
			const snrmask_t *mask);
		/* get tgd parameter (m) -------------------------------------------------- */
		double gettgd(int sat);
		/* pseudorange measurement error variance --------------------------------- */
		double varerr(const obsd_t &obs,int sys);
		/* verify excluded satellite ---------------------------------------------- */
		int satexclude(obsd_t &obs);
		/* exclude observation with large residual -------------------------------- */
		void exc_largeres();

		/* psendorange with code bias correction and combination ------------------ */
		double prange(obsd_t &obs,const int iter);
		/* compute observation\coefficient\covariance vector ---------------------- */
		int codearr(const int iter);

		/* write state information to state_file ---------------------------------- */
		virtual void write_state();
		/* update satellite sate vector (ssat) ------------------------------------ */
		virtual void update_ssat();
		/* update solution vector (sol) ------------------------------------------- */
		virtual void update_sol();

		/* growth variance of static rover position ------------------------------- */
		double growth_rate_static();

		/* estimate receiver position --------------------------------------------- */
		int singlepos();	
		/* single point position -------------------------------------------------- */
		int single();
	public:
		/* base station single-position -------------------------------------------- */
		int basepos();
		/* virtual rtk-position function ------------------------------------------- */
		virtual int rtkpos();

	/* Components */
	public:
		int stat;						/* solution stat */
		int numL;						/* number of used observation (size of L) */
		int ns;							/* number of valid satellite */
		int niter;						/* number of iteration */
		/* parameters */
		/* sequence of parameters: ND NI NT NG NC NA */
		vector<double> Lobs,Acoe,Rvar,Rvec,Xpar,Rx;
										/* Lobs: observation vector */
										/* Acoe: coefficients matrix */
										/* Rvar: obs covariance matrix */
										/* Rvec: obs variance vector */
										/* Xpar: parameters vector */
										/* Rx  : parameters covariance matrix */
		int resflag;					/* exclude observation with large residual flag */
		int ncode[2][4];				/* code observation number of each systems [0:rov,1:base] */
		int *ncd;						/* pointer of ncode[2] */
};

/* precise point position class ------------------------------------------------------------------- */
class ppp_t : public single_t{
	/* Constructors */
	public:
		ppp_t();
		~ppp_t();
	/* Implementation functions */
	protected:

	public:
		/* precise point position function -------------------------------------------- */
		int rtkpos();

	/* Components */
	protected:
		/* parameters */
		int numX;						/* current parameters number */
		int iI,iT,iG,iC,iA;				/* start index of kinds of parameters */
		/* ambiguity parameters */
		vector<double> ambfloat,floatvar,fixvar;
		vector<int> ambfix;
										/* ambfloat: float ambiguity vector */
										/* floatvar: float covariance matrix */
										/* ambfix  : fix ambiguity vector */
										/* fixvar  : fix covariance matrix */
};

/* relative position class ------------------------------------------------------------------- */
class relative_t : public single_t{
	/* Constructors */
	public:
		relative_t();
		~relative_t();
	/* Implementation functions */
	protected:
		/* test satellite system -------------------------------------------------- */
		int test_system(int Sysyem,int System_Num);
		/* select common satellites of rover and base ----------------------------- */
		int selcomsat();

		/* initialize centre satellites parameters -------------------------------- */
		void init_censat();
		/* initialize vector and matrix according common satellite -----------------*/
		void init_arrmat();
		/* initialize covariance matrix of all parameters */
		void init_RxALL(unsigned int sat, int freq);

		/* update parameters functions -------------------------------------------- */
		/* update dynamic parameters ---------------------------------------------- */
		void updatexyz();
		/* update troposphere parameters ------------------------------------------ */
		void updatetro();
		/* update glonass ambiguity bias of frequency ----------------------------- */
		void updateglo();
		/* update ambiguity parameters -------------------------------------------- */
		void updateamb();
		/* update ionosphere parameters ------------------------------------------- */
		void updateion();
		/* update parameters covariance matrix ------------------------------------ */
		void updatevar();
		/* update parameters from previous time to current time ------------------- */
		void updatepar();

		/* time difference between rover and base --------------------------------- */
		int timediff_rb();
		/* satellite-single-differenced dynamic parameters ------------------------ */
		double single_distanc(int satnum,int sys);
		/* satellite-single-differenced troposphere parameters -------------------- */
		double single_troppar(int satnum,int sys);
		/* satellite-single-differenced GLO amb-difference parameters ------------- */
		double single_gloambd(int satnum,int fff,int sys);
		/* satellite-single-differenced ionosphere parameters --------------------- */
		double single_ionopar(int satnum,int sys);
		/* satellite-single-differenced ambiguity parameters ---------------------- */
		double single_ambtpar(int satnum,int fff,int sys);
		/* satellite-single-differenced antenna offest ---------------------------- */
		void single_antoffs(int satnum,int sys);
		/* satellite-single-differenced variance ---------------------------------- */
		double single_variance(int satnum,int freq);
		/* ihsat-single-differenced parameters and variance and lam1 -------------- */
		void irfsat_single();

		/* update Rvec vector ----------------------------------------------------- */
		void update_Rvec(int satnum,int freq,int sys);
		/* update Lobs vector and lam2 -------------------------------------------- */
		int update_Lobs(int satnum,int freq,int sys);
		/* update Acoe matrix ----------------------------------------------------- */
		void update_Acoe(int satnum,int freq,int sys);
		/* update Rarr matrix ----------------------------------------------------- */
		void update_Rvar();

		/* double-differenced LC ambiguity using wide-lane and N1 ----------------- */
		double LC_WN(int satnum,int sys);
		/* baseline-constraint equation for moving-baseling ----------------------- */
		void base_line();
		/* double-differenced value ----------------------------------------------- */
		double double_value(int satnum,int freq,int sys);
		/* double-differenced observation equation -------------------------------- */
		int double_diff();

		/* double-differenced ambiguity to single ambiguity ----------------------- */
		void ddamb2single(vector<double> &Dx_coe,vector<double> &Damb_Fix,
			vector<double> &R_Damb);
		/* single to double-difference tansformation matrix (Dx_coe) -------------- */
		int single2doul(vector<double> &Dx_coe);
		/* get fixed solution ----------------------------------------------------- */
		int get_fixsolution();
		/* get fixed wide-lane ambiguity ------------------------------------------ */
		int fix_wide_narr(vector<double> &DLC,vector<double> &R_DLC);
		/* get LC ambiguity (wide-narrow lane) fixed solution --------------------- */
		int get_LCfixed();

		/* write state information to state_file ---------------------------------- */
		virtual void write_state();
		void write_jump();
		/* update satellite sate vector (ssat) ------------------------------------ */
		void update_ssat();
		/* update solution vector (sol) ------------------------------------------- */
		void update_sol();
	public:
		/* relative position function --------------------------------------------- */
		int rtkpos();
	/* Components */
	protected:
		int nsol;						/* number of continue total solution */
		/* station geographic parameters (ini. in  updatepar()) */
		double odt;						/* time difference between obs. of rover and base (s) */
		double Rxyz[3],Bxyz[3];			/* rover and base position with tidal correction (ecef) */
		double Rblh[3],Bblh[3];			/* rover and base position (geodetic) */
		double Rtide[3],Btide[3];		/* tide correction for rover and base */
		double baseline;				/* base-line length */
		double Rbl;						/* variance of baseline constraint */

		/* common satellite observation (ini. in selcomsat()) */
		vector<unsigned int> comsat;	/* common satellites vector (rover and base) */
		vector<int> rovsat,bassat;		/* common satellites index in rover and base obs */
		int comnum,cennum;				/* common satellites number (rover and base) */
		int L_nsat[4][NFREQ];			/* number of satellites that have available phase obs. irfsat
										 * of NFREQ freq. of 4 system  */
		char comsys[4];					/* systems list of common satellites */

		/* centre satellite parameters (sat 1) */
		unsigned char rfsat[2][4];		/* centre satellite of 4 system (0:last,1:now) */
		int irfsat[4];					/* index of reference satellite of 4 system */
		double disRB1[4],troRB1[4],gloRB1[4][2],ionRB1[4],ambRB1[4][3],ant1[4][3];	
										/* chsat single-differenced correction 
										* ionRB1 represents L1 ion delay */
		double varRB1[4][NFREQ*2];			/* chsat single-differenced variance */

		/* another satellite parameters (sat 2) */
		double disRB2,troRB2,gloRB2,ionRB2,ambRB2,ant2[3];
										/* satnum single-differenced correction
										 * ionRB2 represents L1 ion delay */
		double *lam1,*lam2;				/* pointor to lamda vector in nav */	

		/* observation parameters */
		int nobs[4][NFREQ*2];				/* number of 4 system code/phase observation */
		obsd_t *rp,*bp;					/* obs. pointor of rover and base */

		/* parameters (ini. in iniarrmat()) */
		vector<double> Rxvec;			/* parameters variance vector */
		vector<double> Airfsat[4];		/* Acoe vector of rfsat single-difference */
		vector<double> Asatnum,Ais;	
										/* Acoe vector of atnum single-difference and
										 * Acoe vector of double-difference */
		
		int numX;						/* current parameters number */
		int iT,iG,iI,iA;				/* start index of kinds of parameters */
		int nI,nA;						/* ionosphere, ambiguity parameters */
		
		/* ionosphere parameters  */
		double ifact1,ifact2;			/* frequency factor of ionosphere delay */
		int ion_reset[4];				/* ionosphere parameter reset flag */

		/* ambiguity parameters (some ini. in doublediff()) */
		int iniamb;						/* set initial ambiguity if lock count < iniar */
		int nreset[NFREQ];				/* number of reseted ambiguity of each frequency */
		int reset_flag[NFREQ];			/* flag of reset all ambiguity */
		vector<int> ambnum;				/* amb number of one satellite frequency */
		vector<double> fix_amb,sum_var;
										/* fix_amb: fixed  double-ambiguity vector */
										/* sum_var: sum of squared residuals of fix_amb */

		/* fixed solution */
		int n_Damb,fix_num;				/* number of all and fixed DD ambiguity */
		double fix_rate;				/* DD ambiguity fixed rate */
		vector<int> fix_flag;			/* fix flag of each DD ambiguity */
		vector<double> fix_Xpar,fix_Rx; 

		/* final solution iterator */
		vector<double>::iterator sPar,sRx;
};
#endif
