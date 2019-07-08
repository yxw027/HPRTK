#ifndef ADJUSTMENT_H
#define ADJUSTMENT_H

#include "hprtk_lib.h"
#define  POINTER ;

/* parent adjustment functions -------------------------------------------------------------------- */
class adjfunc_t{
	/* Constructor */
	public:
		adjfunc_t();
		virtual ~adjfunc_t();
	/* Implementaion functions */
	protected:
	/* Helmert component covariance estimate for multi-GNSS observation ----------- */
	int helmert_est(const vector<double> &A,const vector<double> &Rx,
		int numL,int numX,const int nobs[4][NFREQ*2]);
	public:
		/* least-squre adjustment function ---------------------------------------- */
		int lsq(const vector<double> &A,const vector<double> &L,
			const vector<double> &R,vector<double> &X,vector<double> &Rx,
			int numL,int numX);
		/* estimate parameter X array and covariance Rx --------------------------- */
		virtual int adjustment(const vector<double> &A,const vector<double> &L,
			const vector<double> &R,vector<double> &X,vector<double> &Rx,
			int numL,int numX,const int nobs[4][NFREQ*2]);

	/* Components */
	protected:
		/* normal parameters */
		vector<double> V;				/* residual vector */
		vector<double> dX;				/* parameter correction value */
		/* Helmert component covariance estimate for multi-GNSS observation */
		vector<double> newR;			/* correct covariance matrix R */
		vector<double> Rx_ori;			/* original Rx matrix */
		int H_niter;					/* iteration number */
		/* for robust */

	public:
		int nsys;						/* number of systems */
		vector<double> sgm2;			/* new and old unit weight covariance (sigma^2) */
};

/* subclass adjustment functions ------------------------------------------------------------------ */
/* least square adjustment function --------------------------------------------------------------- */
class lsadj_t : public adjfunc_t{
	/* Constructor */
	public:
		lsadj_t();
		~lsadj_t();
	/* Implementaion functions */
	public:
		/* estimate parameter X array and covariance Rx --------------------------- */
		int adjustment(const vector<double> &A,const vector<double> &L,
			const vector<double> &R,vector<double> &X,vector<double> &Rx,
			int numL,int numX,const int nobs[4][NFREQ*2]);
};

/* kalman filter adjustment function -------------------------------------------------------------- */
class kalmanadj_t : public adjfunc_t{
	/* Constructor */
	public:
		kalmanadj_t();
		~kalmanadj_t();
	/* Implementaion functions */
		/* base functions */
	public:
		/* estimate parameter X array and covariance Rx --------------------------- */
		virtual int adjustment(const vector<double> &A,const vector<double> &L,
			const vector<double> &R,vector<double> &X,vector<double> &Rx,
			int numL,int numX,const int nobs[4][NFREQ*2]);
};

/* helmert components covariance estimate for kalman filter adjustment function ------------------- */
class helmert_t : public kalmanadj_t {
	/* Constructor */
	public:
		helmert_t();
		~helmert_t();
	/* Implementaion functions */
	/* base functions */
	public:
		/* estimate parameter X array and covariance Rx --------------------------- */
		int adjustment(const vector<double> &A,const vector<double> &L,
			const vector<double> &R,vector<double> &X,vector<double> &Rx,
			int numL,int numX,const int nobs[4][NFREQ*2]);
};
#endif
