#ifndef AMBIGUITY_H
#define AMBIGUITY_H

#include "hprtk_lib.h"
/* integar ambiguity strategy parent class -------------------------------------------------------- */
class intamb_t{
	/* Constructor */
	public:
		intamb_t();
		virtual ~intamb_t();
	/* Implementaion functions */
	public:
		/* resolve integer ambiguity ---------------------------------------------- */
		virtual int int_amb(vector<double> &Float_Amb,vector<double> &Amb_Var,vector<double> &Fix_Amb,
			int Amb_Num,int Fix_Num,vector<double> &Sum_Var);
	/* Components */
	protected:
		int state;						/* resolve integer ambiguity state */
		int n_amb,n_fix;				/* number of ambiguity and number of fixed solution */
		string errmsg;					/* error message */
};

/* lambda/mlambda integer least-square estimation ------------------------------------------------- */
class lambda_t : public intamb_t {
	/* Constructor */
	public:
		lambda_t();
		~lambda_t();
	/* Implementaion functions */
	protected:
		/* integer gauss transformation ------------------------------------------- */
		void gauss(int num_i,int num_j);
		/* permutations ----------------------------------------------------------- */
		void permutations(int num_j,double del);

		/* compute factorization matrix L and D ----------------------------------- */
		int factorization_LD(vector<double> &Amb_Var);
		/* compute reduction matrix Z --------------------------------------------- */
		void reduction_Z();
		/* mlambda search to get z_fix -------------------------------------------- */
		int search_zfix(vector<double> &Sum_Var);
	public:
		/* resolve integer ambiguity using lambda/mlambda ------------------------- */
		virtual int int_amb(vector<double> &Float_Amb,vector<double> &Amb_Var,vector<double> &Fix_Amb,
			int Amb_Num,int Fix_Num,vector<double> &Sum_Var);
	/* Components */
	protected:
		vector<double> L,D,Z_trans,z_float,z_fix;
};

#endif