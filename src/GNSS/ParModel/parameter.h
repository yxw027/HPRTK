#ifndef PARAMETER_H
#define PARAMETER_H

#include "ConfigFile/config.h"

/* parameter function ----------------------------------------------------------------------------- */
class parafunc_t {
	/* Constructors */
	public:
		parafunc_t();
		~parafunc_t();
	/* Implementaion functions */
	public:
		/* frequency number ------------------------------------------------------- */
		int N_Freqency(prcopt_t *opt);
		/* dynamic parameter number (xyz or xyz+vel[3]+acc[3]) -------------------- */
		int N_Dynamic(prcopt_t *opt);
		/* troposphere parameter number ------------------------------------------- */
		int N_Tro(prcopt_t *opt);
		/* GLO receiver differenced IFB rate (only for relative position) --------- */
		int N_GLOIFB(prcopt_t *opt);
		/* receiver clock (only for ppp) ------------------------------------------ */
		int N_Clock(prcopt_t *opt);
};

#endif
