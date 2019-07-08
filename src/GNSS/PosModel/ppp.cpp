#include "GNSS/PosModel/position.h"
#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
#define SQR(x)      ((x)*(x))
#define SQRT(x)     ((x)<=0.0?0.0:sqrt(x))

#define INIT_ION	0.5					/* initial Enm of ion POLY model */
#define INIT_GRA	1E-5				/* initial tro gradient */
#define VAR_GRA     SQR(0.001)			/* initial variance of gradient (m^2) */
#define INIT_HWBIAS	1E-6				/* initial glo frequency-dependent amb bias */
#define VAR_HWBIAS  SQR(1.0)			/* initial variance of h/w bias ((m/MHz)^2) */
#define RAT_HWBIAS	1E-6				/* growth rate of std h/w bias (m/MHz/s) */

#define VAR_POS     SQR(30.0)			/* initial variance of receiver pos (m^2) */
#define VAR_VEL     SQR(50.0)			/* initial variance of receiver vel ((m/s)^2) */

#define TTOL_MOVEB  (1.0+2*DTTOL)

/* precise point position class ----------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
ppp_t::ppp_t() {
	numX = 0;
	iI = iT = iA = iG = iC = 0;
}
ppp_t::~ppp_t() {
	ambfloat.clear(); floatvar.clear();
	ambfix.clear();   fixvar.clear();
}
/* Implementation functions ----------------------------------------------------------------------- */
/* precise point position function ------------------------------------------------ */
int ppp_t::rtkpos() {
	/* carrier phase bias correction */
	if (opt->pppopt.find("-DIS_FCB") != string::npos) corr_phase_bias();

	init_sol(1);

	return 1;
}