#include "GNSS/ParModel/parameter.h"

/* parameter function ----------------------------------------------------------------------------- */
/* Constructor -------------------------------------------------------------------- */
parafunc_t::parafunc_t(){
}
parafunc_t::~parafunc_t(){
}
/* Implementaion functions -------------------------------------------------------- */
int parafunc_t::N_Freqency(prcopt_t *opt){
	return opt->nf;
}
int parafunc_t::N_Dynamic(prcopt_t *opt){
	return opt->dynamics ? 6 : 3;
}
int parafunc_t::N_Tro(prcopt_t *opt){
	return opt->tropopt<TROPOPT_EST ? 0 : 
		(opt->mode<PMODE_DGPS ? 1 : 2) * (opt->tropopt==TROPOPT_EST ? 1 : 3);
}
/* only for relative position ----------------------------------------------------- */
int parafunc_t::N_GLOIFB(prcopt_t *opt){
	return opt->navsys&SYS_GLO&&opt->glomodear==2&&opt->ionoopt!=IONOOPT_IFLC&&opt->mode>PMODE_DGPS ? 
		(NFREQGLO>opt->nf? opt->nf : NFREQGLO) : 0;
}
/* only for ppp position ---------------------------------------------------------- */
int parafunc_t::N_Clock(prcopt_t *opt){
	return opt->mode<PMODE_DGPS ? NSYS : 0;
}