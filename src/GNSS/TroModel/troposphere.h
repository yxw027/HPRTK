#ifndef TROPOSPHERE_H
#define TROPOSPHERE_H

#include "GNSS/DataClass/data.h"

/* troposphere model class ------------------------------------------------------------------------ */
class tromod_t{
	/* Construtors */
	public:
		tromod_t();
		~tromod_t();
	/* Implementaion */
	public:
	/* Components */
	public:
		double Ptro[3],Atro[3];			/* troposphere model parameters and its coefficience
										* (only used in estimated mode) */
};

/* parent troposphere functions ------------------------------------------------------------------- */
class trocorr_t{
	/* Constructors */
	public:
		trocorr_t();
		virtual ~trocorr_t();
	/* Implementation functions */
	protected:
		/* base functions */
		/* triangle functions ----------------------------------------------------- */
		double mapf(double el,double a,double b,double c);
		/* troposphere interpc function ------------------------------------------- */
		double interpc(const double coef[],const double lat);
		/* NMF troposphere mapping function --------------------------------------- */
		double nmftropmapf(const obsd_t *obs,const double pos[3],double *mapfw);
		/* get meterological parameters ------------------------------------------- */
		void getmet(double lat,double *met);
		/* sbas troposphere model (sbas) ------------------------------------------ */
		int sbascorr(obsd_t *obs,const double pos[3]);
	public:
		/* standard troposphere model (saastamoinen) ------------------------------ */
		int saascorr(obsd_t *obs,const double pos[3],const double azel[2],
			const double humi);
		/* virtual troposphere correction ----------------------------------------- */
		virtual int correction(obsd_t *obs,const nav_t *nav,const double pos[3],
			const double humi);

	/* Components */
	public:
		tromod_t model;					/*troposphere model component 
										* (only used in estimate mode) */
};

/* subclass troposphere functions ----------------------------------------------------------------- */
/* saastamoinen model troposphere correction ------------------------------------------------------ */
class saastro_t : public trocorr_t{
	/* Constructors */
	public:
		saastro_t();
		~saastro_t();
	/* Implementation functions */
	protected:
		/* base functions */
	public:
		/* saastamoinen model troposphere correction ------------------------------ */
		int correction(obsd_t *obs,const nav_t *nav,const double pos[3],
			const double humi);
};

/* sbas model troposphere correction -------------------------------------------------------------- */
class sbastro_t : public trocorr_t{
	/* Constructors */
	public:
		sbastro_t();
		~sbastro_t();
	/* Implementation functions */
	protected:
		/* base functions */
	public:
		/* sbas model troposphere correction -------------------------------------- */
		int correction(obsd_t *obs,const nav_t *nav,const double pos[3],
			const double humi);
};

/* estimated model troposphere correction --------------------------------------------------------- */
class estitro_t : public trocorr_t{
	/* Constructors */
public:
	estitro_t();
	~estitro_t();
	/* Implementation functions */
protected:
	/* base functions */
public:
	/* estimated model troposphere correction ------------------------------------- */
	int correction(obsd_t *obs,const nav_t *nav,const double pos[3],
		const double humi);
};

#endif
