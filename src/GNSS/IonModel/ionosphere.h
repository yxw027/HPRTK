#ifndef IONOSPHERE_H
#define IONOSPHERE_H

#include "hprtk_lib.h"
#include "GNSS/DataClass/data.h"
#include "ConfigFile/config.h"

/* parent ionosphere functions (vertical delay of GPS L1) ----------------------------------------- */
class ioncorr_t{
	/* Constructors */
	public:
		ioncorr_t();
		virtual ~ioncorr_t();
	/* Implementation functions */
	protected:
		/* base functions */
		/* ionospheric pierce point position -------------------------------------- */
		double ionppp(const double pos[3], const obsd_t *obs, const double re, 
			const double hion, double *posp);
	public:
		/* base ionosphere mapping function --------------------------------------- */
		double ionmapf(const double *pos, const double *azel);
	public:
		/* virtual compute ionospere correction (vertical delay of GPS L1) -------------- */
		virtual int correction(obsd_t *obs, const nav_t *nav, const double pos[3]);
	/* Components */
};

/* subclass ionosphere functions ------------------------------------------------------------------ */
/* ionosphere free combination -------------------------------------------------------------------- */
class LCion_t : public ioncorr_t{
	/* Constructors */
	public:
		LCion_t();
		~LCion_t();
	/* Implementation functions */
	public:
		/* return 0 delya --------------------------------------------------------- */
		virtual int correction(obsd_t *obs,const nav_t *nav,const double pos[3]);
};
/* broadcast ionosphere correction (vertical delay of GPS L1) ------------------------------------- */
class broadion_t : public ioncorr_t{
	/* Constructors */
	public:
		broadion_t();
		virtual ~broadion_t();
	/* Implementation functions */
	protected:
		/* base functions --------------------------------------------------------- */
		/* ionosphere Klobuchar model correction (vertical delay of GPS L1) ------- */
		int klobion(obsd_t *obs, const double ionpara[8], const double pos[3]);
	public:
		/* compute broadcast ionospere correction (vertical delay of GPS L1) ------ */
		virtual int correction(obsd_t *obs, const nav_t *nav, const double pos[3]);
};

/* qzss broadcast ionosphere correction (vertical delay of GPS L1) -------------------------------- */
class qzssion_t : public broadion_t{
	/* Constructors */
	public:
		qzssion_t();
		~qzssion_t();
	/* Implementation functions */
	public:
		/* compute qzss broadcast ionospere correction (vertical delay of GPS L1) - */
		virtual int correction(obsd_t *obs, const nav_t *nav, const double pos[3]);
};

/* sbas ionosphere correction (vertical delay of GPS L1) ------------------------------------------ */
class sbasion_t : public ioncorr_t{
	/* Constructors */
	public:
		sbasion_t();
		~sbasion_t();
	/* Implementation functions */
	protected:
		/* base functions --------------------------------------------------------- */
		/* search igps ------------------------------------------------------------ */
		void searchigp(gtime_t time,const double pos[2],const sbsion_t *ion,
			const sbsigp_t **igp,double &x,double &y);
		/* variance of ionosphere correction (give=GIVEI+1) ----------------------- */
		double varicorr(int udre);
	public:
		/* compute sbas ionospere correction (vertical delay of GPS L1) ----------- */
		virtual int correction(obsd_t *obs, const nav_t *nav, const double pos[3]);
};

/* ionex ionosphere correction (vertical delay of GPS L1) ----------------------------------------- */
class ionexion_t : public broadion_t{
	/* Constructors */
	public:
		ionexion_t();
		virtual ~ionexion_t();
	/* Implementation functions */
	protected:
		/* base functions --------------------------------------------------------- */
		/* data index (i:lat,j:lon,k:hgt) ----------------------------------------- */
		int dataindex(int i,int j,int k,const int *ndata);
		/* interpolate tec grid data ---------------------------------------------- */
		int interptec(const tec_t *tec,int k,const double *posp,double &value,
			double &rms);
		/* ionosphere delay by tec grid data -------------------------------------- */
		int iondelay(const tec_t *tec,const double *pos,obsd_t *obs,
			double *delay,double *var);
	public:
		/* compute ionex ionospere correction (vertical delay of GPS L1) ---------- */
		virtual int correction(obsd_t *obs, const nav_t *nav, const double pos[3]);
};

/* lex ionosphere correction (vertical delay of GPS L1) ------------------------------------------- */
class lexioncor_t : public ioncorr_t{
	/* Constructors */
	public:
		lexioncor_t();
		~lexioncor_t();
	/* Implementation functions */
	protected:
		/* base functions --------------------------------------------------------- */
	public:
		/* compute lex ionospere correction (vertical delay of GPS L1) ------------ */
		virtual int correction(obsd_t *obs, const nav_t *nav, const double pos[3]);
};

/* constrained ionosphere model correction (L1) --------------------------------------------------- */
class constion_t : public ionexion_t{
	/* Constructors */
public:
	constion_t();
	~constion_t();
	/* Implementation functions */
protected:
public:
	/* compute constrained ionosphere model correction (vertical delay of GPS L1) ---- */
	virtual int correction(obsd_t *obs, const nav_t *nav, const double pos[3]);
};

#endif
