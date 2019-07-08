#ifndef SATELLTIE_H
#define SATELLTIE_H

#include "GNSS/DataClass/data.h"

#include "GNSS/AntModel/antenna.h"

/* satellite functions ---------------------------------------------------------------------------- */
/* parent class of satellite functions ------------------------------------------------------------ */
class satellite_t{
	/* Constructors */
	public:
		satellite_t();
		virtual ~satellite_t();
	/* Implementation functions */
	protected:
		/* virtual satellite clocks function -------------------------------------- */
		virtual int satclk(obsd_t *data, const nav_t *nav);
	public:
		/* virtual satellite position function ------------------------------------ */
		virtual int satpos(obsd_t *data, int iode, const nav_t *nav);
		/* compute satellite positions and clocks --------------------------------- */
		int satposclk(obs_t *obs, const nav_t *nav);
	/* Components */
	public:
		satantenna_t *satantfunc;		/* satellite antenna functions (point to rtk_t) */

};
 
/* subclass of satellite functions ---------------------------------------------------------------- */
/* broadcast ephemeris ---------------------------------------------------------------------------- */
class broadcast_t : public satellite_t{
	/* Constructors */
	public:
		broadcast_t();
		virtual ~broadcast_t();
	/* Implementation functions */
	protected:
		/* broadcast functions ---------------------------------------------------- */
		/* variance by ura ephemeris (ref [1] 20.3.3.3.1.1) ----------------------- */
		double var_uraeph(int ura);
		/* variance by ura ssr (ref [4]) ------------------------------------------ */
		double var_urassr(int ura);
		/* glonass orbit differential equations ----------------------------------- */
		void deq(const double *x,double *xdot,const double *acc);
		/* glonass position and velocity by numerical integration ----------------- */
		void glorbit(double t,double *x,const double *acc);

		/* select GPS/GAL/QZS/CMP ephemeris --------------------------------------- */
		int seleph(obsd_t *data,int iode,const nav_t *nav);
		/* select GLO ephemeris --------------------------------------------------- */
		int selgeph(obsd_t *data,int iode,const nav_t *nav);
		/* select SBS ephemeris --------------------------------------------------- */
		int selseph(obsd_t *data,const nav_t *nav);
		/* position functions ----------------------------------------------------- */
		/* position from GPS/GAL/QZS/CMP ephemeris -------------------------------- */
		void eph2pos(obsd_t *data, const eph_t eph);
		/* position bias from GLO ephemeris --------------------------------------- */
		void geph2pos(obsd_t *data,const geph_t geph);
		/* position bias from SBS ephemeris --------------------------------------- */
		void seph2pos(obsd_t *data,const seph_t seph);
		/* clock functions -------------------------------------------------------- */
		/* clock bias from GPS/GAL/QZS/CMP ephemeris ------------------------------ */
		void eph2clk(obsd_t *data,const eph_t eph);
		/* clock bias from GLO ephemeris ------------------------------------------ */
		void geph2clk(obsd_t *data,const geph_t geph);
		/* clock bias from SBS ephemeris ------------------------------------------ */
		void seph2clk(obsd_t *data,const seph_t seph);

		/* broadcast satellite position function ---------------------------------- */
		int broadpos(obsd_t *data,int iode,const nav_t *nav);
		/* broadcast satellite clocks function ------------------------------------ */
		int satclk(obsd_t *data,const nav_t *nav);
	public:
		/* broadcast satellite position function called by satposclk() ------------ */
		virtual int satpos(obsd_t *data,int iode,const nav_t *nav);
};

/* broadcast ephemeris with sbas correction ------------------------------------------------------- */
class broadsbas_t : public broadcast_t{
	/* Constructors */
	public:
		broadsbas_t();
		~broadsbas_t();
	/* Implementation functions */
	protected:
		/* base functions --------------------------------------------------------------- */
	public:
		/* broadcast satellite position function with sbas correction ------------------- */
		int satpos(obsd_t *data,int iode,const nav_t *nav);
};
/* broadcast ephemeris with ssr_apc correction ---------------------------------------------------- */
class broadssrapc_t : public broadcast_t{
	/* Constructors */
	public:
		broadssrapc_t();
		~broadssrapc_t();
	/* Implementation functions */
	protected:
		/* base functions --------------------------------------------------------------- */
	public:
		/* broadcast satellite position function with ssr_apc correction ---------------- */
		int satpos(obsd_t *data,int iode,const nav_t *nav);
};
/* broadcast ephemeris with ssr_com correction ---------------------------------------------------- */
class broadssrcom_t : public broadcast_t{
	/* Constructors */
	public:
		broadssrcom_t();
		~broadssrcom_t();
	/* Implementation functions */
	protected:
		/* base functions --------------------------------------------------------------- */
	public:
		/* broadcast satellite position function with ssr_com correction ---------------- */
		int satpos(obsd_t *data,int iode,const nav_t *nav);
};

/* precise ephemeris ------------------------------------------------------------------------------ */
class preciseph_t : public satellite_t{
	/* Constructors */
	public:
		preciseph_t();
		~preciseph_t();
	/* Implementation functions */
	protected:
		/* base precise ephemeris functions --------------------------------------- */
		/* polynomial interpolation by Lagrange's algorithm (time interpolation) -- */
		double interpolLaR(const double *dt, gtime_t *ptime, const double *ppos, 
			int n);
		/* precise satellite position of one epoch -------------------------------- */
		int precisepos(obsd_t *data, const nav_t *nav);
		/* precise satellite clocks function -------------------------------------- */
		int satclk(obsd_t *data,const nav_t *nav);
	public:
		/* precise satellite position function ------------------------------------ */
		int satpos(obsd_t *data,int iode,const nav_t *nav);
	/* Components */
	protected:
		double scvar;					/* satellite clock variance */
};

/* qzss lex ephemeris ----------------------------------------------------------------------------- */
class qzsslex_t : public satellite_t{
	/* Constructors */
	public:
		qzsslex_t();
		~qzsslex_t();
	/* Implementation functions */
	protected:
		/* base functions --------------------------------------------------------- */
		/* ura value -------------------------------------------------------------- */
		double vareph(int ura);
		/* qzss lex satellite clocks function ------------------------------------- */
		int satclk(obsd_t *data,const nav_t *nav);
	public:
		/* qzss lex satellite position function ----------------------------------- */
		int satpos(obsd_t *data,int iode,const nav_t *nav);
};
#endif