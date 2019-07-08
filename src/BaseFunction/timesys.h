/* Time function head file */

#ifndef TIMESYS_H
#define TIMESYS_H

#include "hprtk_lib.h"

/* time structure --------------------------------------------------------------------------------- */
class gtime_t{
	/* Constructors */
	public:
		gtime_t();
		/* initialize with epoch array -------------------------------------------- */
		gtime_t(const double *epoch);
		~gtime_t();
	public:
	/* implementation function */
		/* string to time --------------------------------------------------------- */
		int str2time(string s);
		/* ep time to string ------------------------------------------------------ */
		string time2str(int n);
		/* calender day/time (ep) to time ----------------------------------------- */
		gtime_t *epoch2time(const double *inep);
		/* time to calender day/time (ep) ----------------------------------------- */
		void time2epoch();
		/* gps week time to time -------------------------------------------------- */
		gtime_t *gpst2time(int week, double sss);
		/* time to gps week tme --------------------------------------------------- */
		double time2gpst(int *week) const;
		/* galileo week time to time ---------------------------------------------- */
		gtime_t *gst2time(int week, double sss);
		/* time to galileo week time ---------------------------------------------- */
		double time2gst(int *week);
		/* Beidou week time to time ----------------------------------------------- */
		gtime_t *bdt2time(int week, double sss);
		/* time to Beidou week time ----------------------------------------------- */
		double time2bdt(int *week);
		/* add time --------------------------------------------------------------- */
		gtime_t *timeadd(double dsec);
		/* difference with other time --------------------------------------------- */
		double timediff(const gtime_t t2) const;
		/* get current time in utc ------------------------------------------------ */
		gtime_t *timeget();
		/* set current time in utc ------------------------------------------------ */
		void timeset();
		/* gps time to utc -------------------------------------------------------- */
		gtime_t *gpst2utc();
		/* utc to gps time p------------------------------------------------------- */
		gtime_t *utc2gpst();
		/* gps time to Beidou time ------------------------------------------------ */
		gtime_t *gpst2bdt();
		/* Beidou time to gps time ------------------------------------------------ */
		gtime_t *bdt2gpst();
		/* time to day and sec ---------------------------------------------------- */
		double time2sec(gtime_t &day);
		/* utc to Greenwich mean sidereal time ------------------------------------ */
		double utc2gmst(double ut1_utc);
		/* day of year to time ---------------------------------------------------- */
		int doy2time(int year,int doy);
		/* time to day of year ---------------------------------------------------- */
		double time2doy();
		/* read leap seconds table ------------------------------------------------ */
		int read_leaps(const string file);
		/* adjust time considering week handover ---------------------------------- */
		gtime_t *adjweek(gtime_t t0);
		/* adjust time considering week handover ---------------------------------- */
		gtime_t *adjday(gtime_t t0);
		/* screen data by time ---------------------------------------------------- */
		int screent(gtime_t ts, gtime_t te, double tint);
		/* read leap seconds table by text ---------------------------------------- */
		int read_leaps_text(const string file);
		/* read leap seconds table by usno ---------------------------------------- */
		int read_leaps_usno(const string file);
		/* next download time ----------------------------------------------------- */
		gtime_t *nextdltime(const int *topts,int stat);

		/* copy gtime_t ----------------------------------------------------------- */
		gtime_t *copy_gtime(gtime_t t0);
	/* Components */
	public:
		time_t time;					/* time (s) expressed by standard time_t */
		double sec;						/* fraction of second under 1 s */
		double ep[6];					/* {year,month,day,hour,min,sec} */
		string sep;						/* ep in string format */
		int doy;						/* day of year */
		int sys;						/* time system */
};

#endif