#ifndef DECODE_H
#define DECODE_H

#include "GNSS/DataClass/data.h"
#include "BaseFunction/timesys.h"
#include "GNSS/rtkpro.h"

/* decode data for kinds of formats --------------------------------------------------------------- */
class decode_data{
	/* Consstructor */
	public:
		decode_data();
		virtual ~decode_data();
	/* Virtual Implementation functions */
	public:
		virtual int decode(unsigned char data);
	/* Components */
	public:
		gtime_t time;					/* message time */
		obs_t obs;						/* observation data */
		nav_t nav;						/* satellite ephemerides */
		sta_t sta;						/* station parameters */
		lexmsg_t lexmsg;				/* LEX message */
		sbsmsg_t sbsmsg;				/* SBAS message */
		ssr_t ssr[MAXSAT];				/* output of ssr corrections */
		string opt;						/* receiver dependent options */
		int ephsat;						/* sat number of update ephemeris (0:no satellite) */
		int format;						/* receiver stream format (only for raw_t) */
		dgps_t *dgps;					/* output of dgps corrections (only for rtcm2) */
		rtksvr_t  *Svr;					/* Pointer to RTK server structure
									(when running in that environment otherwise NULL) */
};

#endif
