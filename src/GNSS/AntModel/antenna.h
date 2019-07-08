#ifndef ANTENNA_H
#define ANTENNA_H

#include "GNSS/DataClass/data.h"
#include "ConfigFile/config.h"

/* satellite antenna phase center offest ---------------------------------------------------------- */
class satantenna_t{
	/* Constructor */
	public:
		satantenna_t();
		~satantenna_t();
	/* Implementation functions */
	protected:
	public:
		/* satellite antenna phase center offest for one obsd_t ------------------- */
		void satantoff(obsd_t *data,const nav_t *nav);
};

/* receiver antenna phase center offest ----------------------------------------------------------- */
class recantenna_t{
	/* Constructor */
	public:
		recantenna_t();
		~recantenna_t();
	/* Implementation functions */
	protected:
	public:
		/* receiver antenna phase center offest for one obsd_t -------------------- */
		void recantoff(const prcopt_t *opt,int rovbas,obsd_t *data);
};
#endif