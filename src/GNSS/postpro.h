#ifndef POSTPRO_H
#define POSTPRO_H

#include "GNSS/rtkpro.h"
#include "GNSS/ReadFile/readfile.h"

/* post-processing server type -------------------------------------------------------------------- */
class postsvr_t{
	/* Constructor */
	public:
		postsvr_t();
		~postsvr_t();
	/* Implementation functions */
	protected:
		/* set sat\rcv antenna information ---------------------------------------- */
		void setpcv(inatx_t *atx,int satrcv);
		/* new rtk according to opt ----------------------------------------------- */
		void inirtk(prcopt_t *Prcopt,filopt_t *Filopt);
		/* update navigation data ------------------------------------------------- */
		void updatenav();
		/* update station information to prcopt ----------------------------------- */
		void updatesta();

		/* initialize read stream ------------------------------------------------- */
		int ini_Read_Stream();
		/* rover/base observation synchronization --------------------------------- */
		int obs_synchron();

		/* write solution header to output stream --------------------------------- */
		void writesolhead();
		/* write solution to output stream ---------------------------------------- */
		void writesol();
		/* write solution to each solstream --------------------------------------- */
		void writesolstr(int index);
	public:
		/* initialize postsvr ----------------------------------------------------- */
		int postsvrini(option_t *option);
		/* read Navigation file --------------------------------------------------- */
		int readNav();
		/* post-position epoch by epoch ------------------------------------------- */
		int Post_Position_Epoch();
	/* Components */
	protected:
		rtk_t *rtk;						/* RTK control/result struct */
		obs_t obs[2];					/* observation data {0:rover,1:base} */
		prcopt_t *prcopt;				/* processing options */
		pstopt_t *pstopt;				/* post-procssing options */
		solopt_t solopt[2];				/* solution options */
		file_t solstream[2];			/* solution output stream */
		string rec_ant;					/* receiver antenna file path */
		string blq_sta;					/* station ocean-loading tide file path */
		inrnxO_t rover,base;			/* read observation file stream */
		inrnxN_t readN;					/* read navigation file stream */
		ineph_t readEph;				/* read precise ephemeris stream */
	public:
		nav_t *nav;						/* navigation data */
};

#endif
