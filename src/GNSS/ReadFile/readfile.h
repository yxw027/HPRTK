/* Read Rinex Files */
/* 
 * 2017-7-22 class inrnx_t as the parent class for the read of all 
 * 			 kinds of input files
 * 
 * 
 */

#ifndef READFILE_H
#define READFILE_H

#include "hprtk_lib.h"
#include "BaseFunction/timesys.h"
#include "GNSS/DataClass/data.h"
#include "ConfigFile/config.h"

/* :structure: signal index type ------------------------------------------------------------------ */
typedef struct {                        /* signal index type */
    int n;                              /* number of index */
    int frq[MAXOBSTYPE];                /* signal frequency (1:L1,2:L2,...) */
    int pos[MAXOBSTYPE];                /* signal index in obs data (-1:no) */
    unsigned char pri [MAXOBSTYPE];     /* signal priority (15-0) */
    unsigned char type[MAXOBSTYPE];     /* type (0:C,1:L,2:D,3:S) */
    unsigned char code[MAXOBSTYPE];     /* obs code (CODE_L??) */
    double shift[MAXOBSTYPE];           /* phase shift (cycle) */
} sigind_t;


/* parent class inrnx_t ------------------------------------------------------------------------------
* O,N,G,H,J,L,C file ------------------------------------------------------------------------------ */
class inrnx_t{
	/* Constructors */
	public:
		inrnx_t();
		inrnx_t(string file, string option);
		virtual ~inrnx_t();
	/* implementation functions */
	protected:
		/* set system mask -------------------------------------------------------- */
		void set_sysmask(prcopt_t *prcopt);
	public:
		/* test oepn of file stream ----------------------------------------------- */
		int test_open();
		/* close file ------------------------------------------------------------- */
		void closeF();
		/* base function of read head --------------------------------------------- */
		int readhead();
		/* vritual function of read head for O,N,G,H,J,L,C ------------------------ */
		virtual int Head();
		/* virtual function of read body for O,N,G,H,J,L,C ------------------------ */
		virtual int Body();				
	/* Components */
	protected:
		ifstream inf;					/* in-stream of O file */
		string opt;						/* options */
		double ver;						/* rinex version */
		string type;					/* type of rinex file */
		int sat_sys;					/* satellite system */
		int tsys;						/* time system */
		gtime_t ts;						/* processing start time */
		gtime_t te;						/* processing end time */
		double tint;					/* processing interval */
		int mask;						/* mask satellite system */
		string buff;					/* read buff */
	public:
		string errmsg;					/* read rinex file error message */
};

/* derived class inrnxO_t ----------------------------------------------------------------------------
* O file - Observation ---------------------------------------------------------------------------- */
class inrnxO_t : public inrnx_t{
	/* Constructors */
	public:
		inrnxO_t();
		inrnxO_t(string file, string option, int rcvnum);
		inrnxO_t(string file, string option, gtime_t TS, gtime_t TE, int TI, int rcvnum);
		~inrnxO_t();

	/* Implementation functions */
	protected:
		/* convert rinex obs type ver.2 -> ver.3 ---------------------------------- */
		void convcode(int sys, string str, string &tp);
		/* save slips ------------------------------------------------------------- */
		void saveslips(unsigned char slips[][NFREQ],  obsd_t &data);
		/* restore slips ---------------------------------------------------------- */
		void restslips(unsigned char slips[][NFREQ],  obsd_t &data);
		/* set signal index ------------------------------------------------------- */
		void set_index(int syt, string tobss[MAXOBSTYPE], sigind_t &ind);
		/* decode obs epoch ------------------------------------------------------- */
		int decode_obsepoch(gtime_t &t, int &flag, vector<int> &sats);
		/* decode obs data -------------------------------------------------------- */
		int decode_obsdata(obsd_t &obs);
		/* read "O" file to obs_t vector ------------------------------------------ */
		int readrnxobsb(int &flag, vector<obsd_t> &data);
	/* inherited functions from class inrnx_t */
	public:
		/* initialization --------------------------------------------------------- */
		void ini_ReadO(string file, string option, gtime_t TS, gtime_t TE, int TI, int rcvnum);
		/* read head of "O" file -------------------------------------------------- */
		virtual int Head(nav_t *nav,sta_t *sta);
		/* read boy of "O" file --------------------------------------------------- */
		virtual int Body(obs_t *obs,prcopt_t *prcopt);
		/* read one epoch body of "O" file ---------------------------------------- */
		virtual int One_Epoch_Body(obs_t *obs,prcopt_t *prcopt);

	/* Components */
	public:
		int rcv;						/* receiver number */
		string tobs[7][MAXOBSTYPE]; 	/* observations' type of all systems */
		sigind_t index[7]={0};			/* observation's signal index of all systems */
};

/* derived class inrnxN_t ----------------------------------------------------------------------------
* N file - Navigation ephemeris -------------------------------------------------------------------- */
class inrnxN_t : public inrnx_t{
	/* Constructors */
	public:
		inrnxN_t();
		inrnxN_t(string file, string option);
		~inrnxN_t();
	/* Implementation functions */
	protected:
		/* ura value (m) to ura index --------------------------------------------- */
		int uraindex(double value);

		/* decode glonass ephemeris ----------------------------------------------- */
		int decode_geph(gtime_t toc,int sat,geph_t *geph);
		/* decode geo ephemeris --------------------------------------------------- */
		int decode_seph(gtime_t toc,int sat,seph_t *seph);
		/* decode ephemeris ------------------------------------------------------- */
		int decode_eph(gtime_t toc,int sat,eph_t *eph);
		/* add data to nav_t ------------------------------------------------------ */
		void addnav(int sys,nav_t *nav);
		/* read "O" file to obs_t vector ------------------------------------------ */
		int readrnxnavb(nav_t *nav);
	public:
		/* initialization --------------------------------------------------------- */
		void ini_ReadN(string file, string option);
		/* read head of "O" file -------------------------------------------------- */
		virtual int Head(nav_t *nav);
		/* read boy of "O" file --------------------------------------------------- */
		virtual int Body(nav_t *nav,prcopt_t *prcopt);
	/* Components */
	public:
		double data[64];				/* one satellite data */
};

/* read earth rotation parameters file (.ERP) ----------------------------------------------------- */
class inerp_t{
	/* Constructors */
	public:
		inerp_t();
		inerp_t(string file);
		~inerp_t();
	/* Implementation functions */
	public:
		/* open file -------------------------------------------------------------- */
		void open_file();
		/* read earth rotation parameters file ------------------------------------ */
		int readerp(erp_t *erp);
	/* Components */
	protected:
		string file_path;				/* erp file path */
		ifstream inf;					/* in-stream of erp file */
};

/* read ocean-loading tide file (.BLQ) ------------------------------------------------------------ */
class inblq_t{
	/* Constructors */
	public:
		inblq_t();
		inblq_t(string file);
		~inblq_t();
	/* Implementation functions */
	public:
		/* open file -------------------------------------------------------------- */
		void open_file();
		/* read earth rotation parameters file ------------------------------------ */
		int readblq(const string staname,double *ocean_par);
	/* Components */
	protected:
		string file_path;				/* erp file path */
		ifstream inf;					/* in-stream of erp file */
};

/* read antenna information file (.ATX) ----------------------------------------------------------- */
class inatx_t{
	/* Constructors */
	public:
		inatx_t();
		inatx_t(string file);
		~inatx_t();
	/* Implementation functions */
	public:
		/* open file -------------------------------------------------------------- */
		void open_file();
		/* read earth rotation parameters file ------------------------------------ */
		int readatx();
	/* Components */
	protected:
		string file_path;				/* atx file path */
		ifstream inf;					/* in-stream of atx file */
		string buff;
	public:
		vector<pcv_t> pcv;				/* pcv vector */
};

/* read precise ephemeris file -------------------------------------------------------------------- */
class ineph_t{
	/* Constructors */
	public:
		ineph_t();
		ineph_t(string file,int pred);
		~ineph_t();
	/* Implementation functions */
	protected:
		/* set system mask -------------------------------------------------------- */
		void set_sysmask(const prcopt_t *prcopt);
		/* read precise ephemeris file header ------------------------------------- */
		int Head();
		/* read precise ephemeris file body --------------------------------------- */
		int Body(nav_t *nav);
	public:
		/* initialization --------------------------------------------------------- */
		void ini_readEph(string file,int pred);
		/* open file -------------------------------------------------------------- */
		void open_file();
		/* read precise ephemeris file -------------------------------------------- */
		int readsp3(nav_t *nav,const prcopt_t *opt);
	/* Components */
	protected:
		char type;						/* precise ephemeris type */
		string satellite[MAXSAT];		/* satellites flag */
		int ns;							/* number of satellites */
		double bfact[2];				/* base fact for pos/vel and clock bias */
		string tsys;					/* time system */
		string buff;					/* stream buff */
		int mask;						/* mask satellite system */
		string opt;						/* options */
		int pred_flag;					/* flag of use predicted data 
										 * 1: only observed + 2: only predicted + 4: not combined */
	public:
		gtime_t ephtime;				/* precise ephemeris time */
		string file_path;				/* sp3/eph file path */
		ifstream inf;					/* in-stream of sp3 file */
};

/* read ionex tec grid file ----------------------------------------------------------------------- */
class inionex_t{
	/* Constructors */
	public:
		inionex_t();
		inionex_t(string file);
		~inionex_t();
	/* Implementation functions */
	protected:
		/* data index (i:lat,j:lon,k:hgt) ----------------------------------------- */
		int dataindex(int i, int j, int k, const int *ndata);

		/* read P1P2 DCB ---------------------------------------------------------- */
		void P1P2DCB(nav_t *nav);
		/* read head of ionex tec grid file --------------------------------------- */
		int Head(nav_t *nav);
		/* add one epoch tec map data to nav_t ------------------------------------ */
		tec_t* addtec2nav(nav_t *nav);
		/* read body of ionex tec grid file --------------------------------------- */
		int Body(nav_t *nav);
	public:
		/* initialization --------------------------------------------------------- */
		void ini_rdIonex(string file);
		/* open file -------------------------------------------------------------- */
		void open_file();
		/* read ionex tec grid file ----------------------------------------------- */
		int readIonex(nav_t *nav);
	/* Components */
	protected:
		double tec_factor;				/* vtec factor */
		string buff;					/* stream buff */
		gtime_t iontime;
	public:
		double lats[3],lons[3],hgts[3];	/* start/end/interval of lat lon hight */
		double REarth;					/* radius of Earth */
		double version;					/* version of ionex file */

		string file_path;				/* ionex tec grid file */
		ifstream inf;					/* in-stream of ionex file */
};
#endif