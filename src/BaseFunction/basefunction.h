/* Base functions for calculation -----------------------------------------------------------------
* options : -DLAPACK   use LAPACK/BLAS
*           -DMKL      use Intel MKL
*           -DWIN32    use WIN32 API
*           -DNOCALLOC no use calloc for zero matrix
*           -DIERS_MODEL use GMF instead of NMF
*           -DDLL      built for shared library
*           -DCPUTIME_IN_GPST cputime operated in gpst
*
* references :
*     [1] IS-GPS-200D, Navstar GPS Space Segment/Navigation User Interfaces,
*         7 March, 2006
*     [2] RTCA/DO-229C, Minimum operational performanc standards for global
*         positioning system/wide area augmentation system airborne equipment,
*         RTCA inc, November 28, 2001
*     [3] M.Rothacher, R.Schmid, ANTEX: The Antenna Exchange Format Version 1.4,
*         15 September, 2010
*     [4] A.Gelb ed., Applied Optimal Estimation, The M.I.T Press, 1974
*     [5] A.E.Niell, Global mapping functions for the atmosphere delay at radio
*         wavelengths, Jounal of geophysical research, 1996
*     [6] W.Gurtner and L.Estey, RINEX The Receiver Independent Exchange Format
*         Version 3.00, November 28, 2007
*     [7] J.Kouba, A Guide to using International GNSS Service (IGS) products,
*         May 2009
*     [8] China Satellite Navigation Office, BeiDou navigation satellite system
*         signal in space interface control document, open service signal B1I
*         (version 1.0), Dec 2012
*     [9] J.Boehm, A.Niell, P.Tregoning and H.Shuh, Global Mapping Function
*         (GMF): A new empirical mapping function base on numerical weather
*         model data, Geophysical Research Letters, 33, L07304, 2006
*     [10] GLONASS/GPS/Galileo/Compass/SBAS NV08C receiver series BINR interface
*         protocol specification ver.1.3, August, 2012
*
--------------------------------------------------------------------------------------------------- */
#ifndef BASEFUNCTION_H
#define BASEFUNCTION_H

#include "hprtk_lib.h"
#include "GNSS/DataClass/data.h"
#include "BaseFunction/timesys.h"

/* constants -------------------------------------------------------------------------------------- */
#define POLYCRC32   0xEDB88320u /* CRC32 polynomial */
#define POLYCRC24Q  0x1864CFBu  /* CRC24Q polynomial */
/* chi-sqr(n) (alpha=0.001) ------------------------------------------------------- */
const double ChiSqures[100] ={
	10.8,13.8,16.3,18.5,20.5,22.5,24.3,26.1,27.9,29.6,
	31.3,32.9,34.5,36.1,37.7,39.3,40.8,42.3,43.8,45.3,
	46.8,48.3,49.7,51.2,52.6,54.1,55.5,56.9,58.3,59.7,
	61.1,62.5,63.9,65.2,66.6,68.0,69.3,70.7,72.1,73.4,
	74.7,76.0,77.3,78.6,80.0,81.3,82.6,84.0,85.4,86.7,
	88.0,89.3,90.6,91.9,93.3,94.7,96.0,97.4,98.7,100 ,
	101 ,102 ,103 ,104 ,105 ,107 ,108 ,109 ,110 ,112 ,
	113 ,114 ,115 ,116 ,118 ,119 ,120 ,122 ,123 ,125 ,
	126 ,127 ,128 ,129 ,131 ,132 ,133 ,134 ,135 ,137 ,
	138 ,139 ,140 ,142 ,143 ,144 ,145 ,147 ,148 ,149
};
/* carrier wave length (m) -------------------------------------------------------- */
const double WaveLengths[MAXFREQ] ={
	CLIGHT/FREQ1 ,CLIGHT/FREQ2, CLIGHT/FREQ5, CLIGHT/FREQ6, CLIGHT/FREQ7,
	CLIGHT/FREQ8, CLIGHT/FREQ9
};
/* stream format strings ---------------------------------------------------------- */
const string FormatStrs[32] ={
	"RTCM 2",                   /*  0 */
	"RTCM 3",                   /*  1 */
	"NovAtel OEM6",             /*  2 */
	"NovAtel OEM3",             /*  3 */
	"u-blox",                   /*  4 */
	"Superstar II",             /*  5 */
	"Hemisphere",               /*  6 */
	"SkyTraq",                  /*  7 */
	"GW10",                     /*  8 */
	"Javad",                    /*  9 */
	"NVS BINR",                 /* 10 */
	"BINEX",                    /* 11 */
	"Trimble RT17",             /* 12 */
	"Septentrio",               /* 13 */
	"CMR/CMR+",                 /* 14 */
	"LEX Receiver",             /* 15 */
	"RINEX",                    /* 16 */
	"SP3",                      /* 17 */
	"RINEX CLK",                /* 18 */
	"SBAS",                     /* 19 */
	"NMEA 0183",                /* 20 */
	""
};
/* functions -------------------------------------------------------------------------------------- */
/* decoded binary data to ASCII data -------------------------------------------------------------- */
/* extract unsigned/signed bits --------------------------------------------------- */
unsigned int getbitu(const unsigned char *ChBuff,int BitPos,int BitLen);
int getbits(const unsigned char *ChBuff,int BitPos,int BitLen);
/* set unsigned/signed bits ------------------------------------------------------- */
void setbitu(unsigned char *ChBuff,int BitPos,int BitLen,unsigned int ByteData);
/* crc-24q parity ----------------------------------------------------------------- */
unsigned int rtk_crc24q(const unsigned char *ChBuff,int BitLen);
/* crc-16 parity ------------------------------------------------------------------ */
unsigned short rtk_crc16(const unsigned char *ChBuff,int BitLen);
/* decode navigation data word ---------------------------------------------------- */
int decode_word(unsigned int NavWord,unsigned char *ChData);

/* math (vector, matrix) functions ---------------------------------------------------------------- */
/* initialize identity matrix --------------------------------------------------------
* args   : double *VecA          I   matrix to be processed
*          int    SizeA          I   dimension of VecA
* --------------------------------------------------------------------------------- */
template <typename Iter>
void eyemat(Iter VecA,const int dimA){
	for (int i=0; i< dimA; i++)
		for (int j=0; j<dimA; j++)
			VecA[j+i*dimA]= i==j ? 1.0 : 0.0;
}
/* sum of vector and matrix ----------------------------------------------------------
* args   : double *VecA          I   vector a (n x 1)
*          int    SizeA          I   size of vector a
* return : sum of VecA
* --------------------------------------------------------------------------------- */
template <typename Iter>
double sum(const Iter VecA, const int SizeA){
	double S=0.0;
	for (int i=0; i<SizeA; i++){
		S+=VecA[i];
	}
	return S;
}
/* average of vector -------------------------------------------------------------- */
template <typename Iter>
double ave_vec(const Iter VecA, const int SizeA){
	double S=0.0,num=0;
	for (int i=0; i<SizeA; i++){
		if (VecA[i]!=0.0) { 
			S+=VecA[i]; num+=1; 
		}
	}
	return S/num;
}
/* inner product ---------------------------------------------------------------------
* inner product of vectors
* args   : double *VecA,*VecB     I   vector a,b (n x 1)
*          int    SizeVec         I   size of vector a,b
* return : VecA'*VecB
* --------------------------------------------------------------------------------- */
template <typename Iter1,typename Iter2>
double dot(const Iter1 VecA,const Iter2 VecB,int SizeVec){
	double dInn=0.0;

	while (--SizeVec>=0) dInn+=VecA[SizeVec]*VecB[SizeVec];
	return dInn;
}
/* euclid norm -----------------------------------------------------------------------
* euclid norm of vector
* args   : double *VecA        I   vector a (n x 1)
*          int    SizeVec      I   size of vector a
* return : || VecA ||
*---------------------------------------------------------------------------------- */
template <typename Iter>
double norm(const Iter VecA,int SizeVec){
	return sqrt(dot(VecA,VecA,SizeVec));
}
/* geometrical distance of 2 vectors -------------------------------------------------
* args   : double *VecA,*VecB  I   vector A,B (SizeAB * 1)
         : int    SizeAB       I   size of vector A,B
* return : distance between VecA and VecB
* --------------------------------------------------------------------------------- */
template <typename Iter1, typename Iter2>
double distance(const Iter1 VecA, const Iter2 VecB, int SizeAB){
	vector<double> vecAB (SizeAB,0.0);
	for (int i=0; i<SizeAB; i++)
		vecAB[i] = VecA[i] - VecB[i];
	return norm(vecAB.begin(),SizeAB);
}
/* normalize 3d vector ------------------------------------------------------------ */
template <typename Iter1,typename Iter2>
int normv3(const Iter1 VecA,Iter2 VecB){
	double r;
	if ((r=norm(VecA,3))<=0.0) return 0;
	VecB[0]=VecA[0]/r;
	VecB[1]=VecA[1]/r;
	VecB[2]=VecA[2]/r;
	return 1;
}
/* outer product of 3d vectors ---------------------------------------------------- */
template <typename Iter1,typename Iter2,typename Iter3>
void cross3(const Iter1 VecA,const Iter2 VecB,Iter3 VecC){
	VecC[0]= VecA[1]*VecB[2] - VecA[2]*VecB[1];
	VecC[1]= VecA[2]*VecB[0] - VecA[0]*VecB[2];
	VecC[2]= VecA[0]*VecB[1] - VecA[1]*VecB[0];
}
/* trace of matrix ---------------------------------------------------------------- */
template <typename Iter>
double matrace(const Iter VecA, const int SizeA){
	double trace=0.0;
	for (int i=0; i<SizeA; i++){
		trace+=VecA[i+i*SizeA];
	}
	return trace;
}
/* copy matrix -------------------------------------------------------------------- */
template <typename Iter1,typename Iter2>
void matcpy(Iter1 MatDst,const Iter2 MatSrc,int n,int m){
	for (int i=0; i<n*m; i++) MatDst[i]= MatSrc[i];
}
/* multiply matrix -------------------------------------------------------------------
*   1 NN : A(sizeAB,sizeA) * B(sizeB,sizeAB)
*   2 NT : A(sizeAB,sizeA) * B(sizeAB,sizeB)
*   3 TN : A(sizeA,sizeAB) * B(sizeB,sizeAB)
*   4 TT : A(sizeA,sizeAB) * B(sizeAB,sizeB)
* result : MatC(sizeB,sizeA)
* comment: colume matrix multiplication (contrary to the normal linear algebra)
* --------------------------------------------------------------------------------- */
void matmul_pnt(const char *TraFlag,int SizeA,int SizeB,int SizeAB,double CoeAB,
	const double *MatA,const double *MatB,double CoeC,double *MatC);
void matmul_vec(const char *TraFlag,int SizeA,int SizeB,int SizeAB,double CoeAB,
	const vector<double> &MatA,const vector<double> &MatB,double CoeC,vector<double> &MatC);
void matmul_vec(const char *TraFlag,int SizeA,int SizeB,int SizeAB,double CoeAB,
	const vector<double> &MatA,const vector<double> &MatB,double CoeC,vector<double> &MatC,
	const int StartA, const int StartB, const int StartC);
template <typename Iter1,typename Iter2,typename Iter3>
void matmul(const char *TraFlag,int SizeA,int SizeB,int SizeAB,double CoeAB,
	const Iter1 MatA,const Iter2 MatB,double CoeC,Iter3 MatC) {
	double LineAB;
	int i,j,x,f=TraFlag[0]=='N' ? (TraFlag[1]=='N' ? 1 : 2) : (TraFlag[1]=='N' ? 3 : 4);

	for (i=0; i<SizeA; i++) for (j=0; j<SizeB; j++) {
		LineAB=0.0;
		switch (f) {
		case 1: for (x=0; x<SizeAB; x++) LineAB+= MatA[i+x*SizeA]  *MatB[x+j*SizeAB]; break;
		case 2: for (x=0; x<SizeAB; x++) LineAB+= MatA[i+x*SizeA]  *MatB[j+x*SizeB]; break;
		case 3: for (x=0; x<SizeAB; x++) LineAB+= MatA[x+i*SizeAB] *MatB[x+j*SizeAB]; break;
		case 4: for (x=0; x<SizeAB; x++) LineAB+= MatA[x+i*SizeAB] *MatB[j+x*SizeB]; break;
		}
		if (CoeC==0.0) MatC[i+j*SizeA] = CoeAB*LineAB;
		else MatC[i+j*SizeA] = CoeAB*LineAB + CoeC*MatC[i+j*SizeA];
	}
}
/* (static) LU decomposition ------------------------------------------------------ */
int LUdcmp(vector<double> &MatSrc,int Size,vector<int> &index);
/* LU back-substitution ----------------------------------------------------------- */
void LUbksb(vector<double> &MatB,int Size,vector<int> &index,vector<double> &MatA,int startA);
/* inverse of matrix -------------------------------------------------------------- */
int matinv(vector<double> &MatSrc,int Size);
/* polynomial coefficients estimate ----------------------------------------------- */
int polyest(const vector<double> &A,const vector<double> &L,vector<double> &Coe,
	const int numL,const int numX,double &sigma);
/* solve linear equation ---------------------------------------------------------- */
int solve_line(const string TransFlag,const vector<double> &A,const vector<double> &Y,
	vector<double> &X,int Xnum,int SolNum);
/* get index ---------------------------------------------------------------------- */
int getindex(double value, const double *range);
/* get number of items ------------------------------------------------------------ */
int nitem(const double *range);

/* data format transfer functions ----------------------------------------------------------------- */
/* replace keywords in file path -------------------------------------------------- */
int reppath(const string SrcPath, string &DstPath, gtime_t GpsTime, 
	const string RovID, const string BaseID);
/* convert double number to string ------------------------------------------------ */
string doul2str(int StrLen,int DecLen,const string StrFiller,const double SrcNum,string &DstStr);
/* convert int number to string --------------------------------------------------- */
string int2str(int StrLen,const string StrFiller,const int SrcNum,string &Dststr);
/* convert string to double number ------------------------------------------------ */
int str2double(const string SrcStr, double &DstNum);
/* convert string to int number --------------------------------------------------- */
int str2int(const string SrcStr, int &DstNum);
/* convert between matrix and vector, Iter1 to Iter2 ------------------------------ */
template <typename Iter1,typename Iter2>
int vecarr(const Iter1 VecArr1,Iter2 VecArr2,int SizeVA){
	while (--SizeVA>=0) VecArr1[SizeVA] = VecArr2[SizeVA];
	return 1;
}

/* time and position transfer functions ----------------------------------------------------------- */
/* get tick time ------------------------------------------------------------------ */
unsigned int tickget();
/* sleep ms ----------------------------------------------------------------------- */
void sleepms(int ms);
/* adjust gps week number --------------------------------------------------------- */
int adjgpsweek(int UnWeek);
/* convert degree to deg-min-sec -----------------------------------------------------
* convert degree to degree-minute-second
* args   : double  Degree       I   degree
*          double *DMS          O   degree-minute-second {deg,min,sec}
*          int     NumDec       I   number of decimals of second
* return : none
*---------------------------------------------------------------------------------- */
template <typename Iter1>
void deg2dms(const double Degree,Iter1 DMS,int NumDec)
{
	double sign=Degree<0.0 ? -1.0 : 1.0,a=fabs(Degree);
	double unit=pow(0.1,NumDec);
	DMS[0]=floor(a); a=(a-DMS[0])*60.0;
	DMS[1]=floor(a); a=(a-DMS[1])*60.0;
	DMS[2]=floor(a/unit+0.5)*unit;
	if (DMS[2]>=60.0) {
		DMS[2]=0.0;
		DMS[1]+=1.0;
		if (DMS[1]>=60.0) {
			DMS[1]=0.0;
			DMS[0]+=1.0;
		}
	}
	DMS[0]*=sign;
}
/* convert deg-min-sec to degree -----------------------------------------------------
* convert degree-minute-second to degree
* args   : double *DMS      I   degree-minute-second {deg,min,sec}
* return : degree
*---------------------------------------------------------------------------------- */
template <typename Iter1>
double dms2deg(const Iter1 DMS)
{
	double sign=DMS[0]<0.0 ? -1.0 : 1.0;
	return sign*(fabs(DMS[0])+DMS[1]/60.0+DMS[2]/3600.0);
}
/* ecef to local coordinate transfromation matrix ------------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *XyzPos      I   geodetic position {lat,lon} (rad)
*          double *TranMat     O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*---------------------------------------------------------------------------------- */
template <typename Iter1,typename Iter2>
void xyz2enu(const Iter1 BlhPos,Iter2 TranMat){
	double sinLat=sin(BlhPos[0]),cosLat=cos(BlhPos[0]),sinLon=sin(BlhPos[1]),cosLon=cos(BlhPos[1]);

	TranMat[0]=-sinLon;			TranMat[3]=cosLon;			TranMat[6]=0.0;
	TranMat[1]=-sinLat*cosLon;	TranMat[4]=-sinLat*sinLon;	TranMat[7]=cosLat;
	TranMat[2]=cosLat*cosLon;	TranMat[5]=cosLat*sinLon;	TranMat[8]=sinLat;
}
/* transform ecef to geodetic postion ------------------------------------------------
* transform ecef position to geodetic position
* args   : double *XyzPos        I   ecef position {x,y,z} (m)
*          int     DatumSys      I   datum {0:WGS84,1:CGCS2000}
*          double *BlhPos        O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height
Iter1/Iter2 is a double pointer or a vector<double> iterator
*---------------------------------------------------------------------------------- */
template <typename Iter1,typename Iter2>
void ecef2pos(const Iter1 XyzPos,int DatumSys,Iter2 BlhPos)
{
	double re,fe;
	if (DatumSys==CGCS2000) { re=RE_CGCS2000; fe=EE_CGCS2000; }
	else { re=RE_WGS84; fe=EE_WGS84; }
	double e2=fe*(2.0-fe),r2=dot(XyzPos,XyzPos,2),z,zk,v=re,sinp;

	for (z=XyzPos[2],zk=0.0; fabs(z-zk)>=1E-4;) {
		zk=z;
		sinp=z/sqrt(r2+z*z);
		v=re/sqrt(1.0-e2*sinp*sinp);
		z=XyzPos[2]+v*e2*sinp;
	}
	BlhPos[0]=r2>1E-12 ? atan(z/sqrt(r2)) : (XyzPos[2]>0.0 ? PI/2.0 : -PI/2.0);
	BlhPos[1]=r2>1E-12 ? atan2(XyzPos[1],XyzPos[0]) : 0.0;
	BlhPos[2]=sqrt(r2+z*z)-v;
}
/* transform ecef vector to local tangental coordinate ---------------------------- */
template <typename Iter1,typename Iter2,typename Iter3>
void ecef2enu(const Iter1 BlhPos,const Iter2 SightVec,Iter3 EnuPos){
	double E[9];

	xyz2enu(BlhPos,E);
	matmul("NN",3,1,3,1.0,E,SightVec,0.0,EnuPos);
}
/* transform local vector to ecef coordinate -----------------------------------------
* transform local tangental coordinate vector to ecef
* args   : double *BlhPos      I   geodetic position {lat,lon} (rad)
*          double *EnuPos      I   vector in local tangental coordinate {e,n,u}
*          double *XyzVec      O   difference vector in ecef coordinate {x,y,z}
* return : none

*---------------------------------------------------------------------------------- */
template <typename Iter1,typename Iter2,typename Iter3>
void enu2ecef(const Iter1 BlhPos,const Iter2 EnuPos,Iter3 XyzVec) {
	double E[9];

	xyz2enu(BlhPos,E);
	matmul("TN",3,1,3,1.0,E,EnuPos,0.0,XyzVec);
}
/* transform covariance to local tangental coordinate --------------------------------
* transform ecef covariance to local tangental coordinate
* args   : double *BlhPos      I   geodetic position {lat,lon} (rad)
*          double *XyzVar      I   covariance in ecef coordinate
*          double *BlhVar      O   covariance in local tangental coordinate
* return : none
*---------------------------------------------------------------------------------- */
template <typename Iter1,typename Iter2,typename Iter3>
void covenu(const Iter1 BlhPos,const Iter2 XyzVar,Iter3 BlhVar)
{
	double E[9],EP[9];

	xyz2enu(BlhPos,E);
	matmul("NN",3,3,3,1.0,E,XyzVar,0.0,EP);
	matmul("NT",3,3,3,1.0,EP,E,0.0,BlhVar);
}
/* transform covariance to local tangental coordinate --------------------------------
* transform ecef covariance to local tangental coordinate
* args   : double *BlhPos      I   geodetic position {lat,lon} (rad)
*          double *BlhVar      I   covariance in local tangental coordinate
*          double *XyzVar      O   covariance in xyz coordinate
* return : none
*---------------------------------------------------------------------------------- */
template <typename Iter1,typename Iter2,typename Iter3>
void covecef(const Iter1 BlhPos,const Iter2 BlhVar,Iter3 XyzVar)
{
	double E[9],EQ[9];

	xyz2enu(BlhPos,E);
	matmul("TN",3,3,3,1.0,E,BlhVar,0.0,EQ);
	matmul("NN",3,3,3,1.0,EQ,E,0.0,XyzVar);
}
/* transform geodetic to ecef position -----------------------------------------------
* transform geodetic position to ecef position
* args   : double *BlhPos      I   geodetic position {lat,lon,h} (rad,m)
*          int     DatumSys    I   datum {0:WGS84,1:CGCS2000}
*          double *XyzPos      O   ecef position {x,y,z} (m)
* return : none
* notes  : WGS84, ellipsoidal height
Iter1/Iter2 is a double pointer or a vector<double> iterator
*---------------------------------------------------------------------------------- */
template <typename Iter1,typename Iter2>
void pos2ecef(const Iter1 BlhPos,int DatumSys,Iter2 XyzPos)
{
	double re,fe;
	if (DatumSys==CGCS2000) { re=RE_CGCS2000; fe=EE_CGCS2000; }
	else { re=RE_WGS84; fe=EE_WGS84; }
	double sinLat=sin(BlhPos[0]),cosLat=cos(BlhPos[0]),sinLon=sin(BlhPos[1]),cosLon=cos(BlhPos[1]);
	double e2=fe*(2.0-fe),v=re/sqrt(1.0-e2*sinLat*sinLat);

	XyzPos[0]=(v+BlhPos[2])*cosLat*cosLon;
	XyzPos[1]=(v+BlhPos[2])*cosLat*sinLon;
	XyzPos[2]=(v*(1.0-e2)+BlhPos[2])*sinLat;
}
/* geometric distance ------------------------------------------------------------- */
template <typename Iter1,typename Iter2,typename Iter3>
double geodist(const Iter1 SatPos,const Iter2 RecPos,Iter3 SightVec){
	double r;
	int i;

	if (norm(SatPos,3)<RE_WGS84) return -1.0;
	for (i=0; i<3; i++) SightVec[i]=SatPos[i]-RecPos[i];
	r=norm(SightVec,3);
	for (i=0; i<3; i++) SightVec[i]/=r;
	return r+OMGE*(SatPos[0]*RecPos[1] - SatPos[1]*RecPos[0])/CLIGHT;
}
/* satellite azimuth/elevation angle ---------------------------------------------- */
template <typename Iter1,typename Iter2,typename Iter3>
double satazel(const Iter1 BlhPos,const Iter2 SightVec,Iter3 AzEl){
	double az=0.0,el=PI/2.0,enu[3];

	if ((BlhPos[2])>-RE_WGS84) {
		ecef2enu(BlhPos,SightVec,enu);
		az=dot(enu,enu,2)<1E-12 ? 0.0 : atan2(enu[0],enu[1]);
		if (az<0.0) az+=2*PI;
		el=asin(enu[2]);
	}
	AzEl[0]=az; AzEl[1]=el;
	return el;
}

/* geography and astronomy functions -------------------------------------------------------------- */
/* astronomical arguments: f={l,l',F,D,OMG} (rad) --------------------------------- */
void ast_args(double t,double *f);
/* eci to ecef transformation matrix ---------------------------------------------- */
void eci2ecef(gtime_t tutc,const double *erpv,double *U,double *gmst);
/* sun and moon position ---------------------------------------------------------- */
void sunmoonpos(gtime_t UT1Time,double *ERPValue,double *SunPos,
	double *MoonPos,double *gmst);
/* get earth rotation parameter values -------------------------------------------- */
int geterpv(const erp_t *Earth_Par,gtime_t GPS_Time,double *Earth_Value);

/* satellite data functions ----------------------------------------------------------------------- */
/* satellite system+prn/slot number to satellite number --------------------------- */
int satno(int StaSys,int PrnNum);
/* satellite number to satellite system ------------------------------------------- */
int satsys(int SatNum,int *PrnNum);
/* satellite id to satellite number ----------------------------------------------- */
int satid2no(string SatID);
/* satellite number to satellite id ----------------------------------------------- */
int satno2id(int sat,string &SatID);
/* satellite carrier wave length -------------------------------------------------- */
double satwavelen(int SatNum,int FrqNum,const nav_t *NavData);

/* observation and code transfer functions -------------------------------------------------------- */
/* obs type string to obs code ---------------------------------------------------- */
unsigned char obs2code(string ObsCode,int *ObsFre);
/* obs code to obs code string ---------------------------------------------------- */
string code2obs(unsigned char ObsCode,int *ObsFre);
/* satellite code to satellite system --------------------------------------------- */
int code2sys(char SysCode);
/* get code priority -------------------------------------------------------------- */
int getcodepri(int SatSys,unsigned char ObsCode,string CodeOpt);
/* add fatal callback function ---------------------------------------------------- */
void add_fatal(fatalfunc_t *func);

/* GPS data functions ----------------------------------------------------------------------------- */
/* arrange observation data ------------------------------------------------------- */
int sortobs(obs_t &SrcObs);
/* station-cross doppler single-difference ---------------------------------------- */
double dopsingle_d(const obsd_t *rov,const obsd_t *bas,const int fre);
/* station-cross single-difference observation ------------------------------------ */
double single_diff(const obsd_t *rov,const obsd_t *bas,const int fre);
/* compute ionosphere-free combination -------------------------------------------- */
double iono_free(const int fres,const obsd_t *rov,const obsd_t *bas,const double *lam);
/* compute geometry-free combination ---------------------------------------------- */
double geometry_free(const int fres,const obsd_t *rov,const obsd_t *bas, const double *lam);
/* compute Melbourne-Wubbena combination ------------------------------------------ */
double Mel_Wub(const int fres,const obsd_t *rov,const obsd_t *bas,const double *lam);
/* compute narrow-lane ambiguity -------------------------------------------------- */
double Narrow(const int fres,const obsd_t *rov,const obsd_t *bas,const double *lam);
/* compute ambiguity combination -------------------------------------------------- */
double amb_cmb(const double Lr,const double P);
/* compute single time-cross difference -------------------------------------------------- */
double single_time(const double obs_t1,const double obs_t2);

/* system functions ------------------------------------------------------------------------------- */
/* execute command ---------------------------------------------------------------- */
int execcmd(const string StrCmd);
/* create directory --------------------------------------------------------------- */
void createdir(const string StrPath);
/* uncompress file ---------------------------------------------------------------- */
int rtk_uncompress(const string SrcFile,string UncFile);

#endif