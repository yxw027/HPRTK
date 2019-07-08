/* Read Rinex Files */
/* 
 * 2017-7-22 class inrnx_t as the parent class for the read of all 
 * 			 kinds of input files
 * 
 * */
#include "GNSS/ReadFile/readfile.h"
#include "BaseFunction/basefunction.h"

 /* constant -------------------------------------------------------------------------------------- */
#define SQR(x)      ((x)*(x))

#define NUMSYS      6                   /* number of systems */
#define MAXRNXLEN   (16*MAXOBSTYPE+4)   /* max rinex record length */
#define MAXPOSHEAD  1024                /* max head line position */
#define MINFREQ_GLO -7                  /* min frequency number glonass */
#define MAXFREQ_GLO 13                  /* max frequency number glonass */
#define NINCOBS     262144              /* inclimental number of obs data */

const sigind_t csigind={ 0 };

static const int navsys[] = {             /* satellite systems */
	SYS_GPS,SYS_GLO,SYS_GAL,SYS_QZS,SYS_SBS,SYS_CMP,SYS_IRN,0
};
static const string syscodes = "GREJSCI";	/* satellite system codes */
static const string OBSCODES = "CLDS";    /* obs type codes */
static const string frqcodes = "1256789"; /* frequency codes */

static const double ura_eph[] = {         /* ura values (ref [3] 20.3.3.3.1.1) */
	2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
	3072.0,6144.0,0.0
};
static const double ura_nominal[] = {     /* ura nominal values */
	2.0,2.8,4.0,5.7,8.0,11.3,16.0,32.0,64.0,128.0,256.0,512.0,1024.0,
	2048.0,4096.0,8192.0
};

/* functions of parent class "inrnx_t" ---------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
inrnx_t::inrnx_t(){
	opt="";
	ver=2.10;
	ver=2.10;
	ts=gtime_t();
	te=gtime_t();
	tint=0.0;
	mask=SYS_NONE;
}
inrnx_t::inrnx_t(string file, string option){
	inf.open(file,ios::in);
	opt=option;
	ver=2.10;
	ts=gtime_t();
	te=gtime_t();
	tint=0.0;
}
inrnx_t::~inrnx_t() {
	inf.close();
}
/* Implementation functions ------------------------------------------------------- */
/* set system mask ---------------------------------------------------------------- */
void inrnx_t::set_sysmask(prcopt_t *prcopt){
	int i,f;
	string sys;
	
	f=opt.find("-SYS=");
	if (f==string::npos) { 
		if (prcopt) mask=prcopt->navsys; return;
		mask=SYS_ALL; return;
	}
		
	sys=opt.substr(f+5);
	for (i=0;i<(int)(sys.length())&&sys[i]!=' ';i++){
		switch (sys[i]){
			case 'G': mask|=SYS_GPS; break;
			case 'R': mask|=SYS_GLO; break;
			case 'E': mask|=SYS_GAL; break;
			case 'J': mask|=SYS_QZS; break;
			case 'C': mask|=SYS_CMP; break;
			case 'I': mask|=SYS_IRN; break;
			case 'S': mask|=SYS_SBS; break;
		}
	}
}
/* test oepn of file stream ------------------------------------------------------- */
int inrnx_t::test_open() {
	return inf.is_open();
}
/* close file --------------------------------------------------------------------- */
void inrnx_t::closeF(){
	if (inf.is_open()) inf.close();
}
/* base function of read head ----------------------------------------------------- */
int inrnx_t::readhead(){
	int i=0;

	while (getline(inf,buff)&&!inf.eof()){

		if (buff.find("RINEX VERSION / TYPE")!=string::npos){
			str2double(buff.substr(0,9),ver);//version
			type = buff.substr(20,1);	//rinex type
			switch (buff[40]){
				case ' ':
				case 'G': sat_sys=SYS_GPS;  tsys=TSYS_GPS; break;
				case 'R': sat_sys=SYS_GLO;  tsys=TSYS_UTC; break;
				case 'E': sat_sys=SYS_GAL;  tsys=TSYS_GAL; break; /* v.2.12 */
				case 'S': sat_sys=SYS_SBS;  tsys=TSYS_GPS; break;
				case 'J': sat_sys=SYS_QZS;  tsys=TSYS_QZS; break; /* v.3.02 */
				case 'C': sat_sys=SYS_CMP;  tsys=TSYS_CMP; break; /* v.2.12 */
				case 'I': sat_sys=SYS_IRN;  tsys=TSYS_IRN; break; /* v.3.03 */
				case 'M': sat_sys=SYS_NONE; tsys=TSYS_GPS; break; /* mixed */
				default:
				break;
			}
			break;
		}
	}

	return 1;
}
/* vritual function of read head for O,N,G,H,J,L,C -------------------------------- */
int inrnx_t::Head(){
	return 0;
}
/* virtual function of read body for O,N,G,H,J,L,C -------------------------------- */
int inrnx_t::Body(){
	return 0;
}

/* Class inrnxO_t - read Observation file -----------------------------------------------------------
-----------------------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------------------- */
inrnxO_t::inrnxO_t(){
}
inrnxO_t::inrnxO_t(string file, string option, int rcvnum): 
	inrnx_t(file, option), rcv(rcvnum){
}
inrnxO_t::inrnxO_t(string file,string option,gtime_t TS,gtime_t TE,int TI,int rcvnum):
	inrnx_t(file, option), rcv(rcvnum){
	ts=TS; te=TE; tint=TI;
}
inrnxO_t::~inrnxO_t(){
}

/* Implementation functions ------------------------------------------------------- */
/* convert rinex obs type ver.2 -> ver.3 ------------------------------------------ */
void inrnxO_t::convcode(int sys, string str, string &tp){
	if	(!str.compare("P1")) { /* ver.2.11 GPS L1PY,GLO L2P */
		if      (sys==SYS_GPS) tp="C1W";
  		else if (sys==SYS_GLO) tp="C1P";
	}
	else if (!str.compare("P2")) { /* ver.2.11 GPS L2PY,GLO L2P */
		if      (sys==SYS_GPS) tp="C2W";
		else if (sys==SYS_GLO) tp="C2P";
	}
	else if (!str.compare("C1")) { /* ver.2.11 GPS L1C,GLO L1C/A */
		if      (ver>=2.12) ; /* reject C1 for 2.12 */
		else if (sys==SYS_GPS) tp="C1C";
		else if (sys==SYS_GLO) tp="C1C";
		else if (sys==SYS_GAL) tp="C1X"; /* ver.2.12 */
		else if (sys==SYS_QZS) tp="C1C";
		else if (sys==SYS_SBS) tp="C1C";
	}
	else if (!str.compare("C2")) {
		if (sys==SYS_GPS) {
		    if (ver>=2.12) tp="C2W"; /* L2P(Y) */
		    else           tp="C2X"; /* L2C */
		}
		else if (sys==SYS_GLO) tp="C2C";
		else if (sys==SYS_QZS) tp="C2X";
		else if (sys==SYS_CMP) tp="C1X"; /* ver.2.12 B1 */
	}
	else if (ver>=2.12&&str[1]=='A') { /* ver.2.12 L1C/A */
		if      (sys==SYS_GPS) tp=str.substr(0,1)+"1C";
		else if (sys==SYS_GLO) tp=str.substr(0,1)+"1C";
		else if (sys==SYS_QZS) tp=str.substr(0,1)+"1C";
		else if (sys==SYS_SBS) tp=str.substr(0,1)+"1C";
	}
	else if (ver>=2.12&&str[1]=='B') { /* ver.2.12 GPS L1C */
		if      (sys==SYS_GPS) tp=str.substr(0,1)+"1X";
		else if (sys==SYS_QZS) tp=str.substr(0,1)+"1X";
	}
	else if (ver>=2.12&&str[1]=='C') { /* ver.2.12 GPS L2C */
		if      (sys==SYS_GPS) tp=str.substr(0,1)+"2X";
		else if (sys==SYS_QZS) tp=str.substr(0,1)+"2X";
	}
	else if (ver>=2.12&&str[1]=='D') { /* ver.2.12 GLO L2C/A */
		if      (sys==SYS_GLO) tp=str.substr(0,1)+"2C";
	}
	else if (ver>=2.12&&str[1]=='1') { /* ver.2.12 GPS L1PY,GLO L1P */
		if      (sys==SYS_GPS) tp=str.substr(0,1)+"1W";
		else if (sys==SYS_GLO) tp=str.substr(0,1)+"1P";
		else if (sys==SYS_GAL) tp=str.substr(0,1)+"1X"; /* tentative */
		else if (sys==SYS_CMP) tp=str.substr(0,1)+"1X"; /* extension */
	}
	else if (ver<2.12&&str[1]=='1') {
		if      (sys==SYS_GPS) tp=str.substr(0,1)+"1C";
		else if (sys==SYS_GLO) tp=str.substr(0,1)+"1C";
		else if (sys==SYS_GAL) tp=str.substr(0,1)+"1X"; /* tentative */
		else if (sys==SYS_QZS) tp=str.substr(0,1)+"1C";
		else if (sys==SYS_SBS) tp=str.substr(0,1)+"1C";
	}
	else if (str[1]=='2') {
		if      (sys==SYS_GPS) tp=str.substr(0,1)+"2W";
		else if (sys==SYS_GLO) tp=str.substr(0,1)+"2P";
		else if (sys==SYS_QZS) tp=str.substr(0,1)+"2X";
		else if (sys==SYS_CMP) tp=str.substr(0,1)+"1X"; /* ver.2.12 B1 */
	}
	else if (str[1]=='5') {
		if      (sys==SYS_GPS) tp=str.substr(0,1)+"5X";
		else if (sys==SYS_GAL) tp=str.substr(0,1)+"5X";
		else if (sys==SYS_QZS) tp=str.substr(0,1)+"5X";
		else if (sys==SYS_SBS) tp=str.substr(0,1)+"5X";
	}
	else if (str[1]=='6') {
		if      (sys==SYS_GAL) tp=str.substr(0,1)+"6X";
		else if (sys==SYS_QZS) tp=str.substr(0,1)+"6X";
		else if (sys==SYS_CMP) tp=str.substr(0,1)+"6X"; /* ver.2.12 B3 */
	}
	else if (str[1]=='7') {
		if      (sys==SYS_GAL) tp=str.substr(0,1)+"7X";
		else if (sys==SYS_CMP) tp=str.substr(0,1)+"7X"; /* ver.2.12 B2 */
	}
	else if (str[1]=='8') {
		if      (sys==SYS_GAL) tp=str.substr(0,1)+"8X";
	}
}
/* save slips --------------------------------------------------------------------- */
void inrnxO_t::saveslips(unsigned char slips[][NFREQ], obsd_t &data)
{
	int i;
	for (i=0;i<NFREQ;i++) {
		if (data.LLI[i]&1) slips[data.sat-1][i]|=1;
	}
}
/* restore slips */
void inrnxO_t::restslips(unsigned char slips[][NFREQ], obsd_t &data)
{
	int i;
	for (i=0;i<NFREQ;i++) {
		if (slips[data.sat-1][i]&1) data.LLI[i]|=1;
		slips[data.sat-1][i]=0;
	}
}
/* set signal index --------------------------------------------------------------- */
void inrnxO_t::set_index(int syt, string tobss[MAXOBSTYPE], sigind_t &ind)
{
	size_t p;
	string str,optstr;
	double shift;
	int i,j,k,n;
	
	for (i=n=0;tobss[i][0];i++,n++) {
		ind.code[i]=obs2code(tobss[i].substr(1),ind.frq+i);
		ind.type[i]=((p=OBSCODES.find(tobss[i][0]))!=string::npos)?(int)p:0;
		ind.pri[i]=getcodepri(syt,ind.code[i],opt);
		ind.pos[i]=-1;
		
		/* frequency index for beidou */
		if (syt==SYS_CMP) {
			if      (ind.frq[i]==5) ind.frq[i]=2; /* B2 */
			else if (ind.frq[i]==4) ind.frq[i]=3; /* B3 */
		}
	}
	/* parse phase shift options */
	switch (syt) {
		case SYS_GPS: optstr="-GL%2s=%lf"; break;
		case SYS_GLO: optstr="-RL%2s=%lf"; break;
		case SYS_GAL: optstr="-EL%2s=%lf"; break;
		case SYS_QZS: optstr="-JL%2s=%lf"; break;
		case SYS_SBS: optstr="-SL%2s=%lf"; break;
		case SYS_CMP: optstr="-CL%2s=%lf"; break;
		case SYS_IRN: optstr="-IL%2s=%lf"; break;
	}
	for (p=0;p<opt.length();p++) {
		if (opt.substr(p,3)==optstr.substr(0,3)){
			str=opt.substr(p+3,2); 
			str2double(opt.substr(p+5),shift);
			for (i=0;i<n;i++) {
				if (code2obs(ind.code[i],NULL).compare(str)!=0) continue;
				ind.shift[i]=shift;
			}
	  }
	}
	/* assign index for highest priority code */
	for (i=0;i<NFREQ;i++) { // loop of frequency
		for (int tp=0; tp<OBSCODES.size(); tp++) { // loop of observation types
			for (j=0,k=-1; j<n; j++) { // loop of observation data
				if (ind.frq[j]==i+1&&ind.type[j]==tp&&ind.pri[j]&&(k<0||ind.pri[j]>ind.pri[k])) {
					k=j;
				}
			}
			if (k<0) continue;

			for (j=0; j<n; j++) {
				if (ind.type[j]==tp&&ind.code[j]==ind.code[k]) ind.pos[j]=i;
			}
		}
	}
	/* assign index of extended obs data */
	/*for (i=0;i<NEXOBS;i++) {
		for (j=0;j<n;j++) {
			if (ind.code[j]&&ind.pri[j]&&ind.pos[j]<0) break;
		}
		if (j>=n) break;
		
		for (k=0;k<n;k++) {
			if (ind.code[k]==ind.code[j]) ind.pos[k]=NFREQ+i;
		}
	}*/
	ind.n=n;
}
/* decode obs epoch --------------------------------------------------------------- */
int inrnxO_t::decode_obsepoch(gtime_t &t, int &flag,vector<int> &sats){
	int i,j,n;
	string satid;
	
	if (buff.length()<32) return 0;

	if (ver<=2.99) { /* ver.2 */
		str2int(buff.substr(29,3),n);
		if (n<=0) return 0;
		
		/* epoch flag: 3:new site,4:header info,5:external event */
		str2int(buff.substr(28,1),flag);
		if (3<=flag&&flag<=5) return n;
			
		if (t.str2time(buff.substr(0,26))!=0) return 0;
		
		for (i=0,j=32;i<n;i++,j+=3) {
			if (j>=68) {
				if (!getline(inf,buff)) break; /* read next line */
				j=32;
			}
			if (i<MAXOBS) {
				satid=buff.substr(j,3);
				sats[i]=satid2no(satid);
			}
		}
	}
	else { /* ver.3 */
		str2int(buff.substr(32,3),n);
		if (n<=0) return 0;
		
		str2int(buff.substr(31,1),flag); 
		if (3<flag&&flag<=5) return n;
		
		if (buff[0]!='>'||t.str2time(buff.substr(1,28))!=0)
			return 0;
	}
	return n;
}
/* decode obs data ---------------------------------------------------------------- */
int inrnxO_t::decode_obsdata(obsd_t &obs){
	sigind_t *ind;
	double val[MAXOBSTYPE] = { 0.0 };
	unsigned char lli[MAXOBSTYPE] = { 0 };
	string satid;
	int i,j,n,m,num,stat=1,p[MAXOBSTYPE],k[16],l[16];
	
	if (ver>2.99){ /* ver.3 */
		satid=buff.substr(0,3);
		obs.sat=(unsigned char)satid2no(satid);
	}
	obs.sys=satsys(obs.sat,&obs.prn);
	if (!obs.sat) stat=0;
	else if (!((obs.sys=satsys(obs.sat,NULL))&mask)) stat=0;
	
	switch(satsys(obs.sat,NULL)){
		case SYS_GLO: ind=index+1; break;
		case SYS_GAL: ind=index+2; break;
		case SYS_QZS: ind=index+3; break;
		case SYS_SBS: ind=index+4; break;
		case SYS_CMP: ind=index+5; break;
		default: 	  ind=index;   break;
	}
	for (i=0,j=ver<=2.99?0:3;i<ind->n&&j+15<buff.length();i++,j+=16){
		if (stat){
			str2double(buff.substr(j,14),val[i]);
			val[i] += ind->shift[i];
			str2int(buff.substr(j+14,1),num);
			lli[i]=(unsigned char)num&3;
		}
		/* if the last data, read the next line */
		if (ver<=2.99&&j+17>80&&i+1<ind->n) { /* ver.2 */
			if (!getline(inf,buff)) break; /* read next line */
			j=-16;
		}
	}
	if (!stat) return 0;
		
	for (i=n=m=0;i<ind->n;i++){
		p[i]=ver<=2.11?ind->frq[i]-1:ind->pos[i];
			
		if (ind->type[i]==0&&p[i]==0) k[n++]=i; /* C1? index */
		if (ind->type[i]==0&&p[i]==1) l[m++]=i; /* C2? index */
	}
	if (ver<=2.11){
		/* if multiple codes (C1/P1,C2/P2), select higher priority */
	  if (n>=2) {
	  	if (val[k[0]]==0.0&&val[k[1]]==0.0) {
	  		p[k[0]]=-1; p[k[1]]=-1;
	  	}
	  	else if (val[k[0]]!=0.0&&val[k[1]]==0.0) {
	  		p[k[0]]=0; p[k[1]]=-1;
	  	}
	  	else if (val[k[0]]==0.0&&val[k[1]]!=0.0) {
	  		p[k[0]]=-1; p[k[1]]=0;
	  	}
	  	else if (ind->pri[k[1]]>ind->pri[k[0]]) {
	  		p[k[1]]=0; p[k[0]]=NEXOBS<1?-1:NFREQ;
	  	}
	  	else {
	  		p[k[0]]=0; p[k[1]]=NEXOBS<1?-1:NFREQ;
	  	}
	  }
	  if (m>=2) {
	  	if (val[l[0]]==0.0&&val[l[1]]==0.0) {
	  		p[l[0]]=-1; p[l[1]]=-1;
	  	}
	  	else if (val[l[0]]!=0.0&&val[l[1]]==0.0) {
	  		p[l[0]]=1; p[l[1]]=-1;
	  	}
	  	else if (val[l[0]]==0.0&&val[l[1]]!=0.0) {
	  		p[l[0]]=-1; p[l[1]]=1; 
	  	}
	  	else if (ind->pri[l[1]]>ind->pri[l[0]]) {
	  		p[l[1]]=1; p[l[0]]=NEXOBS<2?-1:NFREQ+1;
	  	}
	  	else {
	  		p[l[0]]=1; p[l[1]]=NEXOBS<2?-1:NFREQ+1;
	  	}
	  }
	}
	/* save obs data */
	for (i=0;i<ind->n;i++){
		if(p[i]<0||val[i]==0.0) continue;
		switch(ind->type[i]){
			case 0: obs.P[p[i]]=val[i]; obs.code[p[i]]=ind->code[i]; break;
			case 1: obs.L[p[i]]=val[i]; obs.LLI [p[i]]=lli[i];       break;
			case 2: obs.D[p[i]]=(float)val[i];                        break;
			case 3: obs.SNR[p[i]]=(unsigned char)(val[i]*4.0+0.5);    break;
		}
	}

	return 1;
}

/* read "O" file to obs_t vector -------------------------------------------------- */
int inrnxO_t::readrnxobsb(int &flag, vector<obsd_t> &data) {
	obsd_t obs;
	gtime_t time;
	int i=0,n=0,nsat=0;
	vector<int> sats(MAXOBS,0);
	
	/* read record */	
	while (getline(inf,buff)&&!inf.eof()){

		/* decode obs epoch */
		if (i==0) {
			if ((nsat=decode_obsepoch(time,flag,sats))<=0) {
			    continue;
			}
		}
		else if (flag<=2||flag==6) {
			
			obs.reset();
			obs.time=time; obs.time.sys=satsys(sats[i-1],NULL);
			obs.sat=(unsigned char)sats[i-1];
			
			/* decode obs data */
			if (decode_obsdata(obs)&&data.size()<MAXOBS){
				data.push_back(obs); n++;
			}
		}
		if (++i>nsat) return n;
		
	}
	return 0;
}
/* initialization --------------------------------------------------------- */
void inrnxO_t::ini_ReadO(string file,string option,gtime_t TS,gtime_t TE,int TI,int rcvnum) {
	inf.open(file,ios::in);
	opt=option;
	ts=TS; te=TE; tint=TI;
}
/* read Observation file head ----------------------------------------------------- */
int inrnxO_t::Head(nav_t *nav,sta_t *sta){
	/* chekc inf */
	if (!inf.is_open()) return 0;

	/* default codes for unknown code */
	const string defcodes[]={
		"CWX    ",  /* GPS: L125____ */
		"CC     ",  /* GLO: L12_____ */
		"X XXXX ",  /* GAL: L1_5678_ */
		"CXXX   ",  /* QZS: L1256___ */
		"C X    ",  /* SBS: L1_5____ */
		"X  XX  ",  /* BDS: L1__67__ */
		"  A   A"   /* IRN: L__5___9 */
	};
	int i,j,k,n,nt,prn,fcn,nline=0;
	string str;
	
	/* base rinex  */
	if (readhead()!=1) { errmsg="read head error!\n"; return 0; }
  
	while (getline(inf,buff)&&!inf.eof()){
  	
  		if (buff.find("MARKER NAME")!=string::npos && sta)
  			sta->name=buff.substr(0,60);
  		else if (buff.find("MARKER NUMBER")!=string::npos && sta)
  			sta->marker=buff.substr(0,20);
  		else if (buff.find("MARKER TYPE")!=string::npos) continue;
  		else if (buff.find("OBSERVER / AGENCY")!=string::npos) continue;
  		else if (buff.find("REC # / TYPE / VERS")!=string::npos && sta){
  			sta->recsno=buff.substr(0,20);
  			sta->rectype=buff.substr(20,20);
  			sta->recver=buff.substr(40,20);
  		}
  		else if (buff.find("ANT # / TYPE")!=string::npos && sta){
  			sta->antsno=buff.substr(0,20);
  			sta->antdes=buff.substr(20,20);
  		}
  		else if (buff.find("APPROX POSITION XYZ")!=string::npos && sta)
  		for (i=0;i<3;i++){
			str2double(buff.substr(i*14,14),sta->pos[i]);
  		}
  		else if (buff.find("ANTENNA: DELTA H/E/N")!=string::npos && sta){
				str2double(buff.substr(0,14),sta->del[2]);  /* h */
				str2double(buff.substr(14,14),sta->del[0]); /* e */
				str2double(buff.substr(28,14),sta->del[1]); /* n */
		}
  		else if (buff.find("ANTENNA: DELTA X/Y/Z")!=string::npos) continue; /* opt ver.3 */
		else if (buff.find("ANTENNA: PHASECENTER")!=string::npos) continue; /* opt ver.3 */
		else if (buff.find("ANTENNA: B.SIGHT XYZ")!=string::npos) continue; /* opt ver.3 */
		else if (buff.find("ANTENNA: ZERODIR AZI")!=string::npos) continue; /* opt ver.3 */
		else if (buff.find("ANTENNA: ZERODIR XYZ")!=string::npos) continue; /* opt ver.3 */
		else if (buff.find("CENTER OF MASS: XYZ" )!=string::npos) continue; /* opt ver.3 */
		else if (buff.find("SYS / # / OBS TYPES" )!=string::npos){ /* ver.3 */
			if ((i=syscodes.find(buff[0]))==string::npos) { 
				errmsg="wrong system "+buff.substr(0,1)+"!\n"; return 0; 
			}
    		str2int(buff.substr(3,3),n);
    		for (j=nt=0,k=7;j<n;j++,k+=4){
    			if (k>58){
    				if (!getline(inf,buff)) break; /* read next line */
    				k=7;
    			}
    			if (nt<MAXOBSTYPE-1) tobs[i][nt++]=buff.substr(k,3);
    		}
    		tobs[i][nt]="\0";
    	
    		/* change beidou B1 code: 3.02 draft -> 3.02/3.03 */
    		if (i==5)
    			for (j=0;j<nt;j++) if (tobs[i][j][1]=='2') tobs[i][j].replace(1,1,"1");
    		for (j=0;j<nt;j++){
    			if (tobs[i][j][2]) continue;
    			if ((k=frqcodes.find(tobs[i][j][1]))==string::npos) continue;
    			tobs[i][j].replace(2,1,defcodes[i].substr(k,1));
    		}
		}
		else if (buff.find("WAVELENGTH FACT L1/2")!=string::npos) continue; /* opt ver.2 */
		else if (buff.find("# / TYPES OF OBSERV")!=string::npos){ /* ver.2 */
			str2int(buff.substr(0,6),n);
    		for (i=nt=0,j=10;i<n;i++,j+=6){
    			if(j>58) {
    				if (!getline(inf,buff)) break;	/* read next line */
    				j=10;
    			}
    			if (nt>=MAXOBSTYPE-1) continue;
    			if (ver<=2.99) {
    				str=buff.substr(j,2);
        			convcode(SYS_GPS,str,tobs[0][nt]);
        			convcode(SYS_GLO,str,tobs[1][nt]);
        			convcode(SYS_GAL,str,tobs[2][nt]);
        			convcode(SYS_QZS,str,tobs[3][nt]);
        			convcode(SYS_SBS,str,tobs[4][nt]);
        			convcode(SYS_CMP,str,tobs[5][nt]);
    			}
    			nt++;
    		}
    		tobs[0][nt]="\0";
		}
		else if (buff.find("SIGNAL STRENGTH UNIT")!=string::npos) continue; /* opt ver.3 */
		else if (buff.find("INTERVAL"			 )!=string::npos) continue; /* opt */
		else if (buff.find("TIME OF FIRST OBS"   )!=string::npos) {
			if      (!buff.compare(48,3,"GPS")) tsys=TSYS_GPS;
			else if (!buff.compare(48,3,"GLO")) tsys=TSYS_UTC;
			else if (!buff.compare(48,3,"GAL")) tsys=TSYS_GAL;
			else if (!buff.compare(48,3,"QZS")) tsys=TSYS_QZS; /* ver.3.02 */
			else if (!buff.compare(48,3,"BDT")) tsys=TSYS_CMP; /* ver.3.02 */
			else if (!buff.compare(48,3,"IRN")) tsys=TSYS_IRN; /* ver.3.03 */
		}
		else if (buff.find("TIME OF LAST OBS"    )!=string::npos) continue; /* opt */
		else if (buff.find("RCV CLOCK OFFS APPL" )!=string::npos) continue; /* opt */
		else if (buff.find("SYS / DCBS APPLIED"  )!=string::npos) continue; /* opt ver.3 */
		else if (buff.find("SYS / PCVS APPLIED"  )!=string::npos) continue; /* opt ver.3 */
		else if (buff.find("SYS / SCALE FACTOR"  )!=string::npos) continue; /* opt ver.3 */
		else if (buff.find("SYS / PHASE SHIFTS"  )!=string::npos) continue; /* ver.3.01 */
		else if (buff.find("GLONASS SLOT / FRQ #")!=string::npos && nav) /* ver.3.02 */
			for (i=0;i<8;i++) {
				if (buff.compare(8*i+4,1,"R")!=0||!buff.compare(8*i+8,2,"  ")) continue;
				str2int(buff.substr(8*i+5,2),prn);
				str2int(buff.substr(8*i+8,2),fcn);
				if (1<=prn&&prn<=MAXPRNGLO) nav->glo_fcn[prn-1]=fcn+8;
			}
		else if (buff.find("GLONASS COD/PHS/BIS" )!=string::npos && nav) /* ver.3.02 */
			for (i=0;i<4;i++) {          
				if      (buff.compare(13*i+1,3,"C1C")) 
					str2double(buff.substr(13*i+5,8),nav->glo_cpbias[0]);
				else if (buff.compare(13*i+1,3,"C1P")) 
					str2double(buff.substr(13*i+5,8),nav->glo_cpbias[1]);
				else if (buff.compare(13*i+1,3,"C2C"))
					str2double(buff.substr(13*i+5,8),nav->glo_cpbias[2]);
				else if (buff.compare(13*i+1,3,"C2P")) 
					str2double(buff.substr(13*i+5,8),nav->glo_cpbias[3]);
			}
		else if (buff.find("LEAP SECONDS")!=string::npos && nav) {/* opt */
			str2int(buff.substr(0,6),nav->leaps);
		}
		else if (buff.find("# OF SALTELLITES")!=string::npos) continue;/* opt */
		else if (buff.find("PRN / # OF OBS"  )!=string::npos) continue;/* opt */
		else if (buff.find("PGM / RUN BY / DATE")!=string::npos) continue;
		else if (buff.find("COMMENT" )!=string::npos) continue;
		if (buff.find("END OF HEADER")!=string::npos) 
			break;
		if (++nline>=MAXPOSHEAD && type.compare(" ")==0) return 0; /* no rinex file */
	}
	
	/* set signal index */
	set_index(SYS_GPS,tobs[0],index[0]);
	set_index(SYS_GLO,tobs[1],index[1]);
	set_index(SYS_GAL,tobs[2],index[2]);
	set_index(SYS_QZS,tobs[3],index[3]);
	set_index(SYS_SBS,tobs[4],index[4]);
	set_index(SYS_CMP,tobs[5],index[5]);
	set_index(SYS_IRN,tobs[6],index[6]);

	return 1;
}
/* read boy of "O" file ----------------------------------------------------------- */
int inrnxO_t::Body(obs_t *obs,prcopt_t *prcopt){
	vector<obsd_t> data;
	unsigned char slips[MAXSAT][NFREQ]={{0}};
	int i,n,flag=0;
	
	if (!obs||!inf.is_open()) return 0;
	
	/* set system mask */
	set_sysmask(prcopt);

	/* read rinex obs data body */
	while ((n=readrnxobsb(flag,data))>=0&&!inf.eof()) {
		
		for (i=0;i<n;i++) {
			
			/* utc -> gpst */
			if (tsys==TSYS_UTC) data[i].time.utc2gpst();
			data[i].time.time2str(3);
			
			/* save cycle-slip */
			saveslips(slips,data[i]);
		}
		/* screen data by time */
		if (n>0&&!data[0].time.screent(ts,te,tint)) continue;
		
		for (i=0;i<n;i++) {
			
			/* restore cycle-slip */
			restslips(slips,data[i]);
			
			obs->rcv=(unsigned char)rcv;
			
			/* save obs data */
			obs->data.push_back(data[i]);
		}
		data.clear();
	}
	
	obs->n=obs->data.size();
	closeF();
	
	return n;
	
}
/* read one epoch body of "O" file ---------------------------------------- */
int inrnxO_t::One_Epoch_Body(obs_t *obs,prcopt_t *prcopt) {
	if (!obs||!inf.is_open()) return 0;

	obs->data.clear(); obs->n=0;
	unsigned char slips[MAXSAT][NFREQ]={{0}};
	int i,n,flag=0;
	
	/* set system mask */
	set_sysmask(prcopt);

	/* read rinex obs data body */
	while ((n=readrnxobsb(flag,obs->data))>=0&&!inf.eof()) {
		
		for (i=0;i<n;i++) {
			
			/* utc -> gpst */
			if (tsys==TSYS_UTC) obs->data[i].time.utc2gpst();
			obs->data[i].time.time2str(3);
			
			/* save cycle-slip */
			saveslips(slips,obs->data[i]);
		}
		/* screen data by time */
		if (n>0&&!obs->data[0].time.screent(ts,te,tint)) { obs->data.clear(); continue; }
		
		for (i=0;i<n;i++) {
			
			/* restore cycle-slip */
			restslips(slips,obs->data[i]);
			
			obs->rcv=(unsigned char)rcv;
		}
		if (n>0) {
			obs->n=obs->data.size();
			break;
		}
	}

	return n;
}

/* derived class inrnxN_t ----------------------------------------------------------------------------
* N file - Navigation ephemeris -------------------------------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
inrnxN_t::inrnxN_t() {
}
inrnxN_t::inrnxN_t(string file, string option) {
	inf.open(file,ios::in);
	opt=option;
	ver=2.10;
	ts=gtime_t();
	te=gtime_t();
	tint=0.0;
}
inrnxN_t::~inrnxN_t() {
}
/* Implementation functions ------------------------------------------------------- */
/* ura value (m) to ura index ----------------------------------------------------- */
int inrnxN_t::uraindex(double value) {
	int i;
    for (i=0;i<15;i++) if (ura_eph[i]>=value) break;
    return i;
}
/* decode glonass ephemeris ------------------------------------------------------- */
int inrnxN_t::decode_geph(gtime_t toc, int sat, geph_t *geph) {
    gtime_t tof;
    double tow,tod;
    int week,dow;
    
    if (satsys(sat,NULL)!=SYS_GLO) {
        return 0;
    }

    geph->sat=sat;
    
    /* toc rounded by 15 min in utc */
    tow=toc.time2gpst(&week);
    toc.gpst2time(week,floor((tow+450.0)/900.0)*900);
    dow=(int)floor(tow/86400.0);
    
    /* time of frame in utc */
    tod=ver<=2.99?data[2]:fmod(data[2],86400.0); /* tod (v.2), tow (v.3) in utc */
    tof.gpst2time(week,tod+dow*86400.0);
    tof.adjday(toc);
    
    geph->toe.copy_gtime(toc)->utc2gpst();   /* toc (gpst) */
    geph->tof.copy_gtime(toc)->utc2gpst();   /* tof (gpst) */
    
    /* iode = tb (7bit), tb =index of UTC+3H within current day */
    geph->iode=(int)(fmod(tow+10800.0,86400.0)/900.0+0.5);
    
    geph->taun=-data[0];       /* -taun */
    geph->gamn= data[1];       /* +gamman */
    
    geph->pos[0]=data[3]*1E3; geph->pos[1]=data[7]*1E3; geph->pos[2]=data[11]*1E3;
    geph->vel[0]=data[4]*1E3; geph->vel[1]=data[8]*1E3; geph->vel[2]=data[12]*1E3;
    geph->acc[0]=data[5]*1E3; geph->acc[1]=data[9]*1E3; geph->acc[2]=data[13]*1E3;
    
    geph->svh=(int)data[ 6];
    geph->frq=(int)data[10];
    geph->age=(int)data[14];
    
    /* some receiver output >128 for minus frequency number */
    if (geph->frq>128) geph->frq-=256;
    
	if (geph->frq<MINFREQ_GLO||MAXFREQ_GLO<geph->frq) {
		geph->svh=-1;
    }

    return 1;
}
/* decode geo ephemeris ----------------------------------------------------------- */
int inrnxN_t::decode_seph(gtime_t toc,int sat,seph_t *seph) {
    int week;
    
    if (satsys(sat,NULL)!=SYS_SBS) {
        return 0;
    }
    
    seph->sat=sat;
    seph->t0 =toc;
    
    toc.time2gpst(&week);
    seph->tof.gpst2time(week,data[2])->adjweek(toc);
    
    seph->af0=data[0];
    seph->af1=data[1];
    
    seph->pos[0]=data[3]*1E3; seph->pos[1]=data[7]*1E3; seph->pos[2]=data[11]*1E3;
    seph->vel[0]=data[4]*1E3; seph->vel[1]=data[8]*1E3; seph->vel[2]=data[12]*1E3;
    seph->acc[0]=data[5]*1E3; seph->acc[1]=data[9]*1E3; seph->acc[2]=data[13]*1E3;
    
    seph->svh=(int)data[6];
    seph->sva=uraindex(data[10]);
    
    return 1;
}
/* decode ephemeris --------------------------------------------------------------- */
int inrnxN_t::decode_eph(gtime_t toc,int sat,eph_t *eph) {
    int sys;
    
    sys=satsys(sat,NULL);
    
    if (!(sys&(SYS_GPS|SYS_GAL|SYS_QZS|SYS_CMP|SYS_IRN))) {
        return 0;
    }
    
    eph->sat=sat;
    eph->toc=toc;
    
    eph->f0=data[0];
    eph->f1=data[1];
    eph->f2=data[2];
    
    eph->A=SQR(data[10]); eph->e=data[ 8]; eph->i0  =data[15]; eph->OMG0=data[13];
    eph->omg =data[17]; eph->M0 =data[ 6]; eph->deln=data[ 5]; eph->OMGd=data[18];
    eph->idot=data[19]; eph->crc=data[16]; eph->crs =data[ 4]; eph->cuc =data[ 7];
    eph->cus =data[ 9]; eph->cic=data[12]; eph->cis =data[14];
    
    if (sys==SYS_GPS||sys==SYS_QZS) {
        eph->iode=(int)data[ 3];      /* IODE */
        eph->iodc=(int)data[26];      /* IODC */
        eph->toes=     data[11];      /* toe (s) in gps week */
        eph->week=(int)data[21];      /* gps week */
        eph->toe.gpst2time(eph->week,data[11])->adjweek(toc);
        eph->ttr.gpst2time(eph->week,data[27])->adjweek(toc);
        
        eph->code=(int)data[20];      /* GPS: codes on L2 ch */
        eph->svh =(int)data[24];      /* sv health */
        eph->sva=uraindex(data[23]);  /* ura (m->index) */
        eph->flag=(int)data[22];      /* GPS: L2 P data flag */
        
        eph->tgd[0]=   data[25];      /* TGD */
        if (sys==SYS_GPS) {
            eph->fit=data[28];        /* fit interval (h) */
        }
        else {
            eph->fit=data[28]==0.0?1.0:2.0; /* fit interval (0:1h,1:>2h) */
        }
    }
    else if (sys==SYS_GAL) { /* GAL ver.3 */
        eph->iode=(int)data[ 3];      /* IODnav */
        eph->toes=     data[11];      /* toe (s) in galileo week */
        eph->week=(int)data[21];      /* gal week = gps week */
        eph->toe.gpst2time(eph->week,data[11])->adjweek(toc);
        eph->ttr.gpst2time(eph->week,data[27])->adjweek(toc);
        
        eph->code=(int)data[20];      /* data sources */
                                      /* bit 0 set: I/NAV E1-B */
                                      /* bit 1 set: F/NAV E5a-I */
                                      /* bit 2 set: F/NAV E5b-I */
                                      /* bit 8 set: af0-af2 toc are for E5a.E1 */
                                      /* bit 9 set: af0-af2 toc are for E5b.E1 */
        eph->svh =(int)data[24];      /* sv health */
                                      /* bit     0: E1B DVS */
                                      /* bit   1-2: E1B HS */
                                      /* bit     3: E5a DVS */
                                      /* bit   4-5: E5a HS */
                                      /* bit     6: E5b DVS */
                                      /* bit   7-8: E5b HS */
        eph->sva =uraindex(data[23]); /* ura (m->index) */
        
        eph->tgd[0]=   data[25];      /* BGD E5a/E1 */
        eph->tgd[1]=   data[26];      /* BGD E5b/E1 */
    }
    else if (sys==SYS_CMP) { /* BeiDou v.3.02 */
		eph->toc.copy_gtime(eph->toc)->bdt2gpst();         /* bdt -> gpst */
        eph->iode=(int)data[ 3];                           /* AODE */
        eph->iodc=(int)data[28];                           /* AODC */
        eph->toes=     data[11];                           /* toe (s) in bdt week */
        eph->week=(int)data[21];                           /* bdt week */
        eph->toe.bdt2time(eph->week,data[11])->bdt2gpst(); /* bdt -> gpst */
        eph->ttr.bdt2time(eph->week,data[27])->bdt2gpst(); /* bdt -> gpst */
        eph->toe.adjweek(toc);
        eph->ttr.adjweek(toc);
        
        eph->svh =(int)data[24];      /* satH1 */
        eph->sva=uraindex(data[23]);  /* ura (m->index) */
        
        eph->tgd[0]=   data[25];      /* TGD1 B1/B3 */
        eph->tgd[1]=   data[26];      /* TGD2 B2/B3 */
    }
    else if (sys==SYS_IRN) { /* IRNSS v.3.03 */
        eph->iode=(int)data[ 3];      /* IODEC */
        eph->toes=     data[11];      /* toe (s) in irnss week */
        eph->week=(int)data[21];      /* irnss week */
        eph->toe.gpst2time(eph->week,data[11])->adjweek(toc);
        eph->ttr.gpst2time(eph->week,data[27])->adjweek(toc);
        eph->svh =(int)data[24];      /* sv health */
        eph->sva=uraindex(data[23]);  /* ura (m->index) */
        eph->tgd[0]=   data[25];      /* TGD */
    }

	if (eph->iode<0||1023<eph->iode) {
		eph->svh=-1;
    }
    if (eph->iodc<0||1023<eph->iodc) {
        eph->svh=-1;
    }
	eph->toc.time2str(3);
	eph->toe.time2str(3);

    return 1;
}
/* add data to nav_t -------------------------------------------------------------- */
void inrnxN_t::addnav(int sys,nav_t *nav) {
	if (sys==SYS_GLO) {
		nav->ngmax=2*++nav->ng;
		nav->geph.push_back(geph_t());
	}
	else if (sys==SYS_SBS) {
		nav->nsmax=2*++nav->ns;
		nav->seph.push_back(seph_t());
	}
	else if (sys==SYS_CMP||sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS) {
		nav->nmax=2*++nav->n;
		nav->eph.push_back(eph_t());
	}
}
/* read "O" file to obs_t vector -------------------------------------------------- */
int inrnxN_t::readrnxnavb(nav_t *nav) {
	 gtime_t toc,last_toc;
    int i=0,j,prn,sat=0,last_sat=0,sp=3,sys=sat_sys,flag=1;
	string id;

	while (getline(inf,buff)&&!inf.eof()) {
		if (buff.compare(0, 3, "   ")!=0) i=0;
		/* first line */
		if (i==0) {
			 /* decode satellite field */
            if (ver>=3.0||sat_sys==SYS_GAL||sat_sys==SYS_QZS||sat_sys==SYS_NONE) { /* ver.3 or GAL/QZS */
				id=buff.substr(0,3);
                sat=satid2no(id);
                sp=4;
                if (ver>=3.0) sys=satsys(sat,NULL);
            }
            else {
                str2int(buff.substr(0,2),prn);
                
                if (sys==SYS_SBS) sat=satno(SYS_SBS,prn+100);

                else if (sys==SYS_GLO) sat=satno(SYS_GLO,prn);

                //else if (93<=prn&&prn<=97) { /* extension */
                //    sat=satno(SYS_QZS,prn+100);
                //}
				else sat=satno(SYS_GPS, prn); 
            }
            /* decode toc field */
            if (toc.str2time(buff.substr(sp,19))) flag=0; 
			else if (last_sat==sat&&last_toc.time==toc.time) flag=0;
			else flag=1;
            /* decode data fields */
            for (j=0;j<3;j++) {
                str2double(buff.substr(sp+19*(j+1),19),data[i++]);
            }
		}
		/* next line */
		else if (flag==1){
			/* decode data fields */
            for (j=0;j<4;j++) {
				if (sp+19*(j+1)<=buff.size()) str2double(buff.substr(sp+19*j,19),data[i++]);
				else data[i++]=0.0;
            }
            /* decode ephemeris */
            if (sys==SYS_GLO&&i>=15) {
				if (!(mask&sys)) continue;
				addnav(sys,nav);
                decode_geph(toc,sat,&nav->geph.back());
				last_sat=sat; last_toc=toc;
				continue;
            }
            else if (sys==SYS_SBS&&i>=15) {
				if (!(mask&sys)) continue;
				addnav(sys,nav);
                decode_seph(toc,sat,&nav->seph.back());
				last_sat=sat; last_toc=toc;
				continue;
            }
			else if (i >= 29 && ver >= 3.0) {
				if (!(mask&sys)) continue;
				addnav(sys,nav);
				decode_eph(toc,sat,&nav->eph.back());
				last_sat=sat; last_toc=toc;
				continue;
			}
            else if (i>=31) {
                if (!(mask&sys)) continue;
				addnav(sys,nav);
                decode_eph(toc,sat,&nav->eph.back());
				last_sat=sat; last_toc=toc;
				continue;
            }
		}
	}
	return 1;
}
/* initialization --------------------------------------------------------- */
void inrnxN_t::ini_ReadN(string file,string option) {
	inf.open(file,ios::in);
	opt=option;
}
/* read head of "N" file ---------------------------------------------------------- */
int inrnxN_t::Head(nav_t *nav) {
	/* chekc inf */
	if (!inf.is_open()) return 0;

	int nline=0,i,j;
	string str;
	
	/* base rinex  */
	if (readhead()!=1) return 0;

	while (getline(inf,buff)&&!inf.eof()) {
		if (buff.find("ION ALPHA",60)!=string::npos) { /* opt ver.2 */
			if (nav) {
				for (i=0,j=2;i<4;i++,j+=12) str2double(buff.substr(j,12),nav->ion_gps[i]);
			}
		}
		else if (buff.find("ION BETA",60)!=string::npos) { /* opt ver.2 */
			if (nav) {
				for (i=0,j=2;i<4;i++,j+=12) str2double(buff.substr(j,12),nav->ion_gps[i+4]);
			}
		}
		else if (buff.find("DELTA-UTC: A0,A1,T,W",60)!=string::npos) { /* opt ver.2 */
			if (nav) {
				for (i=0,j=3;i<2;i++,j+=19) str2double(buff.substr(j,12),nav->utc_gps[i]);
				for (;i<4;i++,j+=9) str2double(buff.substr(j,9),nav->utc_gps[i]);
			}
		}
		else if (buff.find("IONOSPHERIC CORR",60)!=string::npos) { /* opt ver.3 */
			if (nav) {
				if (buff.compare(0,4,"GPSA")==0) {
					for (i=0,j=5;i<4;i++,j+=12) str2double(buff.substr(j,12),nav->ion_gps[i]);
				}
				else if (buff.compare(0,4,"GPSB")==0) {
					for (i=0,j=5;i<4;i++,j+=12) str2double(buff.substr(j,12),nav->ion_gps[i+4]);
				}
				else if (buff.compare(0,3,"GAL")==0) {
					for (i=0,j=5;i<4;i++,j+=12) str2double(buff.substr(j,12),nav->ion_gal[i]);
				}
				else if (buff.compare(0,4,"QZSA")==0) { /* v.3.02 */
					for (i=0,j=5;i<4;i++,j+=12) str2double(buff.substr(j,12),nav->ion_qzs[i]);
				}
				else if (buff.compare(0,4,"QZSB")==0) { /* v.3.02 */
					for (i=0,j=5;i<4;i++,j+=12) str2double(buff.substr(j,12),nav->ion_qzs[i+4]);
				}
				else if (buff.compare(0,4,"BDSA")==0) { /* v.3.02 */
					for (i=0,j=5;i<4;i++,j+=12) str2double(buff.substr(j,12),nav->ion_cmp[i]);
				}
				else if (buff.compare(0,4,"BDSB")==0) { /* v.3.02 */
					for (i=0,j=5;i<4;i++,j+=12) str2double(buff.substr(j,12),nav->ion_cmp[i+4]);
				}
				else if (buff.compare(0,4,"IRNA")==0) { /* v.3.03 */
					for (i=0,j=5;i<4;i++,j+=12) str2double(buff.substr(j,12),nav->ion_irn[i]);
				}
				else if (buff.compare(0,4,"IRNB")==0) { /* v.3.03 */
					for (i=0,j=5;i<4;i++,j+=12) str2double(buff.substr(j,12),nav->ion_irn[i+4]);
				}
			}
		}
		else if (buff.find("TIME SYSTEM CORR",60)!=string::npos) { /* opt ver.3 */
			if (nav) {
				if (buff.compare(0,4,"GPUT")==0) {
					str2double(buff.substr( 5,17),nav->utc_gps[0]);
					str2double(buff.substr(22,16),nav->utc_gps[1]);
					str2double(buff.substr(38, 7),nav->utc_gps[2]);
					str2double(buff.substr(45, 5),nav->utc_gps[3]);
				}
				else if (buff.compare(0,4,"GLUT")==0) {
					str2double(buff.substr( 5,17),nav->utc_glo[0]);
					str2double(buff.substr(22,16),nav->utc_glo[1]);
					str2double(buff.substr(38, 7),nav->utc_glo[2]);
					str2double(buff.substr(45, 5),nav->utc_glo[3]);
				}
				else if (buff.compare(0,4,"GAUT")==0) { /* v.3.02 */
					str2double(buff.substr( 5,17),nav->utc_gal[0]);
					str2double(buff.substr(22,16),nav->utc_gal[1]);
					str2double(buff.substr(38, 7),nav->utc_gal[2]);
					str2double(buff.substr(45, 5),nav->utc_gal[3]);
				}
				else if (buff.compare(0,4,"QZUT")==0) { /* v.3.02 */
					str2double(buff.substr( 5,17),nav->utc_qzs[0]);
					str2double(buff.substr(22,16),nav->utc_qzs[1]);
					str2double(buff.substr(38, 7),nav->utc_qzs[2]);
					str2double(buff.substr(45, 5),nav->utc_qzs[3]);
				}
				else if (buff.compare(0,4,"BDUT")==0) { /* v.3.02 */
					str2double(buff.substr( 5,17),nav->utc_cmp[0]);
					str2double(buff.substr(22,16),nav->utc_cmp[1]);
					str2double(buff.substr(38, 7),nav->utc_cmp[2]);
					str2double(buff.substr(45, 5),nav->utc_cmp[3]);
				}
				else if (buff.compare(0,4,"SBUT")==0) { /* v.3.02 */
					str2double(buff.substr( 5,17),nav->utc_cmp[0]);
					str2double(buff.substr(22,16),nav->utc_cmp[1]);
					str2double(buff.substr(38, 7),nav->utc_cmp[2]);
					str2double(buff.substr(45, 5),nav->utc_cmp[3]);
				}
				else if (buff.compare(0,4,"IRUT")==0) { /* v.3.03 */
					str2double(buff.substr( 5,17),nav->utc_irn[0]);
					str2double(buff.substr(22,16),nav->utc_irn[1]);
					str2double(buff.substr(38, 7),nav->utc_irn[2]);
					str2double(buff.substr(45, 5),nav->utc_irn[3]);
				}
			}
		}
		else if (buff.find("LEAP SECONDS",60)!=string::npos) { /* opt */
			if (nav) str2int(buff.substr(0,6),nav->leaps);
		}
		if (buff.find("END OF HEADER")!=string::npos) return 1;
		if (++nline>=MAXPOSHEAD && type.compare(" ")==0) break; /* no rinex file */
	}
	return 0;
}
/* read boy of "N" file ----------------------------------------------------------- */
int inrnxN_t::Body(nav_t *nav,prcopt_t *prcopt) {
	if (!nav) return 0;

	set_sysmask(prcopt);

	readrnxnavb(nav);

	closeF();
	return nav->n>0||nav->ng>0||nav->ns>0;
}


/* read earth rotation parameters file ------------------------------------------------------------ */
/* Constructors ------------------------------------------------------------------- */
inerp_t::inerp_t(){
}
inerp_t::inerp_t(string file){
	inf.open(file,ios::in);
	file_path=file;
}
inerp_t::~inerp_t(){
	if (inf.is_open()) inf.close();
}
/* Implementation functions ------------------------------------------------------- */
/* open file ---------------------------------------------------------------------- */
void inerp_t::open_file(){
	inf.open(file_path);
}
/* read earth rotation parameters file -------------------------------------------- */
int inerp_t::readerp(erp_t *erp){
	string buff;
	double v[14]={0};

	if (!inf.is_open()) return 0;

	/* initialize erp->data */
	erp->data.clear();

	while (getline(inf,buff)&&!inf.eof()){
		if (sscanf(buff.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
			v,v+1,v+2,v+3,v+4,v+5,v+6,v+7,v+8,v+9,v+10,v+11,v+12,v+13)<5) 
			continue;
		erp->data.push_back(erpd_t());
		erp->data.back().mjd=v[0];
		erp->data.back().xp=v[1]*1E-6*AS2R;
		erp->data.back().yp=v[2]*1E-6*AS2R;
		erp->data.back().ut1_utc=v[3]*1E-7;
		erp->data.back().lod=v[4]*1E-7;
		erp->data.back().xpr=v[12]*1E-6*AS2R;
		erp->data.back().ypr=v[13]*1E-6*AS2R;	
	}
	erp->n=erp->data.size();
	erp->nmax=erp->n+128;

	inf.close();
	return 1;
}

/* read ocean-loading tide (.BLQ) file ------------------------------------------------------------ */
/* Constructors ------------------------------------------------------------------- */
inblq_t::inblq_t(){
}
inblq_t::inblq_t(string file){
	inf.open(file,ios::in);
	file_path=file;
}
inblq_t::~inblq_t(){
	if (inf.is_open()) inf.close();
}
/* Implementation functions ------------------------------------------------------- */
/* open file ---------------------------------------------------------------------- */
void inblq_t::open_file(){
	inf.open(file_path);
}
/* read earth rotation parameters file -------------------------------------------- */
int inblq_t::readblq(const string staname,double *ocean_par){
	string buff,NAME="    ";
	if (!inf.is_open()) inf.open(file_path,ios::in);

	/* 4-character station name */
	transform(staname.begin(),staname.begin()+4,NAME.begin(),::toupper);

	while (inf.is_open()&&getline(inf,buff)&&!inf.eof()){
		if (buff.size()<2||buff.compare(0,2,"$$")==0) continue;
		/* read blq value if station name is right */
		if (buff.compare(2,4,NAME)==0) {
			double v[11]={0};
			int n=0;
			while (getline(inf,buff)&&!inf.eof()){
				if (buff.compare(0,2,"$$")==0) continue;
				if (sscanf(buff.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
					v,v+1,v+2,v+3,v+4,v+5,v+6,v+7,v+8,v+9,v+10)<11) continue;
				for (int i=0; i<11; i++) ocean_par[n+i*6]=v[i];
				if (++n==6) { inf.close(); return 1; }
			}
			inf.close(); return 0;
		}
	}

	inf.close();
	return 1;
}

/* read antenna information file (.ATX) ----------------------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
inatx_t::inatx_t(){
}
inatx_t::inatx_t(string file){
	inf.open(file,ios::in);
	file_path=file;
}
inatx_t::~inatx_t(){
	inf.close();
}
/* Implementation functions ------------------------------------------------------- */
/* open file ---------------------------------------------------------------------- */
void inatx_t::open_file(){
	inf.open(file_path,ios::in);
}
/* read earth rotation parameters file -------------------------------------------- */
int inatx_t::readatx(){
	int stat=0;
	int i,f,freq,freqs[]={ 1,2,5,6,7,8,0 };

	if (!inf.is_open()) inf.open(file_path,ios::in);

	/* read atx information to pcv vector */
	while (inf.is_open()&&getline(inf,buff)&&!inf.eof()){
		if (buff.length()<60||buff.find("COMMENT",60)!=string::npos) continue;

		/* start of one antenna information */
		if (buff.find("START OF ANTENNA",60)!=string::npos) {
			pcv.push_back(pcv_t());  stat=1;
		}
		/* end of one antenna information */
		if (buff.find("END OF ANTENNA",60)!=string::npos) stat=0;
		if (!stat) continue;

		/* antnenna type and number */
		if (buff.find("TYPE / SERIAL NO",60)!=string::npos){
			pcv.back().type=buff.substr(0,20);
			pcv.back().code=buff.substr(20,20);
			/* satellite */
			if (pcv.back().code.compare(3,8,"        ")==0) 
				pcv.back().sat=satid2no(pcv.back().code);
		}
		else if (buff.find("VALID FROM",60)!=string::npos) {
			if (!pcv.back().ts.str2time(buff.substr(0,43))) continue;
		}
		else if (buff.find("VALID UNTIL",60)!=string::npos) {
			if (!pcv.back().te.str2time(buff.substr(0,43))) continue;
		}
		else if (buff.find("START OF FREQUENCY",60)!=string::npos) {
			if (sscanf(buff.c_str()+4,"%d",&f)<1) continue;
			for (i=0; i<NFREQ; i++) if (freqs[i]==f) break;
			if (i<NFREQ) freq=i+1;
		}
		else if (buff.find("END OF FREQUENCY",60)!=string::npos) {
			freq=0;
		}
		else if (buff.find("NORTH / EAST / UP",60)!=string::npos) {
			double neu[3]={ 0.0 };
			if (freq<1||NFREQ<freq) continue;
			if (sscanf(buff.c_str(),"%lf %lf %lf",neu,neu+1,neu+2)<3) continue;
			pcv.back().off[freq-1][0]=1E-3*neu[pcv.back().sat ? 0 : 1]; /* x or e */
			pcv.back().off[freq-1][1]=1E-3*neu[pcv.back().sat ? 1 : 0]; /* y or n */
			pcv.back().off[freq-1][2]=1E-3*neu[2];           /* z or u */
		}
		else if (buff.find("NOAZI")!=string::npos) {
			if (freq<1||NFREQ<freq) continue;
			double *v=pcv.back().var[freq-1];
			if ((i=sscanf(buff.c_str()+8,
				"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
				v,v+1,v+2,v+3,v+4,v+5,v+6,v+7,v+8,v+9,v+10,v+11,v+12,v+13,v+14,v+15,v+16,v+17,v+18))
				<=0) continue;
			for (; i<19; i++) pcv.back().var[freq-1][i]=pcv.back().var[freq-1][i-1];
			for (i=0; i<19; i++) pcv.back().var[freq-1][i]*=1E-3;
		}
	}

	inf.close();
	return 1;
}

/* read precise ephemeris file -------------------------------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
ineph_t::ineph_t() {
	type=' ';
	ephtime=gtime_t();
	for (int i=0; i<MAXSAT; i++) satellite[i]=" ";
	bfact[0]=bfact[1]=0;
	tsys="GPS";
	ns=0;
	mask=SYS_ALL; 
	opt="";
	pred_flag=1;
}
ineph_t::ineph_t(string file,int pred){
	type=' ';
	ephtime=gtime_t();
	for (int i=0; i<MAXSAT; i++) satellite[i]=" ";
	bfact[0]=bfact[1]=0;
	tsys="GPS";
	ns=0;
	mask=SYS_ALL; 
	opt="";
	pred_flag=pred;
	inf.open(file,ios::in);
	file_path=file;
}
ineph_t::~ineph_t() {
	if (inf.is_open()) inf.close();
}
/* Implementation functions ------------------------------------------------------- */
void ineph_t::set_sysmask(const prcopt_t *prcopt){
	int i,f;
	string sys;
	
	f=opt.find("-SYS=");
	if (f==string::npos) { 
		if (prcopt) mask=prcopt->navsys; return;
		mask=SYS_ALL; return;
	}
		
	sys=opt.substr(f+5);
	for (i=0;i<(int)(sys.length())&&sys[i]!=' ';i++){
		switch (sys[i]){
			case 'G': mask|=SYS_GPS; break;
			case 'R': mask|=SYS_GLO; break;
			case 'E': mask|=SYS_GAL; break;
			case 'J': mask|=SYS_QZS; break;
			case 'C': mask|=SYS_CMP; break;
			case 'I': mask|=SYS_IRN; break;
			case 'S': mask|=SYS_SBS; break;
		}
	}
}
/* read precise ephemeris file header --------------------------------------------- */
int ineph_t::Head() {
	if (!inf.is_open()) return 0;
	int column=0,flag=1,nsat=0;
	/* name of last block (# ,##,+ ,++,%c,%f,%i,/*) */
	string last_block;

	while (getline(inf,buff)&&!inf.eof()) {
		/* stop first line of body */
		if (!buff.substr(0,2).compare("%i")||column>30) break; 
		/* 1st line of head */
		if (column==0) {
			type=buff[2];
			ephtime.str2time(buff.substr(3,28));
		}
		/* satellite list */
		else if (!buff.substr(0,2).compare("+ ")) {
			/* first line of "+  " */
			if (!last_block.compare("##")||column==2) str2int(buff.substr(4,2),ns);
			for (int i=0; i<17&&nsat<ns; i++) {
				satellite[nsat++]=buff.substr(9+i*3,3);
			}
		}
		else if (!buff.substr(0,2).compare("%c")&&!last_block.compare("++"))
			tsys=buff.substr(9,3);
		else if (!buff.substr(0,2).compare("%f")&&!last_block.compare("%c")) {
			str2double(buff.substr( 3,10),bfact[0]);
			str2double(buff.substr(14,12),bfact[1]);
		}

		last_block=buff.substr(0,2);
		column++;
	}
	return 1;
}
/* read precise ephemeris file body ----------------------------------------------- */
int ineph_t::Body(nav_t *nav) {
	if (!inf.is_open()) return 0;
	peph_t peph;
	pclk_t pclk;
	gtime_t time;
	double val,std=0.0,base;
    int sat,sys,prn,n=ns*(type=='P'?1:2),pred_o,pred_c,v;

	nav->ne=nav->nemax=0;

	while (getline(inf,buff)&&!inf.eof()) {
		if (!buff.compare("EOF")) break; //end of file

		if (buff[0]!='*'||time.str2time(buff.substr(3,28))) continue;

		if (!tsys.compare("UTC")) time.utc2gpst(); /* utc->gpst */

		peph.time = pclk.time = time;
        
        for (int i=0;i<MAXSAT;i++) {
            for (int j=0;j<4;j++) {
                peph.pos[i][j]=0.0;
                peph.std[i][j]=0.0f;
                peph.vel[i][j]=0.0;
                peph.vst[i][j]=0.0f;
            }
            for (int j=0;j<3;j++) {
                peph.cov[i][j]=0.0f;
                peph.vco[i][j]=0.0f;
            }
			pclk.clk[i]=pclk.std[i]=0.0;
        }
        for (int i=pred_o=pred_c=v=0;i<n&&getline(inf,buff);i++) {
            
            if (buff.length()<4||(buff[0]!='P'&&buff[0]!='V')) continue;
            
            sys=buff[1]==' '?SYS_GPS:code2sys(buff[1]);
            str2int(buff.substr(2,2),prn);
            if      (sys==SYS_SBS) prn+=100;
            else if (sys==SYS_QZS) prn+=192; /* extension to sp3-c */
            
			if (!(sys&mask)) continue;
            if (!(sat=satno(sys,prn))) continue;
            
            if (buff[0]=='P') {
                pred_c=buff.length()>=76&&buff[75]=='P';
                pred_o=buff.length()>=80&&buff[79]=='P';
            }
            for (int j=0;j<4;j++) {
				std=0.0;

                /* read option for predicted value */
                if (j< 3&&(pred_flag&1)&& pred_o) continue;
                if (j< 3&&(pred_flag&2)&&!pred_o) continue;
                if (j==3&&(pred_flag&1)&& pred_c) continue;
                if (j==3&&(pred_flag&2)&&!pred_c) continue;
                
				str2double(buff.substr(4+j*14,14),val);
				if (buff.length()>=80) str2double(buff.substr(61+j*3,j<3?2:3),std);
                
                if (buff[0]=='P') { /* position */
                    if (val!=0.0&&fabs(val-999999.999999)>=1E-6) {
                        peph.pos[sat-1][j]=val*(j<3?1000.0:1E-6);
                        v=1; /* valid epoch */
                    }
                    if ((base=bfact[j<3?0:1])>0.0&&std>0.0) {
                        peph.std[sat-1][j]=(float)(pow(base,std)*(j<3?1E-3:1E-12));
                    }
                }
                else if (v) { /* velocity */
                    if (val!=0.0&&fabs(val-999999.999999)>=1E-6) {
                        peph.vel[sat-1][j]=val*(j<3?0.1:1E-10);
                    }
                    if ((base=bfact[j<3?0:1])>0.0&&std>0.0) {
                        peph.vst[sat-1][j]=(float)(pow(base,std)*(j<3?1E-7:1E-16));
                    }
                }
				/* precise clock */
				if (j==3&&v) {
					pclk.clk[sat-1]=peph.pos[sat-1][j];
					pclk.std[sat-1]=peph.std[sat-1][j];
				}
            }
        }
        if (v) {
			nav->peph.push_back(peph);
			nav->ne++; nav->nemax=2*nav->ne;
			nav->pclk.push_back(pclk);
			nav->nc++; nav->ncmax=2*nav->nc;
        }
	}

	return 1;
}
/* initialization --------------------------------------------------------- */
void ineph_t::ini_readEph(string file,int pred) {
	type=' ';
	ephtime=gtime_t();
	for (int i=0; i<MAXSAT; i++) satellite[i]=" ";
	bfact[0]=bfact[1]=0;
	tsys="GPS";
	ns=0;
	mask=SYS_ALL; 
	opt="";
	pred_flag=pred;
	inf.open(file,ios::in);
	file_path=file;
}
/* open file ---------------------------------------------------------------------- */
void ineph_t::open_file(){
	inf.open(file_path,ios::in);
}
/* read precise ephemeris file ---------------------------------------------------- */
int ineph_t::readsp3(nav_t *nav,const prcopt_t *opt) {
	set_sysmask(opt);

	Head();
	Body(nav);

	return 1;
}
/* read ionex tec grid file ----------------------------------------------------------------------- */
/* Constructors ------------------------------------------------------------------- */
inionex_t::inionex_t() {
	hgts[0]=hgts[1]=450; hgts[2]=0.0;
	lats[0]=87.5; lats[1]=-87.5; lats[2]=-2.5;
	lons[0]=-180.0; lons[1]=180.0; lons[2]=5.0;
	REarth=6371.0 ;
	tec_factor=-1.0;
}
inionex_t::inionex_t(string file) {
	hgts[0]=hgts[1]=450; hgts[2]=0.0;
	lats[0]=87.5; lats[1]=-87.5; lats[2]=-2.5;
	lons[0]=-180.0; lons[1]=180.0; lons[2]=5.0;
	REarth=6371.0 ;
	tec_factor=-1.0;
	/* file and stream */
	file_path=file;
	inf.open(file,ios::in);
}
inionex_t::~inionex_t() {}
/* Implementation functions ------------------------------------------------------- */
/* data index (i:lat,j:lon,k:hgt) ----------------------------------------- */
int inionex_t::dataindex(int i,int j,int k,const int *ndata) {
	if (i<0||ndata[0]<=i||j<0||ndata[1]<=j||k<0||ndata[2]<=k) return -1;
	return i+ndata[0]*(j+ndata[1]*k);
}
/* read P1P2 DCB ------------------------------------------------------------------ */
void inionex_t::P1P2DCB(nav_t *nav) {
	int sat;

	for (int i=0;i<MAXSAT;i++) nav->cbias[i][0]=0.0;

	while (getline(inf,buff)&&!inf.eof()) {
		if (buff.length()<60) continue;

		if (buff.find("PRN / BIAS / RMS")!=string::npos) {

			if (!(sat=satid2no(buff.substr(3,3)))) {
				continue;
			}
			str2double(buff.substr(6,10),nav->cbias[sat-1][0]);
		}
		else if (buff.find("END OF AUX DATA")!=string::npos) break;
	}
}
/* read head of ionex tec grid file ----------------------------------------------- */
int inionex_t::Head(nav_t *nav) {
	if (!inf.is_open()) return 0;
	while (getline(inf,buff)&&!inf.eof()) {
		if (buff.length()<60) continue;

		if (buff.find("IONEX VERSION / TYPE")!=string::npos) {
			if (buff[20]=='I') str2double(buff.substr(0,8),version);
		}
		else if (buff.find("BASE RADIUS")!=string::npos) {
			str2double(buff.substr(0,8),REarth);
		}
		else if (buff.find("HGT1 / HGT2 / DHGT")!=string::npos) {
			str2double(buff.substr(2, 6),hgts[0]);
			str2double(buff.substr(8, 6),hgts[1]);
			str2double(buff.substr(14,6),hgts[2]);
		}
		else if (buff.find("LAT1 / LAT2 / DLAT")!=string::npos) {
			str2double(buff.substr(2, 6),lats[0]);
			str2double(buff.substr(8, 6),lats[1]);
			str2double(buff.substr(14,6),lats[2]);
		}
		else if (buff.find("LON1 / LON2 / DLON")!=string::npos) {
			str2double(buff.substr(2, 6),lons[0]);
			str2double(buff.substr(8, 6),lons[1]);
			str2double(buff.substr(14,6),lons[2]);
		}
		else if (buff.find("EXPONENT")!=string::npos) {
			str2double(buff.substr(0,6),tec_factor);
		}
		else if (buff.find("START OF AUX DATA")!=string::npos&&
			buff.find("DIFFERENTIAL CODE BIASES")!=string::npos) {
			P1P2DCB(nav);
		}
		else if (buff.find("END OF HEADER")!=string::npos) {
			return version;
		}
	}
}
/* add one epoch tec map data to nav_t ------------------------------------ */
tec_t*  inionex_t::addtec2nav(nav_t *nav) {
	int ndata[3];

	ndata[0]=nitem(lats);
	ndata[1]=nitem(lons);
	ndata[2]=nitem(hgts);
	if (ndata[0]<=1||ndata[1]<=1||ndata[2]<=0) return NULL;

	nav->tec.push_back(tec_t());
	if (nav->nt>=nav->ntmax) {
		nav->ntmax+=256;
	}
	nav->tec.back().rb=REarth;
	for (int i=0;i<3;i++) {
		nav->tec.back().ndata[i]=ndata[i];
		nav->tec.back().lats[i]=lats[i];
		nav->tec.back().lons[i]=lons[i];
		nav->tec.back().hgts[i]=hgts[i];
	}
	int n=ndata[0]*ndata[1]*ndata[2];

	nav->tec.back().data.assign(n,0.0);
	nav->tec.back().rms.assign(n,0.0);

	for (int i=0;i<n;i++) {
		nav->tec.back().data[i]=0.0;
		nav->tec.back().rms [i]=0.0f;
	}
	nav->nt++;
	return &nav->tec.back();
}
/* read body of ionex tec grid file ----------------------------------------------- */
int inionex_t::Body(nav_t *nav) {
	if (!inf.is_open()) return 0;

	tec_t *p=NULL;
	int type=0;

	while (getline(inf,buff)&&!inf.eof()) {
		if (buff.length()<60) continue;

		if (buff.find("START OF TEC MAP")!=string::npos) {
			if (p=(addtec2nav(nav))) type=1;
		}
		else if (buff.find("END OF TEC MAP")!=string::npos) {
			type=0;
			p=NULL;
		}
		else if (buff.find("START OF RMS MAP")!=string::npos) {
			type=2;
			p=NULL;
		}
		else if (buff.find("END OF RMS MAP")!=string::npos) {
			type=0;
			p=NULL;
		}
		else if (buff.find("EPOCH OF CURRENT MAP")!=string::npos) {
			if (iontime.str2time(buff.substr(0,36))) continue;

			if (type==2) {
				for (int i=nav->nt-1;i>=0;i--) {
					if (fabs(iontime.timediff(nav->tec[i].time))>=1.0) continue;
					p=&nav->tec[i];
					break;
				}
			}
			else if (p) p->time=iontime;
		}
		else if (buff.find("LAT/LON1/LON2/DLON/H")!=string::npos&&p) {
			double lat,lon[3],hgt,x;
			str2double(buff.substr( 2,6),lat);
			str2double(buff.substr( 8,6),lon[0]);
			str2double(buff.substr(14,6),lon[1]);
			str2double(buff.substr(20,6),lon[2]);
			str2double(buff.substr(26,6),hgt);

			int i=getindex(lat,p->lats);
			int k=getindex(hgt,p->hgts);
			int n=nitem(lon);

			for (int m=0;m<n;m++) {
				int index;

				if (m%16==0&&!getline(inf,buff)) break;

				int j=getindex(lon[0]+lon[2]*m,p->lons);
				if ((index=dataindex(i,j,k,p->ndata))<0) continue;

				str2double(buff.substr(m%16*5,5),x);
				if (x==9999.0) continue;

				if (type==1) p->data[index]=x*pow(10.0,tec_factor);
				else p->rms[index]=(float)(x*pow(10.0,tec_factor));
			}
		}
	}
	return 1;
}
/* initialization ----------------------------------------------------------------- */
void inionex_t::ini_rdIonex(string file) {
	hgts[0]=hgts[1]=450; hgts[2]=0.0;
	lats[0]=87.5; lats[1]=-87.5; lats[2]=-2.5;
	lons[0]=-180.0; lons[1]=180.0; lons[2]=5.0;
	REarth=6371.0 ;
	tec_factor=-1.0;
	/* file and stream */
	file_path=file;
	inf.open(file,ios::in);
}
/* open file ---------------------------------------------------------------------- */
void inionex_t::open_file() {
	inf.open(file_path,ios::in);
}
/* read ionex tec grid file ------------------------------------------------------- */
int inionex_t::readIonex(nav_t *nav) {
	if (!Head(nav)) return 0;
	if (!Body(nav)) return 0;
	inf.close();
	return 1;
}