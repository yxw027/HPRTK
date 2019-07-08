#include "crescent.h"

#include "BaseFunction/basefunction.h"

/* constant --------------------------------------------------------------------------------------- */
#define CRESSYNC    "$BIN"      /* hemis bin sync code */
#define ID_CRESPOS   1          /* hemis msg id: bin 1 position/velocity */
#define ID_CRESGLOEPH 65        /* hemis msg id: bin 65 glonass ephemeris */
#define ID_CRESGLORAW 66        /* hemis msg id: bin 66 glonass L1/L2 phase and code */
#define ID_CRESRAW2 76          /* hemis msg id: bin 76 dual-freq raw */
#define ID_CRESWAAS 80          /* hemis msg id: bin 80 waas messages */
#define ID_CRESIONUTC 94        /* hemis msg id: bin 94 ion/utc parameters */
#define ID_CRESEPH  95          /* hemis msg id: bin 95 raw ephemeris */
#define ID_CRESRAW  96          /* hemis msg id: bin 96 raw phase and code */

#define SNR2CN0_L1  30.0        /* hemis snr to c/n0 offset (db) L1 */
#define SNR2CN0_L2  30.0        /* hemis snr to c/n0 offset (db) L2 */

static const char rcsid[]="$Id: crescent.c,v 1.2 2008/07/14 00:05:05 TTAKA Exp $";

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))
static unsigned short U2(unsigned char *p) { unsigned short u; memcpy(&u,p,2); return u; }
static unsigned int   U4(unsigned char *p) { unsigned int   u; memcpy(&u,p,4); return u; }
static short          I2(unsigned char *p) { short          i; memcpy(&i,p,2); return i; }
static int            I4(unsigned char *p) { int            i; memcpy(&i,p,4); return i; }
static float          R4(unsigned char *p) { float          r; memcpy(&r,p,4); return r; }
static double         R8(unsigned char *p) { double         r; memcpy(&r,p,8); return r; }

/* input cresent raw message -------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
cres::cres(){
}
cres::~cres(){
}

/* checksum --------------------------------------------------------------------------------------- */
int cres::chksum(int len){
	unsigned short sum=0;
	int i;

	for (i=8; i<len-4; i++) sum+=buff[i];
	return (sum>>8)==buff[len-3]&&(sum&0xFF)==buff[len-4]&&
		buff[len-2]==0x0D&&buff[len-1]==0x0A;
}
/* decode bin 1 postion/velocity ---------------------------------------------------------------------
   DO NOTHING!!! ---------------------------------------------------------------------------------- */
int cres::decode_crespos(){
	int ns,week,mode;
	double tow,pos[3],vel[3],std;
	string tstr;
	unsigned char *p=buff+8;
	gtime_t ttt;

	if (len!=64) {
		return -1;
	}
	ns  =U1(p+1);
	week=U2(p+2);
	tow =R8(p+4);
	pos[0]=R8(p+12);
	pos[1]=R8(p+20);
	pos[2]=R4(p+28);
	vel[0]=R4(p+32);
	vel[1]=R4(p+36);
	vel[2]=R4(p+40);
	std =R4(p+44);
	mode=U2(p+48);

	ttt.gpst2time(week,tow)->time2str(3);
	tstr = ttt.sep;

	return 0;
}
/* decode bin 96 raw phase and code --------------------------------------- */
int cres::decode_cresraw(){
	gtime_t ttt;
	double tow,tows,toff=0.0,cp,pr,dop,snr;
	int i,j,n,prn,sat,week,word2,lli=0;
	unsigned int word1,sn,sc;
	unsigned char *p=buff+8;

	if (len!=312) {
		return -1;
	}
	week=U2(p+2);
	tow =R8(p+4);
	tows=floor(tow*1000.0+0.5)/1000.0; /* round by 1ms */
	ttt.gpst2time(week,tows);

	/* time tag offset correction */
	if (opt.find("-TTCORR")!=string::npos) {
		toff=CLIGHT*(tows-tow);
	}
	for (i=n=0,p+=12; i<12&&n<MAXOBS; i++,p+=24) {
		word1=U4(p);
		word2=I4(p+4);
		if ((prn=word1&0xFF)==0) continue; /* if 0, no data */
		if (!(sat=satno(prn<=MAXPRNGPS ? SYS_GPS : SYS_SBS,prn))) {
			continue;
		}
		pr=R8(p+ 8)-toff;
		cp=R8(p+16)-toff;
		if (!(word2&1)) cp=0.0; /* invalid phase */
		sn =(word1>>8)&0xFF;
		snr=sn==0 ? 0.0 : 10.0*log10(0.8192*sn)+SNR2CN0_L1;
		sc =(unsigned int)(word1>>24);
		if (time.time!=0) {
			lli=(int)((unsigned char)sc-(unsigned char)lockt[sat-1][0])>0;
		}
		lockt[sat-1][0]=(unsigned char)sc;
		dop=word2/16/4096.0;

		obs.data[n].time=ttt;
		obs.data[n].sat =sat;
		obs.data[n].P[0]=pr;
		obs.data[n].L[0]=cp/WaveLengths[0];
		obs.data[n].D[0]=-(float)(dop/WaveLengths[0]);
		obs.data[n].SNR[0]=(unsigned char)(snr*4.0+0.5);
		obs.data[n].LLI[0]=(unsigned char)lli;
		obs.data[n].code[0]=CODE_L1C;

		for (j=1; j<NFREQ; j++) {
			obs.data[n].L[j]=obs.data[n].P[j]=0.0;
			obs.data[n].D[j]=0.0;
			obs.data[n].SNR[j]=obs.data[n].LLI[j]=0;
			obs.data[n].code[j]=CODE_NONE;
		}
		n++;
	}
	time=ttt;
	obs.n=n;
	return 1;
}
/* decode bin 76 dual-freq raw phase and code ----------------------------- */
int cres::decode_cresraw2(){
	gtime_t ttt;
	double tow,tows,toff=0.0,cp[2]={ 0 },pr1,pr[2]={ 0 },dop[2]={ 0 },snr[2]={ 0 };
	int i,j,n=0,prn,sat,week,lli[2]={ 0 };
	unsigned int word1,word2,word3,sc,sn;
	unsigned char *p=buff+8;

	if (len!=460) {
		return -1;
	}
	tow =R8(p);
	week=U2(p+8);
	tows=floor(tow*1000.0+0.5)/1000.0; /* round by 1ms */
	ttt.gpst2time(week,tows);

	/* time tag offset correction */
	if (opt.find("-TTCORR")!=string::npos) {
		toff=CLIGHT*(tows-tow);
	}
	if (fabs(ttt.timediff(time))<1e-9) {
		n=obs.n;
	}
	for (i=0,p+=16; i<15&&n<MAXOBS; i++) {
		word1=U4(p+324+4*i); /* L1CACodeMSBsPRN */
		if ((prn=word1&0xFF)==0) continue; /* if 0, no data */
		if (!(sat=satno(prn<=MAXPRNGPS ? SYS_GPS : SYS_SBS,prn))) {
			continue;
		}
		pr1=(word1>>13)*256.0; /* upper 19bit of L1CA pseudorange */

		word1=U4(p+144+12*i); /* L1CASatObs */
		word2=U4(p+148+12*i);
		word3=U4(p+152+12*i);
		sn=word1&0xFFF;
		snr[0]=sn==0 ? 0.0 : 10.0*log10(0.1024*sn)+SNR2CN0_L1;
		sc=(unsigned int)(word1>>24);
		if (time.time!=0) {
			lli[0]=(int)((unsigned char)sc-(unsigned char)lockt[sat-1][0])>0;
		}
		else {
			lli[0]=0;
		}
		lli[0]|=((word1>>12)&7) ? 2 : 0;
		lockt[sat-1][0]=(unsigned char)sc;
		dop[0]=((word2>>1)&0x7FFFFF)/512.0;
		if ((word2>>24)&1) dop[0]=-dop[0];
		pr[0]=pr1+(word3&0xFFFF)/256.0;
		cp[0]=floor(pr[0]/WaveLengths[0]/8192.0)*8192.0;
		cp[0]+=((word2&0xFE000000)+((word3&0xFFFF0000)>>7))/524288.0;
		if (cp[0]-pr[0]/WaveLengths[0]<-4096.0) cp[0]+=8192.0;
		else if (cp[0]-pr[0]/WaveLengths[0]> 4096.0) cp[0]-=8192.0;

		if (i<12) {
			word1=U4(p  +12*i); /* L2PSatObs */
			word2=U4(p+4+12*i);
			word3=U4(p+8+12*i);
			sn=word1&0xFFF;
			snr[1]=sn==0 ? 0.0 : 10.0*log10(0.1164*sn)+SNR2CN0_L2;
			sc=(unsigned int)(word1>>24);
			if (time.time==0) {
				lli[1]=(int)((unsigned char)sc-(unsigned char)lockt[sat-1][1])>0;
			}
			else {
				lli[1]=0;
			}
			lli[1]|=((word1>>12)&7) ? 2 : 0;
			lockt[sat-1][1]=(unsigned char)sc;
			dop[1]=((word2>>1)&0x7FFFFF)/512.0;
			if ((word2>>24)&1) dop[1]=-dop[1];
			pr[1]=(word3&0xFFFF)/256.0;
			if (pr[1]!=0.0) {
				pr[1]+=pr1;
				if (pr[1]-pr[0]<-128.0) pr[1]+=256.0;
				else if (pr[1]-pr[0]> 128.0) pr[1]-=256.0;
				cp[1]=floor(pr[1]/WaveLengths[1]/8192.0)*8192.0;
				cp[1]+=((word2&0xFE000000)+((word3&0xFFFF0000)>>7))/524288.0;
				if (cp[1]-pr[1]/WaveLengths[1]<-4096.0) cp[1]+=8192.0;
				else if (cp[1]-pr[1]/WaveLengths[1]> 4096.0) cp[1]-=8192.0;
			}
			else cp[1]=0.0;
		}
		obs.data[n].time=ttt;
		obs.data[n].sat =sat;
		for (j=0; j<NFREQ; j++) {
			if (j==0||(j==1&&i<12)) {
				obs.data[n].P[j]=pr[j]==0.0 ? 0.0 : pr[j]-toff;
				obs.data[n].L[j]=cp[j]==0.0 ? 0.0 : cp[j]-toff/WaveLengths[j];
				obs.data[n].D[j]=-(float)dop[j];
				obs.data[n].SNR[j]=(unsigned char)(snr[j]*4.0+0.5);
				obs.data[n].LLI[j]=(unsigned char)lli[j];
				obs.data[n].code[j]=j==0 ? CODE_L1C : CODE_L2P;
			}
			else {
				obs.data[n].L[j]=obs.data[n].P[j]=0.0;
				obs.data[n].D[j]=0.0;
				obs.data[n].SNR[j]=obs.data[n].LLI[j]=0;
				obs.data[n].code[j]=CODE_NONE;
			}
		}
		n++;
	}
	time=ttt;
	obs.n=n;
	if (opt.find("-ENAGLO")!=string::npos) return 0; /* glonass follows */
	return 1;
}
/* decode bin 95 ephemeris ------------------------------------------------ */
int cres::decode_creseph(){
	decode_frame dec;
	unsigned int word;
	int i,j,k,prn,sat;
	unsigned char *p=buff+8,bbuff[90];

	if (len!=140) {
		return -1;
	}
	prn=U2(p);
	if (!(sat=satno(SYS_GPS,prn))) {
		return -1;
	}
	for (i=0; i<3; i++) for (j=0; j<10; j++) {
		word=U4(p+8+i*40+j*4)>>6;
		for (k=0; k<3; k++) bbuff[i*30+j*3+k]=(unsigned char)((word>>(8*(2-k)))&0xFF);
	}
	if (dec.decode(bbuff)!=1||
		dec.decode(bbuff+30)!=2||
		dec.decode(bbuff+60)!=3) {
		return -1;
	}
	if (opt.find("-EPHALL")==string::npos) {
		if (dec.eph.iode==nav.eph[sat-1].iode) return 0; /* unchanged */
	}
	dec.eph.sat=sat;
	nav.eph[sat-1]=dec.eph;
	ephsat=sat;
	return 2;
}
/* decode bin 94 ion/utc parameters --------------------------------------- */
int cres::decode_cresionutc(){
	int i;
	unsigned char *p=buff+8;

	if (len!=108) {
		return -1;
	}
	for (i=0; i<8; i++) nav.ion_gps[i]=R8(p+i*8);
	nav.utc_gps[0]=R8(p+64);
	nav.utc_gps[1]=R8(p+72);
	nav.utc_gps[2]=(double)U4(p+80);
	nav.utc_gps[3]=(double)U2(p+84);
	nav.leaps=I2(p+90);
	return 9;
}
/* decode bin 80 waas messages -------------------------------------------- */
int cres::decode_creswaas(){
	double tow;
	unsigned int word;
	int i,j,k,prn;
	unsigned char *p=buff+8;

	if (len!=52) {
		return -1;
	}
	prn=U2(p);
	if (prn<MINPRNSBS||MAXPRNSBS<prn) {
		return -1;
	}
	sbsmsg.prn=prn;
	sbsmsg.tow=U4(p+4);
	tow=time.time2gpst(&sbsmsg.week);
	if (sbsmsg.tow<tow-302400.0) sbsmsg.week++;
	else if (sbsmsg.tow>tow+302400.0) sbsmsg.week--;

	for (i=k=0; i<8&&k<29; i++) {
		word=U4(p+8+i*4);
		for (j=0; j<4&&k<29; j++) sbsmsg.msg[k++]=(unsigned char)(word>>(3-j)*8);
	}
	sbsmsg.msg[28]&=0xC0;
	return 3;
}
/* decode bin 66 glonass L1/L2 code and carrier phase --------------------- */
int cres::decode_cresgloraw(){
	gtime_t ttt;
	double tow,tows,toff=0.0,cp[2]={ 0 },pr1,pr[2]={ 0 },dop[2]={ 0 },snr[2]={ 0 };
	int i,j,n=0,prn,sat,week,lli[2]={ 0 };
	unsigned int word1,word2,word3,sc,sn;
	unsigned char *p=buff+8;

	if (opt.find("-ENAGLO")==string::npos) return 0;

	if (len!=364) {
		return -1;
	}
	tow =R8(p);
	week=U2(p+8);
	tows=floor(tow*1000.0+0.5)/1000.0; /* round by 1ms */
	ttt.gpst2time(week,tows);

	/* time tag offset correction */
	if (opt.find("-TTCORR")!=string::npos) {
		toff=CLIGHT*(tows-tow);
	}
	if (fabs(ttt.timediff(time))<1e-9) {
		n=obs.n;
	}
	for (i=0,p+=16; i<12&&n<MAXOBS; i++) {
		word1=U4(p+288+4*i); /* L1CACodeMSBsSlot */
		if ((prn=word1&0xFF)==0) continue; /* if 0, no data */
		if (!(sat=satno(SYS_GLO,prn))) {
			continue;
		}
		pr1=(word1>>13)*256.0; /* upper 19bit of L1CA pseudorange */

							   /* L1Obs */
		word1=U4(p  +12*i);
		word2=U4(p+4+12*i);
		word3=U4(p+8+12*i);
		sn=word1&0xFFF;
		snr[0]=sn==0 ? 0.0 : 10.0*log10(0.1024*sn)+SNR2CN0_L1;
		sc=(unsigned int)(word1>>24);
		if (time.time!=0) {
			lli[0]=(int)((unsigned char)sc-(unsigned char)lockt[sat-1][0])>0;
		}
		else {
			lli[0]=0;
		}
		lli[0]|=((word1>>12)&7) ? 2 : 0;
		lockt[sat-1][0]=(unsigned char)sc;
		dop[0]=((word2>>1)&0x7FFFFF)/512.0;
		if ((word2>>24)&1) dop[0]=-dop[0];
		pr[0]=pr1+(word3&0xFFFF)/256.0;
		cp[0]=floor(pr[0]/WaveLengths[0]/8192.0)*8192.0;
		cp[0]+=((word2&0xFE000000)+((word3&0xFFFF0000)>>7))/524288.0;
		if (cp[0]-pr[0]/WaveLengths[0]<-4096.0) cp[0]+=8192.0;
		else if (cp[0]-pr[0]/WaveLengths[0]> 4096.0) cp[0]-=8192.0;

		/* L2Obs */
		word1=U4(p+144+12*i);
		word2=U4(p+148+12*i);
		word3=U4(p+152+12*i);
		sn=word1&0xFFF;
		snr[1]=sn==0 ? 0.0 : 10.0*log10(0.1164*sn)+SNR2CN0_L2;
		sc=(unsigned int)(word1>>24);
		if (time.time==0) {
			lli[1]=(int)((unsigned char)sc-(unsigned char)lockt[sat-1][1])>0;
		}
		else {
			lli[1]=0;
		}
		lli[1]|=((word1>>12)&7) ? 2 : 0;
		lockt[sat-1][1]=(unsigned char)sc;
		dop[1]=((word2>>1)&0x7FFFFF)/512.0;
		if ((word2>>24)&1) dop[1]=-dop[1];
		pr[1]=(word3&0xFFFF)/256.0;
		if (pr[1]!=0.0) {
			pr[1]+=pr1;
			if (pr[1]-pr[0]<-128.0) pr[1]+=256.0;
			else if (pr[1]-pr[0]> 128.0) pr[1]-=256.0;
			cp[1]=floor(pr[1]/WaveLengths[1]/8192.0)*8192.0;
			cp[1]+=((word2&0xFE000000)+((word3&0xFFFF0000)>>7))/524288.0;
			if (cp[1]-pr[1]/WaveLengths[1]<-4096.0) cp[1]+=8192.0;
			else if (cp[1]-pr[1]/WaveLengths[1]> 4096.0) cp[1]-=8192.0;
		}
		obs.data[n].time=ttt;
		obs.data[n].sat =sat;
		for (j=0; j<NFREQ; j++) {
			if (j==0||(j==1&&i<12)) {
				obs.data[n].P[j]=pr[j]==0.0 ? 0.0 : pr[j]-toff;
				obs.data[n].L[j]=cp[j]==0.0 ? 0.0 : cp[j]-toff/WaveLengths[j];
				obs.data[n].D[j]=-(float)dop[j];
				obs.data[n].SNR[j]=(unsigned char)(snr[j]*4.0+0.5);
				obs.data[n].LLI[j]=(unsigned char)lli[j];
				obs.data[n].code[j]=j==0 ? CODE_L1C : CODE_L2P;
			}
			else {
				obs.data[n].L[j]=obs.data[n].P[j]=0.0;
				obs.data[n].D[j]=0.0;
				obs.data[n].SNR[j]=obs.data[n].LLI[j]=0;
				obs.data[n].code[j]=CODE_NONE;
			}
		}
		n++;
	}
	time=ttt;
	obs.n=n;
	return 1;
}
/* decode bin 65 glonass ephemeris ---------------------------------------- */
int cres::decode_cresgloeph(){
	geph_t geph;
	unsigned char *p=buff+8,str[12];
	int i,j,k,sat,prn,frq,ttt,no;

	if (opt.find("-ENAGLO")==string::npos) return 0;

	prn =U1(p);   p+=1;
	frq =U1(p)-8; p+=1+2;
	ttt=U4(p);   p+=4;

	if (!(sat=satno(SYS_GLO,prn))) {
		return -1;
	}
	for (i=0; i<5; i++) {
		for (j=0; j<3; j++) for (k=3; k>=0; k--) {
			str[k+j*4]=U1(p++);
		}
		if ((no=getbitu(str,1,4))!=i+1) {
			return -1;
		}
		memcpy(subfrm[sat-1]+10*i,str,10);
	}
	/* decode glonass ephemeris strings */
	geph.tof=time;
	if (!decode_glostr(sat,geph)||geph.sat!=sat) return -1;
	geph.frq=frq;

	if (opt.find("-EPHALL")==string::npos) {
		if (geph.iode==nav.geph[prn-1].iode) return 0; /* unchanged */
	}
	nav.geph[prn-1]=geph;
	ephsat=sat;
	return 2;
}
/* sync code -------------------------------------------------------------------------------------- */
int cres::sync_cres(unsigned char data){
	buff[0]=buff[1]; buff[1]=buff[2]; buff[2]=buff[3]; buff[3]=data;
	return buff[0]==CRESSYNC[0]&&buff[1]==CRESSYNC[1]&&
		buff[2]==CRESSYNC[2]&&buff[3]==CRESSYNC[3];
}
/* decode crescent raw message -------------------------------------------- */
int cres::decode_cres(){
	int type=U2(buff+4);
	string str;

	if (!chksum(len)) {
		return -1;
	}
	if (outtype) {
		msgtype="HEMIS "+int2str(2," ",type,str)+" ("+int2str(4," ",len,str)+"):";
	}
	switch (type) {
		case ID_CRESPOS: return decode_crespos();
		case ID_CRESRAW: return decode_cresraw();
		case ID_CRESRAW2: return decode_cresraw2();
		case ID_CRESEPH: return decode_creseph();
		case ID_CRESWAAS: return decode_creswaas();
		case ID_CRESIONUTC: return decode_cresionutc();
		case ID_CRESGLORAW: return decode_cresgloraw();
		case ID_CRESGLOEPH: return decode_cresgloeph();
	}
	return 0;
}

/* input cresent raw message ---------------------------------------------------------------------- */
int cres::decode(unsigned char data){
	/* synchronize frame */
	if (nbyte==0) {
		if (!sync_cres(data)) return 0;
		nbyte=4;
		return 0;
	}
	buff[nbyte++]=data;

	if (nbyte==8) {
		if ((len=U2(buff+6)+12)>MAXRAWLEN) {
			nbyte=0;
			return -1;
		}
	}
	if (nbyte<8||nbyte<len) return 0;
	nbyte=0;

	/* decode crescent raw message */
	return decode_cres();
}