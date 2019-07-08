#include "RtkStream/stream.h"

#include "BaseFunction/basefunction.h"

/* Constant */
static const string rcsid="$Id$";

/* constants -------------------------------------------------------------------------------------- */

#define TINTACT             200         /* period for stream active (ms) */
#define SERIBUFFSIZE        4096        /* serial buffer size (bytes) */
#define TIMETAGH_LEN        64          /* time tag file header length */
#define MAXCLI              32          /* max client connection for tcp svr */
#define MAXSTATMSG          32          /* max length of status message */
#define DEFAULT_MEMBUF_SIZE 4096        /* default memory buffer size (bytes) */

#define NTRIP_AGENT         "MPP/" "1.0.1"
#define NTRIP_CLI_PORT      2101        /* default ntrip-client connection port */
#define NTRIP_SVR_PORT      80          /* default ntrip-server connection port */
#define NTRIP_MAXRSP        32768       /* max size of ntrip response */
#define NTRIP_MAXSTR        256         /* max length of mountpoint string */
#define NTRIP_RSP_OK_CLI    "ICY 200 OK\r\n" /* ntrip response: client */
#define NTRIP_RSP_OK_SVR    "OK\r\n"    /* ntrip response: server */
#define NTRIP_RSP_SRCTBL    "SOURCETABLE 200 OK\r\n" /* ntrip response: source table */
#define NTRIP_RSP_TBLEND    "ENDSOURCETABLE"
#define NTRIP_RSP_HTTP      "HTTP/"     /* ntrip response: http */
#define NTRIP_RSP_ERROR     "ERROR"     /* ntrip response: error */
#define NTRIP_RSP_UNAUTH    "HTTP/1.0 401 Unauthorized\r\n"
#define NTRIP_RSP_ERR_PWD   "ERROR - Bad Pasword\r\n"
#define NTRIP_RSP_ERR_MNTP  "ERROR - Bad Mountpoint\r\n"

#define FTP_CMD             "wget"      /* ftp/http command */
#define FTP_TIMEOUT         30          /* ftp/http timeout (s) */

#define MIN(x,y)            ((x)<(y)?(x):(y))
/* global options ------------------------------------------------------------*/

static int inactout =10000;				/* inactive timeout (ms) */
static int ticonnect=10000;				/* interval to re-connect (ms) */
static int tirate   =1000;				/* avraging time for data rate (ms) */
static int resebuff =32768;				/* receive/send buffer size (bytes) */
static string localdir="\0";			/* local directory for ftp/http (1024) */
static string proxyaddr="\0";			/* http/ntrip/ftp proxy address (256) */
static unsigned int tick_master=0;		/* time tick master for replay */
static int fswapmargin=30;				/* file swap margin (s) */

										/* get socket error --------------------------------------------------------- */
#ifdef WIN32
static int errsock(void) { return WSAGetLastError(); }
#else
static int errsock(void) { return errno; }
#endif

/* ftp thread ------------------------------------------------------------------------------------- */
#ifdef WIN32
static DWORD WINAPI ftpthread(void *arg){
#else
static void *ftpthread(void *arg){
#endif

	ftp_t *ftp=(ftp_t *)arg;
	fstream inf;
	gtime_t time;
	string remote,local,tmpfile,errfile;
	string cmd,env="",opt,proxyopt="",proto;
	size_t p;
	int ret;

	if (localdir=="") {
		ftp->error=11;
		ftp->state=3;
		return 0;
	}
	/* replace keyword in file path and local path */
	time.timeget()->utc2gpst()->timeadd(ftp->topts[0]);
	reppath(ftp->file,remote,time,"","");

	if ((p=remote.find('/'))!=string::npos) p++; else p=0;
	local=localdir+FILEPATHSEP+remote.substr(p);
	errfile=local+".err";

	/* if local file exist, skip download */
	tmpfile=local;
	if ((p=tmpfile.find('.')!=string::npos)&&
		(!tmpfile.substr(p).compare(".z")||!tmpfile.substr(p).compare(".Z")||
			!tmpfile.substr(p).compare(".gz")||!tmpfile.substr(p).compare(".GZ")||
			!tmpfile.substr(p).compare(".zip")||!tmpfile.substr(p).compare(".ZIP"))) {
		tmpfile=tmpfile.substr(0,p);
	}
	inf.open(tmpfile,ios::in|ios::binary);
	if (inf.is_open()) {
		inf.close();
		ftp->local=tmpfile;
		ftp->state=2;
		return 0;
	}
	/* proxy settings for wget (ref [2]) */
	if (proxyaddr!="") {
		proto=ftp->proto ? "http" : "ftp";
		env="set "+proto+"_proxy=http://"+proxyaddr+" & ";
		proxyopt="--proxy=on ";
	}
	/* download command (ref [2]) */
	if (ftp->proto==0) { /* ftp */
		opt="--ftp-user="+ftp->user+" --ftp-password="+ftp->passwd+" --glob=off --passive-ftp "+
			proxyopt+"-t 1 -T "+to_string(FTP_TIMEOUT)+" -O \""+local+"\"";
		cmd=env+FTP_CMD+" "+opt+" \"ftp://"+ftp->addr+"/"+remote+"\" 2> \""+errfile+"\"\n";
	}
	else { /* http */
		opt=proxyopt+"-t 1 -T "+to_string(FTP_TIMEOUT)+" -O \""+local+"\"";
		cmd=env+FTP_CMD+" "+opt+" \"http://"+ftp->addr+"/"+remote+"\" 2> \""+errfile+"\"\n";
	}
	/* execute download command */
	if ((ret=execcmd(cmd))) {
		remove(local.c_str());
		ftp->error=ret;
		ftp->state=3;
		return 0;
	}
	remove(errfile.c_str());

	/* uncompress downloaded file */
	if ((p=local.find('.')!=string::npos)&&
		(!local.substr(p).compare(".z")||!local.substr(p).compare(".Z")||
			!local.substr(p).compare(".gz")||!local.substr(p).compare(".GZ")||
			!local.substr(p).compare(".zip")||!local.substr(p).compare(".ZIP"))) {

		if (rtk_uncompress(local,tmpfile)) {
			remove(local.c_str());
			local=tmpfile;
		}
		else {
			ftp->error=12;
			ftp->state=3;
			return 0;
		}
	}
	ftp->local=local;
	ftp->state=2; /* ftp completed */

	return 0;
}

/* base64 encoder --------------------------------------------------------------------------------- */
static int encbase64(char *str,const unsigned char *byte,int n)
{
	const char table[]=
		"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	int i,j,k,b;

	for (i=j=0; i/8<n;) {
		for (k=b=0; k<6; k++,i++) {
			b<<=1; if (i/8<n) b|=(byte[i/8]>>(7-i%8))&0x1;
		}
		str[j++]=table[b];
	}
	while (j&0x3) str[j++]='=';
	str[j]='\0';

	return j;
}

/* stream type ---------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
Constructor --------------------------------------------------------------------------------------- */
stream_t::stream_t(){
	Stype=Smode=Sstate=0;
	inb=inr=outb=outr=tick_i=tick_o=tact=inbt=outbt=0;
	initlock(&Slock);
}
stream_t::~stream_t(){
}
/* Implementation functions ----------------------------------------------------------------------- */
/* lock/unlock stream ----------------------------------------------------------------------------- */
void stream_t::strlock(){
	tolock(&Slock);
}
void stream_t::strunlock(){
	tounlock(&Slock);
}
/* set bitrate ------------------------------------------------------------ */
int stream_t::set_brate(int brate){
	string strpath=path,strbuff;
	int strtype=Stype,strmode=Smode;
	size_t sp1,sp2;

	if (strtype!=STR_SERIAL) return 0;

	if ((sp1=strpath.find(':'))==string::npos){
		strpath+=":"+to_string(brate);
	}
	else {
		if ((sp2=strpath.substr(sp1+1).find(':'))!=string::npos)
			strpath.insert(sp1,":"+to_string(brate));
	}

	StreamClose();
	return StreamOpen(strpath.c_str(),strtype,strmode);
}

/* TCP functions ---------------------------------------------------------------------------------- */
/* decode tcp/ntrip path (path=[user[:passwd]@]addr[:port][/mntpnt[:str]]) -------- */
void stream_t::decodetcppath(const string path,string *Addr,string *Port,string *User,
	string *Passwd,string *Mntpnt,string *Strp){

	size_t sp1=0,sp2=0;
	string bufpath=path;

	/* read user and passwd */
	if ((sp1=bufpath.rfind('@'))!=string::npos){
		if ((sp2=bufpath.find(':'))!=string::npos && sp2<sp1){
			if (User) *User=bufpath.substr(0,sp2);
			if (Passwd) *Passwd=bufpath.substr(sp2+1,sp1-sp2-1);
		}
		else if (User) *User=bufpath.substr(0,sp1);
		/* erase bufpath */
		bufpath.erase(0,sp1+1);
	}
	/* read mntpnt and str */
	if ((sp1=bufpath.find('/'))!=string::npos){
		if ((sp2=bufpath.rfind(':'))!=string::npos&&sp2>sp1){
			if (Mntpnt) *Mntpnt=bufpath.substr(sp1+1,sp2-sp1-1);
			if (Strp) *Strp=bufpath.substr(sp2+1);
		}
		else if (Mntpnt) *Mntpnt=bufpath.substr(sp1+1);
		/* erase bufpath */
		bufpath.erase(sp1);
	}
	/* read addr and port */
	if ((sp1=bufpath.find(':'))!=string::npos){
		if (Addr) *Addr=bufpath.substr(0,sp1);
		if (Port) *Port=bufpath.substr(sp1+1);
	}
	else if (Addr) *Addr=bufpath;
}
/* set socket option ------------------------------------------------------------------------------ */
int stream_t::setsock(socket_t sock){
	int bs=resebuff,kmode=1;
#ifdef WIN32
	int tv=0;
#else
	struct timeval tv={ 0 };
#endif

	if (setsockopt(sock,SOL_SOCKET,SO_RCVTIMEO,(const char *)&tv,sizeof(tv))==-1||
		setsockopt(sock,SOL_SOCKET,SO_SNDTIMEO,(const char *)&tv,sizeof(tv))==-1) {
		msg="sockopt error: notimeo";
		closesocket(sock);
		return 0;
	}
	if (setsockopt(sock,SOL_SOCKET,SO_RCVBUF,(const char *)&bs,sizeof(bs))==-1||
		setsockopt(sock,SOL_SOCKET,SO_SNDBUF,(const char *)&bs,sizeof(bs))==-1) {
		msg="sockopt error: bufsiz";
	}
	if (setsockopt(sock,IPPROTO_TCP,TCP_NODELAY,(const char *)&kmode,sizeof(kmode))==-1) {
		msg="sockopt error: nodelay";
	}
	return 1;
}
/* non-block accept ------------------------------------------------------------------------------- */
socket_t stream_t::accept_nb(socket_t sock,struct sockaddr *Saddr,socklen_t *len){
	struct timeval tv={ 0 };
	fd_set rs;
	int ret;

	FD_ZERO(&rs); FD_SET(sock,&rs);
	ret=select(sock+1,&rs,NULL,NULL,&tv);
	if (ret<=0) return (socket_t)ret;
	return accept(sock,Saddr,len);
}
/* non-block connect ------------------------------------------------------------------------------ */
int stream_t::connect_nb(socket_t sock,struct sockaddr *Saddr,socklen_t len){
#ifdef WIN32
	u_long umode=1;
	int err;

	ioctlsocket(sock,FIONBIO,&umode);
	if (connect(sock,Saddr,len)==-1) {
		err=errsock();
		if (err==WSAEWOULDBLOCK||err==WSAEINPROGRESS||
			err==WSAEALREADY   ||err==WSAEINVAL) return 0;
		if (err!=WSAEISCONN) return -1;
	}
#else
	struct timeval tv={ 0 };
	fd_set rs,ws;
	int err,flag;

	flag=fcntl(sock,F_GETFL,0);
	fcntl(sock,F_SETFL,flag|O_NONBLOCK);
	if (connect(sock,Saddr,len)==-1) {
		err=errsock();
		if (err!=EISCONN&&err!=EINPROGRESS&&err!=EALREADY) return -1;
		FD_ZERO(&rs); FD_SET(sock,&rs); ws=rs;
		if (select(sock+1,&rs,&ws,NULL,&tv)==0) return 0;
	}
#endif
	return 1;
}
/* non-block receive ------------------------------------------------------------------------------ */
int stream_t::recv_nb(socket_t sock, unsigned char *buff,int n){
	struct timeval tv={ 0 };
	fd_set rs;
	int ret,nr;

	FD_ZERO(&rs); FD_SET(sock,&rs);
	ret=select(sock+1,&rs,NULL,NULL,&tv);
	if (ret<=0) return ret;
	nr=recv(sock,(char *)buff,n,0);
	return nr<=0 ? -1 : nr;
}
/* non-block send --------------------------------------------------------------------------------- */
int stream_t::send_nb(socket_t sock,unsigned char *buff,int n){
	struct timeval tv={ 0 };
	fd_set ws;
	int ret,ns;

	FD_ZERO(&ws); FD_SET(sock,&ws);
	ret=select(sock+1,NULL,&ws,NULL,&tv);
	if (ret<=0) return ret;
	ns=send(sock,(char *)buff,n,0);
	return ns<n ? -1 : ns;
}
/* generate tcp socket ---------------------------------------------------------------------------- */
int stream_t::gentcp(tcp_t &tcp,int type){
	struct hostent *hp;
#ifdef SVR_REUSEADDR
	int opt=1;
#endif


	/* generate socket */
	if ((tcp.sock=socket(AF_INET,SOCK_STREAM,0))==(socket_t)-1) {
		msg="socket error "; msg+=to_string(errsock());
		tcp.state=-1;
		return 0;
	}
	if (!setsock(tcp.sock)) {
		tcp.state=-1;
		return 0;
	}
	memset(&tcp.addr,0,sizeof(tcp.addr));
	tcp.addr.sin_family=AF_INET;
	tcp.addr.sin_port=htons(tcp.port);

	if (type==0) { /* server socket */

#ifdef SVR_REUSEADDR
				   /* multiple-use of server socket */
		setsockopt(tcp.sock,SOL_SOCKET,SO_REUSEADDR,(const char *)&opt,
			sizeof(opt));
#endif
		if (bind(tcp.sock,(struct sockaddr *)&tcp.addr,sizeof(tcp.addr))==-1) {
			msg="bind error "; msg+=to_string(errsock());
			msg+=" : "; msg+=to_string(tcp.port);
			closesocket(tcp.sock);
			tcp.state=-1;
			return 0;
		}
		listen(tcp.sock,5);
	}
	else { /* client socket */
		if (!(hp=gethostbyname(tcp.saddr.c_str()))) {
			msg="address error (";msg+=tcp.saddr+")";
			closesocket(tcp.sock);
			tcp.state=0;
			tcp.tcon=ticonnect;
			tcp.tdis=tickget();
			return 0;
		}
		memcpy(&tcp.addr.sin_addr,hp->h_addr,hp->h_length);
	}
	tcp.state=1;
	tcp.tact=tickget();
	return 1;
}
/* disconnect tcp --------------------------------------------------------------------------------- */
void stream_t::discontcp(tcp_t &tcp,int tcon){
	closesocket(tcp.sock);
	tcp.state=0;
	tcp.tcon=tcon;
	tcp.tdis=tickget();
}
/* special functions -------------------------------------------------------------- */
/* get stream from file_t time (only for file_t) ---------------------------------- */
gtime_t stream_t::strgettime(){
	gtime_t nothing;
	return nothing;
}
/* sync file_t stream with another (only for file_t) ---------------- */
void stream_t::strsync(void *str_file){
}
/* common functions --------------------------------------------------------------- */
/* send receiver command ---------------------------------------------------------- */
void stream_t::SendCmd(const char *cmd){
	unsigned char cbuf[1024];
	const char *p=cmd,*q;
	char cmdmsg[1024],cmdend[]="\r\n";
	int n,m,ms,brate;

	for (;;) {
		for (q=p;; q++) if (*q=='\r'||*q=='\n'||*q=='\0') break;
		n=(int)(q-p); strncpy(cmdmsg,p,n); cmdmsg[n]='\0';

		if (!*cmdmsg||*cmdmsg=='#') { /* null or comment */
			;
		}
		else if (*cmdmsg=='!') { /* binary escape */

			if (!strncmp(cmdmsg+1,"WAIT",4)) { /* wait */
				if (sscanf(cmdmsg+5,"%d",&ms)<1) ms=100;
				if (ms>3000) ms=3000; /* max 3 s */
				sleepms(ms);
			}
			else if (!strncmp(cmdmsg+1,"BRATE",5)) { /* set bitrate */
				if (sscanf(cmdmsg+6,"%d",&brate)<1) brate=9600;
				set_brate(brate);
				sleepms(500);
			}
			else if (!strncmp(cmdmsg+1,"UBX",3)) { /* ublox */
				if ((m=gen_ubx(cmdmsg+4,cbuf))>0) StreamWrite(cbuf,m);
			}
			else if (!strncmp(cmdmsg+1,"STQ",3)) { /* skytraq */
				if ((m=gen_stq(cmdmsg+4,cbuf))>0) StreamWrite(cbuf,m);
			}
			else if (!strncmp(cmdmsg+1,"NVS",3)) { /* nvs */
				if ((m=gen_nvs(cmdmsg+4,cbuf))>0) StreamWrite(cbuf,m);
			}
			else if (!strncmp(cmdmsg+1,"LEXR",4)) { /* lex receiver */
				if ((m=gen_lexr(cmdmsg+5,cbuf))>0) StreamWrite(cbuf,m);
			}
			else if (!strncmp(cmdmsg+1,"HEX",3)) { /* general hex message */
				if ((m=gen_hex(cmdmsg+4,cbuf))>0) StreamWrite(cbuf,m);
			}
		}
		else {
			StreamWrite((unsigned char *)cmdmsg,n);
			StreamWrite((unsigned char *)cmdend,2);
		}
		if (*q=='\0') break; else p=q+1;
	}
}
/* open stream from kinds of format ----------------------------------------------- */
int stream_t::StreamOpen(const char *strpath,int Strtype,int Strmode){
	return 1;
}
/* close stream from kinds of format ---------------------------------------------- */
void stream_t::StreamClose(){
}
int stream_t::StreamRead(unsigned char *buff,int n){
	return 0;
}
int stream_t::StreamWrite(unsigned char *buff,int n){
	return 0;
}

/* stream :: file control type -----------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
file_t::file_t(){
	fp=fp_tag=fp_tmp=fp_tag_tmp=NULL;
	mode=timetag=repmode=offset=0;
	time=wtime=gtime_t();
	tick=tick_f=fpos=0;
	start=speed=swapintv=0;
}
file_t::~file_t(){
}
/* open file --------------------------------------------------------------------------------------- */
int file_t::openfile_(gtime_t ttime){
	FILE *ffp;
	string rw;
	char tagpath[MAXSTRPATH + 4] = "";
	char tagh[TIMETAGH_LEN+1]="";

	time.timeget()->utc2gpst();
	tick=tick_f=tickget();
	fpos=0;

	/* use stdin or stdout if file path is null */
	if (fpath=="") {
		fp=mode&STR_MODE_R ? stdin : stdout;
		return 1;
	}
	/* replace keywords */
	reppath(fpath,openpath,ttime,"","");

	/* create directory */
	if ((mode&STR_MODE_W)&&!(mode&STR_MODE_R)) {
		createdir(openpath);
	}
	if (mode&STR_MODE_R) rw="rb"; else rw="wb";

	if (!(fp=fopen(openpath.c_str(),rw.c_str()))) {
		msg="file open error: "+openpath;
		return 0;
	}

	sprintf(tagpath, "%s.tag",openpath.c_str());

	if (timetag) { /* output/sync time-tag */

		if (!(fp_tag=fopen(tagpath,rw.c_str()))) {
			msg="tag open error: ";
			msg.insert(msg.length(),tagpath);
			fclose(fp);
			return 0;
		}

		if (mode&STR_MODE_R) {
			if (fread(&tagh,TIMETAGH_LEN,1,fp_tag)==1&&
				fread(&time,sizeof(time),1,fp_tag)==1) {
				memcpy(&tick_f,tagh+TIMETAGH_LEN-4,sizeof(tick_f));
			}
			else {
				tick_f=0;
			}
			/* adust time to read playback file */
			time.timeset();
		}
		else {
			sprintf(tagh, "TIMETAG MPP %s","1.0.1");
			memcpy(tagh+TIMETAGH_LEN-4,&tick_f,sizeof(tick_f));
			fwrite(&tagh,1,TIMETAGH_LEN,fp_tag);
			fwrite(&time,1,sizeof(time),fp_tag);
			/* time tag file structure   */
			/*   HEADER(60)+TICK(4)+TIME(12)+ */
			/*   TICK0(4)+FPOS0(4/8)+    */
			/*   TICK1(4)+FPOS1(4/8)+... */
		}
	}
	else if (mode&STR_MODE_W) { /* remove time-tag */
		if ((ffp=fopen(tagpath,"rb"))) {
			fclose(ffp);
			remove(tagpath);
		}
	}
	return 1;
}
/* open new swap file ------------------------------------------------------------------------------ */
void file_t::swapfile(gtime_t ttime){
	string open;

	/* return if old swap file open */
	if (fp_tmp||fp_tag_tmp) return;

	/* check path of new swap file */
	reppath(fpath,open,ttime,"","");

	if (!open.compare(openpath)) {
		return;
	}
	/* save file pointer to temporary pointer */
	fp_tmp=fp;
	fp_tag_tmp=fp_tag;

	/* open new swap file */
	openfile_(ttime);
}
/* close old swap file ----------------------------------------------------------------------------- */
void file_t::swapclose(){
	if (fp_tmp) fclose(fp_tmp);
	if (fp_tag_tmp) fclose(fp_tag_tmp);
	fp_tmp=fp_tag_tmp=NULL;
}
/* get stream from file_t time -------------------------------------------- */
gtime_t file_t::strgettime(){
	gtime_t file_time=time;
	return *file_time.timeadd(start);
}
/* sync file_t stream with another (only for file_t) ---------------------- */
void file_t::strsync(void *str_file){
	file_t *file2=(file_t *) str_file;
	if (!fp_tag||!file2->fp_tag) return;
	repmode=0; file2->repmode=1;
	file2->offset=(int)(tick_f-file2->tick_f);
}
/* open from file download -------------------------------------------------------- */
int file_t::StreamOpen(const char *strpath,int Strtype,int Strmode){
	Stype=Strtype; mode=Smode=Strmode;
	path=strpath;
	tick_i=tick_o=tickget();

	if (!(Smode&(STR_MODE_R|STR_MODE_W))) { Sstate=-1; return 0; }
	char *p;
	/* file options */
	for (p=(char *)strpath; (p=strstr(p,"::")); p+=2) { /* file options */
		if (*(p+2)=='T') timetag=1;
		else if (*(p+2)=='+') sscanf(p+2,"+%lf",&start);
		else if (*(p+2)=='x') sscanf(p+2,"x%lf",&speed);
		else if (*(p+2)=='S') sscanf(p+2,"S=%lf",&swapintv);
	}
	if (start<=0.0) start=0.0; if(swapintv<=0.0) swapintv=0.0;

	fpath=strpath; size_t strp;
	if((strp=fpath.find("::"))!=string::npos) fpath=fpath.substr(0,strp)+'\0';
	initlock(&lock);

	gtime_t time0;
	time0.timeget()->utc2gpst();

	if (!openfile_(time0)) { Sstate=-1; return 0; }

	Sstate=1;
	return 1;
}
/* close from file download ------------------------------------------------------- */
void file_t::StreamClose(){
	strlock();

	if (fp) fclose(fp);
	if (fp_tag) fclose(fp_tag);
	if (fp_tmp) fclose(fp_tmp);
	if (fp_tag_tmp) fclose(fp_tag_tmp);
	fp=fp_tag=fp_tmp=fp_tag_tmp=NULL;

	Stype=0; ;Smode=0; Sstate=0; inr=outr=0;
	path="\0"; msg="\0";

	strunlock();
}
/* read from file download -------------------------------------------------------- */
int file_t::StreamRead(unsigned char *buff,int n){
	unsigned int ticke;
	int nr=0,flag=0;
	struct timeval tv={ 0 };
	unsigned int t,ticks;
	size_t ffpos;

	if (!(Smode&STR_MODE_R)) return 0;

	strlock();

	if (fp==stdin) {
#ifndef WIN32
		fd_set rs;
		/* input from stdin */
		FD_ZERO(&rs); FD_SET(0,&rs);
		if (!select(1,&rs,NULL,NULL,&tv)) return 0;
		if ((nr=(int)read(0,buff,n))<0) return 0;
		return nr;
#else
		nr=0;
#endif
	}
	else if (fp_tag) {
		if (repmode) { /* slave */
			t=(unsigned int)(tick_master+offset);
		}
		else { /* master */
			t=(unsigned int)((tickget()-tick)*speed+start*1000.0);
		}
		for (;;) { /* seek file position */
			if (fread(&ticks,sizeof(ticks),1,fp_tag)<1||
				fread(&ffpos,sizeof(ffpos),1,fp_tag)<1) {
				fseek(fp,0,SEEK_END);
				msg="end";
				break;
			}
			if (repmode||speed>0.0) {
				if ((int)(ticks-t)<1) continue;
			}
			if (!repmode) tick_master=ticks;

			msg="T";
			msg+=to_string((int)ticks<0 ? 0.0 : (int)ticks/1000.0);

			if ((int)(ffpos-fpos)>=n) {
				fseek(fp,ffpos,SEEK_SET);
				fpos=ffpos;
				nr=0; flag=1;
				break;
			}
			n=(int)(ffpos-fpos);

			if (repmode||speed>0.0) {
				fseek(fp_tag,-(long)(sizeof(ticks)+sizeof(ffpos)),SEEK_CUR);
			}
			break;
		}
	}
	if (n>0&&!flag) {
		nr=(int)fread(buff,1,n,fp);
		fpos+=nr;
		if (nr<=0) msg="end";
	}

	inb+=nr;
	ticke=tickget(); if (nr>0) tact=ticke;

	if ((int)(ticke-tick_i)>=tirate) {
		inr=(inb-inbt)*8000/(ticke-tick_i);
		tick_i=ticke; inbt=inb;
	}
	strunlock();
	return nr;
}/* write from file download ------------------------------------------------------ */
int file_t::StreamWrite(unsigned char *buff,int n){
	unsigned int ticke;
	int ns=0;

	unsigned int ticks=tickget();
	int week1,week2;
	double tow1,tow2,intv;
	size_t ffpos,fpos_tmp;

	if (!(Smode&STR_MODE_W)) return 0;

	strlock();

	wtime.timeget()->utc2gpst(); /* write time in gpst */

	 /* swap writing file */
	if (swapintv>0.0&&wtime.time!=0) {
		intv=swapintv*3600.0;
		tow1=wtime.time2gpst(&week1);
		tow2=wtime.time2gpst(&week2);
		tow2+=604800.0*(week2-week1);

		/* open new swap file */
		if (floor((tow1+fswapmargin)/intv)<floor((tow2+fswapmargin)/intv)) {
			swapfile(*wtime.timeadd(fswapmargin));
		}
		/* close old swap file */
		if (floor((tow1-fswapmargin)/intv)<floor((tow2-fswapmargin)/intv)) {
			swapclose();
		}
	}
	if (!fp) ns=0;
	else {
		ns=(int)fwrite(buff,1,n,fp);
		ffpos=ftell(fp);
		fflush(fp);

		if (fp_tmp) {
			fwrite(buff,1,n,fp_tmp);
			fpos_tmp=ftell(fp_tmp);
			fflush(fp_tmp);
		}
		if (fp_tag) {
			ticks-=tick;
			fwrite(&ticks,1,sizeof(ticks),fp_tag);
			fwrite(&ffpos,1,sizeof(ffpos),fp_tag);
			fflush(fp_tag);

			if (fp_tag_tmp) {
				fwrite(&ticks,1,sizeof(ticks),fp_tag_tmp);
				fwrite(&fpos_tmp,1,sizeof(fpos_tmp),fp_tag_tmp);
				fflush(fp_tag_tmp);
			}
		}
	}

	outb+=ns;
	ticke=tickget(); if (ns>0) tact=ticke;

	if ((int)(ticke-tick_o)>tirate) {
		outr=(int)((double)(outb-outbt)*8000/(ticke-tick_o));
		tick_o=ticke; outbt=outb;
	}
	strunlock();
	return ns;
}
/* stream :: tcp server type -------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
tcpsvr_t::tcpsvr_t(){
	svr.state=svr.port=svr.tcon=0;
	svr.tact=svr.tdis=0;
	for (int i=0; i<MAXCLI; i++){
		cli[i].state=cli[i].port=cli[i].tcon=0;
		cli[i].tact=cli[i].tdis=0;
	}
}
tcpsvr_t::~tcpsvr_t(){
}
/* update tcp server ------------------------------------------------------------------------------ */
void tcpsvr_t::updatetcpsvr(){
	string saddr="";
	int i,n=0;

	if (svr.state==0) return;

	for (i=0; i<MAXCLI; i++) {
		if (!cli[i].state) continue;
		saddr=cli[i].saddr;
		n++;
	}
	if (n==0) {
		svr.state=1;
		msg="waiting...";
		return;
	}
	svr.state=2;
	if (n==1) msg=saddr; else msg=to_string(n)+" clients";

}
/* accept client connection ----------------------------------------------------------------------- */
int tcpsvr_t::accsock(){
	struct sockaddr_in addr;
	socket_t sock;
	socklen_t len=sizeof(addr);
	int i,err;

	for (i=0; i<MAXCLI; i++) if (cli[i].state==0) break;
	if (i>=MAXCLI) return 0; /* too many client */

	if ((sock=accept_nb(svr.sock,(struct sockaddr *)&addr,&len))==(socket_t)-1) {
		err=errsock();
		msg="accept error "; msg+=to_string(err);
		closesocket(svr.sock);
		svr.state=0;
		return 0;
	}
	if (sock==0) return 0;
	if (!setsock(sock)) return 0;

	cli[i].sock=sock;
	memcpy(&cli[i].addr,&addr,sizeof(addr));
	cli[i].saddr=inet_ntoa(addr.sin_addr);
	msg=cli[i].saddr;
	cli[i].state=2;
	cli[i].tact=tickget();
	return 1;
}
/* wait socket accept ----------------------------------------------------------------------------- */
int tcpsvr_t::waittcpsvr(){
	if (svr.state<=0) return 0;

	while (accsock());

	updatetcpsvr();
	return svr.state==2;
}
/* open from tcp server download -------------------------------------------------- */
int tcpsvr_t::StreamOpen(const char *strpath,int Strtype,int Strmode){
	Stype=Strtype; Smode=Strmode;
	path=strpath;
	tick_i=tick_o=tickget();

	string bufport;
	decodetcppath(path,&svr.saddr,&bufport,NULL,NULL,NULL,NULL);
	if (!str2int(bufport,svr.port)) msg="port err"+bufport;

	if (!gentcp(svr,0)) { Sstate=-1; return 0; }

	Sstate=1;
	return 1;
}
/* close from tcp server download ------------------------------------------------- */
void tcpsvr_t::StreamClose(){
	strlock();

	for (int i=0; i<MAXCLI; i++) {
		if (cli[i].state) closesocket(cli[i].sock);
	}
	closesocket(svr.sock);

	Stype=0; ; Smode=0; Sstate=0; inr=outr=0;
	path="\0"; msg="\0";

	strunlock();
}
/* read from tcp server download -------------------------------------------------- */
int tcpsvr_t::StreamRead(unsigned char *buff,int n){
	unsigned int tick;
	int nr,i,err;

	if (!(Smode&STR_MODE_R)) return 0;

	strlock();

	if (!waittcpsvr()) nr=0;
	else {
		for (i=0; i<MAXCLI; i++) {
			if (cli[i].state!=2) continue;

			if ((nr=recv_nb(cli[i].sock,buff,n))==-1) {
				if ((err=errsock())) /* receive error */
					msg="receive error!\n";
				discontcp(cli[i],ticonnect);
				updatetcpsvr();
				nr=0;
				break;
			}
			if (nr>0) {
				cli[i].tact=tickget();
				break;
			}
		}
	}
	if (i>=MAXCLI) nr=0;

	inb+=nr;
	tick=tickget(); if (nr>0) tact=tick;

	if ((int)(tick-tick_i)>=tirate) {
		inr=(inb-inbt)*8000/(tick-tick_i);
		tick_i=tick; inbt=inb;
	}
	strunlock();
	return nr;
	/* write from tcp server download --------------------------------------------- */
}
int tcpsvr_t::StreamWrite(unsigned char *buff,int n){
	unsigned int tick;
	int ns=0,i,err;

	if (!(Smode&STR_MODE_W)) return 0;

	strlock();

	if (!waittcpsvr()) ns=0;
	else {
		for (i=0; i<MAXCLI; i++) {
			if (cli[i].state!=2) continue;

			if ((ns=send_nb(cli[i].sock,buff,n))==-1) {
				if ((err=errsock())) /* error send */
					msg="error send!\n";;
				discontcp(cli[i],ticonnect);
				updatetcpsvr();
				ns=0;
				break;
			}
			if (ns>0) cli[i].tact=tickget();
		}
	}
	outb+=ns;
	tick=tickget(); if (ns>0) tact=tick;

	if ((int)(tick-tick_o)>tirate) {
		outr=(int)((double)(outb-outbt)*8000/(tick-tick_o));
		tick_o=tick; outbt=outb;
	}
	strunlock();
	return ns;
}
/* stream :: tcp cilent type -------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
tcpcli_t::tcpcli_t(){
	svr.state=svr.port=svr.tcon=0;
	svr.tact=svr.tdis=0;
	toinact=tirecon=0;
}
tcpcli_t::~tcpcli_t(){
}
/* connect server --------------------------------------------------------------------------------- */
int tcpcli_t::consock(){
	int stat,err;

	/* wait re-connect */
	if (svr.tcon<0||(svr.tcon>0&&
		(int)(tickget()-svr.tdis)<svr.tcon)) {
		return 0;
	}
	/* non-block connect */
	if ((stat=connect_nb(svr.sock,(struct sockaddr *)&svr.addr,
		sizeof(svr.addr)))==-1) {
		err=errsock();
		msg="connect error "; msg+=to_string(err);
		closesocket(svr.sock);
		svr.state=0;
		return 0;
	}
	if (!stat) { /* not connect */
		msg="connecting...";
		return 0;
	}
	msg=svr.saddr;
	svr.state=2;
	svr.tact=tickget();
	return 1;
}
/* wait socket connect ---------------------------------------------------------------------------- */
int tcpcli_t::waittcpcli(){
	if (svr.state<0) return 0;

	if (svr.state==0) { /* close */
		if (!gentcp(svr,1)) return 0;
	}
	if (svr.state==1) { /* wait */
		if (!consock()) return 0;
	}
	if (svr.state==2) { /* connect */
		if (toinact>0&&
			(int)(tickget()-svr.tact)>toinact) {
			msg="timeout";
			discontcp(svr,tirecon);
			return 0;
		}
	}
	return 1;
}
/* open from tcp cilent download -------------------------------------------------- */
int tcpcli_t::StreamOpen(const char *strpath,int Strtype,int Strmode){
	Stype=Strtype; Smode=Strmode;
	path=strpath;
	tick_i=tick_o=tickget();

	string bufport;
	decodetcppath(path,&svr.saddr,&bufport,NULL,NULL,NULL,NULL);
	if (!str2int(bufport,svr.port)){
		msg="port error"; 
		Sstate=-1; return 0;
	}
	toinact=inactout;
	toinact=ticonnect;

	Sstate=1;
	return 1;
}
/* close from tcp cilent download ------------------------------------------------- */
void tcpcli_t::StreamClose(){
	strlock();

	closesocket(svr.sock);

	Stype=0; ; Smode=0; Sstate=0; inr=outr=0;
	path="\0"; msg="\0";

	strunlock();
}
/* read from tcp cilent download -------------------------------------------------- */
int tcpcli_t::StreamRead(unsigned char *buff,int n){
	unsigned int tick;
	int nr,err;

	if (!(Smode&STR_MODE_R)) return 0;

	strlock();

	if (!waittcpcli()) nr=0;
	else {
		if ((nr=recv_nb(svr.sock,buff,n))==-1) {
			if ((err=errsock())) {
				msg="recv error "; msg+=to_string(err);
			}
			else {
				msg="disconnected";
			}
			discontcp(svr,tirecon);
			nr=0;
		}
		else if (nr>0) svr.tact=tickget();
	}

	inb+=nr;
	tick=tickget(); if (nr>0) tact=tick;

	if ((int)(tick-tick_i)>=tirate) {
		inr=(inb-inbt)*8000/(tick-tick_i);
		tick_i=tick; inbt=inb;
	}
	strunlock();
	return nr;
}
/* write from tcp cilent download ------------------------------------------------- */
int tcpcli_t::StreamWrite(unsigned char *buff,int n){
	unsigned int tick;
	int ns,err;

	if (!(Smode&STR_MODE_W)) return 0;

	strlock();

	if (!waittcpcli()) ns=0;
	else {
		if ((ns=send_nb(svr.sock,buff,n))==-1) {
			if ((err=errsock())) {
				msg="send error "; msg+=to_string(err);
			}
			else {
				msg="disconnected";
			}
			discontcp(svr,tirecon);
			ns=0;
		}
		else if (ns>0) svr.tact=tickget();
	}

	outb+=ns;
	tick=tickget(); if (ns>0) tact=tick;

	if ((int)(tick-tick_o)>tirate) {
		outr=(int)((double)(outb-outbt)*8000/(tick-tick_o));
		tick_o=tick; outbt=outb;
	}
	strunlock();
	return ns;
}
/* stream :: tcpsvr_t :: serial control type ---------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
serial_t::serial_t(){
	error=0;
#ifdef WIN32
	state=wp=rp=buffsize=0;
#endif
}
serial_t::~serial_t(){
	if (buff) delete [] buff;
}
/* write serial thread ------------------------------------------------------------ */
#ifdef WIN32
static DWORD WINAPI serialthread(void *arg){
	/* initialize arg to serial_t */
	serial_t *sert=(serial_t *) arg;
	DWORD ns; int n;
	unsigned int Tick;
	unsigned char rbuff[128];

	for (;;){
		Tick=tickget();
		while ((n=sert->readseribuff(rbuff,sizeof(rbuff)))>0){
			if (!WriteFile(sert->dev,rbuff,n,&ns,NULL)) sert->error=1;
		}
		if (sert->state) break;
		sleepms(10-(int)(tickget()-Tick));
	}
	if (sert->buff) delete [] sert->buff;

	return 0;
}
#endif
/* write serial data ------------------------------------------------------------------------------- */
int serial_t::writeseribuff(unsigned char *bbuff,int n){
	int ns,wwp;

	tolock(&lock);
	for (ns=0; ns<n; ns++) {
		buff[wwp=wp]=bbuff[ns];
		if (++wwp>=buffsize) wwp=0;
		if (wwp!=rp) wp=wwp;
		else {
			break;
		}
	}
	tounlock(&lock);
	return ns;
}
/* read serial data ------------------------------------------------------- */
int serial_t::readseribuff(unsigned char *rbuff,int nmax){
	int ns;

	tolock(&lock);
	for (ns=0; rp!=wp&&ns<nmax; ns++) {
		rbuff[ns]=buff[rp];
		if (++rp>=buffsize) rp=0;
	}
	tounlock(&lock);

	return ns;
}
/* open from serial download ------------------------------------------------------ */
int serial_t::StreamOpen(const char *strpath,int Strtype,int Strmode){
	Stype=Strtype; Smode=Strmode;
	path=strpath;
	tick_i=tick_o=tickget();

	const int br[]={
		300,600,1200,2400,4800,9600,19200,38400,57600,115200,230400
	};
	int i,brate=9600,bsize=8,stopb=1,tcp_port=0;
	char parity='N',strDev[128],strPort[128],fctr[64]="",path_tcp[32];

#ifdef WIN32
	DWORD Winerr,rw=0,siz=sizeof(COMMCONFIG);
	COMMCONFIG cc={ 0 };
	COMMTIMEOUTS co={ MAXDWORD,0,0,0,0 }; /* non-block-read */
	char dcb[64]="";
#else
	const speed_t bs[]={
		B300,B600,B1200,B2400,B4800,B9600,B19200,B38400,B57600,B115200,B230400
	};
	struct termios ios={ 0 };
	int rw=0;
#endif

	size_t strp;
	if ((strp=path.find(':'))!=string::npos) {
		strncpy(strPort,strpath,strp); strPort[strp]='\0';
		sscanf(strpath+strp,":%d:%d:%c:%d:%s",&brate,&bsize,&parity,&stopb,fctr);
	}
	else strcpy(strPort,strpath);

	if ((strp=path.find('#'))!=string::npos) sscanf(strpath+strp,"#%d",&tcp_port);

	for (i=0; i<11; i++) if (br[i]==brate) break;
	if (i>=12) {
		msg="bitrate error!";
		Sstate=-1; return 0;
	}
	parity=(char)toupper((int)parity);

#ifdef WIN32
	sprintf(strDev, "\\\\.\\%s",strPort);
	if (Strmode&STR_MODE_R) rw|=GENERIC_READ;
	if (Strmode&STR_MODE_W) rw|=GENERIC_WRITE;

	/* for the convert of strDev and dcd */
	CString Cstr=CString(strDev);
	dev=CreateFile(Cstr,rw,0,0,OPEN_EXISTING,0,NULL);
	if (dev==INVALID_HANDLE_VALUE) {
		msg=strPort; msg+=" open error"+to_string((int)GetLastError());
		Sstate=-1; return 0;
	}
	if (!GetCommConfig(dev,&cc,&siz)) {
		msg=strPort; msg+="getconfig error "+to_string((int)GetLastError());
		CloseHandle(dev);
		Sstate=-1; return 0;
	}
	sprintf(dcb, "baud=%d parity=%c data=%d stop=%d",brate,parity,bsize,stopb);
	Cstr=dcb;
	if (!BuildCommDCB(Cstr,&cc.dcb)) {
		msg=strPort; msg+="buiddcb error "+to_string((int)GetLastError());
		CloseHandle(dev);
		Sstate=-1; return 0;
	}
	if (!strcmp(fctr,"rts")) {
		cc.dcb.fRtsControl=RTS_CONTROL_HANDSHAKE;
	}
	SetCommConfig(dev,&cc,siz); /* ignore error to support novatel */
	SetCommTimeouts(dev,&co);
	ClearCommError(dev,&Winerr,NULL);
	PurgeComm(dev,PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR);

	/* create write thread */
	initlock(&lock);
	buffsize=resebuff;
	if (!(buff=new unsigned char [buffsize])) {
		CloseHandle(dev);
		Sstate=-1; return 0;
	}
	state=1;
	if (!(thread=CreateThread(NULL,0,serialthread,this,0,NULL))) {
		msg=strPort; msg+="serial thread error "+to_string((int)GetLastError());
		CloseHandle(dev);
		state=0;
		Sstate=-1;	return 0;
	}
	msg=strPort;
#else
	sprintf(strDev, 128, "/dev/%s",strPort);

	if ((Strmode&STR_MODE_R)&&(Strmode&STR_MODE_W)) rw=O_RDWR;
	else if (Strmode&STR_MODE_R) rw=O_RDONLY;
	else if (Strmode&STR_MODE_W) rw=O_WRONLY;

	if ((dev=open(strDev,rw|O_NOCTTY|O_NONBLOCK))<0) {
		msg=strDev; msg+="open error "+to_string(errno);
		Sstate=-1; return 0;
	}
	tcgetattr(dev,&ios);
	ios.c_iflag=0;
	ios.c_oflag=0;
	ios.c_lflag=0;     /* non-canonical */
	ios.c_cc[VMIN]=0; /* non-block-mode */
	ios.c_cc[VTIME]=0;
	cfsetospeed(&ios,bs[i]);
	cfsetispeed(&ios,bs[i]);
	ios.c_cflag|=bsize==7 ? CS7 : CS8;
	ios.c_cflag|=parity=='O' ? (PARENB|PARODD) : (parity=='E' ? PARENB : 0);
	ios.c_cflag|=stopb==2 ? CSTOPB : 0;
	ios.c_cflag|=!strcmp(fctr,"rts") ? CRTSCTS : 0;
	tcsetattr(dev,TCSANOW,&ios);
	tcflush(dev,TCIOFLUSH);
	msg=strDev;
#endif

	/* open tcp sever to output received stream */
	if (tcp_port>0) {
		sprintf(path_tcp,":%d",tcp_port);
		return tcpsvr_t::StreamOpen(path_tcp,Strtype,Strmode);
	}

	Sstate=1;
	return 1;
}
/* close from serial download ---------------------------------------------------- */
void serial_t::StreamClose(){
	strlock();

#ifdef WIN32
	state=0;
	WaitForSingleObject(thread,10000);
	CloseHandle(dev);
	CloseHandle(thread);
#else
	close(dev);
#endif
	tcpsvr_t::StreamClose();

	Stype=0; ; Smode=0; Sstate=0; inr=outr=0;
	path="\0"; msg="\0";

	strunlock();
}
/* read from serial download ----------------------------------------------------- */
int serial_t::StreamRead(unsigned char *bbuff,int n){
	unsigned int tick;

	if (!(Smode&STR_MODE_R)) return 0;

	strlock();

#ifdef WIN32
	DWORD nr;
#else
	int nr;
#endif
#ifdef WIN32
	if (!ReadFile(dev,bbuff,n,&nr,NULL)) return 0;
#else
	if ((nr=read(dev,buff,n))<0) return 0;
#endif
	/* write received stream to tcp server port */
	if (nr>0) {
		tcpsvr_t::StreamWrite(bbuff,(int)nr);
	}

	inb+=(int)nr;
	tick=tickget(); if (nr>0) tact=tick;

	if ((int)(tick-tick_i)>=tirate) {
		inr=(inb-inbt)*8000/(tick-tick_i);
		tick_i=tick; inbt=inb;
	}
	strunlock();
	return (int)nr;
}
/* write from serial download ----------------------------------------------------- */
int serial_t::StreamWrite(unsigned char *bbuff,int n){
	unsigned int tick;
	int ns;

	if (!(Smode&STR_MODE_W)) return 0;

	strlock();

#ifdef WIN32
	if ((ns=writeseribuff(bbuff,n))<n) error=1;
#else
	if (write(dev,bbuff,n)<0) {
		error=1;
		ns=0;
	}
#endif

	outb+=ns;
	tick=tickget(); if (ns>0) tact=tick;

	if ((int)(tick-tick_o)>tirate) {
		outr=(int)((double)(outb-outbt)*8000/(tick-tick_o));
		tick_o=tick; outbt=outb;
	}
	strunlock();
	return ns;
}
/* stream :: tcpcli_t :: ntrip control type ----------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
ntrip_t::ntrip_t(){
	state=type=nb=0;
}
ntrip_t::~ntrip_t(){
}
/* send ntrip server request ----------------------------------------------------------------------- */
int ntrip_t::reqntrip_s(){
	char bbuff[256+NTRIP_MAXSTR],*p=bbuff;

	p+=sprintf(p, "SOURCE %s %s\r\n",passwd.c_str(),mntpnt.c_str());
	p+=sprintf(p, "Source-Agent: NTRIP %s\r\n",NTRIP_AGENT);
	p+=sprintf(p, "STR: %s\r\n",str.c_str());
	p+=sprintf(p, "\r\n");

	if (tcpcli_t::StreamWrite((unsigned char *)bbuff,p-bbuff)!=p-bbuff) return 0;

	state=1;
	return 1;
}
/* send ntrip client request ----------------------------------------------------------------------- */
int ntrip_t::reqntrip_c(){
	char bbuff[1024],uuser[512],*p=bbuff;

	if (url.length()) p+=sprintf(p, "GET %s/%s HTTP/1.0\r\n",url.c_str(),mntpnt.c_str());
	else p+=sprintf(p, "GET /%s HTTP/1.0\r\n",mntpnt.c_str());
	p+=sprintf(p, "User-Agent: NTRIP %s\r\n",NTRIP_AGENT);

	if (user=="") {
		p+=sprintf(p, "Accept: */*\r\n");
		p+=sprintf(p, "Connection: close\r\n");
	}
	else {
		sprintf(uuser, "%s:%s",user.c_str(),passwd.c_str());
		p+=sprintf(p, "Authorization: Basic ");
		p+=encbase64(p,(unsigned char *)uuser,strlen(uuser));
		p+=sprintf(p, "\r\n");
	}
	p+=sprintf(p, "\r\n");

	if (tcpcli_t::StreamWrite((unsigned char *)bbuff,p-bbuff)!=p-bbuff) return 0;

	state=1;
	return 1;
}
/* send ntrip server response ---------------------------------------------------------------------- */
int ntrip_t::rspntrip_s(){
	int i,nnb,ps;
	char *p,*q;

	buff[nb]='0';

	if ((p=strstr((char *)buff,NTRIP_RSP_OK_SVR))) { /* ok */
		q=(char *)buff;
		p+=strlen(NTRIP_RSP_OK_SVR);
		nb-=p-q;
		for (i=0; i<nb; i++) *q++=*p++;
		state=2;
		msg=svr.saddr+"/"+mntpnt;
		return 1;
	}
	else if ((p=strstr((char *)buff,NTRIP_RSP_ERROR))) { /* error */
		nnb=nb<MAXSTATMSG ? nb : MAXSTATMSG;
		msg=string((char *)buff,nnb); msg[nnb]=0;
		if ((ps=msg.find('\r'))!=string::npos) msg[ps]='\0';
		nb=0;
		buff[0]='\0';
		state=0;
		discontcp(svr,tirecon);
	}
	else if (nb>=NTRIP_MAXRSP) { /* buffer overflow */
		msg="response overflow";
		nb=0;
		buff[0]='\0';
		state=0;
		discontcp(svr,tirecon);
	}
	return 0;
}
/* send ntrip client response ---------------------------------------------------------------------- */
int ntrip_t::rspntrip_c(){
	int i;
	char *p,*q;

	buff[nb]='0';

	if ((p=strstr((char *)buff,NTRIP_RSP_OK_CLI))) { /* ok */
		q=(char *)buff;
		p+=strlen(NTRIP_RSP_OK_CLI);
		nb-=p-q;
		for (i=0; i<nb; i++) *q++=*p++;
		state=2;
		msg=svr.saddr+"/"+mntpnt;
		return 1;
	}
	if ((p=strstr((char *)buff,NTRIP_RSP_SRCTBL))) { /* source table */
		if (mntpnt=="") { /* source table request */
			state=2;
			msg="source table received";
			return 1;
		}
		msg="no mountp. reconnect...";
		nb=0;
		buff[0]='\0';
		state=0;
		discontcp(svr,tirecon);
	}
	else if ((p=strstr((char *)buff,NTRIP_RSP_HTTP))) { /* http response */
		if ((q=strchr(p,'\r'))) *q='\0'; else buff[128]='\0';
		msg=string(p);
		nb=0;
		buff[0]='\0';
		state=0;
		discontcp(svr,tirecon);
	}
	else if (nb>=NTRIP_MAXRSP) { /* buffer overflow */
		msg="response overflow";
		nb=0;
		buff[0]='\0';
		state=0;
		discontcp(svr,tirecon);
	}
	return 0;
}
/* wait ntrip request/response --------------------------------------------------------------------- */
int ntrip_t::waitntrip(){
	int n;
	char *p;

	if (state<0) return 0; /* error */

	if (svr.state<2) state=0; /* tcp disconnected */

	if (state==0) { /* send request */
		if (!(type==0 ? reqntrip_s() : reqntrip_c())) {
			return 0;
		}
	}
	if (state==1) { /* read response */
		p=(char *)buff+nb;
		if ((n=tcpcli_t::StreamRead((unsigned char *)p,NTRIP_MAXRSP-nb-1))==0) {
			return 0;
		}
		nb+=n; buff[nb]='\0';

		/* wait response */
		return type==0 ? rspntrip_s() : rspntrip_c();
	}
	return 1;
}
/* open from ntrip download ------------------------------------------------------- */
int ntrip_t::StreamOpen(const char *strpath,int Strtype,int Strmode){
	Stype=Strtype; Smode=Strmode;
	path=strpath;
	tick_i=tick_o=tickget();

	string strAddr="\0",strPort="\0",tpath="\0";
	type=Strtype==STR_NTRIPSVR?0:1;

	/* decode tcp/ntrip path */
	decodetcppath(path,&strAddr,&strPort,&user,&passwd,&mntpnt,&str);
	/* use default port if no port specified */
	if (strPort=="\0") strPort=to_string(type? NTRIP_CLI_PORT : NTRIP_SVR_PORT);
	tpath=strAddr+":"+strPort;

	/* ntrip access via proxy server */
	if (proxyaddr!="\0"){
		url="http://"+tpath; proxyaddr=tpath;
	}
	/* open tcp client stream */
	if (!(tcpcli_t::StreamOpen(tpath.c_str(),Strtype,Strmode))){
		Sstate=-1; return 0;
	}

	Sstate=1;
	return 1;
}
/* close from ntrip download ------------------------------------------------------ */
void ntrip_t::StreamClose(){
	strlock();

	closesocket(svr.sock);

	Stype=0; ; Smode=0; Sstate=0; inr=outr=0;
	path="\0"; msg="\0";

	strunlock();
}
/* read from ntrip download ------------------------------------------------------- */
int ntrip_t::StreamRead(unsigned char *bbuff,int n){
	unsigned int tick;
	int nr,nnb;

	if (!(Smode&STR_MODE_R)) return 0;

	strlock();

	if (!waitntrip()) nr=0;
	else {
		if (nb>0) { /* read response buffer first */
			nnb=nb<=n ? nb : n;
			memcpy(bbuff,buff+nb-nnb,nnb);
			nb=0;
			nr=nnb;
		}
		else nr=tcpcli_t::StreamRead(bbuff,n);
	}

	inb+=nr;
	tick=tickget(); if (nr>0) tact=tick;

	if ((int)(tick-tick_i)>=tirate) {
		inr=(inb-inbt)*8000/(tick-tick_i);
		tick_i=tick; inbt=inb;
	}
	strunlock();
	return nr;
}
/* write from ntrip download ------------------------------------------------------ */
int ntrip_t::StreamWrite(unsigned char *bbuff,int n){
	unsigned int tick;
	int ns;

	if (!(Smode&STR_MODE_W)) return 0;

	strlock();

	if (!waitntrip()) ns=0;

	else ns=tcpcli_t::StreamWrite(bbuff,n);

	outb+=ns;
	tick=tickget(); if (ns>0) tact=tick;

	if ((int)(tick-tick_o)>tirate) {
		outr=(int)((double)(outb-outbt)*8000/(tick-tick_o));
		tick_o=tick; outbt=outb;
	}
	strunlock();
	return ns;
}
/* stream :: tcpsvr_t :: ntrip caster control type ---------------------------------------------------
--------------------------------------------------------------------------------------------------- */
ntripc_t::ntripc_t(){
	state=type=0;
}
ntripc_t::~ntripc_t(){
}
/* disconnect ntrip-caster connection -------------------------------------------------------------- */
void ntripc_t::discon_ntripc(int i){
	discontcp(cli[i],ticonnect);
	con[i].nb=0;
	con[i].buff[0]='\0';
	con[i].state=0;
}
/* test mountpoint in source table ----------------------------------------------------------------- */
int ntripc_t::test_mntpnt(const char *mmntpnt){
	char *p,str[256];

	tolock(&lock_srctbl);

	if (!srctbl) {
		tounlock(&lock_srctbl);
		return 1;
	}
	for (p=srctbl; (p=strstr(p,"STR;")); p++) {
		if (sscanf(p,"STR;%255[^;]",str)&&!strcmp(str,mmntpnt)) break;
	}
	tounlock(&lock_srctbl);

	return p!=NULL;
}
/* send ntrip source table ------------------------------------------------------------------------- */
void ntripc_t::send_srctbl(socket_t sock){
	char buff[1024],*p=buff;
	int len;
	gtime_t ti;
	ti.timeget()->time2str(0);
	tolock(&lock_srctbl);

	len=srctbl ? strlen(srctbl) : 0;
	p+=sprintf(p, "%s",NTRIP_RSP_SRCTBL);
	p+=sprintf(p, "Server: %s %s %s\r\n","MPP","1.0.1","b26");
	p+=sprintf(p, "Date: %s UTC\r\n",ti.sep.c_str());
	p+=sprintf(p, "Connection: close\r\n");
	p+=sprintf(p, "Content-Type: text/plain\r\n");
	p+=sprintf(p, "Content-Length: %d\r\n\r\n",len);
	send_nb(sock,(unsigned char *)buff,strlen(buff));
	if (len>0) {
		send_nb(sock,(unsigned char *)srctbl,len);
	}
	tounlock(&lock_srctbl);
}
/* test ntrip-caster client request ---------------------------------------------------------------- */
void ntripc_t::rsp_ntripc_c(int i){
	const char *rsp1=NTRIP_RSP_UNAUTH,*rsp2=NTRIP_RSP_OK_CLI;
	char url[256]="",mmntpnt[256]="",proto[256]="",uuser[256],user_pwd[256],*p,*q;

	con[i].buff[con[i].nb]='\0';

	if (con[i].nb>=NTRIP_MAXRSP-1) { /* buffer overflow */
		discon_ntripc(i);
		return;
	}
	/* test GET and User-Agent */
	if (!(p=strstr((char *)con[i].buff,"GET"))||!(q=strstr(p,"\r\n"))||
		!(q=strstr(q,"User-Agent:"))||!strstr(q,"\r\n")) {
		discon_ntripc(i);
		return;
	}
	/* test protocol */
	if (sscanf(p,"GET %255s %255s",url,proto)<2||strcmp(proto,"HTTP/1.0")) {
		discon_ntripc(i);
		return;
	}
	if ((p=strchr(url,'/'))) strcpy(mmntpnt,p+1);

	/* test mountpoint */
	if (!*mmntpnt||!test_mntpnt(mmntpnt)) {

		/* send source table */
		send_srctbl(cli[i].sock);
		discon_ntripc(i);
		return;
	}
	/* test authentication */
	if (passwd!="") {
		sprintf(uuser, "%s:%s",user.c_str(),passwd.c_str());
		q=user_pwd;
		q+=sprintf(q, "Authorization: Basic ");
		q+=encbase64(q,(unsigned char *)uuser,strlen(uuser));
		if (!(p=strstr((char *)con[i].buff,"Authorization:"))||
			strncmp(p,user_pwd,strlen(user_pwd))) {
			send_nb(cli[i].sock,(unsigned char *)rsp1,
				strlen(rsp1));
			discon_ntripc(i);
			return;
		}
	}
	/* send OK response */
	send_nb(cli[i].sock,(unsigned char *)rsp2,strlen(rsp2));

	con[i].state=1;
	con[i].mntpnt=mmntpnt;
}
/* test ntrip-caster client request ---------------------------------------------------------------- */
void ntripc_t::rsp_ntripc_s(int i){
	const char *rsp1=NTRIP_RSP_ERR_MNTP,*rsp2=NTRIP_RSP_ERR_PWD;
	const char *rsp3=NTRIP_RSP_OK_CLI;
	char ppasswd[256]="",mmntpnt[256]="",str[NTRIP_MAXSTR]="",*p,*q;
	int j,n;

	con[i].buff[con[i].nb]='\0';

	if (con[i].nb>=NTRIP_MAXRSP-1) { /* buffer overflow */
		discon_ntripc(i);
		return;
	}
	/* test SOURCE and Source-Agent */
	if (!(p=strstr((char *)con[i].buff,"SOURCE"))||!(q=strstr(p,"\r\n"))||
		!(q=strstr(q,"Source-Agent:"))||!strstr(q,"\r\n\r\n")) {
		discon_ntripc(i);
		return;
	}
	sscanf(p,"SOURCE %255s %255s",ppasswd,mmntpnt);

	if ((p=strstr((char *)con[i].buff,"STR: "))&&(q=strstr(p,"\r\n"))) {
		n=MIN(q-(p+5),255);
		strncpy(str,p+5,n);
		str[n]='\0';
	}
	/* test mountpoint */
	if (!*mmntpnt||!test_mntpnt(mmntpnt)) {
		send_nb(cli[i].sock,(unsigned char *)rsp1,strlen(rsp1));
		discon_ntripc(i);
		return;
	}
	/* test password */
	if (passwd!=""&&strcmp(ppasswd,passwd.c_str())) {
		send_nb(cli[i].sock,(unsigned char *)rsp2,strlen(rsp2));
		discon_ntripc(i);
		return;
	}
	/* test mountpoint busy */
	for (j=0; j<MAXCLI; j++) {
		if (con[j].state&&!strcmp(mmntpnt,con[j].mntpnt.c_str())) {
			send_nb(cli[i].sock,(unsigned char *)rsp1,strlen(rsp1));
			discon_ntripc(i);
			return;
		}
	}
	/* send OK response */
	send_nb(cli[i].sock,(unsigned char *)rsp3,strlen(rsp3));

	con[i].state=1;
	con[i].mntpnt=mmntpnt;
	con[i].str=str;
}
/* handle ntrip-caster connect request ------------------------------------------------------------- */
void ntripc_t::wait_ntripc(){
	unsigned char *buff;
	int i,n,nmax,err;

	state=svr.state;

	if (!waittcpsvr()) return;

	for (i=0; i<MAXCLI; i++) {
		if (cli[i].state!=2||con[i].state) continue;

		/* receive ntrip-caster request */
		buff=con[i].buff+con[i].nb;
		nmax=NTRIP_MAXRSP-con[i].nb-1;

		if ((n=recv_nb(cli[i].sock,buff,nmax))==-1) {
			if ((err=errsock())) /* receive error */
				msg="receive error!\n";
			discon_ntripc(i);
			continue;
		}
		if (n<=0) continue;

		/* test ntrip-caster request */
		con[i].nb+=n;
		if (type) {
			rsp_ntripc_c(i);
		}
		else {
			rsp_ntripc_s(i);
		}
	}
}
/* open from ntrip caster download ------------------------------------------------ */
int ntripc_t::StreamOpen(const char *strpath,int Strtype,int Strmode){
	Stype=Strtype; Smode=Strmode;
	path=strpath;
	tick_i=tick_o=tickget();

	string strPort="\0",tpath;
	type=Strtype==STR_NTRIPC_S?0:1;
	initlock(&lock_srctbl);

	/* decode tcp/ntrip path */
	decodetcppath(path,NULL,&strPort,&user,&passwd,NULL,NULL);

	/* use default port if no port specified */
	if (strPort=="\0") strPort=to_string(type? NTRIP_CLI_PORT : NTRIP_SVR_PORT);
	tpath=":"+strPort;

	/* open tcp server stream */
	if (!(tcpsvr_t::StreamOpen(tpath.c_str(),Strtype,Strmode))){
		Sstate=-1; return 0;
	}

	Sstate=1;
	return 1;
}
/* close from ntrip caster download ----------------------------------------------- */
void ntripc_t::StreamClose(){
	strlock();

	tcpsvr_t::StreamClose();
	if (srctbl) delete [] srctbl;

	Stype=0; ; Smode=0; Sstate=0; inr=outr=0;
	path="\0"; msg="\0";

	strunlock();
}
/* read from ntrip caster download ------------------------------------------------ */
int ntripc_t::StreamRead(unsigned char *buff,int n){
	unsigned int tick;
	int nr,i,err;

	if (!(Smode&STR_MODE_R)) return 0;

	strlock();

	wait_ntripc();

	for (i=0; i<MAXCLI; i++) {
		if (!con[i].state) continue;

		nr=recv_nb(cli[i].sock,buff,n);

		if (nr<0) {
			if ((err=errsock()))  /* receive error */
				msg="receive error!\n";
			discon_ntripc(i);
		}
		else if (nr>0) {
			cli[i].tact=tickget();

			/* record received mountpoint */
			mntpnt=con[i].mntpnt;
			break;
		}
	}
	if (i<=MAXCLI) nr=0;

	inb+=nr;
	tick=tickget(); if (nr>0) tact=tick;

	if ((int)(tick-tick_i)>=tirate) {
		inr=(inb-inbt)*8000/(tick-tick_i);
		tick_i=tick; inbt=inb;
	}
	strunlock();
	return nr;
}
/* write from ntrip caster download ----------------------------------------------- */
int ntripc_t::StreamWrite(unsigned char *buff,int n){
	unsigned int tick;
	int i,ns=0,err;

	if (!(Smode&STR_MODE_W)) return 0;

	strlock();

	wait_ntripc();

	for (i=0; i<MAXCLI; i++) {
		if (!con[i].state) continue;

		/* skip if not selected mountpoint */
		if (mntpnt!=""&&mntpnt.compare(con[i].mntpnt)) {
			continue;
		}
		ns=send_nb(cli[i].sock,buff,n);

		if (ns<n) {
			if ((err=errsock())) /* send error */
				msg="send error!\n";
			discon_ntripc(i);
		}
		else {
			cli[i].tact=tickget();
		}
	}

	outb+=ns;
	tick=tickget(); if (ns>0) tact=tick;

	if ((int)(tick-tick_o)>tirate) {
		outr=(int)((double)(outb-outbt)*8000/(tick-tick_o));
		tick_o=tick; outbt=outb;
	}
	strunlock();
	return ns;
}
/* stream :: udp type --------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
udp_t::udp_t(){
	state=type=port=0;
}
udp_t::~udp_t(){
}
/* generate udp socket ---------------------------------------------------- */
int udp_t::genudp(int udptype,int strport,string Saddr){
	struct hostent *hp;
	int bs=resebuff,opt=1;

	state=2; type=udptype; port=strport; saddr=Saddr;

	if ((sock=socket(AF_INET,SOCK_DGRAM,0))==(socket_t)-1) {
		msg="socket error "+to_string(errsock());
		Sstate=-1;
		return 0;
	}
	if (setsockopt(sock,SOL_SOCKET,SO_RCVBUF,(const char *)&bs,sizeof(bs))==-1||
		setsockopt(sock,SOL_SOCKET,SO_SNDBUF,(const char *)&bs,sizeof(bs))==-1) {
		msg="sockopt error: bufsiz";
	}
	memset(&addr,0,sizeof(addr));
	addr.sin_family=AF_INET;
	addr.sin_port=htons(port);

	if (!type) { /* udp server */
		addr.sin_addr.s_addr=htonl(INADDR_ANY);
#ifdef SVR_REUSEADDR
		setsockopt(udp->sock,SOL_SOCKET,SO_REUSEADDR,(const char *)&opt,sizeof(opt));
#endif
		if (bind(sock,(struct sockaddr *)&addr,sizeof(addr))==-1) {
			msg="bind error "+to_string(errsock())+" : "+to_string(port);
			closesocket(sock);
			Sstate=-1;
			return 0;
		}
	}
	else { /* udp client */
		if (!saddr.compare("255.255.255.255")&&
			setsockopt(sock,SOL_SOCKET,SO_BROADCAST,(const char *)&opt,
				sizeof(opt))==-1) {
			msg="sockopt error: broadcast";
		}
		if (!(hp=gethostbyname(saddr.c_str()))) {
			msg="address error "+saddr;
			closesocket(sock);
			Sstate=-1;
			return 0;
		}
		memcpy(&addr.sin_addr,hp->h_addr,hp->h_length);
	}
	Sstate=1;
	return 1;
}
/* open from udp download --------------------------------------------------------- */
int udp_t::StreamOpen(const char *strpath,int Strtype,int Strmode){
	Stype=Strtype; Smode=Strmode;
	path=strpath;
	tick_i=tick_o=tickget();

	string strPort,strAddr;
	int nPort;

	decodetcppath(path,&strAddr,&strPort,NULL,NULL,NULL,NULL);

	if (sscanf(strPort.c_str(),"%d",&nPort)<1){
		msg="port error: "+strPort;
		Sstate=-1; return 0;
	}

	/* udp server or client */
	if (Strtype==STR_UDPSVR) {
		return genudp(0,nPort,""); 
	}
	else return genudp(1,nPort,strAddr);

	Sstate=1;
	return 1;
}
/* close from udp download -------------------------------------------------------- */
void udp_t::StreamClose(){
	strlock();

	closesocket(sock);

	Stype=0; ; Smode=0; Sstate=0; inr=outr=0;
	path="\0"; msg="\0";

	strunlock();
}
/* read from udp download --------------------------------------------------------- */
int udp_t::StreamRead(unsigned char *buff,int n){
	unsigned int tick;
	struct timeval tv={ 0 };
	fd_set rs;
	int ret,nr;

	if (!(Smode&STR_MODE_R)) return 0;

	strlock();

	FD_ZERO(&rs); FD_SET(sock,&rs);
	ret=select(sock+1,&rs,NULL,NULL,&tv);
	if (ret<=0) return ret;
	nr=recvfrom(sock,(char *)buff,n,0,NULL,NULL);
	nr=nr<=0 ? -1 : nr;

	inb+=nr;
	tick=tickget(); if (nr>0) tact=tick;

	if ((int)(tick-tick_i)>=tirate) {
		inr=(inb-inbt)*8000/(tick-tick_i);
		tick_i=tick; inbt=inb;
	}
	strunlock();
	return nr;
}
/* write from udp download -------------------------------------------------------- */
int udp_t::StreamWrite(unsigned char *buff,int n){
	unsigned int tick;
	int ns;

	if (!(Smode&STR_MODE_W)) return 0;

	strlock();

	ns=(int)sendto(sock,(char *)buff,n,0,
		(struct sockaddr *)&addr,sizeof(addr));

	outb+=ns;
	tick=tickget(); if (ns>0) tact=tick;

	if ((int)(tick-tick_o)>tirate) {
		outr=(int)((double)(outb-outbt)*8000/(tick-tick_o));
		tick_o=tick; outbt=outb;
	}
	strunlock();
	return ns;
}
/* stream :: ftp download control type ---------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
ftp_t::ftp_t(){
	state=proto=error=0;
	topts[0]=topts[1]=topts[2]=topts[3]=0;
	tnext=gtime_t();
}
ftp_t::~ftp_t(){
}
/* decode ftp path path = [user[:passwd]@]addr[/file[::T=topts(0:3)]] */
void ftp_t::decodeftppath(){
	string bufpath=path;
	topts[3]=0;
	size_t str1,str2;

	/* user and passwd */
	if ((str1=bufpath.rfind('@'))!=string::npos){
		if ((str2=bufpath.find(':'))!=string::npos&&str2<str1){
			user=bufpath.substr(0,str2);
			passwd=bufpath.substr(str1+1,str1-str2-1);
		}
		else user=bufpath.substr(0,str1);
		/* erase bufpath */
		bufpath.erase(0,str1+1);
	}
	/* file topts */
	if ((str1=bufpath.find('/'))!=string::npos){
		if ((str2=bufpath.find("::"))!=string::npos&&str2>str1){
			file=bufpath.substr(str1+1,str2-str1-1);
			sscanf(bufpath.c_str()+str2+2,"T=%d,%d,%d,%d",topts,topts+1,topts+2,topts+3);
		}
		else file=bufpath.substr(str1+1);
		/* erase bufpath */
		bufpath.erase(str1);
	}
	/* addr */
	addr=bufpath;
}
/* open from ftp download --------------------------------------------------------- */
int ftp_t::StreamOpen(const char *strpath,int Strtype,int Strmode){
	Stype=Strtype; Smode=Strmode;
	path=strpath;
	tick_i=tick_o=tickget();

	proto=Strtype==STR_FTP?0:1;

	/* decode ftp path */
	decodeftppath();

	/* set first download time */
	gtime_t nowtime;
	tnext=*nowtime.timeget()->timeadd(10.0);

	Sstate=1;
	return 1;
}
/* close from ftp download -------------------------------------------------------- */
void ftp_t::StreamClose(){
	strlock();

	Stype=0; ; Smode=0; Sstate=0; inr=outr=0;
	path="\0"; msg="\0";

	strunlock();
}
/* read from ftp download --------------------------------------------------------- */
int ftp_t::StreamRead(unsigned char *buff,int n){
	unsigned int tick;
	int nr;
	gtime_t time;
	unsigned char *p,*q;

	if (!(Smode&STR_MODE_R)) return 0;

	strlock();

	time.timeget()->utc2gpst();

	if (time.timediff(tnext)<0.0) { /* until download time? */
		return 0;
	}
	if (state<=0) { /* ftp/http not executed? */
		state=1;
		msg=proto ? "http://" : "ftp://"; msg+=addr;

#ifdef WIN32
		if (!(thread=CreateThread(NULL,0,ftpthread,this,0,NULL)))
#else
		if (pthread_create(&thread,NULL,ftpthread,ftp))
#endif
		{
			state=3;
			msg="ftp thread error";
			return 0;
		}
	}
	if (state<=1) return 0; /* ftp/http on going? */

	if (state==3) { /* ftp error */
		msg=proto ? "hhtp error " : "ftp error "; msg+=to_string(error);

		/* set next retry time */
		tnext.nextdltime(topts,0);
		state=0;
		return 0;
	}
	/* return local file path if ftp completed */
	p=buff;
	q=(unsigned char *)local.c_str();
	while (*q&&(int)(p-buff)<n) *p++=*q++;
	p+=sprintf((char *)p,"\r\n");

	/* set next download time */
	tnext.nextdltime(topts,1);
	state=0;

	msg="";

	return (int)(p-buff);

	inb+=nr;
	tick=tickget(); if (nr>0) tact=tick;

	if ((int)(tick-tick_i)>=tirate) {
		inr=(inb-inbt)*8000/(tick-tick_i);
		tick_i=tick; inbt=inb;
	}
	strunlock();
	return nr;
}
/* stream :: memory buffer type ----------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
membuf_t::membuf_t(){
	state=wp=rp=bufsize=0;
}
membuf_t::~membuf_t(){
	if (buf) delete [] buf;
}
/* open from memory buffer -------------------------------------------------------- */
int membuf_t::StreamOpen(const char *strpath,int Strtype,int Strmode){
	Stype=Strtype; Smode=Strmode;
	path=strpath;
	tick_i=tick_o=tickget();

	if (!str2int(path,bufsize)) bufsize=DEFAULT_MEMBUF_SIZE;
	state=1;
	if (!(buf=new unsigned char [bufsize])){
		Sstate=-1; return 0;
	}
	initlock(&lock);

	msg = "membuf sizebuf="+to_string(bufsize);

	Sstate=1;
	return 1;
}
/* close from memory buffer ------------------------------------------------------- */
void membuf_t::StreamClose(){
	strlock();

	Stype=0; ; Smode=0; Sstate=0; inr=outr=0;
	path="\0"; msg="\0";

	strunlock();
}
/* read from memory buffer -------------------------------------------------------- */
int membuf_t::StreamRead(unsigned char *buff,int n){
	unsigned int tick;
	int i,nr=0;

	if (!(Smode&STR_MODE_R)) return 0;

	strlock();

	tolock(&lock);

	for (i=rp; i!=wp&&nr<n; i++) {
		if (i>=bufsize) i=0;
		buff[nr++]=buf[i];
	}
	rp=i;
	tounlock(&lock);

	inb+=nr;
	tick=tickget(); if (nr>0) tact=tick;

	if ((int)(tick-tick_i)>=tirate) {
		inr=(inb-inbt)*8000/(tick-tick_i);
		tick_i=tick; inbt=inb;
	}
	strunlock();
	return nr;
}
/* write from memory buffer ------------------------------------------------------- */
int membuf_t::StreamWrite(unsigned char *buff,int n){
	unsigned int tick;
	int i,ns;

	if (!(Smode&STR_MODE_W)) return 0;

	strlock();

	tolock(&lock);

	for (i=0; i<n; i++) {
		buf[wp++]=buff[i];
		if (wp>=bufsize) wp=0;
		if (wp==rp) {
			msg="mem-buffer overflow";
			state=-1;
			ns=i+1;
			break;
		}
	}
	tounlock(&lock);
	if (i>=n) ns=i;

	outb+=ns;
	tick=tickget(); if (ns>0) tact=tick;

	if ((int)(tick-tick_o)>tirate) {
		outr=(int)((double)(outb-outbt)*8000/(tick-tick_o));
		tick_o=tick; outbt=outb;
	}
	strunlock();
	return ns;
}

///* stream converter type -----------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------- */
//strconv_t::strconv_t(){
//	/* num */
//	itype=otype=nmsg=stasel=0;
//	/* class */
//	rtcm=NULL; raw=NULL; out=NULL;
//	for (int i=0; i<32; i++){
//		tint[i]=msgs[i]=tick[i]=ephsat[i]=0;
//	}
//}
//strconv_t::~strconv_t(){
//}
//
///* stream server type --------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------- */
//strsvr_t::strsvr_t(){
//	/* num */
//	state=cycle=buffsize=nmeacycle=relayback=nstr=npb=0;
//	nmeapos[0]=nmeapos[1]=nmeapos[2]=0.0;
//	for (int i=0; i<16; i++) stream[i]=NULL;
//}
//strsvr_t::~strsvr_t(){
//}
