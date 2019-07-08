#include "GNSS/postpro.h"
#include "BaseFunction/basefunction.h"
#include "GNSS/PosModel/position.h"
/* post-processing server type -------------------------------------------------------------------- */
/* Constructor -------------------------------------------------------------------- */
postsvr_t::postsvr_t() {
	nav=new nav_t;
	obs[0]=obs_t(); obs[1]=obs_t();
	solopt[0]=solopt[1]=solopt_t();
	solstream[0]=solstream[1]=file_t();
}
postsvr_t::~postsvr_t() {
	if (rtk) delete rtk;
	if (nav) delete nav;
	obs[0].~obs_t(); obs[1].~obs_t();
	solopt[0].~solopt_t(); solopt[1].~solopt_t();
	solstream[0].~file_t(); solstream[1].~file_t();
}
/* Implementation functions ------------------------------------------------------- */
/* set sat\rcv antenna information ------------------------------------------------ */
void postsvr_t::setpcv(inatx_t *atx,int satrcv) {
	/* update satellite antenna to rtk->nav */
	if (satrcv==0){
		for (int i=0; i<MAXSAT; i++){
			if (!(satsys(i+1,NULL)&rtk->opt->navsys)) continue;
			for (int np=0; np<atx->pcv.size(); np++){
				if (atx->pcv[np].sat!=i+1) continue;
				nav->pcvs[i]=atx->pcv[np];
				/* use satellite L2 offset if L5 offset does not exists */
				if (rtk->opt->nf>2&&norm(nav->pcvs[i].off[2],3)<=0.0){
					matcpy(nav->pcvs[i].off[2],nav->pcvs[i].off[1], 3,1);
					matcpy(nav->pcvs[i].var[2],nav->pcvs[i].var[1],19,1);
				}
				break;
			}
		}
	}
	/* update receiver antenna to rtk->opt */
	else {
		/* loop receiver */
		for (int i=0; i<2; i++) for (int np=0; np<atx->pcv.size(); np++){
			/* antenna tpye check */
			if (rtk->opt->anttype[i].compare(0,20,atx->pcv[np].type)==0){
				rtk->opt->pcvr[i]=atx->pcv[np];
				/* use satellite L2 offset if L5 offset does not exists */
				if (rtk->opt->nf>2&&norm(rtk->opt->pcvr[i].off[2],3)<=0.0){
					matcpy(rtk->opt->pcvr[i].off[2],rtk->opt->pcvr[i].off[1],3,1);
					matcpy(rtk->opt->pcvr[i].var[2],rtk->opt->pcvr[i].var[1],19,1);
				}
				break;
			}
		}
	}
}
/* new rtk according to opt ------------------------------------------------------- */
void postsvr_t::inirtk(prcopt_t *Prcopt,filopt_t *Filopt) {
	if (rtk) delete rtk;
	if (Prcopt->mode==PMODE_SINGLE) rtk=new single_t;
	else if (Prcopt->mode<PMODE_DGPS) rtk=new ppp_t;
	else  rtk=new relative_t;
	rtk->opt=Prcopt;
	/* set base station position */
	for (int i=0; i<6; i++) {
		rtk->rb[i]=i<3 ? Prcopt->rb[i] : 0.0;
	}
	/* initialize navigation and observation pointer */
	rtk->nav=nav; rtk->obsr=obs; rtk->obsb=obs+1;

	/* read erp file */
	if (Filopt->erp.length()>10){
		inerp_t erp(Filopt->erp);
		erp.readerp(&nav->erp);
		rtk->tidefunc.init_erp(rtk->opt,nav);
	}
	/* read sat antenna information file */
	if (Filopt->satantp.length()>10){
		inatx_t sat(Filopt->satantp);
		sat.readatx();
		setpcv(&sat,0);
	}
	/* receiver antenna information file */
	rec_ant=Filopt->rcvantp;
	/* station ocean-loading tide file */
	blq_sta=Filopt->blq;
	/* read ionex tec grid file */
	if (Filopt->iono.length()>10) {
		inionex_t ion(Filopt->iono);
		ion.readIonex(nav);
	}

	/* open test file */
	if (Filopt->test.length()>5) rtk->state_file.open(Filopt->test,ios::out);
}
/* update navigation data --------------------------------------------------------- */
void postsvr_t::updatenav(){
	int i,j;
	for (i=0; i<MAXSAT; i++) for (j=0; j<NFREQ; j++) {
		nav->lam[i][j]=satwavelen(i+1,j,nav);
	}
}
/* update station information to prcopt ------------------------------------------- */
void postsvr_t::updatesta(){
	/* rover station information */
	prcopt->name[0]=obs[0].sta.name.substr(0,4);
	prcopt->anttype[0]=obs[0].sta.antdes;
	for (int i=0; i<3; i++) prcopt->antdel[0][i]=obs[0].sta.del[i];
	/* base station information */
	if (prcopt->mode>=PMODE_DGPS) {
		prcopt->name[1]=obs[1].sta.name.substr(0,4);
		prcopt->anttype[1]=obs[1].sta.antdes;
		for (int i=0; i<3; i++) prcopt->antdel[1][i]=obs[1].sta.del[i];
	}
	/* receiver antenna information and ocean-loading correction */
	if (rec_ant.length()>10&&prcopt->posopt[0]){
		inatx_t rcv(rec_ant);
		rcv.readatx();
		setpcv(&rcv,1);
	}
	/* read blq file */
	if (blq_sta.length()>10&&prcopt->tidecorr&2) {
		inblq_t blq(blq_sta);
		blq.readblq(prcopt->name[0],nav->ocean_par[0]);
		if (prcopt->mode>=PMODE_DGPS) blq.readblq(prcopt->name[1],nav->ocean_par[1]);
	}
	/* tidal displacement functions */
	rtk->tidefunc.init_otl(rtk->opt,nav);
}
/* initialize read stream --------------------------------------------------------- */
int postsvr_t::ini_Read_Stream() {
	/* initialize rover_obs and navigation read stream */
	rover.ini_ReadO(pstopt->rover_obs,"",pstopt->time_start,pstopt->time_end,pstopt->time_inter,0);
	readN.ini_ReadN(pstopt->nav,"");
	/* read head of rover observation file */
	rover.Head(nav,&obs[0].sta);
	/* read navigation file */
	readN.Head(nav); readN.Body(nav,prcopt); readN.closeF(); 
	updatenav();

	/* open stat */
	int stat=rover.test_open();

	/* initailize base_obs read stream */
	if (prcopt->mode>=PMODE_DGPS&&pstopt->base_obs.length()>10) {
		base.ini_ReadO(pstopt->base_obs,"",pstopt->time_start,pstopt->time_end,pstopt->time_inter,0);
		/* read head of base observation file */
		base.Head(nav,&obs[1].sta);
		stat=stat&&base.test_open();
	}
	/* update station information */
	updatesta();
	/* read precise ephemeris */
	if (prcopt->sateph==EPHOPT_PREC&&pstopt->prseph.length()>10) {
		readEph.ini_readEph(pstopt->prseph,pstopt->predict);
		readEph.readsp3(nav,prcopt);
	}
	
	return stat;
}
/* rover/base observation synchronization ----------------------------------------- */
int postsvr_t::obs_synchron() {
	if (!base.test_open()) return 0;

	/* test this base observation time */
	if (obs[1].n>0&&fabs(obs[0].data[0].time.timediff(obs[1].data[0].time))<1E-3) return 1;
	else if (obs[1].n>0&&obs[0].data[0].time.timediff(obs[1].data[0].time)<0) return 0;
	/* read new base observation */
	while (base.One_Epoch_Body(obs+1,prcopt)) {
		// test time sychronization of rover and base
		if (obs[1].n>0&&fabs(obs[0].data[0].time.timediff(obs[1].data[0].time))<1E-3) return 1;
		else if (obs[1].n>0&&obs[0].data[0].time.timediff(obs[1].data[0].time)<0) break;
	}

	return 0;
}
/* write solution header to output stream ----------------------------------------- */
void postsvr_t::writesolhead() {
	unsigned char buff1[1024]={0};
	unsigned char buff2[1024]={0};
	int n;

	if (pstopt->outflag[0]) {
		n=solopt[0].outsolheads(buff1);
		solstream[0].StreamWrite(buff1,n);
	}
	if (pstopt->outflag[1]) {
		n=solopt[1].outsolheads(buff2);
		solstream[1].StreamWrite(buff2,n);
	}
}
/* write solution to output stream ------------------------------------------------ */
void postsvr_t::writesol() {
	if (pstopt->outflag[0]) writesolstr(0);
	if (pstopt->outflag[1]) writesolstr(1);
}
/* write solution to each solstream ----------------------------------------------- */
void postsvr_t::writesolstr(int index) {
	unsigned char buff[MAXSOLMSG+1]={ 0 };
	char *p=(char *)buff;

	/* write solution to buff */
	/* [1] write solution time */
	string soltime=rtk->sol.back().fortime(solopt+index);
	p+=sprintf(p, "%s%s",soltime.c_str(),solopt[index].sep.c_str());

	/* [2] position solution */
	string strpv;
	if (
		rtk->sol.back().stat==SOLQ_NONE||
		solopt[index].posf==SOLF_ENU&&
			(solopt[index].origin==0&&norm(rtk->rb,3)<=0.0||
			solopt[index].origin!=0&&norm(rtk->opt->ru,3)<=0.0)
		)
		return;
	else {
		strpv=rtk->sol.back().forposvel(solopt+index,rtk);
	}
	p+=sprintf(p, "%s\n",strpv.c_str());

	/* write solution to stream[index+3] */
	solstream[index].StreamWrite(buff,p-(char *)buff);
}
/* initialize postsvr ------------------------------------------------------------- */
int postsvr_t::postsvrini(option_t *option) {
	/* option pointor */
	prcopt=&option->prcopt;
	pstopt=&option->pstopt;

	/* solution types */
	for (int i=0; i<2; i++) {
		solopt[i]=option->solopt[i];
		if (pstopt->outflag[i]&&!solstream[i].StreamOpen(pstopt->output[i].c_str(),STR_FILE,STR_MODE_W)){
			for (i--; i>=0; i--) solstream[i].StreamClose();
			return 0;
		}
	}
	writesolhead();
	/* initialize rtk */
	inirtk(prcopt,&option->filopt);
	rtk->rtkinit();

	return 1;
}
/* read Navigation file --------------------------------------------------- */
int postsvr_t::readNav() {
	/* initialize navigation file stream */
	readN.ini_ReadN(pstopt->nav,"");
	/* read navigation file */
	readN.Head(nav); readN.Body(nav,prcopt); readN.closeF(); 
	return 1;
}
/* post-position epoch by epoch ------------------------------------------- */
int postsvr_t::Post_Position_Epoch() {
	/* read observation and navigation file */
	if (!ini_Read_Stream()) return 0;

	/* read observation file and post-position epoch by epoch */
	while (rover.One_Epoch_Body(obs,prcopt)) {
		if (prcopt->mode>=PMODE_DGPS&&!obs_synchron()) continue;

		/* SPP for base station */
		if (rtk->opt->mode>=PMODE_DGPS) {
			/* arrange observation data */
			sortobs(obs[1]);
			rtk->basepos();
		}

		/* test */
		/*if (!obs[0].data[0].time.sep.compare("2017/12/01 07:56:27.000"))
			rtk->obsr=obs;*/

		/* position process */
		/* arrange observation data */
		sortobs(obs[0]);
		rtk->rtkpos();

		if (rtk->sol.back().stat!=SOLQ_NONE) writesol();
	}

	return 1;
}