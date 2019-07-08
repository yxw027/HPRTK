#include "GNSS/AdjModel/adjustment.h"

#include "BaseFunction/basefunction.h"

/* parent adjustment functions -------------------------------------------------------------------- */
/* Constructor -------------------------------------------------------------------- */
adjfunc_t::adjfunc_t(){
	H_niter=0;
}
adjfunc_t::~adjfunc_t(){
	sgm2.clear();
}
/* Implementaion functions -------------------------------------------------------- */
/* Helmert component covariance estimate for multi-GNSS observation ------------------
* formula : S * sgm2 = W
* base matrix:
* P   = R^-1
* Px  = Qx = Rx^-1
* QQ  = APA'+Px
* for the ith system :
* Sii = ni - 2tr(Qi*QQ^-1) + tr(Qi*QQ^-1*Qi*QQ^-1)
* Sij = tr(Qi*QQ^-1*Qj*QQ^-1) (j!=i)
* Wi  = Vi*Pi*Vi
* --------------------------------------------------------------------------------- */
int adjfunc_t::helmert_est(const vector<double> &A,const vector<double> &Rx,
	int numL,int numX,const int nobs[4][NFREQ*2]) {
	/* initialization */
	vector<double> PP(newR.begin(),newR.end()),Px(Rx.begin(),Rx.end()),QQ(numX*numX,0.0),
		Qi[5],Pi,VPi,Ai,APi,
		sgm,S,W;

	/* compute P */
	if (matinv(PP,numL)==-1||matinv(Px,numX)==-1) return -1;
	
	/* loop of satellite systems to get Qi,QQ and W */
	int sys_n[4]={0};
	for (int sys=nsys=0,nnow=0; sys<4; sys++) {
		for (int fff=0; fff<NFREQ*2; fff++) sys_n[nsys]+=nobs[sys][fff];
		if (sys_n[nsys]==0) continue;
		
		/* initialize vectors */
		VPi.assign(sys_n[nsys],0.0); Pi.assign(sys_n[nsys]*sys_n[nsys],0.0);
		APi.assign(numX*sys_n[nsys],0.0); 
		Ai.assign(A.begin()+nnow*numX,A.begin()+(nnow+sys_n[nsys])*numX); //Ai
		Qi[nsys].assign(numX*numX,0.0); W.push_back(0.0);

		for (int i=0;i<sys_n[nsys];i++) for (int j=0;j<sys_n[nsys];j++)
			Pi[j + i*sys_n[nsys]]=PP[j+nnow + (i+nnow)*numL]; // Pi
		matmul_vec("NN",1,sys_n[nsys],sys_n[nsys],1.0,V,Pi,0.0,VPi,nnow,0,0); //VPi
		matmul_vec("NT",1,1,sys_n[nsys],1.0,VPi,V,0.0,W,0,nnow,nsys); //W[nsys]

		matmul_vec("NN",numX,sys_n[nsys],sys_n[nsys],1.0,Ai,Pi,0.0,APi); //APi
		matmul_vec("NT",numX,numX,sys_n[nsys],1.0,APi,Ai,0.0,Qi[nsys]); //Qi[nsys]
		for (int i=0; i<numX; i++) for (int j=0; j<numX; j++) QQ[j+i*numX]+=Qi[nsys][j+i*numX];//QQ

		nnow+=sys_n[nsys]; nsys++; 
	}
	if (nsys<2) return -1;
	/* Qi and W of X parameter */
	//int nsys_all=nsys+1; vector<double> dXP(numX,0.0);
	//Qi[nsys].assign(Px.begin(),Px.end()); //Qx
	//W.push_back(0.0);
	//matmul_vec("NN",1,numX,numX,1.0,dX.begin(),Px.begin(),0.0,dXP.begin()); //dXP
	//matmul_vec("NT",1,1,numX,1.0,dXP.begin(),dX.begin(),0.0,W.begin()+nsys); //Wx
	/* QQ^-1 for all systems and X parameters */
	for (int i=0; i<numX; i++) for (int j=0; j<numX; j++) QQ[j+i*numX]+=Px[j+i*numX]; //QQ = APA'+Px
	if (matinv(QQ,numX)==-1) return -1;
	

	/* initilaize sgm2 and S */
	sgm.assign(nsys,0.0); S.assign(nsys*nsys,0.0);

	/* S */
	vector<double> QiQ(numX*numX,0.0),QjQ(numX*numX,0.0),QQQQ(numX*numX,0.0);
	for (int i=0; i<nsys; i++) {
		matmul_vec("NN",numX,numX,numX,1.0,Qi[i],QQ,0.0,QiQ); //QiQ
		for (int j=0; j<nsys; j++) {
			matmul_vec("NN",numX,numX,numX,1.0,Qi[j],QQ,0.0,QjQ); //QjQ
			matmul_vec("NN",numX,numX,numX,1.0,QiQ,QjQ,0.0,QQQQ); //QQQQ
			S[j+i*nsys]+=matrace(QQQQ.begin(),numX);
			 //this system
			if (j==i) S[j+i*nsys]+=sys_n[i]-2*matrace(QiQ,numX);
		}
	}
	if (solve_line("T",S,W,sgm,nsys,1)) return -1;

	/* normalize and save sgm */
	for (int i=nsys; i>0; i--) sgm[i-1]/=sgm[0];
	if (H_niter==0) sgm2.assign(sgm.begin(),sgm.end());
	else for (int i=0; i<nsys; i++) sgm2[i]*=sgm[i];

	/* update newR */
	for (int sys=0,nnow=0; sys<nsys; sys++) {
		for (int i=0;i<sys_n[sys];i++) for (int j=0;j<sys_n[sys];j++)
			newR[j+nnow + (i+nnow)*numL]*=sgm[sys];
		nnow+=sys_n[sys];
	}
	PP.clear(); Px.clear(); QQ.clear(); 
	Pi.clear(); VPi.clear(); Ai.clear(); APi.clear();
	sgm.clear(); S.clear(); W.clear();
	for (int i=0; i<5; i++) Qi[i].clear();

	return 0;
}
/* least-squre adjustment function ------------------------------------------------ */
int adjfunc_t::lsq(const vector<double> &A,const vector<double> &L,
	const vector<double> &R,vector<double> &X,vector<double> &Rx,
	int numL,int numX){

	if (numL<numX) return -1;

	vector<double> P,AP(numL*numX,0),APL(numX,0),Q(numX*numX,0);
	dX.assign(numX,0.0);

	/* weight matrix P */
	P.assign(R.begin(),R.end());
	if (matinv(P,numL)==-1) return -1;

	/* compute AP,APA */
	matmul_vec("NN",numX,numL,numL,1.0,A,P,0.0,AP); //AP
	matmul_vec("NT",numX,1,numL,1.0,AP,L,0.0,APL);  //APL
																		/* cofactor matrix Q */
	matmul_vec("NT",numX,numX,numL,1.0,AP,A,0.0,Q);
	if (matinv(Q,numX)==-1) return -1;
	/* compute dX */
	matmul_vec("NN",numX,1,numX,1.0,Q,APL,0.0,dX);  //dX

																		/* update X and Rx */
	for (int i=0; i<numX; i++) X[i]+=dX[i];

	/* compute Rx */
	vector<double> LP(numL,1),VPV(1,0.0);
	double coe;
	matmul_vec("NN",1,numL,numL,1.0,L,P,0.0,LP); //LP
	matmul_vec("NT",1,1,numL,1.0,LP,L,0.0,VPV); //LPL to VPV
	matmul_vec("NT",1,1,numX,-1.0,APL,dX,1.0,VPV); //VPV = LPL-(APL)X
	coe=VPV[0]/(numL>numX ? numL-numX : 1);
	Rx.assign(Q.begin(),Q.end());
	for (size_t i=0; i<Rx.size(); i++) Rx[i]=coe*Rx[i];

	int flag=norm(dX.begin(),numX)<1E-3 ? 1 : 0;

	P.clear(); AP.clear(); APL.clear(); Q.clear(); 

	return flag;
}
/* estimate parameter X array and covariance Rx (virtual)-------------------------- */
int adjfunc_t::adjustment(const vector<double> &A,const vector<double> &L,
	const vector<double> &R,vector<double> &X,vector<double> &Rx,
	int numL,int numX,const int nobs[4][NFREQ*2]){
	return 0;
}

/* subclass adjustment functions ------------------------------------------------------------------ */
/* least square adjustment function --------------------------------------------------------------- */
/* Constructor -------------------------------------------------------------------- */
lsadj_t::lsadj_t(){
}
lsadj_t::~lsadj_t(){
}
/* Implementaion functions -------------------------------------------------------- */
/* estimate parameter X array and covariance Rx --------------------------------------
*   nobs[4][NFREQ*2] not used
* --------------------------------------------------------------------------------- */
int lsadj_t::adjustment(const vector<double> &A,const vector<double> &L,
	const vector<double> &R,vector<double> &X,vector<double> &Rx,
	const int numL,const int numX,const int nobs[4][NFREQ*2]){
	
	return lsq(A,L,R,X,Rx,numL,numX);
}
 
/* kalman filter adjustment function -------------------------------------------------------------- */
/* Constructor -------------------------------------------------------------------- */
kalmanadj_t::kalmanadj_t(){
}
kalmanadj_t::~kalmanadj_t(){
}
/* Implementaion functions -------------------------------------------------------- */
/* estimate parameter X array and covariance Rx --------------------------------------
*   kalman filter state update as follows:
*   Q=A'*Rx*A+R, K=Rx*A*Q^-1 
*   xp=x+K*L, Rx(new)=(E-K*A')*Rx
*   nobs[4][NFREQ*2] not used
* --------------------------------------------------------------------------------- */
int kalmanadj_t::adjustment(const vector<double> &A,const vector<double> &L,
	const vector<double> &R,vector<double> &X,vector<double> &Rx,
	int numL,int numX,const int nobs[4][NFREQ*2]){

	vector<double> RxA(numL*numX,0.0),Q(R.begin(),R.end()),K(numL*numX,0.0),
		E(numX*numX,0.0),Rx_;
	eyemat(E.begin(),numX); Rx_.assign(Rx.begin(),Rx.end());
	dX.assign(numX,0.0);

	matmul_vec("NN",numX,numL,numX,1.0,Rx,A,0.0,RxA); // Rx*A
	matmul_vec("TN",numL,numL,numX,1.0,A,RxA,1.0,Q); //Q=A'*RxA

	if (matinv(Q,numL)==0){
		matmul_vec("NN",numX,numL,numL,1.0,RxA,Q,0.0,K); // K=RxA*Q^-1
		matmul_vec("NN",numX,1,numL,1.0,K,L,0.0,dX); // dX=K*L
		matmul_vec("NT",numX,numX,numL,-1.0,K,A,1.0,E); // E=E-K*A'
		matmul_vec("NN",numX,numX,numX,1.0,E,Rx_,0.0,Rx); //Rx=E*Rx_

		/* residual of L */
		for(int i=0 ;i<numX; i++) X[i]+=dX[i];
		V.assign(L.begin(),L.end());
		matmul_vec("TN",numL,1,numX,-1.0,A,dX,1.0,V);
	}
	else return -1;

	RxA.clear(); Q.clear(); K.clear(); E.clear(); Rx_.clear();
	
	return 0;
}

/* helmert components covariance estimate for kalman filter adjustment function ------------------- */
/* estimate parameter X array and covariance Rx ----------------------------------- */
helmert_t::helmert_t() {
	
}
helmert_t::~helmert_t() {
	newR.clear();
}
int helmert_t::adjustment(const vector<double> &A,const vector<double> &L,
	const vector<double> &R,vector<double> &X,vector<double> &Rx,
	int numL,int numX,const int nobs[4][NFREQ*2]) {

	newR.assign(R.begin(),R.end()); Rx_ori.assign(Rx.begin(),Rx.end());
	vector<double> X_ori(X);
	
	for (H_niter=0; H_niter<3; H_niter++) {
		Rx_ori.assign(Rx.begin(),Rx.end()); X_ori.assign(X.begin(),X.end());
		if (kalmanadj_t::adjustment(A,L,newR,X_ori,Rx_ori,numL,numX,nobs)) 
			return kalmanadj_t::adjustment(A,L,R,X,Rx,numL,numX,nobs);
		if (helmert_est(A,Rx,numL,numX,nobs)) 
			return kalmanadj_t::adjustment(A,L,R,X,Rx,numL,numX,nobs);
	}
	return kalmanadj_t::adjustment(A,L,newR,X,Rx,numL,numX,nobs);
}