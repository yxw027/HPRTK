/* resolve integer ambiguity -------------------------------------------------------------------------
* lambda reference :
*     [1] P.J.G.Teunissen, The least-square ambiguity decorrelation adjustment:
*         a method for fast GPS ambiguity estimation, J.Geodesy, Vol.70, 65-82,
*         1995
*     [2] X.-W.Chang, X.Yang, T.Zhou, MLAMBDA: A modified LAMBDA method for
*         integer least-squares estimation, J.Geodesy, Vol.79, 552-565, 2005
* ------------------------------------------------------------------------------------------------- */
#include "GNSS/AmbModel/ambiguity.h"
#include "BaseFunction/basefunction.h"

/* Constant --------------------------------------------------------------------------------------- */
#define LOOPMAX     10000           /* maximum count of search loop */

#define SGN(x)      ((x)<=0.0?-1.0:1.0)
#define ROUND(x)    (floor((x)+0.5))
#define SWAP(x,y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)

/* integar ambiguity strategy parent class -------------------------------------------------------- */
/* Constructor -------------------------------------------------------------------- */
intamb_t::intamb_t() {
	state=-1;
	n_amb=n_fix=0;
}
intamb_t::~intamb_t() {
}
/* Implementation functions ------------------------------------------------------- */
int intamb_t::int_amb(vector<double> &Float_Amb,vector<double> &Amb_Var,vector<double> &Fix_Amb,
	int Amb_Num,int Fix_Num,vector<double> &Sum_Var) {
	return 0;
}

/* lambda/mlambda integer least-square estimation ------------------------------------------------- */
/* Constructor -------------------------------------------------------------------- */
lambda_t::lambda_t() {
}
lambda_t::~lambda_t() {
	L.clear(); D.clear(); Z_trans.clear(); z_float.clear(); z_fix.clear();
}
/* Implementation functions ------------------------------------------------------- */
/* integer gauss transformation ------------------------------------------- */
void lambda_t::gauss(int num_i,int num_j) {
	int k,mu;
    
    if ((mu=(int)ROUND(L[num_i+num_j*n_amb]))!=0) {
        for (k=num_i;k<n_amb;k++) L[k+n_amb*num_j]      -=(double)mu*L[k+num_i*n_amb];
        for (k=0;k<n_amb;k++)     Z_trans[k+n_amb*num_j]-=(double)mu*Z_trans[k+num_i*n_amb];
    }
}
/* permutations ----------------------------------------------------------- */
void lambda_t::permutations(int num_j,double del) {
	int k;
    double eta,lam,a0,a1;
    
    eta=D[num_j]/del;
    lam=D[num_j+1]*L[num_j+1+num_j*n_amb]/del;
    D[num_j]=eta*D[num_j+1]; D[num_j+1]=del;
    for (k=0;k<=num_j-1;k++) {
        a0=L[num_j+k*n_amb]; a1=L[num_j+1+k*n_amb];
        L[num_j+k*n_amb]=-L[num_j+1+num_j*n_amb]*a0+a1;
        L[num_j+1+k*n_amb]=eta*a0+lam*a1;
    }
    L[num_j+1+num_j*n_amb]=lam;
    for (k=num_j+2;k<n_amb;k++) SWAP(L[k+num_j*n_amb],L[k+(num_j+1)*n_amb]);
    for (k=0;k<n_amb;k++) SWAP(Z_trans[k+num_j*n_amb],Z_trans[k+(num_j+1)*n_amb]);
}

/* compute factorization matrix L and D ----------------------------------- */
int lambda_t::factorization_LD(vector<double> &Amb_Var){
	 int i,j,k,info=0;
    double a;
	vector<double> A(Amb_Var.begin(),Amb_Var.end());
    
    for (i=n_amb-1;i>=0;i--) {
        if ((D[i]=A[i+i*n_amb])<=0.0) {info=-1; break;}
        a=sqrt(D[i]);
        for (j=0;j<=i;j++) L[i+j*n_amb]=A[i+j*n_amb]/a;
        for (j=0;j<=i-1;j++) for (k=0;k<=j;k++) A[j+k*n_amb]-=L[i+k*n_amb]*L[i+j*n_amb];
        for (j=0;j<=i;j++) L[i+j*n_amb]/=L[i+i*n_amb];
    }
	A.clear();
	if (info) errmsg="LD factorization error!";
    return info;
}
/* compute reduction matrix Z --------------------------------------------- */
void lambda_t::reduction_Z(){
	int i,j,k;
    double del;
    
    j=n_amb-2; k=n_amb-2;
    while (j>=0) {
        if (j<=k) for (i=j+1;i<n_amb;i++) gauss(i,j);
        del=D[j]+L[j+1+j*n_amb]*L[j+1+j*n_amb]*D[j+1];
        if (del+1E-6<D[j+1]) { /* compared considering numerical error */
            permutations(j,del);
            k=j; j=n_amb-2;
        }
        else j--;
    }
}
/* mlambda search to get z_fix -------------------------------------------- */
int lambda_t::search_zfix(vector<double> &Sum_Var) {
	int i,j,k,c,nn=0,imax=0;
    double newdist,maxdist=1E99,y;
	vector<double> S(n_amb*n_amb,0.0),dist(n_amb,0.0),zb(n_amb,0.0),z(n_amb,0.0),step(n_amb,0.0);
    
    k=n_amb-1; dist[k]=0.0;
    zb[k]=z_float[k];
    z[k]=ROUND(zb[k]); y=zb[k]-z[k]; step[k]=SGN(y);
    for (c=0;c<LOOPMAX;c++) {
        newdist=dist[k]+y*y/D[k];
        if (newdist<maxdist) {
            if (k!=0) {
                dist[--k]=newdist;
                for (i=0;i<=k;i++)
                    S[k+i*n_amb]=S[k+1+i*n_amb]+(z[k+1]-zb[k+1])*L[k+1+i*n_amb];
                zb[k]=z_float[k]+S[k+k*n_amb];
                z[k]=ROUND(zb[k]); y=zb[k]-z[k]; step[k]=SGN(y);
            }
            else {
                if (nn<n_fix) {
                    if (nn==0||newdist>Sum_Var[imax]) imax=nn;
                    for (i=0;i<n_amb;i++) z_fix[i+nn*n_amb]=z[i];
                    Sum_Var[nn++]=newdist;
                }
                else {
                    if (newdist<Sum_Var[imax]) {
                        for (i=0;i<n_amb;i++) z_fix[i+imax*n_amb]=z[i];
                        Sum_Var[imax]=newdist;
                        for (i=imax=0;i<n_fix;i++) if (Sum_Var[imax]<Sum_Var[i]) imax=i;
                    }
                    maxdist=Sum_Var[imax];
                }
                z[0]+=step[0]; y=zb[0]-z[0]; step[0]=-step[0]-SGN(step[0]);
            }
        }
        else {
            if (k==n_amb-1) break;
            else {
                k++;
                z[k]+=step[k]; y=zb[k]-z[k]; step[k]=-step[k]-SGN(step[k]);
            }
        }
    }
    for (i=0;i<n_fix-1;i++) { /* sort by Sum_Var */
        for (j=i+1;j<n_fix;j++) {
            if (Sum_Var[i]<Sum_Var[j]) continue;
            SWAP(Sum_Var[i],Sum_Var[j]);
            for (k=0;k<n_amb;k++) SWAP(z_fix[k+i*n_amb],z_fix[k+j*n_amb]);
        }
    }
	S.clear(); dist.clear(); zb.clear(); z.clear(); step.clear();
    
    if (c>=LOOPMAX) {
        errmsg="search loop count overflow\n_amb";
        return -1;
    }
    return 0;
}
/* resolve integer ambiguity using lambda/mlambda --------------------------------- */
int lambda_t::int_amb(vector<double> &Float_Amb,vector<double> &Amb_Var,vector<double> &Fix_Amb,
	int Amb_Num,int Fix_Num,vector<double> &Sum_Var) {
	/* initialization */
	state=-1; n_amb=Amb_Num; n_fix=Fix_Num;
    
    if (n_amb<=0||n_fix<=0) return -1;
	L.assign(n_amb*n_amb,0.0); D.assign(n_amb,0.0); Z_trans.assign(n_amb*n_amb,0.0);
	z_float.assign(n_amb,0.0); z_fix.assign(n_amb*n_fix,0.0);
	eyemat(Z_trans.begin(),n_amb);
    
    /* LD factorization */
    if (!(state=factorization_LD(Amb_Var))) {
        
        /* lambda reduction */
        reduction_Z();
        matmul_vec("TN",n_amb,1,n_amb,1.0,Z_trans,Float_Amb,0.0,z_float); /* z=Z'*a */
        
        /* mlambda search */
        if (!(state=search_zfix(Sum_Var))) {
            
            state=solve_line("T",Z_trans,z_fix,Fix_Amb,n_amb,n_fix); /* F=Z'\E */
        }
    }
	L.clear(); D.clear(); Z_trans.clear(); z_float.clear(); z_fix.clear();
    return state;
}