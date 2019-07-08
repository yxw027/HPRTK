/* This is just for ambiguity test! ------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
#include "GNSS/rtkpro.h"
#include "BaseFunction/basefunction.h"
#include "GNSS/AmbModel/ambiguity.h"

#define SQR(x)      ((x)*(x))

int ambmain(int argc,char* argv[]) {
	double DDA_[]={ 2.3, 4.2, 15.1 },
		R_DDA_[]={3.31,2.69,2.54, 2.69,2.18,2.06, 2.54,2.06,1.95 };

	vector<double> DDA(DDA_,DDA_+3),R_DDA(R_DDA_,R_DDA_+9),FIX(6,0.0),VAR(2,0.0);

	lambda_t lambda;
	int flag=lambda.int_amb(DDA,R_DDA,FIX,3,2,VAR);

	return 1;
}