/* This is just for rtk test! ---------------------------------------------------------------------------
* 2018/05/14	read O file
--------------------------------------------------------------------------------------------------- */
#include "GNSS/rtkpro.h"
#include "BaseFunction/basefunction.h"

/* git branch test */
int rtkmain(int argc,char* argv[]){
	string optfile=argv[1];
	string stopflag="Go!";
	char *msg[3]={0};

	/* processing options */
	option_t option=option_t();
	if (!option.readrtkopt(optfile)) return 0;

	/* initialize rtk processing server */
	rtksvr_t rtkpro;
	rtkpro.rtksvrini(&option);
	rtkpro.rtksvrstart();

	cout << "You can press some keys to stop process.\n";
	cin >>stopflag;

	rtkpro.rtksvrstop(msg);

	return 1;
}
