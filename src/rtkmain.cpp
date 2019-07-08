/* This is just for rtk test! ---------------------------------------------------------------------------
* 2018/05/14	read O file
--------------------------------------------------------------------------------------------------- */
#include "GNSS/rtkpro.h"
#include "BaseFunction/basefunction.h"

int rtkmain(int argc,char* argv[]){
	string optfile=argv[1];
	char stopflag='G';
	char *msg[3]={0};

	/* processing options */
	option_t option=option_t();
	if (!option.readrtkopt(optfile)) return 0;

	/* initialize rtk processing server */
	rtksvr_t rtkpro;
	rtkpro.rtksvrini(&option);
	rtkpro.rtksvrstart();

	while (stopflag!='s'&& stopflag!='S'){
		if (_kbhit()) stopflag=_getch();
		sleepms(1000);
	}
	rtkpro.rtksvrstop(msg);

	return 1;
}