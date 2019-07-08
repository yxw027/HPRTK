#include "BaseFunction/timesys.h"

int timemain() {
	gtime_t time;
	time.doy2time(2016,61);
	time.time2str(0);
	return 1;
}