#include "BaseFunction/basefunction.h"
#include "BaseFunction/timesys.h"
#include "GNSS/EphModel/satellite.h"
#include "GNSS/ReadFile/readfile.h"
#include "ConfigFile/config.h"

int ephmain(int argc,char* argv[]) {
	string eph_File=argv[1],out=argv[2];
	int time_Interval; //unit s
	string warning="Parameter: 1:eph_file 2:out_file 3:time_interval 4:satellite_system 5:format\n";
	warning+="3: time_intervel    : in unit of second\n";
	warning+="4: satellite_system : 1:gps+2:sbas+4:glo+8:gal+16:qzs+32:bds\n";
	warning+="5: format           : 0:xyz, 1:blh\n";
	if (!str2int(argv[3],time_Interval)) { 
		cout << "time_interval error!\n" << warning;
		return 0; 
	}

	nav_t nav;						/* satellite information */
	prcopt_t prcopt;				/* processing options */
	ineph_t readEph(eph_File,2);	/* read precise ephemeris stream */
	preciseph_t pre_Eph;
	if (!str2int(argv[4],prcopt.navsys)){ 
		cout << "satellite_system error!\n" << warning; 
		return 0; 
	}
	int format=1;
	if (!str2int(argv[5],format)) { 
		cout << "format error!\n" << warning;
		return 0; 
	}
	readEph.readsp3(&nav,&prcopt);
	/* satellite and output file */
	int sys[MAXSAT];
	string prn[MAXSAT];
	for (int i=0 ;i<MAXSAT; i++) sys[i]=satno2id(i+1,prn[i]);

	/* write solution to out_File */
	obsd_t obs;
	obs.sigtime=readEph.ephtime;
	/* start time of predicted presice ephemeris */
	obs.sigtime.timeadd(86400)->time2epoch();
	/* time items and wait for start */
	gtime_t nowtime;
	unsigned int start; int runtime;
	//nowtime.timeget()->utc2gpst();
	nowtime.str2time("2017 10 22  6  0  0.00000000");
	sleepms((int)(1000*obs.sigtime.timediff(nowtime)));

	/* loop of time */
	fstream output;
	string CMD;
	for (int i=0; i<86400/time_Interval; i++) {
		start=tickget();
		obs.sigtime.timeadd(i*time_Interval);
		string time;
		int2str(5,"0",i*time_Interval,time);
		output.open(out+time+".txt",ios::out);

		/* loop of satellite */
		for (int j=0; j<MAXSAT; j++) {
			if (!(sys[j]&prcopt.navsys)) continue;
			obs.sat=j+1;
			if (pre_Eph.satpos(&obs,0,&nav)) {
				double blh[3]={0.0};
				if (format) {
					ecef2pos(obs.posvel,CGCS2000,blh);
					output << setw(14) << R2D*blh[1];
					output << setw(14) << R2D*blh[0];
				}
				else for (int k=0; k<3; k++) output << setw(14) << obs.posvel[k];
				output << "     " << prn[j] << "\n";
			}
		}
		output.close();
		/* map */
		CMD="gmt pscoast -R-180/180/-90/90 -JQ90/0/26.6 -A10000 -G255/250/205 -S135/206/250 -Df -B30/30:.\"BeiDou Satellite System\": -X1.3 -Y5 -K >";
		CMD+=time+".ps";
		system(CMD.c_str());
		/* satellite */
		CMD="gmt psxy "; CMD+=out+time+".txt -R -J -O -Sc0.5 -Gyellow -Wthinnest >>";
		CMD+=time+".ps";
		system(CMD.c_str());
		/* remove file */

		runtime = (int)(tickget()-start);
		sleepms((int)(time_Interval*1000)-runtime);
	}
	return 1;
}