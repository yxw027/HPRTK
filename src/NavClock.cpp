#include "GNSS/postpro.h"
#include "BaseFunction/basefunction.h"

int navmain(int argc,char* argv[]){
	option_t option;
	postsvr_t post;
	
	if (!option.readpostopt(argv[1])) return 0;

	/* read navigation file */
	post.postsvrini(&option);
	post.readNav();
	
	fstream Nav;
	Nav.open("G:\\HPRTK\\Test\\Single\\SDU_BDS\\Post\\2018274ClockBias\\GPSClock.txt",ios::out);

	string time="2018/10/01 ",buff;
	Nav.setf(ios::scientific);
	Nav << setprecision(4);
	if (Nav.is_open()) {
		//{loop of hour of one day} {loop of satellite}
		for (int h=0; h<24; h++) {
			time="2018/10/01 "+int2str(2,"0",h,buff)+":00";
			Nav << time;
			for (int prn=1; prn<33; prn++) {
				unsigned int sat=satno(SYS_GPS,prn);
				for (int i=0; i<post.nav->n; i++) {
					if (post.nav->eph[i].toe.sep.find(time)!=string::npos&&post.nav->eph[i].sat==sat)
						Nav << setw(12) << post.nav->eph[i].f0;
				}
			}
			Nav << "\n";
		}

	}

	Nav.close();

	return 0;
}