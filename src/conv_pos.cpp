#include "BaseFunction/basefunction.h"

/* convert xyz to enu */
int x2emain(int argc, char* argv[]) {
	string xyzfile=argv[1], enufile=argv[2],buff;
	ifstream xyzf;
	fstream enuf;
	xyzf.open(xyzfile); enuf.open(enufile,ios::out);

	int xyz2enu=0; double oriblh[3]={ 0 }, orixyz[3]={ 0 };
	while (xyzf.is_open()&&getline(xyzf,buff)) {
		if (xyz2enu==1) {
			double xyz[3]={ 0 },rr[3]={ 0 }, enu[3]={ 0 };
			for (int i=0; i<3; i++) {
				str2double(buff.substr(23+i*15,15),xyz[i]);
				rr[i]=xyz[i]-orixyz[i];
			}
			ecef2enu(oriblh,rr,enu);
			string strenu;
			for (int i=0; i<3; i++) {
				doul2str(15,4," ",enu[i],strenu);
				buff.replace(23+i*15,15,strenu);
			}
		}
		else if (xyz2enu==0&&buff.find("origin")!=string::npos) {
			for (int i=0; i<3; i++) str2double(buff.substr(23+i*15,15),orixyz[i]);
			ecef2pos(orixyz,WGS84,oriblh);
			xyz2enu=1;
		}
		enuf << buff << "\n";
	}
	xyzf.close(); enuf.close();

	return 1;
}