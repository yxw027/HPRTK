#include "BaseFunction/basefunction.h"
#include "BaseFunction/timesys.h"

/* divide observation file into 24 hour parts
 * argv1 : obs file
 * argv2 : out directory
 * -------------------------------------------------------------------------------- */
int dobsmain(int argc,char* argv[]) {
	string oFile=argv[1];
	string outDty=argv[2];
	string outName1=oFile.substr(oFile.length()-12,7),outName2=oFile.substr(oFile.length()-4,4);
	vector <string> vecHead;
	string buff;

	//file stream
	ifstream obsstm;
	fstream outstm;

	/* read observation file head to vecHead */
	obsstm.open(oFile,ios::in); vecHead.clear();
	int First_Time_Line=0; string timeLine;
	while (obsstm.is_open()&&getline(obsstm,buff)&&!obsstm.eof()) {
		vecHead.push_back(buff);
		if (buff.find("TIME OF FIRST OBS")!=string::npos) First_Time_Line=vecHead.size()-1;
		if (buff.find("END OF HEADER")!=string::npos) { 
			//read next line and set orignal start time
			getline(obsstm,buff); timeLine=buff.substr(0,12);
			break; 
		}
	}

	for (int i=0; i<24; i++) {
		//open out stream
		char nnn='a'+i;
		string Flag; Flag.clear(); Flag.push_back(nnn);
		string outFile=outDty+"\\"+outName1+Flag+outName2;
		outstm.open(outFile,ios::out);

		//hour and time 
		string staHour,endHour,endTime=timeLine;
		int2str(2," ",i,staHour);; int2str(2," ",i+1,endHour);
		endTime.replace(10,2,endHour);		

		//wirte head to stream
		for (int j=0; j<vecHead.size(); j++) {
			if (j==First_Time_Line) vecHead[j].replace(22,2,staHour);
			outstm << vecHead[j] <<"\n";
		}

		//write time line
		outstm << buff << "\n";
		/* read data to stream */
		while (obsstm.is_open()&&getline(obsstm,buff)&&!obsstm.eof()) {
			if (buff.find(endTime)!=string::npos) break; 
			outstm << buff << "\n";
		}

		//close out stream
		outstm.close();
	}
	obsstm.close();
	return 1;
}