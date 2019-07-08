#include "Decode/decode.h"

/* decode data for kinds of formats ------------------------------------------------------------------
------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------- */
decode_data::decode_data(){
	format=-1;
	ephsat=0; dgps=NULL; Svr=NULL;
	/* class */
	obs.n=0; obs.data.assign(MAXOBS,obsd_t());
	nav.n=MAXSAT; nav.ng=MAXPRNGLO;
	nav.eph.assign(MAXSAT,eph_t()); nav.geph.assign(MAXPRNGLO,geph_t());

	for (int i=0; i<MAXSAT; i++) ssr[i]=ssr_t();
}
decode_data::~decode_data(){
	dgps=NULL; Svr=NULL;
}
int decode_data::decode(unsigned char data){
	return 0;
}