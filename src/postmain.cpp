/* This is just for test! ---------------------------------------------------------------------------
* 2017/10/18	read O file
--------------------------------------------------------------------------------------------------- */
#include "GNSS/postpro.h"

int main(int argc,char* argv[]){
	option_t option;
	postsvr_t post;
	
	if (!option.readpostopt(argv[1])) return 0;
	
	if (!post.postsvrini(&option)) return 0;
	post.Post_Position_Epoch();

	return 0;
}