#include "sign.h"

int main(int argc, char** argv)
{
	ros::init(argc,argv,"scanline");
        sign_mark begin;
        begin.Msg_Subsribe();

        ros::spin();
	return 0;
}
