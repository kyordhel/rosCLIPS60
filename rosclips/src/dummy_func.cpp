#include "dummy_func.h"
#include "ros/ros.h"

extern "C" {
	#include "clips/clips.h"
	// #include "user_functions.h"
}

/**
 * Opens the network conection
 * @remark Dummy function
 * @return Always 1
 */
int CONDOR_open_network_conection() {
	ROS_INFO("Function %s has been deprecated. Request ignored.", __func__);

	/* check for exactly one argument */
	if(ArgCountCheck("CONDOR_open_network_conection", EXACTLY, 4) == -1)
		return -1;

	/* get the value for the 1st argument */
	// char* address = RtnLexeme(1);
	/* get the value for the 2nd argument */
	// int   port = RtnDouble(2);
	/* get the value for the 3rd argument */
	// int   buffer_size = RtnDouble(3);
	/* get the value for the 4th argument */
	// char* operation = RtnLexeme(4);

	return 1;
}
