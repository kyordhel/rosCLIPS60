   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                     MAIN MODULE                     */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*	Jesus Savage					     */
/*	Mauricio Matamoros				     */
/*                                                           */
/*             FI-UNAM  Version 6.00  05/1/2008              */
/*             FI-UNAM  Version 6.00  04/1/2013              */
/*             FI-UNAM  Version 6.00  04/18/2023             */
/*                                                           */
/*************************************************************/

#include <cstdio>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

extern "C" {
	#include "clips/clips.h"
	// #include "user_functions.h"
}

#include "dummy_func.h"
#include "clips_bridge.h"


/* ** ********************************************************
* Global variables
* *** *******************************************************/
ClipsBridge bridge;

/* ** ********************************************************
* Prototypes
* *** *******************************************************/
int main(int argc, char **argv);
void send_message(ClipsBridge& br, std::string const& msg);

/* ** ********************************************************
* C-compatible Prototypes
* *** *******************************************************/
extern "C" {
	VOID UserFunctions();
	int CONDOR_send_data_client_network();
}


/* ** ********************************************************
* Main (program anchor)
* *** *******************************************************/
int main(int argc, char **argv){

	ros::init(argc, argv, "talker");
	ros::NodeHandle n;

	if( !bridge.init(argc, argv, n) )
		return -1;
	ros::Rate loop_rate(10);

	bridge.runAsync();
	// Give control to ROS
	ros::spin();
	return 0;
}


/* ** ********************************************************
* Function definitions
* *** *******************************************************/
VOID UserFunctions(){
	DefineFunction("CONDOR_open_network_conection", 'i',
		CONDOR_open_network_conection, "CONDOR_open_network_conection");
	DefineFunction("CONDOR_send_data_client_network", 'i',
		CONDOR_send_data_client_network, "CONDOR_send_data_client_network");
}

int CONDOR_send_data_client_network(){
	/* check for exactly two arguments */
	if(ArgCountCheck("CONDOR_send_data_client_network", EXACTLY, 2) == -1)
		return -1;

	/* get the values for the 1st, 2rd arguments */
	int c = RtnDouble(1);
	char* message = RtnLexeme(2);

	/* It sends the data */
	send_message(bridge, message);
	return 0;
}

void send_message(ClipsBridge& br, std::string const& s){
	std_msgs::String msg;
	msg.data = s;
	br.pub.publish(msg);
	ros::spinOnce();
}

