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
inline int bridge_publish_invoker(ClipsBridge& br, std::string const& topic_name, std::string const& message);
inline int bridge_subscribe_invoker(ClipsBridge& br, std::string const& topic_name, std::string const& fact_name);

/* ** ********************************************************
* C-compatible Prototypes
* *** *******************************************************/
extern "C" {
	VOID UserFunctions();
	int CONDOR_send_data_client_network();
	int CLIPS_rossub_wrapper();
	int CLIPS_rospub_wrapper();
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
	// int DefineFunction(functionName, functionType, functionPointer, actualFunctionName);
	// char *functionName, functionType, *actualFunctionName;
	// int (*functionPointer)();
	DefineFunction("CONDOR_open_network_conection", 'i',
		CONDOR_open_network_conection, "CONDOR_open_network_conection");
	DefineFunction("CONDOR_send_data_client_network", 'i',
		CONDOR_send_data_client_network, "CONDOR_send_data_client_network");

	// (rospub ?topic ?str)
	DefineFunction("rospub", 'i', CLIPS_rospub_wrapper, "CLIPS_rospub_wrapper");
	// (rossub ?topic ?fact)
	DefineFunction("rossub", 'i', CLIPS_rossub_wrapper, "CLIPS_rossub_wrapper");
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
	br.publish(s);
}

/**
 * Publishes the given message (second paramenter) to the specified topic (first parameter).
 * The topic type must be std_msgs::String
 * Wrapper for the CLIPS' rospub function. It calls ClipsBrige::publish via friend-function bridge_publish_invoker.
 * @return Zero if unwrapping was successful, -1 otherwise.
 */
int CLIPS_rospub_wrapper(){
	// (rospub ?topic ?str)
	if(ArgCountCheck("rospub", EXACTLY, 2) == -1)
		return -1;

	/* Get the values for the 1st, 2rd arguments */
	char* topic = RtnLexeme(1);
	char* message = RtnLexeme(2);

	/* It sends the data */
	return bridge_publish_invoker(bridge, topic, message);
}

inline
int bridge_publish_invoker(ClipsBridge& br, std::string const& topic_name, std::string const& message){
	return br.publish(topic_name, message);
}

/**
 * Subscribes to the specified topic (first parameter). Values are asserted to the specified fact (second parameter)
 * The topic type must be std_msgs::String
 * Wrapper for the CLIPS' rossub function. It calls ClipsBrige::subscribe via friend-function bridge_subscribe_invoker
 * @return Zero if unwrapping was successful, -1 otherwise.
 */
int CLIPS_rossub_wrapper(){
	// (rossub ?topic ?fact)
	// (assert (?fact ?str))
	if(ArgCountCheck("rossub", EXACTLY, 2) == -1)
		return -1;

	/* Get the values for the 1st, 2rd arguments */
	char* topic = RtnLexeme(1);
	char* fact_name = RtnLexeme(2);

	/* It sends the data */
	return bridge_subscribe_invoker(bridge, topic, fact_name);
}

inline
int bridge_subscribe_invoker(ClipsBridge& br, std::string const& topic_name, std::string const& fact_name){
	return br.subscribe(topic_name, fact_name);
}
