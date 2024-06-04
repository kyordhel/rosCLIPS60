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
/** @file main.cpp
 * Anchor point (main function) for the clipscontrol node
 */

/** @cond */
#include <cstdio>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
/** @endcond */

#include "bridge.h"
#include "clipswrapper.h"

/* ** ********************************************************
* Global variables
* *** *******************************************************/
/**
 * The bridge to interface with CLIPS
 */
Bridge bridge;

/* ** ********************************************************
* Prototypes
* *** *******************************************************/
int main(int argc, char **argv);
inline int bridge_publish_invoker(ClipsBridge& br, std::string const& topicName, std::string const& message);
inline int bridge_subscribe_invoker(ClipsBridge& br, std::string const& topicName, std::string const& factName);

/* ** ********************************************************
* C-compatible Prototypes
* *** *******************************************************/
extern "C" {
	void UserFunctions();
	int CLIPS_rossub_wrapper();
	int CLIPS_rospub_wrapper();
}


/* ** ********************************************************
* Main (program anchor)
* *** *******************************************************/
/**
 * Program anchor
 * @param  argc The number of arguments to the program
 * @param  argv The arguments passed to the program
 * @return      The program exit code
 */
int main(int argc, char **argv){

	ros::init(argc, argv, "talker");
	ros::NodeHandle n;

	if( !bridge.init(argc, argv, n) )
		return -1;
	ros::Rate loop_rate(10);

	bridge.runAsync();
	// Give control to ROS
	ros::spin();
	bridge.stop();
	std::cout << std::endl;
	return 0;
}


/* ** ********************************************************
* Function definitions
* *** *******************************************************/

/**
 * Defines and sets up userfunctions to use within clips scripts
 *
 * @remark Function invoked by CLIPS to setup user functions
 */
void UserFunctions(){
	// int clips::defineFunction(functionName, functionType, functionPointer, actualFunctionName);
	// char *functionName, functionType, *actualFunctionName;
	// int (*functionPointer)();

	// (rospub ?topic ?str)
	// DefineFunction("rospub", 'i', CLIPS_rospub_wrapper, "CLIPS_rospub_wrapper");
	clips::defineFunction("rospub", 'i', CLIPS_rospub_wrapper);
	// (rossub ?topic ?fact)
	// DefineFunction("rossub", 'i', CLIPS_rossub_wrapper, "CLIPS_rossub_wrapper");
	clips::defineFunction("rossub", 'i', CLIPS_rossub_wrapper);
}


/**
 * Publishes the given message (second paramenter) to the specified topic (first parameter).
 * The topic type must be std_msgs::String
 * Wrapper for the CLIPS' rospub function. It calls ClipsBrige::publish via friend-function bridge_publish_invoker.
 * @return Zero if unwrapping was successful, -1 otherwise.
 */
int CLIPS_rospub_wrapper(){
	// (rospub ?topic ?str)
	if(clips::argCountCheck("rospub", clips::ArgCountRestriction::Exactly, 2) == -1)
		return -1;

	/* Get the values for the 1st, 2rd arguments */
	std::string topic = clips::returnLexeme(1);
	std::string message = clips::returnLexeme(2);

	/* It sends the data */
	return bridge_publish_invoker(bridge, topic, message);
}

inline
int bridge_publish_invoker(ClipsBridge& br, std::string const& topicName, std::string const& message){
	return br.publish(topicName, message);
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
	if(clips::argCountCheck("rossub", clips::ArgCountRestriction::Exactly, 2) == -1)
		return -1;

	/* Get the values for the 1st, 2rd arguments */
	std::string topic = clips::returnLexeme(1);
	std::string factName = clips::returnLexeme(2);

	/* It sends the data */
	return bridge_subscribe_invoker(bridge, topic, factName);
}

inline
int bridge_subscribe_invoker(ClipsBridge& br, std::string const& topicName, std::string const& factName){
	return br.subscribe(topicName, factName);
}
