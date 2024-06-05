/* ** *****************************************************************
* main.cpp
*
* Author: Mauricio Matamoros
*
* ** *****************************************************************/
/** @file clipscontrol/main.cpp
 * Anchor point (main function) for the clipscontrol node
 */

/** @cond */
#include <regex>
#include <thread>
#include <iostream>
#include "ncurseswin.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
/** @endcond */


using namespace clipscontrol;

/* ** ********************************************************
* Global variables
* *** *******************************************************/

/**
 * A ros::Publisher object that publishes to the topic rosclips listens to
 */
ros::Publisher pub;

/**
 * The topic rosclips listens to
 */
std::string topicClipsIn     = "clips_in";

/**
 * The topic to read from rosclips (or any topic to listen to)
 */
std::string topicClipsOut    = "clips_out";

/**
 * The topic where rosclips reports its status
 */
std::string topicClipsStatus = "clips_status";



/* ** ********************************************************
* Prototypes
* *** *******************************************************/
void sendToCLIPS(const std::string& s);
void asyncNcwTask(NCursesWin* ncw);
void cloutSubsCallback(const std_msgs::String::ConstPtr& msg, const std::string& topic, NCursesWin* ncw);
void clstatSubsCallback(const std_msgs::String::ConstPtr& msg, NCursesWin* ncw);
void asyncCheckClipsOnlineTask(NCursesWin* ncw, const ros::Subscriber* sub);


/* ** ********************************************************
* Main (program anchor)
* *** *******************************************************/
/**
 * Program anchor
 * @param  argc The number of arguments to the program
 * @param  argv The arguments passed to the program
 * @return      The program exit code
 */
int main(int argc, char** argv){

	NCursesWin ncw; // Creates and initializes gui
	ros::init(argc, argv, "clipscontrol");

	// Setup ROS
	ros::NodeHandle nh;
	ros::Rate rate(10);
	// The topic to write/send commands to
	pub = nh.advertise<std_msgs::String>(topicClipsIn , 10);
	// The topic to read clps output from
	ros::Subscriber subclo = nh.subscribe<std_msgs::String>(topicClipsOut, 10,
		boost::bind(cloutSubsCallback, _1, topicClipsOut, &ncw) );
	// The topic to read status from
	ros::Subscriber subcls = nh.subscribe<std_msgs::String>(topicClipsStatus, 1,
		boost::bind(clstatSubsCallback, _1, &ncw) );


	// boost::bind(&ClipsBridge::subscriberCallback, this, _1, topicName)
	pubfunc pf(sendToCLIPS);
	ncw.addPublisher(pf);

	std::thread ncwThread = std::thread(asyncNcwTask, &ncw);
	std::thread ccoThread = std::thread(asyncCheckClipsOnlineTask, &ncw, &subcls);
	ros::spin();
	ncw.exitPoll();
	ncwThread.join();
	ccoThread.join();
	return 0;
}


/* ** ********************************************************
* Function definitions
* *** *******************************************************/
/**
 * Callback for the rosclips clipsout topic subscription.
 * It will print the message in the main window of the GUI.
 * @param msg    The received message
 * @param topic  The topic from where the message comes from
 * @param ncw    The GUI main window to update
 */
void cloutSubsCallback(const std_msgs::String::ConstPtr& msg,
	                   const std::string& topic, NCursesWin* ncw){
	// ncw->print("["+topic+"]: " + msg->data);
	if(msg->data.back() != '\n')
		ncw->print(msg->data + "\n");
	else
		ncw->print(msg->data);
}


/**
 * Callback for the rosclips status topic subscription.
 * It will update the watch flags in the main window of the GUI
 * @param msg The received message
 * @param ncw The GUI main window to update
 */
void clstatSubsCallback(const std_msgs::String::ConstPtr& msg, NCursesWin* ncw){
	static std::regex rxWatch("watching:(\\d+)");

	int flags;
	std::smatch match;

	if (!regex_search(msg->data, match, rxWatch))
		return;
	flags = std::stoi(match.str(1));
	ncw->setWatchFlags(flags);
}


/**
 * Sends the given strng to rosclips
 * @param s The string to send
 */
void sendToCLIPS(const std::string& s){
	std_msgs::String msg;
	msg.data = s;
	pub.publish(msg);
}


/**
 * Polls the GUI main window. Executed in a separated thread.
 * @param ncw The GUI main window
 */
void asyncNcwTask(NCursesWin* ncw){
	ncw->poll();
	std::cout << "GUI terminated. Awaiting for ROS..." << std::endl;
	ros::shutdown();
	std::cout << "Done." << std::endl;
}


/**
 * Prdiodically checks whether rosclips is online
 * @param ncw The GUI main window
 * @param sub The ros::Subscriber object listening to the rosclips status topic
 */
void asyncCheckClipsOnlineTask(NCursesWin* ncw, const ros::Subscriber* sub){
	do{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		ncw->setCLIPSStatus(sub->getNumPublishers() > 0 ?
			NCursesWin::CLIPSStatus::Online : NCursesWin::CLIPSStatus::Offline);
	}
	while( ros::ok() );
}
