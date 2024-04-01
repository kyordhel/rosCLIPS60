#include <regex>
#include <thread>
#include <iostream>
#include "ncurseswin.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

ros::Publisher pub;

std::string topicClipsIn     = "clips_in";
std::string topicClipsOut    = "clips_out";
std::string topicClipsStatus = "clips_status";

void sendToCLIPS(const std::string& s);
void asyncNcwTask(NCursesWin* ncw);
void cloutSubsCallback(const std_msgs::String::ConstPtr& msg, const std::string& topic, NCursesWin* ncw);
void clstatSubsCallback(const std_msgs::String::ConstPtr& msg, NCursesWin* ncw);

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


	// boost::bind(&ClipsBridge::subscriberCallback, this, _1, topic_name)
	pubfunc pf(sendToCLIPS);
	ncw.addPublisher(pf);

	std::thread ncwThread = std::thread(asyncNcwTask, &ncw);
	ros::spin();
	ncw.exitPoll();
	ncwThread.join();
	return 0;
}


void cloutSubsCallback(const std_msgs::String::ConstPtr& msg,
	                   const std::string& topic, NCursesWin* ncw){
	std::cout << "Message received: |" << msg->data << "|" << std::endl;
}


void clstatSubsCallback(const std_msgs::String::ConstPtr& msg, NCursesWin* ncw){
	static std::regex rxWatch("watching:(\\d+)");

	int flags;
	std::smatch match;

	if (!regex_search(msg->data, match, rxWatch))
		return;
	flags = std::stoi(match.str(1));
	ncw->setWatchFlags(flags);
}


void sendToCLIPS(const std::string& s){
	std_msgs::String msg;
	msg.data = s;
	pub.publish(msg);
}


void asyncNcwTask(NCursesWin* ncw){
	ncw->poll();
	std::cout << "GUI terminated. Awaiting for ROS..." << std::endl;
	ros::shutdown();
	std::cout << "Done." << std::endl;
}
