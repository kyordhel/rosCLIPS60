#include <thread>
#include <iostream>
#include "ncurseswin.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

bool running;
ros::Publisher pub;
std::thread ncwThread;

void sendToCLIPS(const std::string& s);
void asyncNcwTask(NCursesWin* ncw);

int main(int argc, char** argv){
	ros::init(argc, argv, "clipscontrol");
	ros::NodeHandle nh;
	ros::Rate rate(10);

	pub = nh.advertise<std_msgs::String>("clips_in" , 10);
	pubfunc pf(sendToCLIPS);

	NCursesWin ncw;
	ncw.addPublisher(pf);

	ncwThread = std::thread(asyncNcwTask, &ncw);
	ros::spin();
	ncwThread.join();
	return 0;
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
