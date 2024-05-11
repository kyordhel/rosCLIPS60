#include "bridge.h"
#include "clipswrapper.h"



/* ** ********************************************************
* Constructor
* *** *******************************************************/
Bridge::Bridge() : ClipsBridge(){}



/* ** ********************************************************
*
* Class methods
* Initialization
*
* *** *******************************************************/
void Bridge::initCLIPS(int argc, char **argv){
	// Call parent class method
	ClipsBridge::initCLIPS(argc, argv);

	// Further CLIPS initialization (routers, etc).
}


void Bridge::initServices(ros::NodeHandle& nh){
	// Call parent class method
	ClipsBridge::initServices(nh);

	// ros::ServiceServer
	// srv = nh.advertiseService("serviceName",
	// 	&Bridge::serviceHandler, this);
	// srvServers["serviceName"] = srv;
}


void Bridge::initPublishers(ros::NodeHandle& nh){
	// Call parent class method
	ClipsBridge::initPublishers(nh);

	// ros::Publisher
	// pub = nh.advertise<topicType>(topicName, 10);
	// publishers[topicName] = pub;
}


void Bridge::initSubscribers(ros::NodeHandle& nh){
	// Call parent class method
	ClipsBridge::initSubscribers(nh);

	// ros::Subscriber
	// sub = nh.subscribe<topicType>("topicName", 1,
	// 		boost::bind(&Bridge::subscriberHandler, this, _1));
	// subscribers["topicName"] = sub;
}



/* ** ********************************************************
*
* Class methods: Callbacks/Handlers
*
* *** *******************************************************/
// bool Bridge::serviceHandler(requestType& req, responseType& res){
// 	// Do something and assign res value
// }


// void Bridge::subscriberHandler(const messageTypePtr& msg){
// 	// Do something
// }
