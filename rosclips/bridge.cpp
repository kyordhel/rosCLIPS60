/* ** *****************************************************************
* bridge.h
*
* Author: Mauricio Matamoros
*
* ** *****************************************************************/

#include "bridge.h"
#include "clipswrapper.h"



/* ** ********************************************************
* Constructor
* *** *******************************************************/
Bridge::Bridge() : ClipsBridge(),
	serviceQuery("/clips/query"){}



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
	clips::QueryRouter& qr = clips::QueryRouter::getInstance();
	qr.addLogicalName("wdisplay"); // Capture display info
	qr.addLogicalName("wtrace");   // Capture trace info
	qr.addLogicalName("stdout");   // Capture everything else

}


void Bridge::initServices(ros::NodeHandle& nh){
	// Call parent class method
	ClipsBridge::initServices(nh);

	ros::ServiceServer
	srv = nh.advertiseService(serviceQuery,
		&Bridge::srvQueryKDB, this);
	srvServers[serviceQuery] = srv;
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

bool Bridge::srvQueryKDB(rosclips::QueryKDB::Request& req, rosclips::QueryKDB::Response& res){
	clips::query(req.query, res.result);
	return true;
}
