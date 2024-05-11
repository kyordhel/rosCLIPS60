#ifndef __BRIDGE_H__
#define __BRIDGE_H__

#pragma once

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include "clips_bridge.h"


class ClipsBridge;

class Bridge : public ClipsBridge{
public:
	Bridge();

// Disable copy and constructructor and assignment op
private:
	Bridge(Bridge const& obj)        = delete;
	Bridge& operator=(Bridge const&) = delete;


// Overriden class members
protected:
	virtual void initCLIPS(int argc, char **argv);
	virtual void initPublishers(ros::NodeHandle& nh);
	virtual void initServices(ros::NodeHandle& nh);
	virtual void initSubscribers(ros::NodeHandle& nh);

private:
	// Additional class methods go here
	// E.g. special ROS callbacks for messages and services
};

#endif // __BRIDGE_H__
