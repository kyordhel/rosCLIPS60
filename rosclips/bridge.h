/* ** *****************************************************************
* bridge.h
*
* Author: Mauricio Matamoros
*
* ** *****************************************************************/
/** @file bridge.h
 * Definition of the Bridge class: A bridge between ROS and CLIPS
 * for the rosclips node
 */

#ifndef __BRIDGE_H__
#define __BRIDGE_H__
#pragma once

/** @cond */
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
/** @endcond */

#include "clips_bridge.h"


class ClipsBridge;

/**
 * Implements a specialized bridge for the rosclips node
 *
 * Inherits from ClipsBridge
 */
class Bridge : public ClipsBridge{
public:
	/**
	 * Initializes a new instance of bridge
	 */
	Bridge();

// Disable copy and constructructor and assignment op
private:
	/**
	 * Copy constructor disabled
	 */
	Bridge(Bridge const& obj)        = delete;
	/**
	 * Copy assignment operator disabled
	 */
	Bridge& operator=(Bridge const&) = delete;


// Overriden class members
protected:
	/**
	 * Initializes CLIPS.
	 * It calls clips::initialize(), clips::rerouteStdin(argc, argv)
	 * and clips::clear() in that order before loading the file
	 * specified by ClipsBridge::clips_file.
	 * @param argc The main's argc after being passed to ros::init
	 * @param argv The main's argv after being passed to ros::init
	 */
	virtual void initCLIPS(int argc, char **argv);

	/**
	 * Initializes all topic subscribers, including the topic
	 * specified by topicIn.
	 * @param nh The NodeHandle provided by the ROS node (will
	 *           pass *ClipsBridge::nodeHandle. Provided to keep
	 *           code short).
	 */
	virtual void initPublishers(ros::NodeHandle& nh);

	/**
	 * Initializes all topic subscribers, including the topic
	 * specified by topicIn.
	 * @param nh The NodeHandle provided by the ROS node (will
	 *           pass *ClipsBridge::nodeHandle. Provided to keep
	 *           code short).
	 */
	virtual void initServices(ros::NodeHandle& nh);

	/**
	 * Initializes all topic subscribers, including the topic
	 * specified by topicIn.
	 * @param nh The NodeHandle provided by the ROS node (will
	 *           pass *ClipsBridge::nodeHandle. Provided to keep
	 *           code short).
	 */
	virtual void initSubscribers(ros::NodeHandle& nh);

private:
	// Additional class methods go here
	// E.g. special ROS callbacks for messages and services
};

#endif // __BRIDGE_H__
