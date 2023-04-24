#ifndef __CLIPS_BRIDGE_H__
#define __CLIPS_BRIDGE_H__


#include <thread>
#include <string>
#include <unordered_map>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sync_queue.h"


class ClipsBridge{
private:
	// char buf[10];
	// char dummy[30];
	std::string clips_file;
	std::string logicalName;
	// VOID *vTheModule;
	// int show_facts;

	bool flg_rules;
	bool flg_facts;
	bool flg_trace;

	int num;
	// int number;

	long start;
	long end;
	long max;

	sync_queue<std::string> queue;
	std::thread asyncThread;

	std::string topic_in;
	std::string topic_out;

	ros::NodeHandle* nodeHandle;
	std::unordered_map<std::string, ros::Publisher> publishers;
	std::unordered_map<std::string, ros::Subscriber> subscribers;
	std::unordered_map<std::string, std::string> topic_facts;

public:
	ClipsBridge();

	// Disable copy constructor and assignment op.
private:
	ClipsBridge(ClipsBridge const& obj);
	ClipsBridge& operator=(ClipsBridge const&);


public:
	bool init(int argc, char **argv, ros::NodeHandle& nh);
	void load_clp(std::string const& fpath);
	void load_dat(std::string const& fpath);
	void run();
	void runAsync();
	void subscriberCallback(std_msgs::String::ConstPtr const& msg, std::string const& topic);

private:
	void assertFact(std::string const& s);
	void initCLIPS(int argc, char **argv);
	void parseMessage(std::string& m);
	bool parseArgs(int argc, char **argv);
	void printDefaultArgs(std::string const& pname);
	void printHelp(std::string const& pname);
	int publish(std::string const& message);
	int publish(std::string const& topic_name, std::string const& message);
	int subscribe(std::string const& topic_name, std::string const& fact_name);

	friend void send_message(ClipsBridge& br, std::string const& msg);
	friend int bridge_publish_invoker(ClipsBridge& br, std::string const& topic_name, std::string const& message);
	friend int bridge_subscribe_invoker(ClipsBridge& br, std::string const& topic_name, std::string const& fact_name);
};

#endif // __CLIPS_BRIDGE_H__
