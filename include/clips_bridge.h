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
	// VOID *vTheModule;
	// int show_facts;

	bool flg_rules;
	bool flg_facts;
	bool flg_trace;

	// int number;
	int num;
	bool running;

	sync_queue<std::string> queue;
	std::thread asyncThread;

	std::string topicIn;
	std::string topicOut;

	ros::NodeHandle* nodeHandle;
	std::unordered_map<std::string, ros::Publisher> publishers;
	std::unordered_map<std::string, ros::Subscriber> subscribers;
	std::unordered_map<std::string, ros::ServiceServer> srvServers;
	std::unordered_map<std::string, std::string> topic_facts;

public:
	ClipsBridge();
	~ClipsBridge();

	// Disable copy constructor and assignment op.
private:
	ClipsBridge(ClipsBridge const& obj);
	ClipsBridge& operator=(ClipsBridge const&);


public:
	bool init(int argc, char **argv, ros::NodeHandle& nh);
	bool loadFile(std::string const& fpath);
	bool loadClp(std::string const& fpath);
	bool loadDat(std::string const& fpath);
	void run();
	void runAsync();
	void stop();
	void subscriberCallback(std_msgs::String::ConstPtr const& msg, std::string const& topic);

private:
	void assertFact(std::string const& s);
	void sendCommand(std::string const& s);
	// std::string& eval(std::string const& s); // Unsupported in 6.0

	void initCLIPS(int argc, char **argv);
	void initPublishers(ros::NodeHandle& nh);
	void initServices(ros::NodeHandle& nh);
	void initSubscribers(ros::NodeHandle& nh);
	void parseMessage(const std::string& m);
	void handleCommand(const std::string& c);
	void handleLog(const std::string& arg);
	void handlePrint(const std::string& arg);
	void handleRun(const std::string& arg);
	void handleWatch(const std::string& arg);
	bool parseArgs(int argc, char **argv);
	void printDefaultArgs(std::string const& pname);
	void printHelp(std::string const& pname);
	bool publish(std::string const& message);
	bool publish(std::string const& topic_name, std::string const& message);
	bool subscribe(std::string const& topic_name, std::string const& fact_name);

	void printFacts();
	void printRules();
	void printAgenda();

	friend void send_message(ClipsBridge& br, std::string const& msg);
	friend int bridge_publish_invoker(ClipsBridge& br, std::string const& topic_name, std::string const& message);
	friend int bridge_subscribe_invoker(ClipsBridge& br, std::string const& topic_name, std::string const& fact_name);
};

#endif // __CLIPS_BRIDGE_H__
