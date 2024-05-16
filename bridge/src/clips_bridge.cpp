#include "clips_bridge.h"

#include <cstdio>
#include <fstream>
#include <sstream>

#include "utils.h"
#include "clipswrapper.h"



/* ** ********************************************************
* Macros
* *** *******************************************************/
#define contains(s1,s2) s1.find(s2) != std::string::npos


/* ** ********************************************************
* Local helpers
* *** *******************************************************/
static inline
bool ends_with(const std::string& s, const std::string& end){
	if (end.size() > s.size()) return false;
	return std::equal(end.rbegin(), end.rend(), s.rbegin());
}

static inline
void split_path(const std::string& fpath, std::string& dir, std::string& fname){
	size_t slashp = fpath.rfind("/");
	if(slashp == std::string::npos){
		dir = std::string();
		fname = fpath;
		return;
	}
	dir = fpath.substr(0, slashp);
	fname = fpath.substr(slashp+1);
}

static inline
std::string get_current_path(){
	char buff[FILENAME_MAX];
	getcwd(buff, sizeof(buff));
	return std::string(buff);
}




/* ** ********************************************************
* Constructor
* *** *******************************************************/
ClipsBridge::ClipsBridge():
	// clips_file("cubes.dat"),
	topicIn("clips_in"), topicOut("clips_out"), topicStatus("clips_status"),
	flg_facts(false), flg_rules(false), nodeHandle(NULL), clppath(get_current_path()){
}

ClipsBridge::~ClipsBridge(){
	stop();
}


/* ** ********************************************************
*
* Class methods
* Initialization
*
* *** *******************************************************/
bool ClipsBridge::init(int argc, char **argv, ros::NodeHandle& nh, int delay){
	if( !parseArgs(argc, argv) ) return false;

	this->nodeHandle = &nh;
	initSubscribers(nh);
	initPublishers(nh);
	initServices(nh);

	std::this_thread::sleep_for(std::chrono::milliseconds(delay));

	initCLIPS(argc, argv);
	publishStatus();

	return true;
}


void ClipsBridge::initCLIPS(int argc, char **argv){
	clips::initialize();
	clips::rerouteStdin(argc, argv);
	clips::clear();
	std::cout << "Clips ready" << std::endl;

	// Load clp files specified in file
	loadFile(clips_file);
	if(flg_facts) clips::toggleWatch(clips::WatchItem::Facts);
	if(flg_rules) clips::toggleWatch(clips::WatchItem::Rules);
	// clips::reset();

	// clips::printFacts();
}


void ClipsBridge::initPublishers(ros::NodeHandle& nh){
	ros::Publisher pub = nh.advertise<std_msgs::String>(topicOut, 100);
	publishers[topicOut] = pub;

	pub = nh.advertise<std_msgs::String>(topicStatus, 1);
	publishers[topicStatus] = pub;
}


void ClipsBridge::initServices(ros::NodeHandle& nh){
}


void ClipsBridge::initSubscribers(ros::NodeHandle& nh){
	ros::Subscriber
	sub = nh.subscribe<std_msgs::String>(topicIn, 100,
		boost::bind(&ClipsBridge::subscriberCallback, this, _1, topicIn)
	);
	subscribers[topicIn] = sub;
}



/* ** ********************************************************
*
* Class methods: Clips wrappers
*
* *** *******************************************************/
void ClipsBridge::assertFact(std::string const& s) {
	std::string as = "(network " + s + ")";
	clips::assertString( "(network " + s + ")" );
	clips::setFactListChanged(0);
	ROS_INFO("Asserted string %s", as.c_str());
}


void ClipsBridge::clearCLIPS(){
	clips::clear();
	ROS_INFO("KDB cleared (clear)");
}


void ClipsBridge::resetCLIPS(){
	clips::reset();
	ROS_INFO("KDB reset (reset)");
}


void ClipsBridge::sendCommand(std::string const& s){
	ROS_INFO("Executing command: %s", s.c_str());
	clips::sendCommand(s);
}


bool ClipsBridge::loadClp(const std::string& fpath){
	ROS_INFO("Loading file '%s'...", fpath.c_str() );
	if( !clips::load( fpath ) ){
		ROS_ERROR("Error in file '%s' or does not exist", fpath.c_str());
		return false;
	}
	ROS_INFO("File %s loaded successfully", fpath.c_str());
	return true;
}


bool ClipsBridge::loadDat(const std::string& fpath){
	if( fpath.empty() ) return false;
	std::ifstream fs;
	fs.open(fpath);

	if( fs.fail() || !fs.is_open() ){
		ROS_ERROR("File '%s' does not exists", fpath.c_str());
		return false;
	}

	bool err = false;
	std::string line, fdir, fname;
	std::string here = get_current_path();
	split_path(fpath, fdir, fname);
	if(!fdir.empty()) chdir(fdir.c_str());
	ROS_INFO("Loading '%s'...", fname.c_str());
	while(!err && std::getline(fs, line) ){
		if(line.empty()) continue;
		// size_t slashp = fpath.rfind("/");
		// if(slashp != std::string::npos) line = fdir + line;
		if (!loadClp(line)) err = true;
	}
	fs.close();
	chdir(here.c_str());
	ROS_INFO(err? "Aborted." : "Done.");

	return !err;
}


bool ClipsBridge::loadFile(std::string const& fpath){
	ROS_INFO("Current path '%s'\n", get_current_path().c_str() );
	if(ends_with(fpath, ".dat"))
		return loadDat(fpath);
	else if(ends_with(fpath, ".clp"))
		return loadClp(fpath);
	return false;
}


/**
 * Parses messages from subscribed topics
 * Re-implements original parse_network_message by Jesús Savage
 * @param m String contained in the topic message
 */
void ClipsBridge::parseMessage(const std::string& m){
	if((m[0] == 0) && (m.length()>1)){
		handleCommand(m.substr(1));
		return;
	}
	assertFact( m );
}


static inline
void splitCommand(const std::string& s, std::string& cmd, std::string& arg){
	std::string::size_type sp = s.find(" ");
	if(sp == std::string::npos){
		cmd = s;
		arg.clear();
	}
	else{
		cmd = s.substr(0, sp);
		arg = s.substr(sp+1);
	}
}


void ClipsBridge::handleCommand(const std::string& c){
	std::string cmd, arg;
	splitCommand(c, cmd, arg);

	// ROS_INFO("Received command %s", c.c_str());
	if(cmd == "assert") { clips::assertString(arg); }
	else if(cmd == "reset") { resetCLIPS(); }
	else if(cmd == "clear") { clearCLIPS(); }
	else if(cmd == "raw")   { sendCommand(arg); }
	else if(cmd == "print") { handlePrint(arg); }
	else if(cmd == "watch") { handleWatch(arg); }
	else if(cmd == "load")  { loadFile(arg); }
	else if(cmd == "run")   { handleRun(arg); }
	else if(cmd == "log")   { handleLog(arg); }
	else return;

	// ROS_INFO("Handled command %s", c.c_str());
}


void ClipsBridge::handleLog(const std::string& arg){
}


void ClipsBridge::handlePrint(const std::string& arg){
	if(arg == "facts"){       clips::printFacts();  }
	else if(arg == "rules"){  clips::printRules();  }
	else if(arg == "agenda"){ clips::printAgenda(); }
}


void ClipsBridge::handleRun(const std::string& arg){
	int n = std::stoi(arg);
	clips::run(n);
}


void ClipsBridge::handleWatch(const std::string& arg){
	if(arg == "functions"){    clips::toggleWatch(clips::WatchItem::Deffunctions); }
	else if(arg == "globals"){ clips::toggleWatch(clips::WatchItem::Globals);      }
	else if(arg == "facts"){   clips::toggleWatch(clips::WatchItem::Facts);        }
	else if(arg == "rules"){   clips::toggleWatch(clips::WatchItem::Rules);        }
	publishStatus();
}



/* ** ********************************************************
*
* Class methods: ROS-related
*
* *** *******************************************************/
bool ClipsBridge::publish(std::string const& message){
	return publish(topicOut, message);
}


bool ClipsBridge::publish(std::string const& topic_name, std::string const& message){
	if (publishers.find(topic_name) == publishers.end()){
		// Topic not in publishers. Insert.
		ros::Publisher pub = nodeHandle->advertise<std_msgs::String>( std::string(topic_name), 10);
		publishers[topic_name] = pub;
		ROS_INFO("Added publisher for topic %s", topic_name.c_str());
	}
	ros::Publisher& pub = publishers[topic_name];
	if (pub.getNumSubscribers() < 1) return false;
	std_msgs::String msg;
	msg.data = message;
	pub.publish(msg);
	// ROS_INFO("Published <%s> on %s", message.c_str(), pub.getTopic().c_str());
	return true;
}


bool ClipsBridge::publishStatus(){
	std::string status("watching:" + std::to_string((int)clips::getWatches()));
	return publish(topicStatus, status);
}


bool ClipsBridge::subscribe(std::string const& topic_name, std::string const& fact_name){
	if (subscribers.find(topic_name) == subscribers.end()){
		// Topic not in subscribers. Insert.
		// ros::Subscriber sub = nodeHandle->subscribe(topic_name, 10, &ClipsBridge::subscriberCallback, this);
		ros::Subscriber sub = nodeHandle->subscribe<std_msgs::String>(topicIn, 100,
			boost::bind(&ClipsBridge::subscriberCallback, this, _1, topic_name)
		);
		ROS_INFO("Subscribed to topic %s", topic_name.c_str());
		subscribers[topic_name] = sub;
		topic_facts[topic_name] = fact_name;
	}
	return true;
}


/* ** ********************************************************
*
* Class methods: Multithreaded execution
*
* *** *******************************************************/
void ClipsBridge::stop(){
	running = false;
	if(asyncThread.joinable())
		asyncThread.join();
}

void ClipsBridge::runAsync(){
	asyncThread = std::thread(&ClipsBridge::run, this);
}


void ClipsBridge::run(){
	if(running) return;
	running = true;
	// Loop forever
	while(running && ros::ok()){
		if( queue.empty() ){
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
			continue;
		}
		parseMessage( queue.consume() );
	}
}



/* ** ********************************************************
*
* Class methods: Callbacks
*
* *** *******************************************************/
void ClipsBridge::subscriberCallback(std_msgs::String::ConstPtr const& msg, std::string const& topic) {
	if(msg->data.length() < 1) return;
	if (topic == topicIn){
		queue.produce( msg->data );
	}
	else if(msg->data[0] != 0)
		clips::assertString( "(" + topic + " " + msg->data + ")" );
}



/* ** ********************************************************
*
* Class methods: Misc
*
* *** *******************************************************/
bool ClipsBridge::parseArgs(int argc, char **argv){
	std::string pname(argv[0]);
	pname = pname.substr(pname.find_last_of("/") + 1);
	// Read input parameters
	if (argc <= 1) {
		printDefaultArgs(pname);
		return true;
	}

	for(int i = 1; i < argc; ++i){
		if (!strcmp(argv[i], "-h") || (i+1 >= argc) ){
			printHelp( pname );
			return false;
		}
		else if (!strcmp(argv[i],"-d")){
			clppath = std::string(argv[++i]);

			if(chdir(argv[i]) != 0){
				fprintf(stderr, "Can't access {%s}: %s\n", argv[i], strerror(errno));
				printf("Reset clppath  to {%s}\n", get_current_path().c_str() );
			}
		}
		else if (!strcmp(argv[i],"-e")){
			clips_file = std::string(argv[++i]);
		}
		else if (!strcmp(argv[i],"-w")){
			flg_facts = atoi(argv[++i]);
		}
		else if (!strcmp(argv[i],"-r")){
			flg_rules = atoi(argv[++i]);
		}
		else if (!strcmp(argv[i],"-i")){
			topicIn = std::string(argv[++i]);
		}
		else if (!strcmp(argv[i],"-o")){
			topicOut = std::string(argv[++i]);
		}
		else if (!strcmp(argv[i],"-s")){
			topicStatus = std::string(argv[++i]);
		}
	}
	return true;
}



void ClipsBridge::printDefaultArgs(std::string const& pname){
	std::cout << "Using default parameters:" << std::endl;
	std::cout << "    "   << pname;
	std::cout << " -i "   << topicIn;
	std::cout << " -o "   << topicOut;
	std::cout << " -s "   << topicStatus;
	std::cout << " -d "   << clppath;
	std::cout << " -e "   << ( (clips_file.length() > 0) ? clips_file : "''");
	std::cout << " -w "   << flg_facts;
	std::cout << " -r "   << flg_rules;
	std::cout << std::endl << std::endl;
}



void ClipsBridge::printHelp(std::string const& pname){
	std::cout << "Usage:" << std::endl;
	std::cout << "    " << pname << " ";
	std::cout << "-i input_topic ";
	std::cout << "-o output_topic ";
	std::cout << "-s status_topic ";
	std::cout << "-d clp base path (where clips files are)";
	std::cout << "-e clips_file ";
	std::cout << "-w watch_facts ";
	std::cout << "-r watch_rules ";
	std::cout << std::endl << std::endl;
	std::cout << "Example:" << std::endl;
	std::cout << "    " << pname << " -e virbot.dat -w 1 -r 1"  << std::endl;
}

