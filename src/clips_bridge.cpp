#include "clips_bridge.h"

#include <cstdio>
#include <fstream>
#include <sstream>

extern "C" {
	#include "clips/clips.h"
	// #include "user_functions.h"
}

#include "utils.h"
// #include "dummy_func.h"



/* ** ********************************************************
* Macros
* *** *******************************************************/
#define contains(s1,s2) s1.find(s2) != std::string::npos




/* ** ********************************************************
* Constructor
* *** *******************************************************/
ClipsBridge::ClipsBridge(){

	// Default values
	start = -1;
	end   = -1;
	max   = -1;

	topic_in   = "clips_in";
	topic_out  = "clips_out";
	// clips_file = "virbot.dat";
	clips_file = "cubes.dat";
	flg_facts  = false;
	flg_rules  = false;
	flg_trace  = false;
	num        = 100;
	nodeHandle = NULL;
}



/* ** ********************************************************
* Class methods
* *** *******************************************************/
void ClipsBridge::assertFact(std::string const& s) {
	std::stringstream ss;
	ss << "(network " << s << ")";
	AssertString( (char*)ss.str().c_str() );
	SetFactListChanged(0);
}


bool ClipsBridge::init(int argc, char **argv, ros::NodeHandle& nh){
	if( !parseArgs(argc, argv) ) return false;

	nodeHandle = &nh;
	ros::Publisher pub = nh.advertise<std_msgs::String>(topic_out, 100);
	publishers[topic_out] = pub;

	ros::Subscriber sub = nodeHandle->subscribe<std_msgs::String>(topic_in, 100,
		boost::bind(&ClipsBridge::subscriberCallback, this, _1, topic_in)
	);
	subscribers[topic_in] = sub;

	initCLIPS(argc, argv);
	return true;
}


void ClipsBridge::initCLIPS(int argc, char **argv){

	InitializeCLIPS();
	RerouteStdin(argc, argv);

	// Load clp files specified in dat file
	load_dat(clips_file);
	Reset();

	logicalName = "stdout";
	Facts( (char*) logicalName.c_str(), NULL, start, end, max);
}


void ClipsBridge::load_clp(const std::string& fpath){
	ROS_INFO("Loading file '%s'...", fpath.c_str() );
	int flag = Load( (char*) fpath.c_str() );
	if(flag <= 0){
		ROS_ERROR("Error in file '%s' or does not exist  %d", fpath.c_str());
		exit(0);
	}
	ROS_INFO("File %s loaded successfully", fpath.c_str());
}


void ClipsBridge::load_dat(const std::string& fpath){
	std::ifstream fs;
	fs.open(fpath);

	if( fs.fail() || !fs.is_open() ){
		ROS_ERROR("File '%s' does not exists", fpath);
		exit(0);
	}

	std::string line;
	while( std::getline(fs, line) )
		if ( !line.empty() ) load_clp(line);

	fs.close();
}


void ClipsBridge::parseMessage(std::string& m){
	if(contains(m, "CONTINUE") )
		return;
	else if(contains(m, "FACTS") ){
		Facts((char*) logicalName.c_str(), NULL, start, end, max);
		flg_facts = 1;
	}
	else if( contains(m, "RULES") )
		flg_rules = 1;
	else if( contains(m, "UNTRACE") )
		flg_trace = 0;
	else if( contains(m, "TRACE") )
		flg_trace = 1;
	else if( contains(m, "NUMBER") ){
		num = utils::xtractInt(m);
		ROS_INFO("Number %d", num);
	}
	else if( contains(m, "UNWATCH") ){
		flg_rules = 0;
		flg_facts = 0;
	}
	else if( !strcmp(m.c_str(), "RESET") ){
		Clear();
		load_dat(clips_file);
		Reset();
	}
	else{
		assertFact( m );
	}
}


bool ClipsBridge::parseArgs(int argc, char **argv){
	// Read input parameters
	if (argc <= 1) {
		printDefaultArgs(argv[0]);
		return true;
	}

	for(size_t i = 1; i < argc; ++i){
		if (!strcmp(argv[i], "-h") || (i+1 >= argc) ){
			printHelp( argv[0] );
			return false;
		}
		else if (!strcmp(argv[i],"-e")){
			clips_file = std::string(argv[i+1]);
		}
		else if (!strcmp(argv[i],"-w")){
			flg_facts = atoi(argv[i+1]);
		}
		else if (!strcmp(argv[i],"-r")){
			flg_rules = atoi(argv[i+1]);
		}
		else if (!strcmp(argv[i],"-t")){
			flg_trace = atoi(argv[i+1]);
		}
		else if (!strcmp(argv[i],"-n")){
			num = atoi(argv[i+1]);
		}
		else if (!strcmp(argv[i],"-i")){
			topic_in = std::string(argv[i+1]);
		}
		else if (!strcmp(argv[i],"-o")){
			topic_out = std::string(argv[i+1]);
		}
	}
	return true;
}



void ClipsBridge::printDefaultArgs(std::string const& pname){
	std::cout << "Using default parameters:" << std::endl;
	std::cout << "    "   << pname;
	std::cout << " -i "   << topic_in;
	std::cout << " -o "   << topic_out;
	std::cout << " -e "   << clips_file;
	std::cout << " -w "   << flg_facts;
	std::cout << " -r "   << flg_rules;
	std::cout << " -num " << num;
	std::cout << " -t "   << flg_trace;
	std::cout << std::endl << std::endl;
}



void ClipsBridge::printHelp(std::string const& pname){
	std::cout << "Usage:" << std::endl;
	std::cout << "    ./" << pname;
	std::cout << "-i input_topic ";
	std::cout << "-o output_topic ";
	std::cout << "-e clips_file ";
	std::cout << "-w watch_facts ";
	std::cout << "-r watch_rules ";
	std::cout << "-num num_rules ";
	std::cout << "-t trace";
	std::cout << std::endl << std::endl;
	std::cout << "Example:" << std::endl;
	std::cout << "    ./" << pname << " -e virbot.dat -w 1 -r 1 -num 20"  << std::endl;
}


void ClipsBridge::subscriberCallback(std_msgs::String::ConstPtr const& msg, std::string const& topic) {
	ROS_INFO("CLIPS in: [%s] via %s", msg->data.c_str(), topic.c_str());
	if (topic == topic_in)
		queue.produce(msg->data);
	else if (topic_facts.find("f") != topic_facts.end()){
		std::string fact = topic_facts[topic];
		std::stringstream ss;
		ss << "(" << fact << " " << msg->data.c_str() << ")";
		AssertString( (char*)ss.str().c_str() );
	}
}


void ClipsBridge::run(){
	// Loop forever
	while(ros::ok()){
		SetWatchItem("facts", flg_facts, NULL);
		SetWatchItem("rules", flg_rules, NULL);

		Run(num);

		// // it checks if function waitsec finished
		// if(flag_time == 1){
		// 	AssertString(buffer_time);
		// 	SetFactListChanged(0);
		// 	flag_time = 0;
		// }

		if (flg_trace == 1){
			while( queue.empty() )
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}

		if( queue.empty() ){
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
			continue;
		}
		SetWatchItem("facts", 1, NULL);

		// For each message in queue
		while( !queue.empty() ){
			parseMessage( queue.consume() );
		}
	}
}


void ClipsBridge::runAsync(){
	asyncThread = std::thread(&ClipsBridge::run, this);
}

int ClipsBridge::publish(std::string const& message){
	return publish(topic_out, message);
}

int ClipsBridge::publish(std::string const& topic_name, std::string const& message){
	if (publishers.find(topic_name) == publishers.end()){
		// Topic not in publishers. Insert.
		ros::Publisher pub = nodeHandle->advertise<std_msgs::String>( std::string(topic_name), 10);
		publishers[topic_name] = pub;
		ROS_INFO("Added publisher for topic /%s", topic_name.c_str());
	}
	ros::Publisher& pub = publishers[topic_name];
	std_msgs::String msg;
	msg.data = message;
	// ROS_INFO("Published <%s> on /%s", message.c_str(), pub.getTopic().c_str());
	pub.publish(msg);
	ros::spinOnce();
	return 0;
}


int ClipsBridge::subscribe(std::string const& topic_name, std::string const& fact_name){
	if (subscribers.find(topic_name) == subscribers.end()){
		// Topic not in subscribers. Insert.
		// ros::Subscriber sub = nodeHandle->subscribe(topic_name, 10, &ClipsBridge::subscriberCallback, this);
		ros::Subscriber sub = nodeHandle->subscribe<std_msgs::String>(topic_in, 100,
			boost::bind(&ClipsBridge::subscriberCallback, this, _1, topic_name)
		);
		ROS_INFO("Subscribed to topic /%s", topic_name.c_str());
		subscribers[topic_name] = sub;
		topic_facts[topic_name] = fact_name;
	}
	return 0;
}


