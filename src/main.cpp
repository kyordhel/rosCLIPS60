   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                     MAIN MODULE                     */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*	Jesus Savage					     */
/*	Mauricio Matamoros				     */
/*                                                           */
/*             FI-UNAM  Version 6.00  05/1/2008              */
/*             FI-UNAM  Version 6.00  04/1/2013              */
/*             FI-UNAM  Version 6.00  04/18/2023             */
/*                                                           */
/*************************************************************/

#include <cstdio>
#include <fstream>
#include <thread>
#include <string>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

extern "C" {
	#include "clips/clips.h"
	// #include "user_functions.h"
}

#include "utils.h"
#include "sync_queue.h"
#include "dummy_func.h"

#define contains(s1,s2) s1.find(s2) != std::string::npos

/* ** ********************************************************
* Classes
* *** *******************************************************/
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
	ros::Publisher pub;
	ros::Subscriber sub;

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
	void subscriberCallback(std_msgs::String::ConstPtr const& msg);

private:
	void assertFact(std::string const& s);
	void initCLIPS(int argc, char **argv);
	void parseMessage(std::string& m);
	bool parseArgs(int argc, char **argv);
	void printDefaultArgs(std::string const& pname);
	void printHelp(std::string const& pname);

	friend void send_message(ClipsBridge& br, std::string const& msg);
};


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
}


void ClipsBridge::assertFact(std::string const& s) {
	std::stringstream ss;
	ss << "(network " << s << ")";
	AssertString( (char*)ss.str().c_str() );
	SetFactListChanged(0);
}

bool ClipsBridge::init(int argc, char **argv, ros::NodeHandle& nh){
	if( !parseArgs(argc, argv) ) return false;

	pub = nh.advertise<std_msgs::String>(topic_out, 1000);
	sub = nh.subscribe(topic_in, 1000, &ClipsBridge::subscriberCallback, this);

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
	std::cout << "-i input topic";
	std::cout << "-o output topic";
	std::cout << "-e clips file";
	std::cout << "-w watch facts";
	std::cout << "-r watch rules";
	std::cout << "-num num rules";
	std::cout << "-t trace";
	std::cout << std::endl << std::endl;
	std::cout << "Example:" << std::endl;
	std::cout << "    ./" << pname << " -e virbot.dat -w 1 -r 1 -num 20"  << std::endl;
}

void ClipsBridge::subscriberCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("CLIPS in: [%s]", msg->data.c_str());
	queue.produce(msg->data);
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
			while( queue.empty() ) usleep(20);
		}

		if( queue.empty() ) continue;
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


/* ** ********************************************************
* Global variables
* *** *******************************************************/
ClipsBridge bridge;

/* ** ********************************************************
* Prototypes
* *** *******************************************************/
int main(int argc, char **argv);
void send_message(ClipsBridge& br, std::string const& msg);

extern "C" {
	VOID UserFunctions();
	int CONDOR_send_data_client_network();
}

int main(int argc, char **argv){

	ros::init(argc, argv, "talker");
	ros::NodeHandle n;

	if( !bridge.init(argc, argv, n) )
		return -1;
	ros::Rate loop_rate(10);

	bridge.runAsync();
	// Give control to ROS
	ros::spin();
	// while (ros::ok()) {
	// 	std_msgs::String msg;

	// 	std::stringstream ss;
	// 	ss << "hello world ";
	// 	msg.data = ss.str();

	// 	ROS_INFO("%s", msg.data.c_str());

	// 	chatter_pub.publish(msg);

	// 	ros::spinOnce();

	// 	loop_rate.sleep();
	// }
	return 0;
}

VOID UserFunctions(){
	DefineFunction("CONDOR_open_network_conection", 'i',
		CONDOR_open_network_conection, "CONDOR_open_network_conection");
	DefineFunction("CONDOR_send_data_client_network", 'i',
		CONDOR_send_data_client_network, "CONDOR_send_data_client_network");
}

int CONDOR_send_data_client_network(){
	/* check for exactly two arguments */
	if(ArgCountCheck("CONDOR_send_data_client_network", EXACTLY, 2) == -1)
		return -1;

	/* get the values for the 1st, 2rd arguments */
	int c = RtnDouble(1);
	char* message = RtnLexeme(2);

	/* It sends the data */
	send_message(bridge, message);
	// send_data_client_network(Connections[c], message);
	return 0;
}

void send_message(ClipsBridge& br, std::string const& s){
	std_msgs::String msg;
	msg.data = s;
	br.pub.publish(msg);
	ros::spinOnce();
}

