#include <cstring>
#include "queryrouter.h"

namespace clips{
/* ** ***************************************************************
*
* Static prototypes for interface with CLIPS
*
** ** **************************************************************/
extern "C"{
	static int queryFunction(char* logicalName);
	static int printFunction(char *logicalName, char *str);
	static int exitFunction(int exitCode);
}


/* ** ***************************************************************
*
* QueryRouter class members
*
** ** **************************************************************/

QueryRouter& QueryRouter::getInstance(
			const std::string& routerName,
			clips::RouterPriority priority)
{
	// Guaranteed to be destroyed.
	// Instantiated on first use.
	static QueryRouter instance(routerName, priority);
	return instance;
}


QueryRouter::QueryRouter(const std::string& routerName, clips::RouterPriority priority):
	routerName(routerName), priority(priority),
	registered(false), enabled(false){}

QueryRouter::~QueryRouter(){
	unregisterR();
}


void QueryRouter::enable(){
	if(enabled) return;
	if(!registered) registerR();
	enabled = clips::activateRouter(routerName);
}


void QueryRouter::disable(){
	if(!enabled) return;
	clips::deactivateRouter(routerName);
	enabled = false;
}


bool QueryRouter::isEnabled(){
	return enabled;
}


bool QueryRouter::hasLogicalName(const std::string& ln){
	return logicalNames.count(ln) > 0;
}

void QueryRouter::addLogicalName(const std::string& ln){
	if( !hasLogicalName(ln) )
		logicalNames.insert(ln);
}

void QueryRouter::removeLogicalName(const std::string& ln){
	if( hasLogicalName(ln) )
		logicalNames.erase(ln);
}


std::string QueryRouter::getName(){
	return routerName;
}


clips::RouterPriority QueryRouter::getPriority(){
	return priority;
}


std::string QueryRouter::read(){
	std::string copy(buffer);
	buffer.clear();
	return copy;
}


void QueryRouter::write(const std::string& s){
	buffer+=s;
}


void QueryRouter::registerR(){
	if(registered) return;

	clips::addRouter(routerName,
		priority,       // Priority
		queryFunction,  // Query function
		printFunction,  // Print function
		NULL,           // Getc function
		NULL,           // Ungetc function
		exitFunction    // Exit function
	);
}


void QueryRouter::unregisterR(){
	if(!registered) return;
	clips::deactivateRouter(routerName);
	clips::deleteRouter(routerName);
}



/* ** ***************************************************************
*
* Static function definitions
*
** ** **************************************************************/

/*
We want to recognize any output that is sent to the logical name
"wtrace" because all tracing information is sent to this logical
name. The recognizer function for our router is defined below.
*/
int queryFunction(char* logicalName){
	QueryRouter& qr = QueryRouter::getInstance();
	return qr.hasLogicalName(logicalName);
}

/*
We now need to define a function which will print the tracing in-
formation to our trace file. The print function for our router is
defined below.
*/
int printFunction(char *logicalName, char *str){
	static std::set<std::string> clpln = { "stdin", "stdout", "wclips", "wdialog", "wdisplay", "werror", "wwarning", "wtrace" };

	QueryRouter& qr = QueryRouter::getInstance();
	if(!qr.hasLogicalName(logicalName)) return 0;
	if(!qr.isEnabled()){
		if(clpln.count(logicalName) > 0)
			clips::print(logicalName, str);
		return true;
	}

	qr.write(str);
	if(clpln.count(logicalName) > 0){
		clips::deactivateRouter(qr.getName());
		clips::print(logicalName, str);
		clips::activateRouter(qr.getName());
	}
	return true;
}

/*
When we exit CLIPS the trace file needs to be closed.
function for our router is defined below.
*/
int exitFunction(int exitCode){
	return 1;
}

} // end namespace clips
