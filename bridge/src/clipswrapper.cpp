/* ** ***************************************************************
* clipswrapper.cpp
*
* Author: Mauricio Matamoros
*
* Wrapper for most used CLIPS functions required/used by the bridge
*
** ** **************************************************************/

#include <map>
#include <stack>
#include "clipswrapper.h"


extern "C" {
	#include "clips/clips.h"
	#include "clips/commline.h"
	#include "clips/prcdrfun.h"
}


/* ** ***************************************************************
*
* Helpers
*
** ** **************************************************************/
static inline
char* clipsstr(const std::string& s){
	return s.length() ? (char*)s.c_str() : NULL;
}


namespace clips{
std::map<WatchItem,std::string> watchItems = {
	{WatchItem::Facts,            "facts"},
	{WatchItem::Rules,            "rules"},
	{WatchItem::Activations,      "activations"},
	{WatchItem::Focus,            "focus"},
	{WatchItem::Compilations,     "compilations"},
	{WatchItem::Statistics,       "statistics"},
	{WatchItem::Globals,          "globals"},
	{WatchItem::Deffunctions,     "deffunctions"},
	{WatchItem::Instances,        "instances"},
	{WatchItem::Slots,            "slots"},
	{WatchItem::Messages,         "messages"},
	{WatchItem::MessageHandlers,  "message-handlers"},
	{WatchItem::GenericFunctions, "generic-functions"},
	{WatchItem::Methods,          "methods"},
};


bool isValidClipsString(std::string const& str){
	if((str.length() < 1) || (str[0] != '('))
		return false;

	size_t i;
	std::stack<char> stack;
	for(i = 0; i < str.length(); ++i){
		if(str[i] == '\\'){ ++i; continue; }
		if(str[i] == '('){ stack.push(str[i]); continue; }
		if(str[i] == ')'){
			if(stack.empty()) return false;
			stack.pop();
		}
	}
	return stack.empty();
}


int run(int maxRules){
	return Run(maxRules);
}

void initialize(){
	InitializeCLIPS();
}

void rerouteStdin(int argc, char** argv){
	RerouteStdin(argc, argv);
}

void clear(){
	Clear();
}

void reset(){
	Reset();
}

bool getFactListChanged(){
	return getFactListChanged();
}

void setFactListChanged(const bool changed){
	SetFactListChanged(changed);
}

int returnArgCount(){
	return RtnArgCount();
}

double returnDouble(const int& argPos){
	return RtnDouble(argPos);
}

int returnInt(const int& argPos){
	return RtnLong(argPos);
}

std::string returnLexeme(const int& argPos){
	return RtnLexeme(argPos);
}

long returnLong(const int& argPos){
	return RtnLong(argPos);
}


void assertString(const std::string& s){
	AssertString( clipsstr(s) );
}


void printAgenda(
	const std::string& logicalName,
	const std::string& module){
	Agenda( clipsstr(logicalName), clipsstr(module) );
}


void printFacts(
	const std::string& logicalName,
	const std::string& module,
	size_t start, size_t end, size_t max){
	Facts( clipsstr(logicalName), clipsstr(module), start, end, max);
}


void printDefrules(
	const std::string& logicalName,
	const std::string& module){
	ListDefrules( clipsstr(logicalName), clipsstr(module) );
}


bool print(const std::string& ln, const std::string& str){
	return PrintCLIPS( clipsstr(ln), clipsstr(str) );
}


std::vector<std::string> getDefruleList(std::string const& module){
	DATA_OBJECT obj;
	struct multifield* theList;
	std::vector<std::string> rules;

	char* ptrModule = (module.length() > 0) ? (char*) module.c_str() : NULL;

	GetDefruleList(&obj, ptrModule);
	if( !(theList = (struct multifield *)obj.value) )
		return rules;

	for(int i = 1; theList && i <= obj.end; ++i) {
		if(GetMFType(theList, i) != SYMBOL) continue;
		SYMBOL_HN* value = (SYMBOL_HN*)GetMFValue(theList, i);
		rules.push_back((char*)(value->contents));
	}
	return rules;
}


bool load(std::string const& fpath){
	return Load( clipsstr(fpath) ) > 0;
}


void sendCommandRaw(std::string const& s, bool verbose){
	// Resets the pretty print save buffer.
	FlushPPBuffer();
	// Sets PPBufferStatus flag to boolean
	// value of ON or OFF
	SetPPBufferStatus(OFF);
	// Processes a completed command
	// RouteCommand(as, command, verbose); // CLIPS 6.24
	RouteCommand( clipsstr(s) );
	// Returns the EvaluationError flag
	int res = GetEvaluationError();
	// Resets the pretty print save buffer.
	FlushPPBuffer();
	// Sets the HaltExecution flag
	SetHaltExecution(FALSE);
	// Sets the EvaluationError flag
	SetEvaluationError(FALSE);
	// Removes all variables from the list
	// of currently bound local variables.
	FlushBindList();
}

bool sendCommand(std::string const& s, bool verbose){
	if(!isValidClipsString(s)) return false;
	sendCommandRaw(s, verbose);
	return true;
}


bool watch(const WatchItem& item){
	bool result = true;
	if((int)(item & WatchItem::All))
		return Watch((char*)"all");
	if((int)(item & WatchItem::Facts))            result&= Watch((char*)"facts");
	if((int)(item & WatchItem::Rules))            result&= Watch((char*)"rules");
	if((int)(item & WatchItem::Activations))      result&= Watch((char*)"activations");
	if((int)(item & WatchItem::Focus))            result&= Watch((char*)"focus");
	if((int)(item & WatchItem::Compilations))     result&= Watch((char*)"compilations");
	if((int)(item & WatchItem::Statistics))       result&= Watch((char*)"statistics");
	if((int)(item & WatchItem::Globals))          result&= Watch((char*)"globals");
	if((int)(item & WatchItem::Deffunctions))     result&= Watch((char*)"deffunctions");
	if((int)(item & WatchItem::Instances))        result&= Watch((char*)"instances");
	if((int)(item & WatchItem::Slots))            result&= Watch((char*)"slots");
	if((int)(item & WatchItem::Messages))         result&= Watch((char*)"messages");
	if((int)(item & WatchItem::MessageHandlers))  result&= Watch((char*)"message-handlers");
	if((int)(item & WatchItem::GenericFunctions)) result&= Watch((char*)"generic-functions");
	if((int)(item & WatchItem::Methods))          result&= Watch((char*)"methods");
	return result;
}


bool unwatch(const WatchItem& item){
	bool result = true;
	if((int)(item & WatchItem::All))
		return Unwatch((char*)"all");
	if((int)(item & WatchItem::Facts))            result&= Unwatch((char*)"facts");
	if((int)(item & WatchItem::Rules))            result&= Unwatch((char*)"rules");
	if((int)(item & WatchItem::Activations))      result&= Unwatch((char*)"activations");
	if((int)(item & WatchItem::Focus))            result&= Unwatch((char*)"focus");
	if((int)(item & WatchItem::Compilations))     result&= Unwatch((char*)"compilations");
	if((int)(item & WatchItem::Statistics))       result&= Unwatch((char*)"statistics");
	if((int)(item & WatchItem::Globals))          result&= Unwatch((char*)"globals");
	if((int)(item & WatchItem::Deffunctions))     result&= Unwatch((char*)"deffunctions");
	if((int)(item & WatchItem::Instances))        result&= Unwatch((char*)"instances");
	if((int)(item & WatchItem::Slots))            result&= Unwatch((char*)"slots");
	if((int)(item & WatchItem::Messages))         result&= Unwatch((char*)"messages");
	if((int)(item & WatchItem::MessageHandlers))  result&= Unwatch((char*)"message-handlers");
	if((int)(item & WatchItem::GenericFunctions)) result&= Unwatch((char*)"generic-functions");
	if((int)(item & WatchItem::Methods))          result&= Unwatch((char*)"methods");
	return result;
}


WatchItem toggleWatch(const WatchItem& item){
	int result = 0;
	for(const auto& kv : watchItems){
		if(!(int)(item & kv.first)) continue;
		char* iname = (char*)kv.second.c_str();
		switch( GetWatchItem(iname) ){
			case  0: Watch(iname); break;
			case  1: Unwatch(iname); break;
			default: continue;
		}
		if( GetWatchItem(iname) == 1 )
			result|= (int)kv.first;
	}
	return (WatchItem)result;
}


WatchItem getWatches(){
	int result = 0;
	for(const auto& kv : watchItems){
		char* iname = (char*)kv.second.c_str();
		if( GetWatchItem(iname) == 1 )
			result|= (int)kv.first;
	}
	return (WatchItem)result;
}

inline
bool watching(const WatchItem& item){
	return (int)getWatches() & (int)item;
}


bool argCountCheck(const std::string& functionName, const ArgCountRestriction& restriction, int count){
	bool result = ArgCountCheck(clipsstr(functionName), (int)restriction, count);
	return result;
}

bool defineFunction_impl(const std::string& functionName,
	const char& returnType,
	int (*functionPointer)(),
	const std::string& actualFunctionName,
	const std::string& restrictions
){
	if(restrictions.length())
		return DefineFunction2(clipsstr(functionName), returnType, functionPointer,
			clipsstr(actualFunctionName), clipsstr(restrictions));
	return DefineFunction(clipsstr(functionName), returnType, functionPointer, clipsstr(actualFunctionName));
}

} // end namespace
