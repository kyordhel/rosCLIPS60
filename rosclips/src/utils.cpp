#include "utils.h"
#include <cstdio>

namespace utils{

int32_t xtractInt(std::string const& s){
	char* ptr = (char*) s.c_str();
	bool minus = false;
	while( *ptr != 0 ){
		if( (*ptr >= '0') && (*ptr <= '9') )
			break;
		minus = *ptr == '-';
		++ptr;
	}
	if(!*ptr) return 0;
	if(minus) --ptr;
	int i = 0;
	std::sscanf(ptr, "%d", &i);
	return i;
}

uint32_t xtractUint(std::string const& s){
	char* ptr = (char*) s.c_str();
	while( *ptr != 0 ){
		if( (*ptr >= '0') && (*ptr <= '9') )
			break;
		++ptr;
	}
	if(!*ptr) return 0;
	int i = 0;
	std::sscanf(ptr, "%d", &i);
	return i;
}

double xtractDouble(std::string const& s){
	char* ptr = (char*) s.c_str();
	bool minus = false;
	while( *ptr != 0 ){
		if( (*ptr >= '0') && (*ptr <= '9') )
			break;
		minus = *ptr == '-';
		++ptr;
	}
	if(!*ptr) return 0;
	if(minus) --ptr;
	double d = 0;
	std::sscanf(ptr, "%lf", &d);
	return d;
}

}