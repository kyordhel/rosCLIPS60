#include "clipswrapperrouter.h"


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

int addRouter(const std::string& routerName, RouterPriority priority,
	int (*queryFunction)(char*),
	int (*printFunction)(char*,char*),
	int (*getcFunction)(char*),
	int (*ungetcFunction)(int,char*),
	int (*exitFunction)(int)
){
	return AddRouter( clipsstr(routerName), (int)priority,
		queryFunction,
		printFunction,
		getcFunction,
		ungetcFunction,
		exitFunction
	);
}

int activateRouter(const std::string& routerName){
	return ActivateRouter( clipsstr(routerName) );
}

int deactivateRouter(const std::string& routerName){
	return DeactivateRouter( clipsstr(routerName) );
}

int deleteRouter(const std::string& routerName){
	return DeleteRouter( clipsstr(routerName) );
}

} // end namespace