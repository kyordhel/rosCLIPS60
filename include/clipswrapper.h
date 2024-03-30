#ifndef __CLIPSWRAPPER_H__
#define __CLIPSWRAPPER_H__
#include <cstdio>
#include <string>
#include <vector>
// #include <>
//

#define GET_MACRO(_1,_2,_3,_4,_5,NAME,...) NAME
#define defineFunction(...) GET_MACRO(__VA_ARGS__, __defineFunction5, __defineFunction4, __defineFunction3, __defineFunction2)(__VA_ARGS__)
#define __defineFunction2(p0, p1) defineFunction_impl(#p0, p1, #p0, #p0)
#define __defineFunction3(p0, p1, p2) defineFunction_impl(p0, p1, p2, #p2)
#define __defineFunction4(p0, p1, p2, p3) defineFunction_impl(p0, p1, p2, p3)
#define __defineFunction5(p0, p1, p2, p3, p4) defineFunction_impl(p0, p1, p2, p3, p4)

namespace clips{

enum class WatchItem : int{
	Facts            = 0x0001,
	Rules            = 0x0002,
	Activations      = 0x0004,
	Focus            = 0x0008,
	Compilations     = 0x0010,
	Statistics       = 0x0020,
	Globals          = 0x0040,
	Deffunctions     = 0x0080,
	Instances        = 0x0100,
	Slots            = 0x0200,
	Messages         = 0x0400,
	MessageHandlers  = 0x0800,
	GenericFunctions = 0x1000,
	Methods          = 0x2000,
	All              = 0xffff
};

enum class ArgCountRestriction : char{
	Exactly    = 0,
	AtLeast    = 1,
	NoMoreThan = 2,
};

void assertString(const std::string& s);
void printFacts(
	const std::string& logicalName="stdout",
	const std::string& module="",
	size_t start=-1, size_t end=-1, size_t max=-1);
void printRules(
	const std::string& logicalName="stdout",
	const std::string& module="");
void printAgenda(
	const std::string& logicalName="stdout",
	const std::string& module="");

std::vector<std::string> getDefruleList(std::string const& module="");



int load(std::string const& fpath);
bool sendCommand(std::string const& s);
bool watch(const WatchItem& item);
bool unwatch(const WatchItem& item);
WatchItem toggleWatch(const WatchItem& item);


bool argCountCheck(
	const std::string& functionName,
	const ArgCountRestriction& restriction,
	int count);

bool defineFunction_impl(
	const std::string& functionName,
	const char& returnType,
	int (*functionPointer)(),
	const std::string& actualFunctionName,
	const std::string& restrictions=""
);





inline constexpr WatchItem operator&(WatchItem x, WatchItem y) {
	return static_cast<WatchItem>(static_cast<int>(x) & static_cast<int>(y));
}

inline constexpr WatchItem operator|(WatchItem x, WatchItem y) {
	return static_cast<WatchItem>(static_cast<int>(x) & static_cast<int>(y));
}

inline constexpr WatchItem operator^(WatchItem x, WatchItem y) {
	return static_cast<WatchItem>(static_cast<int>(x) & static_cast<int>(y));
}



}
#endif // __CLIPSWRAPPER_H__