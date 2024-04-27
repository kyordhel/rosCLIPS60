#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>

namespace utils{

	int32_t xtractInt(std::string const& s);
	uint32_t xtractUint(std::string const& s);
	double xtractDouble(std::string const& s);

} // end namespace

#endif // __UTILS_H__