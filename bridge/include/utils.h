/* ** ***************************************************************
* utils.h
*
* Author: Mauricio Matamoros
*
* Utility/miscellaneous/helper functions for the bridge
*
** ** **************************************************************/
/** @file utils.h
 * Utility functions
 */

#ifndef __UTILS_H__
#define __UTILS_H__
#pragma once

/** @cond */
#include <string>
/** @endcond */

namespace utils{

/**
 * Retrieves the first integer found in the given string if any.
 * If no integer is found returns zero.
 * @param  s The string containing an integer
 * @return   The first int32 found in the given string, zero if none.
 */
int32_t xtractInt(std::string const& s);

/**
 * Retrieves the first unsigned integer found in the given string,
 * if any. If no integer is found returns zero.
 * @param  s The string containing an unsigned integer
 * @return   The first uint32 found in the given string, zero if none.
 */
uint32_t xtractUint(std::string const& s);

/**
 * Retrieves the first decimal number found in the given string,
 * if any as a double-precision float. If no integer is found returns
 * zero.
 *
 * @param  s The string containing a decimal number
 * @return   The first number found in the given string, zero if none.
 */
double xtractDouble(std::string const& s);

} // end namespace

#endif // __UTILS_H__