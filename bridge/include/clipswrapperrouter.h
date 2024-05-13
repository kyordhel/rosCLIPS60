#ifndef __CLIPSWRAPPERROUTER_H__
#define __CLIPSWRAPPERROUTER_H__
#pragma once

#include <cstdio>
#include <string>


namespace clips{

/* ** ***************************************************************
*
* Enumerations
*
** ** **************************************************************/

enum class LogicalName {
	/**
	 * The default for all user inputs. The read and readline
	 * functions read from stdin if t is specified as the logical name.
	 */
	stdin    = 0x01,
	/**
	 * The default for all user outputs. The format and printout
	 * functions send output to stdout if t is specified as the logical name.
	 */
	stdout   = 0x02,
	/**
	 * The CLIPS prompt is sent to this logical name.
	 */
	wclips   = 0x04,
	/**
	 * All informational messages are sent to this logical name.
	 */
	wdialog  = 0x08,
	/**
	 * Requests to display CLIPS information, such as facts or
	 * rules, are sent to this logical name.
	 */
	wdisplay = 0x10,
	/**
	 * All error messages are sent to this logical name.
	 */
	werror   = 0x20,
	/**
	 * All warning messages are sent to this logical name.
	 */
	wwarning = 0x40,
	/**
	 * All watch information is sent to this logical name (with the
	 * exception of compilations which is sent to wdialog).
	 */
	wtrace   = 0x80

};

enum class RouterPriority : int{
	UniqueNonShared   = 50,
	StandardShared    = 40,
	UniqueShared      = 30,
	StandardNonShared = 20,
	DefaultUserIO     = 10,
	Reserved          =  0
};

int addRouter(const std::string& routerName, RouterPriority priority,
	int (*queryFunction)(char*),
	int (*printFunction)(char*,char*),
	int (*getcFunction)(char*),
	int (*ungetcFunction)(int,char*),
	int (*exitFunction)(int)
);

int activateRouter(const std::string& routerName);
int deactivateRouter(const std::string& routerName);
int deleteRouter(const std::string& routerName);


inline constexpr LogicalName operator&(LogicalName x, LogicalName y) {
	return static_cast<LogicalName>(static_cast<int>(x) & static_cast<int>(y));
}

inline constexpr LogicalName operator|(LogicalName x, LogicalName y) {
	return static_cast<LogicalName>(static_cast<int>(x) | static_cast<int>(y));
}

inline constexpr LogicalName operator^(LogicalName x, LogicalName y) {
	return static_cast<LogicalName>(static_cast<int>(x) ^ static_cast<int>(y));
}

}
#endif // __CLIPSWRAPPER_H__