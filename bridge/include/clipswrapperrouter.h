/* ** ***************************************************************
* clipswrapperrouter.h
*
* Author: Mauricio Matamoros
*
* Wrapper for CLIPS' router-functions required/used by the bridge
*
** ** **************************************************************/

#ifndef __CLIPSWRAPPERROUTER_H__
#define __CLIPSWRAPPERROUTER_H__
#pragma once

/** @cond */
#include <cstdio>
#include <string>
/** @endcond */


/** @file clipswrapperrouter.h
 * CLIPS C++ wrapping Library file.  You should NOT @c \#include this file
 * in your programs as it is called by @c clipswrapper.h.
 *
 * All CLIPS functions required/used by the bridge are exposed here using the
 * camelCase naming convention and are all contained in the namespace @c clips
 * (except for names which are defined as macros in C).
 */

namespace clips{

/* ** ***************************************************************
*
* Enumerations
*
** ** **************************************************************/

/**
 * Enumerates all CLIPS logical names as OR-able flags
 */
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

/**
 * Enumerates all CLIPS' router priorities, including their values
 */
enum class RouterPriority : int{
	/**
	 * Any router that uses "unique" logical names and does not
	 * want to share I/O with catch-all routers.
	 */
	UniqueNonShared   = 50,

	/**
	 * Any router that wants to grab standard I/O and is willing to
	 * share it with other routers.
	 * @remark A dribble file is a good example of this type of
	 * router. The dribble file router needs to grab all output
	 * that normally would go to the terminal so it can be
	 * placed in the dribble file, but this same output also needs to
	 * be sent to the router which displays output on the terminal.
	 */
	StandardShared    = 40,

	/**
	 * Any router that uses "unique" logical names and is willing to
	 * share I/O with catch-all routers.
	 */
	UniqueShared      = 30,

	/**
	 * Any router that wants to grab standard logical names and is
	 * not willing to share them with other routers.
	 */
	StandardNonShared = 20,

	/**
	 * This priority is used by a router which redefines the default
	 * user interface I/O router. Only one router should use this
	 * priority.
	 */
	DefaultUserIO     = 10,

	/**
	 * This priority is used by the default router for handling stan-
	 * dard and file logical names. Other routers should not use this
	 * priority.
	 */
	Reserved          =  0
};


/**
 * Adds a new I/O router to the list of I/O routers.
 * @param routerName     The name of the I/O router. This name is used
 *                       to reference the router by the other I/O
 *                       router handling functions.
 * @param priority       The priority of the I/O router. I/O routers are
 *                       queried in descending order of priorities.
 * @param queryFunction  A pointer to the int(char*) query function
 *                       associated with this router. This query
 *                       function should accept a single argument,
 *                       a (char*) logical name, and return either
 *                       TRUE (1) or FALSE (0) depending upon whether
 *                       the router recognizes the logical name.
 * @param printFunction  A pointer to the int(char*, char*)print function
 *                       associated with this router. This print function
 *                       should accept two arguments: a logical name and a
 *                       character string. The return value of the print
 *                       function is not meaningful.
 * @param getcFunction   A pointer to the int(char*) get character
 *                       function associated with this router. The get
 *                       character function should accept a single
 *                       argument, a logical name. The return value of
 *                       the get character function should be an
 *                       integer which represents the character or end
 *                       of file (EOF) read from the source represented
 *                       by logical name.
 * @param ungetcFunction A pointer to the int(int,char*)unget character
 *                       function associated with this router. The
 *                       ungetc character function accepts two arguments:
 *                       a logical name and a character. The return value
 *                       of the unget character function should be an
 *                       integer which represents the character which was
 *                       passed to it as an argument if the ungetc is
 *                       successful or end of file (EOF) is the ungetc is
 *                       not successful.
 * @param exitFunction   A pointer to the exit function associated with
 *                       this router. The exit function should accept a
 *                       single argument: the exit code represented by num
 * @return               Returns zero if the router could not be added,
 *                       otherwise a non-zero value is returned.
 */
int addRouter(const std::string& routerName, RouterPriority priority,
	int (*queryFunction)(char*),
	int (*printFunction)(char*,char*),
	int (*getcFunction)(char*),
	int (*ungetcFunction)(int,char*),
	int (*exitFunction)(int)
);


/**
 * Activates an existing I/O router. This router will be queried to
 * see if it can handle an I/O request. Newly created routers do not
 * have to be activated.
 * @param  routerName The name of the I/O router to be activated.
 * @return            Returns a non-zero value if the logical name is
 *                    recognized, otherwise it returns zero.
 */
int activateRouter(const std::string& routerName);


/**
 * The function DeactivateRouter deactivates an existing I/O
 * router. This router will not be queried to see if it can handle
 * an I/O request.
 * @param  routerName The name of the I/O router to be deactivated.
 * @return            Returns a non-zero value if the logical name is
 *                    recognized, otherwise it returns zero.
 */
int deactivateRouter(const std::string& routerName);

/**
 * The function DeleteRouter removes an existing I/O router from
 * the list of I/O routers.
 * @param  routerName The name of the I/O router to be deleted.
 * @return            Returns a non-zero value if the logical name is recognized, otherwise it returns zero.
 */
int deleteRouter(const std::string& routerName);

/**
 * Returns a bitwise AND operation over the values of two LogicalName flags
 */
inline constexpr LogicalName operator&(LogicalName x, LogicalName y) {
	return static_cast<LogicalName>(static_cast<int>(x) & static_cast<int>(y));
}

/**
 * Returns a bitwise AND operation over the values of two LogicalName flags
 */
inline constexpr LogicalName operator|(LogicalName x, LogicalName y) {
	return static_cast<LogicalName>(static_cast<int>(x) | static_cast<int>(y));
}

/**
 * Returns a bitwise AND operation over the values of two LogicalName flags
 */
inline constexpr LogicalName operator^(LogicalName x, LogicalName y) {
	return static_cast<LogicalName>(static_cast<int>(x) ^ static_cast<int>(y));
}

}
#endif // __CLIPSWRAPPER_H__