/* ** ***************************************************************
* clipswrapper.h
*
* Author: Mauricio Matamoros
*
* Wrapper for most used CLIPS functions required/used by the bridge
*
** ** **************************************************************/
/** @file clipswrapper.h
 * CLIPS C++ wrapping Library file.  You should @c \#include this file
 * in your programs, rather than any of the CLIPS' @a *.h implementation files.
 *
 * All CLIPS functions required/used by the bridge are exposed here using the
 * camelCase naming convention and are all contained in the namespace @c clips
 * (except for names which are defined as macros in C).
 */

#ifndef __CLIPSWRAPPER_H__
#define __CLIPSWRAPPER_H__
#pragma once

/** @cond */
#include <cstdio>
#include <string>
#include <vector>

#include "clipswrapperrouter.h"
/** @endcond */

#include "queryrouter.h"



/** @cond */
#define GET_MACRO(_1,_2,_3,_4,_5,NAME,...) NAME
/** @endcond */

/**
 * Describes and registers an external function to CLIPS so it can be
 * accessed by CLIPS programs.
 * @remark           This macro generates a set of overloads that
 *                   simplify coding by removing redundant parameters
 *                   such as typing the name of a function twice.
 *                   It wraps over DefineFunction and DefineFunction2
 * @param  CLIPSfunc Optional. The CLIPS function name: a string that
 *                   will be used to call the function from within CLIPS.
 *                   When ommitted the macro uses the actual name of the
 *                   C function taken from funcPtr.
 * @param  retType   A string containing the type of the value which
 *                   will be returned to CLIPS. It must be any of:
 *                        a:  External Address;
 *                        b:  Boolean;
 *                        c:  Character;
 *                        d:  Double Precision Float;
 *                        f:  Single Precision Float;
 *                        i:  Integer;
 *                        j:  Unknown Data Type (Symbol, String, or
 *                            Instance Name Expected);
 *                        k:  Unknown Data Type (Symbol or String
 *                            Expected);
 *                        l:  Long Integer;
 *                        m:  Multifield;
 *                        n:  Unknown Data Type (Integer or Float
 *                            Expected);
 *                        o:  Instance Name;
 *                        s:  String;
 *                        u:  Unknown Data Type (Any Type Expected);
 *                        v:  Void—No Return Value;
 *                        w:  Symbol;
 *                        x:  Instance Address
 *
 * @param  funcPtr   A pointer to the actual function in C.
 *                   It should be declared as @c extern C{}
 * @param  funcName  Optional. A string representation of funcPtr.
 *                   Both funcPtr and funcName must be identical.
 *                   When ommitted the macro uses the actual name of the
 *                   C function taken from funcPtr.
 * @param  restrict  A restriction string to be parsed by CLIPS
 *                   DefineFunction2 functionRestriction argument.
 *                   It must have the following format:
 *                   \<min-args> \<max-args> [\<default-type> \<types>*]
 *                   Types are as follow
 *                       a: External Address;
 *                       d: Float;
 *                       e: Instance Address, Instance Name, or Symbol;
 *                       f: Float;
 *                       g: Integer, Float, or Symbol;
 *                       h: Instance Address, Instance Name, Fact
 *                          Address, Integer, or Symbol;
 *                       i: Integer;
 *                       j: Symbol, String, or Instance Name;
 *                       k: Symbol or String;
 *                       l: Integer;
 *                       m: Multifield;
 *                       n: Integer or Float;
 *                       o: Instance Name;
 *                       p: Instance Name or Symbol;
 *                       q: Symbol, String, or Multifield;
 *                       s: String;
 *                       u: Any Data Type;
 *                       w: Symbol;
 *                       x: Instance Address;
 *                       y: Fact Address;
 *                       z: Fact address, Integer, or Symbol;
 *
 * @return           true if the function was successfully registered.
 *                   false otherwise.
 */
#define defineFunction(...) GET_MACRO(__VA_ARGS__, __defineFunction5, __defineFunction4, __defineFunction3, __defineFunction2)(__VA_ARGS__)
/** @cond */
#define __defineFunction2(p0, p1) defineFunction_impl(#p0, p1, #p0, #p0)
#define __defineFunction3(p0, p1, p2) defineFunction_impl(p0, p1, p2, #p2)
#define __defineFunction4(p0, p1, p2, p3) defineFunction_impl(p0, p1, p2, p3)
#define __defineFunction5(p0, p1, p2, p3, p4) defineFunction_impl(p0, p1, p2, p3, p4)
/** @endcond */

namespace clips{


/* ** ***************************************************************
*
* Enumerations
*
** ** **************************************************************/

/**
 * Enumerates all CLIPS watchable item types as OR-able flags
 */
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
	All              = 0x3fff
};


/**
 * Enumerates CLIPS argument count restriction types for
 * user-defined functions
 */
enum class ArgCountRestriction : char{
	Exactly    = 0,
	AtLeast    = 1,
	NoMoreThan = 2,
};


/* ** ***************************************************************
*
* Basic interface functions
*
** ** **************************************************************/


/**
 * Initializes the CLIPS system. Must be called prior to any other
 * CLIPS function call. This function should be called only once.
 *
 * @remark Wrapper for InitializeCLIPS()
 */
void initialize();

/**
 * Resets the CLIPS environment
 * It is the C equivalent of the CLIPS reset command.
 *
 * @remark Wrapper for Reset()
 */
void reset();

/**
 * Loads a set of constructs into the CLIPS data base.
 * It is the C equivalent of the CLIPS load command.
 * @remark       Wrapper for Load
 * @param  fpath A string representing the name of the file.
 * @return       true if the file was successfully opened.
 *               false otherwise
 */
bool load(std::string const& fpath);

/**
 * Allows rules to execute
 * It is the C equivalent of the CLIPS run command.
 * @param  maxRules Optional. An integer indicating how many rules
 *                  should fire before returning. A negative value
 *                  will fire rules until the agenda is empty.
 *                  Default: -1
 * @return          Returns the number of rules that were fired.
 */
int run(int maxRules = -1);

/**
 * Prints the list of all facts currently in the fact-list.
 * It is the C equivalent of the CLIPS facts command.
 * @remark            Wrapper for Facts
 * @param logicalName Optional. The logical name to which the listing
 *                    output is sent. Default is "stdout".
 * @param module      Optional. The name of the module containing the
 *                    facts to be listed (facts visible to that module).
 *                    An empty string indicates that all facts in all
 *                    modules should be listed. Default: emtpy string.
 * @param start       Optional. The index of the last fact to be listed.
 *                    Facts with indices less than value are not
 *                    listed. A value of -1 prints from the first fact.
 *                    Default: -1
 * @param end         Optional. The index of the last fact to be listed.
 *                    Facts with indices greater than value are not
 *                    listed. A value of -1 prints till the last fact.
 *                    Default: -1
 * @param max         Optional. The maximum number of facts to be listed.
 *                    A value of -1 does not restrict the output.
 *                    Default: -1
 */
void printFacts(
	const std::string& logicalName="stdout",
	const std::string& module="",
	size_t start=-1, size_t end=-1, size_t max=-1);

/**
 * Prints the list of defrules
 * It is the C equivalent of the CLIPS list-defrules command.
 * @remark            Wrapper for ListDefrules
 * @param logicalName Optional. The logical name to which the listing
 *                    output is sent. Default is "stdout".
 * @param module      Optional. The name of the module containing the
 *                    defrules to be listed. An empty string indicates
 *                    that defrules in all modules should be listed.
 */
void printDefrules(
	const std::string& logicalName="stdout",
	const std::string& module="");

/**
 * Prints the list of defrules
 * It is the C equivalent of the CLIPS list-defrules command.
 * @remark            Wrapper for ListDefrules
 * @param logicalName Optional. The logical name to which the listing
 *                    output is sent. Default is "stdout".
 * @param module      Optional. The name of the module containing the
 *                    defrules to be listed. An empty string indicates
 *                    that defrules in all modules should be listed.
 */
inline
void printRules(
	const std::string& logicalName="stdout",
	const std::string& module=""){printDefrules(logicalName,module);}

/**
 * Prints the list of rules currently on the agenda
 * It is the C equivalent of the CLIPS agenda command.
 * @remark            Wrapper for Agenda
 * @param logicalName Optional. The logical name to which the listing
 *                    output is sent. Default is "stdout".
 * @param module      Optional. The name of the module containing the
 *                    agenda to be listed.
 *                    An empty string indicates that the agenda for all
 *                    modules should be listed. Default: emtpy string.
 */
void printAgenda(
	const std::string& logicalName="stdout",
	const std::string& module="");

/**
 * Asserts a fact into the CLIPS fact-list
 * It is the C equivalent of the CLIPS assert-string command
 * @param s A string containing a list of primitive data types
 *          (symbols, strings, integers, floats, and/or instance
 *          names).
 */
void assertString(const std::string& s);

/**
 * Queries all active routers until it finds a router that recognizes
 * the logical name associated with this I/O request to print a string.
 * It then calls the print function associated with that router.
 * @remark             Wrapper for PrintCLIPS
 * @param  logicalName The logical name associated with the location at
 *                     which the string is to be printed.
 * @param  str         The string that is to be printed.
 * @return             true if the logical name is recognized,
 *                     false otherwise.
 */
bool print(
	const std::string& logicalName,
	const std::string& str);

// Watch
// Unwatch
// Retract
// PrintCLIPS
// FindDefrule
// Undefrule



/**
 * Returns the list of defrules in the specified module
 * It is the C equivalent of the CLIPS get-defrule-list function.
 * @remark       Wrapper for getDefruleList. defrules are returned as
 *               a vector of string rather than as a multifield DO.
 * @param module Optional. The name of the module from which the list
 *               will be extracted. An empty string indicates that the
 *               list is to be extracted from all modules.
 *               Default: empty string
 */
std::vector<std::string> getDefruleList(std::string const& module="");

/**
 * Re-routes stdin and arguments passed to the application for
 * CLIPS to handle.
 * @remark     Wrapper for RerouteStdin
 * @param argc The main's function argc
 * @param argv The main's function argv
 */
void rerouteStdin(int argc, char** argv);

/**
 * Clears the CLIPS environment.
 * It is the C equivalent of the CLIPS clear command.
 * @remark             Wrapper for Clear
 */
void clear();


/**
 * Injects a command into clips for its evaluation and execution
 * @param  s       The string containing the CLIPS code to inject
 * @param  verbose Unsupported for CLIPS 6.0
 * @return         true if the stirng was successfully injected,
 *                 false otherwise
 */
bool sendCommand(std::string const& s, bool verbose=false);


/**
 * Injects a command or \p query into clips for its evaluation and
 * execution, capturing whatever output is produced by CLIPS
 * in \p result
 * @param  query  The query to inject to CLIPS.
 * @param  result When this function returns, contains the output
 *                yielded by CLIPS during the execution.
 * @param  steps  When this function returns, contains the number
 *                of steps executed when evaluating \p query.
 * @return        True if the command was executed regardless of
 *                the number of execution steps, false otherwise.
 */
bool query(const std::string& query, std::string& result);

/**
 * Injects a command or \p query into clips for its evaluation and
 * execution, capturing whatever output is produced by CLIPS
 * in \p result
 * @param  query  The query to inject to CLIPS.
 * @param  result When this function returns, contains the output
 *                yielded by CLIPS during the execution.
 * @param  steps  When this function returns, contains the number
 *                of steps executed when evaluating \p query.
 * @return        True if the command was executed regardless of
 *                the number of execution steps, false otherwise.
 */
bool query(const std::string& query, std::string& result, int& steps);


/**
 * Determines if any changes to the fact list have occurred.
 * This function is primarily used to determine when to update a
 * display tracking the fact list.
 * It is the user's responsibility to call @c setFactListChanged(false)
 * to reset the internal flag when this function returns true.
 * Otherwise, this function will continue to return true even when no
 * changes have occurred.
 * @remark        Wrapper for getFactListChanged
 * @return        true if changes to the fact list have occurred,
 *                false otherwise.
 */
bool getFactListChanged();


/**
 * Sets the internal boolean flag which indicates when changes to the
 * fact list have occurred. This function is normally used to reset
 * the flag to zero after GetFactListChanged() returns non-zero.
 * @remark        Wrapper for SetFactListChanged
 * @param changed Indicates whether changes in the fact list have
 *                occurred or not.
 */
void setFactListChanged(bool changed);



/* ** ***************************************************************
*
* DefineFunction-related
*
** ** **************************************************************/

/**
 * [argCountCheck description]
 * @param  functionName [description]
 * @param  restriction  [description]
 * @param  count        [description]
 * @return              [description]
 */
bool argCountCheck(
	const std::string& functionName,
	const ArgCountRestriction& restriction,
	int count);

/**
 * Implementation for the defineFunction macro (use macro instead).
 * Describes and registers an external function to CLIPS so it can be
 * accessed by CLIPS programs.
 * @remark                    It wraps over DefineFunction and DefineFunction2
 * @param  functionName       The CLIPS function name.
 * @param  returnType         A string containing the type of the value
 *                            which will be returned to CLIPS.
 * @param  functionPointer    A pointer to the actual function in C.
 * @param  actualFunctionName Name used for funcPtr but as string.
 * @param  restrictions       Optional. Restriction string used by CLIPS
 *                            DefineFunction2. Default: empty.
 * @return                    true if the function was successfully
 *                            registered. false otherwise.
 */
bool defineFunction_impl(
	const std::string& functionName,
	const char& returnType,
	int (*functionPointer)(),
	const std::string& actualFunctionName,
	const std::string& restrictions=""
);

/**
 * Returns an integer telling with how many arguments a
 * user-function was called from CLIPS
 * @remark Wrapper for RtnArgCount
 * @return The number of arguments passed to the user function.
 */
int returnArgCount();

/**
 * Returns a floating-point number from either an INTEGER or FLOAT
 * data type that was passed to a user function
 * @remark        Wrapper for RtnDouble
 * @param  argPos The position of the argument to retrieve/convert
 * @return        The string representation of the argument
 */
double returnDouble(const int& argPos);

/**
 * Returns an integer number from either an INTEGER or FLOAT
 * data type that was passed to a user function
 * @remark        Wrapper for RtnLong
 * @param  argPos The position of the argument to retrieve/convert
 * @return        The string representation of the argument
 */
int returnInt(const int& argPos);

/**
 * Returns an std::string from either a symbol, string, or instance
 * name data type that was passed to a user function
 * @remark        Wrapper for RtnLexeme
 * @param  argPos The position of the argument to retrieve/convert
 * @return        The string representation of the argument
 */
std::string returnLexeme(const int& argPos);

/**
 * Returns an integer number from either an INTEGER or FLOAT
 * data type that was passed to a user function
 * @remark        Wrapper for RtnLong
 * @param  argPos The position of the argument to retrieve/convert
 * @return        The string representation of the argument
 */
long returnLong(const int& argPos);



/* ** ***************************************************************
*
* Watch-related
*
** ** **************************************************************/
/**
 * Activates the tracing of the specified item(s)
 * @param  item The item(s) to activate the watch on
 * @return      True if tracing on the specified watch item was
 *              sucessfully set, false otherwise. When several
 *              items are ORed, returns true only if ALL watches
 *              were set.
 */
bool watch(const WatchItem& item);


/**
 * Deactivates the tracing of the specified item(s)
 * @param  item The item(s) to deactivate the watch on
 * @return      True if tracing on the specified watch item was
 *              sucessfully cleared, false otherwise. When several
 *              items are ORed, returns true only if ALL watches
 *              were cleared.
 */
bool unwatch(const WatchItem& item);

/**
 * Toggles the tracing of the specified item(s)
 * @param  item The item(s) wose tracing will be toggled
 * @return      An ORed WatchItem containing the watch status of all
 *              the requested items after the toggle opperation.
 */
WatchItem toggleWatch(const WatchItem& item);

/**
 * Toggles the tracing status of the specified item(s)
 * @param  item The items being traced (watched).
 * @return      An ORed WatchItem containing the watch status of all
 *              the items being traced (watched).
 */
WatchItem getWatches();

/**
 * Gets a value indicating whether the specified item is being watched
 * @param  item The item to be queried.
 * @return      True if the specified item is being traced (watched),
 *              false otherwise. When several items are queried returns
 *              true only if ALL items are being traced (watched).
 */
bool watching(const WatchItem& item);



/**
 * Returns a bitwise AND operation over the values of two WatchItem flags
 */
inline constexpr WatchItem operator&(WatchItem x, WatchItem y) {
	return static_cast<WatchItem>(static_cast<int>(x) & static_cast<int>(y));
}

/**
 * Returns a bitwise OR operation over the values of two WatchItem flags
 */
inline constexpr WatchItem operator|(WatchItem x, WatchItem y) {
	return static_cast<WatchItem>(static_cast<int>(x) | static_cast<int>(y));
}

/**
 * Returns a bitwise eXclussive-OR operation over the values of two WatchItem flags
 */
inline constexpr WatchItem operator^(WatchItem x, WatchItem y) {
	return static_cast<WatchItem>(static_cast<int>(x) ^ static_cast<int>(y));
}


}
#endif // __CLIPSWRAPPER_H__