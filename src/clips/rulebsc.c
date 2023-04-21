   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*          DEFRULE BASIC COMMANDS HEADER FILE         */
   /*******************************************************/

/*************************************************************/
/* Purpose: Implements core commands for the defrule         */
/*   construct such as clear, reset, save, undefrule,        */
/*   ppdefrule, list-defrules, and                           */
/*   get-defrule-list.                                       */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _RULEBSC_SOURCE_

#include "setup.h"

#if DEFRULE_CONSTRUCT

#include <stdio.h>
#define _CLIPS_STDIO_

#include "argacces.h"
#include "constrct.h"
#include "router.h"
#include "watch.h"
#include "extnfunc.h"
#include "ruledef.h"
#include "engine.h"
#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
#include "rulebin.h"
#endif
#if CONSTRUCT_COMPILER && (! RUN_TIME)
#include "rulecmp.h"
#endif

#include "rulebsc.h"

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

#if DEBUGGING_FUNCTIONS
   globle BOOLEAN                 WatchRules = OFF;
#endif

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                    ResetDefrules(void);
   static VOID                    SaveDefrules(char *);
#if (! RUN_TIME)
   static int                     ClearDefrulesReady(void);
   static VOID                    ClearDefrules(void);
#endif
#else
   static VOID                    ResetDefrules();
   static VOID                    SaveDefrules();
#if (! RUN_TIME)
   static int                     ClearDefrulesReady();
   static VOID                    ClearDefrules();
#endif
#endif

/*************************************************************/
/* DefruleBasicCommands: Initializes basic defrule commands. */
/*************************************************************/
globle VOID DefruleBasicCommands()
  {
   AddResetFunction("defrule",ResetDefrules,70);
   AddSaveFunction("defrule",SaveDefrules,0);
#if (! RUN_TIME)
   AddClearReadyFunction("defrule",ClearDefrulesReady,0);
   AddClearFunction("defrule",ClearDefrules,0);
#endif
   
#if DEBUGGING_FUNCTIONS
   AddWatchItem("rules",0,&WatchRules,70,DefruleWatchAccess,DefruleWatchPrint);
#endif

#if ! RUN_TIME
   DefineFunction2("get-defrule-list",'m',PTIF GetDefruleListFunction,"GetDefruleListFunction","01w");
   DefineFunction2("undefrule",'v',PTIF UndefruleCommand,"UndefruleCommand","11w");
   DefineFunction2("defrule-module",'w',PTIF DefruleModuleFunction,"DefruleModuleFunction","11w");

#if DEBUGGING_FUNCTIONS
   DefineFunction2("rules",'v', PTIF ListDefrulesCommand,"ListDefrulesCommand","01w");
   DefineFunction2("list-defrules",'v', PTIF ListDefrulesCommand,"ListDefrulesCommand","01w");
   DefineFunction2("ppdefrule",'v',PTIF PPDefruleCommand,"PPDefruleCommand","11w");
#endif

#if (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE)
   DefruleBinarySetup();
#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
   DefruleCompilerSetup();
#endif

#endif
  }
  
/*****************************************************/
/* ResetDefrules: Defrule reset routine for use with */
/*   the reset command. Sets the current entity time */
/*   tag (used by the conflict resolution strategies */
/*   for recency) to zero. The focus stack is also   */
/*   cleared.                                        */
/*****************************************************/
static VOID ResetDefrules()
  {
   struct defmodule *theModule;

   CurrentEntityTimeTag = 0L;
   ClearFocusStack();
   theModule = (struct defmodule *) FindDefmodule("MAIN");
   Focus((VOID *) theModule);
  }

#if (! RUN_TIME)

/******************************************************************/
/* ClearDefrulesReady: Indicates whether defrules can be cleared. */
/******************************************************************/
static int ClearDefrulesReady()
  {
   if (ExecutingRule != NULL) return(CLIPS_FALSE);

   ClearFocusStack();
   if (GetCurrentModule() == NULL) return(CLIPS_FALSE);
   
   CurrentEntityTimeTag = 0L;
   
   return(CLIPS_TRUE);
  }

/***************************************************************/
/* ClearDefrules: Pushes the MAIN module as the current focus. */
/***************************************************************/
static VOID ClearDefrules()
  {
   struct defmodule *theModule;
   
   theModule = (struct defmodule *) FindDefmodule("MAIN");
   Focus((VOID *) theModule);
  }
#endif

/**********************************************/
/* SaveDefrules: Defrule save routine */
/*   for use with the save command.           */
/**********************************************/
static VOID SaveDefrules(logicalName)
  char *logicalName;
  { SaveConstruct(logicalName,DefruleConstruct); }

/**********************************************/
/* UndefruleCommand: CLIPS access routine */
/*   for the undefrule command.           */
/**********************************************/
globle VOID UndefruleCommand()
  { UndefconstructCommand("undefrule",DefruleConstruct); }

/************************************/
/* Undefrule: C access routine  */
/*   for the undefrule command. */
/************************************/
globle BOOLEAN Undefrule(theDefrule)
  VOID *theDefrule;
  { return(Undefconstruct(theDefrule,DefruleConstruct)); }
 
/************************************************/
/* GetDefruleListFunction: CLIPS access routine */
/*   for the get-defrule-list function.         */
/************************************************/
globle VOID GetDefruleListFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  { GetConstructListFunction("get-defrule-list",returnValue,DefruleConstruct); }

/****************************************/
/* GetDefruleList: C access routine for */
/*   the get-defrule-list function.     */
/****************************************/
globle VOID GetDefruleList(returnValue,theModule)
  DATA_OBJECT_PTR returnValue;
  VOID *theModule;
  { GetConstructList(returnValue,DefruleConstruct,(struct defmodule *) theModule); }

/*******************************************/
/* DefruleModuleFunction: C access routine */
/*   for the defrule-module function.      */
/*******************************************/
globle SYMBOL_HN *DefruleModuleFunction()
  { return(GetConstructModuleCommand("defrule-module",DefruleConstruct)); }

#if DEBUGGING_FUNCTIONS

/**********************************************/
/* PPDefruleCommand: CLIPS access routine */
/*   for the ppdefrule command.           */
/**********************************************/
globle VOID PPDefruleCommand()
  { PPConstructCommand("ppdefrule",DefruleConstruct); }

/***************************************/
/* PPDefrule: C access routine for */
/*   the ppdefrule command.        */
/***************************************/
globle int PPDefrule(defruleName,logicalName)
  char *defruleName, *logicalName;
  { return(PPConstruct(defruleName,logicalName,DefruleConstruct)); }

/*********************************************/
/* ListDefrulesCommand: CLIPS access routine */
/*   for the list-defrules command.          */
/*********************************************/
globle VOID ListDefrulesCommand()
  { ListConstructCommand("list-defrules",DefruleConstruct); }

/**************************************/
/* ListDefrules: C access routine for */
/*   the list-defrules command.       */
/**************************************/
globle VOID ListDefrules(logicalName,theModule)
  char *logicalName;
  VOID *theModule;
  { ListConstruct(DefruleConstruct,logicalName,(struct defmodule *) theModule); }
  
/***************************************************************/
/* GetDefruleWatchActivations: C access routine for retrieving */
/*   the current watch value of a defrule's activations.       */
/***************************************************************/
globle BOOLEAN GetDefruleWatchActivations(rulePtr)
  VOID *rulePtr;
  {
   struct defrule *thePtr;

   thePtr = (struct defrule *) rulePtr;
   while (thePtr != NULL)
     {
      if (thePtr->watchActivation) return(CLIPS_TRUE);
      thePtr = thePtr->disjunct;
     }
     
   return(CLIPS_FALSE);
  }
  
/***********************************************************/
/* GetDefruleWatchFirings: C access routine for retrieving */
/*   the current watch value of a defrule's firings.       */
/***********************************************************/
globle BOOLEAN GetDefruleWatchFirings(rulePtr)
  VOID *rulePtr;
  {
   struct defrule *thePtr;

   thePtr = (struct defrule *) rulePtr;
   while (thePtr != NULL)
     {
      if (thePtr->watchFiring) return(CLIPS_TRUE);
      thePtr = thePtr->disjunct;
     }
     
   return(CLIPS_FALSE);
  }

/************************************************************/
/* SetDefruleWatchActivations: C access routine for setting */
/*   the current watch value of a defrule's activations.    */
/************************************************************/
globle VOID SetDefruleWatchActivations(newState,rulePtr)
  int newState;
  VOID *rulePtr;
  {
   struct defrule *thePtr;

   thePtr = (struct defrule *) rulePtr;
   while (thePtr != NULL)
     {
      thePtr->watchActivation = newState;
      thePtr = thePtr->disjunct;
     }
  }
  
/********************************************************/
/* SetDefruleWatchFirings: C access routine for setting */
/*   the current watch value of a defrule's firings.    */
/********************************************************/
globle VOID SetDefruleWatchFirings(newState,rulePtr)
  int newState;
  VOID *rulePtr;
  {
   struct defrule *thePtr;

   thePtr = (struct defrule *) rulePtr;
   while (thePtr != NULL)
     {
      thePtr->watchFiring = newState;
      thePtr = thePtr->disjunct;
     }
  }
  
/*******************************************************************/
/* DefruleWatchAccess: Access function for setting the watch flags */
/*   associated with rules (activations and rule firings).         */
/*******************************************************************/
globle BOOLEAN DefruleWatchAccess(code,newState,argExprs)
  int code,newState;
  struct expr *argExprs;
  {
   if (code)
     return(ConstructSetWatchAccess(DefruleConstruct,newState,argExprs,
                                    GetDefruleWatchActivations,SetDefruleWatchActivations));
   else
     return(ConstructSetWatchAccess(DefruleConstruct,newState,argExprs,
                                    GetDefruleWatchFirings,SetDefruleWatchFirings));
  }

/*****************************************************************/
/* DefruleWatchPrint: Access routine for printing which defrules */
/*   have their watch flag set via the list-watch-items command. */
/*****************************************************************/
globle BOOLEAN DefruleWatchPrint(log,code,argExprs)
  char *log;
  int code;
  struct expr *argExprs;
  {
   if (code)
     return(ConstructPrintWatchAccess(DefruleConstruct,log,argExprs,
                                      GetDefruleWatchActivations,SetDefruleWatchActivations));
   else
     return(ConstructPrintWatchAccess(DefruleConstruct,log,argExprs,
                                      GetDefruleWatchActivations,SetDefruleWatchActivations));
  }

#endif /* DEBUGGING_FUNCTIONS */

#endif /* DEFTEMPLATE_CONSTRUCT */


