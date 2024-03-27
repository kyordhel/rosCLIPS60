   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*         DEFGLOBAL BASIC COMMANDS HEADER FILE        */
   /*******************************************************/

/*************************************************************/
/* Purpose: Implements core commands for the defglobal       */
/*   construct such as clear, reset, save, undefglobal,      */
/*   ppdefglobal, list-defglobals, and get-defglobals-list.  */
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

#define _GLOBLBSC_SOURCE_

#include "setup.h"

#if DEFGLOBAL_CONSTRUCT

#include "constrct.h"
#include "extnfunc.h"
#include "watch.h"

#include "globlcom.h"
#include "globldef.h"

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
#include "globlbin.h"
#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
#include "globlcmp.h"
#endif

#include "globlbsc.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                    SaveDefglobals(char *);
   static VOID                    ResetDefglobalAction(struct constructHeader *,VOID *);
#if DEBUGGING_FUNCTIONS && (! RUN_TIME)
   static BOOLEAN                 DefglobalWatchAccess(int,int,struct expr *);
   static BOOLEAN                 DefglobalWatchPrint(char *,int,struct expr *);
#endif
#else
   static VOID                    SaveDefglobals();
   static VOID                    ResetDefglobalAction();
#if DEBUGGING_FUNCTIONS && (! RUN_TIME)
   static BOOLEAN                 DefglobalWatchAccess();
   static BOOLEAN                 DefglobalWatchPrint();
#endif
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

#if DEBUGGING_FUNCTIONS
   globle BOOLEAN               WatchGlobals = OFF;
#endif

/*****************************************************************/
/* DefglobalBasicCommands: Initializes basic defglobal commands. */
/*****************************************************************/
globle VOID DefglobalBasicCommands()
  {
   AddSaveFunction("defglobal",SaveDefglobals,40);
   AddResetFunction("defglobal",ResetDefglobals,50);
   
#if ! RUN_TIME
   DefineFunction2("get-defglobal-list",'m',PTIF GetDefglobalListFunction,"GetDefglobalListFunction","01w");
   DefineFunction2("undefglobal",'v',PTIF UndefglobalCommand,"UndefglobalCommand","11w");
   DefineFunction2("defglobal-module",'w',PTIF DefglobalModuleFunction,"DefglobalModuleFunction","11w");

#if DEBUGGING_FUNCTIONS
   DefineFunction2("list-defglobals",'v', PTIF ListDefglobalsCommand,"ListDefglobalsCommand","01w");
   DefineFunction2("ppdefglobal",'v',PTIF PPDefglobalCommand,"PPDefglobalCommand","11w");
   AddWatchItem("globals",0,&WatchGlobals,0,DefglobalWatchAccess,DefglobalWatchPrint);
#endif

#if (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE)
   DefglobalBinarySetup();
#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
   DefglobalCompilerSetup();
#endif

#endif
  }
  
/*************************************************************/
/* ResetDefglobals: Defglobal reset routine for use with the */
/*   reset command. Restores the values of the defglobals.   */
/*************************************************************/
globle VOID ResetDefglobals()
  { 
   if (! GetResetGlobals()) return;
   DoForAllConstructs(ResetDefglobalAction,DefglobalModuleIndex,CLIPS_TRUE,NULL);
  }

/******************************************************/
/* ResetDefglobalAction: Action to be applied to each */
/*   defglobal construct during a reset command.      */
/******************************************************/
#if IBM_TBC
#pragma argsused
#endif
static VOID ResetDefglobalAction(theConstruct,buffer)
  struct constructHeader *theConstruct;   
  VOID *buffer;
  {
#if MAC_MPW
#pragma unused(buffer)
#endif
   struct defglobal *theDefglobal = (struct defglobal *) theConstruct;
   DATA_OBJECT assignValue;
   
   if (EvaluateExpression(theDefglobal->initial,&assignValue))
     {
      assignValue.type = SYMBOL;
      assignValue.value = CLIPSFalseSymbol;
     }

   QSetDefglobalValue(theDefglobal,&assignValue);
  }

/******************************************/
/* SaveDefglobals: Defglobal save routine */
/*   for use with the save command.       */
/******************************************/
static VOID SaveDefglobals(logicalName)
  char *logicalName;
  { SaveConstruct(logicalName,DefglobalConstruct); }

/********************************************/
/* UndefglobalCommand: CLIPS access routine */
/*   for the undefglobal command.           */
/********************************************/
globle VOID UndefglobalCommand()
  { UndefconstructCommand("undefglobal",DefglobalConstruct); }

/**********************************/
/* Undefglobal: C access routine  */
/*   for the undefglobal command. */
/**********************************/
globle BOOLEAN Undefglobal(theDefglobal)
  VOID *theDefglobal;
  { return(Undefconstruct(theDefglobal,DefglobalConstruct)); }
 
/**************************************************/
/* GetDefglobalListFunction: CLIPS access routine */
/*   for the get-defglobal-list function.         */
/**************************************************/
globle VOID GetDefglobalListFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  { GetConstructListFunction("get-defglobal-list",returnValue,DefglobalConstruct); }

/******************************************/
/* GetDefglobalList: C access routine for */
/*   the get-defglobal-list function.     */
/******************************************/
globle VOID GetDefglobalList(returnValue,theModule)
  DATA_OBJECT_PTR returnValue;
  VOID *theModule;
  { GetConstructList(returnValue,DefglobalConstruct,theModule); }

/*******************************************/
/* DefglobalModuleFunction: CLIPS access routine */
/*   for the defglobal-module function.         */
/***********************************************/
globle SYMBOL_HN *DefglobalModuleFunction()
  { return(GetConstructModuleCommand("defglobal-module",DefglobalConstruct)); }

#if DEBUGGING_FUNCTIONS

/********************************************/
/* PPDefglobalCommand: CLIPS access routine */
/*   for the ppdefglobal command.           */
/********************************************/
globle VOID PPDefglobalCommand()
  { PPConstructCommand("ppdefglobal",DefglobalConstruct); }

/*************************************/
/* PPDefglobal: C access routine for */
/*   the ppdefglobal command.        */
/*************************************/
globle int PPDefglobal(defglobalName,logicalName)
  char *defglobalName, *logicalName;
  { return(PPConstruct(defglobalName,logicalName,DefglobalConstruct)); }

/***********************************************/
/* ListDefglobalsCommand: CLIPS access routine */
/*   for the list-defglobals command.          */
/***********************************************/
globle VOID ListDefglobalsCommand()
  { ListConstructCommand("list-defglobals",DefglobalConstruct); }

/****************************************/
/* ListDefglobals: C access routine for */
/*   the list-defglobals command.       */
/****************************************/
globle VOID ListDefglobals(logicalName,vTheModule)
  char *logicalName;
  VOID *vTheModule;
  { 
   struct defmodule *theModule = (struct defmodule *) vTheModule;
   
   ListConstruct(DefglobalConstruct,logicalName,theModule);
  }

/******************************************************/
/* GetDefglobalWatch: C access routine for retrieving */
/*   the current watch value of a defglobal.          */
/******************************************************/
globle BOOLEAN GetDefglobalWatch(theGlobal)
  VOID *theGlobal;
  { return(((struct defglobal *) theGlobal)->watch); }

/******************************************************/
/* SetDeftemplateWatch:  C access routine for setting */
/*   the current watch value of a deftemplate.        */
/******************************************************/
globle VOID SetDefglobalWatch(newState,theGlobal)
  int newState;
  VOID *theGlobal;
  { ((struct defglobal *) theGlobal)->watch = newState; }

#if ! RUN_TIME

/********************************************************/
/* DefglobalWatchAccess: Access routine for setting the */
/*   watch flag of a defglobal via the watch command.   */
/********************************************************/
#if IBM_TBC
#pragma argsused
#endif
static BOOLEAN DefglobalWatchAccess(code,newState,argExprs)
  int code, newState;
  EXPRESSION *argExprs;
  {
#if MAC_MPW
#pragma unused(code)
#endif
   return(ConstructSetWatchAccess(DefglobalConstruct,newState,argExprs,
                                  GetDefglobalWatch,SetDefglobalWatch));
  }

/*********************************************************************/
/* DefglobalWatchPrint: Access routine for printing which defglobals */
/*   have their watch flag set via the list-watch-items command.     */
/*********************************************************************/
#if IBM_TBC
#pragma argsused
#endif
static BOOLEAN DefglobalWatchPrint(log,code,argExprs)
  char *log; 
  int code;
  EXPRESSION *argExprs;
  {
#if MAC_MPW
#pragma unused(code)
#endif
   return(ConstructPrintWatchAccess(DefglobalConstruct,log,argExprs,
                                    GetDefglobalWatch,SetDefglobalWatch));
  }

#endif
  
#endif /* DEBUGGING_FUNCTIONS */

#endif /* DEFGLOBAL_CONSTRUCT */


                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           