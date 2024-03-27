   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*        DEFTEMPLATE BASIC COMMANDS HEADER FILE       */
   /*******************************************************/

/*************************************************************/
/* Purpose: Implements core commands for the deftemplate     */
/*   construct such as clear, reset, save, undeftemplate,    */
/*   ppdeftemplate, list-deftemplates, and                   */
/*   get-deftemplate-list.                                   */
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

#define _TMPLTBSC_SOURCE_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "argacces.h"
#include "clipsmem.h"
#include "scanner.h"
#include "router.h"
#include "extnfunc.h"
#include "constrct.h"
#include "cstrccom.h"
#include "factrhs.h"
#include "cstrcpsr.h"
#include "tmpltpsr.h"
#include "tmpltdef.h"
#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
#include "tmpltbin.h"
#endif
#if CONSTRUCT_COMPILER && (! RUN_TIME)
#include "tmpltcmp.h"
#endif

#include "tmpltbsc.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                    ResetDeftemplates(void);
   static VOID                    ClearDeftemplates(void);
   static VOID                    SaveDeftemplates(char *);
#else
   static VOID                    ResetDeftemplates();
   static VOID                    ClearDeftemplates();
   static VOID                    SaveDeftemplates();
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

#if DEBUGGING_FUNCTIONS
   globle int                     DeletedTemplateDebugFlags = CLIPS_FALSE;
#endif

/*********************************************************************/
/* DeftemplateBasicCommands: Initializes basic deftemplate commands. */
/*********************************************************************/
globle VOID DeftemplateBasicCommands()
  {
   AddResetFunction("deftemplate",ResetDeftemplates,0);
   AddClearFunction("deftemplate",ClearDeftemplates,0);
   AddSaveFunction("deftemplate",SaveDeftemplates,10);
   
#if ! RUN_TIME
   DefineFunction2("get-deftemplate-list",'m',PTIF GetDeftemplateListFunction,"GetDeftemplateListFunction","01w");
   DefineFunction2("undeftemplate",'v',PTIF UndeftemplateCommand,"UndeftemplateCommand","11w");
   DefineFunction2("deftemplate-module",'w',PTIF DeftemplateModuleFunction,"DeftemplateModuleFunction","11w");

#if DEBUGGING_FUNCTIONS
   DefineFunction2("list-deftemplates",'v', PTIF ListDeftemplatesCommand,"ListDeftemplatesCommand","01w");
   DefineFunction2("ppdeftemplate",'v',PTIF PPDeftemplateCommand,"PPDeftemplateCommand","11w");
#endif

#if (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE)
   DeftemplateBinarySetup();
#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
   DeftemplateCompilerSetup();
#endif

#endif
  }
  
/*************************************************************/
/* ResetDeftemplates: Deftemplate reset routine for use with */
/*   the reset command. Asserts the initial-fact fact when   */
/*   the deftemplate construct has been disabled.            */
/*************************************************************/
static VOID ResetDeftemplates()
  { 
#if ! DEFFACTS_CONSTRUCT
   struct fact *factPtr;

   factPtr = StringToFact("(initial-fact)");

   if (factPtr == NULL) return;

   Assert((VOID *) factPtr);
#endif
 }

/*****************************************************************/
/* ClearDeftemplates: Deftemplate clear routine for use with the */
/*   clear command. Creates the initial-facts deftemplate.       */
/*****************************************************************/
static VOID ClearDeftemplates()
  {
#if (! RUN_TIME) && (! BLOAD_ONLY)
   CreateImpliedDeftemplate(AddSymbol("initial-fact"),CLIPS_FALSE);
#endif
  }  

/**********************************************/
/* SaveDeftemplates: Deftemplate save routine */
/*   for use with the save command.           */
/**********************************************/
static VOID SaveDeftemplates(logicalName)
  char *logicalName;
  { SaveConstruct(logicalName,DeftemplateConstruct); }

/**********************************************/
/* UndeftemplateCommand: CLIPS access routine */
/*   for the undeftemplate command.           */
/**********************************************/
globle VOID UndeftemplateCommand()
  { UndefconstructCommand("undeftemplate",DeftemplateConstruct); }

/************************************/
/* Undeftemplate: C access routine  */
/*   for the undeftemplate command. */
/************************************/
globle BOOLEAN Undeftemplate(theDeftemplate)
  VOID *theDeftemplate;
  { return(Undefconstruct(theDeftemplate,DeftemplateConstruct)); }
 
/****************************************************/
/* GetDeftemplateListFunction: CLIPS access routine */
/*   for the get-deftemplate-list function.         */
/****************************************************/
globle VOID GetDeftemplateListFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  { GetConstructListFunction("get-deftemplate-list",returnValue,DeftemplateConstruct); }

/************************************************/
/* GetDeftemplateListFunction: C access routine */
/*   for the get-deftemplate-list function.     */
/************************************************/
globle VOID GetDeftemplateList(returnValue,theModule)
  DATA_OBJECT_PTR returnValue;
  VOID *theModule;
  { GetConstructList(returnValue,DeftemplateConstruct,theModule); }

/***************************************************/
/* DeftemplateModuleFunction: CLIPS access routine */
/*   for the deftemplate-module function.          */
/***************************************************/
globle SYMBOL_HN *DeftemplateModuleFunction()
  { return(GetConstructModuleCommand("deftemplate-module",DeftemplateConstruct)); }

#if DEBUGGING_FUNCTIONS

/**********************************************/
/* PPDeftemplateCommand: CLIPS access routine */
/*   for the ppdeftemplate command.           */
/**********************************************/
globle VOID PPDeftemplateCommand()
  { PPConstructCommand("ppdeftemplate",DeftemplateConstruct); }

/***************************************/
/* PPDeftemplate: C access routine for */
/*   the ppdeftemplate command.        */
/***************************************/
globle int PPDeftemplate(deftemplateName,logicalName)
  char *deftemplateName, *logicalName;
  { return(PPConstruct(deftemplateName,logicalName,DeftemplateConstruct)); }

/*************************************************/
/* ListDeftemplatesCommand: CLIPS access routine */
/*   for the list-deftemplates command.          */
/*************************************************/
globle VOID ListDeftemplatesCommand()
  { ListConstructCommand("list-deftemplates",DeftemplateConstruct); }

/******************************************/
/* ListDeftemplates: C access routine for */
/*   the list-deftemplates command.       */
/******************************************/
globle VOID ListDeftemplates(logicalName,theModule)
  char *logicalName;
  VOID *theModule;
  { ListConstruct(DeftemplateConstruct,logicalName,(struct defmodule *) theModule); }
  
/********************************************************/
/* GetDeftemplateWatch: C access routine for retrieving */
/*   the current watch value of a deftemplate.          */
/********************************************************/
globle BOOLEAN GetDeftemplateWatch(theTemplate)
  VOID *theTemplate;
  { return(((struct deftemplate *) theTemplate)->watch); }

/******************************************************/
/* SetDeftemplateWatch:  C access routine for setting */
/*   the current watch value of a deftemplate.        */
/******************************************************/
globle VOID SetDeftemplateWatch(newState,theTemplate)
  int newState;
  VOID *theTemplate;
  { ((struct deftemplate *) theTemplate)->watch = newState; }

/**********************************************************/
/* DeftemplateWatchAccess: Access routine for setting the */
/*   watch flag of a deftemplate via the watch command.   */
/**********************************************************/
#if IBM_TBC
#pragma argsused
#endif
globle BOOLEAN DeftemplateWatchAccess(code,newState,argExprs)
  int code, newState;
  EXPRESSION *argExprs;
  {
#if MAC_MPW
#pragma unused(code)
#endif
   return(ConstructSetWatchAccess(DeftemplateConstruct,newState,argExprs,
                                  GetDeftemplateWatch,SetDeftemplateWatch));
  }

/*************************************************************************/
/* DeftemplateWatchPrint: Access routine for printing which deftemplates */
/*   have their watch flag set via the list-watch-items command.         */
/*************************************************************************/
#if IBM_TBC
#pragma argsused
#endif
globle BOOLEAN DeftemplateWatchPrint(log,code,argExprs)
  char *log; 
  int code;
  EXPRESSION *argExprs;
  {
#if MAC_MPW
#pragma unused(code)
#endif
   return(ConstructPrintWatchAccess(DeftemplateConstruct,log,argExprs,
                                    GetDeftemplateWatch,SetDeftemplateWatch));
  }

#endif /* DEBUGGING_FUNCTIONS */

#endif /* DEFTEMPLATE_CONSTRUCT */


