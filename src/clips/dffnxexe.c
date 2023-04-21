   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                                                     */
   /*******************************************************/

/*************************************************************/
/* Purpose: CLIPS Deffunction Execution Routines             */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/
   
/* =========================================
   *****************************************
               EXTERNAL DEFINITIONS
   =========================================
   ***************************************** */
#include "setup.h"

#if DEFFUNCTION_CONSTRUCT

#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#include "constrct.h"
#include "prcdrfun.h"
#include "prccode.h"
#include "router.h"
#include "utility.h"
#include "watch.h"

#define _DFFNXEXE_SOURCE_
#include "dffnxexe.h"

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */
#define BEGIN_TRACE ">> "
#define END_TRACE   "<< "

/* =========================================
   *****************************************
               MACROS AND TYPES
   =========================================
   ***************************************** */   

/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER

static VOID UnboundDeffunctionErr(void);

#if DEBUGGING_FUNCTIONS
static VOID WatchDeffunction(char *);
#endif

#else

static VOID UnboundDeffunctionErr();

#if DEBUGGING_FUNCTIONS
static VOID WatchDeffunction();
#endif

#endif
      

/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
globle DEFFUNCTION *ExecutingDeffunction = NULL;

/* =========================================
   *****************************************
      INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

/****************************************************
  NAME         : CallDeffunction
  DESCRIPTION  : Executes the body of a deffunction
  INPUTS       : 1) The deffunction
                 2) Argument expressions
                 3) Data object buffer to hold result
  RETURNS      : Nothing useful
  SIDE EFFECTS : Deffunction executed and result
                 stored in data object buffer
  NOTES        : Used in EvaluateExpression()
 ****************************************************/
globle VOID CallDeffunction(dptr,args,result)
  DEFFUNCTION *dptr;
  EXPRESSION *args;
  DATA_OBJECT *result;
  {
   int oldce;
   DEFFUNCTION *previouslyExecutingDeffunction;

   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   EvaluationError = CLIPS_FALSE;
   if (HaltExecution)
     return;
   oldce = ExecutingConstruct();
   SetExecutingConstruct(CLIPS_TRUE);
   previouslyExecutingDeffunction = ExecutingDeffunction;
   ExecutingDeffunction = dptr;
   CurrentEvaluationDepth++;
   dptr->executing++;
   PushProcParameters(args,CountArguments(args),GetDeffunctionName((VOID *) dptr),
                      "deffunction",UnboundDeffunctionErr);
   if (EvaluationError)
     {
      dptr->executing--;
      ExecutingDeffunction = previouslyExecutingDeffunction;
      CurrentEvaluationDepth--;
      PeriodicCleanup(CLIPS_FALSE,CLIPS_TRUE);
      SetExecutingConstruct(oldce);
      return;
     }

#if DEBUGGING_FUNCTIONS
   if (dptr->trace)
     WatchDeffunction(BEGIN_TRACE);
#endif
   EvaluateProcActions(dptr->header.whichModule->theModule,
                       dptr->code,dptr->numberOfLocalVars,
                       result,UnboundDeffunctionErr);
#if DEBUGGING_FUNCTIONS
   if (dptr->trace)
     WatchDeffunction(END_TRACE);
#endif
   ReturnFlag = CLIPS_FALSE;

   dptr->executing--;
   PopProcParameters();
   ExecutingDeffunction = previouslyExecutingDeffunction;
   CurrentEvaluationDepth--;
   PropagateReturnValue(result);
   PeriodicCleanup(CLIPS_FALSE,CLIPS_TRUE);
   SetExecutingConstruct(oldce);
  }
   
/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
      
/*******************************************************
  NAME         : UnboundDeffunctionErr
  DESCRIPTION  : Print out a synopis of the currently
                   executing deffunction for unbound
                   variable errors
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Error synopsis printed to WERROR
  NOTES        : None
 *******************************************************/
static VOID UnboundDeffunctionErr()
  {
   PrintCLIPS(WERROR,"deffunction ");
   PrintCLIPS(WERROR,GetDeffunctionName((VOID *) ExecutingDeffunction));
   PrintCLIPS(WERROR,".\n");
  }

#if DEBUGGING_FUNCTIONS

/***************************************************
  NAME         : WatchDeffunction
  DESCRIPTION  : Displays a message indicating when
                 a deffunction began and ended
                 execution
  INPUTS       : The beginning or end trace string
                 to print when deffunction starts
                 or finishes respectively
  RETURNS      : Nothing useful
  SIDE EFFECTS : Watch message printed
  NOTES        : None
 ***************************************************/
static VOID WatchDeffunction(tstring)
  char *tstring;
  {
   PrintCLIPS(WTRACE,"DFN ");
   PrintCLIPS(WTRACE,tstring);
   if (ExecutingDeffunction->header.whichModule->theModule != ((struct defmodule *) GetCurrentModule()))
     {
      PrintCLIPS(WTRACE,GetDefmoduleName((VOID *) 
                        ExecutingDeffunction->header.whichModule->theModule));
      PrintCLIPS(WTRACE,"::");
     }
   PrintCLIPS(WTRACE,ValueToString(ExecutingDeffunction->header.name));
   PrintCLIPS(WTRACE," ED:");
   PrintLongInteger(WTRACE,(long) CurrentEvaluationDepth);
   PrintProcParamArray(WTRACE);
  }

#endif
#endif

/***************************************************
  NAME         : 
  DESCRIPTION  : 
  INPUTS       : 
  RETURNS      : 
  SIDE EFFECTS : 
  NOTES        : 
 ***************************************************/
