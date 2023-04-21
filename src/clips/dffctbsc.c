   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*         DEFFACTS BASIC COMMANDS HEADER FILE         */
   /*******************************************************/

/*************************************************************/
/* Purpose: Implements core commands for the deffacts        */
/*   construct such as clear, reset, save, undeffacts,       */
/*   ppdeffacts, list-deffacts, and get-deffacts-list.       */
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

#define _DFFCTBSC_SOURCE_

#include "setup.h"

#if DEFFACTS_CONSTRUCT

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
#include "tmpltdef.h"
#include "cstrcpsr.h"
#include "dffctpsr.h"
#include "dffctdef.h"
#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
#include "dffctbin.h"
#endif
#if CONSTRUCT_COMPILER && (! RUN_TIME)
#include "dffctcmp.h"
#endif

#include "dffctbsc.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                    ResetDeffacts(void);
   static VOID                    ClearDeffacts(void);
   static VOID                    SaveDeffacts(char *);
   static VOID                    ResetDeffactsAction(struct constructHeader *,VOID *);
#else
   static VOID                    ResetDeffacts();
   static VOID                    ClearDeffacts();
   static VOID                    SaveDeffacts();
   static VOID                    ResetDeffactsAction();
#endif

/***************************************************************/
/* DeffactsBasicCommands: Initializes basic deffacts commands. */
/***************************************************************/
globle VOID DeffactsBasicCommands()
  {
   AddResetFunction("deffacts",ResetDeffacts,0);
   AddClearFunction("deffacts",ClearDeffacts,0);
   AddSaveFunction("deffacts",SaveDeffacts,10);
   
#if ! RUN_TIME
   DefineFunction2("get-deffacts-list",'m',PTIF GetDeffactsListFunction,"GetDeffactsListFunction","01w");
   DefineFunction2("undeffacts",'v',PTIF UndeffactsCommand,"UndeffactsCommand","11w");
   DefineFunction2("deffacts-module",'w',PTIF DeffactsModuleFunction,"DeffactsModuleFunction","11w");

#if DEBUGGING_FUNCTIONS
   DefineFunction2("list-deffacts",'v', PTIF ListDeffactsCommand,"ListDeffactsCommand","01w");
   DefineFunction2("ppdeffacts",'v',PTIF PPDeffactsCommand,"PPDeffactsCommand","11w");
#endif

#if (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE)
   DeffactsBinarySetup();
#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
   DeffactsCompilerSetup();
#endif

#endif
  }
  
/**********************************************************/
/* ResetDeffacts: Deffacts reset routine for use with the */
/*   reset command. Asserts all of the facts contained in */
/*   deffacts constructs.                                 */
/**********************************************************/
static VOID ResetDeffacts()
  { DoForAllConstructs(ResetDeffactsAction,DeffactsModuleIndex,CLIPS_TRUE,NULL); }

/*****************************************************/
/* ResetDeffactsAction: Action to be applied to each */
/*   deffacts construct during a reset command.      */
/*****************************************************/
#if IBM_TBC
#pragma argsused
#endif
static VOID ResetDeffactsAction(theConstruct,buffer)
  struct constructHeader *theConstruct;   
  VOID *buffer;
  {
#if MAC_MPW
#pragma unused(buffer)
#endif
   DATA_OBJECT result;
   struct deffacts *theDeffacts = (struct deffacts *) theConstruct;
         
   if (theDeffacts->assertList == NULL) return;
   
   SetEvaluationError(CLIPS_FALSE);
   EvaluateExpression(theDeffacts->assertList,&result);
  }

/**********************************************************/
/* ClearDeffacts: Deffacts clear routine for use with the */
/*   clear command. Creates the initial-facts deffacts.   */
/**********************************************************/
static VOID ClearDeffacts()
  {
#if (! RUN_TIME) && (! BLOAD_ONLY)
   struct expr *stub;
   struct deffacts *newDeffacts;

   stub = GenConstant(FCALL,FindFunction("assert"));
   stub->argList = GenConstant(DEFTEMPLATE_PTR,FindDeftemplate("initial-fact"));
   ExpressionInstall(stub);

   newDeffacts = get_struct(deffacts);
   newDeffacts->header.whichModule = 
      (struct defmoduleItemHeader *) GetDeffactsModuleItem(NULL);
   newDeffacts->header.name = (SYMBOL_HN *) AddSymbol("initial-fact");
   IncrementSymbolCount(newDeffacts->header.name);
   newDeffacts->assertList = PackExpression(stub);
   newDeffacts->header.next = NULL;
   newDeffacts->header.ppForm = NULL;
   ReturnExpression(stub);

   AddConstructToModule(&newDeffacts->header);
#endif
  }  

/***************************************/
/* SaveDeffacts: Deffacts save routine */
/*   for use with the save command.    */
/***************************************/
static VOID SaveDeffacts(logicalName)
  char *logicalName;
  { SaveConstruct(logicalName,DeffactsConstruct); }

/*******************************************/
/* UndeffactsCommand: CLIPS access routine */
/*   for the undeffacts command.           */
/*******************************************/
globle VOID UndeffactsCommand()
  { UndefconstructCommand("undeffacts",DeffactsConstruct); }

/*********************************/
/* Undeffacts: C access routine  */
/*   for the undeffacts command. */
/*********************************/
globle BOOLEAN Undeffacts(theDeffacts)
  VOID *theDeffacts;
  { return(Undefconstruct(theDeffacts,DeffactsConstruct)); }
 
/*************************************************/
/* GetDeffactsListFunction: CLIPS access routine */
/*   for the get-deffacts-list function.         */
/*************************************************/
globle VOID GetDeffactsListFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  { GetConstructListFunction("get-deffacts-list",returnValue,DeffactsConstruct); }

/*****************************************/
/* GetDeffactsList: C access routine for */
/*   the get-deffacts-list function.     */
/*****************************************/
globle VOID GetDeffactsList(returnValue,theModule)
  DATA_OBJECT_PTR returnValue;
  VOID *theModule;
  { GetConstructList(returnValue,DeffactsConstruct,(struct defmodule *) theModule); }

/********************************************/
/* DeffactsModuleFunction: C access routine */
/*   for the deffacts-module function.      */
/********************************************/
globle SYMBOL_HN *DeffactsModuleFunction()
  { return(GetConstructModuleCommand("deffacts-module",DeffactsConstruct)); }

#if DEBUGGING_FUNCTIONS

/*******************************************/
/* PPDeffactsCommand: CLIPS access routine */
/*   for the ppdeffacts command.           */
/*******************************************/
globle VOID PPDeffactsCommand()
  { PPConstructCommand("ppdeffacts",DeffactsConstruct); }

/************************************/
/* PPDeffacts: C access routine for */
/*   the ppdeffacts command.        */
/************************************/
globle int PPDeffacts(deffactsName,logicalName)
  char *deffactsName, *logicalName;
  { return(PPConstruct(deffactsName,logicalName,DeffactsConstruct)); }

/*********************************************/
/* ListDeffactsCommand: CLIPS access routine */
/*   for the list-deffacts command.          */
/*********************************************/
globle VOID ListDeffactsCommand()
  { ListConstructCommand("list-deffacts",DeffactsConstruct); }

/**************************************/
/* ListDeffacts: C access routine for */
/*   the list-deffacts command.       */
/**************************************/
globle VOID ListDeffacts(logicalName,theModule)
  char *logicalName;
  VOID *theModule;
  { ListConstruct(DeffactsConstruct,logicalName,(struct defmodule *) theModule); }

#endif /* DEBUGGING_FUNCTIONS */

#endif /* DEFFACTS_CONSTRUCT */


