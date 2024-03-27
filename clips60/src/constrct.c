   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00 05/12/93             */
   /*                                                     */
   /*                  CONSTRUCT MODULE                   */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _CONSTRCT_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "setup.h"

#include "constant.h"
#include "clipsmem.h"
#include "router.h"
#include "scanner.h"
#include "watch.h"
#include "prcdrfun.h"
#include "prcdrpsr.h"
#include "argacces.h"
#include "exprnpsr.h"
#include "multifld.h"
#include "moduldef.h"
#include "utility.h"

#include "constrct.h"

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle int                   ClearInProgress = CLIPS_FALSE;
   globle int                   ResetInProgress = CLIPS_FALSE;

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

#if (! RUN_TIME) && (! BLOAD_ONLY)
   static struct callFunctionItem   *ListOfSaveFunctions = NULL;
   static BOOLEAN                    PrintWhileLoading = CLIPS_FALSE;
   static BOOLEAN                    WatchCompilations = ON;
#endif

   static struct construct          *ListOfConstructs = NULL;
   static struct callFunctionItem   *ListOfResetFunctions = NULL;
   static struct callFunctionItem   *ListOfClearFunctions = NULL;
   static struct callFunctionItem   *ListOfClearReadyFunctions = NULL;
   static int                        Executing = CLIPS_FALSE;
#if ANSI_COMPILER
   static int                      (*BeforeResetFunction)(void) = NULL;
#else
   static int                      (*BeforeResetFunction)() = NULL;
#endif

#if (! RUN_TIME) && (! BLOAD_ONLY)

/*****************************************************************************/
/* FindConstruct: Determines whether a construct is in the ListOfConstructs. */
/*****************************************************************************/
globle struct construct *FindConstruct(name)
  char *name;
  {
   struct construct *currentPtr;

   currentPtr = ListOfConstructs;
   while (currentPtr != NULL)
     {
      if (strcmp(name,currentPtr->constructName) == 0)
        { return(currentPtr); }
      currentPtr = currentPtr->next;
     }

   return(NULL);
  }

/***********************************************************/
/* RemoveConstruct: Removes a construct and its associated */
/*   parsing function from the ListOfConstructs.           */
/***********************************************************/
globle int RemoveConstruct(name)
  char *name;
  {
   struct construct *currentPtr, *lastPtr;

   lastPtr = NULL;
   currentPtr = ListOfConstructs;

   while (currentPtr != NULL)
     {
      if (strcmp(name,currentPtr->constructName) == 0)
        {
         if (lastPtr == NULL)
           { ListOfConstructs = currentPtr->next; }
         else
           { lastPtr->next = currentPtr->next; }
         rtn_struct(construct,currentPtr);
         return(1);
        }
      lastPtr = currentPtr;
      currentPtr = currentPtr->next;
     }

   return(0);
  }

/************************************************/
/* Save: C access routine for the save command. */
/************************************************/
globle int Save(fileName)
  char *fileName;
  {
   struct callFunctionItem *saveFunction;
   FILE *filePtr;


   if ((filePtr = fopen(fileName,"w")) == NULL)
     { return(CLIPS_FALSE); }
   SetFastSave(filePtr);

   /*==========================*/
   /* Save unusual constructs. */
   /*==========================*/

   saveFunction = ListOfSaveFunctions;
   while (saveFunction != NULL)
     {
#if ANSI_COMPILER
      ((* (VOID (*)(char *)) saveFunction->func))((char *) filePtr);
#else
      (*saveFunction->func)((char *) filePtr);
#endif

      saveFunction = saveFunction->next;
     }

   fclose(filePtr);
   SetFastSave(NULL);

   return(CLIPS_TRUE);
  }

/************************************************************************/
/* RemoveSaveFunction: Removes a function from the ListOfSaveFunctions. */
/************************************************************************/
globle BOOLEAN RemoveSaveFunction(name)
  char *name;
  {
   int found;

   ListOfSaveFunctions = RemoveFunctionFromCallList(name,ListOfSaveFunctions,&found);

   if (found) return(CLIPS_TRUE);
   
   return(CLIPS_FALSE);
  }

/**************************************************************/
/* SetCompilationsWatch: Sets the value of WatchCompilations. */
/**************************************************************/
globle VOID SetCompilationsWatch(value)
  int value;
  {
   WatchCompilations = value;
  }

/*****************************************************************/
/* GetCompilationsWatch: Returns the value of WatchCompilations. */
/*****************************************************************/
globle BOOLEAN GetCompilationsWatch()
  { return(WatchCompilations); }

/**************************************************************/
/* SetPrintWhileLoading: Sets the value of PrintWhileLoading. */
/**************************************************************/
globle VOID SetPrintWhileLoading(value)
  BOOLEAN value;
  {
   PrintWhileLoading = value;
  }

/*****************************************************************/
/* GetPrintWhileLoading: Returns the value of PrintWhileLoading. */
/*****************************************************************/
globle BOOLEAN GetPrintWhileLoading()
  {
   return(PrintWhileLoading);
  }
#endif

/************************************************************/
/* InitializeConstructs: Initializes the Construct Manager. */
/************************************************************/
globle VOID InitializeConstructs()
  {
#if (! RUN_TIME)
   DefineFunction2("clear",   'v', PTIF ClearCommand,   "ClearCommand", "00");
   DefineFunction2("reset",   'v', PTIF ResetCommand,   "ResetCommand", "00");
#endif

#if DEBUGGING_FUNCTIONS && (! RUN_TIME) && (! BLOAD_ONLY)
   AddWatchItem("compilations",0,&WatchCompilations,30,NULL,NULL);
#endif
  }
  
/**************************************/
/* ClearCommand: CLIPS access routine */
/*   for the clear command.           */
/**************************************/
globle VOID ClearCommand()
  {
   if (ArgCountCheck("clear",EXACTLY,0) == -1) return;
   Clear();
   return;
  }

/**************************************/
/* ResetCommand: CLIPS access routine */
/*   for the reset command.           */
/**************************************/
globle VOID ResetCommand()
  {
   if (ArgCountCheck("reset",EXACTLY,0) == -1) return;
   Reset();
   return;
  }

/****************************/
/* Reset: C access routine  */
/*   for the reset command. */
/****************************/
globle VOID Reset()
  {
   struct callFunctionItem *resetPtr;

   if (ResetInProgress) return;

   ResetInProgress = CLIPS_TRUE;
   
   if (CurrentEvaluationDepth == 0) SetHaltExecution(CLIPS_FALSE);

   if ((BeforeResetFunction != NULL) ? ((*BeforeResetFunction)() == CLIPS_FALSE) : 
                                       CLIPS_FALSE)
     {
      ResetInProgress = CLIPS_FALSE;
      return;
     }
   
   resetPtr = ListOfResetFunctions;
   while ((resetPtr != NULL) && (GetHaltExecution() == CLIPS_FALSE))
     {
      (*resetPtr->func)();
      resetPtr = resetPtr->next;
     }

   SetCurrentModule((VOID *) FindDefmodule("MAIN"));
   ResetInProgress = CLIPS_FALSE;
  }

/******************************************************************/
/* SetBeforeResetFunction: Sets the value of BeforeResetFunction. */
/******************************************************************/
globle int (*SetBeforeResetFunction(theFunction))(VOID_ARG)
  int (*theFunction)(VOID_ARG);
  {
   int (*tempFunction)(VOID_ARG);

   tempFunction = BeforeResetFunction;
   BeforeResetFunction = theFunction;
   return(tempFunction);
  }

/**************************************************************/
/* AddResetFunction: Adds a function to ListOfResetFunctions. */
/**************************************************************/
globle BOOLEAN AddResetFunction(name,functionPtr,priority)
  char *name;
#if ANSI_COMPILER
  VOID (*functionPtr)(void);
#else
  VOID (*functionPtr)();
#endif
  int priority;
  {
   ListOfResetFunctions = AddFunctionToCallList(name,priority,
                                                functionPtr,
                                                ListOfResetFunctions);
   return(1);
  }

/**************************************************************************/
/* RemoveResetFunction: Removes a function from the ListOfResetFunctions. */
/**************************************************************************/
globle BOOLEAN RemoveResetFunction(name)
  char *name;
  {
   int found;

   ListOfResetFunctions = RemoveFunctionFromCallList(name,ListOfResetFunctions,&found);

   if (found) return(CLIPS_TRUE);
   
   return(CLIPS_FALSE);
  }

/**************************************************/
/* Clear: C access routine for the clear command. */
/**************************************************/
globle VOID Clear()
  {
   struct callFunctionItem *theFunction;
   
   /*===================================*/
   /* Calls all before clear functions. */
   /*===================================*/

#if DEBUGGING_FUNCTIONS
   ActivateRouter(WTRACE);
#endif

   if (ClearReady() == CLIPS_FALSE)
     {
      PrintErrorID("CONSTRCT",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Some constructs are still in use. Clear cannot continue.\n");
#if DEBUGGING_FUNCTIONS
      DeactivateRouter(WTRACE);
#endif
      return;
     }

   ClearInProgress = CLIPS_TRUE;
   
   /*======================================*/
   /* Don't watch anything during a clear. */
   /*======================================*/

   /*============================*/
   /* Calls all clear functions. */
   /*============================*/

   theFunction = ListOfClearFunctions;
   while (theFunction != NULL)
     {
      (*theFunction->func)();
      theFunction = theFunction->next;
     }

   /*=======================*/
   /* Restore watch values. */
   /*=======================*/

#if DEBUGGING_FUNCTIONS
   DeactivateRouter(WTRACE);
#endif
      
   ClearInProgress = CLIPS_FALSE;
  }

/*******************************************************************/
/* ClearReady: Returns TRUE if a clear can be performed, otherwise */
/*   FALSE. Note that this is destructively determined (e.g. facts */
/*   will be deleted as part of the determination).                */ 
/*******************************************************************/
globle BOOLEAN ClearReady()
  {
   struct callFunctionItem *theFunction;
   
   theFunction = ListOfClearReadyFunctions;
   while (theFunction != NULL)
     {
      if ((* ((int (*)(VOID_ARG)) *theFunction->func))() == CLIPS_FALSE)
        { return(CLIPS_FALSE); }
      theFunction = theFunction->next;
     }
   return(CLIPS_TRUE);
  }

/************************************************************************/
/* AddClearReadyFunction: Adds a function to ListOfClearReadyFunctions. */
/************************************************************************/
globle BOOLEAN AddClearReadyFunction(name,functionPtr,priority)
  char *name;
#if ANSI_COMPILER
  int (*functionPtr)(void);
#else
  int (*functionPtr)();
#endif
  int priority;
  {
   ListOfClearReadyFunctions = AddFunctionToCallList(name,priority,
#if ANSI_COMPILER
                                                (VOID (*)(void)) functionPtr,
#else
                                                (VOID (*)()) functionPtr,
#endif
                                                ListOfClearReadyFunctions);
   return(1);
  }

/************************************************************************************/
/* RemoveClearReadyFunction: Removes a function from the ListOfClearReadyFunctions. */
/************************************************************************************/
globle BOOLEAN RemoveClearReadyFunction(name)
  char *name;
  {
   int found;

   ListOfClearReadyFunctions = RemoveFunctionFromCallList(name,ListOfClearReadyFunctions,&found);

   if (found) return(CLIPS_TRUE);
   
   return(CLIPS_FALSE);
  }

/**************************************************************/
/* AddClearFunction: Adds a function to ListOfClearFunctions. */
/**************************************************************/
globle BOOLEAN AddClearFunction(name,functionPtr,priority)
  char *name;
#if ANSI_COMPILER
  VOID (*functionPtr)(void);
#else
  VOID (*functionPtr)();
#endif
  int priority;
  {
   ListOfClearFunctions = AddFunctionToCallList(name,priority,
#if ANSI_COMPILER
                                                (VOID (*)(void)) functionPtr,
#else
                                                (VOID (*)()) functionPtr,
#endif
                                                ListOfClearFunctions);
   return(1);
  }

/**************************************************************************/
/* RemoveClearFunction: Removes a function from the ListOfClearFunctions. */
/**************************************************************************/
globle BOOLEAN RemoveClearFunction(name)
  char *name;
  {
   int found;

   ListOfClearFunctions = RemoveFunctionFromCallList(name,ListOfClearFunctions,&found);

   if (found) return(CLIPS_TRUE);
   
   return(CLIPS_FALSE);
  }

/************************************************************/
/* ExecutingConstruct: Returns CLIPS_TRUE if a construct is */
/*   currently being executed, otherwise CLIPS_FALSE.       */
/************************************************************/
globle int ExecutingConstruct()
  { return(Executing); }

/************************************************************/
/* SetExecutingConstruct: Sets the value of the executing   */
/*   variable indicating that actions such as reset, clear, */
/*   etc should not be performed.                           */
/************************************************************/
globle VOID SetExecutingConstruct(value)
  int value;
  {
   Executing = value;
  }

/******************************************************************/
/* OldGetConstructList: Returns a list of all the construct names in */
/*   a multifield value. Does not check number of arguments;      */
/*   assumes restriction string in DefineFunction2 call was "00"  */
/******************************************************************/
globle VOID OldGetConstructList(returnValue,nextFunction,nameFunction)
  DATA_OBJECT_PTR returnValue;
#if ANSI_COMPILER
  VOID *(*nextFunction)(VOID *);
  char *(*nameFunction)(VOID *);
#else
  VOID *(*nextFunction)();
  char *(*nameFunction)();
#endif
  {
   VOID *theConstruct;
   long count = 0;
   struct multifield *theList;

   theConstruct = NULL;
   while ((theConstruct = (*nextFunction)(theConstruct)) != NULL) count++;
   
   SetpType(returnValue,MULTIFIELD);
   SetpDOBegin(returnValue,1);
   SetpDOEnd(returnValue,count);
   theList = (struct multifield *) CreateMultifield((int) count);
   SetpValue(returnValue,(VOID *) theList);
   
   count = 1;
   theConstruct = NULL;
   while ((theConstruct = (*nextFunction)(theConstruct)) != NULL) 
     {
      if (HaltExecution == CLIPS_TRUE)
        {
         SetMultifieldErrorValue(returnValue);
         return;
        }
      SetMFType(theList,count,SYMBOL);
      SetMFValue(theList,count,AddSymbol((*nameFunction)(theConstruct)));
      count++;
     }
  }
  
/******************************************************************/
/* DeinstallConstructHeader: */
/******************************************************************/
globle VOID DeinstallConstructHeader(theHeader)
  struct constructHeader *theHeader;
  {   
   DecrementSymbolCount(theHeader->name);
   if (theHeader->ppForm != NULL)
     {
      rm(theHeader->ppForm,
         (int) sizeof(char) * ((int) strlen(theHeader->ppForm) + 1));
      theHeader->ppForm = NULL;
     }
  }

/*************************************************************/
/* AddConstruct: Adds a construct and its associated parsing */
/*   function to the ListOfConstructs.                       */
/*************************************************************/
globle struct construct *AddConstruct(name,pluralName,
                                      parseFunction,findFunction,
                                      getConstructNameFunction,getPPFormFunction,
                                      getModuleItemFunction,getNextItemFunction,
                                      setNextItemFunction,isConstructDeletableFunction,
                                      deleteFunction,freeFunction)
  char *name, *pluralName;
#if ANSI_COMPILER
  int (*parseFunction)(char *);
  VOID *(*findFunction)(char *);
  SYMBOL_HN *(*getConstructNameFunction)(struct constructHeader *);
  char *(*getPPFormFunction)(struct constructHeader *);
  struct defmoduleItemHeader *(*getModuleItemFunction)(struct constructHeader *);
  VOID *(*getNextItemFunction)(VOID *);
  VOID (*setNextItemFunction)(struct constructHeader *,struct constructHeader *);
  BOOLEAN (*isConstructDeletableFunction)(VOID *);
  int (*deleteFunction)(VOID *);
  VOID (*freeFunction)(VOID *);
#else
  int (*parseFunction)();
  VOID *(*findFunction)();
  SYMBOL_HN *(*getConstructNameFunction)();
  char *(*getPPFormFunction)();
  struct defmoduleItemHeader *(*getModuleItemFunction)();
  VOID *(*getNextItemFunction)();
  VOID (*setNextItemFunction)();
  BOOLEAN (*isConstructDeletableFunction)();
  int (*deleteFunction)();
  VOID (*freeFunction)();
#endif
  {
   struct construct *newPtr;

   newPtr = get_struct(construct);

   newPtr->constructName = name;
   newPtr->pluralName = pluralName;
   newPtr->parseFunction = parseFunction;
   newPtr->findFunction = findFunction;
   newPtr->getConstructNameFunction = getConstructNameFunction;
   newPtr->getPPFormFunction = getPPFormFunction;
   newPtr->getModuleItemFunction = getModuleItemFunction;
   newPtr->getNextItemFunction = getNextItemFunction;
   newPtr->setNextItemFunction = setNextItemFunction;
   newPtr->isConstructDeletableFunction = isConstructDeletableFunction;
   newPtr->deleteFunction = deleteFunction;
   newPtr->freeFunction = freeFunction;   
   
   /*============================================================*/
   /* Add the construct to the list of constructs and return it. */
   /*============================================================*/
   
   newPtr->next = ListOfConstructs;
   ListOfConstructs = newPtr;
   return(newPtr);
  }
  
/****************************************************************/
/* AddSaveFunction: Adds a function to the ListOfSaveFunctions. */
/****************************************************************/
globle BOOLEAN AddSaveFunction(name,functionPtr,priority)
  char *name;
#if ANSI_COMPILER
  VOID (*functionPtr)(char *);
#else
  VOID (*functionPtr)();
#endif
  int priority;
  {
#if (! RUN_TIME) && (! BLOAD_ONLY)
   ListOfSaveFunctions = AddFunctionToCallList(name,priority,
#if ANSI_COMPILER
                                                (VOID (*)(void)) functionPtr,
#else
                                                (VOID (*)()) functionPtr,
#endif
                                                ListOfSaveFunctions);
#endif
   return(1);
  }
