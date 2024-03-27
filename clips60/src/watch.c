   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                    WATCH MODULE                     */
   /*******************************************************/
                   
                   
/*************************************************************/
/* Purpose: Support functions for the watch and unwatch      */
/*   commands.                                               */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian Donnell                                        */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _WATCH_SOURCE_

#include "setup.h"

#if DEBUGGING_FUNCTIONS

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "constant.h"
#include "clipsmem.h"
#include "router.h"
#include "argacces.h"
#include "extnfunc.h"
#include "watch.h"

/*************************/
/* STRUCTURE DEFINITIONS */
/*************************/

struct watchItem
  {
   char *name;
   int *flag;
   int code,priority;
#if ANSI_COMPILER
   BOOLEAN (*accessFunc)(int,int,struct expr *);
   BOOLEAN (*printFunc)(char *,int,struct expr *);
#else
   BOOLEAN (*accessFunc)();
   BOOLEAN (*printFunc)();
#endif
   struct watchItem *next;
  };

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/
#if ANSI_COMPILER
static struct watchItem *ValidWatchItem(char *,int *);
static BOOLEAN RecognizeWatchRouters(char *);
static int CaptureWatchPrints(char *,char *);
#else
static struct watchItem *ValidWatchItem();
static BOOLEAN RecognizeWatchRouters();
static int CaptureWatchPrints();
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static struct watchItem      *ListOfWatchItems = NULL;

/*************************************************************/
/* AddWatchItem: Adds an item to the list of watchable items */
/*   that can be set using the watch and unwatch commands.   */
/*   Returns FALSE if the item is already in the list,       */
/*   otherwise returns TRUE.                                 */
/*************************************************************/
globle BOOLEAN AddWatchItem(name,code,flag,priority,accessFunc,printFunc)
  char *name;
  int *flag;
  int code,priority;
#if ANSI_COMPILER
  BOOLEAN (*accessFunc)(int,int,struct expr *);
  BOOLEAN (*printFunc)(char *,int,struct expr *);
#else
  BOOLEAN (*accessFunc)();
  BOOLEAN (*printFunc)();
#endif
  {
   struct watchItem *newPtr, *currentPtr, *lastPtr = NULL;

   /*================================================================*/
   /* Find the insertion point in the watchable items list to place  */
   /* the new item. If the item is already in the list return FALSE. */
   /*================================================================*/
   
   for (currentPtr = ListOfWatchItems, lastPtr = NULL; 
        currentPtr != NULL; 
        currentPtr = currentPtr->next)
     {
      if (strcmp(currentPtr->name,name) == 0) return(CLIPS_FALSE);
      if (priority < currentPtr->priority) lastPtr = currentPtr;
     }
     
   /*============================*/
   /* Create the new watch item. */
   /*============================*/
   
   newPtr = get_struct(watchItem);
   newPtr->name = name;
   newPtr->flag = flag;
   newPtr->code = code;
   newPtr->priority = priority;
   newPtr->accessFunc = accessFunc;
   newPtr->printFunc = printFunc;
   
   /*=================================================*/
   /* Insert the new item in the list of watch items. */
   /*=================================================*/
   
   if (lastPtr == NULL)
     {
      newPtr->next = ListOfWatchItems;
      ListOfWatchItems = newPtr;
     }
   else
     {
      newPtr->next = lastPtr->next;
      lastPtr->next = newPtr;
     }

   /*==================================================*/
   /* Return TRUE to indicate the item has been added. */
   /*==================================================*/
   
   return(CLIPS_TRUE);
  }

/**************************************************/
/* Watch: C access routine for the watch command. */
/**************************************************/
globle BOOLEAN Watch(itemName)
  char *itemName;
  {
   return(SetWatchItem(itemName,ON,NULL));
  }
  
/******************************************************/
/* Unwatch: C access routine for the unwatch command. */
/******************************************************/
globle BOOLEAN Unwatch(itemName)
  char *itemName;
  {
   return(SetWatchItem(itemName,OFF,NULL));
  }
   
/********************************************************************/
/* SetWatchItem: Sets the state of a specified watch item to either */
/*   on or off. Returns TRUE if the item was set, otherwise FALSE.  */
/********************************************************************/
globle BOOLEAN SetWatchItem(itemName,newState,argExprs)
  char *itemName;
  int newState;
  struct expr *argExprs;
  {
   struct watchItem *wPtr;

   /*======================================================*/
   /* If the new state isn't on or off, then return FALSE. */
   /*======================================================*/
   
   if ((newState != ON) && (newState != OFF)) return(CLIPS_FALSE);

   /*===================================================*/
   /* If the name of the watch item to set is all, then */
   /* all watch items are set to the new state and TRUE */
   /* is returned.                                      */
   /*===================================================*/
   
   if (strcmp(itemName,"all") == 0)
     {
      for (wPtr = ListOfWatchItems; wPtr != NULL; wPtr = wPtr->next)
        {
         /* ==============================================
            Set the global flag only if alone is specified
            ============================================== */
         if (argExprs == NULL)
           *(wPtr->flag) = newState;
         if ((wPtr->accessFunc == NULL) ? CLIPS_FALSE :
             ((*wPtr->accessFunc)(wPtr->code,newState,argExprs) == CLIPS_FALSE))
           {
            SetEvaluationError(CLIPS_TRUE);
            return(CLIPS_FALSE);
           }
        }
      return(CLIPS_TRUE);
     }

   /*=================================================*/
   /* Search for the watch item to be set in the list */
   /* of watch items. If found, set the watch item to */
   /* its new state and return TRUE.                  */
   /*=================================================*/
   
   for (wPtr = ListOfWatchItems; wPtr != NULL; wPtr = wPtr->next)
     {
      if (strcmp(itemName,wPtr->name) == 0)
        {
         if (argExprs == NULL)
           *(wPtr->flag) = newState;
         if ((wPtr->accessFunc == NULL) ? CLIPS_FALSE :
             ((*wPtr->accessFunc)(wPtr->code,newState,argExprs) == CLIPS_FALSE))
           {
            SetEvaluationError(CLIPS_TRUE);
            return(CLIPS_FALSE);
           }
         return(CLIPS_TRUE);
        }
     }

   /*=================================================*/
   /* If the specified item was not found in the list */
   /* of watchable items then return FALSE.           */
   /*=================================================*/

   return(CLIPS_FALSE);
  }

/*********************************************************************/
/* GetWatchItem: Gets the current state of the specified watch item. */
/*   Returns the state of the watch item (0 for off and 1 for on) if */
/*   the watch item is found in the list of watch items, otherwise   */
/*   -1 is returned.                                                 */               
/*********************************************************************/
globle int GetWatchItem(itemName)
  char *itemName;
  {
   struct watchItem *wPtr;
   
   for (wPtr = ListOfWatchItems; wPtr != NULL; wPtr = wPtr->next)
     { if (strcmp(itemName,wPtr->name) == 0) return(*(wPtr->flag)); }

   return(-1);
  }
  
/****************************************************************/
/* ValidWatchItem: Returns TRUE if the specified name is found  */
/*   in the list of watch items, otherwise returns FALSE.       */
/****************************************************************/
static struct watchItem *ValidWatchItem(itemName,recognized)
  char *itemName;
  int *recognized;
  {
   struct watchItem *wPtr;
   
   *recognized = CLIPS_TRUE;
   if (strcmp(itemName,"all") == 0) 
     return(NULL); 
   
   for (wPtr = ListOfWatchItems; wPtr != NULL; wPtr = wPtr->next)
     { if (strcmp(itemName,wPtr->name) == 0) return(wPtr); }

   *recognized = CLIPS_FALSE;
   return(NULL);
  }
  
/*************************************************************/
/* GetNthWatchName: Returns the name associated with the nth */
/*   item in the list of watchable items. If the nth item    */
/*   does not exist, then NULL is returned.                  */
/*************************************************************/
globle char *GetNthWatchName(whichItem)
  int whichItem;
  {
   int i;
   struct watchItem *wPtr;

   for (wPtr = ListOfWatchItems, i = 1;
        wPtr != NULL;
        wPtr->next, i++)
     { if (i == whichItem) return(wPtr->name); }

   return(NULL);
  }

/***************************************************************/
/* GetNthWatchValue: Returns the current state associated with */
/*   the nth item in the list of watchable items. If the nth   */
/*   item does not exist, then -1 is returned.                 */
/***************************************************************/
globle int GetNthWatchValue(whichItem)
  int whichItem;
  {
   int i;
   struct watchItem *wPtr;

   for (wPtr = ListOfWatchItems, i = 1;
        wPtr != NULL;
        wPtr->next, i++)
     { if (i == whichItem) return(*(wPtr->flag)); }

   return(-1);
  }
  
/*****************************************************/
/* WatchCommand: Implements the CLIPS watch command. */
/*   Syntax: (watch <watchable-item>)                */
/*****************************************************/
globle VOID WatchCommand()
  {
   DATA_OBJECT theValue;
   char *argument;
   int recognized;
   struct watchItem *wPtr;
   
   /*========================================*/
   /* Determine which item is to be watched. */
   /*========================================*/
   
   if (ArgTypeCheck("watch",1,SYMBOL,&theValue) == CLIPS_FALSE) return;
   argument = DOToString(theValue);
   wPtr = ValidWatchItem(argument,&recognized);
   if (recognized == CLIPS_FALSE)
     {
      SetEvaluationError(CLIPS_TRUE);
      ExpectedTypeError1("watch",1,"watchable symbol");
      return;
     }
     
   /*=================================================*/
   /* Check to make sure extra arguments are allowed. */
   /*=================================================*/
      
   if (GetNextArgument(GetFirstArgument()) != NULL)
     {
      if ((wPtr == NULL) ? CLIPS_TRUE : (wPtr->accessFunc == NULL))
        {
         SetEvaluationError(CLIPS_TRUE);
         ExpectedCountError("watch",EXACTLY,1);
         return;
        }
     }
   
   /*=====================*/
   /* Set the watch item. */
   /*=====================*/
   
   SetWatchItem(argument,ON,GetNextArgument(GetFirstArgument()));
  }

/*********************************************************/
/* UnwatchCommand: Implements the CLIPS unwatch command. */
/*   Syntax: (unwatch <watchable-item>)                  */
/*********************************************************/
globle VOID UnwatchCommand()
  {
   DATA_OBJECT theValue;
   char *argument;
   int recognized;
   struct watchItem *wPtr;

   /*==========================================*/
   /* Determine which item is to be unwatched. */
   /*==========================================*/
   
   if (ArgTypeCheck("unwatch",1,SYMBOL,&theValue) == CLIPS_FALSE) return;
   argument = DOToString(theValue);
   wPtr = ValidWatchItem(argument,&recognized);
   if (recognized == CLIPS_FALSE)
     {
      SetEvaluationError(CLIPS_TRUE);
      ExpectedTypeError1("unwatch",1,"watchable symbol");
      return;
     }
     
   /*=================================================*/
   /* Check to make sure extra arguments are allowed. */
   /*=================================================*/
      
   if (GetNextArgument(GetFirstArgument()) != NULL)
     {
      if ((wPtr == NULL) ? CLIPS_TRUE : (wPtr->accessFunc == NULL))
        {
         SetEvaluationError(CLIPS_TRUE);
         ExpectedCountError("unwatch",EXACTLY,1);
         return;
        }
     }
   
   /*=====================*/
   /* Set the watch item. */
   /*=====================*/
   
   SetWatchItem(argument,OFF,GetNextArgument(GetFirstArgument()));
  }

/******************************************************************/
/* ListWatchItemsCommand: CLIPS command for listing all watchable */
/*   items and their current state (on or off).                   */
/*   Syntax: (list-watch-items)                                   */
/******************************************************************/
globle VOID ListWatchItemsCommand()
  {
   struct watchItem *wPtr;
   DATA_OBJECT theValue;
   int recognized;
   
   /*=======================*/
   /* List the watch items. */
   /*=======================*/
   
   if (GetFirstArgument() == NULL)
     {
      for (wPtr = ListOfWatchItems; wPtr != NULL; wPtr = wPtr->next)
        { 
         PrintCLIPS(WDISPLAY,wPtr->name);
         if (*(wPtr->flag)) PrintCLIPS(WDISPLAY," = on\n");
         else PrintCLIPS(WDISPLAY," = off\n");
        }
      return;
     }
     
   /*==========================================*/
   /* Determine which item is to be unwatched. */
   /*==========================================*/
   
   if (ArgTypeCheck("list-watch-items",1,SYMBOL,&theValue) == CLIPS_FALSE) return;
   wPtr = ValidWatchItem(DOToString(theValue),&recognized);
   if ((recognized == CLIPS_FALSE) || (wPtr == NULL))
     {
      SetEvaluationError(CLIPS_TRUE);
      ExpectedTypeError1("list-watch-items",1,"watchable symbol");
      return;
     }
     
   /* ==============================================
      Check to make sure extra arguments are allowed
      ============================================== */
   if ((wPtr->printFunc == NULL) && 
       (GetNextArgument(GetFirstArgument()) != NULL))
     {
      SetEvaluationError(CLIPS_TRUE);
      ExpectedCountError("list-watch-items",EXACTLY,1);
      return;
     }
   
   PrintCLIPS(WDISPLAY,wPtr->name);
   if (*(wPtr->flag)) PrintCLIPS(WDISPLAY," = on\n");
   else PrintCLIPS(WDISPLAY," = off\n");
   if (wPtr->printFunc != NULL)
     if ((*wPtr->printFunc)(WDISPLAY,wPtr->code,
                            GetNextArgument(GetFirstArgument())) == CLIPS_FALSE)
       SetEvaluationError(CLIPS_TRUE);
  }
  

/*************************************************************/
/* WatchFunctionDefinitions: Initializes the watch commands. */
/*************************************************************/
globle VOID WatchFunctionDefinitions()
  {   
#if ! RUN_TIME
   DefineFunction2("watch",   'v', PTIF WatchCommand,   "WatchCommand", "1**w");
   DefineFunction2("unwatch", 'v', PTIF UnwatchCommand, "UnwatchCommand", "1**w");
   DefineFunction2("list-watch-items", 'v', PTIF ListWatchItemsCommand,
                   "ListWatchItemsCommand", "0**w");
#endif

   AddRouter(WTRACE,1000,RecognizeWatchRouters,CaptureWatchPrints,NULL,NULL,NULL);
   DeactivateRouter(WTRACE);
  }

/**************************************************/
/* RecognizeWatchRouters: Looks for WTRACE prints */
/**************************************************/
static BOOLEAN RecognizeWatchRouters(log)
  char *log;
  {
   if (strcmp(log,WTRACE) == 0)
     return(CLIPS_TRUE);
   return(CLIPS_FALSE);
  }

/**************************************************/
/* CaptureWatchPrints: Suppresses WTRACE messages */
/**************************************************/
#if IBM_TBC
#pragma argsused
#endif
static int CaptureWatchPrints(log,str)
  char *log,*str;
  {
#if MAC_MPW
#pragma unused(log)
#pragma unused(str)
#endif
   return(1);
  }
  
  
#endif
 
