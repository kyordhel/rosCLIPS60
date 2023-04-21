   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                  DEFMODULE MODULE                   */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
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

#define _MODULDEF_SOURCE_

#include "setup.h"

#include <stdio.h>
#include <string.h>
#define _CLIPS_STDIO_

#include "clipsmem.h"
#include "constant.h"
#include "router.h"
#include "extnfunc.h"
#include "argacces.h"
#include "constrct.h"
#include "modulpsr.h"
#include "modulcmp.h"
#include "modulbsc.h"
#include "utility.h"

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
#include "bload.h"
#include "modulbin.h"
#endif

#include "moduldef.h"

typedef struct moduleStackItem
  {
   BOOLEAN changeFlag;
   struct defmodule *theModule;
   struct moduleStackItem *next;
  } MODULE_STACK_ITEM;

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static struct moduleItem           *LastModuleItem = NULL;
   static struct callFunctionItem     *AfterModuleChangeFunctions = NULL;
   static MODULE_STACK_ITEM           *ModuleStack = NULL;
   static BOOLEAN                      CallModuleChangeFunctions = CLIPS_TRUE;

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct defmodule            *ListOfDefmodules = NULL;
   globle struct defmodule            *CurrentModule = NULL;
   globle struct defmodule            *LastDefmodule = NULL;
   globle int                          NumberOfModuleItems = 0;
   globle struct moduleItem           *ListOfModuleItems = NULL;
   globle long                         ModuleChangeIndex = 0;
   globle int                          MainModuleRedefinable = CLIPS_TRUE;

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
#if (! RUN_TIME)
   static VOID                       ReturnDefmodule(struct defmodule *);
#endif
#else
#if (! RUN_TIME)
   static VOID                       ReturnDefmodule();
#endif
#endif

/**************************************************************/
/* InitializeDefmodules: Initializes the defmodule construct. */
/**************************************************************/
globle VOID InitializeDefmodules()
  {
   DefmoduleBasicCommands();
  
#if (! RUN_TIME)
   CreateMainModule();
#endif

#if DEFMODULE_CONSTRUCT && (! RUN_TIME) && (! BLOAD_ONLY)
   AddConstruct("defmodule","defmodules",ParseDefmodule,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL);
#endif

#if (! RUN_TIME) && DEFMODULE_CONSTRUCT
   DefineFunction2("get-current-module", 'w', 
                   PTIF GetCurrentModuleCommand,  
                   "GetCurrentModuleCommand", "00");

   DefineFunction2("set-current-module", 'w',
                   PTIF SetCurrentModuleCommand,
                   "SetCurrentModuleCommand", "11w");
#endif
  }
  
/******************************************************/
/* RegisterModuleItem: Called to register a construct */
/*   which can be placed within a module.             */
/******************************************************/
globle int RegisterModuleItem(theItem,allocateFunction,freeFunction,
                              bloadModuleReference,constructsToCModuleReference,
                              findFunction)
   char *theItem;
#if ANSI_COMPILER
   VOID *(*allocateFunction)(void);
   VOID (*freeFunction)(VOID *);
   VOID *(*bloadModuleReference)(int);
   VOID  (*constructsToCModuleReference)(FILE *,int,int,int);
   VOID *(*findFunction)(char *);
#else
   VOID *(*allocateFunction)();
   VOID (*freeFunction)();
   VOID *(*bloadModuleReference)();
   VOID  (*constructsToCModuleReference)();
   VOID *(*findFunction)();
#endif
  {   
   struct moduleItem *newModuleItem;
  
   newModuleItem = get_struct(moduleItem);
   newModuleItem->name = theItem;
   newModuleItem->allocateFunction = allocateFunction;
   newModuleItem->freeFunction = freeFunction;
   newModuleItem->bloadModuleReference = bloadModuleReference;
   newModuleItem->constructsToCModuleReference = constructsToCModuleReference;
   newModuleItem->findFunction = findFunction;
   newModuleItem->moduleIndex = NumberOfModuleItems++;
   newModuleItem->next = NULL;
   
   if (LastModuleItem == NULL)
     {
      ListOfModuleItems = newModuleItem;
      LastModuleItem = newModuleItem;
     }
   else
     {
      LastModuleItem->next = newModuleItem;
      LastModuleItem = newModuleItem;
     }

   return(newModuleItem->moduleIndex);
  }
  
/***************************************************************/
/* GetListOfModuleItems: Returns the number of module items. */
/***************************************************************/
globle struct moduleItem *GetListOfModuleItems()
  {   
   return (ListOfModuleItems);
  }
   
/***************************************************************/
/* GetNumberOfModuleItems: Returns the number of module items. */
/***************************************************************/
globle int GetNumberOfModuleItems()
  {   
   return (NumberOfModuleItems);
  }
  
/********************************************************/
/* FindModuleItem: Finds the module item data structure */
/*   corresponding to the specified name.               */
/********************************************************/
globle struct moduleItem *FindModuleItem(theName)
  char *theName;
  {   
   struct moduleItem *theModuleItem;
   
   for (theModuleItem = ListOfModuleItems;
        theModuleItem != NULL;
        theModuleItem = theModuleItem->next)
     { if (strcmp(theModuleItem->name,theName) == 0) return(theModuleItem); }
     
   return(NULL);
  }
    
/**************************************************************/
/* GetCurrentModule: Returns a pointer to the current module. */
/**************************************************************/
globle VOID *GetCurrentModule()
  {   
   return ((VOID *) CurrentModule);
  }

/***********************************************************/
/* SetCurrentModule: Sets the value of the current module. */
/***********************************************************/
globle VOID *SetCurrentModule(xNewValue)
  VOID *xNewValue;
  {
   struct defmodule *newValue = (struct defmodule *) xNewValue;
   struct callFunctionItem *changeFunctions;
   VOID *rv;
   
   rv = (VOID *) CurrentModule;
   CurrentModule = newValue;
   
   /*=============================================*/
   /* The module change functions should only be  */
   /* called if this is a "real" module change.   */
   /* Many routines temporarily change the module */
   /* to look for constructs, etc.                */
   /*=============================================*/
   
   if (CallModuleChangeFunctions)
     {
      ModuleChangeIndex++;
      changeFunctions = AfterModuleChangeFunctions;
      while (changeFunctions != NULL)
        {
         (* (VOID (*)(VOID_ARG)) changeFunctions->func)();
         changeFunctions = changeFunctions->next;
        }
     }
     
   return(rv);
  }
  
/********************************************************/
/* SaveCurrentModule: Saves current module on stack and */
/*   prevents SetCurrentModule() from calling change    */
/*   functions                                          */
/********************************************************/
globle VOID SaveCurrentModule()
  {
   MODULE_STACK_ITEM *tmp;
   
   tmp = get_struct(moduleStackItem);
   tmp->changeFlag = CallModuleChangeFunctions;
   CallModuleChangeFunctions = CLIPS_FALSE;
   tmp->theModule = CurrentModule;
   tmp->next = ModuleStack;
   ModuleStack = tmp;
  }
  
/**********************************************************/
/* RestoreCurrentModule: Restores saved module and resets */
/*   ability of SetCurrentModule() to call changed        */
/*   functions to previous state                          */
/**********************************************************/
globle VOID RestoreCurrentModule()
  {
   MODULE_STACK_ITEM *tmp;

   tmp = ModuleStack;
   ModuleStack = tmp->next;
   CallModuleChangeFunctions = tmp->changeFlag;
   CurrentModule = tmp->theModule;
   rtn_struct(moduleStackItem,tmp);
  }
  
/*************************************************************/
/* GetModuleItem: Returns the data pointer for the specified */
/*   module item in the specified module. If no module is    */
/*   indicated, then the module item for the current module  */
/*   is returned.                                            */
/*************************************************************/
globle VOID *GetModuleItem(theModule,moduleItemIndex)
  struct defmodule *theModule;
  int moduleItemIndex;
  {   
   if (theModule == NULL)
     {
      if (CurrentModule == NULL) return(NULL);
      theModule = CurrentModule;
     }
     
   if (theModule->itemsArray == NULL) return (NULL);
   return ((VOID *) theModule->itemsArray[moduleItemIndex]);
  }
  
/************************************************************/
/* SetModuleItem: Sets the data pointer for the specified   */
/*   module item in the specified module. If no module is   */
/*   indicated, then the module item for the current module */
/*   is returned.                                           */
/************************************************************/
globle VOID SetModuleItem(theModule,moduleItemIndex,newValue)
  struct defmodule *theModule;
  int moduleItemIndex;
  VOID *newValue;
  {   
   if (theModule == NULL)
     {
      if (CurrentModule == NULL) return;
      theModule = CurrentModule;
     }
     
   if (theModule->itemsArray == NULL) return;
   theModule->itemsArray[moduleItemIndex] = (struct defmoduleItemHeader *) newValue;
  }
     
/******************************************************/
/* CreateMainModule: Creates the default MAIN module. */
/******************************************************/
globle VOID CreateMainModule()
  {
   struct defmodule *newDefmodule;
   struct moduleItem *theItem;
   int i;
   struct defmoduleItemHeader *theHeader;

   newDefmodule = get_struct(defmodule);
   newDefmodule->name = (SYMBOL_HN *) AddSymbol("MAIN");
   IncrementSymbolCount(newDefmodule->name);
   
   if (NumberOfModuleItems == 0) newDefmodule->itemsArray = NULL;
   else 
     {
      newDefmodule->itemsArray = (struct defmoduleItemHeader **) gm2((int) sizeof(VOID *) * NumberOfModuleItems);
      for (i = 0, theItem = ListOfModuleItems;
           i < NumberOfModuleItems, theItem != NULL; 
           i++, theItem = theItem->next)
        { 
         if (theItem->allocateFunction == NULL)
           { newDefmodule->itemsArray[i] = NULL; }
         else
           { 
            newDefmodule->itemsArray[i] = (struct defmoduleItemHeader *)
                                          (*theItem->allocateFunction)(); 
            theHeader = (struct defmoduleItemHeader *) newDefmodule->itemsArray[i];
            theHeader->theModule = newDefmodule;
            theHeader->firstItem = NULL;
            theHeader->lastItem = NULL;
           }
        }
     }
   
   newDefmodule->next = NULL;
   newDefmodule->ppForm = NULL;
   newDefmodule->importList = NULL;
   newDefmodule->exportList = NULL;
   newDefmodule->bsaveID = 0L;
   
#if (! BLOAD_ONLY) && (! RUN_TIME) && DEFMODULE_CONSTRUCT
   SetNumberOfDefmodules(1L);
#endif

   LastDefmodule = newDefmodule;
   ListOfDefmodules = newDefmodule;
   SetCurrentModule((VOID *) newDefmodule);
  }

/*********************************************************************/
/* SetListOfDefmodules: Sets the list of defmodules to the specified */
/*   value. Normally used when initializing a run-time module or     */
/*   when bloading a binary file to install the list of defmodules.  */
/*********************************************************************/
globle VOID SetListOfDefmodules(defmodulePtr)
  VOID *defmodulePtr;
  {
   ListOfDefmodules = (struct defmodule *) defmodulePtr;
   
   LastDefmodule = ListOfDefmodules;
   if (LastDefmodule == NULL) return;
   while (LastDefmodule->next != NULL) LastDefmodule = LastDefmodule->next;
  }
  
/*******************************************************************/
/* GetNextDefmodule: If passed a NULL pointer, returns the first   */
/*   defmodule in the ListOfDefmodules. Otherwise returns the next */
/*   defmodule following the defmodule passed as an argument.      */
/*******************************************************************/
globle VOID *GetNextDefmodule(defmodulePtr)
  VOID *defmodulePtr;
  {
   if (defmodulePtr == NULL)
     { return((VOID *) ListOfDefmodules); }
   else
     { return((VOID *) (((struct defmodule *) defmodulePtr)->next)); }
  }

/******************************************************************/
/* GetDefmoduleName: Returns the name of the specified defmodule. */
/******************************************************************/
globle char *GetDefmoduleName(defmodulePtr)
  VOID *defmodulePtr;
  { return(ValueToString(((struct defmodule *) defmodulePtr)->name)); }
  
/************************************************/
/* GetDefmodulePPForm: Returns the pretty print */
/*   representation of the specified defmodule. */
/************************************************/
globle char *GetDefmodulePPForm(defmodulePtr)
  VOID *defmodulePtr;
  { return(((struct defmodule *) defmodulePtr)->ppForm); }
  
#if (! RUN_TIME)
  
/*****************************************************************************/
/* RemoveAllDefmodules: Removes all defmodules from the current environment. */
/*****************************************************************************/
globle VOID RemoveAllDefmodules()
  {
   struct defmodule *nextDefmodule;
     
   while (ListOfDefmodules != NULL)
     {
      nextDefmodule = ListOfDefmodules->next;
      ReturnDefmodule(ListOfDefmodules);
      ListOfDefmodules = nextDefmodule;
     }
   
   CurrentModule = NULL;
   LastDefmodule = NULL;
  }
      
/************************************************************/
/* ReturnDefmodule: Returns the data structures associated  */
/*   with a defmodule construct to the pool of free memory. */
/************************************************************/
static VOID ReturnDefmodule(theDefmodule)
  struct defmodule *theDefmodule;
  {
   int i;
   struct moduleItem *theItem;
   struct portItem *theSpec, *nextSpec;
   
   if (theDefmodule == NULL) return;

   SetCurrentModule((VOID *) theDefmodule);
   
   if (theDefmodule->itemsArray != NULL)
     {    
      for (i = 0, theItem = ListOfModuleItems;
           i < NumberOfModuleItems, theItem != NULL; 
           i++, theItem = theItem->next)
        { 
         if (theItem->freeFunction != NULL)
           { (*theItem->freeFunction)(theDefmodule->itemsArray[i]); }
        }

      rm(theDefmodule->itemsArray,(int) sizeof(VOID *) * NumberOfModuleItems);
    }
        
   DecrementSymbolCount(theDefmodule->name);
   
   theSpec = theDefmodule->importList;
   while (theSpec != NULL)
     { 
      nextSpec = theSpec->next;
      if (theSpec->moduleName != NULL) DecrementSymbolCount(theSpec->moduleName);
      if (theSpec->constructType != NULL) DecrementSymbolCount(theSpec->constructType);
      if (theSpec->constructName != NULL) DecrementSymbolCount(theSpec->constructName);
      rtn_struct(portItem,theSpec);
      theSpec = nextSpec;
     }

   theSpec = theDefmodule->exportList;
   while (theSpec != NULL)
     { 
      nextSpec = theSpec->next;
      if (theSpec->moduleName != NULL) DecrementSymbolCount(theSpec->moduleName);
      if (theSpec->constructType != NULL) DecrementSymbolCount(theSpec->constructType);
      if (theSpec->constructName != NULL) DecrementSymbolCount(theSpec->constructName);
      rtn_struct(portItem,theSpec);
      theSpec = nextSpec;
     }
     
   if (theDefmodule->ppForm != NULL)
     {
      rm(theDefmodule->ppForm,
         (int) sizeof(char) * ((int) strlen(theDefmodule->ppForm) + 1));
     }
        
   rtn_struct(defmodule,theDefmodule);
  }

#endif

/**********************************************************************/
/* FindDefmodule: Searches for a defmodule in the list of defmodules. */
/*   Returns a pointer to the defmodule if found, otherwise NULL.     */
/**********************************************************************/
globle VOID *FindDefmodule(defmoduleName)
  char *defmoduleName;
  {
   struct defmodule *defmodulePtr;
   SYMBOL_HN *findValue;

   if ((findValue = (SYMBOL_HN *) FindSymbol(defmoduleName)) == NULL) return(NULL);

   defmodulePtr = ListOfDefmodules;
   while (defmodulePtr != NULL)
     {
      if (defmodulePtr->name == findValue)
        { return((VOID *) defmodulePtr); }

      defmodulePtr = defmodulePtr->next;
     }

   return(NULL);
  }
  
/*******************************************/
/* GetCurrentModuleCommand: Implements the */
/*   CLIPS get-current-module command.     */
/*******************************************/
globle SYMBOL_HN *GetCurrentModuleCommand()
  {
   struct defmodule *theModule;
   
   ArgCountCheck("get-current-module",EXACTLY,0);
   
   theModule = (struct defmodule *) GetCurrentModule();
   
   if (theModule == NULL) return((SYMBOL_HN *) CLIPSFalseSymbol);
   
   return((SYMBOL_HN *) AddSymbol(ValueToString(theModule->name)));
  }

/*******************************************/
/* SetCurrentModuleCommand: Implements the */
/*   CLIPS set-current-module command.     */
/*******************************************/
globle SYMBOL_HN *SetCurrentModuleCommand()
  {
   DATA_OBJECT argPtr;
   char *argument;
   struct defmodule *theModule;
   SYMBOL_HN *defaultReturn;

   /*=====================================================*/
   /* Check for the correct number and type of arguments. */
   /*=====================================================*/
   
   theModule = ((struct defmodule *) GetCurrentModule());
   if (theModule == NULL) return((SYMBOL_HN *) CLIPSFalseSymbol);
   
   defaultReturn = (SYMBOL_HN *) AddSymbol(ValueToString(((struct defmodule *) GetCurrentModule())->name));
   
   if (ArgCountCheck("set-current-module",EXACTLY,1) == -1)
     { return(defaultReturn); }

   if (ArgTypeCheck("set-current-module",1,SYMBOL,&argPtr) == CLIPS_FALSE)
     { return(defaultReturn); }

   argument = DOToString(argPtr);

   /*================================================*/
   /* Set the current module to the specified value. */
   /*================================================*/
   
   theModule = (struct defmodule *) FindDefmodule(argument);
   
   if (theModule == NULL)
     {
      CantFindItemErrorMessage("defmodule",argument);
      return(defaultReturn);
     }
     
   SetCurrentModule((VOID *) theModule);
   
   /*================================*/
   /* Return the new current module. */
   /*================================*/
   
   return((SYMBOL_HN *) defaultReturn);
  }
  
/**********************************************************/
/* AddAfterModuleChangeFunction:                          */
/**********************************************************/
globle VOID AddAfterModuleChangeFunction(name,func,priority)
  char *name;
  VOID (*func)(VOID_ARG);
  int priority;
  {
   AfterModuleChangeFunctions = 
     AddFunctionToCallList(name,priority,func,AfterModuleChangeFunctions);
  }    

/**********************************************************/
/* IllegalModuleSpecifierMessage:                          */
/**********************************************************/
globle VOID IllegalModuleSpecifierMessage()
  {
   PrintErrorID("MODULDEF",1,CLIPS_TRUE);
   PrintCLIPS(WERROR,"Illegal use of the module specifier.\n");
  }    
  

