   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00 05/12/93             */
   /*                                                     */
   /*              CONSTRUCT COMMANDS MODULE              */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _CSTRCCOM_SOURCE_

#include <string.h>

#include "setup.h"

#include "constant.h"
#include "clipsmem.h"
#include "moduldef.h"
#include "argacces.h"
#include "multifld.h"
#include "modulutl.h"
#include "router.h"

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
#include "bload.h"
#endif

#if (! BLOAD_ONLY) && (! RUN_TIME)
#include "cstrcpsr.h"
#endif

#include "cstrccom.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER

#if DEBUGGING_FUNCTIONS
   static VOID                    ConstructPrintWatch(char *,struct construct *,VOID *,
                                                      BOOLEAN (*)(VOID *));
   static BOOLEAN                 ConstructWatchSupport(struct construct *,char *,
                                                        char *,EXPRESSION *,BOOLEAN,
                                                        BOOLEAN,BOOLEAN (*)(VOID *),
                                                        VOID (*)(BOOLEAN,VOID *));
#endif

#else

#if DEBUGGING_FUNCTIONS
   static VOID                    ConstructPrintWatch();
   static BOOLEAN                 ConstructWatchSupport();
#endif

#endif

#if (! RUN_TIME)

/*****************************************************************/
/* AddConstructToModule: Adds a construct to the current module. */
/*****************************************************************/
globle VOID AddConstructToModule(theConstruct)
  struct constructHeader *theConstruct;
  {
   if (theConstruct->whichModule->lastItem == NULL)
     { theConstruct->whichModule->firstItem = theConstruct; }
   else
     { theConstruct->whichModule->lastItem->next = theConstruct; }
   
   theConstruct->whichModule->lastItem = theConstruct;
   theConstruct->next = NULL;
  }

#endif

/****************************************************/
/* DeleteNamedConstruct: Generic driver routine for */
/*   deleting a specific construct from a module.   */
/****************************************************/
globle BOOLEAN DeleteNamedConstruct(constructName,constructClass)
  char *constructName;
  struct construct *constructClass;
  {
#if (! BLOAD_ONLY)
   VOID *constructPtr;

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
   if (Bloaded() == CLIPS_TRUE) return(CLIPS_FALSE);
#endif

   constructPtr = (*constructClass->findFunction)(constructName);

   if (constructPtr != NULL)
     { return((*constructClass->deleteFunction)(constructPtr)); }
   else
     {
      if (strcmp("*",constructName) == 0)
        {
         (*constructClass->deleteFunction)(NULL);
         return(CLIPS_TRUE);
        }

      return(CLIPS_FALSE);
     }
#else
   return(CLIPS_FALSE);
#endif
  }

/*****************************************************/
/* FindNamedConstruct: Generic routine for searching */
/*   for a specified construct.                      */
/*****************************************************/
globle VOID *FindNamedConstruct(constructName,constructClass)
  char *constructName;
  struct construct *constructClass;
  {
   VOID *theConstruct;
   SYMBOL_HN *findValue; 
   
   SaveCurrentModule();
     
   constructName = ExtractModuleAndConstructName(constructName);

   if ((constructName == NULL) ?
       CLIPS_TRUE :
       ((findValue = (SYMBOL_HN *) FindSymbol(constructName)) == NULL))
     {
      RestoreCurrentModule();
      return(NULL);
     }

   for (theConstruct = (*constructClass->getNextItemFunction)(NULL);
        theConstruct != NULL;
        theConstruct = (*constructClass->getNextItemFunction)(theConstruct))
     { 
      if (findValue == (*constructClass->getConstructNameFunction)(theConstruct)) 
        { 
         RestoreCurrentModule();
         return (theConstruct);
        }
     } 

   RestoreCurrentModule();
   return(NULL);
  }

/****************************************************************************/
/* UndefconstructCommand: Driver routine for the undef<construct> commands. */
/****************************************************************************/
globle VOID UndefconstructCommand(command,constructClass)
  char *command;
  struct construct *constructClass;
  {
   char *constructName;
   char buffer[80];
   
   sprintf(buffer,"%s name",constructClass->constructName);

   constructName = GetConstructName(command,buffer);
   if (constructName == NULL) return;

#if (! RUN_TIME) && (! BLOAD_ONLY)
   if (((*constructClass->findFunction)(constructName) == CLIPS_FALSE) && (strcmp("*",constructName) != 0))
     {
      CantFindItemErrorMessage(constructClass->constructName,constructName);
      return;
     }
   else if (DeleteNamedConstruct(constructName,constructClass) == CLIPS_FALSE)
     {
      CantDeleteItemErrorMessage(constructClass->constructName,constructName);
      return;
     }

   return;
#else
   CantDeleteItemErrorMessage(constructClass->constructName,constructName);
   return;
#endif
  }
   
/*************************************************************************/
/* PPConstructCommand: Driver routine for the ppdef<construct> commands. */
/*************************************************************************/
globle VOID PPConstructCommand(command,constructClass)
  char *command;
  struct construct *constructClass;
  {
   char *constructName;
   char buffer[80];
   
   sprintf(buffer,"%s name",constructClass->constructName);

   constructName = GetConstructName(command,buffer);
   
   if (constructName == NULL) return;

   if (PPConstruct(constructName,WDISPLAY,constructClass) == CLIPS_FALSE)
     { CantFindItemErrorMessage(constructClass->constructName,constructName); }
  }
  
/****************************************************************/
/* PPConstruct: Driver routine for pretty printing a construct. */
/****************************************************************/
globle int PPConstruct(constructName,logicalName,constructClass)
  char *constructName;
  char *logicalName;
  struct construct *constructClass;
  {
   VOID *constructPtr;

   constructPtr = (*constructClass->findFunction)(constructName);
   if (constructPtr == NULL) return(CLIPS_FALSE);

   if ((*constructClass->getPPFormFunction)(constructPtr) == NULL) return(CLIPS_TRUE);
   PrintInChunks(logicalName,(*constructClass->getPPFormFunction)(constructPtr));
   return(CLIPS_TRUE);
  }

/********************************************************************************/
/* GetConstructModuleCommand: Driver routine for def<construct>-module routines */
/********************************************************************************/
globle SYMBOL_HN *GetConstructModuleCommand(command,constructClass)
  char *command;
  struct construct *constructClass;
  {
   char *constructName;
   char buffer[80];
   struct defmodule *constructModule;
   
   sprintf(buffer,"%s name",constructClass->constructName);

   constructName = GetConstructName(command,buffer);
   
   if (constructName == NULL) return((SYMBOL_HN *) CLIPSFalseSymbol);

   constructModule = GetConstructModule(constructName,constructClass);
   if (constructModule == NULL)
     {
      CantFindItemErrorMessage(constructClass->constructName,constructName);
      return((SYMBOL_HN *) CLIPSFalseSymbol);
     }
   return(constructModule->name);
  }
  
/*****************************************************************************/
/* GetConstructModule: Driver routine for getting the module for a construct */
/*****************************************************************************/
globle struct defmodule *GetConstructModule(constructName,constructClass)
  char *constructName;
  struct construct *constructClass;
  {
   struct constructHeader *constructPtr;
   int count, position;
   SYMBOL_HN *theName;
   
   if ((position = FindModuleSeparator(constructName)) != CLIPS_FALSE)
     {
      theName = ExtractModuleName(position,constructName);
      if (theName != NULL) return((struct defmodule *) FindDefmodule(ValueToString(theName)));
     }
     
   constructPtr = (struct constructHeader *) 
                  FindImportedConstruct(constructClass->constructName,NULL,constructName,
                                        &count,CLIPS_TRUE,NULL);
   if (constructPtr == NULL) return(NULL);

   return(constructPtr->whichModule->theModule);
  }

/***************************************************************/
/* Undefconstruct: Generic C routine for deleting a construct. */
/***************************************************************/
globle BOOLEAN Undefconstruct(theConstruct,constructClass)
  VOID *theConstruct;
  struct construct *constructClass;
  {
#if BLOAD_ONLY || RUN_TIME
   return(CLIPS_FALSE);
#else
   VOID *currentConstruct,*nextConstruct;
   BOOLEAN success;
   
   /*================================================*/
   /* Delete all constructs of the specified type if */
   /* the construct pointer is the NULL pointer.     */
   /*================================================*/

   if (theConstruct == NULL)
     {
      currentConstruct = (*constructClass->getNextItemFunction)(NULL);
      success = CLIPS_TRUE;
      while (currentConstruct != NULL)
        {
         nextConstruct = (*constructClass->getNextItemFunction)(currentConstruct);
         if ((*constructClass->isConstructDeletableFunction)(currentConstruct))
           {
            RemoveConstructFromModule((struct constructHeader *) currentConstruct);
            (*constructClass->freeFunction)(currentConstruct);
           }
         else
           {
            CantDeleteItemErrorMessage(constructClass->constructName,
                        ValueToString((*constructClass->getConstructNameFunction)(currentConstruct)));
            success = CLIPS_FALSE;
           }
         currentConstruct = nextConstruct;
        }

      return(success);
     }
     
   /*==================================================*/
   /* Return FALSE if the construct cannot be deleted. */
   /*==================================================*/

   if ((*constructClass->isConstructDeletableFunction)(theConstruct) == CLIPS_FALSE)
     { return(CLIPS_FALSE); }

   RemoveConstructFromModule((struct constructHeader *) theConstruct);
   (*constructClass->freeFunction)(theConstruct);
   return(CLIPS_TRUE);
#endif
  }
  
/**********************************************************/
/* SaveConstruct: Generic routine for saving a construct. */
/**********************************************************/
globle VOID SaveConstruct(logicalName,constructClass)
  char *logicalName;
  struct construct *constructClass;
  {
   struct defmodule *theModule;
   char *ppform;
   struct constructHeader *theConstruct;

   SaveCurrentModule();
   
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
      theConstruct = (struct constructHeader *)
                     (*constructClass->getNextItemFunction)(NULL);
      
      while (theConstruct != NULL)
        {
         ppform = (*constructClass->getPPFormFunction)(theConstruct);    
         if (ppform != NULL)
           {
            PrintInChunks(logicalName,ppform);
            PrintCLIPS(logicalName,"\n");
           }
         theConstruct = (struct constructHeader *)
                        (*constructClass->getNextItemFunction)(theConstruct);
        }
        
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }
     
   RestoreCurrentModule();
  }
  
/*********************************************************/
/* GetConstructModuleName: Generic routine for returning */
/*   the name of the module to which a construct belongs */
/*********************************************************/
globle char *GetConstructModuleName(theConstruct)
  struct constructHeader *theConstruct;
  { return(GetDefmoduleName((VOID *) theConstruct->whichModule->theModule)); }

/*********************************************************/
/* GetConstructNameString: Generic routine for returning */
/*   the name string of a construct.                     */
/*********************************************************/
globle char *GetConstructNameString(theConstruct)
  struct constructHeader *theConstruct;
  { return(ValueToString(theConstruct->name)); }

/**********************************************************/
/* GetConstructNamePointer: Generic routine for returning */
/*   the name pointer of a construct.                     */
/**********************************************************/
globle SYMBOL_HN *GetConstructNamePointer(theConstruct)
  struct constructHeader *theConstruct;
  { return(theConstruct->name); }

/***************************************************/
/* GetConstructListFunction: Generic CLIPS Routine */
/*   for retrieving the constructs in a module.    */
/***************************************************/
globle VOID GetConstructListFunction(functionName,returnValue,constructClass)
  char *functionName;
  DATA_OBJECT_PTR returnValue;
  struct construct *constructClass;
  {
   struct defmodule *theModule;
   DATA_OBJECT result;
   int numArgs;
   
   /*===========================================*/
   /* Determine if a module name was specified. */
   /*===========================================*/
   
   if ((numArgs = ArgCountCheck(functionName,NO_MORE_THAN,1)) == -1)
     {      
      SetMultifieldErrorValue(returnValue);
      return;
     }

   if (numArgs == 1)
     {
      RtnUnknown(1,&result);

      if (GetType(result) != SYMBOL)
        {
         SetMultifieldErrorValue(returnValue);
         ExpectedTypeError1(functionName,1,"defmodule name");
         return;
        }
        
      if ((theModule = (struct defmodule *) FindDefmodule(DOToString(result))) == NULL)
        {
         if (strcmp("*",DOToString(result)) != 0) 
           {
            SetMultifieldErrorValue(returnValue);
            ExpectedTypeError1(functionName,1,"defmodule name");
            return;
           }
           
         theModule = NULL;
        }
     }
   else
     { theModule = ((struct defmodule *) GetCurrentModule()); }
  
   /*=====================*/
   /* Get the constructs. */
   /*=====================*/
   
   GetConstructList(returnValue,constructClass,theModule);
  }
  
/********************************************/
/* GetConstructList: Generic C Routine for  */
/*   retrieving the constructs in a module. */
/********************************************/
globle VOID GetConstructList(returnValue,constructClass,theModule)
  DATA_OBJECT_PTR returnValue;
  struct construct *constructClass;
  struct defmodule *theModule;
  { 
   VOID *theConstruct;
   long count = 0;
   struct multifield *theList;
   SYMBOL_HN *theName;
   struct defmodule *loopModule;
   int allModules = CLIPS_FALSE;
   
   /*==========================*/
   /* Save the current module. */
   /*==========================*/
   
   SaveCurrentModule();
      
   /*=======================================*/
   /* If the module specified is NULL, then */
   /* get all constructs in all modules.    */
   /*=======================================*/

   if (theModule == NULL)
     {
      theModule = (struct defmodule *) GetNextDefmodule(NULL);
      allModules = CLIPS_TRUE;
     }
     
   /*=================================================*/
   /* Count the number of constructs to be retrieved. */
   /*=================================================*/
   
   loopModule = theModule;
   while (loopModule != NULL)
     {
      SetCurrentModule((VOID *) loopModule);
      theConstruct = NULL;
      while ((theConstruct = (*constructClass->getNextItemFunction)(theConstruct)) != NULL) 
        { count++; }
        
      if (allModules) loopModule = (struct defmodule *) GetNextDefmodule(loopModule);
      else loopModule = NULL;
     }
   
   /*===========================================================*/
   /* Create the multifield value to store the construct names. */
   /*===========================================================*/
   
   SetpType(returnValue,MULTIFIELD);
   SetpDOBegin(returnValue,1);
   SetpDOEnd(returnValue,count);
   theList = (struct multifield *) CreateMultifield((int) count);
   SetpValue(returnValue,(VOID *) theList);
   
   /*====================================================*/
   /* Store the construct names in the multifield value. */
   /*====================================================*/
   
   loopModule = theModule;
   count = 1;
   while (loopModule != NULL)
     {
      SetCurrentModule((VOID *) loopModule);
      theConstruct = NULL;
      while ((theConstruct = (*constructClass->getNextItemFunction)(theConstruct)) != NULL) 
        {
         theName = (*constructClass->getConstructNameFunction)(theConstruct);
         SetMFType(theList,count,SYMBOL);
         if (allModules)
           {
            char buffer[512];
            
            strcpy(buffer,GetDefmoduleName(loopModule));
            strcat(buffer,"::");
            strcat(buffer,ValueToString(theName));
            SetMFValue(theList,count,AddSymbol(buffer));
           }
         else
           { SetMFValue(theList,count,AddSymbol(ValueToString(theName))); }
         count++;
        } 
 
      if (allModules) loopModule = (struct defmodule *) GetNextDefmodule(loopModule);
      else loopModule = NULL;
     }
   
   RestoreCurrentModule();
  }

/***********************************************/
/* ListConstructCommand: Generic CLIPS Routine */
/*   for listing the constructs in a module.   */
/***********************************************/
globle VOID ListConstructCommand(functionName,constructClass)
  char *functionName;
  struct construct *constructClass;
  {
   struct defmodule *theModule;
   DATA_OBJECT result;
   int numArgs;
   
   /*===========================================*/
   /* Determine if a module name was specified. */
   /*===========================================*/
   
   if ((numArgs = ArgCountCheck(functionName,NO_MORE_THAN,1)) == -1) return;

   if (numArgs == 1)
     {
      RtnUnknown(1,&result);

      if (GetType(result) != SYMBOL)
        {
         ExpectedTypeError1(functionName,1,"defmodule name");
         return;
        }
        
      if ((theModule = (struct defmodule *) FindDefmodule(DOToString(result))) == NULL)
        {
         if (strcmp("*",DOToString(result)) != 0) 
           {
            ExpectedTypeError1(functionName,1,"defmodule name");
            return;
           }
           
         theModule = NULL;
        }
     }
   else
     { theModule = ((struct defmodule *) GetCurrentModule()); }
  
   /*======================*/
   /* List the constructs. */
   /*======================*/
   
   ListConstruct(constructClass,WDISPLAY,theModule);
  }
  
/****************************************************************************/
/* ListConstruct: Generic C Routine for listing the constructs in a module. */
/****************************************************************************/
globle VOID ListConstruct(constructClass,logicalName,theModule)
  char *logicalName;
  struct construct *constructClass;
  struct defmodule *theModule;
  {
   VOID *constructPtr;
   SYMBOL_HN *constructName;
   long count = 0;
   int allModules = CLIPS_FALSE;
        
   /*==========================*/
   /* Save the current module. */
   /*==========================*/
   
   SaveCurrentModule();
   
   /*=======================================*/
   /* If the module specified is NULL, then */
   /* list all constructs in all modules.   */
   /*=======================================*/
  
   if (theModule == NULL)
     { 
      theModule = (struct defmodule *) GetNextDefmodule(NULL);
      allModules = CLIPS_TRUE;
     }
     
   /*===========================*/
   /* Print out the constructs. */
   /*===========================*/
   
   while (theModule != NULL)
     {
      if (allModules) 
        {
         PrintCLIPS(logicalName,GetDefmoduleName(theModule));
         PrintCLIPS(logicalName,":\n");
        }
        
      SetCurrentModule((VOID *) theModule);
      constructPtr = (*constructClass->getNextItemFunction)(NULL);
      while (constructPtr != NULL)
        {
         if (HaltExecution == CLIPS_TRUE) return;
  
         constructName = (*constructClass->getConstructNameFunction)(constructPtr);
           
         if (constructName != NULL)
           {
            if (allModules) PrintCLIPS(WDISPLAY,"   ");
            PrintCLIPS(logicalName,ValueToString(constructName));
            PrintCLIPS(logicalName,"\n");
           }
            
         constructPtr = (*constructClass->getNextItemFunction)(constructPtr);
         count++;
        }
        
      if (allModules) theModule = (struct defmodule *) GetNextDefmodule(theModule);
      else theModule = NULL;
     }

   /*=================================================*/
   /* Print the tally and restore the current module. */
   /*=================================================*/
   
   PrintTally(WDISPLAY,count,constructClass->constructName,
                             constructClass->pluralName);
   
   RestoreCurrentModule();
  }
  
/**********************************************************/
/* SetNextConstruct: Sets the next field of one construct */
/*   to point to another construct of the same type.      */
/**********************************************************/
globle VOID SetNextConstruct(theConstruct,targetConstruct)
  struct constructHeader *theConstruct, *targetConstruct;
  { theConstruct->next = targetConstruct; }
  
/********************************************************************/
/* GetConstructModuleItem: Returns the construct module for a given */
/*   construct (note that this is a pointer to a data structure     */
/*   like the deffactsModule, not a pointer to an environment       */
/*   module which contains a number of types of constructs.         */
/********************************************************************/
globle struct defmoduleItemHeader *GetConstructModuleItem(theConstruct)
  struct constructHeader *theConstruct;
  { return(theConstruct->whichModule); }
  
/***************************************************************/
/* GetConstructPPForm: Returns the pretty print representation */
/*   for the specified construct.                              */
/***************************************************************/
globle char *GetConstructPPForm(theConstruct)
  struct constructHeader *theConstruct;
  { return(theConstruct->ppForm); }

/****************************************************/
/* GetNextConstructItem: Returns the next construct */
/*   items from a list of constructs.               */
/****************************************************/
globle struct constructHeader *GetNextConstructItem(theConstruct,moduleIndex)
  struct constructHeader *theConstruct;
  int moduleIndex;
  {   
   struct defmoduleItemHeader *theModuleItem;
   
   if (theConstruct == NULL)
     {
      theModuleItem = (struct defmoduleItemHeader *) 
                      GetModuleItem(NULL,moduleIndex);
      if (theModuleItem == NULL) return(NULL);
      return(theModuleItem->firstItem);
     }
   
   return(theConstruct->next);
  }

/*********************************************************************/
/* GetConstructModuleItemByIndex: Returns a pointer to the defmodule */
/*  item for the specified construct. If theModule is NULL,          */
/*  then the construct module item for the current module is         */
/*  returned, otherwise the construct module item for the            */
/*  specified construct is returned.                                 */
/*********************************************************************/
globle struct defmoduleItemHeader *GetConstructModuleItemByIndex(theModule,moduleIndex)
  struct defmodule *theModule;
  int moduleIndex;
  { 
   if (theModule != NULL)
     { 
      return((struct defmoduleItemHeader *)
             GetModuleItem(theModule,moduleIndex));
     }
     
   return((struct defmoduleItemHeader *)
          GetModuleItem(((struct defmodule *) GetCurrentModule()),moduleIndex));
  }
  
/**************************************************************/
/* FreeConstructHeaderModule: Deallocates the data structures */
/*   associated with the construct module item header.        */ 
/**************************************************************/
globle VOID FreeConstructHeaderModule(theModuleItem,constructClass)
  struct defmoduleItemHeader *theModuleItem;
  struct construct *constructClass;
  {
   struct constructHeader *thisOne, *nextOne;
   
   thisOne = theModuleItem->firstItem;
   
   while (thisOne != NULL)
     { 
      nextOne = thisOne->next;
      (*constructClass->freeFunction)(thisOne); 
      thisOne = nextOne;
     }
  } 

/**********************************************/
/* DoForAllConstructs: Executes an action for */
/*   all constructs of a specified class.     */
/**********************************************/
globle long DoForAllConstructs(actionFunction,moduleItemIndex,interruptable,userBuffer)
#if ANSI_COMPILER
  VOID (*actionFunction)(struct constructHeader *,VOID *);
#else
  VOID (*actionFunction)();
#endif
  int moduleItemIndex;
  int interruptable;
  VOID *userBuffer;
  {
   struct constructHeader *theConstruct;   
   struct defmoduleItemHeader *theModuleItem;
   VOID *theModule;
   long moduleCount = 0L;
        
   SaveCurrentModule();
   theModule = GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
      moduleCount++;
      theModuleItem = (struct defmoduleItemHeader *)
                      GetModuleItem(theModule,moduleItemIndex);
   
      theConstruct = theModuleItem->firstItem;
                
      while (theConstruct != NULL)
        {
         if (interruptable)
           { 
            if (GetHaltExecution() == CLIPS_TRUE) 
              {
               RestoreCurrentModule();
               return(-1L); 
              }
           }
            
         (*actionFunction)(theConstruct,userBuffer);
           
         theConstruct = theConstruct->next;
        }
        
      theModule = GetNextDefmodule(theModule);
     }
     
   RestoreCurrentModule();
   return(moduleCount);
  }
  
/*****************************************************/
/* InitializeConstructHeader: Initializes construct  */
/*   header info, including to which module item the */
/*   new construct belongs                           */
/*****************************************************/
globle VOID InitializeConstructHeader(constructType,theConstruct,theConstructName)
  char *constructType;
  struct constructHeader *theConstruct;
  SYMBOL_HN *theConstructName;
  {
   struct moduleItem *theModuleItem;
   struct defmoduleItemHeader *theItemHeader;
   
   theModuleItem = FindModuleItem(constructType);
   theItemHeader = (struct defmoduleItemHeader *) 
                   GetModuleItem(NULL,theModuleItem->moduleIndex);
                   
   theConstruct->whichModule = theItemHeader;
   theConstruct->name = theConstructName;
   theConstruct->ppForm = NULL;
   theConstruct->bsaveID = 0L;
   theConstruct->next = NULL;
  }

/*****************************************************/
/* SetConstructPPForm: Sets construct pretty-print   */
/*   form and deletes old one                        */
/*****************************************************/
globle VOID SetConstructPPForm(theConstruct,ppForm)
  struct constructHeader *theConstruct;
  char *ppForm;
  {
   if (theConstruct->ppForm != NULL)
     {
      rm((VOID *) theConstruct->ppForm,
         (int) ((strlen(theConstruct->ppForm) + 1) * sizeof(char)));
     }
   theConstruct->ppForm = ppForm;
  }

#if DEBUGGING_FUNCTIONS

/******************************************************/
/* ConstructPrintWatchAccess: Provides an interface   */
/*   to the list-watch-items function for a construct */
/******************************************************/
globle BOOLEAN ConstructPrintWatchAccess(constructClass,log,argExprs,
                                         getWatchFunc,setWatchFunc)
  struct construct *constructClass;
  char *log;
  EXPRESSION *argExprs;
#if ANSI_COMPILER
  BOOLEAN (*getWatchFunc)(VOID *);
  VOID (*setWatchFunc)(BOOLEAN,VOID *);
#else
  BOOLEAN (*getWatchFunc)();
  VOID (*setWatchFunc)();
#endif
  {
   return(ConstructWatchSupport(constructClass,"list-watch-items",log,argExprs,
                                CLIPS_FALSE,CLIPS_FALSE,getWatchFunc,setWatchFunc));
  }

/**************************************************/
/* ConstructSetWatchAccess: Provides an interface */
/*   to the watch function for a construct        */
/**************************************************/
globle BOOLEAN ConstructSetWatchAccess(constructClass,newState,argExprs,
                                       getWatchFunc,setWatchFunc)
  struct construct *constructClass;
  BOOLEAN newState;
  EXPRESSION *argExprs;
#if ANSI_COMPILER
  BOOLEAN (*getWatchFunc)(VOID *);
  VOID (*setWatchFunc)(BOOLEAN,VOID *);
#else
  BOOLEAN (*getWatchFunc)();
  VOID (*setWatchFunc)();
#endif
  {
   return(ConstructWatchSupport(constructClass,"watch",WERROR,argExprs,
                                CLIPS_TRUE,newState,getWatchFunc,setWatchFunc));
  }

/******************************************************/
/* ConstructWatchSupport: Generic construct interface */
/*   into watch and list-watch-items                  */
/******************************************************/
static BOOLEAN ConstructWatchSupport(constructClass,funcName,log,argExprs,
                                     setFlag,newState,getWatchFunc,setWatchFunc)
  struct construct *constructClass;
  char *funcName,*log;
  EXPRESSION *argExprs;
  BOOLEAN setFlag,newState;
#if ANSI_COMPILER
  BOOLEAN (*getWatchFunc)(VOID *);
  VOID (*setWatchFunc)(BOOLEAN,VOID *);
#else
  BOOLEAN (*getWatchFunc)();
  VOID (*setWatchFunc)();
#endif
  {
   struct defmodule *theModule;
   VOID *theConstruct;
   DATA_OBJECT constructName;
   int argIndex = 2;
   
   /* =====================================
      If no constructs are specified,
      show/set the trace for all constructs
      ===================================== */
   if (argExprs == NULL)
     {
      SaveCurrentModule();
      theModule = (struct defmodule *) GetNextDefmodule(NULL);
      while (theModule != NULL)
        {
         SetCurrentModule((VOID *) theModule);
         if (setFlag == CLIPS_FALSE)
           {
            PrintCLIPS(log,GetDefmoduleName((VOID *) theModule));
            PrintCLIPS(log,":\n");
           }
         theConstruct = (*constructClass->getNextItemFunction)(NULL);
         while (theConstruct != NULL)
           {
            if (setFlag)
              (*setWatchFunc)(newState,theConstruct);
            else
              {
               PrintCLIPS(log,"   ");
               ConstructPrintWatch(log,constructClass,theConstruct,getWatchFunc);
              }
            theConstruct = (*constructClass->getNextItemFunction)(theConstruct);
           }
         theModule = (struct defmodule *) GetNextDefmodule((VOID *) theModule);
        }
      RestoreCurrentModule();
      return(CLIPS_TRUE);
     }
     
   /* ===============================================
      Show/set the trace for each specified construct
      =============================================== */
   while (argExprs != NULL)
     {
      if (EvaluateExpression(argExprs,&constructName))
        return(CLIPS_FALSE);
      if ((constructName.type != SYMBOL) ? CLIPS_TRUE :
          ((theConstruct = LookupConstruct(constructClass,
                                           DOToString(constructName),CLIPS_TRUE)) == NULL))
        {
         ExpectedTypeError1(funcName,argIndex,constructClass->constructName);
         return(CLIPS_FALSE);
        }
      if (setFlag)
        (*setWatchFunc)(newState,theConstruct);
      else
        ConstructPrintWatch(log,constructClass,theConstruct,getWatchFunc);
      argIndex++;
      argExprs = GetNextArgument(argExprs);
     }
   return(CLIPS_TRUE);
  }
  
/******************************************************/
/* ConstructPrintWatch: Displays the trace value of a */
/*   construct for list-watch-items                   */
/******************************************************/
static VOID ConstructPrintWatch(log,constructClass,theConstruct,getWatchFunc)
  char *log;
  struct construct *constructClass;
  VOID *theConstruct;
#if ANSI_COMPILER
  BOOLEAN (*getWatchFunc)(VOID *);
#else
  BOOLEAN (*getWatchFunc)();
#endif
  {
   PrintCLIPS(log,ValueToString((*constructClass->getConstructNameFunction)(theConstruct)));
   PrintCLIPS(log,(*getWatchFunc)(theConstruct) ? " = on\n" : " = off\n");
  }
   
#endif

/*************************************************************/
/* LookupConstruct: Finds a construct in the current or      */
/*   imported modules - or looks up in a non-imported module */
/*   if allowed to do so                                     */
/*************************************************************/
globle VOID *LookupConstruct(constructClass,constructName,moduleNameAllowed)
  struct construct *constructClass;
  char *constructName;
  BOOLEAN moduleNameAllowed;
  {
   VOID *theConstruct;
   char *constructType;
   int moduleCount;
   
   constructType = constructClass->constructName;
   theConstruct = 
     FindImportedConstruct(constructType,NULL,constructName,&moduleCount,CLIPS_TRUE,NULL);

   /*===========================================*/
   /* Return NULL if the reference is ambiguous */
   /* (it was found in more than one module).   */
   /*===========================================*/
   
   if (theConstruct != NULL)
     {
      if (moduleCount > 1)
        {
         AmbiguousReferenceErrorMessage(constructType,constructName);
         return(NULL);
        }
      return(theConstruct);
     }
     
   if (moduleNameAllowed && FindModuleSeparator(constructName))
     theConstruct = (*constructClass->findFunction)(constructName);
   return(theConstruct);
  }

