   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                  DEFGLOBAL MODULE                   */
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

#define _GLOBLDEF_SOURCE_

#include "setup.h"

#if DEFGLOBAL_CONSTRUCT

#include <stdio.h>
#define _CLIPS_STDIO_

#include "clipsmem.h"
#include "modulpsr.h"
#include "multifld.h"
#include "router.h"
#include "strngrtr.h"
#include "modulutl.h"
#include "globlbsc.h"
#include "globlpsr.h"
#include "globlcom.h"

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
#include "bload.h"
#include "globlbin.h"
#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
#include "globlcmp.h"
#endif

#include "globldef.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                   *AllocateModule(void);
   static VOID                    FreeModule(VOID *);
   static VOID                    ReturnDefglobal(VOID *);
   static VOID                    InitializeDefglobalModules(void);
   static BOOLEAN                 GetDefglobalValue2(VOID *,DATA_OBJECT_PTR);
   static int                     QGetDefglobalValue(VOID *,DATA_OBJECT_PTR);
   static VOID                    IncrementDefglobalBusyCount(VOID *);
   static VOID                    DecrementDefglobalBusyCount(VOID *);
#else
   static VOID                   *AllocateModule();
   static VOID                    FreeModule();
   static VOID                    ReturnDefglobal();
   static VOID                    InitializeDefglobalModules();
   static BOOLEAN                 GetDefglobalValue2();
   static int                     QGetDefglobalValue();
   static VOID                    IncrementDefglobalBusyCount();
   static VOID                    DecrementDefglobalBusyCount();
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct construct       *DefglobalConstruct;
   globle int                     DefglobalModuleIndex;
   globle int                     ChangeToGlobals = CLIPS_FALSE;

   globle struct entityRecord   GlobalInfo =         { GBL_VARIABLE,0,0,0,
                                                       NULL, 
                                                       NULL,
                                                       NULL, 
                                                       GetDefglobalValue2,
                                                       NULL,NULL,
                                                       NULL,NULL,NULL };
                                               
   globle struct entityRecord   DefglobalPtrRecord = { DEFGLOBAL_PTR,0,0,0,
                                                       NULL,NULL,NULL, 
                                                       QGetDefglobalValue,
                                                       NULL,
                                                       DecrementDefglobalBusyCount,
                                                       IncrementDefglobalBusyCount,
                                                       NULL,NULL,NULL,NULL };

/**************************************************************/
/* InitializeDefglobals: Initializes the defglobal construct. */
/**************************************************************/
globle VOID InitializeDefglobals()
  {   
   InstallPrimitive(&GlobalInfo,GBL_VARIABLE);
   InstallPrimitive((ENTITY_RECORD_PTR) &DefglobalPtrRecord,DEFGLOBAL_PTR);

   InitializeDefglobalModules(); 

   DefglobalBasicCommands();
   DefglobalCommands();

   DefglobalConstruct = 
      AddConstruct("defglobal","defglobals",ParseDefglobal,FindDefglobal,
                   GetConstructNamePointer,GetConstructPPForm,
                   GetConstructModuleItem,GetNextDefglobal,SetNextConstruct,
                   IsDefglobalDeletable,Undefglobal,ReturnDefglobal);
  }

/*********************************************************/
/* InitializeDefglobalModules: Initializes the defglobal */
/*   construct for use with the defmodule construct.     */
/*********************************************************/
static VOID InitializeDefglobalModules()
  {
   DefglobalModuleIndex = RegisterModuleItem("defglobal",
                                    AllocateModule,
                                    FreeModule,
#if BLOAD_AND_BSAVE || BLOAD || BLOAD_ONLY
                                    BloadDefglobalModuleReference,
#else
                                    NULL,
#endif
#if CONSTRUCT_COMPILER && (! RUN_TIME)
                                    DefglobalCModuleReference,
#else
                                    NULL,
#endif
                                    FindDefglobal);

#if (! BLOAD_ONLY) && (! RUN_TIME) && DEFMODULE_CONSTRUCT
   AddPortConstructItem("defglobal",SYMBOL);
#endif
  }

/*************************************************/
/* AllocateModule: Allocates a defglobal module. */
/*************************************************/
static VOID *AllocateModule()
  { return((VOID *) get_struct(defglobalModule)); }
  
/***********************************************/
/* FreeModule: Deallocates a defglobal module. */ 
/***********************************************/
static VOID FreeModule(theItem) 
  VOID *theItem;
  {
   FreeConstructHeaderModule(theItem,DefglobalConstruct);
   rtn_struct(defglobalModule,theItem);
  } 

/**************************************************************/
/* GetDefglobalModuleItem: Returns a pointer to the defmodule */
/*  item for the specified defglobal or defmodule.            */
/**************************************************************/
globle struct defglobalModule *GetDefglobalModuleItem(theModule)
  struct defmodule *theModule;
  { return((struct defglobalModule *) GetConstructModuleItemByIndex(theModule,DefglobalModuleIndex)); }
    
/**********************************************************************/
/* FindDefglobal: Searches for a defglobal in the list of defglobals. */
/*   Returns a pointer to the defglobal if found, otherwise NULL.     */
/**********************************************************************/
globle VOID *FindDefglobal(defglobalName)
  char *defglobalName;
  { return(FindNamedConstruct(defglobalName,DefglobalConstruct)); }

/*****************************************************************/
/* GetNextDefglobal: If passed a NULL pointer, returns the first */
/*   defglobal in the defglobal list. Otherwise returns the next */
/*   defglobal following the defglobal passed as an argument.    */
/*****************************************************************/
globle VOID *GetNextDefglobal(defglobalPtr)
  VOID *defglobalPtr;
  { return((VOID *) GetNextConstructItem(defglobalPtr,DefglobalModuleIndex)); }
  
/********************************************************/
/* IsDefglobalDeletable: Returns TRUE if a particular   */
/*   defglobal can be deleted, otherwise returns FALSE. */
/********************************************************/
#if IBM_TBC
#pragma argsused
#endif
globle BOOLEAN IsDefglobalDeletable(ptr)
  VOID *ptr;
  {
#if MAC_MPW
#pragma unused(ptr)
#endif
#if BLOAD_ONLY || RUN_TIME
   return(FALSE);
#else
#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded()) return(FALSE);
#endif
   
   if (((struct defglobal *) ptr)->busyCount) return(CLIPS_FALSE);
   
   return(CLIPS_TRUE);
#endif
  }
  
/************************************************************/
/* ReturnDefglobal: Returns the data structures associated  */
/*   with a defglobal construct to the pool of free memory. */
/************************************************************/
static VOID ReturnDefglobal(vTheDefglobal)
  VOID *vTheDefglobal;
  {
#if (! BLOAD_ONLY) && (! RUN_TIME)
   struct defglobal *theDefglobal = (struct defglobal *) vTheDefglobal;
   
   if (theDefglobal == NULL) return;
   
   ValueDeinstall(&theDefglobal->current);
   if (theDefglobal->current.type == MULTIFIELD)
     { ReturnMultifield(theDefglobal->current.value); }
      
   RemoveHashedExpression(theDefglobal->initial);

   DeinstallConstructHeader(&theDefglobal->header);
   
   rtn_struct(defglobal,theDefglobal);
   
   ChangeToGlobals = CLIPS_TRUE;
#endif
  } 
  
/************************************************/
/* QSetDefglobalValue: Lowest level routine for */
/*   setting a defglobal's value.               */
/************************************************/
globle VOID QSetDefglobalValue(theGlobal,vPtr)
  struct defglobal *theGlobal;
  DATA_OBJECT_PTR vPtr;
  {

   if (vPtr->value == NULL)
     {
      EvaluateExpression(theGlobal->initial,vPtr);
      if (EvaluationError)
        {
         vPtr->type = SYMBOL;
         vPtr->value = CLIPSFalseSymbol;
        }
     }

#if DEBUGGING_FUNCTIONS
   if (theGlobal->watch)
     {
      PrintCLIPS(WTRACE,":== ?*");
      PrintCLIPS(WTRACE,ValueToString(theGlobal->header.name));
      PrintCLIPS(WTRACE,"* ==> ");
      PrintDataObject(WTRACE,vPtr);
      PrintCLIPS(WTRACE," <== ");
      PrintDataObject(WTRACE,&theGlobal->current);
      PrintCLIPS(WTRACE,"\n");
     }
#endif

   ValueDeinstall(&theGlobal->current);
   if (theGlobal->current.type == MULTIFIELD)
     { ReturnMultifield(theGlobal->current.value); }

   theGlobal->current.type = vPtr->type;
   if (vPtr->type != MULTIFIELD) theGlobal->current.value = vPtr->value;
   else DuplicateMultifield(&theGlobal->current,vPtr);
   ValueInstall(&theGlobal->current);

   ChangeToGlobals = CLIPS_TRUE;
  } 
  
/**************************************************************/
/* QFindDefglobal: Searches for a defglobal in the list of    */
/*   defglobals. Returns a pointer to the defglobal if found, */
/*   otherwise NULL.                                          */
/**************************************************************/
globle struct defglobal *QFindDefglobal(defglobalName)
  SYMBOL_HN *defglobalName;
  {
   struct defglobal *theDefglobal;

   for (theDefglobal = (struct defglobal *) GetNextDefglobal(NULL);
        theDefglobal != NULL;
        theDefglobal = (struct defglobal *) GetNextDefglobal(theDefglobal))
     { if (defglobalName == theDefglobal->header.name) return (theDefglobal); }

   return(NULL);
  }
  
/*********************************************************************/
/* GetDefglobalValueForm: Returns the pretty print representation of */
/*   the current value of the specified defglobal. For example, if   */
/*   the current value of ?*x* is 5, the string "?*x* = 5" would be  */
/*   returned.                                                       */
/*********************************************************************/
globle VOID GetDefglobalValueForm(buffer,bufferLength,vTheGlobal)
  char *buffer;
  int bufferLength;
  VOID *vTheGlobal;
  {
   struct defglobal *theGlobal = (struct defglobal *) vTheGlobal;

   OpenStringDestination("GlobalValueForm",buffer,bufferLength);
   PrintCLIPS("GlobalValueForm","?*");
   PrintCLIPS("GlobalValueForm",ValueToString(theGlobal->header.name));
   PrintCLIPS("GlobalValueForm","* = ");
   PrintDataObject("GlobalValueForm",&theGlobal->current);
   CloseStringDestination("GlobalValueForm");
  }
  
/*********************************************************/
/* GetGlobalsChanged: Returns the defglobal change flag. */
/*********************************************************/
globle int GetGlobalsChanged()
  { return(ChangeToGlobals); }

/******************************************************/
/* SetGlobalsChanged: Sets the defglobal change flag. */
/******************************************************/
globle VOID SetGlobalsChanged(value)
  int value;
  { ChangeToGlobals = value; }
  
/**********************************************************/
/* GetDefglobalValue2: Returns the value of the specified */
/*   global variable in the supplied DATA_OBJECT.         */
/**********************************************************/
static BOOLEAN GetDefglobalValue2(theValue,vPtr)
  VOID *theValue;
  DATA_OBJECT_PTR vPtr;
  { 
   struct defglobal *theGlobal;
   int count;
   
   theGlobal = (struct defglobal *)
               FindImportedConstruct("defglobal",NULL,ValueToString(theValue),
               &count,CLIPS_TRUE,NULL);
               
   if (theGlobal == NULL)
     {
      PrintErrorID("GLOBLDEF",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Global variable ?*");
      PrintCLIPS(WERROR,ValueToString(theValue));
      PrintCLIPS(WERROR,"* is unbound.\n");
      vPtr->type = SYMBOL;
      vPtr->value = CLIPSFalseSymbol;
      SetEvaluationError(CLIPS_TRUE);
      return(CLIPS_FALSE);
     }
     
   if (count > 1) 
     {
      AmbiguousReferenceErrorMessage("defglobal",ValueToString(theValue));
      vPtr->type = SYMBOL;
      vPtr->value = CLIPSFalseSymbol;
      SetEvaluationError(CLIPS_TRUE);
      return(CLIPS_FALSE);
     }
   
   QGetDefglobalValue(theGlobal,vPtr);
     
   return(CLIPS_TRUE);
  }

/***************************************************************/
/* QGetDefglobalValue: Returns the value of a global variable. */
/***************************************************************/
static int QGetDefglobalValue(vTheGlobal,vPtr)
  VOID *vTheGlobal;
  DATA_OBJECT_PTR vPtr;
  {
   struct defglobal *theGlobal = (struct defglobal *) vTheGlobal;
   
   vPtr->type = theGlobal->current.type;
   vPtr->value = theGlobal->current.value;
   vPtr->begin = theGlobal->current.begin;
   vPtr->end = theGlobal->current.end;

   /*===========================================================*/
   /* If the global contains a multifield value, return a copy  */
   /* of the value so that routines which use this value are    */
   /* not affected if the value of the global is later changed. */
   /*===========================================================*/

   if (vPtr->type == MULTIFIELD)
     {
      vPtr->value = CreateMultifield(vPtr->end + 1);
      CopyMemory(struct field,vPtr->end + 1,
                                &((struct multifield *) vPtr->value)->theFields[0],
                                &((struct multifield *) theGlobal->current.value)->theFields[theGlobal->current.begin]);
     }
     
   return(TRUE);
  }
  
/*********************************************************/
/* GetDefglobalValue: Returns the value of the specified */
/*   global variable in the supplied DATA_OBJECT.        */
/*********************************************************/
globle BOOLEAN GetDefglobalValue(variableName,vPtr)
  char *variableName;
  DATA_OBJECT_PTR vPtr;
  {
   struct defglobal *theDefglobal;

   if ((theDefglobal = (struct defglobal *) FindDefglobal(variableName)) != NULL)
     {
      vPtr->type = theDefglobal->current.type;
      vPtr->value = theDefglobal->current.value;
      vPtr->begin = theDefglobal->current.begin;
      vPtr->end = theDefglobal->current.end;
      if (vPtr->type == MULTIFIELD)
        {
         vPtr->value = CreateMultifield(vPtr->end + 1);
         CopyMemory(struct field,vPtr->end + 1,
                                   &((struct multifield *) vPtr->value)->theFields[0],
                                   &((struct multifield *) theDefglobal->current.value)->theFields[theDefglobal->current.begin]);
        }
      
      return(CLIPS_TRUE);
     }

   return(CLIPS_FALSE);
  }

/***************************************************************/
/* SetDefglobalValue: Sets the value of the specified global   */
/*   variable to the value stored in the supplied DATA_OBJECT. */
/***************************************************************/
globle BOOLEAN SetDefglobalValue(variableName,vPtr)
  char *variableName;
  DATA_OBJECT_PTR vPtr;
  {
   struct defglobal *theGlobal;

   if ((theGlobal = QFindDefglobal(AddSymbol(variableName))) != NULL)
     {
      QSetDefglobalValue(theGlobal,vPtr);
      return(CLIPS_TRUE);
     }
   return(CLIPS_FALSE);
  }
  
/**********************************************************/
/* DecrementDefglobalBusyCount: Decrements the busy count */
/*   of a defglobal data structure.                       */
/**********************************************************/
static VOID DecrementDefglobalBusyCount(vTheGlobal)
  VOID *vTheGlobal;
  {
   struct defglobal *theGlobal = (struct defglobal *) vTheGlobal;
   
   if (! ClearInProgress) theGlobal->busyCount--;
  }

/**********************************************************/
/* IncrementDefglobalBusyCount: Increments the busy count */
/*   of a defglobal data structure.                       */
/**********************************************************/
static VOID IncrementDefglobalBusyCount(vTheGlobal)
  VOID *vTheGlobal;
  {
   struct defglobal *theGlobal = (struct defglobal *) vTheGlobal;
   
   theGlobal->busyCount++;
  } 

/***********************************************************************/
/* UpdateDefglobalScope: Updates the scope flag of all the defglobals. */
/***********************************************************************/
globle VOID UpdateDefglobalScope()
  {
   struct defglobal *theDefglobal;
   int moduleCount;
   struct defmodule *theModule;
   struct defmoduleItemHeader *theItem;
   
   for (theModule = (struct defmodule *) GetNextDefmodule(NULL);
        theModule != NULL;
        theModule = (struct defmodule *) GetNextDefmodule(theModule))
     {
      theItem = (struct defmoduleItemHeader *)
                GetModuleItem(theModule,DefglobalModuleIndex);
      
      for (theDefglobal = (struct defglobal *) theItem->firstItem;
           theDefglobal != NULL ;
           theDefglobal = (struct defglobal *) GetNextDefglobal(theDefglobal))
        {  
         if (FindImportedConstruct("defglobal",theModule,
                                   ValueToString(theDefglobal->header.name),
                                   &moduleCount,CLIPS_TRUE,NULL) != NULL)
           { theDefglobal->inScope = CLIPS_TRUE; }
         else
           { theDefglobal->inScope = CLIPS_FALSE; }
        }  
     }
  }
  
/*****************************************************************/
/* GetNextDefglobalInScope:                    */
/*****************************************************************/
globle VOID *GetNextDefglobalInScope(vTheGlobal)
  VOID *vTheGlobal;
  {          
   static struct defmodule *theDefmodule = NULL;
   static long lastModuleIndex = -1;
   struct defglobal *theGlobal = (struct defglobal *) vTheGlobal;
   struct defmoduleItemHeader *theItem;
   
   if (theGlobal == NULL)
     {       
      if (lastModuleIndex != ModuleChangeIndex)
        {
         UpdateDefglobalScope();
         lastModuleIndex = ModuleChangeIndex;
        }
      theDefmodule = (struct defmodule *) GetNextDefmodule(NULL);
      theItem = (struct defmoduleItemHeader *)
                GetModuleItem(theDefmodule,DefglobalModuleIndex);
      theGlobal = (struct defglobal *) theItem->firstItem;
     }
   else
     { theGlobal = (struct defglobal *) GetNextDefglobal(theGlobal); }
   
   while (theDefmodule != NULL)
     {
      while (theGlobal != NULL)
        {
         if (theGlobal->inScope) return((VOID *) theGlobal);
         theGlobal = (struct defglobal *) GetNextDefglobal(theGlobal);
        }
        
      theDefmodule = (struct defmodule *) GetNextDefmodule(theDefmodule);
      theItem = (struct defmoduleItemHeader *)
                GetModuleItem(theDefmodule,DefglobalModuleIndex);
      theGlobal = (struct defglobal *) theItem->firstItem;
     }
     
   return(NULL);
  }

#endif /* DEFGLOBAL_CONSTRUCT */


