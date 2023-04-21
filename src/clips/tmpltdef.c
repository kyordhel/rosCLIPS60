   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 DEFTEMPLATE MODULE                  */
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

#define _TMPLTDEF_SOURCE_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT

#include <stdio.h>
#define _CLIPS_STDIO_

#include "clipsmem.h"
#include "exprnops.h"
#include "cstrccom.h"
#include "network.h"
#include "tmpltpsr.h"
#include "tmpltbsc.h"
#include "tmpltcom.h"
#include "router.h"
#include "modulpsr.h"
#include "modulutl.h"

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
#include "bload.h"
#include "tmpltbin.h"
#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
#include "tmpltcmp.h"
#endif

#include "tmpltdef.h"


/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                   *AllocateModule(void);
   static VOID                    FreeModule(VOID *);
   static VOID                    ReturnDeftemplate(VOID *);
   static VOID                    InitializeDeftemplateModules(void);
   static VOID                    IncrementDeftemplateBusyCount(VOID *);
   static VOID                    DecrementDeftemplateBusyCount(VOID *);
#else
   static VOID                   *AllocateModule();
   static VOID                    FreeModule();
   static VOID                    ReturnDeftemplate();
   static VOID                    InitializeDeftemplateModules();
   static VOID                    IncrementDeftemplateBusyCount();
   static VOID                    DecrementDeftemplateBusyCount();
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct construct       *DeftemplateConstruct;
   globle int                     DeftemplateModuleIndex;
   globle struct entityRecord     DeftemplatePtrRecord = { DEFTEMPLATE_PTR,1,0,0,
                                                           NULL,
                                                           NULL,NULL, 
                                                           NULL,
                                                           NULL,
                                                           DecrementDeftemplateBusyCount,
                                                           IncrementDeftemplateBusyCount,
                                                           NULL,NULL,NULL,NULL };

/******************************************************************/
/* InitializeDeftemplates: Initializes the deftemplate construct. */
/******************************************************************/
globle VOID InitializeDeftemplates()
  {
   InitializeFacts();
   
   InitializeDeftemplateModules(); 

   DeftemplateBasicCommands();
   
   DeftemplateCommands();

   DeftemplateConstruct = 
      AddConstruct("deftemplate","deftemplates",ParseDeftemplate,FindDeftemplate,
                   GetConstructNamePointer,GetConstructPPForm,
                   GetConstructModuleItem,GetNextDeftemplate,SetNextConstruct,
                   IsDeftemplateDeletable,Undeftemplate,ReturnDeftemplate);
                   
   InstallPrimitive((ENTITY_RECORD_PTR) &DeftemplatePtrRecord,DEFTEMPLATE_PTR);
  }

/*************************************************************/
/* InitializeDeftemplateModules: Initializes the deftemplate */
/*   construct for use with the defmodule construct.         */
/*************************************************************/
static VOID InitializeDeftemplateModules()
  {
   DeftemplateModuleIndex = RegisterModuleItem("deftemplate",
                                    AllocateModule,
                                    FreeModule,
#if BLOAD_AND_BSAVE || BLOAD || BLOAD_ONLY
                                    BloadDeftemplateModuleReference,
#else
                                    NULL,
#endif
#if CONSTRUCT_COMPILER && (! RUN_TIME)
                                    DeftemplateCModuleReference,
#else
                                    NULL,
#endif
                                    FindDeftemplate);
                                    
#if (! BLOAD_ONLY) && (! RUN_TIME) && DEFMODULE_CONSTRUCT
   AddPortConstructItem("deftemplate",SYMBOL);
#endif
  }

/***************************************************/
/* AllocateModule: Allocates a deftemplate module. */
/***************************************************/
static VOID *AllocateModule()
  { return((VOID *) get_struct(deftemplateModule)); }
  
/*************************************************/
/* FreeModule: Deallocates a deftemplate module. */ 
/*************************************************/
static VOID FreeModule(theItem)
  VOID *theItem;
  {
   FreeConstructHeaderModule(theItem,DeftemplateConstruct);
   rtn_struct(deftemplateModule,theItem);
  } 

/****************************************************************/
/* GetDeftemplateModuleItem: Returns a pointer to the defmodule */
/*  item for the specified deftemplate or defmodule.            */
/****************************************************************/
globle struct deftemplateModule *GetDeftemplateModuleItem(theModule)
  struct defmodule *theModule;
  { return((struct deftemplateModule *) GetConstructModuleItemByIndex(theModule,DeftemplateModuleIndex)); }
    
/***********************************************************/
/* FindDeftemplate: Searches for a deftemplate in the list */
/*   of deftemplates. Returns a pointer to the deftemplate */
/*   if found, otherwise NULL.                             */
/***********************************************************/
globle VOID *FindDeftemplate(deftemplateName)
  char *deftemplateName;
  { return(FindNamedConstruct(deftemplateName,DeftemplateConstruct)); }

/***********************************************************************/
/* GetNextDeftemplate: If passed a NULL pointer, returns the first     */
/*   deftemplate in the ListOfDeftemplates. Otherwise returns the next */
/*   deftemplate following the deftemplate passed as an argument.      */
/***********************************************************************/
globle VOID *GetNextDeftemplate(deftemplatePtr)
  VOID *deftemplatePtr;
  { return((VOID *) GetNextConstructItem(deftemplatePtr,DeftemplateModuleIndex)); }

/**********************************************************/
/* IsDeftemplateDeletable: Returns TRUE if a particular   */
/*   deftemplate can be deleted, otherwise returns FALSE. */
/**********************************************************/
globle BOOLEAN IsDeftemplateDeletable(vTheDeftemplate)
  VOID *vTheDeftemplate;
  {
#if BLOAD_ONLY || RUN_TIME
   return(FALSE);
#else
   struct deftemplate *theDeftemplate = (struct deftemplate *) vTheDeftemplate;
#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded()) return(FALSE);
#endif

   if (theDeftemplate->busyCount > 0) return(CLIPS_FALSE);
   if (theDeftemplate->patternNetwork != NULL) return(CLIPS_FALSE);
   
   return(CLIPS_TRUE);
#endif
  }
  
/**************************************************************/
/* ReturnDeftemplate: Returns the data structures associated  */
/*   with a deftemplate construct to the pool of free memory. */
/**************************************************************/
static VOID ReturnDeftemplate(vTheConstruct)
  VOID *vTheConstruct;
  {
#if (! BLOAD_ONLY) && (! RUN_TIME)
   struct deftemplate *theConstruct = (struct deftemplate *) vTheConstruct;
   struct templateSlot *slotPtr;
   
   if (theConstruct == NULL) return;
   
   /*====================================================================*/
   /* If a template is redefined, then we want to save its debug status. */
   /*====================================================================*/

#if DEBUGGING_FUNCTIONS
   DeletedTemplateDebugFlags = 0;
   if (theConstruct->watch) BitwiseSet(DeletedTemplateDebugFlags,0);
#endif

   /*===========================================*/
   /* Free storage used by the templates slots. */
   /*===========================================*/
   
   slotPtr = theConstruct->slotList;
   while (slotPtr != NULL)
     {
      DecrementSymbolCount(slotPtr->slotName);
      RemoveHashedExpression(slotPtr->defaultList);
      slotPtr->defaultList = NULL;
      RemoveConstraint(slotPtr->constraints);
      slotPtr = slotPtr->next;
     }
     
   ReturnSlots(theConstruct->slotList);
   
   /*==================================*/
   /* Free storage used by the header. */
   /*==================================*/
   
   DeinstallConstructHeader(&theConstruct->header);
     
   rtn_struct(deftemplate,theConstruct);
#endif
  }
  
/************************************************************/
/* DecrementDeftemplateBusyCount: Decrements the busy count */
/*   of a deftemplate data structure.                       */
/************************************************************/
static VOID DecrementDeftemplateBusyCount(vTheTemplate)
  VOID *vTheTemplate;
  {
   struct deftemplate *theTemplate = (struct deftemplate *) vTheTemplate;
   
   if (! ClearInProgress) theTemplate->busyCount--;
  }

/************************************************************/
/* IncrementDeftemplateBusyCount: Increments the busy count */
/*   of a deftemplate data structure.                       */
/************************************************************/
static VOID IncrementDeftemplateBusyCount(vTheTemplate)
  VOID *vTheTemplate;
  {
   struct deftemplate *theTemplate = (struct deftemplate *) vTheTemplate;
   
   theTemplate->busyCount++;
  } 
    
/*******************************************************************/
/* PrintTemplateFact: Prints a fact using the deftemplate format.  */
/*   Returns CLIPS_TRUE if the fact was printed using this format, */
/*   otherwise CLIPS_FALSE.                                        */
/*******************************************************************/
globle VOID PrintTemplateFact(logicalName,theFact)
  char *logicalName;
  struct fact *theFact;
  {
   struct field *sublist;
   int i;
   struct deftemplate *theDeftemplate;
   struct templateSlot *slotPtr;

   /*==============================*/
   /* Initialize some information. */
   /*==============================*/
   
   theDeftemplate = theFact->whichDeftemplate;
   sublist = theFact->theProposition.theFields;

   /*=============================================*/
   /* Print the relation name of the deftemplate. */
   /*=============================================*/

   PrintCLIPS(logicalName,"("); 
   PrintCLIPS(logicalName,theDeftemplate->header.name->contents);
   if (theDeftemplate->slotList != NULL) PrintCLIPS(logicalName," ");

   /*===================================================*/
   /* Print each of the field slots of the deftemplate. */
   /*===================================================*/

   slotPtr = theDeftemplate->slotList;

   i = 0;
   while (slotPtr != NULL)
     {
      /*===========================================*/
      /* Print the closing parenthesis of the slot */
      /* and the slot name.                        */
      /*===========================================*/

      PrintCLIPS(logicalName,"(");
      PrintCLIPS(logicalName,slotPtr->slotName->contents);

      /*======================================================*/
      /* Print the value of the slot for a single field slot. */
      /*======================================================*/

      if (slotPtr->multislot == CLIPS_FALSE)
        { 
         PrintCLIPS(logicalName," ");
         PrintAtom(logicalName,sublist[i].type,sublist[i].value); 
        }

      /*==========================================================*/
      /* Else print the value of the slot for a multi field slot. */
      /*==========================================================*/

      else
        { 
         struct multifield *theSegment;
         
         theSegment = (struct multifield *) sublist[i].value;
         if (theSegment->multifieldLength > 0)
           {
            PrintCLIPS(logicalName," ");
            PrintMultifield(logicalName,sublist[i].value,0,theSegment->multifieldLength-1,CLIPS_FALSE);
           }
        }

      /*============================================*/
      /* Print the closing parenthesis of the slot. */
      /*============================================*/

      i++;
      PrintCLIPS(logicalName,")");
      slotPtr = slotPtr->next;
      if (slotPtr != NULL) PrintCLIPS(logicalName," ");
     }
     
   PrintCLIPS(logicalName,")");
  }

/***************************************************************************/
/* UpdateDeftemplateScope: Updates the scope flag of all the deftemplates. */
/***************************************************************************/
globle VOID UpdateDeftemplateScope()
  {
   struct deftemplate *theDeftemplate;
   int moduleCount;
   struct defmodule *theModule;
   struct defmoduleItemHeader *theItem;
   
   for (theModule = (struct defmodule *) GetNextDefmodule(NULL);
        theModule != NULL;
        theModule = (struct defmodule *) GetNextDefmodule(theModule))
     {
      theItem = (struct defmoduleItemHeader *)
                GetModuleItem(theModule,DeftemplateModuleIndex);
      
      for (theDeftemplate = (struct deftemplate *) theItem->firstItem;
           theDeftemplate != NULL ;
           theDeftemplate = (struct deftemplate *) GetNextDeftemplate(theDeftemplate))
        {  
         if (FindImportedConstruct("deftemplate",theModule,
                                   ValueToString(theDeftemplate->header.name),
                                   &moduleCount,CLIPS_TRUE,NULL) != NULL)
           { theDeftemplate->inScope = CLIPS_TRUE; }
         else
           { theDeftemplate->inScope = CLIPS_FALSE; }
        }  
     }
  }
  
/****************************************************************/
/* FindSlot: Finds a specified slot in a deftemplate structure. */
/****************************************************************/
globle struct templateSlot *FindSlot(theDeftemplate,name,whichOne)
  struct deftemplate *theDeftemplate;
  SYMBOL_HN *name;
  int *whichOne;
  {
   struct templateSlot *slotPtr;

   *whichOne = 1;
   slotPtr = theDeftemplate->slotList;
   while (slotPtr != NULL)
     {
      if (slotPtr->slotName == name)
        { return(slotPtr); }
      (*whichOne)++;
      slotPtr = slotPtr->next;
     }

   *whichOne = -1;
   return(NULL);
  } 

/****************************************************************/
/* InvalidDeftemplateSlotMessage:  */
/****************************************************************/
globle VOID InvalidDeftemplateSlotMessage(slotName,deftemplateName)
  char *slotName;
  char *deftemplateName;
  {
   PrintErrorID("TMPLTDEF",1,CLIPS_TRUE);
   PrintCLIPS(WERROR,"Invalid slot ");
   PrintCLIPS(WERROR,slotName);
   PrintCLIPS(WERROR," not defined in corresponding deftemplate ");
   PrintCLIPS(WERROR,deftemplateName);
   PrintCLIPS(WERROR,".\n");
  }
  
/****************************************************************/
/* SingleFieldSlotCardinalityError:  */
/****************************************************************/
globle VOID SingleFieldSlotCardinalityError(slotName)
  char *slotName;
  {
   PrintErrorID("TMPLTDEF",2,CLIPS_TRUE);
   PrintCLIPS(WERROR,"The single field slot ");
   PrintCLIPS(WERROR,slotName);
   PrintCLIPS(WERROR," can only contain a single field value.\n");
  }

#endif /* DEFTEMPLATE_CONSTRUCT */


