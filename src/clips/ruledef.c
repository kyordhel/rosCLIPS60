   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                   DEFRULE MODULE                    */
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

#define _RULEDEF_SOURCE_

#include "setup.h"

#if DEFRULE_CONSTRUCT

#include <stdio.h>
#define _CLIPS_STDIO_

#include "clipsmem.h"
#include "engine.h"
#include "pattern.h"
#include "rulebsc.h"
#include "rulecom.h"
#include "drive.h"
#include "rulepsr.h"
#include "ruledlt.h"
#include "agenda.h"

#if BLOAD || BLOAD_AND_BSAVE || BLOAD_ONLY
#include "bload.h"
#include "rulebin.h"
#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
#include "rulecmp.h"
#endif

#include "ruledef.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                   *AllocateModule(void);
   static VOID                    FreeModule(VOID *);
   static VOID                    InitializeDefruleModules(void);
#else
   static VOID                   *AllocateModule();
   static VOID                    FreeModule();
   static VOID                    InitializeDefruleModules();
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct construct       *DefruleConstruct;
   globle int                     DefruleModuleIndex;
   globle long                    CurrentEntityTimeTag = 0L;

/**********************************************************/
/* InitializeDefrules: Initializes the defrule construct. */
/**********************************************************/
globle VOID InitializeDefrules()
  {
   InitializeEngine();
   InitializeAgenda();
      
   InitializeDefruleModules(); 

   AddReservedPatternSymbol("and",NULL);
   AddReservedPatternSymbol("not",NULL);
   AddReservedPatternSymbol("or",NULL);
   AddReservedPatternSymbol("test",NULL);
   AddReservedPatternSymbol("logical",NULL);
   AddReservedPatternSymbol("exists",NULL);
   AddReservedPatternSymbol("forall",NULL);
   
   DefruleBasicCommands();
   
   DefruleCommands();

   DefruleConstruct = 
      AddConstruct("defrule","defrules",
                   ParseDefrule,FindDefrule,
                   GetConstructNamePointer,GetConstructPPForm,
                   GetConstructModuleItem,GetNextDefrule,SetNextConstruct,
                   IsDefruleDeletable,Undefrule,ReturnDefrule);
  }

/*****************************************************/
/* InitializeDefruleModules: Initializes the defrule */
/*   construct for use with the defmodule construct. */
/*****************************************************/
static VOID InitializeDefruleModules()
  {
   DefruleModuleIndex = RegisterModuleItem("defrule",
                                    AllocateModule,
                                    FreeModule,
#if BLOAD_AND_BSAVE || BLOAD || BLOAD_ONLY
                                    BloadDefruleModuleReference,
#else
                                    NULL,
#endif
#if CONSTRUCT_COMPILER && (! RUN_TIME)
                                    DefruleCModuleReference,
#else
                                    NULL,
#endif
                                    FindDefrule);
  }

/***********************************************/
/* AllocateModule: Allocates a defrule module. */
/***********************************************/
static VOID *AllocateModule()
  { 
   struct defruleModule *theItem;
   
   theItem = get_struct(defruleModule);
   theItem->agenda = NULL;
   return((VOID *) theItem); 
  }
  
/*********************************************/
/* FreeModule: Deallocates a defrule module. */ 
/*********************************************/
static VOID FreeModule(theItem)
  VOID *theItem;
  {
   FreeConstructHeaderModule(theItem,DefruleConstruct);
   rtn_struct(defruleModule,theItem);
  } 

/************************************************************/
/* GetDefruleModuleItem: Returns a pointer to the defmodule */
/*  item for the specified defrule or defmodule.            */
/************************************************************/
globle struct defruleModule *GetDefruleModuleItem(theModule)
  struct defmodule *theModule;
  { return((struct defruleModule *) GetConstructModuleItemByIndex(theModule,DefruleModuleIndex)); }
    
/****************************************************************/
/* FindDefrule: Searches for a defrule in the list of defrules. */
/*   Returns a pointer to the defrule if found, otherwise NULL. */
/****************************************************************/
globle VOID *FindDefrule(defruleName)
  char *defruleName;
  { return(FindNamedConstruct(defruleName,DefruleConstruct)); }

/***************************************************************/
/* GetNextDefrule: If passed a NULL pointer, returns the first */
/*   defrule in the ListOfDefrules. Otherwise returns the next */
/*   defrule following the defrule passed as an argument.      */
/***************************************************************/
globle VOID *GetNextDefrule(defrulePtr)
  VOID *defrulePtr;
  { return((VOID *) GetNextConstructItem(defrulePtr,DefruleModuleIndex)); }

/******************************************************/
/* IsDefruleDeletable: Returns TRUE if a particular   */
/*   defrule can be deleted, otherwise returns FALSE. */
/******************************************************/
globle BOOLEAN IsDefruleDeletable(vTheDefrule)
  VOID *vTheDefrule;
  {
#if BLOAD_ONLY || RUN_TIME
   return(FALSE);
#else
   struct defrule *theDefrule = (struct defrule *) vTheDefrule;
#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded()) return(FALSE);
#endif
   
   if (theDefrule->executing) return(CLIPS_FALSE);

   if (JoinOperationInProgress) return(CLIPS_FALSE);
   
   return(CLIPS_TRUE);
#endif
  }

#endif /* DEFRULE_CONSTRUCT */


