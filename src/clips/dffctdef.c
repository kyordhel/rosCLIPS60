   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                   DEFFACTS MODULE                   */
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

#define _DFFCTDEF_SOURCE_

#include "setup.h"

#if DEFFACTS_CONSTRUCT

#include <stdio.h>
#define _CLIPS_STDIO_

#include "clipsmem.h"
#include "dffctpsr.h"
#include "dffctbsc.h"

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
#include "bload.h"
#include "dffctbin.h"
#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
#include "dffctcmp.h"
#endif

#include "dffctdef.h"

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct construct       *DeffactsConstruct;
   globle int                     DeffactsModuleIndex;

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                   *AllocateModule(void);
   static VOID                    FreeModule(VOID *);
   static VOID                    ReturnDeffacts(VOID *);
   static VOID                    InitializeDeffactsModules(void);
#else
   static VOID                   *AllocateModule();
   static VOID                    FreeModule();
   static VOID                    ReturnDeffacts();
   static VOID                    InitializeDeffactsModules();
#endif

/***********************************************************/
/* InitializeDeffacts: Initializes the deffacts construct. */
/***********************************************************/
globle VOID InitializeDeffacts()
  {
   InitializeDeffactsModules(); 

   DeffactsBasicCommands();

   DeffactsConstruct = 
      AddConstruct("deffacts","deffacts",ParseDeffacts,FindDeffacts,
                   GetConstructNamePointer,GetConstructPPForm,
                   GetConstructModuleItem,GetNextDeffacts,SetNextConstruct,
                   IsDeffactsDeletable,Undeffacts,ReturnDeffacts);
  }

/*******************************************************/
/* InitializeDeffactsModules: Initializes the deffacts */
/*   construct for use with the defmodule construct.   */
/*******************************************************/
static VOID InitializeDeffactsModules()
  {
   DeffactsModuleIndex = RegisterModuleItem("deffacts",
                                    AllocateModule,
                                    FreeModule,
#if BLOAD_AND_BSAVE || BLOAD || BLOAD_ONLY
                                    BloadDeffactsModuleReference,
#else
                                    NULL,
#endif
#if CONSTRUCT_COMPILER && (! RUN_TIME)
                                    DeffactsCModuleReference,
#else
                                    NULL,
#endif
                                    FindDeffacts);
  }

/************************************************/
/* AllocateModule: Allocates a deffacts module. */
/************************************************/
static VOID *AllocateModule()
  { return((VOID *) get_struct(deffactsModule)); }
  
/**********************************************/
/* FreeModule: Deallocates a deffacts module. */ 
/**********************************************/
static VOID FreeModule(theItem)
  VOID *theItem;
  {
   FreeConstructHeaderModule(theItem,DeffactsConstruct);
   rtn_struct(deffactsModule,theItem);
  } 

/*************************************************************/
/* GetDeffactsModuleItem: Returns a pointer to the defmodule */
/*  item for the specified deffacts or defmodule.            */
/*************************************************************/
globle struct deffactsModule *GetDeffactsModuleItem(theModule)
  struct defmodule *theModule;
  { return((struct deffactsModule *) GetConstructModuleItemByIndex(theModule,DeffactsModuleIndex)); }
    
/*****************************************************************/
/* FindDeffacts: Searches for a deffact in the list of deffacts. */
/*   Returns a pointer to the deffact if found, otherwise NULL.  */
/*****************************************************************/
globle VOID *FindDeffacts(deffactsName)
  char *deffactsName;
  { return(FindNamedConstruct(deffactsName,DeffactsConstruct)); }

/****************************************************************/
/* GetNextDeffacts: If passed a NULL pointer, returns the first */
/*   deffacts in the ListOfDeffacts. Otherwise returns the next */
/*   deffacts following the deffacts passed as an argument.     */
/****************************************************************/
globle VOID *GetNextDeffacts(deffactsPtr)
  VOID *deffactsPtr;
  { return((VOID *) GetNextConstructItem(deffactsPtr,DeffactsModuleIndex)); }

/*******************************************************/
/* IsDeffactsDeletable: Returns TRUE if a particular   */
/*   deffacts can be deleted, otherwise returns FALSE. */
/*******************************************************/
#if IBM_TBC
#pragma argsused
#endif
globle BOOLEAN IsDeffactsDeletable(ptr)
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
   if (ResetInProgress) return(CLIPS_FALSE);
   return(CLIPS_TRUE);
#endif
  }
  
/***********************************************************/
/* ReturnDeffacts: Returns the data structures associated  */
/*   with a deffacts construct to the pool of free memory. */
/***********************************************************/
static VOID ReturnDeffacts(vTheDeffacts)
  VOID *vTheDeffacts;
  {
#if (! BLOAD_ONLY) && (! RUN_TIME)
   struct deffacts *theDeffacts = (struct deffacts *) vTheDeffacts;
   
   if (theDeffacts == NULL) return;

   ExpressionDeinstall(theDeffacts->assertList);
   ReturnPackedExpression(theDeffacts->assertList);

   DeinstallConstructHeader(&theDeffacts->header);
     
   rtn_struct(deffacts,theDeffacts);
#endif
  }

#endif /* DEFFACTS_CONSTRUCT */


