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
/* Purpose: Binary Load/Save Functions for Construct Headers */
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

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE

#include "bload.h"

#if BLOAD_AND_BSAVE
#include "bsave.h"
#endif

#include "moduldef.h"

#define _CSTRCBIN_SOURCE_
#include "cstrcbin.h"

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */
   
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

#if BLOAD_AND_BSAVE
#endif

#else

#if BLOAD_AND_BSAVE
#endif

#endif
      
/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */

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
      
#if BLOAD_AND_BSAVE

/***************************************************
  NAME         : MarkConstructHeaderNeededItems
  DESCRIPTION  : Marks symbols and other ephemerals
                 needed by a construct header, and
                 sets the binary-save id for the
                 construct
  INPUTS       : 1) The construct header
                 2) The binary-save id to assign
  RETURNS      : Nothing useful
  SIDE EFFECTS : Id set and items marked
  NOTES        : None
 ***************************************************/
globle VOID MarkConstructHeaderNeededItems(theConstruct,theBsaveID)
  struct constructHeader *theConstruct;
  long theBsaveID;
  {
   theConstruct->name->neededSymbol = CLIPS_TRUE;
   theConstruct->bsaveID = theBsaveID;
  }

/******************************************************
  NAME         : AssignBsaveConstructHeaderVals
  DESCRIPTION  : Assigns value to the construct
                 header for saving in the binary file
  INPUTS       : 1) The binary-save buffer for the
                    construct header values
                 2) The actual construct header
  RETURNS      : Nothing useful
  SIDE EFFECTS : Binary-save buffer for construct
                 header written with appropriate values
  NOTES        : Assumes that module items for this
                 construct were saved in the same
                 order as the defmodules.
                 The defmodule binary-save id is
                 used for the whichModule id of
                 this construct.
 ******************************************************/
globle VOID AssignBsaveConstructHeaderVals(theBsaveConstruct,theConstruct)
  struct bsaveConstructHeader *theBsaveConstruct;
  struct constructHeader *theConstruct;
  {
   theBsaveConstruct->name = (long) theConstruct->name->bucket;
   theBsaveConstruct->whichModule = theConstruct->whichModule->theModule->bsaveID;
   if (theConstruct->next != NULL)
     theBsaveConstruct->next = ((struct constructHeader *) theConstruct->next)->bsaveID;
   else
     theBsaveConstruct->next = -1L;
  }

#endif

/***************************************************
  NAME         : UpdateConstructHeader
  DESCRIPTION  : Determines field values for
                 construct header from binary-load
                 buffer
  INPUTS       : 1) The binary-load data for the
                    construct header
                 2) The actual construct header
                 3) The size of a defmodule item for
                    this construct
                 4) The array of all defmodule items
                    for this construct
                 5) The size of this construct
                 6) The array of these constructs
  RETURNS      : Nothing useful
  SIDE EFFECTS : Header values set
  NOTES        : None
 ***************************************************/
LOCALE VOID UpdateConstructHeader(theBsaveConstruct,theConstruct,
                                  itemModuleSize,itemModuleArray,
                                  itemSize,itemArray)
  struct bsaveConstructHeader *theBsaveConstruct;
  struct constructHeader *theConstruct;
  int itemModuleSize;
  VOID *itemModuleArray;
  int itemSize;
  VOID *itemArray;
  {
   long moduleOffset, itemOffset;
   
   moduleOffset = itemModuleSize * theBsaveConstruct->whichModule;
   theConstruct->whichModule =
     (struct defmoduleItemHeader *) &((char HUGE_ADDR *) itemModuleArray)[moduleOffset];
   theConstruct->name = SymbolPointer(theBsaveConstruct->name);
   IncrementSymbolCount(theConstruct->name);
   if (theBsaveConstruct->next != -1L)
     {
      itemOffset = itemSize * theBsaveConstruct->next;
      theConstruct->next = (struct constructHeader *) &((char HUGE_ADDR *) itemArray)[itemOffset];
     }
   else
     theConstruct->next = NULL;
   theConstruct->ppForm = NULL;
   theConstruct->bsaveID = 0L;
  }

/*******************************************************
  NAME         : UnmarkConstructHeader
  DESCRIPTION  : Releases any ephemerals (symbols, etc.)
                 of a construct header for removal
  INPUTS       : The construct header
  RETURNS      : Nothing useful
  SIDE EFFECTS : Busy counts fo ephemerals decremented
  NOTES        : None
 *******************************************************/
globle VOID UnmarkConstructHeader(theConstruct)
  struct constructHeader *theConstruct;
  {
   DecrementSymbolCount(theConstruct->name);
  }

/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

#if BLOAD_AND_BSAVE

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
