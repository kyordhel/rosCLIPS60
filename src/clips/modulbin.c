   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             DEFMODULE BSAVE/BLOAD MODULE            */
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

#define _MODULBIN_SOURCE_

#include "setup.h"

#if (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE) && (! RUN_TIME)

#include <stdio.h>
#define _CLIPS_STDIO_

#include "clipsmem.h"
#include "constrct.h"
#include "moduldef.h"
#include "bload.h"
#include "bsave.h"

#include "modulbin.h"

   
/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static long                             NumberOfDefmodules = 0;
   static long                             NumberOfPortItems = 0;
   static struct portItem HUGE_ADDR       *PortItemArray = NULL;

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct defmodule HUGE_ADDR      *DefmoduleArray = NULL;
   
/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
#if BLOAD_AND_BSAVE
   static VOID                    BsaveFind(void);
   static VOID                    BsaveStorage(FILE *);
   static VOID                    BsaveBinaryItem(FILE *);
#endif
   static VOID                    BloadStorage(void);
   static VOID                    BloadBinaryItem(void);
   static VOID                    UpdateDefmodule(VOID *,long);
   static VOID                    UpdatePortItem(VOID *,long);
   static VOID                    ClearBload(void);
#else
#if BLOAD_AND_BSAVE
   static VOID                    BsaveFind();
   static VOID                    BsaveStorage();
   static VOID                    BsaveBinaryItem();
#endif
   static VOID                    BloadStorage();
   static VOID                    BloadBinaryItem();
   static VOID                    UpdateDefmodule();
   static VOID                    UpdatePortItem();
   static VOID                    ClearBload();
#endif

/*********************************************/
/* DefmoduleBinarySetup: Installs the binary */
/*   save/load feature for defmodules.       */
/*********************************************/
globle VOID DefmoduleBinarySetup()
  {
   AddBeforeBloadFunction("defmodule",RemoveAllDefmodules,2000);

#if BLOAD_AND_BSAVE
   AddBinaryItem("defmodule",0,BsaveFind,NULL,
                             BsaveStorage,BsaveBinaryItem,
                             BloadStorage,BloadBinaryItem,
                             ClearBload);
#endif

   AddAbortBloadFunction("defmodule",CreateMainModule,0);

#if (BLOAD || BLOAD_ONLY)
   AddBinaryItem("defmodule",0,NULL,NULL,NULL,NULL,
                             BloadStorage,BloadBinaryItem,
                             ClearBload);
#endif
  }

/**************************************************************/
/* UpdateDefmoduleItemHeader: Updates the values in defmodule */
/*   item headers from the loaded binary image.               */
/**************************************************************/
globle VOID UpdateDefmoduleItemHeader(theBsaveHeader,theHeader,itemSize,itemArray)
  struct bsaveDefmoduleItemHeader *theBsaveHeader;
  struct defmoduleItemHeader *theHeader;
  int itemSize;
  VOID *itemArray;
  {
   long firstOffset,lastOffset;
   
   theHeader->theModule = ModulePointer(theBsaveHeader->theModule);
   if (theBsaveHeader->firstItem == -1L)
     {
      theHeader->firstItem = NULL;
      theHeader->lastItem = NULL;
     }
   else
     {
      firstOffset = itemSize * theBsaveHeader->firstItem;
      lastOffset = itemSize * theBsaveHeader->lastItem;
      theHeader->firstItem =
        (struct constructHeader *) &((char HUGE_ADDR *) itemArray)[firstOffset];
      theHeader->lastItem =
        (struct constructHeader *) &((char HUGE_ADDR *) itemArray)[lastOffset];
     }
  }

#if BLOAD_AND_BSAVE

/*****************************************************************/
/* AssignBsaveDefmdlItemHdrVals: Assigns the appropriate */
/*   values to a bsave defmodule item header record.             */
/*****************************************************************/
globle VOID AssignBsaveDefmdlItemHdrVals(theBsaveHeader,theHeader)
  struct bsaveDefmoduleItemHeader *theBsaveHeader;
  struct defmoduleItemHeader *theHeader;
  {
   theBsaveHeader->theModule = theHeader->theModule->bsaveID;
   if (theHeader->firstItem == NULL)
     {
      theBsaveHeader->firstItem = -1L;
      theBsaveHeader->lastItem = -1L;
     }
   else
     {
      theBsaveHeader->firstItem = theHeader->firstItem->bsaveID;
      theBsaveHeader->lastItem = theHeader->lastItem->bsaveID;
     }
  }

/**********************************************************/
/* BsaveFind: Counts the number of data structures which  */
/*   must be saved in the binary image for the defmodules */
/*   in the current environment.                          */
/**********************************************************/
static VOID BsaveFind()
  {
   struct defmodule *defmodulePtr;
   struct portItem *theList;

   if (Bloaded()) 
     {
      SaveBloadCount(NumberOfDefmodules);
      SaveBloadCount(NumberOfPortItems);
     }
     
   NumberOfDefmodules = 0;
   NumberOfPortItems = 0;
   
   defmodulePtr = (struct defmodule *) GetNextDefmodule(NULL);
   while (defmodulePtr != NULL)
     {
      NumberOfDefmodules++;
      defmodulePtr->name->neededSymbol = CLIPS_TRUE;
      
      for (theList = defmodulePtr->importList;
           theList != NULL;
           theList = theList->next)
        {
         NumberOfPortItems++;
         if (theList->moduleName != NULL) 
           { theList->moduleName->neededSymbol = CLIPS_TRUE; }
         if (theList->constructType != NULL) 
           { theList->constructType->neededSymbol = CLIPS_TRUE; }
         if (theList->constructName != NULL) 
           { theList->constructName->neededSymbol = CLIPS_TRUE; }
        }
      
      for (theList = defmodulePtr->exportList;
           theList != NULL;
           theList = theList->next)
        {
         NumberOfPortItems++;
         if (theList->moduleName != NULL) 
           { theList->moduleName->neededSymbol = CLIPS_TRUE; }
         if (theList->constructType != NULL) 
           { theList->constructType->neededSymbol = CLIPS_TRUE; }
         if (theList->constructName != NULL) 
           { theList->constructName->neededSymbol = CLIPS_TRUE; }
        }
      
      defmodulePtr = (struct defmodule *) GetNextDefmodule(defmodulePtr);
     }
  }

/*********************************************************/
/* BsaveStorage: Writes out the storage requirements for */
/*    all defmodule structures to the binary file.       */
/*********************************************************/
static VOID BsaveStorage(fp)
  FILE *fp;
  {
   unsigned long space;

   space = sizeof(long) * 2;
   GenWrite(&space,(unsigned long) sizeof(unsigned long int),fp);
   GenWrite(&NumberOfDefmodules,(unsigned long) sizeof(long int),fp);
   GenWrite(&NumberOfPortItems,(unsigned long) sizeof(long int),fp);
  }
  
/*********************************************/
/* BsaveBinaryItem: Writes out all defmodule */
/*   structures to the binary file.          */
/*********************************************/
static VOID BsaveBinaryItem(fp)
  FILE *fp;
  {
   unsigned long int space;
   struct defmodule *defmodulePtr;
   struct bsaveDefmodule newDefmodule;
   struct bsavePortItem newPortItem;
   struct portItem *theList;

   space = NumberOfDefmodules * sizeof(struct bsaveDefmodule);
   space += NumberOfPortItems * sizeof(struct bsavePortItem);
   GenWrite(&space,(unsigned long) sizeof(unsigned long int),fp);

   /*===========================*/
   /* Write out each defmodule. */
   /*===========================*/

   NumberOfDefmodules = 0;
   NumberOfPortItems = 0;
   defmodulePtr = (struct defmodule *) GetNextDefmodule(NULL);
   while (defmodulePtr != NULL)
     {
      newDefmodule.name = (unsigned long) defmodulePtr->name->bucket;

      NumberOfDefmodules++;
      if (defmodulePtr->next != NULL)
        { newDefmodule.next = NumberOfDefmodules; }
      else
        { newDefmodule.next = -1L; }

      if (defmodulePtr->importList == NULL)
        { newDefmodule.importList = -1L; }
      else
        {
         newDefmodule.importList = NumberOfPortItems;
         for (theList = defmodulePtr->importList;
              theList != NULL; 
              theList = theList->next)
           { NumberOfPortItems++; }
        }
        
      if (defmodulePtr->exportList == NULL)
        { newDefmodule.exportList = -1L; }
      else
        {
         newDefmodule.exportList = NumberOfPortItems;
         for (theList = defmodulePtr->exportList;
              theList != NULL; 
              theList = theList->next)
           { NumberOfPortItems++; }
        }
      
      newDefmodule.bsaveID = defmodulePtr->bsaveID; 
      GenWrite(&newDefmodule,(unsigned long) sizeof(struct bsaveDefmodule),fp);
      defmodulePtr = (struct defmodule *) GetNextDefmodule(defmodulePtr);
     }

   /*===========================*/
   /* Write out each port item. */
   /*===========================*/

   NumberOfPortItems = 0;
   defmodulePtr = (struct defmodule *) GetNextDefmodule(NULL);
   while (defmodulePtr != NULL)
     {
      for (theList = defmodulePtr->importList;
           theList != NULL; 
           theList = theList->next)
        { 
         NumberOfPortItems++; 
         if (theList->moduleName == NULL) newPortItem.moduleName = -1L;
         else newPortItem.moduleName = (unsigned long) theList->moduleName->bucket;
         
         if (theList->constructType == NULL) newPortItem.constructType = -1L;
         else newPortItem.constructType = (unsigned long) theList->constructType->bucket;
         
         if (theList->constructName == NULL) newPortItem.constructName = -1L;
         else newPortItem.constructName = (unsigned long) theList->constructName->bucket;
         
         if (theList->next == NULL) newPortItem.next = -1L;
         else newPortItem.next = NumberOfPortItems;
         
         GenWrite(&newPortItem,(unsigned long) sizeof(struct bsavePortItem),fp);
        }
        
      for (theList = defmodulePtr->exportList;
           theList != NULL; 
           theList = theList->next)
        { 
         NumberOfPortItems++; 
         if (theList->moduleName == NULL) newPortItem.moduleName = -1L;
         else newPortItem.moduleName = (unsigned long) theList->moduleName->bucket;
         
         if (theList->constructType == NULL) newPortItem.constructType = -1L;
         else newPortItem.constructType = (unsigned long) theList->constructType->bucket;
         
         if (theList->constructName == NULL) newPortItem.constructName = -1L;
         else newPortItem.constructName = (unsigned long) theList->constructName->bucket;
         
         if (theList->next == NULL) newPortItem.next = -1L;
         else newPortItem.next = NumberOfPortItems;
         
         GenWrite(&newPortItem,(unsigned long) sizeof(struct bsavePortItem),fp);
        }
        
      defmodulePtr = (struct defmodule *) GetNextDefmodule(defmodulePtr);
     }

   /*===========================*/
   /* Restore the bload counts. */
   /*===========================*/
        
   if (Bloaded()) 
     {
      RestoreBloadCount(&NumberOfDefmodules);
      RestoreBloadCount(&NumberOfPortItems);
     }
  }
  
#endif /* BLOAD_AND_BSAVE */

/************************************************/
/* BloadStorage: Loads storage requirements for */
/*   the defmodules used by this binary image.  */
/************************************************/
static VOID BloadStorage()
  {
   unsigned long int space;

   GenRead(&space,(unsigned long) sizeof(unsigned long int));
   GenRead(&NumberOfDefmodules,(unsigned long) sizeof(long int));
   GenRead(&NumberOfPortItems,(unsigned long) sizeof(long int));

   if (NumberOfDefmodules == 0)
     {
      DefmoduleArray = NULL;
      return;
     }

   space = (unsigned long) (NumberOfDefmodules * sizeof(struct defmodule));
   DefmoduleArray = (struct defmodule HUGE_ADDR *) genlongalloc(space);
   
   if (NumberOfPortItems == 0)
     {
      PortItemArray = NULL;
      return;
     }
     
   space = (unsigned long) (NumberOfPortItems * sizeof(struct portItem));
   PortItemArray = (struct portItem HUGE_ADDR *) genlongalloc(space);
  }

/********************************************/
/* BloadBinaryItem: Loads and refreshes the */
/*   defmodules used by this binary image.  */
/********************************************/
static VOID BloadBinaryItem()
  {
   unsigned long int space;

   GenRead(&space,(unsigned long) sizeof(unsigned long int));
   if (NumberOfDefmodules == 0) return;
   BloadandRefresh(NumberOfDefmodules,(unsigned) sizeof(struct bsaveDefmodule),UpdateDefmodule);
   BloadandRefresh(NumberOfPortItems,(unsigned) sizeof(struct bsavePortItem),UpdatePortItem);

   SetListOfDefmodules((VOID *) DefmoduleArray);
   SetCurrentModule((VOID *) GetNextDefmodule(NULL));
  }
  
/**********************************************************/
/* UpdateDefmodule: Bload refresh routine for defmodules. */
/**********************************************************/
static VOID UpdateDefmodule(buf,obji)
  VOID *buf;
  long obji;
  {
   struct bsaveDefmodule *bdp;
   struct moduleItem *theItem;
   int i;
   
   bdp = (struct bsaveDefmodule *) buf;
   DefmoduleArray[obji].name = SymbolPointer(bdp->name);
   IncrementSymbolCount(DefmoduleArray[obji].name);
   if (bdp->next != -1L)
     { DefmoduleArray[obji].next = (struct defmodule *) &DefmoduleArray[bdp->next]; }
   else
     { DefmoduleArray[obji].next = NULL; }
        
   DefmoduleArray[obji].itemsArray = (struct defmoduleItemHeader **) gm2((int) sizeof(VOID *) * GetNumberOfModuleItems());
   
   for (i = 0, theItem = GetListOfModuleItems();
        (i < GetNumberOfModuleItems()) && (theItem != NULL) ; 
        i++, theItem = theItem->next)
     { 
      if (theItem->bloadModuleReference == NULL)
        { DefmoduleArray[obji].itemsArray[i] = NULL; }
      else
        { 
         DefmoduleArray[obji].itemsArray[i] = 
             (struct defmoduleItemHeader *)
             (*theItem->bloadModuleReference)(obji);
        }
     } 
   
   DefmoduleArray[obji].ppForm = NULL;
   
   if (bdp->importList != -1L) 
     { DefmoduleArray[obji].importList = (struct portItem *) &PortItemArray[bdp->importList]; }
   else 
     { DefmoduleArray[obji].importList = NULL; }

   if (bdp->exportList != -1L) 
     { DefmoduleArray[obji].exportList = (struct portItem *) &PortItemArray[bdp->exportList]; }
   else 
     { DefmoduleArray[obji].exportList = NULL; }
   DefmoduleArray[obji].bsaveID = bdp->bsaveID;
  }
  
/*********************************************************/
/* UpdatePortItem: Bload refresh routine for port items. */
/*********************************************************/
static VOID UpdatePortItem(buf,obji)
  VOID *buf;
  long obji;
  {
   struct bsavePortItem *bdp;
   
   bdp = (struct bsavePortItem *) buf;
   
   if (bdp->moduleName != -1L)
     { 
      PortItemArray[obji].moduleName = SymbolPointer(bdp->moduleName);
      IncrementSymbolCount(PortItemArray[obji].moduleName);
     }
   else
     { PortItemArray[obji].moduleName = NULL; }

   if (bdp->constructType != -1L)
     { 
      PortItemArray[obji].constructType = SymbolPointer(bdp->constructType);
      IncrementSymbolCount(PortItemArray[obji].constructType);
     }
   else
     { PortItemArray[obji].constructType = NULL; }

   if (bdp->constructName != -1L)
     { 
      PortItemArray[obji].constructName = SymbolPointer(bdp->constructName);
      IncrementSymbolCount(PortItemArray[obji].constructName);
     }
   else
     { PortItemArray[obji].constructName = NULL; }

   if (bdp->next != -1L)
     { PortItemArray[obji].next = (struct portItem *) &PortItemArray[bdp->next]; }
   else
     { PortItemArray[obji].next = NULL; }
  }
  
/***********************************************************************/
/* ClearBload: Defmodule clear routine when a binary load is in effect. */
/***********************************************************************/
static VOID ClearBload()
  {
   long i;
   unsigned long space;
   struct portItem *theList;

   for (i = 0; i < NumberOfDefmodules; i++)
     { 
      DecrementSymbolCount(DefmoduleArray[i].name);
      for (theList = DefmoduleArray[i].importList;
           theList != NULL;
           theList = theList->next)
        {
         if (theList->moduleName != NULL) DecrementSymbolCount(theList->moduleName);
         if (theList->constructType != NULL) DecrementSymbolCount(theList->constructType);
         if (theList->constructName != NULL) DecrementSymbolCount(theList->constructName);
        }
        
      for (theList = DefmoduleArray[i].exportList;
           theList != NULL;
           theList = theList->next)
        {
         if (theList->moduleName != NULL) DecrementSymbolCount(theList->moduleName);
         if (theList->constructType != NULL) DecrementSymbolCount(theList->constructType);
         if (theList->constructName != NULL) DecrementSymbolCount(theList->constructName);
        }
        
      rm(DefmoduleArray[i].itemsArray,(int) sizeof(VOID *) * GetNumberOfModuleItems());
     }

   space = NumberOfDefmodules * sizeof(struct defmodule);
   if (space != 0) genlongfree((VOID *) DefmoduleArray,space);
   
   space = NumberOfPortItems * sizeof(struct portItem);
   if (space != 0) genlongfree((VOID *) PortItemArray,space);

   SetListOfDefmodules(NULL);
   CreateMainModule();
   MainModuleRedefinable = CLIPS_TRUE;
  }

#endif /*  (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE) && (! RUN_TIME) */


