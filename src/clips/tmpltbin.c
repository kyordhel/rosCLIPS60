   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            DEFTEMPLATE BSAVE/BLOAD MODULE           */
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

#define  _TMPLTBIN_SOURCE_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT && (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE) && (! RUN_TIME)

#include <stdio.h>
#define _CLIPS_STDIO_

#include "clipsmem.h"
#include "bload.h"
#include "bsave.h"
#include "factbin.h"
#include "cstrnbin.h"
#include "factmngr.h"
#include "tmpltpsr.h"
#include "tmpltdef.h"

#include "tmpltbin.h"

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct deftemplate HUGE_ADDR        *DeftemplateArray;
   
/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static long                                 NumberOfDeftemplates;
   static long                                 NumberOfTemplateSlots;
   static long                                 NumberOfTemplateModules;
   static struct templateSlot HUGE_ADDR       *SlotArray;
   static struct deftemplateModule HUGE_ADDR  *ModuleArray;

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
   static VOID                    UpdateDeftemplateModule(VOID *,long);
   static VOID                    UpdateDeftemplate(VOID *,long);
   static VOID                    UpdateDeftemplateSlot(VOID *,long);
   static VOID                    ClearBload(void);
#else
#if BLOAD_AND_BSAVE
   static VOID                    BsaveFind();
   static VOID                    BsaveStorage();
   static VOID                    BsaveBinaryItem();
#endif
   static VOID                    BloadStorage();
   static VOID                    BloadBinaryItem();
   static VOID                    UpdateDeftemplateModule();
   static VOID                    UpdateDeftemplate();
   static VOID                    UpdateDeftemplateSlot();
   static VOID                    ClearBload();
#endif

/***********************************************/
/* DeftemplateBinarySetup: Installs the binary */
/*   save/load feature for deftemplates.       */
/***********************************************/
globle VOID DeftemplateBinarySetup()
  {
#if BLOAD_AND_BSAVE
   AddBinaryItem("deftemplate",0,BsaveFind,NULL,
                             BsaveStorage,BsaveBinaryItem,
                             BloadStorage,BloadBinaryItem,
                             ClearBload);
#endif
#if (BLOAD || BLOAD_ONLY)
   AddBinaryItem("deftemplate",0,NULL,NULL,NULL,NULL,
                             BloadStorage,BloadBinaryItem,
                             ClearBload);
#endif
  }

#if BLOAD_AND_BSAVE

/****************************************************************/
/* BsaveFind: Find expressions function for deftemplate bsaves. */
/****************************************************************/
static VOID BsaveFind()
  {
   struct deftemplate *theDeftemplate;
   struct templateSlot *tsPtr;
   struct defmodule *theModule;

   if (Bloaded())
     {
      SaveBloadCount(NumberOfDeftemplates);
      SaveBloadCount(NumberOfTemplateSlots);
      SaveBloadCount(NumberOfTemplateModules);
     }
     
   NumberOfDeftemplates = 0;
   NumberOfTemplateSlots = 0;
   NumberOfTemplateModules = 0;
   
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
      NumberOfTemplateModules++;
      theDeftemplate = (struct deftemplate *) GetNextDeftemplate(NULL);
      while (theDeftemplate != NULL)
        {
         MarkConstructHeaderNeededItems(&theDeftemplate->header,
                                        NumberOfDeftemplates++);
         tsPtr = theDeftemplate->slotList;
         while (tsPtr != NULL)
           {
            NumberOfTemplateSlots++;
            tsPtr->slotName->neededSymbol = CLIPS_TRUE;
            tsPtr = tsPtr->next;
           }
         theDeftemplate = (struct deftemplate *) GetNextDeftemplate(theDeftemplate);
        }
        
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }
  }

/*********************************************************/
/* BsaveStorage: Writes out the storage requirements for */
/*   all deftemplate data structures to the beginning of */
/*   the binary file.                                    */
/*********************************************************/
static VOID BsaveStorage(fp)
  FILE *fp;
  {
   unsigned long space;

   space = sizeof(long) * 3;
   GenWrite(&space,(unsigned long) sizeof(long int),fp);
   GenWrite(&NumberOfDeftemplates,(unsigned long) sizeof(long int),fp);
   GenWrite(&NumberOfTemplateSlots,(unsigned long) sizeof(long int),fp);
   GenWrite(&NumberOfTemplateModules,(unsigned long) sizeof(long int),fp);
  }

/***********************************************/
/* BsaveBinaryItem: Writes out all deftemplate */
/*   structures to the binary file.            */
/***********************************************/
static VOID BsaveBinaryItem(fp)
  FILE *fp;
  {
   unsigned long space;
   struct deftemplate *theDeftemplate;
   struct bsaveDeftemplate tempDeftemplate;
   struct templateSlot *tsPtr;
   struct bsaveTemplateSlot tempTemplateSlot;
   struct bsaveDeftemplateModule tempTemplateModule;
   struct defmodule *theModule;
   struct deftemplateModule *theModuleItem;
   
   /*=============================================*/
   /* Determine the total amount of space needed. */
   /*=============================================*/
   
   space = (NumberOfDeftemplates * sizeof(struct bsaveDeftemplate)) +
           (NumberOfTemplateSlots * sizeof(struct bsaveTemplateSlot)) +
           (NumberOfTemplateModules * sizeof(struct bsaveDeftemplateModule));
   GenWrite(&space,(unsigned long) sizeof(unsigned long int),fp);
   
   /*===================================================*/
   /* Write out each deftemplate module data structure. */
   /*===================================================*/

   NumberOfDeftemplates = 0;
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
      
      theModuleItem = (struct deftemplateModule *) 
                      GetModuleItem(NULL,FindModuleItem("deftemplate")->moduleIndex);
      AssignBsaveDefmdlItemHdrVals(&tempTemplateModule.header,
                                           &theModuleItem->header);
      GenWrite(&tempTemplateModule,(unsigned long) sizeof(struct bsaveDeftemplateModule),fp);
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }
     
   /*============================================*/
   /* Write out each deftemplate data structure. */
   /*============================================*/

   NumberOfTemplateSlots = 0;
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
      
      theDeftemplate = (struct deftemplate *) GetNextDeftemplate(NULL);
      while (theDeftemplate != NULL)
        {
         AssignBsaveConstructHeaderVals(&tempDeftemplate.header,
                                          &theDeftemplate->header);
         tempDeftemplate.implied = theDeftemplate->implied;
         tempDeftemplate.numberOfSlots = theDeftemplate->numberOfSlots;
         tempDeftemplate.patternNetwork = BsaveFactPatternIndex(theDeftemplate->patternNetwork);

         if (theDeftemplate->slotList != NULL)
           { tempDeftemplate.slotList = NumberOfTemplateSlots; }
         else tempDeftemplate.slotList = -1L;

         GenWrite(&tempDeftemplate,(unsigned long) sizeof(struct bsaveDeftemplate),fp);

         NumberOfTemplateSlots += theDeftemplate->numberOfSlots;

         theDeftemplate = (struct deftemplate *) GetNextDeftemplate(theDeftemplate);
        }
      
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }

   /*==============================================*/
   /* Write out each template slot data structure. */
   /*==============================================*/

   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
   
      theDeftemplate = (struct deftemplate *) GetNextDeftemplate(NULL);
      while (theDeftemplate != NULL)
        {
         tsPtr = theDeftemplate->slotList;
         while (tsPtr != NULL)
           {
            tempTemplateSlot.constraints = ConstraintIndex(tsPtr->constraints); 
            tempTemplateSlot.slotName = (long) tsPtr->slotName->bucket;
            tempTemplateSlot.multislot = tsPtr->multislot;
            tempTemplateSlot.noDefault = tsPtr->noDefault;
            tempTemplateSlot.defaultPresent = tsPtr->defaultPresent;
            tempTemplateSlot.defaultDynamic = tsPtr->defaultDynamic;
            tempTemplateSlot.defaultList = HashedExpressionIndex(tsPtr->defaultList);

            if (tsPtr->next != NULL) tempTemplateSlot.next = 0L;
            else tempTemplateSlot.next = -1L;

            GenWrite(&tempTemplateSlot,(unsigned long) sizeof(struct bsaveTemplateSlot),fp);
            tsPtr = tsPtr->next;
           }

         theDeftemplate = (struct deftemplate *) GetNextDeftemplate(theDeftemplate);
        }
        
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }
     
   if (Bloaded())
     {
      RestoreBloadCount(&NumberOfDeftemplates);
      RestoreBloadCount(&NumberOfTemplateSlots);
      RestoreBloadCount(&NumberOfTemplateModules);
     }
  }
 
#endif /* BLOAD_AND_BSAVE */

/***********************************************/
/* BloadStorage: Allocates the storage for the */
/*   deftemplates used by this binary image.   */
/***********************************************/
static VOID BloadStorage()
  {
   unsigned long int space;

   /*===========================================*/
   /* Determine the number of items to be read. */
   /*===========================================*/
   
   GenRead(&space,(unsigned long) sizeof(unsigned long int));
   GenRead(&NumberOfDeftemplates,(unsigned long) sizeof(long int));
   GenRead(&NumberOfTemplateSlots,(unsigned long) sizeof(long int));
   GenRead(&NumberOfTemplateModules,(unsigned long) sizeof(long int));
   
   /*==============================================*/
   /* Read the deftemplate module data structures. */
   /*==============================================*/
   
   if (NumberOfTemplateModules == 0)
     {
      DeftemplateArray = NULL;
      SlotArray = NULL;
      ModuleArray = NULL;
     }
     
   space = NumberOfTemplateModules * sizeof(struct deftemplateModule);
   ModuleArray = (struct deftemplateModule HUGE_ADDR *) genlongalloc(space);
   
   /*=======================================*/
   /* Read the deftemplate data structures. */
   /*=======================================*/
   
   if (NumberOfDeftemplates == 0)
     {
      DeftemplateArray = NULL;
      SlotArray = NULL;
      return;
     }

   space = NumberOfDeftemplates * sizeof(struct deftemplate);
   DeftemplateArray = (struct deftemplate HUGE_ADDR *) genlongalloc(space);

   /*============================================*/
   /* Read the deftemplate slot data structures. */
   /*============================================*/
   
   if (NumberOfTemplateSlots == 0)
     {
      SlotArray = NULL;
      return;
     }

   space =  NumberOfTemplateSlots * sizeof(struct templateSlot);
   SlotArray = (struct templateSlot HUGE_ADDR *) genlongalloc(space);
  }

/*******************************************************/
/* BloadBinaryItem: Loads and updates the deftemplates */
/*   used by this binary image.                        */
/*******************************************************/
static VOID BloadBinaryItem()
  {
   unsigned long int space;

   GenRead(&space,(unsigned long) sizeof(unsigned long int));
   BloadandRefresh(NumberOfTemplateModules,(unsigned) sizeof(struct bsaveDeftemplateModule),
                   UpdateDeftemplateModule);
   BloadandRefresh(NumberOfDeftemplates,(unsigned) sizeof(struct bsaveDeftemplate),
                   UpdateDeftemplate);
   BloadandRefresh(NumberOfTemplateSlots,(unsigned) sizeof(struct bsaveTemplateSlot),
                   UpdateDeftemplateSlot);
  }
  
/*************************************************/
/* UpdateDeftemplateModule: Updates pointers in  */
/*   bloaded deftemplate module data structures. */
/*************************************************/
static VOID UpdateDeftemplateModule(buf,obji)
  VOID *buf;
  long obji;
  {
   struct bsaveDeftemplateModule *bdmPtr;
   
   bdmPtr = (struct bsaveDeftemplateModule *) buf;
   UpdateDefmoduleItemHeader(&bdmPtr->header,&ModuleArray[obji].header,
                             (int) sizeof(struct deftemplate),
                             (VOID *) DeftemplateArray);
  }
  
/**************************************************/
/* UpdateDeftemplate: Updates pointers in bloaded */
/*   deftemplate data structures.                 */
/**************************************************/
static VOID UpdateDeftemplate(buf,obji)
  VOID *buf;
  long obji;
  {
   struct deftemplate *theDeftemplate;
   struct bsaveDeftemplate *bdtPtr;
   
   bdtPtr = (struct bsaveDeftemplate *) buf;
   theDeftemplate = (struct deftemplate *) &DeftemplateArray[obji];

   UpdateConstructHeader(&bdtPtr->header,&theDeftemplate->header,
                         (int) sizeof(struct deftemplateModule),(VOID *) ModuleArray,
                         (int) sizeof(struct deftemplate),(VOID *) DeftemplateArray);
   
   if (bdtPtr->slotList != -1L)
     { theDeftemplate->slotList = (struct templateSlot *) &SlotArray[bdtPtr->slotList]; }
   else
     { theDeftemplate->slotList = NULL; }
   
   if (bdtPtr->patternNetwork != -1L)
     { theDeftemplate->patternNetwork = (struct factPatternNode *) BloadFactPatternPointer(bdtPtr->patternNetwork); }
   else
     { theDeftemplate->patternNetwork = NULL; }
     
   theDeftemplate->implied = bdtPtr->implied;
#if DEBUGGING_FUNCTIONS
   theDeftemplate->watch = WatchFacts;
#endif
   theDeftemplate->inScope = CLIPS_FALSE;
   theDeftemplate->numberOfSlots = bdtPtr->numberOfSlots;
  }

/******************************************************/
/* UpdateDeftemplateSlot: Updates pointers in bloaded */
/*   deftemplate slot data structures.                */
/******************************************************/
static VOID UpdateDeftemplateSlot(buf,obji)
  VOID *buf;
  long obji;
  {
   struct templateSlot *tsPtr;
   struct bsaveTemplateSlot *btsPtr;

   btsPtr = (struct bsaveTemplateSlot *) buf;
   tsPtr = (struct templateSlot *) &SlotArray[obji];

   tsPtr->slotName = SymbolPointer(btsPtr->slotName);
   IncrementSymbolCount(tsPtr->slotName);
   tsPtr->defaultList = HashedExpressionPointer(btsPtr->defaultList);
   tsPtr->constraints = ConstraintPointer(btsPtr->constraints);
   
   tsPtr->multislot = btsPtr->multislot;
   tsPtr->noDefault = btsPtr->noDefault;
   tsPtr->defaultPresent = btsPtr->defaultPresent;
   tsPtr->defaultDynamic = btsPtr->defaultDynamic;
   
   if (btsPtr->next != -1L)
     { tsPtr->next = (struct templateSlot *) &SlotArray[obji + 1]; }
   else
     { tsPtr->next = NULL; }
  }
  
/******************************************************/
/* ClearBload: Clear function for deftemplate bloads. */
/******************************************************/
static VOID ClearBload()
  {
   unsigned long int space;
   int i;
   
   for (i = 0; i < NumberOfDeftemplates; i++)
     { UnmarkConstructHeader(&DeftemplateArray[i].header); }
     
   for (i = 0; i < NumberOfTemplateSlots; i++)
     { DecrementSymbolCount(SlotArray[i].slotName); }
     
   space =  NumberOfTemplateModules * sizeof(struct deftemplateModule);
   if (space != 0) genlongfree((VOID *) ModuleArray,space);
   
   space = NumberOfDeftemplates * sizeof(struct deftemplate);
   if (space != 0) genlongfree((VOID *) DeftemplateArray,space);

   space =  NumberOfTemplateSlots * sizeof(struct templateSlot);
   if (space != 0) genlongfree((VOID *) SlotArray,space);
   
#if (! BLOAD_ONLY)
   CreateImpliedDeftemplate(AddSymbol("initial-fact"),CLIPS_FALSE);
#endif
  }
  
/************************************************************/
/* BloadDeftemplateModuleReference: Returns the deftemplate */
/*   module pointer for using with the bload function.      */
/************************************************************/
globle VOID *BloadDeftemplateModuleReference(index)
  int index;
  {
   return ((VOID *) &ModuleArray[index]);
  }
  
#endif /* DEFTEMPLATE_CONSTRUCT && (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE) && (! RUN_TIME) */


