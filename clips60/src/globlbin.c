   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            DEFGLOBAL BSAVE/BLOAD MODULE             */
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

#define _GLOBLBIN_SOURCE_

#include "setup.h"

#if DEFGLOBAL_CONSTRUCT && (BLOAD || BLOAD_AND_BSAVE || BLOAD_ONLY) && (! RUN_TIME)

#include <stdio.h>
#define _CLIPS_STDIO_

#include "clipsmem.h"
#include "multifld.h"
#include "globldef.h"
#include "bload.h"
#include "bsave.h"
#include "moduldef.h"
#include "globlbsc.h"

#include "globlbin.h"
  
/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
#if BLOAD_AND_BSAVE
   static VOID                    BsaveFind(void);
   static VOID                    BsaveStorage(FILE *);
   static VOID                    BsaveBinaryItem(FILE *);
#endif
   static VOID                    BloadStorageDefglobals(void);
   static VOID                    BloadBinaryItem(void);
   static VOID                    UpdateDefglobalModule(VOID *,long);
   static VOID                    UpdateDefglobal(VOID *,long);
   static VOID                    ClearBload(void);
#else
#if BLOAD_AND_BSAVE
   static VOID                    BsaveFind();
   static VOID                    BsaveStorage();
   static VOID                    BsaveBinaryItem();
#endif
   static VOID                    BloadStorageDefglobals();
   static VOID                    BloadBinaryItem();
   static VOID                    UpdateDefglobalModule();
   static VOID                    UpdateDefglobal();
   static VOID                    ClearBload();
#endif


/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct defglobal HUGE_ADDR       *DefglobalArray = NULL;
   
/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/
   
   static long                              NumberOfDefglobals = 0;
   static struct defglobalModule HUGE_ADDR *ModuleArray;
   static long                              NumberOfDefglobalModules;

/*******************************************************/
/* DefglobalBinarySetup: Installs the binary save/load */
/*   feature for the defglobal construct.              */
/*******************************************************/
globle VOID DefglobalBinarySetup()
  {
#if (BLOAD_AND_BSAVE || BLOAD)
   AddAfterBloadFunction("defglobal",ResetDefglobals,50);
#endif

#if BLOAD_AND_BSAVE
   AddBinaryItem("defglobal",0,BsaveFind,NULL,
                             BsaveStorage,BsaveBinaryItem,
                             BloadStorageDefglobals,BloadBinaryItem,
                             ClearBload);
#endif

#if (BLOAD || BLOAD_ONLY)
   AddBinaryItem("defglobal",0,NULL,NULL,NULL,NULL,
                             BloadStorageDefglobals,BloadBinaryItem,
                             ClearBload);
#endif
  }

#if BLOAD_AND_BSAVE

/***********************************************************************/
/* BsaveDefglobalFind: Find expressions function for defglobal bsaves. */
/***********************************************************************/
static VOID BsaveFind()
  {
   struct defglobal *defglobalPtr;
   struct defmodule *theModule;

   if (Bloaded())
     {
      SaveBloadCount(NumberOfDefglobalModules);
      SaveBloadCount(NumberOfDefglobals);
     }
   
   NumberOfDefglobals = 0;
   NumberOfDefglobalModules = 0;
    
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
      NumberOfDefglobalModules++;
      
      defglobalPtr = (struct defglobal *) GetNextDefglobal(NULL);
          
      while (defglobalPtr != NULL)
        {
         defglobalPtr->header.bsaveID = NumberOfDefglobals;
         NumberOfDefglobals++;
         defglobalPtr->header.name->neededSymbol = CLIPS_TRUE;
         defglobalPtr = (struct defglobal *) GetNextDefglobal(defglobalPtr);
        }
      
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }
  }

/*****************************************************/
/* BsaveStorage: Writes out storage requirements for */
/*   all defglobal structures to the binary file     */
/*****************************************************/
static VOID BsaveStorage(fp)
  FILE *fp;
  {
   unsigned long space;

   space = sizeof(long) * 2;
   GenWrite(&space,(unsigned long) sizeof(unsigned long int),fp);
   GenWrite(&NumberOfDefglobals,(unsigned long) sizeof(long int),fp);
   GenWrite(&NumberOfDefglobalModules,(unsigned long) sizeof(long int),fp);
  }

/********************************************************/
/* BsaveBinaryItem: Writes out all defglobal structures */
/*   to the binary file                                 */
/********************************************************/
static VOID BsaveBinaryItem(fp)
  FILE *fp;
  {
   unsigned long int space;
   struct defglobal *theDefglobal;
   struct bsaveDefglobal newDefglobal;
   struct defmodule *theModule;
   struct bsaveDefglobalModule tempDefglobalModule;
   struct defglobalModule *theModuleItem;
   int whichModuleItem = 0;
   
   space = NumberOfDefglobals * sizeof(struct bsaveDefglobal) +
           (NumberOfDefglobalModules * sizeof(struct bsaveDefglobalModule));
   GenWrite(&space,(unsigned long) sizeof(unsigned long int),fp);
   
   /*=================================================*/
   /* Write out each defglobal module data structure. */
   /*=================================================*/

   NumberOfDefglobals = 0;
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
      
      theModuleItem = (struct defglobalModule *) 
                      GetModuleItem(NULL,FindModuleItem("defglobal")->moduleIndex);
      AssignBsaveDefmdlItemHdrVals(&tempDefglobalModule.header,
                                           &theModuleItem->header);
      GenWrite(&tempDefglobalModule,(unsigned long) sizeof(struct bsaveDefglobalModule),fp);
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }

   /*===========================*/
   /* Write out each defglobal. */
   /*===========================*/

   NumberOfDefglobals = 0;
   whichModuleItem = 0;
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
  
      theDefglobal = (struct defglobal *) GetNextDefglobal(NULL);
      while (theDefglobal != NULL)
        {
         AssignBsaveConstructHeaderVals(&newDefglobal.header,
                                          &theDefglobal->header);
         newDefglobal.initial = HashedExpressionIndex(theDefglobal->initial);

         GenWrite(&newDefglobal,(unsigned long) sizeof(struct bsaveDefglobal),fp);
         theDefglobal = (struct defglobal *) GetNextDefglobal(theDefglobal);
        }
        
      whichModuleItem++;
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }

   /*===========================*/
   /* Restore the bload counts. */
   /*===========================*/
   
   if (Bloaded())
     {
      RestoreBloadCount(&NumberOfDefglobalModules);
      RestoreBloadCount(&NumberOfDefglobals);
     }
  }
  
#endif /* BLOAD_AND_BSAVE */

/***********************************************/
/* BloadStorageDefglobals: Allocates space for */
/*   the defglobals used by this binary image. */
/***********************************************/
static VOID BloadStorageDefglobals()
  {
   unsigned long int space;

   /*===============================*/
   /* Get the number of defglobals. */
   /*===============================*/

   GenRead(&space,(unsigned long) sizeof(unsigned long int));
   GenRead(&NumberOfDefglobals,(unsigned long) sizeof(long int));
   GenRead(&NumberOfDefglobalModules,(unsigned long) sizeof(long int));
      
   /*============================================*/
   /* Read the defglobal module data structures. */
   /*============================================*/
   
   if (NumberOfDefglobalModules == 0)
     {
      DefglobalArray = NULL;
      ModuleArray = NULL;
     }
     
   space = NumberOfDefglobalModules * sizeof(struct defglobalModule);
   ModuleArray = (struct defglobalModule HUGE_ADDR *) genlongalloc(space);

   /*=====================================*/
   /* Read the defglobal data structures. */
   /*=====================================*/
   
   if (NumberOfDefglobals == 0)
     {
      DefglobalArray = NULL;
      return;
     }
     
   space = (unsigned long) (NumberOfDefglobals * sizeof(struct defglobal));
   DefglobalArray = (struct defglobal HUGE_ADDR *) genlongalloc(space);
  }

/********************************************************************/
/* BloadBinaryItem: Loads the defglobals used by this binary image. */
/********************************************************************/
static VOID BloadBinaryItem()
  {
   unsigned long int space;

   GenRead(&space,(unsigned long) sizeof(unsigned long int));
   
   BloadandRefresh(NumberOfDefglobalModules,
                   (unsigned) sizeof(struct bsaveDefglobalModule),
                   UpdateDefglobalModule);
                   
   BloadandRefresh(NumberOfDefglobals,
                   (unsigned) sizeof(struct bsaveDefglobal),
                   UpdateDefglobal);
  }
  
/***********************************************/
/* UpdateDefglobalModule: Updates pointers in  */
/*   bloaded defglobal module data structures. */
/***********************************************/
static VOID UpdateDefglobalModule(buf,obji)
  VOID *buf;
  long obji;
  {
   struct bsaveDefglobalModule *bdmPtr;
   
   bdmPtr = (struct bsaveDefglobalModule *) buf;
   
   UpdateDefmoduleItemHeader(&bdmPtr->header,&ModuleArray[obji].header,
                             (int) sizeof(struct defglobal),
                             (VOID *) DefglobalArray);
  }
  
/******************************************/
/* UpdateDefglobal: Bload refresh routine */
/*   for defglobal data structures.       */
/******************************************/
static VOID UpdateDefglobal(buf,obji)
  VOID *buf;
  long obji;
  {
   struct bsaveDefglobal *bdp;
   
   bdp = (struct bsaveDefglobal *) buf;
   UpdateConstructHeader(&bdp->header,&DefglobalArray[obji].header,
                         (int) sizeof(struct defglobalModule),(VOID *) ModuleArray,
                         (int) sizeof(struct defglobal),(VOID *) DefglobalArray);
   
#if DEBUGGING_FUNCTIONS
   DefglobalArray[obji].watch = WatchGlobals;
#endif
   DefglobalArray[obji].initial = HashedExpressionPointer(bdp->initial);
   DefglobalArray[obji].current.type = RVOID;
   
  }
    
/************************************************************************/
/* ClearBload: Defglobal clear routine when a binary load is in effect. */
/************************************************************************/
static VOID ClearBload()
  {
   long i;
   unsigned long space;
     
   for (i = 0; i < NumberOfDefglobals; i++)
     {
      UnmarkConstructHeader(&DefglobalArray[i].header);
      
      ValueDeinstall(&(DefglobalArray[i].current));
      if (DefglobalArray[i].current.type == MULTIFIELD)
        { ReturnMultifield(DefglobalArray[i].current.value); }
     }

   space = NumberOfDefglobals * sizeof(struct defglobal);
   if (space != 0) genlongfree((VOID *) DefglobalArray,space);

   space =  NumberOfDefglobalModules * sizeof(struct defglobalModule);
   if (space != 0) genlongfree((VOID *) ModuleArray,space);
  }

/*********************************************************/
/* BloadDefglobalModuleReference: Returns the defglobal  */
/*   module pointer for using with the bload function.   */
/*********************************************************/
globle VOID *BloadDefglobalModuleReference(index)
  int index;
  {
   return ((VOID *) &ModuleArray[index]); 
  }

#endif /* DEFGLOBAL_CONSTRUCT && (BLOAD || BLOAD_AND_BSAVE || BLOAD_ONLY) && (! RUN_TIME) */



