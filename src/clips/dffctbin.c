   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             DEFFACTS BSAVE/BLOAD MODULE             */
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

#define _DFFCTBIN_SOURCE_

#include "setup.h"

#if DEFFACTS_CONSTRUCT && (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE) && (! RUN_TIME)

#include <stdio.h>
#define _CLIPS_STDIO_

#include "clipsmem.h"
#include "dffctdef.h"
#include "moduldef.h"
#include "bload.h"
#include "bsave.h"

#include "dffctbin.h"

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static struct deffacts HUGE_ADDR        *DeffactsArray = NULL;
   static long                              NumberOfDeffacts = 0;
   static struct deffactsModule HUGE_ADDR  *ModuleArray;
   static long                              NumberOfDeffactsModules;

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
#if BLOAD_AND_BSAVE
   static VOID                    BsaveFind(void);
   static VOID                    BsaveExpressions(FILE *);
   static VOID                    BsaveStorage(FILE *);
   static VOID                    BsaveBinaryItem(FILE *);
#endif
   static VOID                    BloadStorage(void);
   static VOID                    BloadBinaryItem(void);
   static VOID                    UpdateDeffactsModule(VOID *,long);
   static VOID                    UpdateDeffacts(VOID *,long);
   static VOID                    ClearBload(void);
#else
#if BLOAD_AND_BSAVE
   static VOID                    BsaveFind();
   static VOID                    BsaveExpressions();
   static VOID                    BsaveStorage();
   static VOID                    BsaveBinaryItem();
#endif
   static VOID                    BloadStorage();
   static VOID                    BloadBinaryItem();
   static VOID                    UpdateDeffactsModule();
   static VOID                    UpdateDeffacts();
   static VOID                    ClearBload();
#endif

/********************************************/
/* DeffactsBinarySetup: Installs the binary */
/*   save/load feature for deffacts.        */
/********************************************/
globle VOID DeffactsBinarySetup()
  {
#if BLOAD_AND_BSAVE
   AddBinaryItem("deffacts",0,BsaveFind,BsaveExpressions,
                             BsaveStorage,BsaveBinaryItem,
                             BloadStorage,BloadBinaryItem,
                             ClearBload);
#endif

#if (BLOAD || BLOAD_ONLY)
   AddBinaryItem("deffacts",0,NULL,NULL,NULL,NULL,
                             BloadStorage,BloadBinaryItem,
                             ClearBload);
#endif
  }

#if BLOAD_AND_BSAVE

/*********************************************************/
/* BsaveFind: Counts the number of data structures which */
/*   must be saved in the binary image for the deffacts  */
/*   in the current environment.                         */
/*********************************************************/
static VOID BsaveFind()
  {
   struct deffacts *deffactsPtr;
   struct defmodule *theModule;

   if (Bloaded())
     {
      SaveBloadCount(NumberOfDeffactsModules);
      SaveBloadCount(NumberOfDeffacts);
     }
   
   NumberOfDeffacts = 0;
   NumberOfDeffactsModules = 0;
   
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
      NumberOfDeffactsModules++;
      
      deffactsPtr = (struct deffacts *) GetNextDeffacts(NULL);
      while (deffactsPtr != NULL)
        {
         MarkConstructHeaderNeededItems(&deffactsPtr->header,NumberOfDeffacts++);
         ExpressionCount += ExpressionSize(deffactsPtr->assertList);
         MarkNeededItems(deffactsPtr->assertList);
         deffactsPtr = (struct deffacts *) GetNextDeffacts(deffactsPtr);
        }
        
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }
  }

/************************************************/
/* BsaveExpressions: Saves the expressions used */
/*   by deffacts to the binary save file.       */
/************************************************/
static VOID BsaveExpressions(fp)
  FILE *fp;
  {
   struct deffacts *deffactsPtr;
   struct defmodule *theModule;

   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
      deffactsPtr = (struct deffacts *) GetNextDeffacts(NULL);
      while (deffactsPtr != NULL)
        {
         BsaveExpression(deffactsPtr->assertList,fp);
         deffactsPtr = (struct deffacts *) GetNextDeffacts(deffactsPtr);
        }
        
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }
  }

/******************************************************/
/* BsaveStorage: Writes out the storage requirements  */
/*    for all deffacts structures to the binary file. */
/******************************************************/
static VOID BsaveStorage(fp)
  FILE *fp;
  {
   unsigned long space;

   space = sizeof(long) * 2;
   GenWrite(&space,(unsigned long) sizeof(unsigned long int),fp);
   GenWrite(&NumberOfDeffacts,(unsigned long) sizeof(long int),fp);
   GenWrite(&NumberOfDeffactsModules,(unsigned long) sizeof(long int),fp);
  }
  
/********************************************/
/* BsaveBinaryItem: Writes out all deffacts */
/*   structures to the binary file.         */
/********************************************/
static VOID BsaveBinaryItem(fp)
  FILE *fp;
  {
   unsigned long int space;
   struct deffacts *theDeffacts;
   struct bsaveDeffacts newDeffacts;
   struct defmodule *theModule;
   struct bsaveDeffactsModule tempDeffactsModule;
   struct deffactsModule *theModuleItem;
   
   space = NumberOfDeffacts * sizeof(struct bsaveDeffacts) +
           (NumberOfDeffactsModules * sizeof(struct bsaveDeffactsModule));
   GenWrite(&space,(unsigned long) sizeof(unsigned long int),fp);
   
   /*================================================*/
   /* Write out each deffacts module data structure. */
   /*================================================*/

   NumberOfDeffacts = 0;
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
      
      theModuleItem = (struct deffactsModule *)
                      GetModuleItem(theModule,DeffactsModuleIndex);  
      theModuleItem = (struct deffactsModule *) GetModuleItem(NULL,DeffactsModuleIndex);
      AssignBsaveDefmdlItemHdrVals(&tempDeffactsModule.header,&theModuleItem->header);
      GenWrite(&tempDeffactsModule,(unsigned long) sizeof(struct bsaveDeffactsModule),fp);
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }
     
   /*=========================*/
   /* Write out each deffact. */
   /*=========================*/

   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
  
      theDeffacts = (struct deffacts *) GetNextDeffacts(NULL);
      while (theDeffacts != NULL)
        {
         AssignBsaveConstructHeaderVals(&newDeffacts.header,&theDeffacts->header);
         if (theDeffacts->assertList != NULL)
           {
            newDeffacts.assertList = ExpressionCount;
            ExpressionCount += ExpressionSize(theDeffacts->assertList);
           }
         else
           { newDeffacts.assertList = -1L; }

         GenWrite(&newDeffacts,(unsigned long) sizeof(struct bsaveDeffacts),fp);
         theDeffacts = (struct deffacts *) GetNextDeffacts(theDeffacts);
        }
        
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }
     
   if (Bloaded())
     {
      RestoreBloadCount(&NumberOfDeffactsModules);
      RestoreBloadCount(&NumberOfDeffacts);
     }
  }
  
#endif /* BLOAD_AND_BSAVE */

/************************************************/
/* BloadStorage: Loads storage requirements for */
/*   the deffacts used by this binary image.    */
/************************************************/
static VOID BloadStorage()
  {
   unsigned long int space;

   /*=============================*/
   /* Get the number of deffacts. */
   /*=============================*/

   GenRead(&space,(unsigned long) sizeof(unsigned long int));
   GenRead(&NumberOfDeffacts,(unsigned long) sizeof(long int));
   GenRead(&NumberOfDeffactsModules,(unsigned long) sizeof(long int));
      
   /*===========================================*/
   /* Read the deffacts module data structures. */
   /*===========================================*/
   
   if (NumberOfDeffactsModules == 0)
     {
      DeffactsArray = NULL;
      ModuleArray = NULL;
     }
     
   space = NumberOfDeffactsModules * sizeof(struct deffactsModule);
   ModuleArray = (struct deffactsModule HUGE_ADDR *) genlongalloc(space);

   /*====================================*/
   /* Read the deffacts data structures. */
   /*====================================*/
   
   if (NumberOfDeffacts == 0)
     {
      DeffactsArray = NULL;
      return;
     }
     
   space = (unsigned long) (NumberOfDeffacts * sizeof(struct deffacts));
   DeffactsArray = (struct deffacts HUGE_ADDR *) genlongalloc(space);
  }

/*****************************************************/
/* BloadBinaryItem: Loads and refreshes the deffacts */
/*   constructs used by this binary image.           */
/*****************************************************/
static VOID BloadBinaryItem()
  {
   unsigned long int space;

   GenRead(&space,(unsigned long) sizeof(unsigned long int));
   
   BloadandRefresh(NumberOfDeffactsModules,
                   (unsigned) sizeof(struct bsaveDeffactsModule),
                   UpdateDeffactsModule);
                   
   BloadandRefresh(NumberOfDeffacts,
                   (unsigned) sizeof(struct bsaveDeffacts),
                   UpdateDeffacts);
  }
  
/**********************************************/
/* UpdateDeffactsModule: Updates pointers in  */
/*   bloaded deffacts module data structures. */
/**********************************************/
static VOID UpdateDeffactsModule(buf,obji)
  VOID *buf;
  long obji;
  {
   struct bsaveDeffactsModule *bdmPtr;
   
   bdmPtr = (struct bsaveDeffactsModule *) buf;
   UpdateDefmoduleItemHeader(&bdmPtr->header,&ModuleArray[obji].header,
                             (int) sizeof(struct deffacts),(VOID *) DeffactsArray);
  }

/*******************************************************/
/* UpdateDeffacts: Bload refresh routine for deffacts. */
/*******************************************************/
static VOID UpdateDeffacts(buf,obji)
  VOID *buf;
  long obji;
  {
   struct bsaveDeffacts *bdp;
   
   bdp = (struct bsaveDeffacts *) buf;
   UpdateConstructHeader(&bdp->header,&DeffactsArray[obji].header,
                         (int) sizeof(struct deffactsModule),(VOID *) ModuleArray,
                         (int) sizeof(struct deffacts),(VOID *) DeffactsArray);
   DeffactsArray[obji].assertList = ExpressionPointer(bdp->assertList);
  }
  
/***********************************************************************/
/* ClearBload: Deffacts clear routine when a binary load is in effect. */
/***********************************************************************/
static VOID ClearBload()
  {
   long i;
   unsigned long space;

   for (i = 0; i < NumberOfDeffacts; i++)
     { UnmarkConstructHeader(&DeffactsArray[i].header); }

   space = NumberOfDeffacts * sizeof(struct deffacts);
   if (space != 0) genlongfree((VOID *) DeffactsArray,space);

   space =  NumberOfDeffactsModules * sizeof(struct deffactsModule);
   if (space != 0) genlongfree((VOID *) ModuleArray,space);
  }
  
/*******************************************************/
/* BloadDeffactsModuleReference: Returns the deffacts  */
/*   module pointer for using with the bload function. */
/*******************************************************/
globle VOID *BloadDeffactsModuleReference(index)
  int index;
  {
   return ((VOID *) &ModuleArray[index]);
  }
  
#endif /* DEFFACTS_CONSTRUCT && (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE) && (! RUN_TIME) */


