
   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                FACT BSAVE/BLOAD MODULE              */
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

#define _FACTBIN_SOURCE_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT && (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE) && (! RUN_TIME)

#include <stdio.h>
#define _CLIPS_STDIO_

#include "clipsmem.h"
#include "tmpltdef.h"
#include "bload.h"
#include "bsave.h"
#include "rulebin.h"
#include "moduldef.h"

#include "factbin.h"

/********************************************/
/* INTERNAL DATA STRUCTURES AND DEFINITIONS */
/********************************************/

struct bsaveFactPatternNode
  {
   struct bsavePatternNodeHeader header;
   unsigned int whichSlot : 8;
   unsigned int whichField : 8;
   unsigned int leaveFields : 8;
   long networkTest;
   long nextLevel;
   long lastLevel;
   long leftNode;
   long rightNode;
  };
  
#define BSAVE_FIND         0
#define BSAVE_PATTERNS     1

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct factPatternNode HUGE_ADDR  *FactPatternArray;

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static long                               NumberOfPatterns;

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
#if BLOAD_AND_BSAVE
   static VOID                    BsaveDriver(int,FILE *,struct factPatternNode *);
   static VOID                    BsaveFind(void);
   static VOID                    BsaveStorage(FILE *);
   static VOID                    BsaveFactPatterns(FILE *);
   static VOID                    BsavePatternNode(struct factPatternNode *,FILE *);
#endif
   static VOID                    BloadStorage(void);
   static VOID                    BloadBinaryItem(void);
   static VOID                    UpdateFactPatterns(VOID *,long);
   static VOID                    ClearBload(void);
#else
#if BLOAD_AND_BSAVE
   static VOID                    BsaveDriver();
   static VOID                    BsaveFind();
   static VOID                    BsaveStorage();
   static VOID                    BsaveFactPatterns();
   static VOID                    BsavePatternNode();
#endif
   static VOID                    BloadStorage();
   static VOID                    BloadBinaryItem();
   static VOID                    UpdateFactPatterns();
   static VOID                    ClearBload();
#endif

/*****************************************************/
/* FactBinarySetup: Initializes the binary load/save */
/*   feature for the fact pattern network.           */
/*****************************************************/
globle VOID FactBinarySetup()
  {
#if BLOAD_AND_BSAVE
   AddBinaryItem("facts",0,BsaveFind,NULL,
                            BsaveStorage,BsaveFactPatterns,
                            BloadStorage,BloadBinaryItem,
                            ClearBload);
#endif
#if BLOAD || BLOAD_ONLY
   AddBinaryItem("facts",0,NULL,NULL,NULL,NULL,
                            BloadStorage,BloadBinaryItem,
                            ClearBload);
#endif
  }

#if BLOAD_AND_BSAVE

/****************************************************************/
/* BsaveFind: Binary save routine which assigns indices to each */
/*   fact pattern node and counts the number of expressions,    */
/*   symbols, and other data structures required for the binary */
/*   image.                                                     */
/****************************************************************/
static VOID BsaveFind()
  {
   struct deftemplate *theDeftemplate;
   struct defmodule *theModule;
   
   if (Bloaded()) SaveBloadCount(NumberOfPatterns);
   NumberOfPatterns = 0L;
     
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
      theDeftemplate = (struct deftemplate *) GetNextDeftemplate(NULL);
      while (theDeftemplate != NULL)
        {
         BsaveDriver(BSAVE_FIND,NULL,theDeftemplate->patternNetwork);
         theDeftemplate = (struct deftemplate *) GetNextDeftemplate(theDeftemplate);
        }
        
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }
  }
   
/***************************************************************/
/* BsaveDriver: Binary save driver routine which handles both  */
/*   finding/marking the data structures to be save and saving */
/*   the data structures to a file.                            */
/***************************************************************/
static VOID BsaveDriver(action,fp,thePattern)
  int action;
  FILE *fp;
  struct factPatternNode *thePattern;
  {
   while (thePattern != NULL)
     {
      switch(action)
        {
         case BSAVE_FIND:
           thePattern->bsaveID = NumberOfPatterns++; 
           break;
           
         case BSAVE_PATTERNS:
           BsavePatternNode(thePattern,fp);  
           break;
           
         default:
           break;
        }      
      
      if (thePattern->nextLevel == NULL)
        {
         while (thePattern->rightNode == NULL)
           {
            thePattern = thePattern->lastLevel;
            if (thePattern == NULL) return;
           }
         thePattern = thePattern->rightNode;
        }
      else
        { thePattern = thePattern->nextLevel; }
     }
  }
  
/*********************************************************/
/* BsaveStorage: Writes out storage requirements for all */
/*   factPatternNode data structures to the binary file  */
/*********************************************************/
static VOID BsaveStorage(fp)
  FILE *fp;
  {
   unsigned long space;

   space = sizeof(long);
   GenWrite(&space,(unsigned long) sizeof(unsigned long int),fp);
   GenWrite(&NumberOfPatterns,(unsigned long) sizeof(long int),fp);
  }
  
/*****************************************************/
/* BsaveFactPatterns: Writes out all factPatternNode */
/*    data structures to the binary file.            */
/*****************************************************/
static VOID BsaveFactPatterns(fp)
  FILE *fp;
  {
   unsigned long int space;
   struct deftemplate *theDeftemplate;
   struct defmodule *theModule;

   space = NumberOfPatterns * sizeof(struct bsaveFactPatternNode);
   GenWrite(&space,(unsigned long) sizeof(unsigned long int),fp);
   
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
      theDeftemplate = (struct deftemplate *) GetNextDeftemplate(NULL);
      while (theDeftemplate != NULL)
        {
         BsaveDriver(BSAVE_PATTERNS,fp,theDeftemplate->patternNetwork);
         theDeftemplate = (struct deftemplate *) GetNextDeftemplate(theDeftemplate);
        }
     theModule = (struct defmodule *) GetNextDefmodule(theModule);
    }
        
   if (Bloaded()) RestoreBloadCount(&NumberOfPatterns);
  }
  
/******************************************************/
/* BsavePatternNode: Writes out a single fact pattern */
/*   node to the binary image save file.              */
/******************************************************/
static VOID BsavePatternNode(thePattern,fp)
  struct factPatternNode *thePattern;
  FILE *fp;
  {
   struct bsaveFactPatternNode tempNode;

   AssignBsavePatternHeaderValues(&tempNode.header,&thePattern->header);
   
   tempNode.whichField = thePattern->whichField;
   tempNode.leaveFields = thePattern->leaveFields;
   tempNode.whichSlot = thePattern->whichSlot;
   tempNode.networkTest = HashedExpressionIndex(thePattern->networkTest);                 
   tempNode.nextLevel =  BsaveFactPatternIndex(thePattern->nextLevel);
   tempNode.lastLevel =  BsaveFactPatternIndex(thePattern->lastLevel);
   tempNode.leftNode =  BsaveFactPatternIndex(thePattern->leftNode);
   tempNode.rightNode =  BsaveFactPatternIndex(thePattern->rightNode);
      
   GenWrite(&tempNode,(unsigned long) sizeof(struct bsaveFactPatternNode),fp);
  }

#endif /* BLOAD_AND_BSAVE */

/*******************************************************/
/* BloadStorage: Determines the amount of storage need */
/*   for the fact pattern network binary image.        */
/*******************************************************/
static VOID BloadStorage()
  {
   unsigned long space;

   GenRead(&space,(unsigned long) sizeof(unsigned long int));
   GenRead(&NumberOfPatterns,(unsigned long) sizeof(long int));

   if (NumberOfPatterns == 0)
     {
      FactPatternArray = NULL;
      return;
     }

   space = NumberOfPatterns * sizeof(struct factPatternNode);
   FactPatternArray = (struct factPatternNode *) genlongalloc(space);
  }

/**********************************************/
/* BloadBinaryItem: Loads in the fact pattern */
/*   network needed for this binary image.    */
/**********************************************/
static VOID BloadBinaryItem()
  {
   unsigned long space;

   GenRead(&space,(unsigned long) sizeof(unsigned long int));

   BloadandRefresh(NumberOfPatterns,(unsigned) sizeof(struct bsaveFactPatternNode),
                   UpdateFactPatterns);
  }
    
/*************************************************/
/* UpdateFactPatterns: Bload refresh routine for */
/*   the factPatternNode structure.              */
/*************************************************/
static VOID UpdateFactPatterns(buf,obji)
  VOID *buf;
  long obji;
  {
   struct bsaveFactPatternNode *bp;
   
   bp = (struct bsaveFactPatternNode *) buf;
   
   UpdatePatternNodeHeader(&FactPatternArray[obji].header,&bp->header);
   
   FactPatternArray[obji].bsaveID = 0L; 
   FactPatternArray[obji].whichField = bp->whichField;
   FactPatternArray[obji].leaveFields = bp->leaveFields;
   FactPatternArray[obji].whichSlot = bp->whichSlot; 
   
   FactPatternArray[obji].networkTest = HashedExpressionPointer(bp->networkTest);
   FactPatternArray[obji].rightNode = BloadFactPatternPointer(bp->rightNode);
   FactPatternArray[obji].nextLevel = BloadFactPatternPointer(bp->nextLevel);
   FactPatternArray[obji].lastLevel = BloadFactPatternPointer(bp->lastLevel);
   FactPatternArray[obji].leftNode  = BloadFactPatternPointer(bp->leftNode);
  }
  
/******************************************************************/
/* ClearBload: Performs a clear for a bload fact pattern network. */
/******************************************************************/
static VOID ClearBload()
  {
   unsigned long int space;

   space = NumberOfPatterns * sizeof(struct factPatternNode);
   if (space != 0) genlongfree((VOID *) FactPatternArray,space);
  }

#endif /* DEFTEMPLATE_CONSTRUCT && (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE) && (! RUN_TIME) */


