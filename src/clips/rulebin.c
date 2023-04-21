
   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*              DEFRULE BSAVE/BLOAD MODULE             */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian L. Donnell                                     */
/*      Barry Cameron                                        */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _RULEBIN_SOURCE_

#include "setup.h"

#if DEFRULE_CONSTRUCT && (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE) && (! RUN_TIME)

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "clipsmem.h"
#include "bload.h"
#include "bsave.h"
#include "reteutil.h"
#include "agenda.h"
#include "engine.h"
#include "rulebsc.h"
#include "pattern.h"
#include "moduldef.h"

#include "rulebin.h"

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static long                              NumberOfDefruleModules;
   static long                              NumberOfDefrules;
   static long                              NumberOfJoins;
   static struct defruleModule HUGE_ADDR   *ModuleArray;
   static struct defrule HUGE_ADDR         *DefruleArray;
   static struct joinNode HUGE_ADDR        *JoinArray;

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
#if BLOAD_AND_BSAVE
   static VOID                    BsaveFind(void);
   static VOID                    BsaveExpressions(FILE *);
   static VOID                    BsaveStorage(FILE *);
   static VOID                    BsaveBinaryItem(FILE *);
   static VOID                    BsaveJoins(FILE *);
#endif
   static VOID                    BloadStorage(void);
   static VOID                    BloadBinaryItem(void);
   static VOID                    UpdateDefruleModule(VOID *,long);
   static VOID                    UpdateDefrule(VOID *,long);
   static VOID                    UpdateJoin(VOID *,long);
   static VOID                    ClearBload(void);
#else
#if BLOAD_AND_BSAVE
   static VOID                    BsaveFind();
   static VOID                    BsaveExpressions();
   static VOID                    BsavePatternExpressions();
   static VOID                    BsaveStorage();
   static VOID                    BsaveBinaryItem();
   static VOID                    BsaveJoins();
#endif
   static VOID                    BloadStorage();
   static VOID                    BloadBinaryItem();
   static VOID                    UpdateDefruleModule();
   static VOID                    UpdateDefrule();
   static VOID                    UpdateJoin();
   static VOID                    ClearBload();
#endif

/*****************************************************/
/* DefruleBinarySetup: Installs the binary save/load */
/*   feature for the defrule construct.              */
/*****************************************************/
globle VOID DefruleBinarySetup()
  {
#if BLOAD_AND_BSAVE
   AddBinaryItem("defrule",20,BsaveFind,BsaveExpressions,
                             BsaveStorage,BsaveBinaryItem,
                             BloadStorage,BloadBinaryItem,
                             ClearBload);
#endif
#if BLOAD || BLOAD_ONLY
   AddBinaryItem("defrule",20,NULL,NULL,NULL,NULL,
                             BloadStorage,BloadBinaryItem,
                             ClearBload);
#endif
  }
  
#if BLOAD_AND_BSAVE

/*************************************************************/
/* BsaveFind: Determines the amount of memory needed to save */
/*   the defrule and joinNode data structures in addition to */
/*   the memory needed for their associated expressions.     */
/*************************************************************/
static VOID BsaveFind()
  {
   struct defrule *rPtr, *tPtr;
   struct defmodule *modulePtr;

   if (Bloaded())
     {
      SaveBloadCount(NumberOfDefruleModules);
      SaveBloadCount(NumberOfDefrules);
      SaveBloadCount(NumberOfJoins);
     }
     
   TagRuleNetwork(&NumberOfDefruleModules,&NumberOfDefrules,&NumberOfJoins);
   
   modulePtr = (struct defmodule *) GetNextDefmodule(NULL);
   while (modulePtr != NULL)
     {
      SetCurrentModule((VOID *) modulePtr);
    
      rPtr = (struct defrule *) GetNextDefrule(NULL);
      while (rPtr != NULL)
        {
         /* ===========================================================
            The binary save ID has already been set by TagRuleNetwork()
            =========================================================== */
         MarkConstructHeaderNeededItems(&rPtr->header,rPtr->header.bsaveID);
#if DYNAMIC_SALIENCE
         ExpressionCount += ExpressionSize(rPtr->dynamicSalience);
         MarkNeededItems(rPtr->dynamicSalience);
#endif
         tPtr = rPtr;
         while (tPtr != NULL)
           {
            ExpressionCount += ExpressionSize(tPtr->actions);
            MarkNeededItems(tPtr->actions);
            tPtr = tPtr->disjunct;
           }

         rPtr = (struct defrule *) GetNextDefrule(rPtr);
        }  
      
      modulePtr = (struct defmodule *) GetNextDefmodule(modulePtr);
     }

   MarkRuleNetwork(1);
  }

/************************************************/
/* BsaveExpressions: Saves the expressions used */
/*   by defrules to the binary save file.       */
/************************************************/
static VOID BsaveExpressions(fp)
  FILE *fp;
  {
   struct defrule *rPtr, *tPtr;  
   struct defmodule *modulePtr; 
   
   modulePtr = (struct defmodule *) GetNextDefmodule(NULL);
   while (modulePtr != NULL)
     {
      SetCurrentModule((VOID *) modulePtr);

      rPtr = (struct defrule *) GetNextDefrule(NULL);
      while (rPtr != NULL)
        {
#if DYNAMIC_SALIENCE
         BsaveExpression(rPtr->dynamicSalience,fp);
#endif
         tPtr = rPtr;
         while (tPtr != NULL)
           {
            BsaveExpression(tPtr->actions,fp);
            tPtr = tPtr->disjunct;
           }
         rPtr = (struct defrule *) GetNextDefrule(rPtr);
        }
        
      modulePtr = (struct defmodule *) GetNextDefmodule(modulePtr);
     }

   MarkRuleNetwork(1);
  }
  
/*****************************************************/
/* BsaveStorage: Writes out storage requirements for */
/*   all defrule structures to the binary file       */
/*****************************************************/
static VOID BsaveStorage(fp)
  FILE *fp;
  {
   unsigned long space;

   space = sizeof(long) * 3;
   GenWrite(&space,(unsigned long) sizeof(unsigned long int),fp);
   GenWrite(&NumberOfDefruleModules,(unsigned long) sizeof(long int),fp);
   GenWrite(&NumberOfDefrules,(unsigned long) sizeof(long int),fp);
   GenWrite(&NumberOfJoins,(unsigned long) sizeof(long int),fp);
  }

/**************************************************************************/
/* BsaveBinaryItem: Writes out all defrule structures to the binary file. */
/**************************************************************************/
static VOID BsaveBinaryItem(fp)
  FILE *fp;
  {
   unsigned long int space;
   struct defrule *rPtr, *tPtr;
   struct bsaveDefrule tempDefrule;
   struct bsaveDefruleModule tempDefruleModule;
   long int disjunctExpressionCount = 0L;
   int first;
   struct defmodule *theModule;
   struct defruleModule *theModuleItem;
   
   /*===============================================*/
   /* Write out the space required by the defrules. */
   /*===============================================*/

   space = (NumberOfDefrules * sizeof(struct bsaveDefrule)) +
           (NumberOfJoins * sizeof(struct bsaveJoinNode)) +
           (NumberOfDefruleModules * sizeof(struct bsaveDefruleModule));
   GenWrite(&space,(unsigned long) sizeof(unsigned long int),fp);
   
   /*===============================================*/
   /* Write out each defrule module data structure. */
   /*===============================================*/

   NumberOfDefrules = 0;
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
            
      theModuleItem = (struct defruleModule *)
                      GetModuleItem(NULL,FindModuleItem("defrule")->moduleIndex);
      AssignBsaveDefmdlItemHdrVals(&tempDefruleModule.header,
                                           &theModuleItem->header);
      GenWrite(&tempDefruleModule,(unsigned long) sizeof(struct bsaveDefruleModule),fp);
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }

   /*=========================*/
   /* Write out each defrule. */
   /*=========================*/

   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
   
      rPtr = (struct defrule *) GetNextDefrule(NULL);
      while (rPtr != NULL)
        {
         tPtr = rPtr;
         first = CLIPS_TRUE;
         while (tPtr != NULL)
           {
            AssignBsaveConstructHeaderVals(&tempDefrule.header,
                                             &tPtr->header);
            tempDefrule.salience = tPtr->salience;
            tempDefrule.localVarCnt = tPtr->localVarCnt;
            tempDefrule.complexity = tPtr->complexity;
            tempDefrule.autoFocus = tPtr->autoFocus;
#if DYNAMIC_SALIENCE
            if (tPtr->dynamicSalience != NULL)
              {
               if (first)
                 {
                  tempDefrule.dynamicSalience = ExpressionCount;
                  disjunctExpressionCount = ExpressionCount;
                  ExpressionCount += ExpressionSize(tPtr->dynamicSalience);
                 }
               else
                 { tempDefrule.dynamicSalience = disjunctExpressionCount; }
              }
            else
#endif
              { tempDefrule.dynamicSalience = -1L; }

            if (tPtr->actions != NULL)
              {
               tempDefrule.actions = ExpressionCount;
               ExpressionCount += ExpressionSize(tPtr->actions);
              }
            else
              { tempDefrule.actions = -1L; }

#if LOGICAL_DEPENDENCIES
            tempDefrule.logicalJoin = BsaveJoinIndex(tPtr->logicalJoin);
#else
            tempDefrule.logicalJoin = -1L;
#endif
            tempDefrule.lastJoin = BsaveJoinIndex(tPtr->lastJoin);

            if (tPtr->disjunct != NULL)
              { tempDefrule.disjunct = NumberOfDefrules; }
            else
              { tempDefrule.disjunct = -1L; }

            GenWrite(&tempDefrule,(unsigned long) sizeof(struct bsaveDefrule),fp);
            first = CLIPS_FALSE;
            tPtr = tPtr->disjunct;
           }
         rPtr = (struct defrule *) GetNextDefrule(rPtr);
        }
        
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }

   /*=============================*/
   /* Write out the Rete Network. */
   /*=============================*/
   
   MarkRuleNetwork(1);
   
   BsaveJoins(fp);
   
   if (Bloaded())
     {
      RestoreBloadCount(&NumberOfDefruleModules);
      RestoreBloadCount(&NumberOfDefrules);
      RestoreBloadCount(&NumberOfJoins);
     }
  }

/*************************************************************/
/* BsaveJoins: Saves the join structures to the binary file. */
/*************************************************************/
static VOID BsaveJoins(fp)
  FILE *fp;
  {
   struct defrule *rulePtr;
   struct joinNode *joinPtr;
   struct bsaveJoinNode tempJoin;
   struct defmodule *theModule;
   
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);

      rulePtr = (struct defrule *) GetNextDefrule(NULL);

      while (rulePtr != NULL)
        {
         joinPtr = rulePtr->lastJoin;
         while (joinPtr != NULL)
           {
            if (joinPtr->marked)
              {
               joinPtr->marked = 0;
               tempJoin.depth = joinPtr->depth;
               tempJoin.rhsType = joinPtr->rhsType;
               tempJoin.firstJoin = joinPtr->firstJoin;
               tempJoin.logicalJoin = joinPtr->logicalJoin;
               tempJoin.joinFromTheRight = joinPtr->joinFromTheRight;
               tempJoin.patternIsNegated = joinPtr->patternIsNegated;
            
               if (joinPtr->joinFromTheRight)
                 { tempJoin.rightSideEntryStructure =  BsaveJoinIndex(joinPtr->rightSideEntryStructure); }
               else
                 { tempJoin.rightSideEntryStructure =  -1L; }
              
               tempJoin.lastLevel =  BsaveJoinIndex(joinPtr->lastLevel);
               tempJoin.nextLevel =  BsaveJoinIndex(joinPtr->nextLevel);
               tempJoin.rightMatchNode =  BsaveJoinIndex(joinPtr->rightMatchNode);
               tempJoin.rightDriveNode =  BsaveJoinIndex(joinPtr->rightDriveNode);
               tempJoin.networkTest = HashedExpressionIndex(joinPtr->networkTest);
/*
               if (joinPtr->networkTest != NULL)
                 {
                  tempJoin.networkTest = ExpressionCount;
                  ExpressionCount += ExpressionSize(joinPtr->networkTest);
                 }
               else
                 { tempJoin.networkTest = -1L; }
              */
               if (joinPtr->ruleToActivate != NULL)
                 {
                  tempJoin.ruleToActivate = 
                     GetDisjunctIndex(joinPtr->ruleToActivate);
                 }
               else
                 { tempJoin.ruleToActivate = -1L; }

               GenWrite(&tempJoin,(unsigned long) sizeof(struct bsaveJoinNode),fp);
              }
  
            joinPtr = GetPreviousJoin(joinPtr);
           }
 
        if (rulePtr->disjunct != NULL) rulePtr = rulePtr->disjunct;
        else rulePtr = (struct defrule *) GetNextDefrule(rulePtr);
       }
       
      theModule = (struct defmodule *) GetNextDefmodule(theModule);
     }
  }
      
/***********************************************************/
/* AssignBsavePatternHeaderValues: Assigns the appropriate */
/*   values to a bsave pattern header record.              */
/***********************************************************/
globle VOID AssignBsavePatternHeaderValues(theBsaveHeader,theHeader)
  struct bsavePatternNodeHeader *theBsaveHeader;
  struct patternNodeHeader *theHeader;
  {
   theBsaveHeader->singlefieldNode = theHeader->singlefieldNode;
   theBsaveHeader->multifieldNode = theHeader->multifieldNode;
   theBsaveHeader->stopNode = theHeader->stopNode;
   theBsaveHeader->beginSlot = theHeader->beginSlot;
   theBsaveHeader->endSlot = theHeader->endSlot;
   theBsaveHeader->entryJoin = BsaveJoinIndex(theHeader->entryJoin);
  }
  
#endif /* BLOAD_AND_BSAVE */

/************************************************/
/* BloadStorage: Loads storage requirements for */
/*   the defrules used by this binary image.    */
/************************************************/
static VOID BloadStorage()
  {
   unsigned long space;

   GenRead(&space,(unsigned long) sizeof(unsigned long int));
   GenRead(&NumberOfDefruleModules,(unsigned long) sizeof(long int));
   GenRead(&NumberOfDefrules,(unsigned long) sizeof(long int));
   GenRead(&NumberOfJoins,(unsigned long) sizeof(long int));
   
   /*==========================================*/
   /* Read the defrule module data structures. */
   /*==========================================*/
   
   if (NumberOfDefruleModules == 0)
     {
      ModuleArray = NULL;
      DefruleArray = NULL;
      JoinArray = NULL;
     }
     
   space = NumberOfDefruleModules * sizeof(struct defruleModule);
   ModuleArray = (struct defruleModule HUGE_ADDR *) genlongalloc(space);
   
   /*===================================*/
   /* Read the defrule data structures. */
   /*===================================*/
   
   if (NumberOfDefrules == 0)
     {
      DefruleArray = NULL;
      JoinArray = NULL;
      return;
     }

   space = NumberOfDefrules * sizeof(struct defrule);
   DefruleArray = (struct defrule *) genlongalloc(space);

   /*================================*/
   /* Read the join data structures. */
   /*================================*/
   
   space = NumberOfJoins * sizeof(struct joinNode);
   JoinArray = (struct joinNode *) genlongalloc(space);
  }

/**********************************************************/
/* BloadBinaryItem: Loads in the join and pattern network */
/*   needed for this binary image.                        */
/**********************************************************/
static VOID BloadBinaryItem()
  {
   unsigned long space;

   /*====================================================*/
   /* Determine the number of structures in the network. */
   /*====================================================*/

   GenRead(&space,(unsigned long) sizeof(unsigned long int));

   /*===================*/
   /* Load the network. */
   /*===================*/
 
   BloadandRefresh(NumberOfDefruleModules,(unsigned) sizeof(struct bsaveDefruleModule),
                   UpdateDefruleModule);
                   
   BloadandRefresh(NumberOfDefrules,(unsigned) sizeof(struct bsaveDefrule),
                   UpdateDefrule);

   BloadandRefresh(NumberOfJoins,(unsigned) sizeof(struct bsaveJoinNode),
                   UpdateJoin);
  }
  
/*********************************************/
/* UpdateDefruleModule: Updates pointers in  */
/*   bloaded defrule module data structures. */
/*********************************************/
static VOID UpdateDefruleModule(buf,obji)
  VOID *buf;
  long obji;
  {
   struct bsaveDefruleModule *bdmPtr;
   
   bdmPtr = (struct bsaveDefruleModule *) buf;
   UpdateDefmoduleItemHeader(&bdmPtr->header,&ModuleArray[obji].header,
                             (int) sizeof(struct defrule),
                             (VOID *) DefruleArray);
   ModuleArray[obji].agenda = NULL;
  }
  
/***************************************************************/
/* UpdateDefrule: Bload refresh routine for defrule structure. */
/***************************************************************/
static VOID UpdateDefrule(buf,obji)
  VOID *buf;
  long obji;
  {
   struct bsaveDefrule *br;
   
   br = (struct bsaveDefrule *) buf;
   UpdateConstructHeader(&br->header,&DefruleArray[obji].header,
                         (int) sizeof(struct defruleModule),(VOID *) ModuleArray,
                         (int) sizeof(struct defrule),(VOID *) DefruleArray);
#if DYNAMIC_SALIENCE
   DefruleArray[obji].dynamicSalience = ExpressionPointer(br->dynamicSalience);
#endif
   DefruleArray[obji].actions = ExpressionPointer(br->actions);
#if LOGICAL_DEPENDENCIES
   DefruleArray[obji].logicalJoin = BloadJoinPointer(br->logicalJoin);
#endif
   DefruleArray[obji].lastJoin = BloadJoinPointer(br->lastJoin);
   DefruleArray[obji].disjunct = BloadDefrulePointer(DefruleArray,br->disjunct);
   DefruleArray[obji].salience = br->salience;
   DefruleArray[obji].localVarCnt = br->localVarCnt;
   DefruleArray[obji].complexity = br->complexity;
   DefruleArray[obji].autoFocus = br->autoFocus;
   DefruleArray[obji].executing = 0;
   DefruleArray[obji].afterBreakpoint = 0;
#if DEBUGGING_FUNCTIONS
   DefruleArray[obji].watchActivation = WatchActivations;
   DefruleArray[obji].watchFiring = WatchRules;
#endif
  }
  
/*************************************************************/
/* UpdateJoin: Bload refresh routine for joinNode structure. */
/*************************************************************/
static VOID UpdateJoin(buf,obji)
  VOID *buf;
  long obji;
  {
   struct bsaveJoinNode *bj;
   
   bj = (struct bsaveJoinNode *) buf;
   JoinArray[obji].firstJoin = bj->firstJoin;
   JoinArray[obji].logicalJoin = bj->logicalJoin;
   JoinArray[obji].joinFromTheRight = bj->joinFromTheRight;
   JoinArray[obji].patternIsNegated = bj->patternIsNegated;
   JoinArray[obji].depth = bj->depth;
   JoinArray[obji].rhsType = bj->rhsType;
   JoinArray[obji].networkTest = HashedExpressionPointer(bj->networkTest);
   JoinArray[obji].nextLevel = BloadJoinPointer(bj->nextLevel);
   JoinArray[obji].lastLevel = BloadJoinPointer(bj->lastLevel);

   if (bj->joinFromTheRight == CLIPS_TRUE)
     { JoinArray[obji].rightSideEntryStructure =  (VOID *) BloadJoinPointer(bj->rightSideEntryStructure); }

   JoinArray[obji].rightMatchNode = BloadJoinPointer(bj->rightMatchNode); 
   JoinArray[obji].rightDriveNode = BloadJoinPointer(bj->rightDriveNode);
   JoinArray[obji].ruleToActivate = BloadDefrulePointer(DefruleArray,bj->ruleToActivate);
   JoinArray[obji].initialize = 0;
   JoinArray[obji].marked = 0;
   JoinArray[obji].bsaveID = 0L;
   JoinArray[obji].beta = NULL;
  }

/**********************************************************/
/* UpdatePatternNodeHeader: Updates the values in pattern */
/*   node headers from the loaded binary image.           */
/**********************************************************/
globle VOID UpdatePatternNodeHeader(theHeader,theBsaveHeader)
  struct patternNodeHeader *theHeader;
  struct bsavePatternNodeHeader *theBsaveHeader;
  {
   struct joinNode *theJoin;
   
   theHeader->singlefieldNode = theBsaveHeader->singlefieldNode;
   theHeader->multifieldNode = theBsaveHeader->multifieldNode;
   theHeader->stopNode = theBsaveHeader->stopNode;
   theHeader->beginSlot = theBsaveHeader->beginSlot;
   theHeader->endSlot = theBsaveHeader->endSlot;
   theHeader->initialize = 0;
   theHeader->marked = 0;
   theHeader->alphaMemory = NULL;
   theHeader->endOfQueue = NULL;
   
   theJoin = BloadJoinPointer(theBsaveHeader->entryJoin);
   theHeader->entryJoin = theJoin;

   while (theJoin != NULL)
     {
      theJoin->rightSideEntryStructure = (VOID *) theHeader;
      theJoin = theJoin->rightMatchNode;
     }
    
  }

/**********************************************************************/
/* ClearBload: Defrule clear routine when a binary load is in effect. */
/**********************************************************************/
static VOID ClearBload()
  {
   unsigned long int space;
   long i;
   struct patternParser *theParser = NULL;
   struct patternEntity *theEntity = NULL;
   VOID *theModule;

   /*===========================================*/
   /* Delete all known entities before removing */
   /* the defrule data structures.              */
   /*===========================================*/
   
   GetNextPatternEntity(&theParser,&theEntity);
   while (theEntity != NULL)
     {
      (*theEntity->theInfo->base.deleteFunction)(theEntity);
      theEntity = NULL;
      GetNextPatternEntity(&theParser,&theEntity);
     }
  
   /*=========================================*/
   /* Remove all activations from the agenda. */
   /*=========================================*/

   SaveCurrentModule();
   for (theModule = GetNextDefmodule(NULL);
        theModule != NULL;
        theModule = GetNextDefmodule(theModule))
     {
      SetCurrentModule(theModule);
      RemoveAllActivations();
     }
   RestoreCurrentModule();
   ClearFocusStack();

   /*==========================================================*/
   /* Remove all partial matches from the beta memories in the */
   /* join network. Alpha memories do not need to be examined  */
   /* since all pattern entities have been deleted by now.     */
   /*==========================================================*/

   for (i = 0; i < NumberOfJoins; i++)
     { FlushAlphaBetaMemory(JoinArray[i].beta); }

   /*================================================*/
   /* Decrement the symbol count for each rule name. */
   /*================================================*/

   for (i = 0; i < NumberOfDefrules; i++)
     { UnmarkConstructHeader(&DefruleArray[i].header); }

   /*==================================================*/
   /* Return the space allocated for the bload arrays. */
   /*==================================================*/

   space = NumberOfDefruleModules * sizeof(struct defruleModule);
   if (space != 0) genlongfree((VOID *) ModuleArray,space);
   
   space = NumberOfDefrules * sizeof(struct defrule);
   if (space != 0) genlongfree((VOID *) DefruleArray,space);

   space = NumberOfJoins * sizeof(struct joinNode);
   if (space != 0) genlongfree((VOID *) JoinArray,space);
  }

/*******************************************************/
/* BloadDefruleModuleReference: Returns the defrule    */
/*   module pointer for using with the bload function. */
/*******************************************************/
globle VOID *BloadDefruleModuleReference(index)
  int index;
  {
   return ((VOID *) &ModuleArray[index]);
  }

#endif /* DEFRULE_CONSTRUCT && (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE) && (! RUN_TIME) */


