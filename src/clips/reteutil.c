   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 RETE UTILITY MODULE                 */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*      Provides a set of utility functions useful to other  */
/*      modules.                                             */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _RETEUTIL_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_

#include "setup.h"

#if DEFRULE_CONSTRUCT

#include "clipsmem.h"
#include "router.h"
#include "retract.h" 
#include "drive.h" 
#include "incrrset.h"
#include "pattern.h"
#include "match.h"
#include "moduldef.h"
#include "reteutil.h"

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct partialMatch   *GlobalLHSBinds = NULL;
   globle struct partialMatch   *GlobalRHSBinds = NULL;
   globle struct joinNode       *GlobalJoin = NULL;

/************************************************************/
/* PrintPartialMatch: Prints out a list of fact id numbers  */
/*   associated with a partial match or rule instantiation. */
/************************************************************/
globle VOID PrintPartialMatch(logicalName,list)
  char *logicalName;
  struct partialMatch *list;
  {
   struct patternEntity *matchingItem;
   short int i;

   for (i = 0; i < list->bcount;)
     {
      if (get_nth_pm_match(list,i)->matchingItem != NULL)
        {
         matchingItem = get_nth_pm_match(list,i)->matchingItem;
         if (matchingItem != NULL) (*matchingItem->theInfo->base.shortPrintFunction)(logicalName,matchingItem);
        }
      i++;
      if (i < list->bcount) PrintCLIPS(logicalName,",");
     }
  }

/******************************************************/
/* CopyPartialMatch:  Copies a list of fact bindings. */
/******************************************************/
globle struct partialMatch *CopyPartialMatch(list,addActivationSlot,addDependencySlot)
  struct partialMatch *list;
  int addActivationSlot;
  int addDependencySlot;
  {
   struct partialMatch *linker;
   short int i;

   linker = get_var_struct(partialMatch,sizeof(struct genericMatch) *
                                        (list->bcount + addActivationSlot + addDependencySlot - 1));

   linker->next = NULL;
   linker->betaMemory = CLIPS_TRUE;
   linker->busy = CLIPS_FALSE;
   linker->activationf = addActivationSlot;
   linker->dependentsf = addDependencySlot;
   linker->notOriginf = CLIPS_FALSE;
   linker->counterf = CLIPS_FALSE;
   linker->bcount = list->bcount;

   for (i = 0; i < linker->bcount; i++) linker->binds[i] = list->binds[i];

   if (addActivationSlot) linker->binds[i++].gm.theValue = NULL;
   if (addDependencySlot) linker->binds[i++].gm.theValue = NULL;

   return(linker);
  }

/*******************************************************/
/* MergePartialMatches:  Merges two lists of bindings. */
/*******************************************************/
globle struct partialMatch *MergePartialMatches(list1,list2,addActivationSlot,addDependencySlot)
  struct partialMatch *list1, *list2;
  int addActivationSlot;
  int addDependencySlot;
  {
   struct partialMatch *linker;
   short int i, j;

   linker = get_var_struct(partialMatch,
                           sizeof(struct genericMatch) *
                            (list1->bcount + list2->bcount + addActivationSlot + addDependencySlot - 1));

   linker->next = NULL;
   linker->betaMemory = CLIPS_TRUE;
   linker->busy = CLIPS_FALSE;
   linker->activationf = addActivationSlot;
   linker->dependentsf = addDependencySlot;
   linker->notOriginf = CLIPS_FALSE;
   linker->counterf = CLIPS_FALSE;
   linker->bcount = list1->bcount + list2->bcount;

   for (i = 0; i < list1->bcount; i++)
     { linker->binds[i] = list1->binds[i]; }

   for (i = list1->bcount, j = 0; i < linker->bcount; i++, j++)
     { linker->binds[i] = list2->binds[j]; }

   if (addActivationSlot) linker->binds[i++].gm.theValue = NULL;
   if (addDependencySlot) linker->binds[i].gm.theValue = NULL;

   return(linker);
  }
  
/***********************************************/
/* InitializePatternHeader:    */
/***********************************************/
globle VOID InitializePatternHeader(theHeader)
  struct patternNodeHeader *theHeader;
  {
   theHeader->entryJoin = NULL;
   theHeader->alphaMemory = NULL; 
   theHeader->endOfQueue = NULL;
   theHeader->singlefieldNode = CLIPS_FALSE;
   theHeader->multifieldNode = CLIPS_FALSE;
   theHeader->stopNode = CLIPS_FALSE;
#if INCREMENTAL_RESET && (! RUN_TIME)
   theHeader->initialize = GetIncrementalReset();
#else
   theHeader->initialize = CLIPS_FALSE;
#endif
   theHeader->marked = CLIPS_FALSE;
   theHeader->beginSlot = CLIPS_FALSE;
   theHeader->endSlot = CLIPS_FALSE;
  }
  
/******************************************************************/
/* CreateAlphaMatch: Given a pointer to an entity (such as a fact */
/*   or instance) which matched a pattern, this function creates  */
/*   a partial match suitable for storing in the alpha memory of  */
/*   the pattern network. Note that the multifield marker which   */
/*   are passed as a calling argument are copied (thus the caller */
/*   is still responsible for freeing these data structures.      */
/******************************************************************/
globle struct partialMatch *CreateAlphaMatch(theEntity,markers,theHeader)
  VOID *theEntity;
  struct multifieldMarker *markers;
  struct patternNodeHeader *theHeader;
  {
   struct partialMatch *theMatch;
   struct alphaMatch *afbtemp;
   
   theMatch = get_struct(partialMatch);
   theMatch->next = NULL;
   theMatch->betaMemory = CLIPS_FALSE;
   theMatch->busy = CLIPS_FALSE;
   theMatch->activationf = CLIPS_FALSE;
   theMatch->dependentsf = CLIPS_FALSE;
   theMatch->notOriginf = CLIPS_FALSE;
   theMatch->counterf = CLIPS_FALSE;
   theMatch->dependentsf = CLIPS_FALSE;
   theMatch->bcount = 1;

   afbtemp = get_struct(alphaMatch);
   afbtemp->next = NULL;
   afbtemp->matchingItem = (struct patternEntity *) theEntity;
   
   if (markers != NULL)
     { afbtemp->markers = CopyMultifieldMarkers(markers); }
   else
     { afbtemp->markers = NULL; }
     
   theMatch->binds[0].gm.theMatch = afbtemp;
   
   if (theHeader->endOfQueue == NULL)
     { 
      theHeader->alphaMemory = theMatch;
      theHeader->endOfQueue = theMatch;
     }
   else
     {
      theHeader->endOfQueue->next = theMatch;
      theHeader->endOfQueue = theMatch;
     }

   return(theMatch);
  }

/*********************************************************/
/* AddSingleMatch: Adds a single bind to a list of binds */
/*   creating a completely new list.                     */
/*********************************************************/
globle struct partialMatch *AddSingleMatch(list,afb,addActivationSlot,addDependencySlot)
  struct partialMatch *list;
  struct alphaMatch *afb;
  int addActivationSlot;
  int addDependencySlot;
  {
   struct partialMatch *linker;
   short int i;

   linker = get_var_struct(partialMatch,sizeof(struct genericMatch) * 
                                        (list->bcount + addActivationSlot + addDependencySlot));

   linker->next = NULL;
   linker->betaMemory = CLIPS_TRUE;
   linker->busy = CLIPS_FALSE;
   linker->activationf = addActivationSlot;
   linker->dependentsf = addDependencySlot;
   linker->notOriginf = CLIPS_FALSE;
   linker->counterf = CLIPS_FALSE;
   linker->bcount = list->bcount + 1;

   for (i = 0; i < list->bcount; i++)
     { linker->binds[i] = list->binds[i]; }

   set_nth_pm_match(linker,i++,afb);
   
   if (addActivationSlot) linker->binds[i++].gm.theValue = NULL;
   if (addDependencySlot) linker->binds[i].gm.theValue = NULL;

   return(linker);
  }
  
/********************************************/
/* CopyMultifieldMarkers:  Copies a list of */
/*   multifieldMarker data structures.      */
/********************************************/
struct multifieldMarker *CopyMultifieldMarkers(theMarkers)
  struct multifieldMarker *theMarkers;
  {
   struct multifieldMarker *head = NULL, *lastMark = NULL, *newMark;

   while (theMarkers != NULL)
     {
      newMark = get_struct(multifieldMarker);
      newMark->next = NULL;
      newMark->whichField = theMarkers->whichField;
      newMark->where = theMarkers->where;
      newMark->startPosition = theMarkers->startPosition;
      newMark->endPosition = theMarkers->endPosition;

      if (lastMark == NULL)
        { head = newMark; }
      else
        { lastMark->next = newMark; }
      lastMark = newMark;

      theMarkers = theMarkers->next;
     }

   return(head);
  }

/****************************************************************/
/* NewPseudoFactPartialMatch:  Creates a partial structure that */
/*   indicates the "pseudo" fact to which a not pattern CE has  */
/*   been bound. Since a non-existant fact has no fact index,   */
/*   the partial match structure is given a pseudo fact index   */
/*   (a unique negative integer).                               */
/****************************************************************/
globle struct partialMatch *NewPseudoFactPartialMatch()
  {
   struct partialMatch *linker;
   struct alphaMatch *tempAlpha;

   linker = get_struct(partialMatch);
   linker->next = NULL;
   linker->betaMemory = CLIPS_TRUE;
   linker->busy = CLIPS_FALSE;
   linker->activationf = CLIPS_FALSE;
   linker->dependentsf = CLIPS_FALSE;
   linker->notOriginf = CLIPS_TRUE;
   linker->counterf = CLIPS_FALSE;
   linker->bcount = 0;
   tempAlpha = get_struct(alphaMatch);
   tempAlpha->next = NULL;
   tempAlpha->matchingItem = NULL;
   tempAlpha->markers = NULL;

   linker->binds[0].gm.theMatch = tempAlpha;
   return(linker);
  }

/******************************************************************/
/* FlushAlphaBetaMemory: Returns all partial matches in a list of */
/*   partial matches either directly to the pool of free memory   */
/*   or to the list of GarbagePartialMatches. Partial matches     */
/*   stored in alpha memories and partial matches which store the */
/*   information for pseudo facts (for not CEs) may be referred   */
/*   to by other data structures and thus must be placed on the   */
/*   list of GarbagePartialMatches.                               */
/******************************************************************/
globle VOID FlushAlphaBetaMemory(pfl)
  struct partialMatch *pfl;
  {
   struct partialMatch *pfltemp;

   while (pfl != NULL)
     {
      pfltemp = pfl->next;
      if (((pfl->notOriginf) && (pfl->counterf == CLIPS_FALSE)) || 
          (pfl->betaMemory == CLIPS_FALSE))
        {
         pfl->next = GarbagePartialMatches;
         GarbagePartialMatches = pfl;
        }
      else
        { ReturnPartialMatch(pfl); }

      pfl = pfltemp;
     }
  }
  
/************************************************************************/
/* TraceErrorToRule: Prints an error message when a error occurs as the */
/*   result of evaluating an expression in the pattern network. Used to */
/*   indicate which rule caused the problem.                            */
/************************************************************************/
globle VOID TraceErrorToRule(joinPtr,indentSpaces)
  struct joinNode *joinPtr;
  char *indentSpaces;
  {
   char *name;

   while (joinPtr != NULL)
     {
      if (joinPtr->ruleToActivate != NULL)
        {
         name = GetDefruleName((VOID *) joinPtr->ruleToActivate);
         PrintCLIPS(WERROR,indentSpaces);
         PrintCLIPS(WERROR,name);
         PrintCLIPS(WERROR,"\n");
        }
      else
        { TraceErrorToRule(joinPtr->nextLevel,indentSpaces); }

      joinPtr = joinPtr->rightDriveNode;
     }

  }
  
#if (CONSTRUCT_COMPILER || BLOAD_AND_BSAVE) && (! RUN_TIME)

/****************************************************************/
/* TagRuleNetwork: Assigns each join in the join network with a */
/*   unique integer ID. Also counts the number of defrule and   */
/*   joinNode data structures currently in use.                 */
/****************************************************************/
globle VOID TagRuleNetwork(moduleCount,ruleCount,joinCount)
  long int *moduleCount, *ruleCount, *joinCount;
  {
   struct defmodule *modulePtr;
   struct defrule *rulePtr;
   struct joinNode *joinPtr;

   *moduleCount = 0;
   *ruleCount = 0;
   *joinCount = 0;
   
   MarkRuleNetwork(0);
   
   modulePtr = (struct defmodule *) GetNextDefmodule(NULL);
   while (modulePtr != NULL)
     {
      (*moduleCount)++;
      SetCurrentModule((VOID *) modulePtr);
      
      rulePtr = (struct defrule *) GetNextDefrule(NULL);

      while (rulePtr != NULL)
        {
         rulePtr->header.bsaveID = *ruleCount;
         (*ruleCount)++;
         joinPtr = rulePtr->lastJoin;
         while (joinPtr != NULL) 
           {
            if (joinPtr->marked == 0)
              {
               joinPtr->marked = 1;
               joinPtr->bsaveID = *joinCount;
               (*joinCount)++;
              }
              
            joinPtr = GetPreviousJoin(joinPtr);
           }
   
         if (rulePtr->disjunct != NULL) rulePtr = rulePtr->disjunct;
         else rulePtr = (struct defrule *) GetNextDefrule(rulePtr);
        }
        
      modulePtr = (struct defmodule *) GetNextDefmodule(modulePtr);
     }
  }
    
/********************************************************/
/* MarkRuleNetwork: Sets the marked flag in each of the */
/*   joins in the join network to the specified value.  */
/********************************************************/
globle VOID MarkRuleNetwork(value)
  int value;
  {
   struct defrule *rulePtr;
   struct joinNode *joinPtr;
   struct defmodule *modulePtr;

   /*==============================================*/
   /* Loop through each rule in the list of rules. */
   /*==============================================*/
      
   modulePtr = (struct defmodule *) GetNextDefmodule(NULL);
   while (modulePtr != NULL)
     {
      SetCurrentModule((VOID *) modulePtr);
      
      rulePtr = (struct defrule *) GetNextDefrule(NULL);
      while (rulePtr != NULL)
        {
         /*=============================*/
         /* Mark each join for the rule */
         /* with the specified value.   */
         /*=============================*/
      
         joinPtr = rulePtr->lastJoin;
         while (joinPtr != NULL)
           {
            joinPtr->marked = value;
            joinPtr = GetPreviousJoin(joinPtr);
           }

         /*=================================*/
         /* Move on to the next rule or the */
         /* next disjunct for this rule.    */
         /*=================================*/
      
         if (rulePtr->disjunct != NULL) rulePtr = rulePtr->disjunct;
         else rulePtr = (struct defrule *) GetNextDefrule(rulePtr);
        }
        
      modulePtr = (struct defmodule *) GetNextDefmodule(modulePtr);
     }
  }
  
#endif

#endif





