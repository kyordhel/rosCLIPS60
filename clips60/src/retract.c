   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                   RETRACT MODULE                    */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _RETRACT_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_

#include "setup.h"

#if DEFRULE_CONSTRUCT

#include "constant.h"
#include "clipsmem.h"
#include "symbol.h"
#include "argacces.h"
#include "router.h" 
#include "network.h" 
#include "agenda.h" 
#include "match.h"
#include "drive.h" 

#if LOGICAL_DEPENDENCIES
#include "lgcldpnd.h" 
#endif

#include "retract.h"

#if ANSI_COMPILER
   static struct partialMatch    *RemovePartialMatches(struct alphaMatch *,
                                                      struct partialMatch *,
                                                      struct partialMatch **,int,
                                                      struct partialMatch **);
   static VOID                    DeletePartialMatches(struct partialMatch *,int);
   static VOID                    ReturnMarkers(struct multifieldMarker *);
   static VOID                    DriveRetractions(void);
   static BOOLEAN                 FindNextConflictingAlphaMatch(struct partialMatch *,
                                                                struct partialMatch *,
                                                                struct joinNode *);
#else
   static struct partialMatch    *RemovePartialMatches();
   static VOID                    DeletePartialMatches();
   static VOID                    ReturnMarkers();
   static VOID                    DriveRetractions();
   static BOOLEAN                 FindNextConflictingAlphaMatch();
#endif

/**************/
/* STRUCTURES */
/**************/

struct rdriveinfo
  {
   struct partialMatch *link;
   struct joinNode *jlist;
   struct rdriveinfo *next;
  };

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static struct rdriveinfo   *DriveRetractionList = NULL;

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct partialMatch *GarbagePartialMatches = NULL;
   globle struct alphaMatch   *GarbageAlphaMatches = NULL;

/************************************************************/
/* NetworkRetract:  Retracts a fact from the pattern and    */
/*   join networks given a pointer to the list of patterns  */
/*   which the fact matched. The fact is retracted from the */
/*   join network first through positive patterns, then     */
/*   through negated patterns, and then any new partial     */
/*   matches created by the retraction are driven through   */
/*   the join network. This ordering prevents partial       */
/*   matches being generated that contain the deleted fact. */
/************************************************************/
globle VOID NetworkRetract(listOfMatchedPatterns)
  struct patternMatch *listOfMatchedPatterns;
  {
   struct patternMatch *tempMatch;
   struct partialMatch *deletedMatches, *theLast;
   struct joinNode *joinPtr;

   /*===============================*/
   /* Delete for positive patterns. */
   /*===============================*/

   tempMatch = listOfMatchedPatterns;
   while (listOfMatchedPatterns != NULL)
     {
      /*================================================*/
      /* Loop through the list of all joins attached to */
      /* this pattern.                                  */
      /*================================================*/

      joinPtr = listOfMatchedPatterns->matchingPattern->entryJoin;

      while (joinPtr != NULL)
        {
         if (joinPtr->patternIsNegated == CLIPS_FALSE)
           { PosEntryRetract(joinPtr,
                             listOfMatchedPatterns->theMatch->binds[0].gm.theMatch,
                             listOfMatchedPatterns->theMatch,
                             (int) joinPtr->depth - 1,CLIPS_TRUE); }
         joinPtr = joinPtr->rightMatchNode;
        }

      listOfMatchedPatterns = listOfMatchedPatterns->next;
     }

   /*===============================*/
   /* Delete for negative patterns. */
   /*===============================*/

   listOfMatchedPatterns = tempMatch;
   while (listOfMatchedPatterns != NULL)
     {
      /*================================================*/
      /* Loop through the list of all joins attached to */
      /* this pattern.                                  */
      /*================================================*/

      joinPtr = listOfMatchedPatterns->matchingPattern->entryJoin;

      while (joinPtr != NULL)
        {
         /*========================================*/
         /* Handle retract based on logic of join. */
         /*========================================*/

         if (joinPtr->patternIsNegated == CLIPS_TRUE)
           {
            if (joinPtr->firstJoin == CLIPS_TRUE)
              {
               CLIPSSystemError("RETRACT",3);
               ExitCLIPS(5);
              }
            else
              { NegEntryRetract(joinPtr,listOfMatchedPatterns->theMatch,CLIPS_TRUE); }
           }

         joinPtr = joinPtr->rightMatchNode;
        }
        
      /*===================================================*/
      /* Remove from the alpha memory of the pattern node. */
      /*===================================================*/
      
      theLast = NULL;
      listOfMatchedPatterns->matchingPattern->alphaMemory = 
      RemovePartialMatches(listOfMatchedPatterns->theMatch->binds[0].gm.theMatch,
                                listOfMatchedPatterns->matchingPattern->alphaMemory,
                                &deletedMatches,0,&theLast);
     listOfMatchedPatterns->matchingPattern->endOfQueue = theLast;
                                
      DeletePartialMatches(deletedMatches,0);
      tempMatch = listOfMatchedPatterns->next;
      rtn_struct(patternMatch,listOfMatchedPatterns);
      listOfMatchedPatterns = tempMatch;
     }

   /*========================================*/
   /* Drive new partial matches generated by */
   /* retraction through the join network.   */
   /*========================================*/

   DriveRetractions();
  }
  
/***************************************************************/
/* PosEntryRetract:  Handles retract for a join of a rule with */
/*    a positive pattern when the retraction is starting from  */
/*    the RHS of that join (empty or positive LHS entry,       */
/*    positive RHS entry), or the LHS of that join (positive   */
/*    LHS entry, negative or positive RHS entry).              */
/***************************************************************/
globle VOID PosEntryRetract(join,theAlphaNode,theMatch,position,duringRetract)
  struct joinNode *join;
  struct alphaMatch *theAlphaNode;
  struct partialMatch *theMatch;
  int position;
  int duringRetract;
  {
   struct partialMatch *deletedMatches;
   struct joinNode *joinPtr;
   struct partialMatch *theLast;

   while (join != NULL)
     {
      /*=========================================*/
      /* Remove the bindings from this join that */
      /* contain the fact to be retracted.       */
      /*=========================================*/

      if (join->beta == NULL) return; /* optimize */
      
      join->beta = RemovePartialMatches(theAlphaNode,join->beta,&deletedMatches,
                                        position,&theLast);

      /*===================================================*/
      /* If no facts were deleted at this join, then there */
      /* is no need to check joins at a lower level.       */
      /*===================================================*/

      if (deletedMatches == NULL) return;

      /*==================================================*/
      /* If there is more than one join below this join,  */
      /* then recursively remove fact bindings from all   */
      /* but one of the lower joins.  Remove the bindings */
      /* from the other join through this loop.           */
      /*==================================================*/

      joinPtr = join->nextLevel;
      if (joinPtr == NULL) 
        {
         DeletePartialMatches(deletedMatches,1);
         return;
        }

      if (((struct joinNode *) (joinPtr->rightSideEntryStructure)) == join)
        { 
         theMatch = deletedMatches;
         while (theMatch != NULL)
           {
            NegEntryRetract(joinPtr,theMatch,duringRetract);
            theMatch = theMatch->next; 
           }
           
         DeletePartialMatches(deletedMatches,1);
         return;
        }
      
      DeletePartialMatches(deletedMatches,1);
      while (joinPtr->rightDriveNode != NULL)
        {
         PosEntryRetract(joinPtr,theAlphaNode,theMatch,position,duringRetract);
         joinPtr = joinPtr->rightDriveNode;
        }

      join = joinPtr;
     }
  }

/*****************************************************************/
/* NegEntryRetract:  Handles retract for a join of a rule with a */
/*    negated pattern when the retraction is starting from the   */
/*    RHS of that join (positve LHS entry, negative RHS entry,   */
/*    retraction starting from RHS).                             */
/*****************************************************************/
VOID NegEntryRetract(theJoin,theMatch,duringRetract)
  struct joinNode *theJoin;
  struct partialMatch *theMatch;
  int duringRetract;
  {
   struct partialMatch *theLHS;
   int result;
   struct rdriveinfo *tempDR;
   struct alphaMatch *tempAlpha;
   struct joinNode *listOfJoins;

   /*========================================================*/
   /* Loop through all LHS partial matches checking for sets */
   /* that satisfied the join expression.                    */
   /*========================================================*/

   for (theLHS = theJoin->beta; theLHS != NULL; theLHS = theLHS->next)
     {
      /*===========================================================*/
      /* Don't bother checking partial matches that are satisfied. */
      /*===========================================================*/
         
      if (theLHS->counterf == CLIPS_FALSE) continue;
      
      /*==================================================*/
      /* If the partial match being removed isn't the one */
      /* preventing the LHS partial match from being      */
      /* satisifed, then don't bother processing it.      */
      /*==================================================*/
                                            
      if (theLHS->binds[theLHS->bcount - 1].gm.theValue != (VOID *) theMatch) continue;
      
      /*================================================*/
      /* Try to find another RHS bind which prevents the */
      /* partial match from being satisifed.            */
      /*================================================*/

      theLHS->binds[theLHS->bcount - 1].gm.theValue = NULL;
      result = FindNextConflictingAlphaMatch(theLHS,theMatch->next,theJoin);

      /*=========================================================*/
      /* If the LHS partial match now has no RHS partial matches */
      /* that conflict with it, then it satisfies the conditions */
      /* of the RHS not CE. Create a partial match and send it   */
      /* to the joins below.                                     */
      /*=========================================================*/
      
      if (result == CLIPS_FALSE)
        {
         theLHS->counterf = CLIPS_FALSE;
         tempAlpha = get_struct(alphaMatch);
         tempAlpha->next = NULL;
         tempAlpha->matchingItem = NULL;
         tempAlpha->markers = NULL;
         theLHS->binds[theLHS->bcount - 1].gm.theMatch = tempAlpha;

         if (theJoin->ruleToActivate != NULL) 
           { AddActivation(theJoin->ruleToActivate,theLHS); }
              
         listOfJoins = theJoin->nextLevel;
         if (listOfJoins != NULL)
           {
            if (((struct joinNode *) (listOfJoins->rightSideEntryStructure)) == theJoin)
              { Drive(theLHS,listOfJoins,RHS); }
            else
              {
               if (duringRetract)
                 {
                  tempDR = get_struct(rdriveinfo);
                  tempDR->link = theLHS;
                  tempDR->jlist = theJoin->nextLevel;
                  tempDR->next = DriveRetractionList;
                  DriveRetractionList = tempDR;
                 }
               else while (listOfJoins != NULL)
                 {  
                  Drive(theLHS,listOfJoins,LHS); 
                  listOfJoins = listOfJoins->rightDriveNode;
                 }
              }
           }
        }
     }
  }

/*************************************************************/
/* FindNextConflictingAlphaMatch:                           */
/*************************************************************/
static BOOLEAN FindNextConflictingAlphaMatch(theBind,possibleConflicts,theJoin)
  struct partialMatch *theBind;
  struct partialMatch *possibleConflicts;
  struct joinNode *theJoin;
  {
   int i, result;
        
   if (theJoin->joinFromTheRight)
     { possibleConflicts = ((struct joinNode *) theJoin->rightSideEntryStructure)->beta; }
           
   while (possibleConflicts != NULL)
     {
      result = CLIPS_FALSE;
      
      if (possibleConflicts->counterf)
        { /* Do Nothing */ }
      else if (theJoin->networkTest == NULL)
        { 
         result = CLIPS_TRUE;
         if (theJoin->joinFromTheRight)
           {
            for (i = 0; i < (theBind->bcount - 1); i++)
              {
               if (possibleConflicts->binds[i].gm.theMatch != theBind->binds[i].gm.theMatch)
                 {
                  result = CLIPS_FALSE;
                  break;
                 }
              }
           }
        }
      else
        {    
         result = EvaluateJoinExpression(theJoin->networkTest,theBind,
                                         possibleConflicts,theJoin);
         if (EvaluationError)
           {
            result = CLIPS_TRUE;
            EvaluationError = CLIPS_FALSE;
           }
        }       
      
      /*================================================*/
      /* If the join expression evaluated to true, then */
      /* the LHS partial match will have one less RHS   */
      /* partial match that conflicts with it.          */
      /*================================================*/

      if (result != CLIPS_FALSE)
        {
         theBind->binds[theBind->bcount - 1].gm.theValue = (VOID *) possibleConflicts;
         return(CLIPS_TRUE);
        }
      
      possibleConflicts = possibleConflicts->next;
     }
     
   return(CLIPS_FALSE);
  }
      
/*************************************************************/
/* RemovePartialMatches: Searches through a list of partial  */
/*   matches and removes any partial match that contains the */
/*   specified fact identifier.                              */
/*************************************************************/
static struct partialMatch *RemovePartialMatches(theAlphaNode,listOfPMs,deleteHead,
                                                 position,returnLast)
  struct alphaMatch *theAlphaNode;
  struct partialMatch *listOfPMs;
  struct partialMatch **deleteHead;
  int position;
  struct partialMatch **returnLast;
  {
   struct partialMatch *head, *lastPM, *nextPM;
   struct partialMatch *lastDelete = NULL;

   lastPM = NULL;
   head = listOfPMs;
   lastPM = listOfPMs;
   
   *deleteHead = NULL;
   while (listOfPMs != NULL)
     {
      if ((listOfPMs->counterf == TRUE) && (position == (listOfPMs->bcount - 1)))
        {
         /* Is this really necessary? */
         lastPM = listOfPMs;
         listOfPMs = listOfPMs->next;
        }
      else if (listOfPMs->binds[position].gm.theMatch == theAlphaNode)
        {
         if ((listOfPMs->activationf) ?
             (listOfPMs->binds[listOfPMs->bcount].gm.theValue != NULL) : FALSE)
           { RemoveActivation((struct activation *) listOfPMs->binds[listOfPMs->bcount].gm.theValue,CLIPS_TRUE,CLIPS_TRUE); }

         if (listOfPMs == head)
           {
            /* Delete bind at beginning of list. */
            nextPM = listOfPMs->next;

            if (*deleteHead == NULL)
              { *deleteHead = listOfPMs; }
            else
              { lastDelete->next = listOfPMs; }
            listOfPMs->next = NULL;
            lastDelete = listOfPMs;
               
            listOfPMs = nextPM;
            head = listOfPMs;
            lastPM = head;
           }
         else
           {
            /* Delete bind after beginning of list. */
            lastPM->next = listOfPMs->next;

            if (*deleteHead == NULL)
              { *deleteHead = listOfPMs; }
            else
              { lastDelete->next = listOfPMs; }
            listOfPMs->next = NULL;
            lastDelete = listOfPMs;
            
            listOfPMs = lastPM->next;
           }
        }
      else
        {
         lastPM = listOfPMs;
         listOfPMs = listOfPMs->next;
        }
     }
     
   *returnLast = lastPM;
   return(head);
  }
  
/*************************************************************/
/* DeletePartialMatches:                             */
/*************************************************************/
static VOID DeletePartialMatches(listOfPMs,betaDelete)
  struct partialMatch *listOfPMs;
  int betaDelete;
  {
   struct partialMatch *nextPM;

   while (listOfPMs != NULL)
     {
      nextPM = listOfPMs->next;
#if LOGICAL_DEPENDENCIES
      if (listOfPMs->dependentsf) AddToDependencyList(listOfPMs);
#endif
      if (betaDelete && 
          ((listOfPMs->notOriginf == CLIPS_FALSE) || (listOfPMs->counterf)))
        { ReturnPartialMatch(listOfPMs); }
      else
        {
         listOfPMs->next = GarbagePartialMatches;
         GarbagePartialMatches = listOfPMs;
        }
        
      listOfPMs = nextPM;
     }
  }

/***************************************************************/
/* ReturnPartialMatch:  Returns the data structures associated */
/*   with a partial match to the pool of free memory.          */
/***************************************************************/
globle VOID ReturnPartialMatch(waste)
  struct partialMatch *waste;
  {
   if (waste->busy)
     {
      waste->next = GarbagePartialMatches;
      GarbagePartialMatches = waste;
      return;
     }

   if (waste->betaMemory == CLIPS_FALSE)
     {
      if (waste->binds[0].gm.theMatch->markers != NULL)
        { ReturnMarkers(waste->binds[0].gm.theMatch->markers); }
      rm(waste->binds[0].gm.theMatch,(int) sizeof(struct alphaMatch));
     }

#if LOGICAL_DEPENDENCIES
   if (waste->dependentsf) RemovePMDependencies(waste);
#endif

   rtn_var_struct(partialMatch,(int) sizeof(struct genericMatch *) *
                  (waste->bcount + waste->activationf + waste->dependentsf - 1),
                  waste);

  }

/************************************************************/
/* ReturnMarkers                                            */
/************************************************************/
static VOID ReturnMarkers(waste)
  struct multifieldMarker *waste;
  {
   struct multifieldMarker *temp;

   while (waste != NULL)
     {
      temp = waste->next;
      rtn_struct(multifieldMarker,waste);
      waste = temp;
     }
  }

/***********************************/
/* DriveRetractions:              */
/***********************************/
static VOID DriveRetractions()
  {
   struct rdriveinfo *tempDR;
   struct joinNode *joinPtr;

   while (DriveRetractionList != NULL)
     {
      joinPtr = DriveRetractionList->jlist;
      while (joinPtr != NULL)
        {
         Drive(DriveRetractionList->link,joinPtr,LHS);
         joinPtr = joinPtr->rightDriveNode;
        }

      tempDR = DriveRetractionList->next;
      rtn_struct(rdriveinfo,DriveRetractionList);
      DriveRetractionList = tempDR;
     }
  }

/************************************************************************/
/* FlushGarbagePartialMatches:  Returns partial matches and associated  */
/*   structures that were removed as part of a retraction. It is        */
/*   necessary to postpone returning these structures to memory because */
/*   RHS actions retrieve their variable bindings directly from the     */
/*   fact data structure through the alpha memory bindings.             */
/************************************************************************/
globle VOID FlushGarbagePartialMatches()
  {
   struct partialMatch *pmPtr;
   struct alphaMatch *amPtr;

   /* The alpha matches here should only belong to the */
   /* fake matches created for negated pseudo facts.   */

   while (GarbageAlphaMatches != NULL)
     {
      amPtr = GarbageAlphaMatches->next;

      rtn_struct(alphaMatch,GarbageAlphaMatches);

      GarbageAlphaMatches = amPtr;
     }

   while (GarbagePartialMatches != NULL)
     {
      pmPtr = GarbagePartialMatches->next;
      if ((GarbagePartialMatches->notOriginf) &&
          (GarbagePartialMatches->counterf == CLIPS_FALSE))
        {
         if (GarbagePartialMatches->binds[GarbagePartialMatches->bcount - 1].gm.theMatch != NULL)
           {
            rtn_struct(alphaMatch,
                       GarbagePartialMatches->binds[GarbagePartialMatches->bcount - 1].gm.theMatch);
           }
        }

      GarbagePartialMatches->busy = CLIPS_FALSE;
      ReturnPartialMatch(GarbagePartialMatches);

      GarbagePartialMatches = pmPtr;
     }
  }

#endif

