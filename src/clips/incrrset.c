   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*              INCREMENTAL RESET MODULE               */
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

#define _INCRRSET_SOURCE_

#include "setup.h"

#include <stdio.h>
#define _CLIPS_STDIO_

#if DEFRULE_CONSTRUCT && INCREMENTAL_RESET 

#include "constant.h"
#include "agenda.h"
#include "router.h"
#include "drive.h" 
#include "pattern.h" 
#include "evaluatn.h"
#include "argacces.h"
#include "incrrset.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if (! RUN_TIME) && (! BLOAD_ONLY)
#if ANSI_COMPILER
   static VOID                    MarkNetworkForIncrementalReset(struct defrule *,int);
   static VOID                    CheckForPrimableJoins(struct defrule *);
   static VOID                    PrimeJoin(struct joinNode *);
   static VOID                    MarkPatternForIncrementalReset(int,struct patternNodeHeader *,int);
#else
   static VOID                    MarkNetworkForIncrementalReset();
   static VOID                    CheckForPrimableJoins();
   static VOID                    PrimeJoin();
   static VOID                    MarkPatternForIncrementalReset();
#endif
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle BOOLEAN              IncrementalResetInProgress = CLIPS_FALSE;

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static BOOLEAN              IncrementalResetFlag = CLIPS_TRUE;

/**************************************************************/
/* IncrementalReset: Incrementally resets the specified rule. */
/**************************************************************/
globle VOID IncrementalReset(tempRule)
  struct defrule *tempRule;
  {
#if (! RUN_TIME) && (! BLOAD_ONLY)
   struct defrule *tempPtr;
   struct patternParser *theParser = NULL;

   if (! GetIncrementalReset()) return;
   
   MarkNetworkForIncrementalReset(tempRule,CLIPS_TRUE);

   /*==========================*/
   /* Begin incremental reset. */
   /*==========================*/

   IncrementalResetInProgress = CLIPS_TRUE;

   /*============================================================*/
   /* If the new rule shares patterns or joins with other rules, */
   /* then it is necessary to update its join network based on   */
   /* existing partial matches it shares with other rules.       */
   /*============================================================*/

   tempPtr = tempRule;
   while (tempPtr != NULL)
     {
      CheckForPrimableJoins(tempPtr);
      tempPtr = tempPtr->disjunct;
     }

   /*===============================================*/
   /* Filter existing data entities through the new */
   /* portions of the pattern and join networks.    */
   /*===============================================*/

   theParser = ListOfPatternParsers;
   while (theParser != NULL)
     {
      if (theParser->incrementalResetFunction != NULL)
        { (*theParser->incrementalResetFunction)(); }
        
      theParser = theParser->next;
     }

   /*========================*/
   /* End incremental reset. */
   /*========================*/

   IncrementalResetInProgress = CLIPS_FALSE;
   
   MarkNetworkForIncrementalReset(tempRule,CLIPS_FALSE);
#endif
  }
  
#if (! RUN_TIME) && (! BLOAD_ONLY)

/**********************************************************************/
/* MarkNetworkForIncrementalReset: Coordinates marking the initialize */
/*   flags in the pattern and join networks both before and after an  */
/*   incremental reset.                                               */
/**********************************************************************/
static VOID MarkNetworkForIncrementalReset(tempRule,value)
  struct defrule *tempRule;
  int value;
  {
   struct joinNode *joinPtr;
   struct patternNodeHeader *patternPtr;

   while (tempRule != NULL)
     {
      joinPtr = tempRule->lastJoin;
      
      while (joinPtr != NULL)
        {
         if ((joinPtr->initialize) && (joinPtr->joinFromTheRight == CLIPS_FALSE))
           {
            joinPtr->initialize = value;
            patternPtr = (struct patternNodeHeader *) GetPatternForJoin(joinPtr);
            
            MarkPatternForIncrementalReset((int) joinPtr->rhsType,patternPtr,value);
           }

         joinPtr = GetPreviousJoin(joinPtr);
        }
        
      tempRule = tempRule->disjunct;
     }
  }
    
/*******************************************************************************/
/* CheckForPrimableJoins: Updates the joins of a rule for an incremental reset */
/*   if portions of that rule are shared with other rules that have already    */
/*   been incrementally reset. A join for a new rule will be updated if it is  */
/*   marked for initialization and either its parent join or its associated    */
/*   entry pattern node has not been marked for initialization. The function   */
/*   PrimeJoin is used to update joins which meet these criteria.              */
/*******************************************************************************/
static VOID CheckForPrimableJoins(tempRule)
  struct defrule *tempRule;
  {
   struct joinNode *joinPtr;
   struct partialMatch *theList;

   joinPtr = tempRule->lastJoin;
   while (joinPtr != NULL)
     {
      if (joinPtr->initialize)
        {
         if (joinPtr->firstJoin == CLIPS_TRUE)
           {
            if (((struct patternNodeHeader *) GetPatternForJoin(joinPtr))->initialize == CLIPS_FALSE)
              { PrimeJoin(joinPtr); }
           }
         else if (joinPtr->lastLevel->initialize == CLIPS_FALSE)
           { PrimeJoin(joinPtr); }
        }
      else if (joinPtr->ruleToActivate == tempRule)
        {
         theList = joinPtr->beta;
         while (theList != NULL)
           {
            AddActivation(tempRule,theList);
            theList = theList->next;
           }
        }

      joinPtr = GetPreviousJoin(joinPtr);
     }
  }
  
/****************************************************************************/
/* PrimeJoin: Updates a join in a rule for an incremental reset. Joins are  */
/*   updated by "priming" them only if the join (or its associated pattern) */
/*   is shared with other rules that have already been incrementally reset. */
/*   A join for a new rule will be updated if it is marked for              */
/*   initialization and either its parent join or its associated entry      */
/*   pattern node has not been marked for initialization.                   */
/****************************************************************************/
static VOID PrimeJoin(joinPtr)
  struct joinNode *joinPtr;
  {
   struct partialMatch *theList;

   /*===========================================================*/
   /* If the join is the first join of a rule, then send all of */
   /* the partial matches from the alpha memory of the pattern  */
   /* associated with this join to the join for processing and  */
   /* the priming process is then complete.                     */
   /*===========================================================*/

   if (joinPtr->firstJoin == CLIPS_TRUE)
     {
      for (theList = ((struct patternNodeHeader *) joinPtr->rightSideEntryStructure)->alphaMemory;
           theList != NULL;
           theList = theList->next)
        { Drive(theList,joinPtr,RHS); }
      return;
     }

   /*======================================================*/
   /* If the join already has partial matches in its beta  */
   /* memory, then don't bother priming it. I don't recall */
   /* if this situation is possible.                       */
   /*======================================================*/

   if (joinPtr->beta != NULL) return;

   /*================================================================*/
   /* Send all partial matches from the preceding join to this join. */
   /*================================================================*/

   for (theList = joinPtr->lastLevel->beta; 
        theList != NULL; 
        theList = theList->next)
     { Drive(theList,joinPtr,LHS); }
  }

/*********************************************************************/
/* MarkPatternForIncrementalReset: Given a pattern node and its type */
/*   (fact, instance, etc.), calls the appropriate function to mark  */
/*   the pattern for an incremental reset. Used to mark the pattern  */
/*   nodes both before and after an incremental reset.               */
/*********************************************************************/
static VOID MarkPatternForIncrementalReset(rhsType,theHeader,value)
  int rhsType;
  struct patternNodeHeader *theHeader;
  int value;
  {
   struct patternParser *tempParser;

   tempParser = GetPatternParser(rhsType);
   
   if (tempParser != NULL) 
     {
      if (tempParser->markIRPatternFunction != NULL)
        { (*tempParser->markIRPatternFunction)(theHeader,value); }
     }
  }

#endif

/*****************************************************/
/* GetIncrementalReset: Returns the current value of */
/*   the IncrementalReset flag.                      */
/*****************************************************/
globle BOOLEAN GetIncrementalReset()
  { return(IncrementalResetFlag); }

/**************************************************/
/* SetIncrementalReset: Sets the current value of */
/*   the IncrementalReset flag.                   */
/**************************************************/
globle BOOLEAN SetIncrementalReset(value)
  int value;
  {
   int ov;

   ov = IncrementalResetFlag;
   if (GetNextDefrule(NULL) != NULL) return(-1);
   IncrementalResetFlag = value;
   return(ov);
  }
  
/****************************************************/
/* SetIncrementalResetCommand: CLIPS access routine */
/*   for the set-incremental-reset command.         */
/****************************************************/
globle int SetIncrementalResetCommand()
  {
   int oldValue;
   DATA_OBJECT argPtr;

   oldValue = GetIncrementalReset();

   /*============================================*/
   /* Check for the correct number of arguments. */
   /*============================================*/
   
   if (ArgCountCheck("set-incremental-reset",EXACTLY,1) == -1)
     { return(oldValue); }

   /*=========================================*/
   /* The incremental reset behavior can't be */
   /* changed when rules are loaded.          */
   /*=========================================*/
   
   if (GetNextDefrule(NULL) != NULL)
     {
      PrintErrorID("INCRRSET",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"The incremental reset behavior cannot be changed with rules loaded.\n");
      SetEvaluationError(CLIPS_TRUE);
      return(oldValue);
     }

   /*==================================================*/
   /* The symbol FALSE disables incremental reset. Any */
   /* other value enables incremental reset.           */
   /*==================================================*/
   
   RtnUnknown(1,&argPtr);

   if ((argPtr.value == CLIPSFalseSymbol) && (argPtr.type == SYMBOL))
     { SetIncrementalReset(CLIPS_FALSE); }
   else
     { SetIncrementalReset(CLIPS_TRUE); }

   /*=======================*/
   /* Return the old value. */
   /*=======================*/
   
   return(oldValue);
  }

/****************************************************/
/* GetIncrementalResetCommand: CLIPS access routine */
/*   for the get-incremental-reset command.         */
/****************************************************/
globle int GetIncrementalResetCommand()
  {
   int oldValue;

   oldValue = GetIncrementalReset();

   if (ArgCountCheck("get-incremental-reset",EXACTLY,0) == -1)
     { return(oldValue); }

   return(oldValue);
  }
    
#endif
