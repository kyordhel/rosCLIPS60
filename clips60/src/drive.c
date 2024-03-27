   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                    DRIVE MODULE                     */
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

#define _DRIVE_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_

#include "setup.h"

#if DEFRULE_CONSTRUCT

#include "constant.h"
#include "clipsmem.h"
#include "reteutil.h"
#include "prntutil.h"
#include "router.h"
#include "agenda.h"
#include "retract.h"

#if LOGICAL_DEPENDENCIES
#include "lgcldpnd.h"
#endif

#if INCREMENTAL_RESET
#include "incrrset.h"
#endif

#include "drive.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                    PPDrive(struct partialMatch *,struct partialMatch *,struct joinNode *);
   static VOID                    PNRDrive(struct joinNode *,struct partialMatch *,
                                           struct partialMatch *);
   static VOID                    EmptyDrive(struct joinNode *,struct partialMatch *);
   static VOID                    JoinNetErrorMessage(struct joinNode *);
#else
   static VOID                    PPDrive();
   static VOID                    PNRDrive();
   static VOID                    EmptyDrive();
   static VOID                    JoinNetErrorMessage();
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle BOOLEAN              JoinOperationInProgress = CLIPS_FALSE;
   
/********************************************************************************/
/* Drive: Primary routine for driving a partial match through the join network. */
/********************************************************************************/
globle VOID Drive(binds,join,enterDirection)
  struct partialMatch *binds;
  struct joinNode *join;
  int enterDirection;
  {
   struct partialMatch *lhsBinds = NULL, *rhsBinds = NULL;
   struct partialMatch *comparePMs = NULL, *newBinds;
   int exprResult;

   /*=========================================================*/
   /* If an incremental reset is being performed and the join */
   /* is not part of the network to be reset, then return.    */
   /*=========================================================*/

#if INCREMENTAL_RESET && (! BLOAD_ONLY) && (! RUN_TIME)
   if (IncrementalResetInProgress && (join->initialize == CLIPS_FALSE)) return;
#endif

   /*=========================================================*/
   /* If the associated LHS pattern is a not CE or the join   */
   /* is a nand join, then we need an additional field in the */
   /* partial match to keep track of the pseudo fact if one   */
   /* is created. The partial match is automatically stored   */
   /* in the beta memory and the counterf slot is used to     */
   /* determine if it is an actual partial match.             */
   /*=========================================================*/

   if ((enterDirection == LHS) && 
       ((join->patternIsNegated) || (join->joinFromTheRight)))
     {
      newBinds = AddSingleMatch(binds,NULL,
                                (join->ruleToActivate == NULL) ? 0 : 1,
                                (int) join->logicalJoin);
      newBinds->notOriginf = CLIPS_TRUE;
      newBinds->counterf = CLIPS_TRUE;
      binds = newBinds;
      binds->next = join->beta;
      join->beta = binds;
     }

   /*==================================================*/
   /* Use a special routine if this is the first join. */
   /*==================================================*/

   if (join->firstJoin)
     {
      EmptyDrive(join,binds);
      return;
     }

   /*============================*/
   /* Initialize some variables. */
   /*============================*/

   if (enterDirection == LHS)
     {
      if (join->joinFromTheRight)
        { comparePMs = ((struct joinNode *) join->rightSideEntryStructure)->beta;}
      else
        { comparePMs = ((struct patternNodeHeader *) join->rightSideEntryStructure)->alphaMemory; }
      lhsBinds = binds;
     }
   else if (enterDirection == RHS)
     {
      if (join->patternIsNegated || join->joinFromTheRight) 
        { comparePMs = join->beta; }
      else 
        { comparePMs = join->lastLevel->beta; }
      rhsBinds = binds;
     }
   else
     {
      CLIPSSystemError("DRIVE",1);
      ExitCLIPS(5);
     }

   /*===================================================*/
   /* Compare each set of binds on the opposite side of */
   /* the join with the set of binds that entered this  */
   /* join.  If the binds don't mismatch, then perform  */
   /* the appropriate action for the logic of the join. */
   /*===================================================*/

   while (comparePMs != NULL)
     {
      exprResult = CLIPS_FALSE;

      /* What does this do???? */
      if (enterDirection == RHS)
        { 
         lhsBinds = comparePMs; 
         if (lhsBinds->counterf && 
             (join->patternIsNegated == CLIPS_FALSE) &&
             (join->joinFromTheRight == CLIPS_FALSE)) 
           {
            comparePMs = comparePMs->next;
            continue;
           }   
       /* end of what does this do??? */
           
         /* If we just entered from the right and the LHS */
         /* match we're looking at is already blocked by some */
         /* other partial match in the alpha memory, then  */
         /* just go ahead and look for other partial matches */
         /* that can be blocked. */
         
        if ((join->patternIsNegated || join->joinFromTheRight) &&
            (enterDirection == LHS) &&
            (lhsBinds->binds[binds->bcount - 1].gm.theValue != NULL))
          { 
           comparePMs = comparePMs->next;
           continue;
          }
        }
      else
        { rhsBinds = comparePMs; }

      /*======================================================*/
      /* Evaluate the join expression for this combination of */
      /* rhs and lhs bindings.                                */
      /*======================================================*/

      if (join->networkTest == NULL)
        { 
         exprResult = CLIPS_TRUE;
         if (join->joinFromTheRight)
           {
            int i;
            
            for (i = 0; i < (lhsBinds->bcount - 1); i++)
              {
               if (lhsBinds->binds[i].gm.theMatch != rhsBinds->binds[i].gm.theMatch)
                 {
                  exprResult = CLIPS_FALSE;
                  break;
                 }
              }
           } 
        }
      else
        {
         exprResult = EvaluateJoinExpression(join->networkTest,lhsBinds,rhsBinds,join);
         if (EvaluationError)
           {
            if (join->patternIsNegated) exprResult = CLIPS_TRUE;
            SetEvaluationError(CLIPS_FALSE);
           }
        }     

      /*====================================================*/
      /* If the join expression evaluated to true (i.e.     */
      /* there were no conflicts between variable bindings, */
      /* all tests were satisfied, etc.), then perform the  */
      /* appropriate action given the logic of this join.   */
      /*====================================================*/

      if (exprResult != CLIPS_FALSE)
        {
         if ((join->patternIsNegated == CLIPS_FALSE) &&
             (join->joinFromTheRight == CLIPS_FALSE))
           { PPDrive(lhsBinds,rhsBinds,join); }
         else if (enterDirection == RHS)
           { PNRDrive(join,comparePMs,rhsBinds); }
         else if (enterDirection == LHS)
           { 
            binds->binds[binds->bcount - 1].gm.theValue = (VOID *) rhsBinds;
            comparePMs = NULL;
            continue;
           }
        }
        
      comparePMs = comparePMs->next;
     }

   /*======================================================*/
   /* If a join with a positive lhs and a negative rhs was */
   /* entered from the lhs side of the join, and the join  */
   /* test failed for all sets of matches for the new      */
   /* bindings on the lhs side (the counter on the lhs is  */
   /* set to zero), then the lhs bindings should be send   */
   /* down to the joins below along with an NID marker     */
   /* that represents the instance of the not pattern that */
   /* was satisfied.                                       */
   /*======================================================*/

   if ((join->patternIsNegated || join->joinFromTheRight) &&
       (enterDirection == LHS) &&
       (binds->binds[binds->bcount - 1].gm.theValue == NULL))
     { PNLDrive(join,binds); }

   return;
  }

/****************************************************************/
/* EvaluateJoinExpression: Evaluates join expressions. Performs */
/*   a faster evaluation for join expressions than if           */
/*   EvaluateExpression were used directly. Function calls to   */
/*   (eq_vars), (neq_vars), /and/, and /or/ are evaluated       */
/*   directly. All other function calls are evaluated using     */
/*   EvaluateExpression.                                        */
/****************************************************************/
globle BOOLEAN EvaluateJoinExpression(joinExpr,lbinds,rbinds,joinPtr)
  struct expr *joinExpr;
  struct partialMatch *lbinds, *rbinds;
  struct joinNode *joinPtr;
  {
   DATA_OBJECT theResult;
   int andLogic, result = CLIPS_TRUE;
   struct partialMatch *oldLHSBinds;
   struct partialMatch *oldRHSBinds;
   struct joinNode *oldJoin;
   
   if (joinExpr == NULL) return(CLIPS_TRUE);

   oldLHSBinds = GlobalLHSBinds;
   oldRHSBinds = GlobalRHSBinds;
   oldJoin = GlobalJoin;
   GlobalLHSBinds = lbinds;
   GlobalRHSBinds = rbinds;
   GlobalJoin = joinPtr;
   
   /* Don't use the extra slot in the partial match added for */
   /* not CEs when doing evaluations in the join network.     */
   
   if (joinPtr->patternIsNegated) lbinds->bcount--;
   
   if (joinExpr->value == PTR_AND)
     {
      andLogic = CLIPS_TRUE;
      joinExpr = joinExpr->argList;
     }
   else if (joinExpr->value == PTR_OR)
     {
      andLogic = CLIPS_FALSE;
      joinExpr = joinExpr->argList;
     }
   else
     { andLogic = CLIPS_TRUE; }

   while (joinExpr != NULL)
     {        
     
      if ((PrimitivesArray[joinExpr->type] == NULL) ? 
          CLIPS_FALSE :
          PrimitivesArray[joinExpr->type]->evaluateFunction != NULL)
        {
         struct expr *oldArgument;
         
         oldArgument = CurrentExpression;
         CurrentExpression = joinExpr;
         result = (*PrimitivesArray[joinExpr->type]->evaluateFunction)(joinExpr->value,&theResult);
         CurrentExpression = oldArgument;
        }

      else if (joinExpr->value == PTR_OR)
        {
         result = CLIPS_FALSE;
         if (EvaluateJoinExpression(joinExpr,lbinds,rbinds,joinPtr) == CLIPS_TRUE)
           {
            if (EvaluationError) 
              {
               if (joinPtr->patternIsNegated) lbinds->bcount++;
         GlobalLHSBinds = oldLHSBinds;
         GlobalRHSBinds = oldRHSBinds;
         GlobalJoin = oldJoin;
               return(CLIPS_FALSE);
              }
            result = CLIPS_TRUE;
           }
         else if (EvaluationError)
           { 
            if (joinPtr->patternIsNegated) lbinds->bcount++;
         GlobalLHSBinds = oldLHSBinds;
         GlobalRHSBinds = oldRHSBinds;
         GlobalJoin = oldJoin;
            return(CLIPS_FALSE); 
           }
        }

      /*==========================================================*/
      /* Evaluate and expressions expressed in the format:        */
      /*   (and <expression 1> <expression 2> ... <expression n>) */
      /* Returns CLIPS_FALSE (0) if any of the expression are     */
      /* CLIPS_FALSE, otherwise returns true (1).                 */
      /*==========================================================*/

      else if (joinExpr->value == PTR_AND)
        {
         result = CLIPS_TRUE;
         if (EvaluateJoinExpression(joinExpr,lbinds,rbinds,joinPtr) == CLIPS_FALSE)
           {
            if (EvaluationError)
              {
               if (joinPtr->patternIsNegated) lbinds->bcount++;
         GlobalLHSBinds = oldLHSBinds;
         GlobalRHSBinds = oldRHSBinds;
         GlobalJoin = oldJoin;
               return(CLIPS_FALSE);
              }
            result = CLIPS_FALSE;
           }
         else if (EvaluationError)
           { 
            if (joinPtr->patternIsNegated) lbinds->bcount++;
         GlobalLHSBinds = oldLHSBinds;
         GlobalRHSBinds = oldRHSBinds;
         GlobalJoin = oldJoin;
            return(CLIPS_FALSE);
           }
        }

      /*==========================================================*/
      /* Evaluate all other expressions using EvaluateExpression. */
      /*==========================================================*/

      else
        {

         EvaluateExpression(joinExpr,&theResult);


         if (EvaluationError)
           {
            JoinNetErrorMessage(joinPtr);
            if (joinPtr->patternIsNegated) lbinds->bcount++;
         GlobalLHSBinds = oldLHSBinds;
         GlobalRHSBinds = oldRHSBinds;
         GlobalJoin = oldJoin;
            return(CLIPS_FALSE);
           }

         if ((theResult.value == CLIPSFalseSymbol) && (theResult.type == SYMBOL))
           { result = CLIPS_FALSE; }
         else
           { result = CLIPS_TRUE; }
        }

      if ((andLogic == CLIPS_TRUE) && (result == CLIPS_FALSE)) 
        {
         if (joinPtr->patternIsNegated) lbinds->bcount++;
         GlobalLHSBinds = oldLHSBinds;
         GlobalRHSBinds = oldRHSBinds;
         GlobalJoin = oldJoin;
         return(CLIPS_FALSE);
        }
      else if ((andLogic == CLIPS_FALSE) && (result == CLIPS_TRUE)) 
        {
         if (joinPtr->patternIsNegated) lbinds->bcount++;
         GlobalLHSBinds = oldLHSBinds;
         GlobalRHSBinds = oldRHSBinds;
         GlobalJoin = oldJoin;
         return(CLIPS_TRUE);
        }

      joinExpr = joinExpr->nextArg;
     }
     
         GlobalLHSBinds = oldLHSBinds;
         GlobalRHSBinds = oldRHSBinds;
         GlobalJoin = oldJoin;
   if (joinPtr->patternIsNegated) lbinds->bcount++;
   return(result);
  }

/*******************************************************************/
/* PPDrive: Handles the merging of an alpha memory partial match   */
/*   with a beta memory partial match for a join that has positive */
/*   LHS entry and positive RHS entry. The partial matches being   */
/*   merged have previously been checked to determine that they    */
/*   satisify the constraints for the join. Once merged, the new   */
/*   partial match is sent to each child join of the join from     */
/*   which the merge took place.                                   */
/*******************************************************************/
static VOID PPDrive(lhsBinds,rhsBinds,join)
  struct partialMatch *lhsBinds, *rhsBinds;
  struct joinNode *join;
  {
   struct partialMatch *linker;
   struct joinNode *listOfJoins;

   /*==================================================*/
   /* Merge the alpha and beta memory partial matches. */
   /*==================================================*/

   linker = MergePartialMatches(lhsBinds,rhsBinds,
                                (join->ruleToActivate == NULL) ? 0 : 1,
                                (int) join->logicalJoin);

   /*=======================================================*/
   /* Add the partial match to the beta memory of the join. */
   /*=======================================================*/

   linker->next = join->beta;
   join->beta = linker;

   /*====================================================*/
   /* Activate the rule satisfied by this partial match. */
   /*====================================================*/

   if (join->ruleToActivate != NULL) AddActivation(join->ruleToActivate,linker);

   /*================================================*/
   /* Send the new partial match to all child joins. */
   /*================================================*/

   listOfJoins = join->nextLevel;
   if (listOfJoins != NULL)
     {
      if (((struct joinNode *) (listOfJoins->rightSideEntryStructure)) == join)
        { Drive(linker,listOfJoins,RHS); }
      else while (listOfJoins != NULL)
        {  
         Drive(linker,listOfJoins,LHS); 
         listOfJoins = listOfJoins->rightDriveNode;
        }
     }

   return;
  }

/**********************************************************************/
/* PNRDrive: Handles the entry of a partial match from the RHS of a   */
/*   join that has positive LHS entry and negative RHS entry (meaning */
/*   the conditional element associated with this join is a not       */
/*   conditional element). Entry of the alpha memory partial match    */
/*   will cause the count value of the associated beta memory partial */
/*   match to be incremented. This in turn may cause partial matches  */
/*   associated with the beta memory partial match to be removed from */
/*   the network.                                                     */
/**********************************************************************/
static VOID PNRDrive(join,lhsBinds,rhsBinds)
  struct joinNode *join;
  struct partialMatch *lhsBinds, *rhsBinds;
  {
   struct joinNode *listOfJoins;

   /*=================================================================*/
   /* The partial matches contained in the beta memory of a positive  */
   /* negative join have a "counter" attached to them which indicates */
   /* the number alpha memory partial matches which are consistent    */
   /* with them. If no alpha memory partial matches are consistent,   */
   /* then the not CE has been satisfied, the counter will have a     */
   /* value of < 0 (which indicates the id associated with the not CE */
   /* and that there are no consistent bindings), and the LHS         */
   /* bindings will have been sent down to the next join.  This       */
   /* routine handles the case in which a partial match entered the   */
   /* negative RHS entry of the join and were consistent with the     */
   /* LHS partial matches. If the counter for the LHS partial match   */
   /* is less than zero, then all partial matches with that "not" id  */
   /* value must be removed from descendent joins. If the counter for */
   /* the partial match is greater than zero, then the counter only   */
   /* needs to be incremented.                                        */
   /*=================================================================*/

   if (lhsBinds->counterf == CLIPS_FALSE)
     {
      lhsBinds->counterf = CLIPS_TRUE;
      
      /*===================================================================*/
      /* If the partial match caused an activation, remove the activation. */
      /*===================================================================*/
      
      if ((lhsBinds->activationf) ?
          (lhsBinds->binds[lhsBinds->bcount].gm.theValue != NULL) : FALSE)
        { RemoveActivation((struct activation *) lhsBinds->binds[lhsBinds->bcount].gm.theValue,CLIPS_TRUE,CLIPS_TRUE); }

      /*===========================================================*/
      /* The counter flag was FALSE. This means that a pointer to  */
      /* the pseudo-fact matching the not CE is stored directly in */
      /* the partial match. Determine the ID of this pseudo-fact   */
      /* and remove all partial matches from descendent joins that */
      /* contain the ID.                                           */
      /*===========================================================*/
      
      listOfJoins = join->nextLevel;   
      if (listOfJoins != NULL)
        {
         if (((struct joinNode *) (listOfJoins->rightSideEntryStructure)) == join)
           { NegEntryRetract(listOfJoins,lhsBinds,CLIPS_FALSE); }
         else while (listOfJoins != NULL)
           {  
            
            PosEntryRetract(listOfJoins,
                            lhsBinds->binds[lhsBinds->bcount - 1].gm.theMatch,
                            lhsBinds,(int) join->depth-1,CLIPS_FALSE);
            listOfJoins = listOfJoins->rightDriveNode;
           }
        }
        
      /*=========================================================================*/
      /* Remove any logical dependency links associated with this partial match. */
      /*=========================================================================*/

#if LOGICAL_DEPENDENCIES     
      if (lhsBinds->dependentsf) AddToDependencyList(lhsBinds);
#endif
      
      /*========================================*/
      /* Put the pseudo-fact on a garbage list. */
      /*========================================*/

      lhsBinds->binds[lhsBinds->bcount - 1].gm.theMatch->next = GarbageAlphaMatches;
      GarbageAlphaMatches = lhsBinds->binds[lhsBinds->bcount - 1].gm.theMatch;
      
      /*===========================================================*/
      /* Set the partial match's count value to 1. Set the counter */
      /* flag to TRUE to indicate that the counter is active.      */
      /*===========================================================*/

      lhsBinds->binds[lhsBinds->bcount - 1].gm.theValue = (VOID *) rhsBinds;
     }
  }

/********************************************************************/
/* PNLDrive: Handles the entry of a partial match from the LHS of a */
/*   join that has positive LHS entry and negative RHS entry        */
/*   (meaning the conditional element associated with this join is  */
/*   a not conditional element). An new partial match is created by */
/*   combining the match from the beta memory with a "pseudo"       */
/*   partial match corresponding to the facts which didn't match    */
/*   the not CE. Once merged, the new partial match is sent to each */
/*   child join of the join from which the merge took place.        */
/********************************************************************/
globle VOID PNLDrive(join,binds)
  struct joinNode *join;
  struct partialMatch *binds;
  {
   struct joinNode *listOfJoins;
   struct alphaMatch *tempAlpha;

    /*=======================================================*/
    /* Create a pseudo-fact representing the facts which did */
    /* not match the not CE associated with this join.       */
    /*=======================================================*/

    tempAlpha = get_struct(alphaMatch);
    tempAlpha->next = NULL;
    tempAlpha->matchingItem = NULL;
    tempAlpha->markers = NULL;

    /*===============================================*/
    /* Store the pointer to the pseudo-fact directly */
    /* in the beta memory partial match.             */
    /*===============================================*/

    binds->counterf = CLIPS_FALSE;
    binds->binds[binds->bcount - 1].gm.theMatch = tempAlpha;

   /*====================================================*/
   /* Activate the rule satisfied by this partial match. */
   /*====================================================*/

   if (join->ruleToActivate != NULL) AddActivation(join->ruleToActivate,binds);

    /*========================================================*/
    /* Send the merged partial match to all descendent joins. */
    /*========================================================*/

    listOfJoins = join->nextLevel;
    if (listOfJoins != NULL)
      {
       if (((struct joinNode *) (listOfJoins->rightSideEntryStructure)) == join)
         { Drive(binds,listOfJoins,RHS); }
       else while (listOfJoins != NULL)
         {  
          Drive(binds,listOfJoins,LHS); 
          listOfJoins = listOfJoins->rightDriveNode;
         }
      }
   }

/***************************************************************/
/* EmptyDrive: Handles the entry of a alpha memory partial     */
/*   match from the RHS of a join that is the first join of    */
/*   a rule (i.e. a join that cannot be entered from the LHS). */
/***************************************************************/
static VOID EmptyDrive(join,rhsBinds)
  struct joinNode *join;
  struct partialMatch *rhsBinds;
  {
   struct partialMatch *linker;
   struct joinNode *listOfJoins;
   int joinExpr;

   /*======================================================*/
   /* Determine if the alpha memory partial match satifies */
   /* the primary join expression. If it doesn't then no   */
   /* further action is taken.                             */
   /*======================================================*/

   if (join->networkTest != NULL)
     {
      joinExpr = EvaluateJoinExpression(join->networkTest,NULL,rhsBinds,join);
      EvaluationError = CLIPS_FALSE;
      if (joinExpr == CLIPS_FALSE) return;
     }

   /*===========================================================*/
   /* The first join of a rule cannot be connected to a NOT CE. */
   /*===========================================================*/

   if (join->patternIsNegated == CLIPS_TRUE)
     {
      CLIPSSystemError("DRIVE",2);
      ExitCLIPS(5);
     }

   /*=========================================================*/
   /* If the join's RHS entry is associated with a pattern CE */
   /* (positive entry), then copy the alpha memory partial    */
   /* match and send it to all child joins.                   */
   /*=========================================================*/

   linker = CopyPartialMatch(rhsBinds,
                             (join->ruleToActivate == NULL) ? 0 : 1,
                             (int) join->logicalJoin);

   /*=======================================================*/
   /* Add the partial match to the beta memory of the join. */
   /*=======================================================*/

   linker->next = join->beta;
   join->beta = linker;

   /*====================================================*/
   /* Activate the rule satisfied by this partial match. */
   /*====================================================*/

   if (join->ruleToActivate != NULL) AddActivation(join->ruleToActivate,linker);

   /*============================================*/
   /* Send the partial match to all child joins. */
   /*============================================*/

   listOfJoins = join->nextLevel;
   while (listOfJoins != NULL)
     {
      Drive(linker,listOfJoins,LHS);
      listOfJoins = listOfJoins->rightDriveNode;
     }
  }

/********************************************************************/
/* JoinNetErrorMessage: Prints an informational message indicating  */
/*   which join of a rule generated an error when a join expression */
/*   was being evaluated.                                           */
/********************************************************************/
static VOID JoinNetErrorMessage(joinPtr)
  struct joinNode *joinPtr;
  {
   char buffer[60];

   PrintErrorID("DRIVE",1,CLIPS_TRUE);
   PrintCLIPS(WERROR,"This error occurred in the join network\n");

   sprintf(buffer,"   Problem resides in join #%d in rule(s):\n",joinPtr->depth);
   PrintCLIPS(WERROR,buffer);

   TraceErrorToRule(joinPtr,"      ");
   PrintCLIPS(WERROR,"\n");
  }

#endif
