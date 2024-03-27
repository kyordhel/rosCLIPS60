   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                  RULE BUILD MODULE                  */
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

#define _RULEBLD_SOURCE_

#include "setup.h"

#if DEFRULE_CONSTRUCT && (! RUN_TIME) && (! BLOAD_ONLY)

#include <stdio.h>
#define _CLIPS_STDIO_

#include "constant.h"
#include "clipsmem.h"
#include "router.h"
#include "watch.h"
#include "constrct.h"
#include "drive.h"
#include "pattern.h"
#include "reteutil.h"
#include "incrrset.h"
#include "rulebld.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static struct joinNode        *FindShareableJoin(struct joinNode *,VOID *,int,int,int,
                                                    struct expr *,
                                                    int,int,int,struct joinNode **);
   static int                     TestJoinForReuse(struct joinNode *,int,int,int,
                                                   struct expr *,
                                                   int,int,int,struct joinNode **);
   static struct joinNode        *CreateNewJoin(struct expr *,
                                                struct joinNode *,VOID *,int,int);
   static VOID                    AttachTestCEsToPatternCEs(struct lhsParseNode *);                                           
#else
   static struct joinNode        *FindShareableJoin();
   static int                     TestJoinForReuse();
   static struct joinNode        *CreateNewJoin();
   static VOID                    AttachTestCEsToPatternCEs(); 
#endif
  
/****************************************************************/
/* ConstructJoins: Integrates a set of pattern and join tests   */
/*   associated with a rule into the pattern and join networks. */
/****************************************************************/
globle struct joinNode *ConstructJoins(logicalJoin,theLHS)
  int logicalJoin;
  struct lhsParseNode *theLHS;
  {
   struct joinNode *lastJoin = NULL;
   struct patternNodeHeader *lastPattern;
   int firstJoin = CLIPS_TRUE;
   int tryToReuse = CLIPS_TRUE;
   struct joinNode *listOfJoins;
   struct joinNode *oldJoin;
   int joinNumber = 1;
   int isLogical;
   struct joinNode *nandReconnect[32];
   int currentDepth = 1;
   int lastIteration = CLIPS_FALSE;
   int rhsType;
   int endDepth;
   
   /*===================================================*/
   /* Remove any test CEs from the LHS and attach their */
   /* expression to the closest preceeding non-negated  */
   /* join at the same not/and depth.                   */
   /*===================================================*/
   
   AttachTestCEsToPatternCEs(theLHS);
     
   /*=====================================================*/
   /* Process each pattern CE in the rule. At this point, */
   /* there should be no and/or/not/test CEs in the LHS.  */
   /*=====================================================*/
   
   while (theLHS != NULL)
     {
      if (theLHS->bottom == NULL) lastIteration = CLIPS_TRUE;
      
      /*==================================================*/
      /* If the pattern is the start of a new not/and CE, */
      /* then remember the join to reconnect to after the */
      /* join from the right is completed.                */
      /*==================================================*/
      
      endDepth = theLHS->endNandDepth;
      while (theLHS->beginNandDepth > currentDepth)
        {
         nandReconnect[currentDepth-1] = lastJoin;
         currentDepth++;
        }
      
      /*============================================================*/
      /* Add the next pattern for this rule to the pattern network. */
      /*============================================================*/
        
      rhsType = theLHS->patternType->positionInArray;
      lastPattern = (*theLHS->patternType->addPatternFunction)(theLHS);
      
      /*======================================================*/
      /* Determine if the join being added is a logical join. */
      /*======================================================*/
      
      if (joinNumber == logicalJoin) isLogical = CLIPS_TRUE;
      else isLogical = CLIPS_FALSE;
      
      /*===============================================*/  
      /* Get the list of joins which could potentially */
      /* be reused in place of the join being added.   */
      /*===============================================*/  
      
      if (firstJoin == CLIPS_TRUE)
        { listOfJoins = lastPattern->entryJoin; }
      else
        { listOfJoins = lastJoin->nextLevel; }
        
      /*=======================================================*/
      /* Determine if the next join to be added can be shared. */
      /*=======================================================*/

      if ((tryToReuse == CLIPS_TRUE) &&
          ((oldJoin = FindShareableJoin(listOfJoins,(VOID *) lastPattern,firstJoin,
                                        (int) theLHS->negated,isLogical,
                                        theLHS->networkTest,
                                        endDepth,currentDepth,
                                        lastIteration,nandReconnect)) != NULL) )
        {
#if DEBUGGING_FUNCTIONS
         if ((GetWatchItem("compilations") == CLIPS_TRUE) && GetPrintWhileLoading())
           { PrintCLIPS(WDIALOG,"=j"); }
#endif
         lastJoin = oldJoin;
        }
      else
        {
         tryToReuse = CLIPS_FALSE;
         lastJoin = CreateNewJoin(theLHS->networkTest,
                                  lastJoin,lastPattern,
                                  CLIPS_FALSE,(int) theLHS->negated);
         lastJoin->rhsType = rhsType;
        }

      /*==========================================================*/
      /* Create any joins from the right needed to handle not/and */
      /* CE combinations and connect them to the join network.    */
      /*==========================================================*/
      
      while (endDepth < currentDepth)
        {
         currentDepth--;
           
         if (lastJoin->nextLevel == NULL) tryToReuse = CLIPS_FALSE;
     
         if (tryToReuse)
           { 
#if DEBUGGING_FUNCTIONS
            if ((GetWatchItem("compilations") == CLIPS_TRUE) && GetPrintWhileLoading())
              { PrintCLIPS(WDIALOG,"=j"); }
#endif
            lastJoin = lastJoin->nextLevel;
           }
         else
           {
            lastJoin = CreateNewJoin(NULL,nandReconnect[currentDepth-1],
                                     lastJoin,CLIPS_TRUE,CLIPS_FALSE);
           }
        }
        
      /*=======================================*/
      /* Move on to the next join to be added. */
      /*=======================================*/

      theLHS = theLHS->bottom;
      joinNumber++;
      firstJoin = CLIPS_FALSE;
     }

   /*===================================================*/
   /* If compilations are being watched, put a carriage */
   /* return after all of the =j's and +j's             */
   /*===================================================*/
   
#if DEBUGGING_FUNCTIONS
   if ((GetWatchItem("compilations") == CLIPS_TRUE) && GetPrintWhileLoading())
     { PrintCLIPS(WDIALOG,"\n"); }
#endif

   /*=============================*/
   /* Return the last join added. */
   /*=============================*/
   
   return(lastJoin);
  }
  
/****************************************************************/
/* AttachTestCEsToPatternCEs: Attaches the expressions found in */
/*   test CEs to the closest preceeding pattern CE that is not  */
/*   negated and is at the same not/and depth.                  */
/****************************************************************/
static VOID AttachTestCEsToPatternCEs(theLHS)
  struct lhsParseNode *theLHS;
  {
   struct lhsParseNode *lastNode, *trackNode, *tempNode;
   
   /*===============================================*/
   /* Look at each pattern on the rule's LHS to see */
   /* if any test CEs should be attached to it.     */
   /*===============================================*/
   
   while (theLHS != NULL)
     {
      /*==============================================*/
      /* If the pattern is negated, then don't bother */
      /* looking for any test CEs to attach to it.    */
      /*==============================================*/
      
      if (theLHS->negated)
        { trackNode = NULL; }
      else
        {
         lastNode = theLHS;
         trackNode = theLHS->bottom;
        }
        
      /*=================================================*/
      /* Check all of the patterns following the current */
      /* pattern to check for test CEs which can be      */
      /* attached to the current pattern.                */
      /*=================================================*/
      
      while (trackNode != NULL)
        {
         /*=======================================================*/
         /* Skip over any CEs that have a higher not/and depth or */
         /* are negated since any test CEs found within these CEs */
         /* would be attached to another pattern with the same    */
         /* depth, rather than the current pattern.               */
         /*=======================================================*/
         
         if ((trackNode->beginNandDepth != theLHS->beginNandDepth) ||
             (trackNode->negated))
           {
            lastNode = trackNode;
            trackNode = trackNode->bottom;
           }
           
         /*======================================================*/
         /* Once a non-negated pattern has been encounted at the */
         /* same not/and depth as the current pattern, then stop */
         /* because any test CEs following this pattern would be */
         /* attached to it rather than the current pattern.      */
         /*======================================================*/
         
         else if (trackNode->type == PATTERN_CE)
           { trackNode = NULL; }
           
         /*==================================================*/
         /* A test CE encountered at the same not/and depth  */
         /* can be added to the network test expressions for */ 
         /* the currentpattern.                              */
         /*==================================================*/
         
         else if (trackNode->type == TEST_CE)
           {
            theLHS->networkTest = CombineExpressions(theLHS->networkTest,
                                                     trackNode->networkTest);
            trackNode->networkTest = NULL;
            tempNode = trackNode->bottom;
            trackNode->bottom = NULL;
            lastNode->bottom = tempNode;
            lastNode->endNandDepth = trackNode->endNandDepth;
            ReturnLHSParseNodes(trackNode);
            trackNode = tempNode;
           }
           
         /*================================================*/
         /* If none of the previous conditions have been   */
         /* met, then there is an internal error in CLIPS. */ 
         /*================================================*/
            
         else
           { 
            CLIPSSystemError("BUILD",1);
            ExitCLIPS(4);
           }
        }
        
      /*====================================*/
      /* Check the next pattern in the LHS. */
      /*====================================*/
      
      theLHS = theLHS->bottom;
     }
  }
  
/********************************************************************/
/* FindShareableJoin: Determines whether a join exists that can be  */
/*   reused for the join currently being added to the join network. */
/*   Returns a pointer to the join to be shared if one if found,    */
/*   otherwise returns a NULL pointer.                              */
/********************************************************************/
static struct joinNode *FindShareableJoin(listOfJoins,rhsStruct,firstJoin,negatedRHS,
                                          isLogical,joinTest,
                                          endDepth,currentDepth,lastIteration,
                                          nandReconnect)
  struct joinNode *listOfJoins;
  VOID *rhsStruct;
  int firstJoin;
  int negatedRHS;
  int isLogical;
  struct expr *joinTest;
  int endDepth, currentDepth, lastIteration;
  struct joinNode **nandReconnect;
  {
   /*========================================*/
   /* Loop through all of the joins in the   */
   /* list of potential candiates for reuse. */
   /*========================================*/
   
   while (listOfJoins != NULL)
     {
      /*=========================================================*/
      /* If the join being tested for reuse is connected on the  */
      /* RHS to the end node of the pattern node associated with */
      /* the join to be added, then determine if the join can    */
      /* be reused. If so, return the join.                      */
      /*=========================================================*/
      
      if (listOfJoins->rightSideEntryStructure == rhsStruct)
        {
         if (TestJoinForReuse(listOfJoins,firstJoin,negatedRHS,isLogical,
                              joinTest,endDepth,currentDepth,
                              lastIteration,nandReconnect))
           { return(listOfJoins); }
        }
        
      /*====================================================*/
      /* Move on to the next potential candidate. Note that */
      /* the rightMatchNode link is used for traversing     */
      /* through the candidates for the first join of a     */
      /* rule and that rightDriveNode link is used for      */
      /* traversing through the candidates for subsequent   */
      /* joins of a rule.                                   */
      /*====================================================*/
           
      if (firstJoin)
        { listOfJoins = listOfJoins->rightMatchNode; }
      else
        { listOfJoins = listOfJoins->rightDriveNode; }
     }
     
   /*================================*/
   /* Return a NULL pointer, since a */
   /* reusable join was not found.   */
   /*================================*/
     
   return(NULL);
  }
  
/**************************************************************/
/* TestJoinForReuse: Determines if the specified join can be  */
/*   shared with a join being added for a rule being defined. */
/*   Returns TRUE if the join can be shared, otherwise FALSE. */
/**************************************************************/
static int TestJoinForReuse(testJoin,firstJoin,negatedRHS,isLogical,joinTest,
                            endDepth,currentDepth,lastIteration,nandReconnect)
  struct joinNode *testJoin;
  int firstJoin;
  int negatedRHS;
  int isLogical;
  struct expr *joinTest;
  int endDepth, currentDepth, lastIteration;
  struct joinNode **nandReconnect;
  {
   /*==================================================*/
   /* The first join of a rule may only be shared with */
   /* a join that has its firstJoin field set to TRUE. */
   /*==================================================*/
   
   if (testJoin->firstJoin != firstJoin) return(CLIPS_FALSE);
      
   /*========================================================*/
   /* A join connected to a not CE may only be shared with a */
   /* join that has its patternIsNegated field set to TRUE.  */
   /*========================================================*/
      
   if (testJoin->patternIsNegated != negatedRHS) return(CLIPS_FALSE);
          
   /*==========================================================*/
   /* If the join added is associated with a logical CE, then  */
   /* either the join to be shared must be associated with a   */
   /* logical CE or the beta memory must be empty (since       */
   /* joins associate an extra field with each partial match). */
   /*==========================================================*/
     
   if ((isLogical == CLIPS_TRUE) && 
       (testJoin->logicalJoin == CLIPS_FALSE) &&
       (testJoin->beta != NULL))
     { return(CLIPS_FALSE); }
      
   /*===============================================================*/
   /* The expression associated with the join must be identical to  */
   /* the networkTest expression stored with the join to be shared. */
   /*===============================================================*/
   
   if (IdenticalExpression(testJoin->networkTest,joinTest) != CLIPS_TRUE) 
     { return(CLIPS_FALSE); }
     
   /*==============================================================*/
   /* If the join being added enters another join from the right,  */
   /* then the series of "joins from the right" for the join being */
   /* added must match the series of "joins from the right" for    */
   /* the join being tested for reuse (i.e. the LHS connections    */
   /* from other joins must be identical for each of the joins in  */
   /* the series of "joins from the right."                        */
   /*==============================================================*/
                     
   for (; endDepth < currentDepth; currentDepth--)
     {
      testJoin = testJoin->nextLevel;
      if (testJoin == NULL) return(CLIPS_FALSE);
      
      if (testJoin->joinFromTheRight == CLIPS_FALSE) 
        { return(CLIPS_FALSE); }
      else if (nandReconnect[currentDepth-2] != testJoin->lastLevel) 
        { return(CLIPS_FALSE); }
     }
     
   /*=============================================================*/
   /* The last join of a rule cannot be shared with the last join */
   /* of another rule. A join cannot be used as the last join of  */
   /* a rule if it already has partial matches in its beta memory */
   /* (because of the extra slot used to point at activations).   */
   /*=============================================================*/
   
   if (lastIteration)
     {
      if (testJoin->ruleToActivate != NULL) return(CLIPS_FALSE);
      
      if (testJoin->beta != NULL) return(CLIPS_FALSE);
     }
   
   /*===========================================================================*/
   /* A join cannot be shared if it is not the last join for a rule and shares  */
   /* part, but not all, of a series of joins connected to other joins from the */
   /* right. This is because the data structure for joins can only point to     */
   /* either a single join that is entered from the right or a series of joins  */
   /* that are entered from the left, but not both. (The last join of a rule    */
   /* does not require any links to other joins so it can be shared).           */
   /*===========================================================================*/
   
   if ((! lastIteration) && (testJoin->nextLevel != NULL))
     {
      if (testJoin->nextLevel->joinFromTheRight == CLIPS_TRUE)
        {
         if (((struct joinNode *) testJoin->nextLevel->rightSideEntryStructure) == testJoin)
           { return(CLIPS_FALSE); }
        }
     }
   
   /*=============================================*/
   /* The join can be shared since all conditions */
   /* for sharing have been satisfied.            */
   /*=============================================*/
   
   return(CLIPS_TRUE);
  }
              
/*************************************************************************/
/* CreateNewJoin: Creates a new join and links it into the join network. */
/*************************************************************************/
static struct joinNode *CreateNewJoin(joinTest,lhsEntryStruct,rhsEntryStruct,
                                      joinFromTheRight,negatedRHSPattern)
  struct expr *joinTest;
  struct joinNode *lhsEntryStruct;
  VOID *rhsEntryStruct;
  int joinFromTheRight;
  int negatedRHSPattern;
  {
   struct joinNode *newJoin;
   
   /*===============================================*/
   /* If compilations are being watch, print +j to  */
   /* indicate that a new join has been created for */
   /* this pattern of the rule (i.e. a join could   */
   /* not be shared with another rule.              */
   /*===============================================*/
   
#if DEBUGGING_FUNCTIONS
   if ((GetWatchItem("compilations") == CLIPS_TRUE) && GetPrintWhileLoading())
     { PrintCLIPS(WDIALOG,"+j"); }
#endif

   /*========================================================*/
   /* Create the new join and initialize some of its values. */
   /*========================================================*/
   
   newJoin = get_struct(joinNode);
   newJoin->beta = NULL;
   newJoin->nextLevel = NULL;
   newJoin->joinFromTheRight = joinFromTheRight;
   newJoin->patternIsNegated = negatedRHSPattern;
#if INCREMENTAL_RESET
   newJoin->initialize = GetIncrementalReset();
#else
   newJoin->initialize = CLIPS_FALSE;
#endif
   newJoin->logicalJoin = CLIPS_FALSE;
   newJoin->ruleToActivate = NULL;
   
   /*==============================================*/
   /* Install the expressions used to determine    */
   /* if a partial match satisfies the constraints */
   /* associated with this join.                   */
   /*==============================================*/
   
   newJoin->networkTest = AddHashedExpression(joinTest);
   
   /*============================================================*/
   /* Initialize the values associated with the LHS of the join. */
   /*============================================================*/
      
   newJoin->lastLevel = lhsEntryStruct;
                
   if (lhsEntryStruct == NULL)
     {
      newJoin->firstJoin = CLIPS_TRUE;
      newJoin->depth = 1;
      newJoin->rightDriveNode = NULL;
     }
   else
     {
      newJoin->firstJoin = CLIPS_FALSE;
      newJoin->depth = lhsEntryStruct->depth + 1;
      newJoin->rightDriveNode = lhsEntryStruct->nextLevel;
      lhsEntryStruct->nextLevel = newJoin;
     }
   
   /*=======================================================*/
   /* Initialize the pointer values associated with the RHS */
   /* of the join (both for the new join and the join or    */
   /* pattern which enters this join from the right.        */
   /*=======================================================*/

   newJoin->rightSideEntryStructure = rhsEntryStruct;
   
   if (joinFromTheRight)
     { 
      newJoin->rightMatchNode = NULL; 
      ((struct joinNode *) rhsEntryStruct)->nextLevel = newJoin;
     }
   else
     {
      newJoin->rightMatchNode = ((struct patternNodeHeader *) rhsEntryStruct)->entryJoin;
      ((struct patternNodeHeader *) rhsEntryStruct)->entryJoin = newJoin;
     }
   
   /*================================*/
   /* Return the newly created join. */
   /*================================*/
   
   return(newJoin);
  }

#endif



