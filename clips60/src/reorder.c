   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                    REORDER MODULE                   */
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

#define _REORDER_SOURCE_

#include "setup.h"

#if (! RUN_TIME) && (! BLOAD_ONLY) && DEFRULE_CONSTRUCT

#include <stdio.h>
#define _CLIPS_STDIO_

#include "clipsmem.h"
#include "cstrnutl.h"
#include "pattern.h"
#include "extnfunc.h"
#include "rulelhs.h"
#include "prntutil.h"
#include "router.h"

#include "reorder.h"

#if ANSI_COMPILER
   static struct lhsParseNode    *ReverseAndOr(struct lhsParseNode *,struct lhsParseNode *,int);
   static struct lhsParseNode    *PerformReorder(struct lhsParseNode *,int *);
   static VOID                    IncrementNandDepth(struct lhsParseNode *,int);
   static struct lhsParseNode    *CreateInitialPattern(struct patternParser *);
   static struct lhsParseNode    *ReorderDriver(struct lhsParseNode *,int *);
   static VOID                    HandleOrphanedTestCEs(struct lhsParseNode *);
   static VOID                    PrintNodes(char *,struct lhsParseNode *);
   static struct lhsParseNode    *AssignPatternIndices(struct lhsParseNode *,int);
   static VOID                    PropagateIndexSlotPatternValues(struct lhsParseNode *,
                                                                  int,int,
                                                                  struct symbolHashNode *,
                                                                  int);
#else
   static struct lhsParseNode    *ReverseAndOr();
   static struct lhsParseNode    *PerformReorder();
   static VOID                    IncrementNandDepth();
   static struct lhsParseNode    *CreateInitialPattern();
   static struct lhsParseNode    *ReorderDriver();
   static VOID                    HandleOrphanedTestCEs();
   static VOID                    PrintNodes();
   static struct lhsParseNode    *AssignPatternIndices();
   static VOID                    PropagateIndexSlotPatternValues();
#endif

/********************************************/
/* ReorderPatterns: Reorders a group of CEs */
/*   to accommodate CLIPS Rete topology.    */
/********************************************/
globle struct lhsParseNode *ReorderPatterns(theLHS,anyChange)
  struct lhsParseNode *theLHS;
  int *anyChange;
  {    
   struct lhsParseNode *newLHS, *patternPtr;
   unsigned int whichCE;
      
   newLHS = GetLHSParseNode();
   newLHS->type = AND_CE;
      
   if (theLHS == NULL) newLHS->right = CreateInitialPattern(NULL);
   else newLHS->right = theLHS;
   
   newLHS = ReorderDriver(newLHS,anyChange);
   
   AddInitialPatterns(newLHS);
   
   /*===========================================================*/
   /* Number the user specified patterns. Patterns added while  */
   /* analyzing the rule (such as placing initial-fact patterns */
   /* before not CEs) are not numbered so that there is no      */
   /* confusion when an error message refers to a CE. Also      */
   /* propagate field and slot values throughout each pattern.  */
   /*===========================================================*/
   
   if (newLHS->type == OR_CE) theLHS = newLHS->right;
   else theLHS = newLHS;
   
   while (theLHS != NULL)
     {
      whichCE = 1;
      patternPtr = theLHS->right;
      while (patternPtr != NULL)
        {
         if (patternPtr->userCE) patternPtr->whichCE = whichCE++;
         patternPtr = patternPtr->bottom;
        }
        
      AssignPatternIndices(theLHS->right,1);
      theLHS = theLHS->bottom;
     }
    
   /*===========================*/
   /* Return the processed LHS. */
   /*===========================*/
   
   return(newLHS);
  }
  
/********************************************/
/* ReorderDriver: Reorders a group of CEs */
/*   to accommodate CLIPS Rete topology.    */
/********************************************/
static struct lhsParseNode *ReorderDriver(theLHS,anyChange)
  struct lhsParseNode *theLHS;
  int *anyChange;
  {
   struct lhsParseNode *argPtr;
   struct lhsParseNode *before, *save;
   int change, newChange;
   *anyChange = CLIPS_FALSE;
   
   change = CLIPS_TRUE;
   while (change)
     {
      /*=======================================*/
      /* Reorder the current level of the LHS. */
      /*=======================================*/
      
      change = CLIPS_FALSE;
      
      if ((theLHS->type == AND_CE) ||
          (theLHS->type == NOT_CE) ||
          (theLHS->type == OR_CE))
        {   
         theLHS = PerformReorder(theLHS,&newChange);
         
         if (newChange) 
           {
            *anyChange = CLIPS_TRUE;
            change = CLIPS_TRUE;
           }
        }
              
      /*=====================================================*/
      /* Recursively reorder CEs at lower levels in the LHS. */
      /*=====================================================*/

      before = NULL;
      argPtr = theLHS->right;

      while (argPtr != NULL)
        {
         save = argPtr->bottom;
         if ((argPtr->type == AND_CE) ||
             (argPtr->type == NOT_CE) ||
             (argPtr->type == OR_CE))
           {
            if (before == NULL)
              {
               argPtr->bottom = NULL;
               theLHS->right = ReorderDriver(argPtr,&newChange);
               theLHS->right->bottom = save;
               before = theLHS->right;
              }
            else
              {
               argPtr->bottom = NULL;
               before->bottom = ReorderDriver(argPtr,&newChange);
               before->bottom->bottom = save;
               before = before->bottom;
              }
             
            if (newChange) 
              {
               *anyChange = CLIPS_TRUE;
               change = CLIPS_TRUE;
              }
           }
         else
           {
            before = argPtr;
           }
           
         argPtr = save;
        }
     }
     
   /*===========================*/
   /* Return the reordered LHS. */
   /*===========================*/
      
   return(theLHS);
  }
  
/********************************************/
/* AddInitialPatterns: */
/********************************************/
globle VOID AddInitialPatterns(theLHS)
  struct lhsParseNode *theLHS;
  {
   struct lhsParseNode *thePattern, *lastPattern;
   struct patternParser *lastType;
   
   /*====================================================*/
   /* If there are multiple disjuncts for the rule, then */
   /* add initial patterns to each disjunct separately.  */
   /*====================================================*/
   
   if (theLHS->type == OR_CE)
     {
      for (thePattern = theLHS->right; 
           thePattern != NULL; 
           thePattern = thePattern->bottom)
        {
         AddInitialPatterns(thePattern);
        }
      
      return;
     }
     
   /*======================================================*/
   /* Determine what the default pattern type for the rule */
   /* should be (in case the rule begins with a test CE.   */
   /*======================================================*/
   
   lastType = NULL;
   thePattern = theLHS->right;
   while ((thePattern != NULL) && (lastType == NULL))
     {
      if (thePattern->type == PATTERN_CE)
        { lastType = thePattern->patternType; }
      thePattern = thePattern->bottom;
     }
        
   /*=======================================*/
   /* Handle the first pattern in the rule. */
   /*=======================================*/
   
   if ((theLHS->right->negated) ||
       (theLHS->right->type == TEST_CE) ||
       (theLHS->right->beginNandDepth > 1))
     {
      thePattern = CreateInitialPattern(lastType);
        
      thePattern->logical = theLHS->logical;
      thePattern->bottom = theLHS->right;
      theLHS->right = thePattern;
     }
     
   /*================================*/
   /* Handle the remaining patterns. */
   /*================================*/
   
   lastPattern = theLHS->right;
   thePattern = theLHS->right->bottom;
   
   while (thePattern != NULL)
     {
      if ((thePattern->beginNandDepth > lastPattern->endNandDepth) &&
          ((thePattern->type == TEST_CE) || (thePattern->negated)))
        {
         if (thePattern->type == PATTERN_CE) 
           { lastType = thePattern->patternType; }
           
         lastPattern->bottom = CreateInitialPattern(lastType);
        
         lastPattern->bottom->logical = thePattern->logical;
         lastPattern->bottom->bottom = thePattern;
         lastPattern->bottom->beginNandDepth = thePattern->beginNandDepth;
         lastPattern->bottom->endNandDepth = thePattern->beginNandDepth;
        }
        
      lastPattern = thePattern;
      thePattern = thePattern->bottom;
     }
     
   HandleOrphanedTestCEs(theLHS->right);
  }
  
/*******************************************/
/* PerformReorder: Reorders a group of CEs */
/*   to accommodate CLIPS Rete topology.   */
/*******************************************/
static struct lhsParseNode *PerformReorder(theLHS,newChange)
  struct lhsParseNode *theLHS;
  int *newChange;
  {
   struct lhsParseNode *argPtr, *lastArg, *nextArg;
   struct lhsParseNode *tempArg, *newNode;
   int count;
   int change;
     
   /*======================================================*/
   /* Loop through the CEs as long as changes can be made. */
   /*======================================================*/
   
   change = CLIPS_TRUE;
   *newChange = CLIPS_FALSE;
   
   while (change)
     {
      change = CLIPS_FALSE;
      count = 1;
      lastArg = NULL;
      argPtr = theLHS->right;

      while (argPtr != NULL)
        {
         /*=============================================================*/
         /* Convert and/or CE combinations into or/and CE combinations. */
         /*=============================================================*/
         
         if ((theLHS->type == AND_CE) && (argPtr->type == OR_CE))
           {
            theLHS = ReverseAndOr(theLHS,argPtr->right,count); 
            
            change = CLIPS_TRUE;
            *newChange = CLIPS_TRUE;
            argPtr = NULL;
           }
           
         /*==============================================================*/
         /* Convert not/or CE combinations into and/not CE combinations. */
         /*==============================================================*/
         
         else if ((theLHS->type == NOT_CE) && (argPtr->type == OR_CE))
           {
            change = CLIPS_TRUE;
            *newChange = CLIPS_TRUE;
            
            tempArg = argPtr->right;
            
            argPtr->right = NULL;
            argPtr->bottom = NULL;
            ReturnLHSParseNodes(argPtr);
            
            theLHS->type = AND_CE;
            theLHS->right = tempArg;

            while (tempArg != NULL)
              {
               newNode = GetLHSParseNode();
               CopyLHSParseNode(newNode,tempArg,CLIPS_FALSE);
               newNode->right = tempArg->right;
               newNode->bottom = NULL;
               
               tempArg->type = NOT_CE;
               tempArg->negated = CLIPS_FALSE;
               tempArg->logical = CLIPS_FALSE;
               tempArg->value = NULL;
               tempArg->expression = NULL;
               tempArg->right = newNode;
               
               tempArg = tempArg->bottom;
              }
              
            argPtr = NULL;
           }
           
         /*====================================================================*/
         /* Replace not CEs containing a pattern CE with a negated pattern CE. */
         /*====================================================================*/

         else if ((theLHS->type == NOT_CE) && (argPtr->type == PATTERN_CE))
           { 
            change = CLIPS_TRUE;
            *newChange = CLIPS_TRUE;
            
            CopyLHSParseNode(theLHS,argPtr,CLIPS_FALSE); 
            
            theLHS->negated = CLIPS_TRUE;
            theLHS->right = argPtr->right;
            
            argPtr->networkTest = NULL;
            argPtr->expression = NULL;
            argPtr->userData = NULL;
            argPtr->right = NULL;
            argPtr->bottom = NULL;
            ReturnLHSParseNodes(argPtr);  
            argPtr = NULL;
           }
           
         /*=========================================================*/
         /* Replace not CEs containing a test CE with an not/and CE */
         /* containing the initial pattern and the test CE.         */
         /*=========================================================*/

         else if ((theLHS->type == NOT_CE) && (argPtr->type == TEST_CE))
           { 
            change = CLIPS_TRUE;
            *newChange = CLIPS_TRUE;
            
            tempArg = GetLHSParseNode();
            tempArg->type = AND_CE;
            tempArg->right = CreateInitialPattern(NULL);
            tempArg->right->logical = argPtr->logical;
            tempArg->right->beginNandDepth = argPtr->beginNandDepth;
            tempArg->right->endNandDepth = argPtr->beginNandDepth;
            tempArg->right->bottom = argPtr;
            theLHS->right = tempArg;
            argPtr = NULL;
           }

         /*============================================================*/
         /* Replace "and" and "not" CEs contained within a not CE with */
         /* just the and CE, but increment the nand depths of the      */
         /* pattern contained within.                                  */
         /*============================================================*/
         
         else if ((theLHS->type == NOT_CE) && 
                  ((argPtr->type == AND_CE) || (argPtr->type == NOT_CE)))
           { 
            change = CLIPS_TRUE;
            *newChange = CLIPS_TRUE;
            
            theLHS->type = argPtr->type;
            theLHS->negated = argPtr->negated;
            theLHS->value = argPtr->value;
            theLHS->logical = argPtr->logical;
            theLHS->right = argPtr->right;
            argPtr->right = NULL;
            argPtr->bottom = NULL;
            ReturnLHSParseNodes(argPtr);  
            
            IncrementNandDepth(theLHS->right,CLIPS_TRUE);
            argPtr = NULL;
           }

         /*========================================================================*/
         /* Remove duplication of and CEs within and CEs and or CEs within or CEs. */
         /*========================================================================*/
         
         else if (((theLHS->type == AND_CE) && (argPtr->type == AND_CE)) ||
                  ((theLHS->type == OR_CE) && (argPtr->type == OR_CE)))
           {    
            if (argPtr->logical) theLHS->logical = CLIPS_TRUE;
            
            change = CLIPS_TRUE;
            *newChange = CLIPS_TRUE;
            tempArg = argPtr->right;
            nextArg = argPtr->bottom;
            argPtr->right = NULL;
            argPtr->bottom = NULL;
            ReturnLHSParseNodes(argPtr);

            if (lastArg == NULL)
              { theLHS->right = tempArg; }
            else
              { lastArg->bottom = tempArg; }

            argPtr = tempArg;
            while (tempArg->bottom != NULL) tempArg = tempArg->bottom;
            tempArg->bottom = nextArg;
           } 
           
         /*===================================================*/
         /* If no changes are needed, move on to the next CE. */
         /*===================================================*/
    
         else
           {
            count++;
            lastArg = argPtr;
            argPtr = argPtr->bottom;
           }
        }
     }
          
   /*=========================*/
   /* Return the reorder LHS. */
   /*=========================*/

   return(theLHS);
  }
  
/**************************************************/
/* ReverseAndOr: Switches and/or constructs into  */
/*   or/and constructs.                           */
/*   For example:                                 */
/*     (and (or a b) (or c d))                    */
/*   would be converted to                        */
/*     (or (and a (or c d)) (and b (or c d))),    */
/*   if the or pointer were pointing at (or a b). */
/**************************************************/
static struct lhsParseNode *ReverseAndOr(listOfCEs,orCE,orPosition)
  struct lhsParseNode *listOfCEs;
  struct lhsParseNode *orCE;
  int orPosition;
  {
   int count;
   struct lhsParseNode *listOfExpandedOrCEs = NULL;
   struct lhsParseNode *lastExpandedOrCE = NULL;
   struct lhsParseNode *copyOfCEs, *replaceCE;

   /*===================================================================*/
   /* Compute all of the CEs that can be formed by expanding the or CE. */
   /*===================================================================*/
      
   while (orCE != NULL)
     {
      /* Find the... */
      copyOfCEs = CopyLHSParseNodes(listOfCEs);
      count = 1;
      replaceCE = copyOfCEs->right;
      while (count != orPosition)
        {
         replaceCE = replaceCE->bottom;
         count++;
        }
        
      ReturnLHSParseNodes(replaceCE->right);
      CopyLHSParseNode(replaceCE,orCE,CLIPS_TRUE);
      replaceCE->right = CopyLHSParseNodes(orCE->right);

      if (lastExpandedOrCE == NULL)
        {
         listOfExpandedOrCEs = copyOfCEs;
         copyOfCEs->bottom = NULL;
         lastExpandedOrCE = copyOfCEs;
        }
      else
        {
         lastExpandedOrCE->bottom = copyOfCEs;
         copyOfCEs->bottom = NULL;
         lastExpandedOrCE = copyOfCEs;
        }

      orCE = orCE->bottom;
     }

   /*==============================*/
   /* Return the original CE list. */
   /*==============================*/
   
   ReturnLHSParseNodes(listOfCEs);

   /*================================================*/
   /* Wrap an or CE around the list of expanded CEs. */
   /*================================================*/
   
   copyOfCEs = GetLHSParseNode();
   copyOfCEs->type = OR_CE;
   copyOfCEs->right = listOfExpandedOrCEs;

   /*================================*/
   /* Return the newly expanded CEs. */
   /*================================*/
   
   return(copyOfCEs);
  }

/************************************************************/
/* CopyLHSParseNodes: Copies a set of conditional elements. */
/************************************************************/
globle struct lhsParseNode *CopyLHSParseNodes(listOfCEs)
  struct lhsParseNode *listOfCEs;
  {
   struct lhsParseNode *newList;

   if (listOfCEs == NULL)
     { return(NULL); }

   newList = get_struct(lhsParseNode);
   CopyLHSParseNode(newList,listOfCEs,CLIPS_TRUE);
   
   newList->right = CopyLHSParseNodes(listOfCEs->right);
   newList->bottom = CopyLHSParseNodes(listOfCEs->bottom);

   return(newList);
  }

/***********************************************************/
/* CopyLHSParseNode: Copies a single conditional elements. */
/***********************************************************/
globle VOID CopyLHSParseNode(dest,src,duplicate)
  struct lhsParseNode *dest, *src;
  int duplicate;
  {
   dest->type = src->type;
   dest->value = src->value;
   dest->negated = src->negated;
   dest->bindingVariable = src->bindingVariable;
   dest->withinMultifieldSlot = src->withinMultifieldSlot;
   dest->multifieldSlot = src->multifieldSlot;
   dest->multiFieldsBefore = src->multiFieldsBefore;
   dest->multiFieldsAfter = src->multiFieldsAfter;
   dest->singleFieldsBefore = src->singleFieldsBefore;
   dest->singleFieldsAfter = src->singleFieldsAfter;
   dest->logical = src->logical; 
   dest->userCE = src->userCE; 
   dest->referringNode = src->referringNode;
   dest->patternType = src->patternType;
   dest->pattern = src->pattern;
   dest->index = src->index;
   dest->slot = src->slot;
   dest->slotNumber = src->slotNumber;
   dest->beginNandDepth = src->beginNandDepth;
   dest->endNandDepth = src->endNandDepth;
  
   if (duplicate) 
     {
      dest->networkTest = CopyExpression(src->networkTest);
      if (src->userData == NULL) dest->userData = NULL;
      else if (src->patternType->copyUserDataFunction == NULL) dest->userData = src->userData;
      else dest->userData = (*src->patternType->copyUserDataFunction)(src->userData);
      dest->expression = CopyLHSParseNodes(src->expression);
      dest->constraints = CopyConstraintRecord(src->constraints);
      if (dest->constraints != NULL) dest->derivedConstraints = CLIPS_TRUE; 
      else dest->derivedConstraints = CLIPS_FALSE;
     }
   else 
     {
      dest->networkTest = src->networkTest;
      dest->userData = src->userData;
      dest->expression = src->expression;
      dest->derivedConstraints = CLIPS_FALSE; 
      dest->constraints = src->constraints;
     }
  }

/****************************************************/
/* GetLHSParseNode: Creates an empty node structure */
/*   used for building conditional elements.        */
/****************************************************/
globle struct lhsParseNode *GetLHSParseNode()
  {
   struct lhsParseNode *newNode;

   newNode = get_struct(lhsParseNode);
   newNode->type = UNKNOWN;
   newNode->value = NULL;
   newNode->negated = CLIPS_FALSE;
   newNode->bindingVariable = CLIPS_FALSE;
   newNode->withinMultifieldSlot = CLIPS_FALSE;
   newNode->multifieldSlot = CLIPS_FALSE;
   newNode->multiFieldsBefore = 0;
   newNode->multiFieldsAfter = 0;
   newNode->singleFieldsBefore = 0;
   newNode->singleFieldsAfter = 0;
   newNode->logical = CLIPS_FALSE; 
   newNode->derivedConstraints = CLIPS_FALSE;
   newNode->userCE = CLIPS_TRUE;
   newNode->constraints = NULL;
   newNode->referringNode = NULL;
   newNode->patternType = NULL;
   newNode->pattern = -1;
   newNode->index = -1;
   newNode->slot = NULL;
   newNode->slotNumber = -1;
   newNode->beginNandDepth = 1;
   newNode->endNandDepth = 1;
   newNode->userData = NULL;
   newNode->networkTest = NULL;
   newNode->expression = NULL;
   newNode->right = NULL;
   newNode->bottom = NULL;

   return(newNode);
  }

/***********************************************************/
/* ReturnLHSParseNodes:  Returns a multiply linked list of */
/*   lhsParseNode structures to the CLIPS memory manager.  */
/***********************************************************/
globle VOID ReturnLHSParseNodes(waste)
  struct lhsParseNode *waste;
  {
   if (waste != NULL)
     {
      ReturnExpression(waste->networkTest);
      ReturnLHSParseNodes(waste->right);
      ReturnLHSParseNodes(waste->bottom);
      ReturnLHSParseNodes(waste->expression);
      if (waste->derivedConstraints) RemoveConstraint(waste->constraints);
      if ((waste->userData != NULL) &&
          (waste->patternType->returnUserDataFunction != NULL))
        { (*waste->patternType->returnUserDataFunction)(waste->userData); }
      rtn_struct(lhsParseNode,waste);
     }
  }
    
/********************************************************/
/* ExpressionToLHSParseNodes: Copies an expression into */
/*   the equivalent lhsParseNode data structures.       */
/********************************************************/
globle struct lhsParseNode *ExpressionToLHSParseNodes(expressionList)
  struct expr *expressionList;
  {
   struct lhsParseNode *newList, *theList;
   struct FunctionDefinition *theFunction;
   int i = 1, theRestriction;

   if (expressionList == NULL)
     { return(NULL); }

   newList = GetLHSParseNode();
   newList->type = expressionList->type;
   newList->value = expressionList->value;
   /*newList->isExpression = CLIPS_TRUE;*/
   newList->right = ExpressionToLHSParseNodes(expressionList->nextArg);
   newList->bottom = ExpressionToLHSParseNodes(expressionList->argList);

   if (newList->type != FCALL) return(newList);
   
   theFunction = (struct FunctionDefinition *) newList->value;
   theList = newList->bottom;
   while (theList != NULL)
     {
      if (theList->type == SF_VARIABLE)
        {
         theRestriction = GetNthRestriction(theFunction,i);
         theList->constraints = ArgumentTypeToConstraintRecord(theRestriction);
         theList->derivedConstraints = CLIPS_TRUE;
        }
        
      i++;
      theList = theList->right;
     }
     
   return(newList);
  }
  
/******************************************************************/
/* LHSParseNodesToExpression: Copies lhsParseNode data structures */
/*   into the equivalent expression data structures.              */
/******************************************************************/
globle struct expr *LHSParseNodesToExpression(nodeList)
  struct lhsParseNode *nodeList;
  {
   struct expr *newList;

   if (nodeList == NULL)
     { return(NULL); }

   newList = get_struct(expr);
   newList->type = nodeList->type;
   newList->value = nodeList->value;
   newList->nextArg = LHSParseNodesToExpression(nodeList->right);
   newList->argList = LHSParseNodesToExpression(nodeList->bottom);

   return(newList);
  }

/*******************************************************/
/* IncrementNandDepth:         */
/*******************************************************/
static VOID IncrementNandDepth(theLHS,lastCE)
  struct lhsParseNode *theLHS;
  int lastCE;
  {
   while (theLHS != NULL)
     {
      if ((theLHS->type == PATTERN_CE) || (theLHS->type == TEST_CE))
        {
         theLHS->beginNandDepth++;
         if ((lastCE == CLIPS_FALSE) || (theLHS->bottom != NULL))
           { theLHS->endNandDepth++; }
        }
      else if ((theLHS->type == OR_CE) || 
               (theLHS->type == AND_CE) || 
               (theLHS->type == NOT_CE))
        {
         IncrementNandDepth(theLHS->right,
                                    (lastCE ? (theLHS->bottom == NULL) : CLIPS_FALSE)); 
        }
        
      theLHS = theLHS->bottom;
     }
  } 
  
/**************************************************************/
/* CreateInitialPattern:  */
/**************************************************************/
static struct lhsParseNode *CreateInitialPattern(theParser)
  struct patternParser *theParser;
  {
   struct lhsParseNode *topNode;
   
   topNode = GetLHSParseNode();
   topNode->type = PATTERN_CE;
   topNode->userCE = CLIPS_FALSE;
   topNode->bottom = NULL;
   
   /*==============================================*/
   /* If a pattern type was not supplied, then try */
   /* to use the (initial-fact) fact pattern.      */
   /*==============================================*/
   
   if (theParser == NULL) 
     { theParser = FindPatternParser("facts"); }
   
   if (theParser != NULL) 
     {
      topNode->right = (*theParser->initialPatternFunction)();
      PropagatePatternType(topNode,theParser);
      return(topNode);
     }
   
   theParser = ListOfPatternParsers;
   while (theParser != NULL)
     {
      if (theParser->initialPatternFunction != NULL)
        { 
         topNode->right = (*theParser->initialPatternFunction)();
         PropagatePatternType(topNode,theParser);
         return(topNode);
        }
        
      theParser = theParser->next;
     }
     
   CLIPSSystemError("DEFRULE",2);
   return(NULL);
  }

/****************************************************************/
/* HandleOrphanedTestCEs:                   */
/****************************************************************/
static VOID HandleOrphanedTestCEs(theLHS)
  struct lhsParseNode *theLHS;
  {
   struct lhsParseNode *trackNode, *tempNode, *lastNode;
   
   for (trackNode = theLHS; trackNode != NULL; trackNode = trackNode->bottom)
     { trackNode->marked = CLIPS_FALSE; }
   
   lastNode = NULL;
   while (theLHS != NULL)
     {
      if (theLHS->negated)
        { trackNode = NULL; }
      else if ((theLHS->type == TEST_CE) && (theLHS->marked == CLIPS_FALSE))
        {
         tempNode = CreateInitialPattern(theLHS->patternType);
         tempNode->logical = theLHS->logical;
         tempNode->beginNandDepth = theLHS->beginNandDepth;
         tempNode->endNandDepth = theLHS->beginNandDepth;
            
         if (lastNode == NULL)
           {
            CLIPSSystemError("REORDER",8);
           }
              
         lastNode->bottom = tempNode;
         tempNode->bottom = theLHS;
         theLHS = tempNode;
         trackNode = theLHS->bottom;
        }
      else
        { trackNode = theLHS->bottom; }
      
      while (trackNode != NULL)
        {         
         if ((trackNode->beginNandDepth != theLHS->beginNandDepth) ||
             (trackNode->negated))
           { trackNode = trackNode->bottom; }
         
         else if (trackNode->type == PATTERN_CE)
           { trackNode = NULL; }
         
         else if (trackNode->type == TEST_CE)
           {
            trackNode->marked = CLIPS_TRUE;
            trackNode->patternType = theLHS->patternType;
            trackNode = trackNode->bottom;
           }
        }
      
      lastNode = theLHS;
      theLHS = theLHS->bottom;
     }
  } 
  
/*************************************************/
/* PrintNodes */
/*************************************************/
static VOID PrintNodes(fileid,theNode)
  char *fileid;
  struct lhsParseNode *theNode;
  {
   if (theNode == NULL) return;

   while (theNode != NULL)
     {
      switch (theNode->type)
        {
         case PATTERN_CE:
           PrintCLIPS(fileid,"(");
           PrintCLIPS(fileid,ValueToString(theNode->right->bottom->value));
           PrintCLIPS(fileid,")");
           break;
            
         case TEST_CE:
         
         case NOT_CE:
           PrintCLIPS(fileid,"(not ");
           PrintNodes(fileid,theNode->right);
           PrintCLIPS(fileid,")");
           break;

         case OR_CE:
           PrintCLIPS(fileid,"(or ");
           PrintNodes(fileid,theNode->right);
           PrintCLIPS(fileid,")");
           break;
         
         case AND_CE:
           PrintCLIPS(fileid,"(and ");
           PrintNodes(fileid,theNode->right);
           PrintCLIPS(fileid,")");
           break;
        }

      theNode = theNode->bottom;
      if (theNode != NULL) PrintCLIPS(fileid," ");
     }

   return;
  }
  
/*******************************************************/
/* AssignPatternIndices: For each pattern in the LHS of a rule, */
/*   determines */
/*******************************************************/
static struct lhsParseNode *AssignPatternIndices(theLHS,startIndex)
  struct lhsParseNode *theLHS;
  int startIndex;
  {
   int depth;
   struct lhsParseNode *theField;
   
   depth = theLHS->beginNandDepth;
   
   while (theLHS != NULL)
     {
      if (theLHS->beginNandDepth > depth)
        { 
         theLHS = AssignPatternIndices(theLHS,startIndex); 
         if (theLHS->endNandDepth < depth) return(theLHS);
         startIndex++;
        }
      else if (theLHS->type == TEST_CE)
        { if (theLHS->endNandDepth < depth) return(theLHS); }
      else if (theLHS->type == PATTERN_CE)
        {
         theLHS->pattern = startIndex; 
         for (theField = theLHS->right; theField != NULL; theField = theField->right) 
           { 
            theField->pattern = startIndex; 
            PropagateIndexSlotPatternValues(theField,theField->pattern,
                                            theField->index,theField->slot,
                                            theField->slotNumber);
           }
           
         if (theLHS->endNandDepth < depth) return(theLHS);
         startIndex++;
        }
      
      theLHS = theLHS->bottom;
     }
   
   return(NULL);
  }
  
/*******************************************************/
/* PropagateIndexSlotPatternValues: */
/*******************************************************/
static VOID PropagateIndexSlotPatternValues(theField,thePattern,theIndex,theSlot,theSlotNumber)
  struct lhsParseNode *theField;
  int thePattern, theIndex;
  struct symbolHashNode *theSlot;
  int theSlotNumber;
  {
   struct lhsParseNode *tmpNode, *andField;
   
   if (theField == NULL) return;
   
   if (theField->multifieldSlot)
     {
      theField->pattern = thePattern;
      if (theIndex > 0) theField->index = theIndex;
      theField->slot = theSlot;
      theField->slotNumber = theSlotNumber;
      
      tmpNode = theField->bottom;
      while (tmpNode != NULL)
        {
         tmpNode->pattern = thePattern;
         tmpNode->slot = theSlot;
         PropagateIndexSlotPatternValues(tmpNode,thePattern,tmpNode->index,
                                         theSlot,theSlotNumber);
         tmpNode = tmpNode->right;
        }
      return;
     }
        
   theField = theField->bottom;
   while (theField != NULL)
     {
      for (andField = theField; andField != NULL; andField = andField->right)
        {
         andField->pattern = thePattern;
         if (theIndex > 0) andField->index = theIndex;
         andField->slot = theSlot;
         andField->slotNumber = theSlotNumber;
        }

      theField = theField->bottom;
     }
   }
     
#endif

