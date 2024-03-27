   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*              RULE CONSTRAINTS MODULE                */
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

#define _RULECSTR_SOURCE_

#include "setup.h"

#if (! RUN_TIME) && (! BLOAD_ONLY) && DEFRULE_CONSTRUCT

#include <stdio.h>
#define _CLIPS_STDIO_

#include "router.h"
#include "reorder.h"
#include "cstrnchk.h"
#include "cstrnops.h"
#include "extnfunc.h"
#include "analysis.h"
#include "prcdrpsr.h"
#include "cstrnutl.h"
#include "rulepsr.h"

#include "rulecstr.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/
 
#if ANSI_COMPILER
   static BOOLEAN                 CheckForUnmatchableConstraints(struct lhsParseNode *,int);
   static BOOLEAN                 MultifieldCardinalityViolation(struct lhsParseNode *);
   static struct lhsParseNode    *UnionVariableConstraints(struct lhsParseNode *,
                                                     struct lhsParseNode *);
   static struct lhsParseNode    *AddToVariableConstraints(struct lhsParseNode *,
                                                    struct lhsParseNode *);
   static VOID                    ConstraintConflictMessage(struct symbolHashNode *,
                                                            int,int,struct symbolHashNode *);
#else
   static BOOLEAN                 CheckForUnmatchableConstraints();
   static BOOLEAN                 MultifieldCardinalityViolation();
   static struct lhsParseNode    *UnionVariableConstraints();
   static struct lhsParseNode    *AddToVariableConstraints();
   static VOID                    ConstraintConflictMessage();
#endif
  
/************************************************************/
/* CheckForUnmatchableConstraints: */
/************************************************************/
static BOOLEAN CheckForUnmatchableConstraints(theNode,whichCE)
  struct lhsParseNode *theNode;
  int whichCE;
  {   
   if (GetStaticConstraintChecking() == CLIPS_FALSE) return(CLIPS_FALSE);
      
   if (UnmatchableConstraint(theNode->constraints))
     {
      ConstraintConflictMessage(theNode->value,whichCE,
                                theNode->index,theNode->slot);
      return(CLIPS_TRUE);
     }
     
   return(CLIPS_FALSE);
  }

/*******************************************************************/
/* ConstraintConflictMessage: Error message used when a constraint */
/*   restriction for a slot prevents any value from matching the   */
/*   pattern constraint.                                           */
/*******************************************************************/
static VOID ConstraintConflictMessage(variableName,thePattern,theField,theSlot)
  struct symbolHashNode *variableName;
  int thePattern, theField;
  struct symbolHashNode *theSlot;
  {
   PrintErrorID("RULECSTR",1,CLIPS_TRUE);
   if (variableName != NULL)
     {
      PrintCLIPS(WERROR,"Variable ?");
      PrintCLIPS(WERROR,ValueToString(variableName));
      PrintCLIPS(WERROR," in CE #");
      PrintLongInteger(WERROR,(long int) thePattern);
     }
   else
     {
      PrintCLIPS(WERROR,"Pattern #");
      PrintLongInteger(WERROR,(long int) thePattern);
     }
     
   if (theSlot == NULL)
     {
      PrintCLIPS(WERROR," field #");
      PrintLongInteger(WERROR,(long int) theField);
     }
   else
     {
      PrintCLIPS(WERROR," slot ");
      PrintCLIPS(WERROR,ValueToString(theSlot));
     }
     
   PrintCLIPS(WERROR,"\nhas constraint conflicts which make the pattern unmatchable.\n");
  }
  
/************************************************************/
/* MultifieldCardinalityViolation: */
/************************************************************/
static BOOLEAN MultifieldCardinalityViolation(theNode)
  struct lhsParseNode *theNode;
  {
   struct lhsParseNode *tmpNode;
   struct expr *tmpMax;
   int minFields = 0;
   int maxFields = 0;
   int positiveInfinity = CLIPS_FALSE;
   CONSTRAINT_RECORD *newConstraint, *tempConstraint;
   
   if (theNode->multifieldSlot == CLIPS_FALSE) return(CLIPS_FALSE);
   
   tmpNode = theNode->bottom;
   while (tmpNode != NULL)
     {
      if ((tmpNode->type == SF_VARIABLE) ||
          (tmpNode->type == SF_WILDCARD))
        {
         minFields++;
         maxFields++;
        }
      else if (tmpNode->constraints != NULL)
        {
         if (tmpNode->constraints->minFields->value != NegativeInfinity)
           { minFields += ValueToLong(tmpNode->constraints->minFields->value); }
           
         tmpMax = tmpNode->constraints->maxFields;
         while (tmpMax->nextArg != NULL) tmpMax = tmpMax->nextArg;
         if (tmpMax->value == PositiveInfinity)
           { positiveInfinity = CLIPS_TRUE; }
         else
           { maxFields += ValueToLong(tmpMax->value); }
        }
      else
        { positiveInfinity = CLIPS_TRUE; }
        
      tmpNode = tmpNode->right;
     }
     
   /*==================================================================*/
   /* Create a constraint record for the cardinality of the sum of the */
   /* cardinalities of the restrictions inside the multifield slot.    */
   /*==================================================================*/
   
   if (theNode->constraints == NULL) tempConstraint = GetConstraintRecord();
   else tempConstraint = CopyConstraintRecord(theNode->constraints);
   ReturnExpression(tempConstraint->minFields);
   ReturnExpression(tempConstraint->maxFields);
   tempConstraint->minFields = GenConstant(INTEGER,AddLong((long) minFields));
   if (positiveInfinity) tempConstraint->maxFields = GenConstant(SYMBOL,PositiveInfinity);
   else tempConstraint->maxFields = GenConstant(INTEGER,AddLong((long) maxFields));
   
   /*================================================================*/
   /* Determine the final cardinality for the multifield slot by     */
   /* intersectingthe cardinality sum of the restrictions within     */
   /* the multifield slot with the original cardinality of the slot. */
   /*================================================================*/
   
   newConstraint = IntersectConstraints(theNode->constraints,tempConstraint);
   if (theNode->derivedConstraints) RemoveConstraint(theNode->constraints);
   RemoveConstraint(tempConstraint);
   theNode->constraints = newConstraint;
   theNode->derivedConstraints = CLIPS_TRUE;
   
   /*===================================================================*/
   /* Determine if the final cardinality for the slot can be satisfied. */
   /*===================================================================*/
   
   if (GetStaticConstraintChecking() == CLIPS_FALSE) return(CLIPS_FALSE);
   if (UnmatchableConstraint(newConstraint)) return(CLIPS_TRUE);
     
   return(CLIPS_FALSE);
  }
  
/************************************************************/
/* ProcessConnectedConstraints: */
/************************************************************/
globle BOOLEAN ProcessConnectedConstraints(theNode,multifieldHeader,patternHead)
  struct lhsParseNode *theNode, *multifieldHeader, *patternHead;
  {
   struct constraintRecord *orConstraints = NULL, *andConstraints;
   struct constraintRecord *tmpConstraints, *rvConstraints;
   struct lhsParseNode *orNode, *andNode;
   struct expr *tmpExpr;
     
   /*==============================*/
   /* Process a single field Slot. */
   /*==============================*/     
   
   for (orNode = theNode->bottom; orNode != NULL; orNode = orNode->bottom)
     {
      /*=================================================*/
      /* Intersect all of the &'ed constraints together. */
      /*=================================================*/
   
      andConstraints = NULL;
      for (andNode = orNode; andNode != NULL; andNode = andNode->right)
        {
         if (! andNode->negated)
           {
            if (andNode->type == RETURN_VALUE_CONSTRAINT)
              {
               if (andNode->expression->type == FCALL)
                 {
                  rvConstraints = FunctionCallToConstraintRecord(andNode->expression->value);
                  tmpConstraints = andConstraints;
                  andConstraints = IntersectConstraints(andConstraints,rvConstraints);
                  RemoveConstraint(tmpConstraints);
                  RemoveConstraint(rvConstraints);
                 }
              }
            else if (ConstantType(andNode->type))
              {
               tmpExpr = GenConstant(andNode->type,andNode->value);
               rvConstraints = ExpressionToConstraintRecord(tmpExpr);
               tmpConstraints = andConstraints;
               andConstraints = IntersectConstraints(andConstraints,rvConstraints);
               RemoveConstraint(tmpConstraints);
               RemoveConstraint(rvConstraints);
               ReturnExpression(tmpExpr); 
              }
            else if (andNode->constraints != NULL)
              {
               tmpConstraints = andConstraints;
               andConstraints = IntersectConstraints(andConstraints,andNode->constraints);
               RemoveConstraint(tmpConstraints);
              }
           }
        }
        
      /*===========================================================*/
      /* Intersect the &'ed constraints with the slot constraints. */
      /*===========================================================*/
      
      tmpConstraints = andConstraints;
      andConstraints = IntersectConstraints(andConstraints,theNode->constraints);
      RemoveConstraint(tmpConstraints);
               
      /*===============================================================*/
      /* Remove any negated constants from the list of allowed values. */
      /*===============================================================*/
      
      for (andNode = orNode; andNode != NULL; andNode = andNode->right)
        {
         if ((andNode->negated) && ConstantType(andNode->type))
             { RemoveConstantFromConstraint(andNode->type,andNode->value,andConstraints); }
        }
        
      /*=======================================================*/
      /* Union the &'ed constraints with the |'ed constraints. */
      /*=======================================================*/
      
      tmpConstraints = orConstraints;
      orConstraints = UnionConstraints(orConstraints,andConstraints);
      RemoveConstraint(tmpConstraints);
      RemoveConstraint(andConstraints);
     }
     
   if (orConstraints != NULL) 
     {
      if (theNode->derivedConstraints) RemoveConstraint(theNode->constraints);
      theNode->constraints = orConstraints;
      theNode->derivedConstraints = CLIPS_TRUE;
     }
   
   if (CheckForUnmatchableConstraints(theNode,(int) patternHead->whichCE)) return(CLIPS_TRUE);
         
   if ((multifieldHeader != NULL) && (theNode->right == NULL))
     {
      if (MultifieldCardinalityViolation(multifieldHeader)) 
        { 
         ConstraintViolationErrorMessage("The group of restrictions",
                                                  NULL,CLIPS_FALSE,
                                                  (int) patternHead->whichCE,
                                                  multifieldHeader->slot,
                                                  multifieldHeader->index,
                                                  CARDINALITY_VIOLATION,
                                                  multifieldHeader->constraints,CLIPS_TRUE);
          return(CLIPS_TRUE);
         }
      }
      
   return(CLIPS_FALSE);
  }

/**********************************/
/* ConstraintReferenceErrorMessage: */
/**********************************/
globle VOID ConstraintReferenceErrorMessage(theVariable,theExpression,whichArgument,
                                            whichCE,slotName,theField)
  struct symbolHashNode *theVariable;
  struct lhsParseNode *theExpression;
  int whichArgument;
  int whichCE;
  struct symbolHashNode *slotName;
  int theField;
  {
   struct expr *temprv;
   
   PrintErrorID("RULECSTR",2,CLIPS_TRUE);

   PrintCLIPS(WERROR,"Previous variable bindings of ?");
   PrintCLIPS(WERROR,ValueToString(theVariable));
   PrintCLIPS(WERROR," caused the type restrictions");
   
   PrintCLIPS(WERROR,"\nfor argument #");
   PrintLongInteger(WERROR,(long int) whichArgument);
   PrintCLIPS(WERROR," of the expression ");
   
   temprv = LHSParseNodesToExpression(theExpression);
   ReturnExpression(temprv->nextArg);
   temprv->nextArg = NULL;
   PrintExpression(WERROR,temprv);
   PrintCLIPS(WERROR,"\n");
   ReturnExpression(temprv);
   
   PrintCLIPS(WERROR,"found in CE #");
   PrintLongInteger(WERROR,(long int) whichCE);
   if (slotName == NULL)
     {
      if (theField > 0)
        {
         PrintCLIPS(WERROR," field #");
         PrintLongInteger(WERROR,(long int) theField);
        }
     }
   else
     {
      PrintCLIPS(WERROR," slot ");
      PrintCLIPS(WERROR,ValueToString(slotName));
     }
           
   PrintCLIPS(WERROR," to be violated.\n");
  }
  
/************************************************************/
/* AddToVariableConstraints: */
/************************************************************/
static struct lhsParseNode *AddToVariableConstraints(list1,list2)
  struct lhsParseNode *list1, *list2;
  {
   CONSTRAINT_RECORD *newConstraints;
   struct lhsParseNode *temp, *trace;
   int found;
      
   while (list2 != NULL)
     {
      temp = list2->right;
      list2->right = NULL;
      
      trace = list1;
      found = CLIPS_FALSE;
      while (trace != NULL)
        {
         if (trace->value == list2->value)
           {
            newConstraints = IntersectConstraints(trace->constraints,list2->constraints);
            RemoveConstraint(trace->constraints);
            trace->constraints = newConstraints;
            ReturnLHSParseNodes(list2);
            trace = NULL;
            found = CLIPS_TRUE;
           }
         else
           { trace = trace->right; }
        }
        
      if (! found)
        {
         list2->right = list1;
         list1 = list2;
        }
        
      list2 = temp;
     }
     
   return(list1);
  }
  
/************************************************************/
/* UnionVariableConstraints: */
/************************************************************/
static struct lhsParseNode *UnionVariableConstraints(list1,list2)
  struct lhsParseNode *list1, *list2;
  {
   struct lhsParseNode *list3 = NULL, *trace, *temp;
   
   while (list1 != NULL)
     {
      trace = list2;
      while (trace != NULL)
        {
         if (list1->value == trace->value)
           {
            temp = GetLHSParseNode();
            temp->derivedConstraints = CLIPS_TRUE;
            temp->value = list1->value;
            temp->constraints = UnionConstraints(list1->constraints,trace->constraints);
            temp->right = list3;
            list3 = temp;
            trace = NULL;
           }
         else
           { trace = trace->right; }
        }
        
      temp = list1->right;
      list1->right = NULL;
      ReturnLHSParseNodes(list1);
      list1 = temp;
     }
     
   ReturnLHSParseNodes(list2);
   
   return(list3);
  }
  
/*****************************************************************/
/* GetExpressionVarConstraints: Given an expression stored using */
/*   the LHS parse node data structures, determines and returns  */
/*   the constraints on variables caused by that expression. For */
/*   example, the expression (+ ?x 1) would imply a numeric type */
/*   constraint for the variable ?x since the addition function  */
/*   expects numeric arguments.                                  */
/*****************************************************************/
globle struct lhsParseNode *GetExpressionVarConstraints(theExpression)
  struct lhsParseNode *theExpression;
  {
   struct lhsParseNode *list1 = NULL, *list2;
   
   while (theExpression != NULL)
     {
      if (theExpression->right != NULL)
        { 
         list2 = GetExpressionVarConstraints(theExpression->right); 
         list1 = AddToVariableConstraints(list2,list1);
        }
      
      if (theExpression->type == SF_VARIABLE)
        {
         list2 = GetLHSParseNode();
         if (theExpression->referringNode != NULL)
           { list2->type = theExpression->referringNode->type; }
         else
           { list2->type = SF_VARIABLE; }
         list2->value = theExpression->value;
         list2->derivedConstraints = CLIPS_TRUE;
         list2->constraints = CopyConstraintRecord(theExpression->constraints);
         list1 = AddToVariableConstraints(list2,list1);
        }
        
      theExpression = theExpression->bottom;
     }
     
   return(list1);   
  }
  
/************************************************************/
/* DeriveVariableConstraints: */
/************************************************************/
globle struct lhsParseNode *DeriveVariableConstraints(theNode)
  struct lhsParseNode *theNode;
  {
   struct lhsParseNode *orNode, *andNode;
   struct lhsParseNode *list1 = NULL, *list2 = NULL, *list3 = NULL;
   int first = CLIPS_TRUE;
     
   /*==============================*/
   /* Process a single field Slot. */
   /*==============================*/     
   
   for (orNode = theNode->bottom; orNode != NULL; orNode = orNode->bottom)
     {
      /*=================================================*/
      /* Intersect all of the &'ed constraints together. */
      /*=================================================*/
     
      list2 = NULL;
      for (andNode = orNode; andNode != NULL; andNode = andNode->right)
        {
         if ((andNode->type == RETURN_VALUE_CONSTRAINT) ||
             (andNode->type == PREDICATE_CONSTRAINT))
           {
            list1 = GetExpressionVarConstraints(andNode->expression);
            list2 = AddToVariableConstraints(list2,list1);
           }
        }
        
      if (first)
        {
         list3 = list2;
         first = CLIPS_FALSE;
        }
      else
        { list3 = UnionVariableConstraints(list3,list2); }
     }
     
   return(list3);
  }
  
/***************************************************/
/* CheckRHSForConstraintErrors:                 */
/***************************************************/
globle BOOLEAN CheckRHSForConstraintErrors(expressionList,theLHS)
  struct expr *expressionList;
  struct lhsParseNode *theLHS;
  {
   struct FunctionDefinition *theFunction;
   int i = 1, theRestriction;
   struct expr *lastOne, *tmpPtr, *checkList;
   CONSTRAINT_RECORD *constraint1, *constraint2, *constraint3, *constraint4;
   struct lhsParseNode *theVariable;
   
   /*==================================================================*/
   /* If there isn't an expression, there can't be a constraint error. */
   /*==================================================================*/
   
   if (expressionList == NULL) return(CLIPS_FALSE);
      
   for (checkList = expressionList; 
        checkList != NULL;
        checkList = checkList->nextArg)
      {
       expressionList = checkList->argList;
       i = 1;
       if (checkList->type == FCALL)
         {
          lastOne = checkList;
          theFunction = (struct FunctionDefinition *) checkList->value;
         }
       else
         { theFunction = NULL; }
   
       while (expressionList != NULL)
         {
          if ((expressionList->type == SF_VARIABLE) && (theFunction != NULL))
            {
             theRestriction = GetNthRestriction(theFunction,i);
             constraint1 = ArgumentTypeToConstraintRecord(theRestriction);
         
             theVariable = FindVariable((SYMBOL_HN *) expressionList->value,theLHS);
             if (theVariable != NULL)
               {
                if (theVariable->type == MF_VARIABLE)
                  {
                   constraint2 = GetConstraintRecord();
                   SetConstraintType(MULTIFIELD,constraint2);
                  }
                else if (theVariable->constraints == NULL)
                  { constraint2 = GetConstraintRecord(); }
                else
                  { constraint2 = CopyConstraintRecord(theVariable->constraints); }
               }
             else
               { constraint2 = NULL; }
           
             constraint3 = FindBindConstraints((SYMBOL_HN *) expressionList->value);
             constraint3 = UnionConstraints(constraint3,constraint2);
             constraint4 = IntersectConstraints(constraint3,constraint1);
         
             RemoveConstraint(constraint1);
             RemoveConstraint(constraint2);
             RemoveConstraint(constraint3);
         
             if (UnmatchableConstraint(constraint4) && GetStaticConstraintChecking())
               {
                PrintErrorID("RULECSTR",3,CLIPS_TRUE);
                PrintCLIPS(WERROR,"Previous variable bindings of ?");
                PrintCLIPS(WERROR,ValueToString((SYMBOL_HN *) expressionList->value));
                PrintCLIPS(WERROR," caused the type restrictions");
                PrintCLIPS(WERROR,"\nfor argument #");
                PrintLongInteger(WERROR,(long int) i);
                PrintCLIPS(WERROR," of the expression ");
                tmpPtr = lastOne->nextArg;
                lastOne->nextArg = NULL;
                PrintExpression(WERROR,lastOne);
                lastOne->nextArg = tmpPtr;
                PrintCLIPS(WERROR,"\nfound in the rule's RHS to be violated.\n");
            
                RemoveConstraint(constraint4);
                return(CLIPS_TRUE);
               }
           
             RemoveConstraint(constraint4);
            }
        
          i++;   
          tmpPtr = expressionList->nextArg;
          expressionList->nextArg = NULL;
          if (CheckRHSForConstraintErrors(expressionList,theLHS)) return(CLIPS_TRUE);
          expressionList->nextArg = tmpPtr;
          expressionList = expressionList->nextArg;
         }
      }
     
   return(CLIPS_FALSE);
  }
  
#endif
