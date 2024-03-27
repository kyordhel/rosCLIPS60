   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             EXPRESSION OPERATIONS MODULE            */
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

#define _EXPRNOPS_SOURCE_

#include "setup.h"

#include <stdio.h>
#define _CLIPS_STDIO_
#if ANSI_COMPILER
#include <stdlib.h>
#endif
#include <string.h>
#include <ctype.h>

#include "clipsmem.h"
#include "router.h"
#include "extnfunc.h"
#include "cstrnchk.h"
#include "prntutil.h"
#include "cstrnutl.h"
#include "cstrnops.h"

#include "exprnops.h"

#define PRIME_ONE   257
#define PRIME_TWO   263
#define PRIME_THREE 269

#if (! RUN_TIME)

/**************************************************************/
/* CheckArgumentAgainstRestriction: Compares an argument to a */
/*   function to the set of restrictions for that function to */
/*   determine if any incompatibilities exist. If so, the     */
/*   value TRUE is returned, otherwise FALSE is returned.     */
/*   Restrictions checked are:                                */
/*     a - external address                                   */
/*     d - float                                              */
/*     e - instance address, instance name, or symbol         */
/*     f - float                                              */
/*     g - integer, float, or symbol                          */
/*     h - instance address, instance name, fact address,     */
/*         integer, or symbol                                 */
/*     i - integer                                            */
/*     j - symbol, string, or instance name                   */
/*     k - symbol or string                                   */
/*     l - integer                                            */
/*     m - multifield                                         */
/*     n - float or integer                                   */
/*     o - instance name                                      */
/*     p - instance name or symbol                            */
/*     q - string, symbol, or multifield                      */
/*     s - string                                             */
/*     u - unknown (any type allowed)                         */
/*     w - symbol                                             */
/*     x - instance address                                   */
/*     y - fact address                                       */
/*     z - fact address, integer, or symbol (*)               */
/**************************************************************/
globle int CheckArgumentAgainstRestriction(theExpression,theRestriction)
  struct expr *theExpression;
  int theRestriction;
  {
   CONSTRAINT_RECORD *cr1, *cr2, *cr3;

   /*=============================*/
   /* Check for the correct type. */
   /*=============================*/
   
   cr1 = ExpressionToConstraintRecord(theExpression);
   
   cr2 = ArgumentTypeToConstraintRecord(theRestriction);
   
   cr3 = IntersectConstraints(cr1,cr2);
   
   RemoveConstraint(cr1);
   RemoveConstraint(cr2);
   
   if (UnmatchableConstraint(cr3))
     { 
      RemoveConstraint(cr3);
      return(CLIPS_TRUE);
     }

   RemoveConstraint(cr3);
   return(CLIPS_FALSE);
  }

#endif

/************************************************************/
/* ConstantExpression: Returns CLIPS_TRUE if the expression */
/*   is a constant, otherwise CLIPS_FALSE.                  */
/************************************************************/
globle BOOLEAN ConstantExpression(testPtr)
  struct expr *testPtr;
  {
   while (testPtr != NULL)
     {
      if ((testPtr->type != SYMBOL) && (testPtr->type != STRING) &&
#if OBJECT_SYSTEM
          (testPtr->type != INSTANCE_NAME) && (testPtr->type != INSTANCE_ADDRESS) &&
#endif
          (testPtr->type != INTEGER) && (testPtr->type != FLOAT))
        { return(CLIPS_FALSE); }
      testPtr = testPtr->nextArg;
     }

   return(CLIPS_TRUE);
  }
  
/************************************************/
/* ConstantType: Returns CLIPS_TRUE if the type */
/*   is a constant, otherwise CLIPS_FALSE.      */
/************************************************/
globle BOOLEAN ConstantType(theType)
  int theType;
  {
   switch (theType)
     {
      case SYMBOL:
      case STRING:
      case INTEGER:
      case FLOAT:
#if OBJECT_SYSTEM
      case INSTANCE_NAME:
      case INSTANCE_ADDRESS:
#endif
        return(CLIPS_TRUE);
        break;
     }
     
   return(CLIPS_FALSE);
  }
  
/*****************************************************************************/
/* IdenticalExpression: Determines if two expressions are identical. Returns */
/*   TRUE if the expressions are identical, otherwise FALSE is returned.     */
/*****************************************************************************/
globle BOOLEAN IdenticalExpression(firstList,secondList)
  struct expr *firstList;
  struct expr *secondList;
  {
   if ((firstList == NULL) && (secondList == NULL))
     { return(CLIPS_TRUE); }

   if ((firstList != NULL) && (secondList == NULL))
     { return(CLIPS_FALSE); }

   if ((firstList == NULL) && (secondList != NULL))
     { return(CLIPS_FALSE); }

   if (firstList->type != secondList->type)
     { return(CLIPS_FALSE); }

   if (firstList->value != secondList->value)
    { return (CLIPS_FALSE); }

   if (IdenticalExpression(firstList->argList,secondList->argList) == CLIPS_FALSE)
     { return(CLIPS_FALSE); }

   for (firstList = firstList->nextArg, secondList = secondList->nextArg;
        (firstList != NULL) && (secondList != NULL);
        firstList = firstList->nextArg, secondList = secondList->nextArg)
     {
      if (firstList->type != secondList->type)
        { return(CLIPS_FALSE); }

      if (firstList->value != secondList->value)
        { return (CLIPS_FALSE); }

      if (IdenticalExpression(firstList->argList,secondList->argList) == CLIPS_FALSE)
        { return(CLIPS_FALSE); }
     }

   if (firstList != secondList) return(CLIPS_FALSE);
   
   return(CLIPS_TRUE);
  }

/****************************************************/
/* CountArguments: Returns the number of structures */
/*   stored in an expression as traversed through   */
/*   the nextArg pointer but not the argList      */
/*   pointer.                                       */
/****************************************************/
globle int CountArguments(testPtr)
  struct expr *testPtr;
  {
   int size = 0;

   while (testPtr != NULL)
     {
      size++;
      testPtr = testPtr->nextArg;
     }

   return(size);
  }

/******************************************/
/* CopyExpresssion: Copies an expression. */
/******************************************/
globle struct expr *CopyExpression(original)
  struct expr *original;
  {
   struct expr *topLevel, *next, *last;

   if (original == NULL) return(NULL);

   topLevel = GenConstant(original->type,original->value);
   topLevel->argList = CopyExpression(original->argList);

   last = topLevel;
   original = original->nextArg;
   while (original != NULL)
     {
      next = GenConstant(original->type,original->value);
      next->argList = CopyExpression(original->argList);

      last->nextArg = next;
      last = next;
      original = original->nextArg;
     }

   return(topLevel);
  }
  
/*************************************************************************/
/* ExpressionContainsVariables: Determines if an expression contains any */
/*   variables. Returns TRUE if the expression contains any variables,   */
/*   otherwise FALSE is returned.                                        */
/*************************************************************************/
globle BOOLEAN ExpressionContainsVariables(theExpression,globalsAreVariables)
  struct expr *theExpression;
  BOOLEAN globalsAreVariables;
  {
   while (theExpression != NULL)
     {
      if (theExpression->argList != NULL)
        {
         if (ExpressionContainsVariables(theExpression->argList,globalsAreVariables))
           { return(CLIPS_TRUE); }
        }
      
      if ((theExpression->type == MF_VARIABLE) ||
               (((theExpression->type == GBL_VARIABLE) || (theExpression->type == MF_GBL_VARIABLE)) 
                    && (globalsAreVariables == CLIPS_TRUE)) ||
               (theExpression->type == SF_VARIABLE) ||
               (theExpression->type == FACT_ADDRESS))
        { return(CLIPS_TRUE); }

      theExpression = theExpression->nextArg;
     }

   return(CLIPS_FALSE);
  }

/****************************************************/
/* ExpressionSize: Returns the number of structures */
/*   stored in an expression.                       */
/****************************************************/
globle int ExpressionSize(testPtr)
  struct expr *testPtr;
  {
   int size = 0;

   while (testPtr != NULL)
     {
      size++;
      if (testPtr->argList != NULL)
        { size += ExpressionSize(testPtr->argList); }
      testPtr = testPtr->nextArg;
     }
   return(size);
  }
  
/******************************************************/
/* GenConstant: Generates a constant expression value */
/*   of type string, symbol, or number.                 */
/******************************************************/
globle struct expr *GenConstant(type,value)
  int type;
  VOID *value;
  {
   struct expr *top;

   top = get_struct(expr);
   top->nextArg = NULL;
   top->argList = NULL;
   top->type = type;
   top->value = value;

   return(top);
  }
  
/*************************************************/
/* PrintExpression: Pretty prints an expression. */
/*************************************************/
globle VOID PrintExpression(fileid,theExpression)
  char *fileid;
  struct expr *theExpression;
  {
   struct expr *oldExpression;

   if (theExpression == NULL)
     { return; }

   while (theExpression != NULL)
     {
      switch (theExpression->type)
        {
         case SF_VARIABLE:
         case GBL_VARIABLE:
            PrintCLIPS(fileid,"?");
            PrintCLIPS(fileid,ValueToString(theExpression->value));
            break;

         case MF_VARIABLE:
         case MF_GBL_VARIABLE:
            PrintCLIPS(fileid,"$?");
            PrintCLIPS(fileid,ValueToString(theExpression->value));
            break;

         case FCALL:
           PrintCLIPS(fileid,"(");
           PrintCLIPS(fileid,ValueToString(ExpressionFunctionCallName(theExpression)));
           if (theExpression->argList != NULL) { PrintCLIPS(fileid," "); }
           PrintExpression(fileid,theExpression->argList);
           PrintCLIPS(fileid,")");
           break;

         default:
           oldExpression = CurrentExpression;
           CurrentExpression = theExpression;
           PrintAtom(fileid,theExpression->type,theExpression->value);
           CurrentExpression = oldExpression;
           break;
        }

      theExpression = theExpression->nextArg;
      if (theExpression != NULL) PrintCLIPS(fileid," ");
     }

   return;
  }

/*************************************************************************/
/* CombineExpressions: Combines two expressions into a single equivalent */
/*   expression. Mainly serves to merge expressions containing "and"     */
/*   and "or" expressions without unnecessary duplication of the "and"   */
/*   and "or" expressions (i.e., two "and" expressions can be merged by  */
/*   placing them as arguments within another "and" expression, but it   */
/*   is more efficient to add the arguments of one of the "and"          */
/*   expressions to the list of arguments for the other and expression). */
/*************************************************************************/
globle struct expr *CombineExpressions(expr1,expr2)
  struct expr *expr1, *expr2;
  {
   struct expr *tempPtr;

   /*===========================================================*/
   /* If the 1st expression is NULL, return the 2nd expression. */
   /*===========================================================*/

   if (expr1 == NULL) return(expr2);

   /*===========================================================*/
   /* If the 2nd expression is NULL, return the 1st expression. */
   /*===========================================================*/

   if (expr2 == NULL) return(expr1);

   /*============================================================*/
   /* If the 1st expression is an "and" expression, and the 2nd  */
   /* expression is not an "and" expression, then include the    */
   /* 2nd expression in the argument list of the 1st expression. */
   /*============================================================*/

   if ((expr1->value == PTR_AND) &&
       (expr2->value != PTR_AND))
     {
      tempPtr = expr1->argList;
      if (tempPtr == NULL)
        {
         rtn_struct(expr,expr1);
         return(expr2);
        }

      while (tempPtr->nextArg != NULL)
        { tempPtr = tempPtr->nextArg; }

      tempPtr->nextArg = expr2;
      return(expr1);
     }

   /*============================================================*/
   /* If the 2nd expression is an "and" expression, and the 1st  */
   /* expression is not an "and" expression, then include the    */
   /* 1st expression in the argument list of the 2nd expression. */
   /*============================================================*/

   if ((expr1->value != PTR_AND) &&
       (expr2->value == PTR_AND))
     {
      tempPtr = expr2->argList;
      if (tempPtr == NULL)
        {
         rtn_struct(expr,expr2);
         return(expr1);
        }

      expr2->argList = expr1;
      expr1->nextArg = tempPtr;

      return(expr2);
     }

   /*===========================================================*/
   /* If both expressions are "and" expressions, then add the   */
   /* 2nd expression to the argument list of the 1st expression */
   /* and throw away the extraneous "and" expression.           */
   /*===========================================================*/

   if ((expr1->value == PTR_AND) &&
       (expr2->value == PTR_AND))
     {
      tempPtr = expr1->argList;
      if (tempPtr == NULL)
        {
         rtn_struct(expr,expr1);
         return(expr2);
        }

      while (tempPtr->nextArg != NULL)
        { tempPtr = tempPtr->nextArg; }

      tempPtr->nextArg = expr2->argList;
      rtn_struct(expr,expr2);

      return(expr1);
     }

   /*=====================================================*/
   /* If neither expression is an "and" expression, then  */
   /* create an "and" expression and add both expressions */
   /* to the argument list of that "and" expression.      */
   /*=====================================================*/

   tempPtr = GenConstant(FCALL,PTR_AND);
   tempPtr->argList = expr1;
   expr1->nextArg = expr2;
   return(tempPtr);
  }
  
/*************************************************************************/
/* AppendExpressions: */
/*************************************************************************/
globle struct expr *AppendExpressions(expr1,expr2)
  struct expr *expr1, *expr2;
  {
   struct expr *tempPtr;

   /*===========================================================*/
   /* If the 1st expression is NULL, return the 2nd expression. */
   /*===========================================================*/

   if (expr1 == NULL) return(expr2);

   /*===========================================================*/
   /* If the 2nd expression is NULL, return the 1st expression. */
   /*===========================================================*/

   if (expr2 == NULL) return(expr1);

   /*====================================*/
   /* Find the end of the 1st expression */
   /* and attach the 2nd expression.     */
   /*====================================*/
          
   tempPtr = expr1;
   while (tempPtr->nextArg != NULL) tempPtr = tempPtr->nextArg;
   tempPtr->nextArg = expr2;
   
   /*===============================*/
   /* Return the merged expression. */
   /*===============================*/
   
   return(expr1);
  }

