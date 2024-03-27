   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                   GENERATE MODULE                   */
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

#define _GENERATE_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_

#include "setup.h"

#if (! RUN_TIME) && (! BLOAD_ONLY) && DEFRULE_CONSTRUCT

#include "constant.h"
#include "clipsmem.h"
#include "symbol.h"
#include "exprnpsr.h"
#include "argacces.h"
#include "extnfunc.h"
#include "router.h"
#include "ruledef.h"
#include "pattern.h"
#include "generate.h"

#if DEFGLOBAL_CONSTRUCT
#include "globlpsr.h"
#endif

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER  
   static VOID                    ExtractAnds(struct lhsParseNode *,int,
                                              struct expr **,struct expr **);
   static VOID                    ExtractFieldTest(struct lhsParseNode *,int,
                                                   struct expr **,struct expr **);
   static struct expr            *GetfieldReplace(struct lhsParseNode *);
   static struct expr            *GenPNConstant(struct lhsParseNode *);
   static struct expr            *GenJNConstant(struct lhsParseNode *);
   static struct expr            *GenJNColon(struct lhsParseNode *);
   static struct expr            *GenPNColon(struct lhsParseNode *);
   static struct expr            *GenJNEq(struct lhsParseNode *);
   static struct expr            *GenPNEq(struct lhsParseNode *);
   static struct expr            *GenJNVariableComparison(struct lhsParseNode *,
                                                          struct lhsParseNode *);
   static struct expr            *GenPNVariableComparison(struct lhsParseNode *,
                                                          struct lhsParseNode *);
   static int                     AllVariablesInPattern(struct lhsParseNode *,
                                                        int);
   static int                     AllVariablesInExpression(struct lhsParseNode *,
                                                           int);
#else
   static VOID                    ExtractAnds();
   static VOID                    ExtractFieldTest();
   static struct expr            *GetfieldReplace();
   static struct expr            *GenPNConstant();
   static struct expr            *GenJNConstant();
   static struct expr            *GenJNColon();
   static struct expr            *GenPNColon();
   static struct expr            *GenJNEq();
   static struct expr            *GenPNEq();
   static struct expr            *GenJNVariableComparison();
   static struct expr            *GenPNVariableComparison();
   static int                     AllVariablesInPattern();
   static int                     AllVariablesInExpression();
#endif

/******************************************************************/
/* FieldConversion: Generates join and pattern tests for a field. */
/******************************************************************/
globle VOID FieldConversion(theField,thePattern)
  struct lhsParseNode *theField;
  struct lhsParseNode *thePattern;
  {
   int testInPatternNetwork = CLIPS_TRUE;
   struct lhsParseNode *patternPtr;
   struct expr *headOfPNExpression, *headOfJNExpression;
   struct expr *lastPNExpression, *lastJNExpression;
   struct expr *tempExpression;
   struct expr *patternNetTest = NULL;
   struct expr *joinNetTest = NULL;

   if (theField == NULL)
     {
      CLIPSSystemError("ANALYSIS",3);
      ExitCLIPS(4);
     }
        
   /*=================================================*/
   /* Determine if constant testing must be performed */
   /* in the join network. Only possible when a field */
   /* contains an or ('|') and references are made to */
   /* variables outside the pattern.                  */
   /*=================================================*/

   if (theField->bottom != NULL)
     {
      if (theField->bottom->bottom != NULL)
        { testInPatternNetwork = AllVariablesInPattern(theField->bottom,theField->pattern); }
     }

   /*======================================*/
   /* Extract pattern network expressions. */
   /*======================================*/

   headOfPNExpression = lastPNExpression = NULL;
   headOfJNExpression = lastJNExpression = NULL;

   patternPtr = theField->bottom;
   while (patternPtr != NULL)
     {
      ExtractAnds(patternPtr,testInPatternNetwork,&patternNetTest,&joinNetTest);

      if (patternNetTest != NULL)
        {
         if (lastPNExpression == NULL)
           { headOfPNExpression = patternNetTest; }
         else
           { lastPNExpression->nextArg = patternNetTest; }
         lastPNExpression = patternNetTest;
        }

      if (joinNetTest != NULL)
        {
         if (lastJNExpression == NULL)
           { headOfJNExpression = joinNetTest; }
         else
           { lastJNExpression->nextArg = joinNetTest; }
         lastJNExpression = joinNetTest;
        }

      patternPtr = patternPtr->bottom;
     }

   if ((headOfPNExpression != NULL) ? (headOfPNExpression->nextArg != NULL) : CLIPS_FALSE)
     {
      tempExpression = GenConstant(FCALL,PTR_OR);
      tempExpression->argList = headOfPNExpression;
      headOfPNExpression = tempExpression;
     }

   if ((headOfJNExpression != NULL) ? (headOfJNExpression->nextArg != NULL) : CLIPS_FALSE)
     {
      tempExpression = GenConstant(FCALL,PTR_OR);
      tempExpression->argList = headOfJNExpression;
      headOfJNExpression = tempExpression;
     }

   /*=============================================*/
   /* Attach expression for the binding instance. */
   /*=============================================*/

   if ((theField->type == MF_VARIABLE) || (theField->type == SF_VARIABLE))
     {
      if ((theField->referringNode != NULL) ?
          (theField->referringNode->pattern == theField->pattern) :
          CLIPS_FALSE)
        {
         tempExpression = GenPNVariableComparison(theField,theField->referringNode);
         headOfPNExpression = CombineExpressions(tempExpression,headOfPNExpression);
        }
      else if ((theField->referringNode != NULL) ?
               (theField->referringNode->pattern > 0) :
               CLIPS_FALSE)
        {
         tempExpression = GenJNVariableComparison(theField,theField->referringNode);
         headOfJNExpression = CombineExpressions(tempExpression,headOfJNExpression);
        }
     }
   
   theField->networkTest = headOfPNExpression;
   thePattern->networkTest = CombineExpressions(thePattern->networkTest,headOfJNExpression);
  }

/****************************************************************************/
/* ExtractAnds: Loops through a single set of subfields bound together by   */
/*   an & connective constraint in a field and generates expressions needed */
/*   for testing conditions in the pattern and join network.                */
/****************************************************************************/
static VOID ExtractAnds(andField,testInPatternNetwork,patternNetTest,joinNetTest)
  struct lhsParseNode *andField;
  int testInPatternNetwork;
  struct expr **patternNetTest;
  struct expr **joinNetTest;
  {
   struct expr *newPNTest, *newJNTest;

   /*=========================================================*/
   /* Loop through the list of field constraints and generate */
   /* test for those that can have pattern network tests.     */
   /*=========================================================*/

   *patternNetTest = NULL;
   *joinNetTest = NULL;
   
   while (andField != NULL)
     {
      ExtractFieldTest(andField,testInPatternNetwork,&newPNTest,&newJNTest);
                       
      *patternNetTest = CombineExpressions(*patternNetTest,newPNTest);
      *joinNetTest = CombineExpressions(*joinNetTest,newJNTest);
      
      andField = andField->right;
     }
  }

/****************************************************************************/
/* ExtractFieldTest:                 */
/****************************************************************************/
static VOID ExtractFieldTest(theField,testInPatternNetwork,patternNetTest,joinNetTest)
  struct lhsParseNode *theField;
  int testInPatternNetwork;
  struct expr **patternNetTest;
  struct expr **joinNetTest;
  {   
   *patternNetTest = NULL;
   *joinNetTest = NULL;
   
   if ((theField->type == STRING) || (theField->type == SYMBOL) ||
#if OBJECT_SYSTEM
       (theField->type == INSTANCE_NAME) ||
#endif
       (theField->type == FLOAT) || (theField->type == INTEGER))
     {
      if (testInPatternNetwork == CLIPS_TRUE) 
        { *patternNetTest = GenPNConstant(theField); }
      else
        { *joinNetTest = GenJNConstant(theField); }
     }
   else if (theField->type == PREDICATE_CONSTRAINT)
     {
      if ((testInPatternNetwork == CLIPS_TRUE) &&
          (AllVariablesInExpression(theField->expression,theField->pattern) == CLIPS_TRUE))
        { *patternNetTest = GenPNColon(theField); }
      else
        { *joinNetTest = GenJNColon(theField); }
     }
   else if (theField->type == RETURN_VALUE_CONSTRAINT)
     {
      if ((testInPatternNetwork == CLIPS_TRUE) &&
          (AllVariablesInExpression(theField->expression,theField->pattern) == CLIPS_TRUE))
        { *patternNetTest = GenPNEq(theField); }
      else
        { *joinNetTest = GenJNEq(theField); }
     }
   else if ((theField->type == SF_VARIABLE) || (theField->type == MF_VARIABLE))
     {
      if ((testInPatternNetwork == CLIPS_TRUE) && 
          ((theField->referringNode != NULL) ?
           (theField->referringNode->pattern == theField->pattern) :
           CLIPS_FALSE))
        { *patternNetTest = GenPNVariableComparison(theField,theField->referringNode); }
      else
        { *joinNetTest = GenJNVariableComparison(theField,theField->referringNode); }
     }
  }
  
/*********************************************************/
/* GenPNConstant: Generates an expression for use in the */
/*  pattern network of a data entity (such as a fact or  */
/*  instance). The expression generated is for comparing */
/*  a constant value against a specified slot/field in   */
/*  the data entity for equality or inequality.          */
/*********************************************************/
static struct expr *GenPNConstant(theField)
  struct lhsParseNode *theField;
  {
   struct expr *top;
   
   /*===============================================*/
   /* If the pattern parser is capable of creating  */
   /* a specialized test, then call the function to */
   /* generate the pattern network test and return  */
   /* the expression generated.                     */
   /*===============================================*/
 
   if (theField->patternType->genPNConstantFunction != NULL)
     { return (*theField->patternType->genPNConstantFunction)(theField); }

   /*===================================================*/
   /* Otherwise, generate a test which uses the eq/neq  */
   /* function to compare the pattern field/slot to the */
   /* constant and then return the expression.          */
   /*===================================================*/
   
   if (theField->negated)
     { top = GenConstant(FCALL,PTR_NEQ); }
   else
     { top = GenConstant(FCALL,PTR_EQ); }

   top->argList = (*theField->patternType->genGetPNValueFunction)(theField);
   top->argList->nextArg = GenConstant(theField->type,theField->value);

   return(top);
  }

/*******************************************************/
/* GenJNConstant: Generates a constant test expression */
/*   for use in the join network.                      */
/*   Convert                                           */
/*      value                                          */
/*   To                                                */
/*      (eq (getvar <pattern> <field>) value)          */
/*   Convert                                           */
/*      ~value                                         */
/*   To                                                */
/*      (neq (getvar <pattern> <field>) value)         */
/*******************************************************/
static struct expr *GenJNConstant(theField)
  struct lhsParseNode *theField;
  {
   struct expr *top;

   /*===============================================*/
   /* If the pattern parser is capable of creating  */
   /* a specialized test, then call the function to */
   /* generate the join network test.               */
   /*===============================================*/
 
   if (theField->patternType->genJNConstantFunction != NULL)
     { 
      return (*theField->patternType->genJNConstantFunction)(theField);
     }

   if (theField->negated)
     { top = GenConstant(FCALL,PTR_NEQ); }
   else
     { top = GenConstant(FCALL,PTR_EQ); }

   /*===========================================================*/
   /* Otherwise, generate a test which uses the eq/neq function */
   /* to compare the pattern field/slot to the constant.        */
   /*===========================================================*/
   
   top->argList = (*theField->patternType->genGetJNValueFunction)(theField);
   top->argList->nextArg = GenConstant(theField->type,theField->value);

   return(top);
  }

/********************************************************/
/* GenJNColon: Generates a test expression for use in   */
/*   the join network for the such that operator.       */
/*   Convert                                            */
/*      :(expression)                                   */
/*   To                                                 */
/*      (expression)                                    */
/*   Convert                                            */
/*      ~:(expression)                                  */
/*   To                                                 */
/*      (! (expresion))                                 */
/*   The variables in the expressions are replaced with */
/*   getvar calls.                                      */
/********************************************************/
static struct expr *GenJNColon(theField)
  struct lhsParseNode *theField;
  {
   struct expr *top, *conversion;

   conversion = GetvarReplace(theField->expression);
   
   if (theField->negated)
     {
      top = GenConstant(FCALL,PTR_NOT);
      top->argList = conversion;
     }
   else
     { top = conversion; }

   return(top);
  }

/********************************************************/
/* GenPNColon: Generates a test expression for use in   */
/*   the pattern network for the such that operator.    */
/*   Convert                                            */
/*      :(expression)                                   */
/*   To                                                 */
/*      (expression)                                    */
/*   Convert                                            */
/*      ~:(expression)                                  */
/*   To                                                 */
/*      (! (expresion))                                 */
/*   The variables in the expressions are replaced with */
/*   getfield calls.                                    */
/********************************************************/
static struct expr *GenPNColon(theField)
  struct lhsParseNode *theField;
  {
   struct expr *top, *conversion;

   conversion = GetfieldReplace(theField->expression);
   
   if (theField->negated)
     {
      top = GenConstant(FCALL,PTR_NOT);
      top->argList = conversion;
     }
   else
     { top = conversion; }

   return(top);
  }

/********************************************************/
/* GenJNEq: Generates a test expression for use in      */
/*   the join network for the = operator.               */
/*   Convert                                            */
/*      =(expression)                                   */
/*   To                                                 */
/*      (eq (getvar <pattern> <field>) (expression))    */
/*   Convert                                            */
/*      ~=(expression)                                  */
/*   To                                                 */
/*      (neq (getvar <pattern> <field>) (expression))   */
/*   The variables in the expressions are replaced with */
/*   getvar calls.                                      */
/********************************************************/
static struct expr *GenJNEq(theField)
  struct lhsParseNode *theField;
  {
   struct expr *top, *conversion;

   conversion = GetvarReplace(theField->expression);

   if (theField->negated)
     { top = GenConstant(FCALL,PTR_NEQ); }
   else
     { top = GenConstant(FCALL,PTR_EQ); }

   top->argList = (*theField->patternType->genGetJNValueFunction)(theField);
   top->argList->nextArg = conversion;

   return(top);
  }

/********************************************************/
/* GenPNEq: Generates a test expression for use in      */
/*   the join network for the = operator.               */
/*   Convert                                            */
/*      =(expression)                                   */
/*   To                                                 */
/*      (eq (getfield <field>) (expression))            */
/*   Convert                                            */
/*      ~=(expression)                                  */
/*   To                                                 */
/*      (neq (getfield <field>) (expression))           */
/*   The variables in the expressions are replaced with */
/*   getfield calls.                                    */
/********************************************************/
static struct expr *GenPNEq(theField)
  struct lhsParseNode *theField;
  {
   struct expr *top, *conversion;

   conversion = GetfieldReplace(theField->expression);

   if (theField->negated)
     { top = GenConstant(FCALL,PTR_NEQ); }
   else
     { top = GenConstant(FCALL,PTR_EQ); }

   top->argList = (*theField->patternType->genGetPNValueFunction)(theField);
   top->argList->nextArg = conversion;

   return(top);
  }

/***************************************************/
/* GetvarReplace: Replaces occurences of variables */
/*   in expressions with get_var calls.            */
/***************************************************/
globle struct expr *GetvarReplace(nodeList)
  struct lhsParseNode *nodeList;
  {
   struct expr *newList;
   
   if (nodeList == NULL) return(NULL);
     
   newList = get_struct(expr);
   newList->type = nodeList->type;
   newList->value = nodeList->value;
   newList->nextArg = GetvarReplace(nodeList->right);
   newList->argList = GetvarReplace(nodeList->bottom);

   if ((nodeList->type == SF_VARIABLE) || (nodeList->type == MF_VARIABLE))
     {
        
      (*nodeList->referringNode->patternType->replaceGetJNValueFunction)
        (newList,nodeList->referringNode);
     }
#if DEFGLOBAL_CONSTRUCT
   else if (newList->type == GBL_VARIABLE)
     { ReplaceGlobalVariable(newList); }
#endif
     
   return(newList);  
  }

/***************************************************/
/* GetfieldReplace: Replaces occurences of variables */
/*   in expressions with get_field calls.            */
/***************************************************/
static struct expr *GetfieldReplace(nodeList)
  struct lhsParseNode *nodeList;
  {
   struct expr *newList;
   
   if (nodeList == NULL)
     { return(NULL); }
     
   newList = get_struct(expr);
   newList->type = nodeList->type;
   newList->value = nodeList->value;
   newList->nextArg = GetfieldReplace(nodeList->right);
   newList->argList = GetfieldReplace(nodeList->bottom);

   if ((nodeList->type == SF_VARIABLE) || (nodeList->type == MF_VARIABLE))
     {
      (*nodeList->referringNode->patternType->replaceGetPNValueFunction)
         (newList,nodeList->referringNode);
     }
#if DEFGLOBAL_CONSTRUCT
   else if (newList->type == GBL_VARIABLE)
     { ReplaceGlobalVariable(newList); }
#endif
     
   return(newList);  
  }

/**************************************************************/
/* GenJNVariableComparison: Generates a join network test for */
/*   comparing two variables found in different patterns.     */
/**************************************************************/
static struct expr *GenJNVariableComparison(selfNode,referringNode)
  struct lhsParseNode *selfNode, *referringNode;
  {
   struct expr *top;
   
   /*========================================================*/
   /* If either pattern is missing a function for generating */
   /* the appropriate test, then no test is generated.       */ 
   /*========================================================*/
            
   if ((selfNode->patternType->genCompareJNValuesFunction == NULL) ||
       (referringNode->patternType->genCompareJNValuesFunction == NULL))
     { return(NULL); }
     
   /*=====================================================*/
   /* If both patterns are of the same type, then use the */
   /* special function for generating the join test.      */
   /*=====================================================*/
   
   if (selfNode->patternType->genCompareJNValuesFunction ==  
       referringNode->patternType->genCompareJNValuesFunction)
       
     { 
      return (*selfNode->patternType->genCompareJNValuesFunction)(selfNode,
                                                                  referringNode);
     }
     
   /*===========================================================*/
   /* If the patterns are of different types, then generate a   */
   /* join test by using the eq/neq function with its arguments */
   /* being function calls to retrieve the appropriate values   */
   /* from the patterns.                                        */
   /*===========================================================*/
   
   if (selfNode->negated) top = GenConstant(FCALL,PTR_NEQ);
   else top = GenConstant(FCALL,PTR_EQ);

   top->argList = (*selfNode->patternType->genGetJNValueFunction)(selfNode);
   top->argList->nextArg = (*referringNode->patternType->genGetJNValueFunction)(referringNode);

   return(top);
  }

/*************************************************************/
/* GenPNVariableComparison: Generates a pattern network test */
/*   for comparing two variables found in the same pattern.  */
/*************************************************************/
static struct expr *GenPNVariableComparison(selfNode,referringNode)
  struct lhsParseNode *selfNode, *referringNode;
  {   
   if (selfNode->patternType->genComparePNValuesFunction != NULL)
     { 
      return (*selfNode->patternType->genComparePNValuesFunction)(selfNode,referringNode);
     }
     
   return(NULL);
  }

/****************************************************************/
/* AllVariablesInPattern: Determines if all of the variable references */
/*   in a field can be referenced within the pattern.           */
/****************************************************************/
static int AllVariablesInPattern(orField,pattern)
  struct lhsParseNode *orField;
  int pattern;
  {
   struct lhsParseNode *andField;

   while (orField != NULL)
     {
      andField = orField;
      while (andField != NULL)
        {
         if ((andField->type == SF_VARIABLE) || (andField->type == MF_VARIABLE))
           { if (andField->referringNode->pattern != pattern) return(CLIPS_FALSE); }
         else if ((andField->type == PREDICATE_CONSTRAINT) ||
                  (andField->type == RETURN_VALUE_CONSTRAINT))
           {
            if (AllVariablesInExpression(andField->expression,pattern) == CLIPS_FALSE)
              { return(CLIPS_FALSE); }
           }

         andField = andField->right;
        }
      orField = orField->bottom;
     }

   return(CLIPS_TRUE);
  }
  
/**************************************************************************/
/* AllVariablesInExpression: Determines if all of the variable references */
/*   in an expression can be referenced within the pattern.               */
/**************************************************************************/
static int AllVariablesInExpression(theExpression,pattern)
  struct lhsParseNode *theExpression;
  int pattern;
  {
   while (theExpression != NULL)
     {
      if ((theExpression->type == SF_VARIABLE) || 
          (theExpression->type == MF_VARIABLE))
        { if (theExpression->referringNode->pattern != pattern) return(CLIPS_FALSE); }
        
      if (AllVariablesInExpression(theExpression->bottom,pattern) == CLIPS_FALSE)
        { return(CLIPS_FALSE); }
        
      theExpression = theExpression->right;
     }

   return(CLIPS_TRUE);
  }

#endif


