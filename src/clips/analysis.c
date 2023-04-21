   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                  ANALYSIS MODULE                    */
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

#define _ANALYSIS_SOURCE_

#include "setup.h"

#if (! RUN_TIME) && (! BLOAD_ONLY) && DEFRULE_CONSTRUCT

#include <stdio.h>
#define _CLIPS_STDIO_

#include "constant.h"
#include "symbol.h"
#include "clipsmem.h"
#include "exprnpsr.h"
#include "reorder.h"
#include "generate.h"
#include "pattern.h"
#include "router.h"
#include "ruledef.h"
#include "cstrnchk.h"
#include "cstrnutl.h"
#include "cstrnops.h"
#include "rulecstr.h"
#include "analysis.h"

#if DEFGLOBAL_CONSTRUCT
#include "globldef.h"
#endif

#define ANALYSIS_SWITCH 0

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/
 
#if ANSI_COMPILER
   static int                     GetVariables(struct lhsParseNode *);
   static BOOLEAN                 UnboundVariablesInPattern(struct lhsParseNode *,int);
   static int                     PropagateVariableToNodes(struct lhsParseNode *,
                                                           int,
                                                           struct symbolHashNode *,
                                                           struct lhsParseNode *,
                                                           int,int,int);
   static struct lhsParseNode    *CheckExpression(struct lhsParseNode *,
                                                  struct lhsParseNode *,
                                                  int,
                                                  struct symbolHashNode *,
                                                  int);
   static VOID                    VariableReferenceErrorMessage(struct symbolHashNode *,
                                                                struct lhsParseNode *,
                                                                int,
                                                                struct symbolHashNode *,
                                                                int);
   static int                     ProcessField(struct lhsParseNode *,
                                               struct lhsParseNode *,
                                               struct lhsParseNode *);
   static int                     ProcessVariable(struct lhsParseNode *,
                                               struct lhsParseNode *,
                                               struct lhsParseNode *);
   static VOID                    VariableMixingErrorMessage(struct symbolHashNode *);
   static int                     PropogateVariableDriver(struct lhsParseNode *,
                                                          struct lhsParseNode *,
                                                          struct lhsParseNode *,
                                                          int,struct symbolHashNode *,
                                                          struct lhsParseNode *,
                                                          int);
#else
   static int                     GetVariables();
   static BOOLEAN                 UnboundVariablesInPattern();
   static int                     PropagateVariableToNodes();
   static struct lhsParseNode    *CheckExpression();
   static VOID                    VariableReferenceErrorMessage();
   static int                     ProcessField();
   static int                     ProcessVariable();
   static VOID                    VariableMixingErrorMessage();
   static int                     PropogateVariableDriver();
#endif
  
/******************************************************************/
/* VariableAnalysis: Propagates variables references to other     */
/*   variables in the LHS and determines if there are any illegal */
/*   variable references (e.g. referring to an unbound variable). */
/******************************************************************/
globle int VariableAnalysis(patternPtr)
  struct lhsParseNode *patternPtr;
  {
   struct lhsParseNode *rv, *theList, *tempList;
   int errorFlag = CLIPS_FALSE;

   /*======================================================*/
   /* Loop through all of the CEs in the rule to determine */
   /* which variables refer to other variables and whether */
   /* any semantic errors exist when refering to variables */
   /* (such as referring to a variable that was not        */
   /* previously bound.                                    */
   /*======================================================*/

   while (patternPtr != NULL)
     {
      /*=========================================================*/
      /* If a pattern CE is encountered, propogate any variables */
      /* found in the pattern and note any illegal references to */
      /* other variables.                                        */
      /*=========================================================*/
      
      if (patternPtr->type == PATTERN_CE)
        {         
         /*====================================================*/
         /* Determine if the fact address associated with this */
         /* pattern illegally refers to other variables.       */
         /*====================================================*/
         
         if ((patternPtr->value != NULL) && 
             (patternPtr->referringNode != NULL))
           {
            errorFlag = CLIPS_TRUE;
            if (patternPtr->referringNode->index == -1)
              {
               PrintErrorID("ANALYSIS",1,CLIPS_TRUE);
               PrintCLIPS(WERROR,"Duplicate pattern-address ?");
               PrintCLIPS(WERROR,ValueToString(patternPtr->value));
               PrintCLIPS(WERROR," found in CE #");
               PrintLongInteger(WERROR,(long) patternPtr->whichCE);
               PrintCLIPS(WERROR,".\n");
              }
            else
              {
               PrintErrorID("ANALYSIS",2,CLIPS_TRUE);
               PrintCLIPS(WERROR,"Pattern-address ?");
               PrintCLIPS(WERROR,ValueToString(patternPtr->value));
               PrintCLIPS(WERROR," used in CE #");
               PrintLongInteger(WERROR,(long) patternPtr->whichCE);
               PrintCLIPS(WERROR," was previously bound within a pattern CE.\n");
              }
           }
         
         /*====================================================*/
         /* Propogate the pattern and field location of bound  */
         /* variables found in this pattern to other variables */
         /* in the same semantic scope as the bound variable.  */
         /*====================================================*/
         
         if (GetVariables(patternPtr)) return(CLIPS_TRUE); 
        }
        
      /*==============================================================*/
      /* If a test CE is encountered, make sure that all references   */
      /* to variables have been previously bound. If they are bound   */
      /* then replace the references to variables with function calls */
      /* to retrieve the variables.                                   */
      /*==============================================================*/
       
      else if (patternPtr->type == TEST_CE)
        { 
         rv = CheckExpression(patternPtr->expression,NULL,(int) patternPtr->whichCE,NULL,0); 
         
         theList = GetExpressionVarConstraints(patternPtr->expression);
         for (tempList = theList; tempList != NULL; tempList = tempList->right)
            {
             if (PropogateVariableDriver(patternPtr,patternPtr,NULL,SF_VARIABLE,
                                  tempList->value,tempList,CLIPS_FALSE))
               { 
                ReturnLHSParseNodes(theList);
                return(CLIPS_TRUE);
               }
            }
     
         ReturnLHSParseNodes(theList);
         
         if (rv != NULL) errorFlag = CLIPS_TRUE;
         else
           { patternPtr->networkTest = GetvarReplace(patternPtr->expression); }
        }

      /*=====================================================*/
      /* Move on to the next pattern in the LHS of the rule. */
      /*=====================================================*/
      
      patternPtr = patternPtr->bottom;
     }
     
   /*==========================================*/
   /* Return the error status of the analysis. */
   /*==========================================*/
   
   return(errorFlag);
  }
  
/****************************************************************/
/* GetVariables: Loops through each field/slot within a pattern */
/*   and propogates the pattern and field location of bound     */
/*   variables found in the pattern to other variables within   */
/*   the same semantic scope as the bound variables.            */
/****************************************************************/
static int GetVariables(thePattern)
  struct lhsParseNode *thePattern;
  {
   struct lhsParseNode *patternHead;
   struct lhsParseNode *multifieldHeader = NULL;
     
   /*======================================================*/
   /* Loop through all the fields/slots found in a pattern */
   /* looking for binding instances of variables.          */
   /*======================================================*/
   
   patternHead = thePattern;
  
   while (thePattern != NULL)
     {
      if (thePattern->multifieldSlot)
        {
         multifieldHeader = thePattern;
         thePattern = thePattern->bottom;
        }
        
      /*==================================================*/
      /* Propogate the binding occurences of single field */
      /* variables, multifield variables, and fact        */
      /* addresses to other occurences of the variable.   */
      /*==================================================*/
        
      if (thePattern != NULL)
        {
         if ((thePattern->type == SF_VARIABLE) ||
             (thePattern->type == MF_VARIABLE) ||
             ((thePattern->type == PATTERN_CE) && (thePattern->value != NULL)))
           {
            if (ProcessVariable(thePattern,multifieldHeader,patternHead))
              { return(CLIPS_TRUE); }
           }
         else
           {   
            if (ProcessField(thePattern,multifieldHeader,patternHead))
              { return(CLIPS_TRUE); }
           }
        }
        
      /*================================================*/
      /* Move on to the next field/slot in the pattern. */
      /*================================================*/
      
      if (thePattern == NULL) 
        { thePattern = multifieldHeader; }
      else if ((thePattern->right == NULL) && (multifieldHeader != NULL))
        {
         thePattern = multifieldHeader;
         multifieldHeader = NULL;
        }
        
      thePattern = thePattern->right;
     }
     
   return(CLIPS_FALSE);
  }
  
/*************************************************************/
/* ProcessVariable:      */
/*************************************************************/
static int ProcessVariable(thePattern,multifieldHeader,patternHead)
  struct lhsParseNode *thePattern, *multifieldHeader, *patternHead;
  {
   int startDepth, theType;
   struct symbolHashNode *theVariable;
   struct constraintRecord *theConstraints;
   
   startDepth = patternHead->beginNandDepth;
    
   /*====================================================*/
   /* Determine the name of the binding variable and the */
   /* index associated with it (-1 for fact addresses,   */
   /* otherwise the self index stored in the variable.   */
   /*====================================================*/
   
   if (thePattern->type == PATTERN_CE) 
     {
      theType = SF_VARIABLE;
      theVariable = (struct symbolHashNode *) thePattern->value;
      if (thePattern->derivedConstraints) RemoveConstraint(thePattern->constraints);
      theConstraints = GetConstraintRecord();
      thePattern->constraints = theConstraints;
      thePattern->constraints->anyAllowed = CLIPS_FALSE;
      thePattern->constraints->instanceAddressesAllowed = CLIPS_TRUE;
      thePattern->constraints->factAddressesAllowed = CLIPS_TRUE;
      thePattern->derivedConstraints = CLIPS_TRUE;
     }
   else 
     {
      theType = thePattern->type;
      theVariable = (struct symbolHashNode *) thePattern->value;
     }
       
   /*===================================================*/
   /* Propagate the variable location to any additional */
   /* constraints associated with the binding variable. */
   /*===================================================*/
   
   if (thePattern->type != PATTERN_CE)
     {
      PropagateVariableToNodes(thePattern->bottom,theType,theVariable,
                               thePattern,startDepth,CLIPS_TRUE,CLIPS_FALSE);  
                               
      if (ProcessField(thePattern,multifieldHeader,patternHead))
        { return(CLIPS_TRUE); }
     }
   
   /*================================================================*/
   /* Propagate the constraints to other fields, slots and patterns. */
   /*================================================================*/
   
   return(PropogateVariableDriver(patternHead,thePattern,multifieldHeader,theType,
                                  theVariable,thePattern,CLIPS_TRUE));
  }
  
/*************************************************************/
/* PropogateVariableDriver:      */
/*************************************************************/
static int PropogateVariableDriver(patternHead,theNode,multifieldHeader,
                                   theType,variableName,theReference,assignReference)
  struct lhsParseNode *patternHead, *theNode, *multifieldHeader;
  int theType;
  struct symbolHashNode *variableName;
  struct lhsParseNode *theReference;
  int assignReference;
  {
   /*===================================================*/
   /* Propogate the variable location to any additional */
   /* constraints associated with the binding variable. */
   /*===================================================*/
   
   if (multifieldHeader != NULL)
     {
      if (PropagateVariableToNodes(multifieldHeader->right,theType,variableName,
                                   theReference,patternHead->beginNandDepth,assignReference,CLIPS_FALSE))
        {
         VariableMixingErrorMessage(variableName);
         return(CLIPS_TRUE);
        }
     }
             
   /*========================================================*/
   /* Propogate the variable location to fields/slots in the */
   /* same pattern which appear after the binding variable.  */
   /*========================================================*/
   
   if (PropagateVariableToNodes(theNode->right,theType,variableName,theReference,
                                patternHead->beginNandDepth,assignReference,CLIPS_FALSE))
     {
      VariableMixingErrorMessage(variableName);
      return(CLIPS_TRUE);
     }
   
   /*======================================================*/
   /* Propogate values to other patterns if the pattern in */
   /* which the variable is found is not a "not" CE or the */
   /* last pattern within a nand CE.           */
   /*======================================================*/
   
   if (((patternHead->type == PATTERN_CE) || (patternHead->type == TEST_CE)) && 
       (patternHead->negated == CLIPS_FALSE) &&
       (patternHead->beginNandDepth <= patternHead->endNandDepth))
     {
      if (PropagateVariableToNodes(patternHead->bottom,theType,variableName,theReference,
                                   patternHead->beginNandDepth,assignReference,CLIPS_FALSE))
       {
         VariableMixingErrorMessage(variableName);
         return(CLIPS_TRUE);
        }
     }
     
   return(CLIPS_FALSE);
  }
   
/*************************************************************/
/* ProcessField:      */
/*************************************************************/
static int ProcessField(thePattern,multifieldHeader,patternHead)
  struct lhsParseNode *thePattern, *multifieldHeader, *patternHead;
  {
   struct lhsParseNode *theList, *tempList;
   
   if (thePattern->type == PATTERN_CE) return(CLIPS_FALSE);

   theList = DeriveVariableConstraints(thePattern);
   for (tempList = theList; tempList != NULL; tempList = tempList->right)
     {
      if (PropogateVariableDriver(patternHead,thePattern,multifieldHeader,tempList->type,
                                  tempList->value,tempList,CLIPS_FALSE))
        { 
         ReturnLHSParseNodes(theList);
         return(CLIPS_TRUE);
        }
     }
     
   ReturnLHSParseNodes(theList);

   /*===================================================================*/
   /* Check for "variable referenced, but not previously bound" errors. */
   /*===================================================================*/
          
   if (UnboundVariablesInPattern(thePattern,(int) patternHead->whichCE)) return(CLIPS_TRUE);
         
   /*==============================*/
   /* Check for constraint errors. */
   /*==============================*/
    
   if (ProcessConnectedConstraints(thePattern,multifieldHeader,patternHead)) return(CLIPS_TRUE); 

   FieldConversion(thePattern,patternHead); 
   
   return(CLIPS_FALSE);
  }
  
/*************************************************************/
/* PropagateVariableToNodes: Propogates variable references  */
/*  to all other variables within the semantic scope of the  */
/*  bound variable. That is, a variable reference cannot be  */
/*  beyond an enclosing not/and CE combination. The          */
/*  restriction of propogating variables beyond an enclosing */
/*  not CE is handled within the GetVariables function.      */
/*************************************************************/
static int PropagateVariableToNodes(theNode,theType,variableName,
                                    theReference,startDepth,
                                    assignReference,ignoreVariableTypes)
  struct lhsParseNode *theNode;
  int theType;
  struct symbolHashNode *variableName;
  struct lhsParseNode *theReference;
  int startDepth;
  int assignReference;
  int ignoreVariableTypes;
  {
   struct constraintRecord *tempConstraints;
   
   while (theNode != NULL)
     {
      /*==================================================*/
      /* If the field/slot contains a predicate or return */
      /* value constraint, then propogate the variable to */
      /* the expression associated with that constraint.  */
      /*==================================================*/
      
      if (theNode->expression != NULL)
        { 
         PropagateVariableToNodes(theNode->expression,theType,variableName,
                                  theReference,startDepth,assignReference,CLIPS_TRUE);
        }
        
      /*======================================================*/
      /* If the field/slot is a single or multifield variable */
      /* with the same name as the propogated variable,       */
      /* then propogate the variable location to this node.   */
      /*======================================================*/
      
      else if (((theNode->type == SF_VARIABLE) || (theNode->type == MF_VARIABLE)) &&
               (theNode->value == (VOID *) variableName))
        {
         if ((theReference->constraints != NULL) && (! theNode->negated))
           {
            tempConstraints = theNode->constraints;
            theNode->constraints = IntersectConstraints(theReference->constraints,
                                                        tempConstraints);
            if (theNode->derivedConstraints)
              { RemoveConstraint(tempConstraints); }
             
            theNode->derivedConstraints = CLIPS_TRUE;
           }
         
         /*======================================================*/
         /* Check for mixing of single and multifield variables. */
         /*======================================================*/
         
         if (ignoreVariableTypes == CLIPS_FALSE)
           {
            if (((theType == SF_VARIABLE) && (theNode->type == MF_VARIABLE)) ||
                ((theType == MF_VARIABLE) && (theNode->type == SF_VARIABLE)))
              { return(CLIPS_TRUE); }
           }
            
         /*=====================================================*/
         /* Don't propogate the variable if it originates from  */
         /* a different type of pattern object and the variable */
         /* reference has already been resolved.                */
         /*=====================================================*/
         
         if (assignReference)
           {
            if (theNode->referringNode == NULL) 
              { theNode->referringNode = theReference; }
            else if (theReference->pattern == theNode->pattern)
              { theNode->referringNode = theReference; }
            else if (theReference->patternType == theNode->patternType)
              { theNode->referringNode = theReference; }
           }
        }
        
      /*========================================================*/
      /* If the field/slot is the node representing the entire  */
      /* pattern, then propogate the variable location to the   */
      /* fact address associated with the pattern (if it is the */
      /* same variable name).                                   */
      /*========================================================*/
      
      else if ((theNode->type == PATTERN_CE) && 
               (theNode->value == (VOID *) variableName) &&
               (assignReference == CLIPS_TRUE))
        {
         if (theType == MF_VARIABLE) return(CLIPS_TRUE);
         
         theNode->referringNode = theReference;
        }
        
      /*=====================================================*/
      /* Propogate the variable to other fields contained    */
      /* within the same & field constraint or same pattern. */
      /*=====================================================*/
       
      if (theNode->right != NULL) 
        {
         if (PropagateVariableToNodes(theNode->right,theType,variableName,
                                      theReference,startDepth,assignReference,ignoreVariableTypes))
           { return(CLIPS_TRUE); }
        }
                                  
      /*============================================================*/
      /* Propogate the variable to other patterns within the same   */
      /* semantic scope (if dealing with the node for an entire     */
      /* pattern) or to the next | field constraint within a field. */
      /*============================================================*/
      
      if (theNode->type == PATTERN_CE)
        {
         if (theNode->endNandDepth < startDepth) theNode = NULL;
         else theNode = theNode->bottom;
        }
      else
        { theNode = theNode->bottom; }
     }
     
   return(CLIPS_FALSE);
  }

/********************************************************************/
/* UnboundVariablesInPattern: Verifies that variables within a field have been */
/*   referenced properly (i.e. that variables have been previously  */
/*   bound if they are not a binding occurrence).                   */
/********************************************************************/
static BOOLEAN UnboundVariablesInPattern(theSlot,pattern)
  struct lhsParseNode *theSlot;
  int pattern;
  {
   struct lhsParseNode *andField;
   struct lhsParseNode *rv;
   int result;
   struct lhsParseNode *orField;
   struct symbolHashNode *slotName;
   CONSTRAINT_RECORD *theConstraints;
   int theField;

   /*===================================================*/
   /* If a multifield slot is being checked, then check */
   /* each of the fields grouped with the multifield.   */
   /*===================================================*/
   
   if (theSlot->multifieldSlot)
     {
      theSlot = theSlot->bottom;
      while (theSlot != NULL)
        {
         if (UnboundVariablesInPattern(theSlot,pattern))
           { return(CLIPS_TRUE); }
         theSlot = theSlot->right;
        }
        
      return(CLIPS_FALSE);
     }
    
   slotName = theSlot->slot;
   theField = theSlot->index;
   theConstraints = theSlot->constraints;
   orField = theSlot->bottom;
   
   while (orField != NULL)
     {
      andField = orField;
      while (andField != NULL)
        {
         if (((andField->type == SF_VARIABLE) || (andField->type == MF_VARIABLE)) &&
             (andField->referringNode == NULL))
           {
            VariableReferenceErrorMessage((SYMBOL_HN *) andField->value,NULL,pattern,
                                          slotName,theField);
            return(CLIPS_TRUE);
           }
         else if ((andField->type == PREDICATE_CONSTRAINT) ||
                  (andField->type == RETURN_VALUE_CONSTRAINT))
           {
            rv = CheckExpression(andField->expression,NULL,pattern,slotName,theField); 
            if (rv != NULL) return(CLIPS_TRUE);
           }
         else if ((andField->type == INTEGER) || (andField->type == FLOAT) ||
                  (andField->type == SYMBOL) || (andField->type == STRING) ||
                  (andField->type == INSTANCE_NAME))
           {
            if (GetStaticConstraintChecking())
              {
               result = ConstraintCheckValue(andField->type,andField->value,theConstraints);
               if (result != NO_VIOLATION)
                 {
                  ConstraintViolationErrorMessage("A literal restriction value",
                                                  NULL,CLIPS_FALSE,pattern,
                                                  slotName,theField,result,
                                                  theConstraints,CLIPS_TRUE);
                  return(CLIPS_TRUE);
                 }
              }
           
           }
         andField = andField->right;
        }
      orField = orField->bottom;
     }

   return(CLIPS_FALSE);
  }

/***********************************************************************/
/* CheckExpression: Verifies that variables within an expression have  */
/*   been referenced properly. All variables within an expression must */
/*   have been previously defined.                                     */
/***********************************************************************/
static struct lhsParseNode *CheckExpression(exprPtr,lastOne,whichCE,slotName,theField)
  struct lhsParseNode *exprPtr, *lastOne;
  int whichCE;
  struct symbolHashNode *slotName;
  int theField;
  {
   struct lhsParseNode *rv;
   int i = 1;

   while (exprPtr != NULL)
     {
      if (exprPtr->type == SF_VARIABLE) 
        { 
         if (exprPtr->referringNode == NULL)
           {
            VariableReferenceErrorMessage((SYMBOL_HN *) exprPtr->value,lastOne,
                                          whichCE,slotName,theField);
            return(exprPtr);
           }
         else if ((UnmatchableConstraint(exprPtr->constraints)) &&
                  GetStaticConstraintChecking())
           {
            ConstraintReferenceErrorMessage((SYMBOL_HN *) exprPtr->value,lastOne,i,
                                            whichCE,slotName,theField);
            return(exprPtr); 
           }
        }
      else if ((exprPtr->type == MF_VARIABLE) && (exprPtr->referringNode == NULL))
        { 
         VariableReferenceErrorMessage((SYMBOL_HN *) exprPtr->value,lastOne,
                                       whichCE,slotName,theField);
         return(exprPtr); 
        }
#if DEFGLOBAL_CONSTRUCT
      else if (exprPtr->type == GBL_VARIABLE)
        {
         if (QFindDefglobal((SYMBOL_HN *) exprPtr->value) == NULL) 
           {
            VariableReferenceErrorMessage((SYMBOL_HN *) exprPtr->value,lastOne,
                                          whichCE,slotName,theField);
            return(exprPtr);
           }
        }
#endif
      else if (((exprPtr->type == FCALL)
#if DEFGENERIC_CONSTRUCT
             || (exprPtr->type == GCALL)
#endif
#if DEFFUNCTION_CONSTRUCT
             || (exprPtr->type == PCALL)
#endif
         ) && (exprPtr->bottom != NULL))
        {
         if ((rv = CheckExpression(exprPtr->bottom,exprPtr,whichCE,slotName,theField)) != NULL)
           { return(rv); }
        }
      
      i++;
      exprPtr = exprPtr->right;
     }

   return(NULL);
  }
 
/**********************************/
/* VariableReferenceErrorMessage: */
/**********************************/
static VOID VariableReferenceErrorMessage(theVariable,theExpression,whichCE,slotName,theField)
  struct symbolHashNode *theVariable;
  struct lhsParseNode *theExpression;
  int whichCE;
  struct symbolHashNode *slotName;
  int theField;
  {
   struct expr *temprv;
   
   PrintErrorID("ANALYSIS",4,CLIPS_TRUE);

   PrintCLIPS(WERROR,"Variable ?");
   PrintCLIPS(WERROR,ValueToString(theVariable));
   PrintCLIPS(WERROR," ");
   
   if (theExpression != NULL)
     {
      temprv = LHSParseNodesToExpression(theExpression);
      ReturnExpression(temprv->nextArg);
      temprv->nextArg = NULL;
      PrintCLIPS(WERROR,"found in the expression ");
      PrintExpression(WERROR,temprv);
      PrintCLIPS(WERROR,"\n");
      ReturnExpression(temprv);
     }
   
   PrintCLIPS(WERROR,"was referenced in CE #");
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
           
   PrintCLIPS(WERROR," before being defined.\n");
  }
  
/*******************************/
/* VariableMixingErrorMessage: */
/*******************************/
static VOID VariableMixingErrorMessage(theVariable)
  struct symbolHashNode *theVariable;
  {
   PrintErrorID("ANALYSIS",3,CLIPS_TRUE);
   PrintCLIPS(WERROR,"Variable ?");
   PrintCLIPS(WERROR,ValueToString(theVariable));
   PrintCLIPS(WERROR," is used as both a single and multifield variable in the LHS\n");
  }

#endif


