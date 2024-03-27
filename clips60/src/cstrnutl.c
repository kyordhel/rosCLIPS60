   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             CONSTRAINT UTILITY MODULE               */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*      Provides functions for constraint checking of        */
/*      data types.                                          */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian Donnell                                        */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _CSTRNUTL_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_
#if ANSI_COMPILER
#include <stdlib.h>
#endif

#include "setup.h"

#include "constant.h"
#include "clipsmem.h"
#include "router.h"
#include "extnfunc.h"
#include "scanner.h"
#include "multifld.h"
#include "argacces.h"

#include "cstrnutl.h"
   
/***********************************************************************************/
/* GetConstraintRecord: Creates and initializes the values of a constraint record. */
/***********************************************************************************/
globle struct constraintRecord *GetConstraintRecord()
  {   
   CONSTRAINT_RECORD *constraints;
   
   constraints = get_struct(constraintRecord);
  
   SetAnyAllowedFlags(constraints,CLIPS_TRUE);
     
   constraints->anyRestriction = CLIPS_FALSE;
   constraints->symbolRestriction = CLIPS_FALSE;
   constraints->stringRestriction = CLIPS_FALSE;
   constraints->floatRestriction = CLIPS_FALSE;
   constraints->integerRestriction = CLIPS_FALSE;
   constraints->instanceNameRestriction = CLIPS_FALSE; 
   constraints->restrictionList = NULL;
   constraints->minValue = GenConstant(SYMBOL,NegativeInfinity);
   constraints->maxValue = GenConstant(SYMBOL,PositiveInfinity);
   constraints->minFields = GenConstant(INTEGER,Zero);
   constraints->maxFields = GenConstant(SYMBOL,PositiveInfinity);
   constraints->bucket = -1;
   constraints->count = 0;
   constraints->next = NULL;
   
   return(constraints);
  }

/*****************************************************/
/* SetAnyAllowedFlags: */
/*****************************************************/
globle VOID SetAnyAllowedFlags(theConstraint,justOne)
  CONSTRAINT_RECORD *theConstraint;
  int justOne;
  {
   int flag1, flag2;
   
   if (justOne) 
     {
      flag1 = CLIPS_TRUE;
      flag2 = CLIPS_FALSE;
     }
   else
     {
      flag1 = CLIPS_FALSE;
      flag2 = CLIPS_TRUE;
     }

   theConstraint->anyAllowed = flag1;
   theConstraint->symbolsAllowed = flag2;
   theConstraint->stringsAllowed = flag2;
   theConstraint->floatsAllowed = flag2;
   theConstraint->integersAllowed = flag2;
   theConstraint->instanceNamesAllowed = flag2;
   theConstraint->instanceAddressesAllowed = flag2;
   theConstraint->externalAddressesAllowed = flag2; 
   theConstraint->multifieldsAllowed = flag2; 
   theConstraint->factAddressesAllowed = flag2; 
  }

/*****************************************************/
/* CopyConstraintRecord: Copies a constraint record. */
/*****************************************************/
globle struct constraintRecord *CopyConstraintRecord(sourceConstraint)
  CONSTRAINT_RECORD *sourceConstraint;
  {   
   CONSTRAINT_RECORD *theConstraint;
   
   if (sourceConstraint == NULL) return(NULL);
   
   theConstraint = get_struct(constraintRecord);
  
   theConstraint->anyAllowed = sourceConstraint->anyAllowed;
   theConstraint->symbolsAllowed = sourceConstraint->symbolsAllowed;
   theConstraint->stringsAllowed = sourceConstraint->stringsAllowed;
   theConstraint->floatsAllowed = sourceConstraint->floatsAllowed;
   theConstraint->integersAllowed = sourceConstraint->integersAllowed;
   theConstraint->instanceNamesAllowed = sourceConstraint->instanceNamesAllowed;
   theConstraint->instanceAddressesAllowed = sourceConstraint->instanceAddressesAllowed;
   theConstraint->externalAddressesAllowed = sourceConstraint->externalAddressesAllowed; 
   theConstraint->multifieldsAllowed = sourceConstraint->multifieldsAllowed; 
   theConstraint->factAddressesAllowed = sourceConstraint->factAddressesAllowed;  
   theConstraint->anyRestriction = sourceConstraint->anyRestriction;
   theConstraint->symbolRestriction = sourceConstraint->symbolRestriction;
   theConstraint->stringRestriction = sourceConstraint->stringRestriction;
   theConstraint->floatRestriction = sourceConstraint->floatRestriction;
   theConstraint->integerRestriction = sourceConstraint->integerRestriction;
   theConstraint->instanceNameRestriction = sourceConstraint->instanceNameRestriction; 
   theConstraint->restrictionList = CopyExpression(sourceConstraint->restrictionList);
   theConstraint->minValue = CopyExpression(sourceConstraint->minValue);
   theConstraint->maxValue = CopyExpression(sourceConstraint->maxValue);
   theConstraint->minFields = CopyExpression(sourceConstraint->minFields);
   theConstraint->maxFields = CopyExpression(sourceConstraint->maxFields);
   theConstraint->bucket = -1;
   theConstraint->count = 0;
   theConstraint->next = NULL;
   
   return(theConstraint);
  }

#if (! RUN_TIME) && (! BLOAD_ONLY)

/*****************************************************/
/* SetAnyRestrictionFlags: */
/*****************************************************/
globle VOID SetAnyRestrictionFlags(theConstraint,justOne)
  CONSTRAINT_RECORD *theConstraint;
  int justOne;
  {
   int flag1, flag2;
   
   if (justOne) 
     {
      flag1 = CLIPS_TRUE;
      flag2 = CLIPS_FALSE;
     }
   else
     {
      flag1 = CLIPS_FALSE;
      flag2 = CLIPS_TRUE;
     }
   
   theConstraint->anyRestriction = flag1;
   theConstraint->symbolRestriction = flag2;
   theConstraint->stringRestriction = flag2;
   theConstraint->floatRestriction = flag2;
   theConstraint->integerRestriction = flag2;
   theConstraint->instanceNameRestriction = flag2;
  }
  
/********************************************************************/
/* SetConstraintType:                        */
/********************************************************************/
globle int SetConstraintType(theType,constraints)
  int theType;
  CONSTRAINT_RECORD *constraints;
  {
   int rv = CLIPS_TRUE;
   
   switch(theType)
     {
      case UNKNOWN:
         rv = constraints->anyAllowed;
         constraints->anyAllowed = CLIPS_TRUE;
         break;
         
      case SYMBOL:
         rv = constraints->symbolsAllowed;
         constraints->symbolsAllowed = CLIPS_TRUE;
         break;
         
      case STRING:
         rv = constraints->stringsAllowed;
         constraints->stringsAllowed = CLIPS_TRUE;
         break;
         
      case SYMBOL_OR_STRING:
         rv = (constraints->stringsAllowed | constraints->symbolsAllowed);
         constraints->symbolsAllowed = CLIPS_TRUE;
         constraints->stringsAllowed = CLIPS_TRUE;
         break;
         
      case INTEGER:
         rv = constraints->integersAllowed;
         constraints->integersAllowed = CLIPS_TRUE;
         break;
         
      case FLOAT:
         rv = constraints->floatsAllowed;
         constraints->floatsAllowed = CLIPS_TRUE;
         break;
         
      case INTEGER_OR_FLOAT:
         rv = (constraints->integersAllowed | constraints->floatsAllowed);
         constraints->integersAllowed = CLIPS_TRUE;
         constraints->floatsAllowed = CLIPS_TRUE; 
         break;    
         
      case INSTANCE_ADDRESS:
         rv = constraints->instanceAddressesAllowed;
         constraints->instanceAddressesAllowed = CLIPS_TRUE;
         break;
         
      case INSTANCE_NAME:
         rv = constraints->instanceNamesAllowed;
         constraints->instanceNamesAllowed = CLIPS_TRUE;
         break;
         
      case INSTANCE_OR_INSTANCE_NAME:
         rv = (constraints->instanceNamesAllowed | constraints->instanceAddressesAllowed);
         constraints->instanceNamesAllowed = CLIPS_TRUE;
         constraints->instanceAddressesAllowed = CLIPS_TRUE; 
         break;    
         
      case EXTERNAL_ADDRESS:
         rv = constraints->externalAddressesAllowed;
         constraints->externalAddressesAllowed = CLIPS_TRUE;
         break;  
         
      case FACT_ADDRESS:
         rv = constraints->factAddressesAllowed;
         constraints->factAddressesAllowed = CLIPS_TRUE;
         break;
         
      case MULTIFIELD:
         rv = constraints->multifieldsAllowed;
         constraints->multifieldsAllowed = CLIPS_TRUE;
         break;
     }
     
   if (theType != UNKNOWN) constraints->anyAllowed = CLIPS_FALSE;
   return(rv);
  }    

#endif

/*************************************************************/
/* CompareNumbers:   */
/*************************************************************/
globle int CompareNumbers(type1,vptr1,type2,vptr2)
  int type1;
  VOID *vptr1;
  int type2;
  VOID *vptr2;
  { 
   if (vptr1 == vptr2) return(EQUAL);
   
   if (vptr1 == PositiveInfinity) return(GREATER_THAN);
   
   if (vptr1 == NegativeInfinity) return(LESS_THAN);
   
   if (vptr2 == PositiveInfinity) return(LESS_THAN);
   
   if (vptr2 == NegativeInfinity) return(GREATER_THAN);
   
   if ((type1 == INTEGER) && (type2 == INTEGER))
     {
      if (ValueToLong(vptr1) < ValueToLong(vptr2))
        { return(LESS_THAN); }
      else if (ValueToLong(vptr1) > ValueToLong(vptr2))
        { return(GREATER_THAN); }
        
      return(EQUAL);
     }
     
   if ((type1 == FLOAT) && (type2 == FLOAT))
     {
      if (ValueToDouble(vptr1) < ValueToDouble(vptr2))
        { return(LESS_THAN); }
      else if (ValueToDouble(vptr1) > ValueToDouble(vptr2))
        { return(GREATER_THAN); }
        
      return(EQUAL);
     }
     
   if ((type1 == INTEGER) && (type2 == FLOAT))
     {
      if (((double) ValueToLong(vptr1)) < ValueToDouble(vptr2))
        { return(LESS_THAN); }
      else if (((double) ValueToLong(vptr1)) > ValueToDouble(vptr2))
        { return(GREATER_THAN); }
        
      return(EQUAL);
     }
       
   if ((type1 == FLOAT) && (type2 == INTEGER))
     {
      if (ValueToDouble(vptr1) < ((double) ValueToLong(vptr2)))
        { return(LESS_THAN); }
      else if (ValueToDouble(vptr1) > ((double) ValueToLong(vptr2)))
        { return(GREATER_THAN); }
        
      return(EQUAL);
     }
     
   return(-1);
  }


/***************************************************************************/
/* ExpressionToConstraintRecord: Converts an expression into a constraint  */
/*   record. For example, an expression representing the symbol BLUE would */
/*   be converted to a  record with allowed types SYMBOL and allow-values  */
/*   BLUE.                                                                 */
/***************************************************************************/
globle CONSTRAINT_RECORD *ExpressionToConstraintRecord(theExpression)
  struct expr *theExpression;
  {
   CONSTRAINT_RECORD *rv;
   
   /*================================================*/
   /* A NULL expression is converted to a constraint */
   /* record with no values allowed.                 */
   /*================================================*/
   
   if (theExpression == NULL) 
     {
      rv = GetConstraintRecord();
      rv->anyAllowed = CLIPS_FALSE;
      return(rv);
     }
   
   /*=============================================================*/
   /* Convert variables and function calls to constraint records. */
   /*=============================================================*/
   
   if ((theExpression->type == SF_VARIABLE) ||
       (theExpression->type == MF_VARIABLE) ||
#if DEFGENERIC_CONSTRUCT
       (theExpression->type == GCALL) ||
#endif
#if DEFFUNCTION_CONSTRUCT
       (theExpression->type == PCALL) ||
#endif
       (theExpression->type == GBL_VARIABLE) ||
       (theExpression->type == MF_GBL_VARIABLE))
     { return(GetConstraintRecord()); }
   else if (theExpression->type == FCALL)
     { return(FunctionCallToConstraintRecord(theExpression->value)); }
     
   /*============================================*/
   /* Convert a constant to a constraint record. */
   /*============================================*/
   
   rv = GetConstraintRecord();
   rv->anyAllowed = CLIPS_FALSE;
   
   if (theExpression->type == FLOAT) 
     {
      rv->floatRestriction = CLIPS_TRUE;
      rv->floatsAllowed = CLIPS_TRUE;
     }
   else if (theExpression->type == INTEGER)  
     {
      rv->integerRestriction = CLIPS_TRUE;
      rv->integersAllowed = CLIPS_TRUE;
     }
   else if (theExpression->type == SYMBOL)  
     {
      rv->symbolRestriction = CLIPS_TRUE;
      rv->symbolsAllowed = CLIPS_TRUE;
     }
   else if (theExpression->type == STRING)  
     {
      rv->stringRestriction = CLIPS_TRUE;
      rv->stringsAllowed = CLIPS_TRUE;
     }
   else if (theExpression->type == INSTANCE_NAME)  
     {
      rv->instanceNameRestriction = CLIPS_TRUE;
      rv->instanceNamesAllowed = CLIPS_TRUE;
     }
   else if (theExpression->type == INSTANCE_ADDRESS) rv->instanceAddressesAllowed = CLIPS_TRUE;

   if (rv->floatsAllowed || rv->integersAllowed || rv->symbolsAllowed ||
       rv->stringsAllowed || rv->instanceNamesAllowed)
     { rv->restrictionList = GenConstant(theExpression->type,theExpression->value); }
     
   return(rv);
  }
    
/***************************************************************/
/* FunctionCallToConstraintRecord: Converts a function call to */
/*   a constraint record. For example, the + function when     */
/*   converted would be a constraint record with allowed types */
/*  INTEGER and FLOAT.                                         */
/***************************************************************/
globle CONSTRAINT_RECORD *FunctionCallToConstraintRecord(theFunction)
  VOID *theFunction;
  {
   CONSTRAINT_RECORD *rv;
   
   rv = GetConstraintRecord();
   rv->anyAllowed = CLIPS_FALSE;
   
   switch ((char) ValueFunctionType(theFunction))
     {
      case 'a':
        rv->externalAddressesAllowed = CLIPS_TRUE;
        break;

      case 'f':
      case 'd':
        rv->floatsAllowed = CLIPS_TRUE;
        break;

      case 'i':
      case 'l':
        rv->integersAllowed = CLIPS_TRUE;
        break;

      case 'j':
        rv->instanceNamesAllowed = CLIPS_TRUE;
        rv->symbolsAllowed = CLIPS_TRUE;
        rv->stringsAllowed = CLIPS_TRUE;
        break;
        
      case 'k':
        rv->symbolsAllowed = CLIPS_TRUE;
        rv->stringsAllowed = CLIPS_TRUE;
        break;

      case 'm':
        rv->multifieldsAllowed = CLIPS_TRUE;
        break;

      case 'n':
        rv->floatsAllowed = CLIPS_TRUE;
        rv->integersAllowed = CLIPS_TRUE;
        break;

      case 'o':
        rv->instanceNamesAllowed = CLIPS_TRUE;
        break;

      case 's':
        rv->stringsAllowed = CLIPS_TRUE;
        break;

      case 'u':
        rv->anyAllowed = CLIPS_TRUE;
        break;

      case 'w':
      case 'c':
      case 'b':
        rv->symbolsAllowed = CLIPS_TRUE;
        break;

      case 'x':
        rv->instanceAddressesAllowed = CLIPS_TRUE;
        break;
        
      case 'v':
        break;
     }
     
   return(rv);
  }
  
/*************************************************************************/
/* ArgumentTypeToConstraintRecord: Converts one of the function argument */
/*   types (used by DefineFunction2) to a constraint record.             */
/*************************************************************************/
globle CONSTRAINT_RECORD *ArgumentTypeToConstraintRecord(theRestriction)
  int theRestriction;
  {
   CONSTRAINT_RECORD *rv;
   
   rv = GetConstraintRecord();
   rv->anyAllowed = CLIPS_FALSE;
   
   switch (theRestriction)
     {
      case 'a':
        rv->externalAddressesAllowed = CLIPS_TRUE;
        break;

      case 'e':
        rv->symbolsAllowed = CLIPS_TRUE;
        rv->instanceNamesAllowed = CLIPS_TRUE;
        rv->instanceAddressesAllowed = CLIPS_TRUE;
        break;

      case 'd':
      case 'f':
        rv->floatsAllowed = CLIPS_TRUE;
        break;

      case 'g':
        rv->integersAllowed = CLIPS_TRUE;
        rv->floatsAllowed = CLIPS_TRUE;
        rv->symbolsAllowed = CLIPS_TRUE;
        break;

      case 'h':
        rv->factAddressesAllowed = CLIPS_TRUE;
        rv->integersAllowed = CLIPS_TRUE;
        rv->symbolsAllowed = CLIPS_TRUE;
        rv->instanceNamesAllowed = CLIPS_TRUE;
        rv->instanceAddressesAllowed = CLIPS_TRUE;
        break;

      case 'i':
      case 'l':
        rv->integersAllowed = CLIPS_TRUE;
        break;

      case 'j':
        rv->symbolsAllowed = CLIPS_TRUE;
        rv->stringsAllowed = CLIPS_TRUE;
        rv->instanceNamesAllowed = CLIPS_TRUE;
        break;

      case 'k':
        rv->symbolsAllowed = CLIPS_TRUE;
        rv->stringsAllowed = CLIPS_TRUE;
        break;

      case 'm':
        rv->multifieldsAllowed = CLIPS_TRUE;
        break;

      case 'n':
        rv->floatsAllowed = CLIPS_TRUE;
        rv->integersAllowed = CLIPS_TRUE;
        break;

      case 'o':
        rv->instanceNamesAllowed = CLIPS_TRUE;
        break;
        
      case 'p':
        rv->instanceNamesAllowed = CLIPS_TRUE;
        rv->symbolsAllowed = CLIPS_TRUE;
        break;

      case 'q':
        rv->symbolsAllowed = CLIPS_TRUE;
        rv->stringsAllowed = CLIPS_TRUE;
        rv->multifieldsAllowed = CLIPS_TRUE;
        break;

      case 's':
        rv->stringsAllowed = CLIPS_TRUE;
        break;

      case 'w':
        rv->symbolsAllowed = CLIPS_TRUE;
        break;

      case 'x':
        rv->instanceAddressesAllowed = CLIPS_TRUE;
        break;

      case 'y':
        rv->factAddressesAllowed = CLIPS_TRUE;
        break;

      case 'z':
        rv->symbolsAllowed = CLIPS_TRUE;
        rv->factAddressesAllowed = CLIPS_TRUE;
        rv->integersAllowed = CLIPS_TRUE;
        break;

      case 'u':
        rv->anyAllowed = CLIPS_TRUE;
        break;
     }
     
   return(rv);
  }
  

 
    
