   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 CONSTRAINT MODULE                   */
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

#define _CONSTRNT_SOURCE_

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
#include "constrnt.h"
#include "argacces.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
#if (! RUN_TIME) && (! BLOAD_ONLY)
   static VOID                     InstallConstraintRecord(CONSTRAINT_RECORD *);
   static int                      ConstraintCompare(struct constraintRecord *,struct constraintRecord *);
#endif
#if (! RUN_TIME)
   static VOID                     ReturnConstraintRecord(CONSTRAINT_RECORD *);
   static VOID                     DeinstallConstraintRecord(CONSTRAINT_RECORD *);
#endif
#else
#if (! RUN_TIME) && (! BLOAD_ONLY)
   static VOID                     InstallConstraintRecord();
   static int                      ConstraintCompare();
#endif
#if (! RUN_TIME)
   static VOID                     ReturnConstraintRecord();
   static VOID                     DeinstallConstraintRecord();
#endif
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct constraintRecord   **ConstraintHashtable;
   
/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/
 
   static BOOLEAN                     StaticConstraintChecking = CLIPS_TRUE;
   static BOOLEAN                     DynamicConstraintChecking = CLIPS_FALSE;
   
#if (! RUN_TIME) && (! BLOAD_ONLY)

/********************************************************/
/* InstallConstraintRecord: Increments the count values */
/*   of all occurrences of primitive data types found   */
/*   in a constraint record.                            */
/********************************************************/
static VOID InstallConstraintRecord(constraints)
  CONSTRAINT_RECORD *constraints;
  {    
   struct expr *tempExpr;
   
   tempExpr = AddHashedExpression(constraints->restrictionList);
   ReturnExpression(constraints->restrictionList);
   constraints->restrictionList = tempExpr;
   
   tempExpr = AddHashedExpression(constraints->maxValue);
   ReturnExpression(constraints->maxValue);
   constraints->maxValue = tempExpr;
   
   tempExpr = AddHashedExpression(constraints->minValue);
   ReturnExpression(constraints->minValue);
   constraints->minValue = tempExpr;
   
   tempExpr = AddHashedExpression(constraints->minFields);
   ReturnExpression(constraints->minFields);
   constraints->minFields = tempExpr;
   
   tempExpr = AddHashedExpression(constraints->maxFields);
   ReturnExpression(constraints->maxFields);
   constraints->maxFields = tempExpr;
  }

#endif

#if (! RUN_TIME)

/*************************************************************/
/* ReturnConstraintRecord: Frees the data structures used by */
/*   a constraint record. If the returnOnlyFields argument   */
/*   is FALSE, then the constraint record is also freed.     */
/*************************************************************/
static VOID ReturnConstraintRecord(constraints)
  CONSTRAINT_RECORD *constraints;
  {    
   if (constraints == NULL) return;
   
   if (constraints->bucket < 0)
     {
      ReturnExpression(constraints->restrictionList);
      ReturnExpression(constraints->maxValue);
      ReturnExpression(constraints->minValue);
      ReturnExpression(constraints->minFields);
      ReturnExpression(constraints->maxFields);
     }
     
   rtn_struct(constraintRecord,constraints);
  }

/***************************************************/
/* DeinstallConstraintRecord: Decrements the count */
/*   values of all occurrences of primitive data   */
/*   types found in a constraint record.           */
/***************************************************/
static VOID DeinstallConstraintRecord(constraints)
  CONSTRAINT_RECORD *constraints;
  {   
   if (constraints->bucket >= 0)
     {
      RemoveHashedExpression(constraints->restrictionList);
      RemoveHashedExpression(constraints->maxValue);
      RemoveHashedExpression(constraints->minValue);
      RemoveHashedExpression(constraints->minFields);
      RemoveHashedExpression(constraints->maxFields);
     }
   else
     {
      ExpressionDeinstall(constraints->restrictionList);
      ExpressionDeinstall(constraints->maxValue);
      ExpressionDeinstall(constraints->minValue);
      ExpressionDeinstall(constraints->minFields);
      ExpressionDeinstall(constraints->maxFields);
     }
  }

#endif
  
/************************************************************************/
/* InitializeConstraints: Initializes the constraint hash table to NULL */
/*   and defines the static and dynamic constraint access functions.    */
/************************************************************************/
globle VOID InitializeConstraints()
   {
#if (! RUN_TIME) && (! BLOAD_ONLY)
    int i;
    
    ConstraintHashtable = (struct constraintRecord **) 
                          gm2((int) sizeof (struct constraintRecord *) * SIZE_CONSTRAINT_HASH);

    if (ConstraintHashtable == NULL) ExitCLIPS(1);

    for (i = 0; i < SIZE_CONSTRAINT_HASH; i++) ConstraintHashtable[i] = NULL;
#endif

#if (! RUN_TIME)    
   DefineFunction2("get-dynamic-constraint-checking",'b',GDCCommand,"GDCCommand", "00");
   DefineFunction2("set-dynamic-constraint-checking",'b',SDCCommand,"SDCCommand", "11");
   
   DefineFunction2("get-static-constraint-checking",'b',GSCCommand,"GSCCommand", "00");
   DefineFunction2("set-static-constraint-checking",'b',SSCCommand,"SSCCommand", "11");
#endif
  }

#if (! RUN_TIME) && (! BLOAD_ONLY)    

/****************************************************************/
/* HashConstraint: Returns a hash value for a given constraint. */
/****************************************************************/
globle int HashConstraint(theConstraint)
  struct constraintRecord *theConstraint;
  {
   int i = 0;
   unsigned int count = 0;
   int hashValue = 0;
   struct expr *tmpPtr;

   count +=       
      (theConstraint->anyAllowed * 17) +
      (theConstraint->symbolsAllowed * 5) +
      (theConstraint->stringsAllowed * 23) +
      (theConstraint->floatsAllowed * 19) +
      (theConstraint->integersAllowed * 29) +
      (theConstraint->instanceNamesAllowed * 31) +
      (theConstraint->instanceAddressesAllowed * 17);
      
   count +=
      (theConstraint->externalAddressesAllowed * 29) +
      (theConstraint->multifieldsAllowed * 29) +
      (theConstraint->factAddressesAllowed * 79) +
      (theConstraint->anyRestriction * 59) +
      (theConstraint->symbolRestriction * 61);
   count +=
      (theConstraint->stringRestriction * 3) +
      (theConstraint->floatRestriction * 37) +
      (theConstraint->integerRestriction * 9) +
      (theConstraint->instanceNameRestriction * 7);
   
   for (tmpPtr = theConstraint->restrictionList; tmpPtr != NULL; tmpPtr = tmpPtr->nextArg)
     { count += GetAtomicHashValue(tmpPtr->type,tmpPtr->value,i++); }
     
   for (tmpPtr = theConstraint->minValue; tmpPtr != NULL; tmpPtr = tmpPtr->nextArg)
     { count += GetAtomicHashValue(tmpPtr->type,tmpPtr->value,i++); }
     
   for (tmpPtr = theConstraint->maxValue; tmpPtr != NULL; tmpPtr = tmpPtr->nextArg)
     { count += GetAtomicHashValue(tmpPtr->type,tmpPtr->value,i++); }
     
   for (tmpPtr = theConstraint->minFields; tmpPtr != NULL; tmpPtr = tmpPtr->nextArg)
     { count += GetAtomicHashValue(tmpPtr->type,tmpPtr->value,i++); }
     
   for (tmpPtr = theConstraint->maxFields; tmpPtr != NULL; tmpPtr = tmpPtr->nextArg)
     { count += GetAtomicHashValue(tmpPtr->type,tmpPtr->value,i++); }
   
   hashValue = (int) (count % SIZE_CONSTRAINT_HASH);
   if (hashValue < 0) hashValue = - hashValue;
   return(hashValue);
  }
  
/************************************************************************/
/* ConstraintCompare: */
/************************************************************************/
static int ConstraintCompare(constraint1,constraint2)
  struct constraintRecord *constraint1, *constraint2;
  {
   struct expr *tmpPtr1, *tmpPtr2;
   
   if ((constraint1->anyAllowed != constraint2->anyAllowed) ||
       (constraint1->symbolsAllowed != constraint2->symbolsAllowed) ||
       (constraint1->stringsAllowed != constraint2->stringsAllowed) ||
       (constraint1->floatsAllowed != constraint2->floatsAllowed) ||
       (constraint1->integersAllowed != constraint2->integersAllowed) ||
       (constraint1->instanceNamesAllowed != constraint2->instanceNamesAllowed) ||
       (constraint1->instanceAddressesAllowed != constraint2->instanceAddressesAllowed) ||
       (constraint1->externalAddressesAllowed != constraint2->externalAddressesAllowed) ||
       (constraint1->multifieldsAllowed != constraint2->multifieldsAllowed) ||
       (constraint1->factAddressesAllowed != constraint2->factAddressesAllowed) ||
       (constraint1->anyRestriction != constraint2->anyRestriction) ||
       (constraint1->symbolRestriction != constraint2->symbolRestriction) ||
       (constraint1->stringRestriction != constraint2->stringRestriction) ||
       (constraint1->floatRestriction != constraint2->floatRestriction) ||
       (constraint1->integerRestriction != constraint2->integerRestriction) ||
       (constraint1->instanceNameRestriction != constraint2->instanceNameRestriction))
     { return(CLIPS_FALSE); }

   for (tmpPtr1 = constraint1->restrictionList, tmpPtr2 = constraint2->restrictionList;
        (tmpPtr1 != NULL) && (tmpPtr2 != NULL); 
        tmpPtr1 = tmpPtr1->nextArg, tmpPtr2 = tmpPtr2->nextArg)
     { 
      if ((tmpPtr1->type != tmpPtr2->type) || (tmpPtr1->value != tmpPtr2->value))
        { return(CLIPS_FALSE); }
     }
   if (tmpPtr1 != tmpPtr2) return(CLIPS_FALSE);

   for (tmpPtr1 = constraint1->minValue, tmpPtr2 = constraint2->minValue;
        (tmpPtr1 != NULL) && (tmpPtr2 != NULL); 
        tmpPtr1 = tmpPtr1->nextArg, tmpPtr2 = tmpPtr2->nextArg)
     { 
      if ((tmpPtr1->type != tmpPtr2->type) || (tmpPtr1->value != tmpPtr2->value))
        { return(CLIPS_FALSE); }
     }
   if (tmpPtr1 != tmpPtr2) return(CLIPS_FALSE);
      
   for (tmpPtr1 = constraint1->maxValue, tmpPtr2 = constraint2->maxValue;
        (tmpPtr1 != NULL) && (tmpPtr2 != NULL); 
        tmpPtr1 = tmpPtr1->nextArg, tmpPtr2 = tmpPtr2->nextArg)
     { 
      if ((tmpPtr1->type != tmpPtr2->type) || (tmpPtr1->value != tmpPtr2->value))
        { return(CLIPS_FALSE); }
     }
   if (tmpPtr1 != tmpPtr2) return(CLIPS_FALSE);
        
   for (tmpPtr1 = constraint1->minFields, tmpPtr2 = constraint2->minFields;
        (tmpPtr1 != NULL) && (tmpPtr2 != NULL); 
        tmpPtr1 = tmpPtr1->nextArg, tmpPtr2 = tmpPtr2->nextArg)
     { 
      if ((tmpPtr1->type != tmpPtr2->type) || (tmpPtr1->value != tmpPtr2->value))
        { return(CLIPS_FALSE); }
     }
   if (tmpPtr1 != tmpPtr2) return(CLIPS_FALSE);
      
   for (tmpPtr1 = constraint1->maxFields, tmpPtr2 = constraint2->maxFields;
        (tmpPtr1 != NULL) && (tmpPtr2 != NULL); 
        tmpPtr1 = tmpPtr1->nextArg, tmpPtr2 = tmpPtr2->nextArg)
     { 
      if ((tmpPtr1->type != tmpPtr2->type) || (tmpPtr1->value != tmpPtr2->value))
        { return(CLIPS_FALSE); }
     }
   if (tmpPtr1 != tmpPtr2) return(CLIPS_FALSE);

   return(CLIPS_TRUE);
  }
  
/******************************************************************/
/* AddConstraint: Adds a constraint to the constraint hash table. */
/******************************************************************/
globle struct constraintRecord *AddConstraint(theConstraint)
  struct constraintRecord *theConstraint;
  {
   struct constraintRecord *tmpPtr;
   int hashValue;

   if (theConstraint == NULL) return(NULL);
   
   hashValue = HashConstraint(theConstraint);

   tmpPtr = ConstraintHashtable[hashValue];

   while (tmpPtr != NULL)
     {
      if (ConstraintCompare(theConstraint,tmpPtr)) 
        {
         tmpPtr->count++;
         ReturnConstraintRecord(theConstraint);
         return(tmpPtr);
        }
      tmpPtr = tmpPtr->next;
     }
     
   InstallConstraintRecord(theConstraint);
   theConstraint->count = 1;
   theConstraint->bucket = hashValue;
   theConstraint->next = ConstraintHashtable[hashValue];
   ConstraintHashtable[hashValue] = theConstraint;
   return(theConstraint);
  }

#endif

#if (! RUN_TIME)

/**************************************************************************/
/* RemoveConstraint: Removes a constraint from the constraint hash table. */
/**************************************************************************/
globle VOID RemoveConstraint(theConstraint)
  struct constraintRecord *theConstraint;
  {
   struct constraintRecord *tmpPtr, *prevPtr = NULL;

   if (theConstraint == NULL) return;
   
   if (theConstraint->bucket < 0)
     {
      ReturnConstraintRecord(theConstraint);
      return;
     }
     
   tmpPtr = ConstraintHashtable[theConstraint->bucket];
   while (tmpPtr != NULL)
     {
      if (tmpPtr == theConstraint)
        {
         theConstraint->count--;
         if (theConstraint->count == 0)
           {
            if (prevPtr == NULL) 
              { ConstraintHashtable[theConstraint->bucket] = theConstraint->next; }
            else
              { prevPtr->next = theConstraint->next; }
            DeinstallConstraintRecord(theConstraint);
            ReturnConstraintRecord(theConstraint);
           }
         return;
        }
        
      prevPtr = tmpPtr;
      tmpPtr = tmpPtr->next;
     }
     
   return;
  }
  
#endif
  
/***********************************************************************/
/* SDCCommand: Implements the set-dynamic-constraint-checking command. */
/***********************************************************************/
globle int SDCCommand()
  {
   int oldValue;
   DATA_OBJECT arg_ptr;

   oldValue = GetDynamicConstraintChecking();

   if (ArgCountCheck("set-dynamic-constraint-checking",EXACTLY,1) == -1)
     { return(oldValue); }

   RtnUnknown(1,&arg_ptr);

   if ((arg_ptr.value == CLIPSFalseSymbol) && (arg_ptr.type == SYMBOL))
     { SetDynamicConstraintChecking(CLIPS_FALSE); }
   else
     { SetDynamicConstraintChecking(CLIPS_TRUE); }

   return(oldValue);
  }

/***********************************************************************/
/* GDCCommand: Implements the get-dynamic-constraint-checking command. */
/***********************************************************************/
globle int GDCCommand()
  {
   int oldValue;

   oldValue = GetDynamicConstraintChecking();

   if (ArgCountCheck("get-dynamic-constraint-checking",EXACTLY,0) == -1)
     { return(oldValue); }

   return(oldValue);
  }
  
/**********************************************************************/
/* SSCCommand: Implements the set-static-constraint-checking command. */
/**********************************************************************/
globle int SSCCommand()
  {
   int oldValue;
   DATA_OBJECT arg_ptr;

   oldValue = GetStaticConstraintChecking();

   if (ArgCountCheck("set-static-constraint-checking",EXACTLY,1) == -1)
     { return(oldValue); }

   RtnUnknown(1,&arg_ptr);

   if ((arg_ptr.value == CLIPSFalseSymbol) && (arg_ptr.type == SYMBOL))
     { SetStaticConstraintChecking(CLIPS_FALSE); }
   else
     { SetStaticConstraintChecking(CLIPS_TRUE); }

   return(oldValue);
  }

/**********************************************************************/
/* GSCCommand: Implements the get-static-constraint-checking command. */
/**********************************************************************/
globle int GSCCommand()
  {
   int oldValue;

   oldValue = GetStaticConstraintChecking();

   if (ArgCountCheck("get-static-constraint-checking",EXACTLY,0) == -1)
     { return(oldValue); }

   return(oldValue);
  }
  
/************************************************/
/* SetDynamicConstraintChecking: Sets the value */
/*   of the DynamicConstraintChecking flag.     */
/************************************************/
globle BOOLEAN SetDynamicConstraintChecking(value)
  int value;
  {
   int ov;

   ov = DynamicConstraintChecking;
   DynamicConstraintChecking = value;
   return(ov);
  }

/*****************************************************/
/* GetDynamicConstraintChecking: Returns the current */
/*   value of the DynamicConstraintChecking flag.    */
/*****************************************************/
globle BOOLEAN GetDynamicConstraintChecking()
  { return(DynamicConstraintChecking); }
  
/***********************************************/
/* SetStaticConstraintChecking: Sets the value */
/*   of the StaticConstraintChecking flag.     */
/***********************************************/
globle BOOLEAN SetStaticConstraintChecking(value)
  int value;
  {
   int ov;

   ov = StaticConstraintChecking;
   StaticConstraintChecking = value;
   return(ov);
  }

/****************************************************/
/* GetStaticConstraintChecking: Returns the current */
/*   value of the StaticConstraintChecking flag.    */
/****************************************************/
globle BOOLEAN GetStaticConstraintChecking()
  { return(StaticConstraintChecking); }
    
