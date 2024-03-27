   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            CONSTRAINT OPERATIONS MODULE             */
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
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _CSTRNOPS_SOURCE_

#include "setup.h"

#include <stdio.h>
#define _CLIPS_STDIO_
#if ANSI_COMPILER
#include <stdlib.h>
#endif

#if (! RUN_TIME)

#include "constant.h"
#include "clipsmem.h"
#include "router.h"
#include "extnfunc.h"
#include "scanner.h"
#include "multifld.h"
#include "constrnt.h"
#include "cstrnchk.h"
#include "cstrnutl.h"

#include "cstrnops.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                     IntersectNumericExpressions(CONSTRAINT_RECORD *,
                                                             CONSTRAINT_RECORD *,
                                                             CONSTRAINT_RECORD *,int);
   static VOID                     IntersectAllowedValueExpressions(CONSTRAINT_RECORD *,
                                                                    CONSTRAINT_RECORD *,
                                                                    CONSTRAINT_RECORD *);
   static int                      FindItemInExpression(int,VOID *,int,struct expr *);
   static VOID                     UpdateRestrictionFlags(CONSTRAINT_RECORD *);
#if (! BLOAD_ONLY)
   static VOID                     UnionSingleValueWithList(struct expr *,
                                                                 struct expr *,
                                                                 struct expr **,
                                                                 struct expr **);
   static VOID                     UnionNumericExpressions(CONSTRAINT_RECORD *,
                                                         CONSTRAINT_RECORD *,
                                                         CONSTRAINT_RECORD *,int);
   static struct expr             *AddToUnionList(struct expr *,struct expr *,
                                                  CONSTRAINT_RECORD *);
   static VOID                     UnionAllowedValueExpressions(CONSTRAINT_RECORD *,
                                                                CONSTRAINT_RECORD *,
                                                                CONSTRAINT_RECORD *);
   static int                      RestrictionOnType(int,CONSTRAINT_RECORD *);
#endif
#else
   static VOID                     IntersectNumericExpressions();
   static VOID                     IntersectAllowedValueExpressions();
   static int                      FindItemInExpression();
   static VOID                     UpdateRestrictionFlags();
#if (! BLOAD_ONLY)
   static VOID                     UnionSingleValueWithList();
   static VOID                     UnionNumericExpressions();
   static struct expr             *AddToUnionList();
   static VOID                     UnionAllowedValueExpressions();
   static int                      RestrictionOnType();
#endif
#endif
   
/*************************************************************/
/* IntersectConstraints:  */
/*************************************************************/
globle struct constraintRecord *IntersectConstraints(c1,c2)
  CONSTRAINT_RECORD *c1, *c2;
  {
   struct constraintRecord *rv;
   int c1Changed = CLIPS_FALSE, c2Changed = CLIPS_FALSE;
   
   if ((c1 == NULL) && (c2 == NULL)) return(GetConstraintRecord());
   
   if (c1 == NULL) return(CopyConstraintRecord(c2));
     
   if (c2 == NULL) return(CopyConstraintRecord(c1));
          
   rv = GetConstraintRecord();
      
   if (c1->anyAllowed && c2->anyAllowed) rv->anyAllowed = CLIPS_TRUE;
   else
     {
      if (c1->anyAllowed) 
        {
         c1Changed = CLIPS_TRUE;
         SetAnyAllowedFlags(c1,CLIPS_FALSE);
        }
      else if (c2->anyAllowed) 
        {
         c2Changed = CLIPS_TRUE;
         SetAnyAllowedFlags(c2,CLIPS_FALSE);
        }
        
      rv->anyAllowed = CLIPS_FALSE;
      rv->symbolsAllowed = (c1->symbolsAllowed && c2->symbolsAllowed);
      rv->stringsAllowed = (c1->stringsAllowed && c2->stringsAllowed);
      rv->floatsAllowed = (c1->floatsAllowed && c2->floatsAllowed);
      rv->integersAllowed = (c1->integersAllowed && c2->integersAllowed);
      rv->instanceNamesAllowed = (c1->instanceNamesAllowed && c2->instanceNamesAllowed);
      rv->instanceAddressesAllowed = (c1->instanceAddressesAllowed && c2->instanceAddressesAllowed);
      rv->externalAddressesAllowed = (c1->externalAddressesAllowed && c2->externalAddressesAllowed);
      rv->multifieldsAllowed = (c1->multifieldsAllowed && c2->multifieldsAllowed);
      rv->factAddressesAllowed = (c1->factAddressesAllowed && c2->factAddressesAllowed);

      if (c1Changed) SetAnyAllowedFlags(c1,CLIPS_TRUE);
      if (c2Changed) SetAnyAllowedFlags(c2,CLIPS_TRUE);
     }
   
   /*==========================================*/
   /* Handle the intersection of restrictions. */
   /*==========================================*/
   
   if (c1->anyRestriction || c2->anyRestriction) rv->anyRestriction = CLIPS_TRUE;
   else
     {
      rv->anyRestriction = CLIPS_FALSE;
      rv->symbolRestriction = (c1->symbolRestriction || c2->symbolRestriction);
      rv->stringRestriction = (c1->stringRestriction || c2->stringRestriction);
      rv->floatRestriction = (c1->floatRestriction || c2->floatRestriction);
      rv->integerRestriction = (c1->integerRestriction || c2->integerRestriction);
      rv->instanceNameRestriction = (c1->instanceNameRestriction || c2->instanceNameRestriction);
     }   

   IntersectAllowedValueExpressions(c1,c2,rv);
   IntersectNumericExpressions(c1,c2,rv,CLIPS_TRUE);
   IntersectNumericExpressions(c1,c2,rv,CLIPS_FALSE);
   
   UpdateRestrictionFlags(rv);
        
   return(rv);
  }   

/*************************************************************/
/* IntersectAllowedValueExpressions:  */
/*************************************************************/
static VOID IntersectAllowedValueExpressions(constraint1,constraint2,newConstraint)
  CONSTRAINT_RECORD *constraint1, *constraint2, *newConstraint;
  {
   struct expr *theList1, *theList2;
   struct expr *theHead = NULL, *tmpExpr;
   
   theList1 = constraint1->restrictionList;
   while (theList1 != NULL)
     {
      if (CheckAllowedValuesConstraint(theList1->type,theList1->value,constraint1) &&
          CheckAllowedValuesConstraint(theList1->type,theList1->value,constraint2))
        {
         tmpExpr = GenConstant(theList1->type,theList1->value);
         tmpExpr->nextArg = theHead;
         theHead = tmpExpr;
        }
        
      theList1 = theList1->nextArg;
     }
     
   theList2 = constraint2->restrictionList;
   while (theList2 != NULL)
     {
      if (FindItemInExpression(theList2->type,theList2->value,CLIPS_TRUE,theHead))
        { /* Do Nothing */ }
      else if (CheckAllowedValuesConstraint(theList2->type,theList2->value,constraint1) &&
          CheckAllowedValuesConstraint(theList2->type,theList2->value,constraint2))
        {
         tmpExpr = GenConstant(theList2->type,theList2->value);
         tmpExpr->nextArg = theHead;
         theHead = tmpExpr;
        }
        
      theList2 = theList2->nextArg;
     }
     
   newConstraint->restrictionList = theHead;
  }
  

/*************************************************************/
/* IntersectNumericExpressions:  */
/*************************************************************/
static VOID IntersectNumericExpressions(constraint1,constraint2,newConstraint,range)
  CONSTRAINT_RECORD *constraint1, *constraint2, *newConstraint;
  int range;
  {
   struct expr *tmpmin1, *tmpmax1, *tmpmin2, *tmpmax2, *theMin, *theMax;
   struct expr *theMinList, *theMaxList, *lastMin = NULL, *lastMax = NULL;
   int cmaxmax, cminmin, cmaxmin, cminmax;
   
   theMinList = NULL;
   theMaxList = NULL;
   
   if (range)
     {
      tmpmin1 = constraint1->minValue;
      tmpmax1 = constraint1->maxValue;
     }
   else
     {
      tmpmin1 = constraint1->minFields;
      tmpmax1 = constraint1->maxFields;
     }
   
   while (tmpmin1 != NULL)
     {
      if (range)
        {
         tmpmin2 = constraint2->minValue;
         tmpmax2 = constraint2->maxValue;
        }
      else
        {
         tmpmin2 = constraint2->minFields;
         tmpmax2 = constraint2->maxFields;
        }
      
      while (tmpmin2 != NULL)
        {
         cmaxmax = CompareNumbers(tmpmax1->type,tmpmax1->value,
                                  tmpmax2->type,tmpmax2->value);
                           
         cminmin = CompareNumbers(tmpmin1->type,tmpmin1->value,
                                  tmpmin2->type,tmpmin2->value);
                               
         cmaxmin = CompareNumbers(tmpmax1->type,tmpmax1->value,
                                  tmpmin2->type,tmpmin2->value);
                           
         cminmax = CompareNumbers(tmpmin1->type,tmpmin1->value,
                                  tmpmax2->type,tmpmax2->value);
                 
         if ((cmaxmin != LESS_THAN) && (cminmax != GREATER_THAN))
           {
            if (cminmin == GREATER_THAN)
              { theMin = GenConstant(tmpmin1->type,tmpmin1->value); }
            else
              { theMin = GenConstant(tmpmin2->type,tmpmin2->value); }
              
            if (cmaxmax == LESS_THAN)
              { theMax = GenConstant(tmpmax1->type,tmpmax1->value); }
            else
              { theMax = GenConstant(tmpmax2->type,tmpmax2->value); }
              
            if (lastMin == NULL) 
              {
               theMinList = theMin;
               theMaxList = theMax;
              }
            else
              {
               lastMin->nextArg = theMin;
               lastMax->nextArg = theMax;
              }
              
            lastMin = theMin;
            lastMax = theMax;
           }
         
         tmpmin2 = tmpmin2->nextArg;
         tmpmax2 = tmpmax2->nextArg;
        }
     
      tmpmin1 = tmpmin1->nextArg;
      tmpmax1 = tmpmax1->nextArg;
     }
     
   if (theMinList != NULL)
     {
      if (range)
        {
         ReturnExpression(newConstraint->minValue);
         ReturnExpression(newConstraint->maxValue);
         newConstraint->minValue = theMinList;
         newConstraint->maxValue = theMaxList;
        }
      else
        {
         ReturnExpression(newConstraint->minFields);
         ReturnExpression(newConstraint->maxFields);
         newConstraint->minFields = theMinList;
         newConstraint->maxFields = theMaxList;
        }
     }
   else
     {
      if (range)
        {
         if (newConstraint->anyAllowed) SetAnyAllowedFlags(newConstraint,CLIPS_FALSE);
         newConstraint->integersAllowed = CLIPS_FALSE;
         newConstraint->floatsAllowed = CLIPS_FALSE;
        }
      else
        {
         SetAnyAllowedFlags(newConstraint,CLIPS_TRUE);
         newConstraint->anyAllowed = CLIPS_FALSE;
        }
     }
  }

/*************************************************************/
/* UpdateRestrictionFlags:  */
/*************************************************************/
static VOID UpdateRestrictionFlags(rv)
  CONSTRAINT_RECORD *rv;
  {
   if ((rv->anyRestriction) && (rv->restrictionList == NULL))
     {
      SetAnyAllowedFlags(rv,CLIPS_TRUE);
      rv->anyAllowed = CLIPS_FALSE;
     }
     
   if ((rv->symbolRestriction) && (rv->symbolsAllowed))
     { rv->symbolsAllowed = FindItemInExpression(SYMBOL,NULL,CLIPS_FALSE,rv->restrictionList); }
     
   if ((rv->stringRestriction)  && (rv->stringsAllowed))
     { rv->stringsAllowed = FindItemInExpression(STRING,NULL,CLIPS_FALSE,rv->restrictionList); }
     
   if ((rv->floatRestriction) && (rv->floatsAllowed))
     { rv->floatsAllowed = FindItemInExpression(FLOAT,NULL,CLIPS_FALSE,rv->restrictionList); }
     
   if ((rv->integerRestriction) && (rv->integersAllowed)) 
     { rv->integersAllowed = FindItemInExpression(INTEGER,NULL,CLIPS_FALSE,rv->restrictionList); }
        
   if ((rv->instanceNameRestriction) && (rv->instanceNamesAllowed))
     { rv->instanceNamesAllowed = FindItemInExpression(INSTANCE_NAME,NULL,CLIPS_FALSE,rv->restrictionList); }
  }

/*************************************************************/
/* FindItemInExpression:  */
/*************************************************************/
static int FindItemInExpression(theType,theValue,useValue,theList)
  int theType;
  VOID *theValue;
  int useValue;
  struct expr *theList;
  {
   while (theList != NULL)
     {
      if (theList->type == theType) 
        {
         if (! useValue) return(CLIPS_TRUE);
         else if (theList->value == theValue) return(CLIPS_TRUE);
        }
      
      theList = theList->nextArg;
     }
   return(CLIPS_FALSE);
  }
  
#if (! BLOAD_ONLY)

/*************************************************************/
/* RestrictionOnType:  */
/*************************************************************/
static int RestrictionOnType(theType,theConstraint)
  int theType;
  CONSTRAINT_RECORD *theConstraint;
  {
   if (theConstraint == NULL) return(CLIPS_FALSE);
   
   if ((theConstraint->anyRestriction) ||
       (theConstraint->symbolRestriction && (theType == SYMBOL)) ||
       (theConstraint->stringRestriction && (theType == STRING)) ||
       (theConstraint->floatRestriction && (theType == FLOAT)) ||
       (theConstraint->integerRestriction && (theType == INTEGER)) ||
       (theConstraint->instanceNameRestriction && (theType == INSTANCE_NAME)))
     { return(CLIPS_TRUE); }
     
   return(CLIPS_FALSE);
  }
   
/*************************************************************/
/* UnionConstraints:  */
/*************************************************************/
globle struct constraintRecord *UnionConstraints(c1,c2)
  CONSTRAINT_RECORD *c1, *c2;
  {
   struct constraintRecord *rv;
   int c1Changed = CLIPS_FALSE, c2Changed = CLIPS_FALSE;
   
   if ((c1 == NULL) && (c2 == NULL)) return(GetConstraintRecord());
   
   if (c1 == NULL) return(CopyConstraintRecord(c2));
   
   if (c2 == NULL) return(CopyConstraintRecord(c1));
          
   rv = GetConstraintRecord();
   
   /*====================================*/
   /* Handle the union of allowed types. */
   /*====================================*/
   
   if (c1->anyAllowed || c2->anyAllowed) rv->anyAllowed = CLIPS_TRUE;
   else
     {
      rv->anyAllowed = CLIPS_FALSE;
      rv->symbolsAllowed = (c1->symbolsAllowed || c2->symbolsAllowed);
      rv->stringsAllowed = (c1->stringsAllowed || c2->stringsAllowed);
      rv->floatsAllowed = (c1->floatsAllowed || c2->floatsAllowed);
      rv->integersAllowed = (c1->integersAllowed || c2->integersAllowed);
      rv->instanceNamesAllowed = (c1->instanceNamesAllowed || c2->instanceNamesAllowed);
      rv->instanceAddressesAllowed = (c1->instanceAddressesAllowed || c2->instanceAddressesAllowed);
      rv->externalAddressesAllowed = (c1->externalAddressesAllowed || c2->externalAddressesAllowed);
      rv->multifieldsAllowed = (c1->multifieldsAllowed || c2->multifieldsAllowed);
      rv->factAddressesAllowed = (c1->factAddressesAllowed || c2->factAddressesAllowed);
     }
     
   /*===================================*/
   /* Handle the union of restrictions. */
   /*===================================*/
   
   if (c1->anyRestriction && c2->anyRestriction) rv->anyRestriction = CLIPS_TRUE;
   else
     {
      if (c1->anyRestriction)
        {
         c1Changed = CLIPS_TRUE;
         SetAnyRestrictionFlags(c1,CLIPS_FALSE);
        }
      else if (c2->anyRestriction)
        {
         c2Changed = CLIPS_TRUE;
         SetAnyRestrictionFlags(c2,CLIPS_FALSE);
        }
        
      rv->anyRestriction = CLIPS_FALSE;
      rv->symbolRestriction = (c1->symbolRestriction && c2->symbolRestriction);
      rv->stringRestriction = (c1->stringRestriction && c2->stringRestriction);
      rv->floatRestriction = (c1->floatRestriction && c2->floatRestriction);
      rv->integerRestriction = (c1->integerRestriction && c2->integerRestriction);
      rv->instanceNameRestriction = (c1->instanceNameRestriction && c2->instanceNameRestriction);

      if (c1Changed) SetAnyRestrictionFlags(c1,CLIPS_FALSE);
      else if (c2Changed) SetAnyRestrictionFlags(c2,CLIPS_FALSE);
     }   
   
   UnionAllowedValueExpressions(c1,c2,rv);
   UnionNumericExpressions(c1,c2,rv,CLIPS_TRUE);
   UnionNumericExpressions(c1,c2,rv,CLIPS_FALSE);
   return(rv);
  }   
  
/*************************************************************/
/* UnionNumericExpressions:  */
/*************************************************************/
static VOID UnionNumericExpressions(constraint1,constraint2,newConstraint,range)
  CONSTRAINT_RECORD *constraint1, *constraint2, *newConstraint;
  int range;
  {
   struct expr *tmpmin, *tmpmax;
   struct expr *theMinList, *theMaxList;
 
   theMinList = NULL;
   theMaxList = NULL;
   
   if (range)
     {
      tmpmin = constraint1->minValue;
      tmpmax = constraint1->maxValue;
     }
   else
     {
      tmpmin = constraint1->minFields;
      tmpmax = constraint1->maxFields;
     }
   
   while (tmpmin != NULL)
     {
      UnionSingleValueWithList(tmpmin,tmpmax,&theMinList,&theMaxList);
      
      tmpmin = tmpmin->nextArg;
      tmpmax = tmpmax->nextArg;
     }
     
   if (range)
     {
      tmpmin = constraint2->minValue;
      tmpmax = constraint2->maxValue;
     }
   else
     {
      tmpmin = constraint2->minFields;
      tmpmax = constraint2->maxFields;
     }   
     
   while (tmpmin != NULL)
     {
      UnionSingleValueWithList(tmpmin,tmpmax,&theMinList,&theMaxList);
      
      tmpmin = tmpmin->nextArg;
      tmpmax = tmpmax->nextArg;
     }
     
   if (theMinList != NULL)
     {
      if (range)
        {
         ReturnExpression(newConstraint->minValue);
         ReturnExpression(newConstraint->maxValue);
         newConstraint->minValue = theMinList;
         newConstraint->maxValue = theMaxList;
        }
      else
        {
         ReturnExpression(newConstraint->minFields);
         ReturnExpression(newConstraint->maxFields);
         newConstraint->minFields = theMinList;
         newConstraint->maxFields = theMaxList;
        }
     }
   else
     {
      if (range)
        {
         if (newConstraint->anyAllowed) SetAnyAllowedFlags(newConstraint,CLIPS_FALSE);
         newConstraint->integersAllowed = CLIPS_FALSE;
         newConstraint->floatsAllowed = CLIPS_FALSE;
        }
      else
        {
         SetAnyAllowedFlags(newConstraint,CLIPS_TRUE);
         newConstraint->anyAllowed = CLIPS_TRUE;
        }
     }
  }   
  
/*************************************************************/
/* UnionSingleValueWithList:  */
/*************************************************************/
static VOID UnionSingleValueWithList(addmin,addmax,theMinList,theMaxList)
  struct expr *addmin, *addmax, **theMinList, **theMaxList;
  {
   struct expr *tmpmin, *tmpmax, *lastmin, *lastmax;
   struct expr *themin, *themax, *nextmin, *nextmax;
   int cmaxmin, cmaxmax, cminmin, cminmax;
     
   /*=========================================================*/
   /* If no values are on the lists, then use the new values. */
   /*=========================================================*/
   
   if (*theMinList == NULL)
     {
      *theMinList = GenConstant(addmin->type,addmin->value);
      *theMaxList = GenConstant(addmax->type,addmax->value);
      return;
     }
     
   lastmin = NULL;
   lastmax = NULL;
   tmpmin = (*theMinList);
   tmpmax = (*theMaxList);
   
   while (tmpmin != NULL)
     {
      cmaxmax = CompareNumbers(addmax->type,addmax->value,
                               tmpmax->type,tmpmax->value);
                           
      cminmin = CompareNumbers(addmin->type,addmin->value,
                               tmpmin->type,tmpmin->value);
                               
      cmaxmin = CompareNumbers(addmax->type,addmax->value,
                               tmpmin->type,tmpmin->value);
                           
      cminmax = CompareNumbers(addmin->type,addmin->value,
                               tmpmax->type,tmpmax->value);
                 
      /*=================================*/
      /* Check to see if the range is    */
      /* contained within another range. */
      /*=================================*/
                
      if (((cmaxmax == LESS_THAN) || (cmaxmax == EQUAL)) &&
          ((cminmin == GREATER_THAN) || (cminmin == EQUAL)))
        { return; }
        
      /*================================*/
      /* Extend the greater than range. */
      /*================================*/
        
      if ((cmaxmax == GREATER_THAN) && 
          ((cminmax == LESS_THAN) || (cminmax == EQUAL)))
        {
         tmpmax->type = addmax->type;
         tmpmax->value = addmax->value;
        }
        
      /*=============================*/
      /* Extend the less than range. */
      /*=============================*/
        
      if ((cminmin == LESS_THAN) && 
          ((cmaxmin == GREATER_THAN) || (cmaxmin == EQUAL)))
        {
         tmpmin->type = addmin->type;
         tmpmin->value = addmin->value;
        }
      
      /*====================*/
      /* Handle insertions. */
      /*====================*/
      
      if (cmaxmin == LESS_THAN)
        {
         if (lastmax == NULL)
           {
            themin = GenConstant(addmin->type,addmin->value);
            themax = GenConstant(addmax->type,addmax->value);
            themin->nextArg = *theMinList;
            themax->nextArg = *theMaxList;
            *theMinList = themin;
            *theMaxList = themax;
            return;
           }
           
         if (CompareNumbers(addmin->type,addmin->value,
                            lastmax->type,lastmax->value) == GREATER_THAN)
           {
            themin = GenConstant(addmin->type,addmin->value);
            themax = GenConstant(addmax->type,addmax->value);
            
            themin->nextArg = lastmin->nextArg;
            themax->nextArg = lastmax->nextArg;
            
            lastmin->nextArg = themin;
            lastmax->nextArg = themax;
            return;
           }
        }
        
      /*==========================*/
      /* Move on to the next one. */
      /*==========================*/
      
      tmpmin = tmpmin->nextArg;
      tmpmax = tmpmax->nextArg;
     }
     
   /*===========================*/
   /* Merge overlapping ranges. */
   /*===========================*/
     
   lastmin = NULL;
   lastmax = NULL;
   tmpmin = (*theMinList);
   tmpmax = (*theMaxList);
   
   while (tmpmin != NULL)
     {
      nextmin = tmpmin->nextArg;
      nextmax = tmpmax->nextArg;
      if (nextmin != NULL)
        {
         cmaxmin = CompareNumbers(tmpmax->type,tmpmax->value,
                                  nextmin->type,nextmin->value);
         if ((cmaxmin == GREATER_THAN) || (cmaxmin == EQUAL))
           {
            tmpmax->type = nextmax->type;
            tmpmax->value = nextmax->value;
            tmpmax->nextArg = nextmax->nextArg;
            tmpmin->nextArg = nextmin->nextArg;
            
            rtn_struct(expr,nextmin);
            rtn_struct(expr,nextmax);
           }
         else
           {
            tmpmin = tmpmin->nextArg;
            tmpmax = tmpmax->nextArg;
           }
        }
      else
        {
         tmpmin = nextmin;
         tmpmax = nextmax;
        }
     }
  }   

/*************************************************************/
/* UnionAllowedValueExpressions:  */
/*************************************************************/
static VOID UnionAllowedValueExpressions(constraint1,constraint2,newConstraint)
  CONSTRAINT_RECORD *constraint1, *constraint2, *newConstraint;
  {
   struct expr *theHead = NULL;
   
   theHead = AddToUnionList(constraint1->restrictionList,theHead,newConstraint);
   theHead = AddToUnionList(constraint2->restrictionList,theHead,newConstraint);
   
   newConstraint->restrictionList = theHead;
  }
    
/*************************************************************/
/* AddToUnionList:   */
/*************************************************************/
static struct expr *AddToUnionList(theList1,theHead,theConstraint)
  struct expr *theList1, *theHead;
  CONSTRAINT_RECORD *theConstraint;
  {
   struct expr *theList2;
   int flag;
   
   while (theList1 != NULL)
     {
      theList2 = theHead;
      flag = CLIPS_TRUE;
      while (theList2 != NULL)
        {
         if ((theList1->type == theList2->type) && 
             (theList1->value == theList2->value))
           { 
            flag = CLIPS_FALSE; 
            theList2 = NULL;
           }
         else
           { theList2 = theList2->nextArg; }
        }
        
      if (flag)
        {
         if (RestrictionOnType(theList1->type,theConstraint))
           {
            theList2 = GenConstant(theList1->type,theList1->value);
            theList2->nextArg = theHead;
            theHead = theList2;
           }
        }
        
      theList1 = theList1->nextArg;
     }
     
   return(theHead);
  }

/*************************************************************/
/* RemoveConstantFromConstraint:  */
/*************************************************************/
globle VOID RemoveConstantFromConstraint(theType,theValue,theConstraint)
  int theType;
  VOID *theValue;
  CONSTRAINT_RECORD *theConstraint;
  {
   struct expr *theList, *lastOne = NULL, *tmpList;
   
   if (theConstraint == NULL) return;
   
   theList = theConstraint->restrictionList;
   theConstraint->restrictionList = NULL;
   
   while (theList != NULL)
     {
      if ((theList->type != theType) || (theList->value != theValue))
        {
         if (lastOne == NULL)
           { theConstraint->restrictionList = theList; }
         else
           { lastOne->nextArg = theList; }
         lastOne = theList;
         theList = theList->nextArg;
         lastOne->nextArg = NULL;
        }
      else
        {
         tmpList = theList;
         theList = theList->nextArg;
         tmpList->nextArg = NULL;
         ReturnExpression(tmpList);
        }
     }
     
   UpdateRestrictionFlags(theConstraint);
  }

#endif

#endif


    

