   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*          FACT RETE ACCESS FUNCTIONS MODULE          */
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

#define _FACTRETE_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT && DEFRULE_CONSTRUCT

#include "clipsmem.h"
#include "extnfunc.h"
#include "router.h"
#if INCREMENTAL_RESET
#include "incrrset.h"
#endif
#include "reteutil.h"
#include "drive.h"
#include "factgen.h"
#include "factmch.h"
#include "factrete.h"

/************************************************************/
/* FactGetVarPNFunction1:  Extracts the nth element from the */
/*   current pattern being matched.                          */
/************************************************************/
globle BOOLEAN FactGetVarPNFunction1(theValue,returnValue)
  VOID *theValue;
  DATA_OBJECT_PTR returnValue;
  {
   int theField, theSlot;
   struct fact *factPtr = NULL;
   struct field *fieldPtr;
   struct multifieldMarker *marks = NULL;
   struct multifield *segmentPtr;
   int extent;
   struct factGetVarPN1Call *hack;

   /*==========================================*/
   /* Retrieve the arguments for the function. */
   /*==========================================*/
   
   hack = (struct factGetVarPN1Call *) ValueToBitMap(theValue);
   
   /*=====================================================*/
   /* Get the pointer to the fact from the partial match. */
   /*=====================================================*/

   factPtr = CurrentPatternFact;
   marks = CurrentPatternMarks;

   /*==========================================================*/
   /* Determine if we want to retrieve the fact address of the */
   /* fact, rather than retrieving a field from the fact.      */
   /*==========================================================*/

   if (hack->factAddress)
     {
      returnValue->type = FACT_ADDRESS;
      returnValue->value = (VOID *) factPtr;
      return(CLIPS_TRUE);
     }
     
   if (hack->allFields)
     {
      theSlot = hack->whichSlot;
      fieldPtr = &factPtr->theProposition.theFields[theSlot];
      returnValue->type = fieldPtr->type;
      returnValue->value = fieldPtr->value;
      if (returnValue->type == MULTIFIELD)
        {
         returnValue->begin = 0;
         returnValue->end = ((struct multifield *) fieldPtr->value)->multifieldLength - 1;
        }
        
      return(CLIPS_TRUE);
     }
   
   /*====================================================*/
   /* If the slot being accessed is a single field slot, */
   /* then just return the single value found in that    */
   /* slot. The multifieldMarker data structures do not  */
   /* have to be considered since access to a single     */
   /* field slot is not affected by variable bindings    */
   /* from multifield slots.                             */
   /*====================================================*/
   
   theField = hack->whichField;
   theSlot = hack->whichSlot;
   fieldPtr = &factPtr->theProposition.theFields[theSlot];

   /*==========================================================*/
   /* Retrieve a value from a multifield slot. First determine */
   /* the range of fields for the variable being retrieved.    */
   /*==========================================================*/
   
   extent = -1;
   theField = AdjustFieldPosition(marks,theField,theSlot,&extent); 
   
   /*=============================================================*/
   /* If a range of values are being retrieved (i.e. a multifield */
   /* variable), then return the values as a multifield.          */
   /*=============================================================*/
   
   if (extent != -1)
     {
      returnValue->type = MULTIFIELD;
      returnValue->value = (VOID *) fieldPtr->value;
      returnValue->begin = theField;
      returnValue->end = theField + extent - 1;
      return(CLIPS_TRUE);
     }

   /*========================================================*/
   /* Otherwise a single field value is being retrieved from */
   /* a multifield slot. Just return the type and value.     */
   /*========================================================*/
   
   segmentPtr = (struct multifield *) fieldPtr->value;
   fieldPtr = &segmentPtr->theFields[theField];

   returnValue->type = fieldPtr->type;
   returnValue->value = fieldPtr->value;
   
   return(CLIPS_TRUE);
  }
  
/***********************************************************************/
/* FactGetVarPNFunction2: Used to extract a value from the nth field of a */
/*   template fact. The field extracted is from a single field slot.   */
/***********************************************************************/
globle BOOLEAN FactGetVarPNFunction2(theValue,returnValue)
  VOID *theValue;
  DATA_OBJECT_PTR returnValue;
  {
   struct fact *factPtr = NULL;
   struct factGetVarPN2Call *hack;
   struct field *fieldPtr;

   /*==========================================*/
   /* Retrieve the arguments for the function. */
   /*==========================================*/
   
   hack = (struct factGetVarPN2Call *) ValueToBitMap(theValue);
   
   /*==============================*/
   /* Get the pointer to the fact. */
   /*==============================*/

   factPtr = CurrentPatternFact;
   
   /*============================================*/
   /* Extract the value from the specified slot. */
   /*============================================*/
   
   fieldPtr = &factPtr->theProposition.theFields[hack->whichSlot];

   returnValue->type = fieldPtr->type;
   returnValue->value = fieldPtr->value;
   
   return(CLIPS_TRUE);
  }

/******************************************************************/
/* FactGetVarPNFunction3: Used to extract a value from the nth field */
/*   of a template fact. The nth field must be a multifield slot  */
/*   that contains at most one multifield variable.               */
/******************************************************************/
globle BOOLEAN FactGetVarPNFunction3(theValue,returnValue)
  VOID *theValue;
  DATA_OBJECT_PTR returnValue;
  {
   struct fact *factPtr = NULL;
   struct multifield *segmentPtr;
   struct field *fieldPtr;
   struct factGetVarPN3Call *hack;

   /*==========================================*/
   /* Retrieve the arguments for the function. */
   /*==========================================*/
   
   hack = (struct factGetVarPN3Call *) ValueToBitMap(theValue);
   
   /*==============================*/
   /* Get the pointer to the fact. */
   /*==============================*/

   factPtr = CurrentPatternFact;
      
   /*============================================================*/
   /* Get the multifield value from which the data is retrieved. */
   /*============================================================*/
   
   segmentPtr = (struct multifield *) factPtr->theProposition.theFields[hack->whichSlot].value;

   /*=========================================*/
   /* If the beginning and end flags are set, */ 
   /* then retrieve a multifield value.       */
   /*=========================================*/
   
   if (hack->fromBeginning && hack->fromEnd)
     {
      returnValue->type = MULTIFIELD;
      returnValue->value = (VOID *) segmentPtr;
      returnValue->begin = hack->beginOffset;
      returnValue->end = segmentPtr->multifieldLength - (hack->endOffset + 1);
      return(CLIPS_TRUE);
     }
     
   /*=====================================================*/
   /* Return a single field value from a multifield slot. */
   /*=====================================================*/
   
   if (hack->fromBeginning)
     { fieldPtr = &segmentPtr->theFields[hack->beginOffset]; }
   else
     { fieldPtr = &segmentPtr->theFields[segmentPtr->multifieldLength - (hack->endOffset + 1)]; }
   
   returnValue->type = fieldPtr->type;
   returnValue->value = fieldPtr->value;
   
   return(CLIPS_TRUE);
  }
  
/*************************************************************************/
/* FactConstantPNFunction2: Used to compare the value stored in a single */
/*   field slot to a constant for either equality or inequality.         */
/*************************************************************************/
#if IBM_TBC
#pragma argsused
#endif
globle BOOLEAN FactConstantPNFunction2(theValue,returnValue)
  VOID *theValue;
  DATA_OBJECT_PTR returnValue;
  {
#if MAC_MPW
#pragma unused(returnValue)
#endif
   struct factConstantPN2Call *hack;
   struct field *fieldPtr;
   struct expr *theConstant;

   /*==========================================*/
   /* Retrieve the arguments for the function. */
   /*==========================================*/
   
   hack = (struct factConstantPN2Call *) ValueToBitMap(theValue);

   /*============================================*/
   /* Extract the value from the specified slot. */
   /*============================================*/
   
   fieldPtr = &CurrentPatternFact->theProposition.theFields[hack->whichSlot];
   
   /*====================================*/
   /* Compare the value to the constant. */
   /*====================================*/
   
   theConstant = GetFirstArgument();
   if (theConstant->type != fieldPtr->type) return(1 - hack->testForEquality);
   if (theConstant->value != fieldPtr->value) return(1 - hack->testForEquality);
   return(hack->testForEquality);
  }
  
/************************************************************************/
/* FactConstantPNFunction4: Used to compare a value contained within a  */
/*   multifield slot to a constant for equality or inequality. The      */
/*   value being retrieved from the slot has no multifields to its      */
/*   right (thus it can be retrieved relative to the beginning.         */
/************************************************************************/
#if IBM_TBC
#pragma argsused
#endif
globle BOOLEAN FactConstantPNFunction4(theValue,returnValue)
  VOID *theValue;
  DATA_OBJECT_PTR returnValue;
  {
#if MAC_MPW
#pragma unused(returnValue)
#endif
   struct factConstantPN4Call *hack;
   struct field *fieldPtr;
   struct expr *theConstant;
   struct multifield *segmentPtr;

   /*==========================================*/
   /* Retrieve the arguments for the function. */
   /*==========================================*/
   
   hack = (struct factConstantPN4Call *) ValueToBitMap(theValue);
   
   /*==========================================================*/
   /* Extract the value from the specified slot. Note that the */
   /* test to determine the slot's type (multifield) should be */
   /* unnecessary since this routine should only be used for   */
   /* multifield slots.                                        */
   /*==========================================================*/
   
   fieldPtr = &CurrentPatternFact->theProposition.theFields[hack->whichSlot];
   
   if (fieldPtr->type == MULTIFIELD)
     {
      segmentPtr = (struct multifield *) fieldPtr->value;

      if (hack->fromBeginning)
        { fieldPtr = &segmentPtr->theFields[hack->offset]; }
      else
        { 
         fieldPtr = &segmentPtr->theFields[segmentPtr->multifieldLength - 
                    (hack->offset + 1)]; 
        }
     }
   
   /*====================================*/
   /* Compare the value to the constant. */
   /*====================================*/
   
   theConstant = GetFirstArgument();
   if (theConstant->type != fieldPtr->type) return(1 - hack->testForEquality);
   if (theConstant->value != fieldPtr->value) return(1 - hack->testForEquality);
   return(hack->testForEquality);
  }
  
/**************************************************************/
/* FactGetVarJNFunction1:  Extracts the nth element from the mth */
/*   pattern of a rule.                                       */
/**************************************************************/
globle BOOLEAN FactGetVarJNFunction1(theValue,returnValue)
  VOID *theValue;
  DATA_OBJECT_PTR returnValue;
  {
   int theField, theSlot;
   struct fact *factPtr = NULL;
   struct field *fieldPtr;
   struct multifieldMarker *marks = NULL;
   struct multifield *segmentPtr;
   int extent;
   struct factGetVarJN1Call *hack;

   /*==========================================*/
   /* Retrieve the arguments for the function. */
   /*==========================================*/
   
   hack = (struct factGetVarJN1Call *) ValueToBitMap(theValue);
   
   /*=====================================================*/
   /* Get the pointer to the fact from the partial match. */
   /*=====================================================*/
     
   if (GlobalRHSBinds == NULL)
     { 
      factPtr = (struct fact *) get_nth_pm_match(GlobalLHSBinds,hack->whichPattern)->matchingItem;
      marks = get_nth_pm_match(GlobalLHSBinds,hack->whichPattern)->markers;
     }
   else if ((GlobalJoin->depth - 1) == hack->whichPattern)
     { 
      factPtr = (struct fact *) get_nth_pm_match(GlobalRHSBinds,0)->matchingItem;
      marks = get_nth_pm_match(GlobalRHSBinds,0)->markers;
     }
   else
     {
      factPtr = (struct fact *) get_nth_pm_match(GlobalLHSBinds,hack->whichPattern)->matchingItem;
      marks = get_nth_pm_match(GlobalLHSBinds,hack->whichPattern)->markers;
     }

   /*==========================================================*/
   /* Determine if we want to retrieve the fact address of the */
   /* fact, rather than retrieving a field from the fact.      */
   /*==========================================================*/

   if (hack->factAddress)
     {
      returnValue->type = FACT_ADDRESS;
      returnValue->value = (VOID *) factPtr;
      return(CLIPS_TRUE);
     }
     
   if (hack->allFields)
     {
      theSlot = hack->whichSlot;
      fieldPtr = &factPtr->theProposition.theFields[theSlot];
      returnValue->type = fieldPtr->type;
      returnValue->value = fieldPtr->value;
      if (returnValue->type == MULTIFIELD)
        {
         returnValue->begin = 0;
         returnValue->end = ((struct multifield *) fieldPtr->value)->multifieldLength - 1;
        }
        
      return(CLIPS_TRUE);
     }
   
   /*====================================================*/
   /* If the slot being accessed is a single field slot, */
   /* then just return the single value found in that    */
   /* slot. The multifieldMarker data structures do not  */
   /* have to be considered since access to a single     */
   /* field slot is not affected by variable bindings    */
   /* from multifield slots.                             */
   /*====================================================*/
   
   theField = hack->whichField;
   theSlot = hack->whichSlot;
   fieldPtr = &factPtr->theProposition.theFields[theSlot];

   if (fieldPtr->type != MULTIFIELD)
     {
      returnValue->type = fieldPtr->type;
      returnValue->value = fieldPtr->value;
      return(CLIPS_TRUE);
     }

   /*==========================================================*/
   /* Retrieve a value from a multifield slot. First determine */
   /* the range of fields for the variable being retrieved.    */
   /*==========================================================*/
   
   extent = -1;
   theField = AdjustFieldPosition(marks,theField,theSlot,&extent); 
   
   /*=============================================================*/
   /* If a range of values are being retrieved (i.e. a multifield */
   /* variable), then return the values as a multifield.          */
   /*=============================================================*/
   
   if (extent != -1)
     {
      returnValue->type = MULTIFIELD;
      returnValue->value = (VOID *) fieldPtr->value;
      returnValue->begin = theField;
      returnValue->end = theField + extent - 1;
      return(CLIPS_TRUE);
     }

   /*========================================================*/
   /* Otherwise a single field value is being retrieved from */
   /* a multifield slot. Just return the type and value.     */
   /*========================================================*/
   
   segmentPtr = (struct multifield *) factPtr->theProposition.theFields[theSlot].value;
   fieldPtr = &segmentPtr->theFields[theField];

   returnValue->type = fieldPtr->type;
   returnValue->value = fieldPtr->value;
   
   return(CLIPS_TRUE);
  }
  
/***********************************************************************/
/* FactGetVarJNFunction2: Used to extract a value from the nth field of a */
/*   template fact. The field extracted is from a single field slot.   */
/***********************************************************************/
globle BOOLEAN FactGetVarJNFunction2(theValue,returnValue)
  VOID *theValue;
  DATA_OBJECT_PTR returnValue;
  {
   struct fact *factPtr = NULL;
   struct factGetVarJN2Call *hack;
   struct field *fieldPtr;

   /*==========================================*/
   /* Retrieve the arguments for the function. */
   /*==========================================*/
   
   hack = (struct factGetVarJN2Call *) ValueToBitMap(theValue);
   
   /*=====================================================*/
   /* Get the pointer to the fact from the partial match. */
   /*=====================================================*/

   if (GlobalRHSBinds == NULL)
     { factPtr = (struct fact *) get_nth_pm_match(GlobalLHSBinds,hack->whichPattern)->matchingItem; }
   else if ((GlobalJoin->depth - 1) == hack->whichPattern)
     { factPtr = (struct fact *) get_nth_pm_match(GlobalRHSBinds,0)->matchingItem; }
   else
     { factPtr = (struct fact *) get_nth_pm_match(GlobalLHSBinds,hack->whichPattern)->matchingItem; }

   /*============================================*/
   /* Extract the value from the specified slot. */
   /*============================================*/
   
   fieldPtr = &factPtr->theProposition.theFields[hack->whichSlot];

   returnValue->type = fieldPtr->type;
   returnValue->value = fieldPtr->value;
   
   return(CLIPS_TRUE);
  }
  
/******************************************************************/
/* FactGetVarJNFunction3: Used to extract a value from the nth field */
/*   of a template fact. The nth field must be a multifield slot  */
/*   that contains at most one multifield variable.               */
/******************************************************************/
globle BOOLEAN FactGetVarJNFunction3(theValue,returnValue)
  VOID *theValue;
  DATA_OBJECT_PTR returnValue;
  {
   struct fact *factPtr = NULL;
   struct multifield *segmentPtr;
   struct field *fieldPtr;
   struct factGetVarJN3Call *hack;

   /*==========================================*/
   /* Retrieve the arguments for the function. */
   /*==========================================*/
   
   hack = (struct factGetVarJN3Call *) ValueToBitMap(theValue);
   
   /*=====================================================*/
   /* Get the pointer to the fact from the partial match. */
   /*=====================================================*/

   if (GlobalRHSBinds == NULL)
     { factPtr = (struct fact *) get_nth_pm_match(GlobalLHSBinds,hack->whichPattern)->matchingItem; }
   else if ((GlobalJoin->depth - 1) == hack->whichPattern)
     { factPtr = (struct fact *) get_nth_pm_match(GlobalRHSBinds,0)->matchingItem; }
   else
     { factPtr = (struct fact *) get_nth_pm_match(GlobalLHSBinds,hack->whichPattern)->matchingItem; }
   
   /*============================================================*/
   /* Get the multifield value from which the data is retrieved. */
   /*============================================================*/
   
   segmentPtr = (struct multifield *) factPtr->theProposition.theFields[hack->whichSlot].value;

   /*=========================================*/
   /* If the beginning and end flags are set, */ 
   /* then retrieve a multifield value.       */
   /*=========================================*/
   
   if (hack->fromBeginning && hack->fromEnd)
     {
      returnValue->type = MULTIFIELD;
      returnValue->value = (VOID *) segmentPtr;
      returnValue->begin = hack->beginOffset;
      returnValue->end = segmentPtr->multifieldLength - (hack->endOffset + 1);
      return(CLIPS_TRUE);
     }
     
   /*=====================================================*/
   /* Return a single field value from a multifield slot. */
   /*=====================================================*/
   
   if (hack->fromBeginning)
     { fieldPtr = &segmentPtr->theFields[hack->beginOffset]; }
   else
     { fieldPtr = &segmentPtr->theFields[segmentPtr->multifieldLength - (hack->endOffset + 1)]; }
   
   returnValue->type = fieldPtr->type;
   returnValue->value = fieldPtr->value;
   
   return(CLIPS_TRUE);
  }
  
/************************************************************/
/* FactSlotLengthTestFunction: Used to determine that the length */
/*  of a multifield slot falls within a specified range.    */
/************************************************************/
globle BOOLEAN FactSlotLengthTestFunction(theValue,returnValue)
  VOID *theValue;
  DATA_OBJECT_PTR returnValue;
  {   
   struct factCheckLengthPNCall *hack;
   struct multifield *segmentPtr;
   
   returnValue->type = SYMBOL;
   returnValue->value = CLIPSFalseSymbol;
   
   hack = (struct factCheckLengthPNCall *) ValueToBitMap(theValue);
      
   segmentPtr = (struct multifield *) CurrentPatternFact->theProposition.theFields[hack->whichSlot].value;

   if (segmentPtr->multifieldLength < hack->minLength) 
     { return(CLIPS_FALSE); }
   
   if (hack->exactly && (segmentPtr->multifieldLength > hack->minLength))
     { return(CLIPS_FALSE); }
     
   returnValue->value = CLIPSTrueSymbol;
   return(CLIPS_TRUE);
  }
  
/**************************************************/
/* FactCompVarsJNFunction1:              */
/**************************************************/
#if IBM_TBC
#pragma argsused
#endif
globle int FactCompVarsJNFunction1(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
#if MAC_MPW
#pragma unused(theResult)
#endif
   int p1, e1, p2, e2;
   struct fact *fact1, *fact2;
   struct multifield *segment1, *segment2;
   int i, type1, type2;
   VOID *value1, *value2;
   struct factCompVarsJN1Call *hack;
         
   /*==============================================*/
   /* Determine which elements are to be extracted */
   /* out of which patterns.                       */
   /*==============================================*/

   hack = (struct factCompVarsJN1Call *) ValueToBitMap(theValue);

   p1 = GlobalJoin->depth - 1;
   e1 = (int) hack->slot1;
   p2 = ((int) hack->pattern2) - 1;
   e2 = (int) hack->slot2;

   /*==============================================*/
   /* Extract the fact pointer and list of segment */
   /* markers for the first pattern.         */
   /*==============================================*/

   if ((p1 > 0) || (GlobalLHSBinds == NULL))
     { fact1 = (struct fact *) GlobalRHSBinds->binds[0].gm.theMatch->matchingItem; }
   else
     { fact1 = (struct fact *) GlobalLHSBinds->binds[p1].gm.theMatch->matchingItem; }

   /*==============================================*/
   /* Extract the fact pointer and list of segment */
   /* markers for the second pattern.        */
   /*==============================================*/

   if (p2 == p1) fact2 = fact1;
   else fact2 = (struct fact *) GlobalLHSBinds->binds[p2].gm.theMatch->matchingItem;

   /*================================================*/
   /* Get the type and value from the first pattern. */
   /*================================================*/
   
   if (e1 > 0)
     { 
      type1 = fact1->theProposition.theFields[e1].type;
      value1 = fact1->theProposition.theFields[e1].value;
     }
   else 
     {
      type1 = FACT_ADDRESS;
      value1 = (VOID *) fact1;
     }
     
   /*=================================================*/
   /* Get the type and value from the second pattern. */
   /*=================================================*/
   
   if (e2 > 0)
     { 
      type2 = fact2->theProposition.theFields[e2].type;
      value2 = fact2->theProposition.theFields[e2].value;
     }
   else 
     {
      type2 = FACT_ADDRESS;
      value2 = (VOID *) fact2;
     }
     
   /*=======================================================*/
   /* If the types don't match, then return the fail value. */
   /*=======================================================*/

   if (type1 != type2) return((int) hack->fail);
   
   /*================================================================*/
   /* If the types aren't multifields, then just compare the values. */
   /*================================================================*/
   
   if (type1 != MULTIFIELD)
     {
      if (value1 != value2) return((int) hack->fail);
      return((int) hack->pass);
     }
     
   /*============================*/
   /* Compare multifield values. */
   /*============================*/
        
   segment1 = (struct multifield *) value1;
   segment2 = (struct multifield *) value2;
      
   if (segment1->multifieldLength != segment2->multifieldLength)
     { return((int) hack->fail); }
        
   for (i = 0; i < segment1->multifieldLength; i++)
     {
      if ((segment1->theFields[i].type != segment2->theFields[i].type) ||
          (segment1->theFields[i].value != segment2->theFields[i].value)) 
        { return((int) hack->fail); }
     }
        
   return((int) hack->pass);
  }
  
/**************************************************/
/* FactCompVarsJNFunction2: Used to compare two single field slots */
/*   of a deftemplate.             */
/**************************************************/
#if IBM_TBC
#pragma argsused
#endif
globle int FactCompVarsJNFunction2(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
#if MAC_MPW
#pragma unused(theResult)
#endif
   int p1, e1, p2, e2;
   struct fact *fact1, *fact2;
   struct factCompVarsJN2Call *hack;
         
   /*=========================================*/
   /* Retrieve the arguments to the function. */
   /*=========================================*/

   hack = (struct factCompVarsJN2Call *) ValueToBitMap(theValue);

   /*=================================================*/
   /* Extract the fact pointers for the two patterns. */
   /*=================================================*/

   p1 = GlobalJoin->depth - 1;
   p2 = ((int) hack->pattern2) - 1;
   
   fact1 = (struct fact *) GlobalRHSBinds->binds[0].gm.theMatch->matchingItem;
   if (p1 != p2)
     { fact2 = (struct fact *) GlobalLHSBinds->binds[p2].gm.theMatch->matchingItem; }
   else
     { fact2 = fact1; }

   /*=====================*/
   /* Compare the values. */
   /*=====================*/
   
   e1 = (int) hack->slot1;
   e2 = (int) hack->slot2;
   
   if (fact1->theProposition.theFields[e1].type !=
       fact2->theProposition.theFields[e2].type)
     { return((int) hack->fail); }
     
   if (fact1->theProposition.theFields[e1].value !=
       fact2->theProposition.theFields[e2].value)
     { return((int) hack->fail); }
        
   return((int) hack->pass);
  }
  
/**************************************************/
/* FactCompVarsJNFunction3: Used to compare two single field values */
/*   found in the 1st slot (which must be a multifield slot) */
/*   of a deftemplate.             */
/**************************************************/
#if IBM_TBC
#pragma argsused
#endif
globle int FactCompVarsJNFunction3(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
#if MAC_MPW
#pragma unused(theResult)
#endif
   int p1, s1, p2, s2;
   struct fact *fact1, *fact2;
   struct factCompVarsJN3Call *hack;
   struct multifield *segment;
   struct field *fieldPtr1, *fieldPtr2;
         
   /*=========================================*/
   /* Retrieve the arguments to the function. */
   /*=========================================*/

   hack = (struct factCompVarsJN3Call *) ValueToBitMap(theValue);

   /*=================================================*/
   /* Extract the fact pointers for the two patterns. */
   /*=================================================*/

   p1 = GlobalJoin->depth - 1;
   p2 = ((int) hack->pattern2) - 1;
   s1 = (int) hack->slot1;
   s2 = (int) hack->slot2;
   
   fact1 = (struct fact *) GlobalRHSBinds->binds[0].gm.theMatch->matchingItem;
   if (p1 != p2)
     { fact2 = (struct fact *) GlobalLHSBinds->binds[p2].gm.theMatch->matchingItem; }
   else
     { fact2 = fact1; }

   /*======================*/
   /* Retrieve the values. */
   /*======================*/
   
   if (fact1->theProposition.theFields[s1].type != MULTIFIELD)
     { fieldPtr1 = &fact1->theProposition.theFields[s1]; }
   else
     {
      segment = (struct multifield *) fact1->theProposition.theFields[s1].value;
      
      if (hack->fromBeginning1)
        { fieldPtr1 = &segment->theFields[hack->offset1]; }
      else
        { fieldPtr1 = &segment->theFields[segment->multifieldLength - (hack->offset1 + 1)]; }
     }
   
   if (fact2->theProposition.theFields[s2].type != MULTIFIELD)
     { fieldPtr2 = &fact2->theProposition.theFields[s2]; }
   else
     {
      segment = (struct multifield *) fact2->theProposition.theFields[s2].value;
      
      if (hack->fromBeginning2)
        { fieldPtr2 = &segment->theFields[hack->offset2]; }
      else
        { fieldPtr2 = &segment->theFields[segment->multifieldLength - (hack->offset2 + 1)]; }
     }
   
   /*=====================*/
   /* Compare the values. */
   /*=====================*/
   
   if (fieldPtr1->type != fieldPtr2->type)
     { return((int) hack->fail); }
     
   if (fieldPtr1->value != fieldPtr2->value)
     { return((int) hack->fail); }
        
   return((int) hack->pass);
  }

/*****************************************************/
/* FactCompVarsPNFunction2: Used to compare two single field */
/*   slots of a deftemplate in the pattern network.  */
/*****************************************************/
globle int FactCompVarsPNFunction2(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   int rv;
   struct field *fieldPtr1, *fieldPtr2;
   struct factCompVarsPN2Call *hack;
     
   /*========================================*/
   /* Extract the arguments to the function. */
   /*========================================*/

   hack = (struct factCompVarsPN2Call *) ValueToBitMap(theValue);
   fieldPtr1 = &CurrentPatternFact->theProposition.theFields[hack->field1];
   fieldPtr2 = &CurrentPatternFact->theProposition.theFields[hack->field2];

   /*=====================*/
   /* Compare the values. */
   /*=====================*/
   
   if (fieldPtr1->type != fieldPtr2->type) rv = (int) hack->fail;
   else if (fieldPtr1->value != fieldPtr2->value) rv = (int) hack->fail;
   else rv = (int) hack->pass;
        
   theResult->type = SYMBOL;
   if (rv) theResult->value = CLIPSTrueSymbol;
   else theResult->value = CLIPSFalseSymbol;
   
   return(rv);
  }
  
/************************************************************************/
/* AdjustFieldPosition:  Given a list of segment markers and the index  */
/*   to a variable in a pattern, this function computes the index       */
/*   to the element in the fact where the variable begins.  In the      */
/*   case of segment variables, it also computes the extent (or length) */
/*   of the segment.  Note that the extent should be given a default    */
/*   value of either -1 or 1 for variables other than segment variables */
/*   before calling this routine.  An extent of -1 for these variables  */
/*   will distinguish their extent as being different when it is        */
/*   necessary to note their difference from a segment variable with an */
/*   extent of 1. For example, given the pattern (data $?x c $?y ?z)    */
/*   and the fact (data a b c d e f x), the actual index in the fact    */
/*   for the 5th item in the pattern (the variable ?z) would be 8 since */
/*   $?x binds to 2 fields and $?y binds to 3 fields.                   */
/************************************************************************/
globle int AdjustFieldPosition(markList,whichField,whichSlot,extent)
  struct multifieldMarker *markList;
  int whichField, whichSlot, *extent;
  {
   int actualIndex;

   actualIndex = whichField;
   while (markList != NULL)
     {
      if (markList->where.whichSlotNumber == whichSlot)
        {
         if (markList->whichField == whichField)
           {
            *extent = (markList->endPosition - markList->startPosition) + 1;
            return(actualIndex);
           }
         else if (markList->whichField > whichField)
           { return(actualIndex); }

         actualIndex += (markList->endPosition - markList->startPosition);
        }
        
      markList = markList->next;
     }

   return(actualIndex);
  }
  
/**************************************************/
/* SCallStoreMultifield:              */
/**************************************************/
#if IBM_TBC
#pragma argsused
#endif
globle int SCallStoreMultifield(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {  
#if MAC_MPW
#pragma unused(theValue)
#endif
   StoreInMultifield(theResult,GetFirstArgument(),CLIPS_FALSE);
   return(CLIPS_TRUE);
  }
  
#endif

