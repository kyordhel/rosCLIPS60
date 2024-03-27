   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*          FACT RETE FUNCTION GENERATION MODULE       */
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

#define _FACTGEN_SOURCE_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT && DEFRULE_CONSTRUCT

#include <stdio.h>
#define _CLIPS_STDIO_

#include "constant.h"
#include "clipsmem.h"
#include "router.h"
#include "scanner.h"
#include "exprnpsr.h"
#include "constrct.h"
#include "network.h"
#include "reteutil.h"
#include "factmch.h"
#include "factrete.h"
#include "factmngr.h"
#include "pattern.h"
#include "factprt.h"

#include "tmpltdef.h"
#include "tmpltlhs.h"
   
#include "factgen.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
#if (! RUN_TIME) && (! BLOAD_ONLY)
   static VOID                      *FactGetVarJN1(struct lhsParseNode *);
   static VOID                      *FactGetVarJN2(struct lhsParseNode *);
   static VOID                      *FactGetVarJN3(struct lhsParseNode *);
   static VOID                      *FactGetVarPN1(struct lhsParseNode *);
   static VOID                      *FactGetVarPN2(struct lhsParseNode *);
   static VOID                      *FactGetVarPN3(struct lhsParseNode *);
#endif
#else
#if (! RUN_TIME) && (! BLOAD_ONLY)
   static VOID                      *FactGetVarJN1();
   static VOID                      *FactGetVarJN2();
   static VOID                      *FactGetVarJN3();
   static VOID                      *FactGetVarPN1();
   static VOID                      *FactGetVarPN2();
   static VOID                      *FactGetVarPN3();
#endif
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/
                                             
   globle struct entityRecord   FactGVInfo1 = { SCALL_GET_VAR1,0,1,0,
                                                PrintFactGetVarJN1,
                                                PrintFactGetVarJN1,NULL, 
                                                FactGetVarJNFunction1,
                                                NULL,NULL,NULL,NULL,NULL,NULL };
                                               
   globle struct entityRecord   FactGVInfo2 = { SCALL_GET_VAR2,0,1,0,
                                                PrintFactGetVarJN2,
                                                PrintFactGetVarJN2,NULL, 
                                                FactGetVarJNFunction2,
                                                NULL,NULL,NULL,NULL,NULL,NULL };
                                                
   globle struct entityRecord   FactGVInfo3 = { SCALL_GET_VAR3,0,1,0,
                                                PrintFactGetVarJN3,
                                                PrintFactGetVarJN3,NULL, 
                                                FactGetVarJNFunction3,
                                                NULL,NULL,NULL,NULL,NULL,NULL };
                                               
   globle struct entityRecord   FactGVPN1Info = { FACT_GETVAR_PN1,0,1,0,
                                                  PrintFactGetVarPN1,
                                                  PrintFactGetVarPN1,NULL, 
                                                  FactGetVarPNFunction1,
                                                  NULL,NULL,NULL,NULL,NULL,NULL };
                                                  
   globle struct entityRecord   FactGVPN2Info = { FACT_GETVAR_PN2,0,1,0,
                                                  PrintFactGetVarPN2,
                                                  PrintFactGetVarPN2,NULL, 
                                                  FactGetVarPNFunction2,
                                                  NULL,NULL,NULL,NULL,NULL,NULL };
                                                  
   globle struct entityRecord   FactGVPN3Info = { FACT_GETVAR_PN3,0,1,0,
                                                  PrintFactGetVarPN3,
                                                  PrintFactGetVarPN3,NULL, 
                                                  FactGetVarPNFunction3,
                                                  NULL,NULL,NULL,NULL,NULL,NULL };
                                               
   globle struct entityRecord   FactJNCVInfo1 = { SCALL_CMP_JN_VARS1,0,1,1,
                                                  PrintFactJNCompVars1,
                                                  PrintFactJNCompVars1,NULL,
                                                  FactCompVarsJNFunction1,
                                                  NULL,NULL,NULL,NULL,NULL,NULL };
                                               
   globle struct entityRecord   FactJNCVInfo2 = { SCALL_CMP_JN_VARS2,0,1,1,
                                                  PrintFactJNCompVars2,
                                                  PrintFactJNCompVars2,NULL,
                                                  FactCompVarsJNFunction2,
                                                  NULL,NULL,NULL,NULL,NULL,NULL };
                                               
   globle struct entityRecord   FactJNCVInfo3 = { SCALL_CMP_JN_VARS3,0,1,1,
                                                  PrintFactJNCompVars3,
                                                  PrintFactJNCompVars3,NULL,
                                                  FactCompVarsJNFunction3,
                                                  NULL,NULL,NULL,NULL,NULL,NULL };
                                               
   globle struct entityRecord   FactPNCVInfo2 = { SCALL_CMP_PN_VARS2,0,1,1,
                                                  PrintFactPNCompVars2,
                                                  PrintFactPNCompVars2,NULL,
                                                  FactCompVarsPNFunction2,
                                                  NULL,NULL,NULL,NULL,NULL,NULL };
                                                
   globle struct entityRecord   StoreMFInfo = { SCALL_STORE_MULTIFIELD,0,1,0,
                                                NULL,NULL,NULL,
                                                SCallStoreMultifield,
                                                NULL,NULL,NULL,NULL,NULL,NULL };
                                                
   globle struct entityRecord   LengthTestInfo = { SCALL_LENGTH_TEST,0,1,0,
                                                   PrintSlotLengthTest,
                                                   PrintSlotLengthTest,NULL,
                                                   FactSlotLengthTestFunction,
                                                   NULL,NULL,NULL,NULL,NULL,NULL };
                                                   
   globle struct entityRecord   FactPNConstantInfo2 = { SCALL_PN_CONSTANT2,0,1,1,
                                                       PrintPNConstant2,
                                                       PrintPNConstant2,NULL,
                                                       FactConstantPNFunction2,
                                                       NULL,NULL,NULL,NULL,NULL,NULL };
                                                   
   globle struct entityRecord   FactPNConstantInfo4 = { SCALL_PN_CONSTANT4,0,1,1,
                                                       PrintPNConstant4,
                                                       PrintPNConstant4,NULL,
                                                       FactConstantPNFunction4,
                                                       NULL,NULL,NULL,NULL,NULL,NULL };

/********************************************************************/
/* InitializeFactReteFunctions:                         */
/********************************************************************/
globle VOID InitializeFactReteFunctions()
  {
#if DEFRULE_CONSTRUCT
   InstallPrimitive((ENTITY_RECORD_PTR) &FactInfo,FACT_ADDRESS);
   InstallPrimitive(&FactGVInfo1,SCALL_GET_VAR1);
   InstallPrimitive(&FactGVInfo2,SCALL_GET_VAR2);
   InstallPrimitive(&FactGVInfo3,SCALL_GET_VAR3);
   InstallPrimitive(&FactGVPN1Info,FACT_GETVAR_PN1);
   InstallPrimitive(&FactGVPN2Info,FACT_GETVAR_PN2);
   InstallPrimitive(&FactGVPN3Info,FACT_GETVAR_PN3);
   InstallPrimitive(&FactJNCVInfo1,SCALL_CMP_JN_VARS1);
   InstallPrimitive(&FactJNCVInfo2,SCALL_CMP_JN_VARS2);
   InstallPrimitive(&FactJNCVInfo3,SCALL_CMP_JN_VARS3);
   InstallPrimitive(&FactPNCVInfo2,SCALL_CMP_PN_VARS2);
   InstallPrimitive(&StoreMFInfo,SCALL_STORE_MULTIFIELD);
   InstallPrimitive(&LengthTestInfo,SCALL_LENGTH_TEST); 
   InstallPrimitive(&FactPNConstantInfo2,SCALL_PN_CONSTANT2);
   InstallPrimitive(&FactPNConstantInfo4,SCALL_PN_CONSTANT4);
#endif
  }

#if (! RUN_TIME) && (! BLOAD_ONLY)

/******************************************************************/
/* FactGenPNConstant: Generates an expression for use in the fact */
/*   pattern network that compares a field from a single field or */
/*   multifield slot against a constant.                          */
/******************************************************************/
globle struct expr *FactGenPNConstant(theField)
  struct lhsParseNode *theField;
  {
   struct expr *top;
   int tempValue;
   struct factConstantPN2Call hack2;
   struct factConstantPN4Call hack4;
   
   /*=================================================================*/
   /* If the value of a single field slot (or relation name) is being */
   /* compared against a constant, then use specialized routines for  */
   /* doing the comparison.                                           */
   /*=================================================================*/
   
   if (theField->withinMultifieldSlot == CLIPS_FALSE)
     {   
      ClearBitString(&hack2,sizeof(struct factConstantPN2Call));

      if (theField->negated) hack2.testForEquality = CLIPS_FALSE;
      else hack2.testForEquality = CLIPS_TRUE;
   
      hack2.whichSlot = theField->slotNumber - 1;
   
      top = GenConstant(SCALL_PN_CONSTANT2,AddBitMap(&hack2,sizeof(struct factConstantPN2Call)));

      top->argList = GenConstant(theField->type,theField->value);
      
      return(top);
     }

   /*=================================================================*/
   /* If a constant comparison is being done within a multifield slot */
   /* and the constant's position has no multifields to the left,     */
   /* then use the same routine used for the single field slot case,  */
   /* but include the offset from the beginning of the slot.          */
   /*=================================================================*/

   else if ((theField->multiFieldsBefore == 0) ||
            ((theField->multiFieldsBefore == 1) && (theField->multiFieldsAfter == 0)))
     {   
      ClearBitString(&hack4,sizeof(struct factConstantPN4Call));

      if (theField->negated) hack4.testForEquality = CLIPS_FALSE;
      else hack4.testForEquality = CLIPS_TRUE;
   
      hack4.whichSlot = theField->slotNumber - 1;
      
      if (theField->multiFieldsBefore == 0)
        {
         hack4.fromBeginning = CLIPS_TRUE;
         hack4.offset = theField->singleFieldsBefore;
        }
      else
        {
         hack4.fromBeginning = CLIPS_FALSE;
         hack4.offset = theField->singleFieldsAfter;
        }

      top = GenConstant(SCALL_PN_CONSTANT4,AddBitMap(&hack4,sizeof(struct factConstantPN4Call)));

      top->argList = GenConstant(theField->type,theField->value);
      
      return(top);
     }
     
   /*===============================================================*/
   /* Otherwise, use the equality or inequality function to compare */
   /* the constant against the value returned by the appropriate    */
   /* pattern network variable retrieval function call.             */
   /*===============================================================*/
   
   else
     {
      if (theField->negated)
        { top = GenConstant(FCALL,PTR_NEQ); }
      else
        { top = GenConstant(FCALL,PTR_EQ); }
        
      tempValue = theField->type;
      theField->type = SF_VARIABLE;
      top->argList = FactGenGetfield(theField);
      theField->type = tempValue;
      
      top->argList->nextArg = GenConstant(theField->type,theField->value);
     }

   /*===============================================================*/
   /* Return the expression for performing the constant comparison. */
   /*===============================================================*/
   
   return(top);
  }
  
/****************************************************/
/* FactGenGetfield: Produces an expression of the format */
/*   (getfield <field>)                     */
/****************************************************/
globle struct expr *FactGenGetfield(theNode)
  struct lhsParseNode *theNode;
  {
   /*===================================================*/
   /* Generate call to retrieve single field slot value */
   /* or the fact relation name.                        */
   /*===================================================*/
   
   if (theNode->withinMultifieldSlot == CLIPS_FALSE)
     { return(GenConstant(FACT_GETVAR_PN2,FactGetVarPN2(theNode))); }
   
   /*=====================================================*/
   /* Generate call to retrieve a value from a multifield */
   /* slot that contains at most one multifield variable. */
   /*=====================================================*/
     
   if (((theNode->type == SF_WILDCARD) || (theNode->type == SF_VARIABLE)) &&
       ((theNode->multiFieldsBefore == 0) ||
        ((theNode->multiFieldsBefore == 1) && (theNode->multiFieldsAfter == 0)))) 
     { return(GenConstant(FACT_GETVAR_PN3,FactGetVarPN3(theNode))); }
   
   if (((theNode->type == MF_WILDCARD) || (theNode->type == MF_VARIABLE)) &&
       (theNode->multiFieldsBefore == 0) && (theNode->multiFieldsAfter == 0))
     { return(GenConstant(FACT_GETVAR_PN3,FactGetVarPN3(theNode))); }
     
   /*=========================================*/
   /* Generate call to retrieve a value using */
   /* the most general retrieval function.    */
   /*=========================================*/
   
   return(GenConstant(FACT_GETVAR_PN1,FactGetVarPN1(theNode)));
  }

/****************************************************/
/* FactGenGetvar:                     */
/****************************************************/
globle struct expr *FactGenGetvar(theNode)
  struct lhsParseNode *theNode;
  {
   /*====================================================*/
   /* Generate call to retrieve single field slot value. */
   /*====================================================*/
   
   if ((theNode->slotNumber > 0) && (theNode->withinMultifieldSlot == CLIPS_FALSE))
     { return(GenConstant(SCALL_GET_VAR2,FactGetVarJN2(theNode))); }
     
   /*=====================================================*/
   /* Generate call to retrieve a value from a multifield */
   /* slot that contains at most one multifield variable. */
   /*=====================================================*/
     
   if (((theNode->type == SF_WILDCARD) || (theNode->type == SF_VARIABLE)) &&
       ((theNode->multiFieldsBefore == 0) ||
        ((theNode->multiFieldsBefore == 1) && (theNode->multiFieldsAfter == 0)))) 
     { return(GenConstant(SCALL_GET_VAR3,FactGetVarJN3(theNode))); }
   
   if (((theNode->type == MF_WILDCARD) || (theNode->type == MF_VARIABLE)) &&
       (theNode->multiFieldsBefore == 0) && 
       (theNode->multiFieldsAfter == 0))
     { return(GenConstant(SCALL_GET_VAR3,FactGetVarJN3(theNode))); }
   
   /*=========================================*/
   /* Generate call to retrieve a value using */
   /* the most general retrieval function.    */
   /*=========================================*/
   
   return(GenConstant(SCALL_GET_VAR1,FactGetVarJN1(theNode)));
  }
  
/****************************************************/
/* FactGenCheckLength:                     */
/****************************************************/
globle struct expr *FactGenCheckLength(theNode)
  struct lhsParseNode *theNode;
  {
   struct factCheckLengthPNCall hack;     
     
   if ((theNode->singleFieldsAfter == 0) && 
       (theNode->type != SF_VARIABLE) && 
       (theNode->type != SF_WILDCARD))
     { return(NULL); }
   
   ClearBitString(&hack,sizeof(struct factCheckLengthPNCall));

   hack.whichSlot = theNode->slotNumber - 1;
   
   if ((theNode->type != MF_VARIABLE) &&
       (theNode->type != MF_WILDCARD) &&
       (theNode->multiFieldsAfter == 0))
     { hack.exactly = 1; }
   else
     { hack.exactly = 0; }
     
   if ((theNode->type == SF_VARIABLE) || (theNode->type == SF_WILDCARD))
     { hack.minLength = 1 + theNode->singleFieldsAfter; }
   else
     { hack.minLength = theNode->singleFieldsAfter; }
   
   return(GenConstant(SCALL_LENGTH_TEST,AddBitMap(&hack,sizeof(struct factCheckLengthPNCall))));
  }

/****************************************************/
/* FactGenCheckZeroLength:                     */
/****************************************************/
globle struct expr *FactGenCheckZeroLength(theSlot)
  int theSlot;
  {
   struct factCheckLengthPNCall hack;     

   ClearBitString(&hack,sizeof(struct factCheckLengthPNCall));
   
   hack.whichSlot = theSlot-1;
   hack.exactly = 1;
   hack.minLength = 0;
   
   return(GenConstant(SCALL_LENGTH_TEST,AddBitMap(&hack,sizeof(struct factCheckLengthPNCall))));
  } 
  
/****************************************************/
/* FactReplaceGetvar:                     */
/****************************************************/
globle VOID FactReplaceGetvar(theItem,theNode)
  struct expr *theItem;
  struct lhsParseNode *theNode;
  {
   if ((theNode->slotNumber > 0) && (theNode->withinMultifieldSlot == CLIPS_FALSE))
     {
      theItem->type = SCALL_GET_VAR2;
      theItem->value = FactGetVarJN2(theNode);
      return;
     }
          
   if (((theNode->type == SF_WILDCARD) || (theNode->type == SF_VARIABLE)) &&
       ((theNode->multiFieldsBefore == 0) ||
        ((theNode->multiFieldsBefore == 1) && (theNode->multiFieldsAfter == 0)))) 
     {
      theItem->type = SCALL_GET_VAR3;
      theItem->value = FactGetVarJN3(theNode);
      return;
     }
   
   if (((theNode->type == MF_WILDCARD) || (theNode->type == MF_VARIABLE)) &&
       (theNode->multiFieldsBefore == 0) && 
       (theNode->multiFieldsAfter == 0))
     { 
      theItem->type = SCALL_GET_VAR3;
      theItem->value = FactGetVarJN3(theNode);
      return;
     }
     
   theItem->type = SCALL_GET_VAR1;
   theItem->value = FactGetVarJN1(theNode);
  }
  
/****************************************************/
/* FactReplaceGetfield:                     */
/****************************************************/
globle VOID FactReplaceGetfield(theItem,theNode)
  struct expr *theItem;
  struct lhsParseNode *theNode;
  {
   if (theNode->withinMultifieldSlot == CLIPS_FALSE)
     {
      theItem->type = FACT_GETVAR_PN2;
      theItem->value = FactGetVarPN2(theNode);
      return;
     }
          
   if (((theNode->type == SF_WILDCARD) || (theNode->type == SF_VARIABLE)) &&
       ((theNode->multiFieldsBefore == 0) ||
        ((theNode->multiFieldsBefore == 1) && (theNode->multiFieldsAfter == 0)))) 
     {
      theItem->type = FACT_GETVAR_PN3;
      theItem->value = FactGetVarPN3(theNode);
      return;
     }
   
   if (((theNode->type == MF_WILDCARD) || (theNode->type == MF_VARIABLE)) &&
       (theNode->multiFieldsBefore == 0) && 
       (theNode->multiFieldsAfter == 0))
     { 
      theItem->type = FACT_GETVAR_PN3;
      theItem->value = FactGetVarPN3(theNode);
      return;
     }
   
   theItem->type = FACT_GETVAR_PN1;
   theItem->value = FactGetVarPN1(theNode);
  }

/****************************************************/
/* FactGetVarJN1:               */
/****************************************************/
static VOID *FactGetVarJN1(theNode)
  struct lhsParseNode *theNode;
  {
   struct factGetVarJN1Call hack;

   ClearBitString(&hack,sizeof(struct factGetVarJN1Call));
   
   /*========================================*/
   /* A slot value of zero indicates that we */
   /* want the pattern address returned.     */
   /*========================================*/
   
   if (theNode->slotNumber <= 0)
     {
      hack.factAddress = 1;
      hack.allFields = 0;
      hack.whichSlot = 0;
      hack.whichField = 0;
     }
     
   /*=====================================================*/
   /* A slot value greater than zero and a field value of */
   /* zero indicate that we want the entire contents of   */
   /* the slot whether it is a single field or multifield */
   /* slot.                                               */
   /*=====================================================*/
   
   else if (theNode->index <= 0)
     {
      hack.factAddress = 0;
      hack.allFields = 1;
      hack.whichSlot = theNode->slotNumber - 1;
      hack.whichField = 0;
     }
     
   /*=====================================================*/
   /* A slot value, m, and a field value, n, both greater */
   /* than zero indicate that we want the nth field of    */
   /* the mth slot.                                       */
   /*=====================================================*/
   
   else
     {
      hack.factAddress = 0;
      hack.allFields = 0;
      hack.whichSlot = theNode->slotNumber - 1;
      hack.whichField = theNode->index - 1;
     }
     
   hack.whichPattern = theNode->pattern - 1;

   return(AddBitMap(&hack,sizeof(struct factGetVarJN1Call)));
  }
  
/*******************************************************************/
/* FactGetVarJN2: Creates the access expression needed to retrieve */
/*  the value from a single field deftemplate slot.                */
/*******************************************************************/
static VOID *FactGetVarJN2(theNode)
  struct lhsParseNode *theNode;
  {
   struct factGetVarJN2Call hack;

   ClearBitString(&hack,sizeof(struct factGetVarJN2Call));
   
   hack.whichSlot = theNode->slotNumber - 1;
   hack.whichPattern = theNode->pattern - 1;

   return(AddBitMap(&hack,sizeof(struct factGetVarJN2Call)));
  }
  
/****************************************************/
/* FactGetVarJN3:               */
/****************************************************/
static VOID *FactGetVarJN3(theNode)
  struct lhsParseNode *theNode;
  {
   struct factGetVarJN3Call hack;
     
   ClearBitString(&hack,sizeof(struct factGetVarJN3Call));
   
   hack.whichSlot = theNode->slotNumber - 1;
   hack.whichPattern = theNode->pattern - 1;
   
   if ((theNode->type == SF_WILDCARD) || (theNode->type == SF_VARIABLE))
     {
      if (theNode->multiFieldsBefore == 0)
        {
         hack.fromBeginning = 1;
         hack.fromEnd = 0;
         hack.beginOffset = theNode->singleFieldsBefore;
         hack.endOffset = 0;
        }
      else
        {
         hack.fromBeginning = 0;
         hack.fromEnd = 1;
         hack.beginOffset = 0;
         hack.endOffset = theNode->singleFieldsAfter;
        }
        
      return(AddBitMap(&hack,sizeof(struct factGetVarJN3Call)));
     }
     
   hack.fromBeginning = 1;
   hack.fromEnd = 1;
   hack.beginOffset = theNode->singleFieldsBefore;
   hack.endOffset = theNode->singleFieldsAfter;
     
   return(AddBitMap(&hack,sizeof(struct factGetVarJN3Call)));
  }
  
/********************************************************/
/* FactGetVarPN1: Creates an expression which retrieves */
/*   a variable from the fact pattern network using the */
/*   most general extraction routine.                   */
/********************************************************/
static VOID *FactGetVarPN1(theNode)
  struct lhsParseNode *theNode;
  {
   struct factGetVarPN1Call hack;

   /*=================================*/
   /* Initialize the argument values. */
   /*=================================*/
   
   ClearBitString(&hack,sizeof(struct factGetVarPN1Call));
   
   /*========================================*/
   /* A slot value of zero indicates that we */
   /* want the pattern address returned.     */
   /*========================================*/
   
   if (theNode->slotNumber <= 0)
     {
      hack.factAddress = 1;
      hack.allFields = 0;
      hack.whichSlot = 0;
      hack.whichField = 0;
     }
     
   /*=====================================================*/
   /* A slot value greater than zero and a field value of */
   /* zero indicate that we want the entire contents of   */
   /* the slot whether it is a single field or multifield */
   /* slot.                                               */
   /*=====================================================*/
   
   else if (theNode->index <= 0)
     {
      hack.factAddress = 0;
      hack.allFields = 1;
      hack.whichSlot = theNode->slotNumber - 1;
      hack.whichField = 0;
     }
     
   /*=====================================================*/
   /* A slot value, m, and a field value, n, both greater */
   /* than zero indicate that we want the nth field of    */
   /* the mth slot.                                       */
   /*=====================================================*/
   
   else
     {
      hack.factAddress = 0;
      hack.allFields = 0;
      hack.whichSlot = theNode->slotNumber - 1;
      hack.whichField = theNode->index - 1;
     }
     
   return(AddBitMap(&hack,sizeof(struct factGetVarPN1Call)));
  }

/*******************************************************************/
/* FactGetVarPN2: Creates the access expression needed to retrieve */
/*  the value from a single field deftemplate slot.                */
/*******************************************************************/
static VOID *FactGetVarPN2(theNode)
  struct lhsParseNode *theNode;
  {
   struct factGetVarPN2Call hack;

   ClearBitString(&hack,sizeof(struct factGetVarPN2Call));
   hack.whichSlot = theNode->slotNumber - 1;

   return(AddBitMap(&hack,sizeof(struct factGetVarPN2Call)));
  }
  
/****************************************************/
/* FactGetVarPN3:               */
/*   Note: this routine is not used for retrieving the value */
/*   of a single field variable found after a multifield     */
/*   variable.       */
/****************************************************/
static VOID *FactGetVarPN3(theNode)
  struct lhsParseNode *theNode;
  {
   struct factGetVarPN3Call hack;
   
   ClearBitString(&hack,sizeof(struct factGetVarPN3Call));
   hack.whichSlot = theNode->slotNumber - 1;
   
   if ((theNode->type == SF_WILDCARD) || (theNode->type == SF_VARIABLE))
     {
      if (theNode->multiFieldsBefore == 0)
        {
         hack.fromBeginning = 1;
         hack.fromEnd = 0;
         hack.beginOffset = theNode->singleFieldsBefore;
         hack.endOffset = 0;
        }
      else
        {
         hack.fromBeginning = 0;
         hack.fromEnd = 1;
         hack.beginOffset = 0;
         hack.endOffset = theNode->singleFieldsAfter;
        }
        
      return(AddBitMap(&hack,sizeof(struct factGetVarPN3Call)));
     }
     
   hack.fromBeginning = 1;
   hack.fromEnd = 1;
   hack.beginOffset = theNode->singleFieldsBefore;
   hack.endOffset = theNode->singleFieldsAfter;
     
   return(AddBitMap(&hack,sizeof(struct factGetVarPN3Call)));
  }
  
/************************************************************/
/* FactPNVariableComparison: Generates the expression used  */
/*   by the pattern network to compare two fields of a fact */
/*   in the same pattern.                                   */
/************************************************************/
globle struct expr *FactPNVariableComparison(selfNode,referringNode)
  struct lhsParseNode *selfNode, *referringNode;
  {
   struct expr *top;
   struct factCompVarsPN1Call hack;
     
   /*================================================================*/
   /* If two single field slots of a deftemplate are being compared, */
   /* then use the following specified variable comparison routine.  */
   /*================================================================*/
   
   ClearBitString(&hack,sizeof(struct factCompVarsPN1Call));
   if ((selfNode->withinMultifieldSlot == CLIPS_FALSE) &&
       (selfNode->slotNumber > 0) &&
       (referringNode->withinMultifieldSlot == CLIPS_FALSE) &&
       (referringNode->slotNumber > 0))
     {
      hack.pass = 0;
      hack.fail = 0;
      hack.field1 = (unsigned int) selfNode->slotNumber - 1;
      hack.field2 = (unsigned int) referringNode->slotNumber - 1;

      if (selfNode->negated) hack.fail = 1;
      else hack.pass = 1; 
      
      top = GenConstant(SCALL_CMP_PN_VARS2,AddBitMap(&hack,sizeof(struct factCompVarsPN1Call)));
     }
     
   /*================================================================*/
   /* Otherwise, use the eq function to compare the values retrieved */
   /* by the appropriate get variable value functions.               */
   /*================================================================*/
   
   else
     {
      if (selfNode->negated) top = GenConstant(FCALL,PTR_NEQ);
      else top = GenConstant(FCALL,PTR_EQ);
      
      top->argList = FactGenGetfield(selfNode);
      top->argList->nextArg = FactGenGetfield(referringNode);
     }

   return(top);
  }
  
/***********************************************************/
/* FactJNVariableComparison: Generates the expression used */
/*   by the join network to compare two fields of a fact   */
/*   in different patterns.                                */
/***********************************************************/
globle struct expr *FactJNVariableComparison(selfNode,referringNode)
  struct lhsParseNode *selfNode, *referringNode;
  {
   struct expr *top;
   struct factCompVarsJN2Call hack2;
   struct factCompVarsJN3Call hack3;

   /*================================================================*/
   /* If two single field slots of a deftemplate are being compared, */
   /* then use the following specified variable comparison routine.  */
   /*================================================================*/
   
   if ((selfNode->withinMultifieldSlot == CLIPS_FALSE) &&
       (selfNode->slotNumber > 0) &&
       (referringNode->withinMultifieldSlot == CLIPS_FALSE) &&
       (referringNode->slotNumber > 0))
     {
      ClearBitString(&hack2,sizeof(struct factCompVarsJN2Call));
      hack2.pass = 0;
      hack2.fail = 0;
      hack2.slot1 = (unsigned int) selfNode->slotNumber - 1;
      hack2.pattern2 = (unsigned int) referringNode->pattern;
      if (referringNode->index < 0) hack2.slot2 = 0;
      else hack2.slot2 = (unsigned int) referringNode->slotNumber - 1;

      if (selfNode->negated) hack2.fail = 1;
      else hack2.pass = 1; 
      
      top = GenConstant(SCALL_CMP_JN_VARS2,AddBitMap(&hack2,sizeof(struct factCompVarsJN2Call)));
     }
     
   /*===============================================================*/
   /* If two single field values are compared and either or both of */
   /* them are contained in multifield slots (and the value can be  */
   /* accessed relative to either the beginning or end of the slot  */
   /* with no intervening multifield variables), then use the       */
   /* following specified variable comparison routine.              */
   /*===============================================================*/
   
   else if ((selfNode->slotNumber > 0) &&
            (selfNode->type == SF_VARIABLE) &&
            ((selfNode->multiFieldsBefore == 0) ||
             ((selfNode->multiFieldsBefore == 1) && 
              (selfNode->multiFieldsAfter == 0))) &&
            (referringNode->slotNumber > 0) &&
            (referringNode->type == SF_VARIABLE) &&
            ((referringNode->multiFieldsBefore == 0) ||
             (referringNode->multiFieldsAfter == 0)))
     {
      ClearBitString(&hack3,sizeof(struct factCompVarsJN3Call));
      hack3.pass = 0;
      hack3.fail = 0;
      hack3.slot1 = (unsigned int) selfNode->slotNumber - 1;
      hack3.pattern2 = (unsigned int) referringNode->pattern;
      hack3.slot2 = (unsigned int) referringNode->slotNumber - 1;
      
      if (selfNode->multiFieldsBefore == 0)
        {
         hack3.fromBeginning1 = 1;
         hack3.offset1 = selfNode->singleFieldsBefore;
        }
      else
        {
         hack3.fromBeginning1 = 0;
         hack3.offset1 = selfNode->singleFieldsAfter;
        }
        
      if (referringNode->multiFieldsBefore == 0)
        {
         hack3.fromBeginning2 = 1;
         hack3.offset2 = referringNode->singleFieldsBefore;
        }
      else
        {
         hack3.fromBeginning2 = 0;
         hack3.offset2 = referringNode->singleFieldsAfter;
        }
        
      if (selfNode->negated) hack3.fail = 1;
      else hack3.pass = 1; 
      
      top = GenConstant(SCALL_CMP_JN_VARS3,AddBitMap(&hack3,sizeof(struct factCompVarsJN3Call)));
     }

   /*===============================================================*/
   /* Otherwise, use the equality or inequality function to compare */
   /* the values returned by the appropriate join network variable  */
   /* retrieval function call.                                      */
   /*===============================================================*/

   else
     {
      if (selfNode->negated)
        { top = GenConstant(FCALL,PTR_NEQ); }
      else
        { top = GenConstant(FCALL,PTR_EQ); }
        
      top->argList = FactGenGetvar(selfNode);
      top->argList->nextArg = FactGenGetvar(referringNode);
     }

   return(top);
  }
  
#endif
  
#endif



