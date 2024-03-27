   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*    INFERENCE ENGINE OBJECT ACCESS ROUTINES MODULE   */
   /*******************************************************/

/**************************************************************/
/* Purpose: RETE Network Interface for Objects                */
/*                                                            */
/* Principal Programmer(s):                                   */
/*      Brian L. Donnell                                      */
/*                                                            */
/* Contributing Programmer(s):                                */
/*                                                            */
/* Revision History:                                          */
/*                                                            */
/**************************************************************/
/* =========================================
   *****************************************
               EXTERNAL DEFINITIONS
   =========================================
   ***************************************** */
#include "setup.h"

#if INSTANCE_PATTERN_MATCHING

#include <stdio.h>
#define _CLIPS_STDIO_

#include "classcom.h"
#include "classfun.h"

#if DEVELOPER
#include "exprnops.h"
#endif
#include "constant.h"
#include "drive.h"
#include "multifld.h"
#include "objrtmch.h"
#include "reteutil.h"
#include "router.h"

#define _OBJRTFNX_SOURCE_
#include "objrtfnx.h"

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */

/* =========================================
   *****************************************
                 MACROS AND TYPES
   =========================================
   ***************************************** */

#define GetInsSlot(ins,si) ins->slotAddresses[ins->cls->slotNameMap[si]-1]

/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER
static VOID PrintObjectGetVarJN1(char *,VOID *);
static BOOLEAN ObjectGetVarJNFunction1(VOID *,DATA_OBJECT *);
static VOID PrintObjectGetVarJN2(char *,VOID *);
static BOOLEAN ObjectGetVarJNFunction2(VOID *,DATA_OBJECT *);
static VOID PrintObjectGetVarPN1(char *,VOID *);
static BOOLEAN ObjectGetVarPNFunction1(VOID *,DATA_OBJECT *);
static VOID PrintObjectGetVarPN2(char *,VOID *);
static BOOLEAN ObjectGetVarPNFunction2(VOID *,DATA_OBJECT *);
static VOID PrintObjectCmpConstant(char *,VOID *);
static VOID PrintSlotLengthTest(char *,VOID *);
static BOOLEAN SlotLengthTestFunction(VOID *,DATA_OBJECT *);
static VOID PrintPNSimpleCompareFunction1(char *,VOID *);
static BOOLEAN PNSimpleCompareFunction1(VOID *,DATA_OBJECT *);
static VOID PrintPNSimpleCompareFunction2(char *,VOID *);
static BOOLEAN PNSimpleCompareFunction2(VOID *,DATA_OBJECT *);
static VOID PrintPNSimpleCompareFunction3(char *,VOID *);
static BOOLEAN PNSimpleCompareFunction3(VOID *,DATA_OBJECT *);
static VOID PrintJNSimpleCompareFunction1(char *,VOID *);
static BOOLEAN JNSimpleCompareFunction1(VOID *,DATA_OBJECT *);
static VOID PrintJNSimpleCompareFunction2(char *,VOID *);
static BOOLEAN JNSimpleCompareFunction2(VOID *,DATA_OBJECT *);
static VOID PrintJNSimpleCompareFunction3(char *,VOID *);
static BOOLEAN JNSimpleCompareFunction3(VOID *,DATA_OBJECT *);
static VOID GetPatternObjectAndMarks(int,INSTANCE_TYPE **,struct multifieldMarker **);
static VOID GetObjectValueGeneral(DATA_OBJECT *,INSTANCE_TYPE *,
                                 struct multifieldMarker *,struct ObjectMatchVar1 *);
static VOID GetObjectValueSimple(DATA_OBJECT *,INSTANCE_TYPE *,struct ObjectMatchVar2 *);
static int CalculateSlotField(struct multifieldMarker *,INSTANCE_SLOT *,int,int *);
static FIELD *GetInsMultiSlotField(INSTANCE_TYPE *,unsigned,unsigned,unsigned);
#else
static VOID PrintObjectGetVarJN1();
static BOOLEAN ObjectGetVarJNFunction1();
static VOID PrintObjectGetVarJN2();
static BOOLEAN ObjectGetVarJNFunction2();
static VOID PrintObjectGetVarPN1();
static BOOLEAN ObjectGetVarPNFunction1();
static VOID PrintObjectGetVarPN2();
static BOOLEAN ObjectGetVarPNFunction2();
static VOID PrintObjectCmpConstant();
static VOID PrintSlotLengthTest();
static BOOLEAN SlotLengthTestFunction();
static VOID PrintPNSimpleCompareFunction1();
static BOOLEAN PNSimpleCompareFunction1();
static VOID PrintPNSimpleCompareFunction2();
static BOOLEAN PNSimpleCompareFunction2();
static VOID PrintPNSimpleCompareFunction3();
static BOOLEAN PNSimpleCompareFunction3();
static VOID PrintJNSimpleCompareFunction1();
static BOOLEAN JNSimpleCompareFunction1();
static VOID PrintJNSimpleCompareFunction2();
static BOOLEAN JNSimpleCompareFunction2();
static VOID PrintJNSimpleCompareFunction3();
static BOOLEAN JNSimpleCompareFunction3();
static VOID GetPatternObjectAndMarks();
static VOID GetObjectValueGeneral();
static VOID GetObjectValueSimple();
static int CalculateSlotField();
static FIELD *GetInsMultiSlotField();
#endif

/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
globle INSTANCE_TYPE *CurrentPatternObject = NULL;
globle INSTANCE_SLOT *CurrentPatternObjectSlot = NULL;
globle int CurrentObjectSlotLength = 1;
globle struct multifieldMarker *CurrentPatternObjectMarks = NULL;
   
/* =========================================
   *****************************************
      INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
   
static struct entityRecord ObjectGVInfo1 = { OBJ_GET_SLOT_JNVAR1,0,1,0,
                                             PrintObjectGetVarJN1,
                                             PrintObjectGetVarJN1,NULL, 
                                             ObjectGetVarJNFunction1,
                                             NULL,NULL,NULL,NULL,NULL,NULL };
                                               
static struct entityRecord ObjectGVInfo2 = { OBJ_GET_SLOT_JNVAR2,0,1,0,
                                             PrintObjectGetVarJN2,
                                             PrintObjectGetVarJN2,NULL, 
                                             ObjectGetVarJNFunction2,
                                             NULL,NULL,NULL,NULL,NULL,NULL };
                                                                                               
static struct entityRecord ObjectGVPNInfo1 = { OBJ_GET_SLOT_PNVAR1,0,1,0,
                                               PrintObjectGetVarPN1,
                                               PrintObjectGetVarPN1,NULL, 
                                               ObjectGetVarPNFunction1,
                                               NULL,NULL,NULL,NULL,NULL,NULL };
                                                  
static struct entityRecord ObjectGVPNInfo2 = { OBJ_GET_SLOT_PNVAR2,0,1,0,
                                               PrintObjectGetVarPN2,
                                               PrintObjectGetVarPN2,NULL, 
                                               ObjectGetVarPNFunction2,
                                               NULL,NULL,NULL,NULL,NULL,NULL };
                                                                                                 
static struct entityRecord ObjectCmpConstantInfo = { OBJ_PN_CONSTANT,0,1,1,
                                                     PrintObjectCmpConstant,
                                                     PrintObjectCmpConstant,NULL, 
                                                     ObjectCmpConstantFunction,
                                                     NULL,NULL,NULL,NULL,NULL,NULL };
                                                                                                 
static struct entityRecord LengthTestInfo = { OBJ_SLOT_LENGTH,0,1,0,
                                              PrintSlotLengthTest,
                                              PrintSlotLengthTest,NULL,
                                              SlotLengthTestFunction,
                                              NULL,NULL,NULL,NULL,NULL,NULL };

static struct entityRecord PNSimpleCompareInfo1 = { OBJ_PN_CMP1,0,1,1,
                                                    PrintPNSimpleCompareFunction1,
                                                    PrintPNSimpleCompareFunction1,NULL,
                                                    PNSimpleCompareFunction1,
                                                    NULL,NULL,NULL,NULL,NULL,NULL };

static struct entityRecord PNSimpleCompareInfo2 = { OBJ_PN_CMP2,0,1,1,
                                                    PrintPNSimpleCompareFunction2,
                                                    PrintPNSimpleCompareFunction2,NULL,
                                                    PNSimpleCompareFunction2,
                                                    NULL,NULL,NULL,NULL,NULL,NULL };

static struct entityRecord PNSimpleCompareInfo3 = { OBJ_PN_CMP3,0,1,1,
                                                    PrintPNSimpleCompareFunction3,
                                                    PrintPNSimpleCompareFunction3,NULL,
                                                    PNSimpleCompareFunction3,
                                                    NULL,NULL,NULL,NULL,NULL,NULL };

static struct entityRecord JNSimpleCompareInfo1 = { OBJ_JN_CMP1,0,1,1,
                                                    PrintJNSimpleCompareFunction1,
                                                    PrintJNSimpleCompareFunction1,NULL,
                                                    JNSimpleCompareFunction1,
                                                    NULL,NULL,NULL,NULL,NULL,NULL };

static struct entityRecord JNSimpleCompareInfo2 = { OBJ_JN_CMP2,0,1,1,
                                                    PrintJNSimpleCompareFunction2,
                                                    PrintJNSimpleCompareFunction2,NULL,
                                                    JNSimpleCompareFunction2,
                                                    NULL,NULL,NULL,NULL,NULL,NULL };

static struct entityRecord JNSimpleCompareInfo3 = { OBJ_JN_CMP3,0,1,1,
                                                    PrintJNSimpleCompareFunction3,
                                                    PrintJNSimpleCompareFunction3,NULL,
                                                    JNSimpleCompareFunction3,
                                                    NULL,NULL,NULL,NULL,NULL,NULL };

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/***************************************************
  NAME         : InstallObjectPrimitives
  DESCRIPTION  : Installs all the entity records
                 associated with object pattern
                 matching operations
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Primitive operations installed
  NOTES        : None
 ***************************************************/
globle VOID InstallObjectPrimitives()
  {
   InstallPrimitive(&ObjectGVInfo1,OBJ_GET_SLOT_JNVAR1);
   InstallPrimitive(&ObjectGVInfo2,OBJ_GET_SLOT_JNVAR2);
   InstallPrimitive(&ObjectGVPNInfo1,OBJ_GET_SLOT_PNVAR1);
   InstallPrimitive(&ObjectGVPNInfo2,OBJ_GET_SLOT_PNVAR2);
   InstallPrimitive(&ObjectCmpConstantInfo,OBJ_PN_CONSTANT);
   InstallPrimitive(&LengthTestInfo,OBJ_SLOT_LENGTH);
   InstallPrimitive(&PNSimpleCompareInfo1,OBJ_PN_CMP1);
   InstallPrimitive(&PNSimpleCompareInfo2,OBJ_PN_CMP2);
   InstallPrimitive(&PNSimpleCompareInfo3,OBJ_PN_CMP3);
   InstallPrimitive(&JNSimpleCompareInfo1,OBJ_JN_CMP1);
   InstallPrimitive(&JNSimpleCompareInfo2,OBJ_JN_CMP2);
   InstallPrimitive(&JNSimpleCompareInfo3,OBJ_JN_CMP3);
  }

/*****************************************************
  NAME         : ObjectCmpConstantFunction
  DESCRIPTION  : Used to compare object slot values
                 against a constant
  INPUTS       : 1) The constant test bitmap
                 2) Data object buffer to hold result
  RETURNS      : CLIPS_TRUE if test successful,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : Buffer set to symbol CLIPS_TRUE if test
                 successful, CLIPS_FALSE otherwise
  NOTES        : Called directly by
                   EvaluatePatternExpression()
 *****************************************************/
globle BOOLEAN ObjectCmpConstantFunction(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   struct ObjectCmpPNConstant *hack;
   DATA_OBJECT theVar;
   EXPRESSION *constantExp;
   int rv;
   SEGMENT *theSegment;
   
   hack = (struct ObjectCmpPNConstant *) ValueToBitMap(theValue);
   if (hack->general)
     {
      EvaluateExpression(GetFirstArgument(),&theVar);
      constantExp = GetFirstArgument()->nextArg;
     }
   else
     {
      constantExp = GetFirstArgument();
      if (CurrentPatternObjectSlot->type == MULTIFIELD)
        {
         theSegment = (struct multifield *) CurrentPatternObjectSlot->value;
         if (hack->fromBeginning)
           {
            theVar.type = theSegment->theFields[hack->offset].type;
            theVar.value = theSegment->theFields[hack->offset].value;
           }
         else
           {
            theVar.type = theSegment->theFields[theSegment->multifieldLength - 
                                      (hack->offset + 1)].type;
            theVar.value = theSegment->theFields[theSegment->multifieldLength - 
                                      (hack->offset + 1)].value;
           }
        }
      else
        {
         theVar.type = (int) CurrentPatternObjectSlot->type;
         theVar.value = CurrentPatternObjectSlot->value;
        }
     }
   if (theVar.type != constantExp->type)
     rv = hack->fail;
   else if (theVar.value != constantExp->value)
     rv = hack->fail;
   else
     rv = hack->pass;
   theResult->type = SYMBOL;
   theResult->value = rv ? CLIPSTrueSymbol : CLIPSFalseSymbol;
   return(rv);
  }
  
/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
static VOID PrintObjectGetVarJN1(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct ObjectMatchVar1 *hack;

   hack = (struct ObjectMatchVar1 *) ValueToBitMap(theValue);
   
   if (hack->objectAddress)
     {
      PrintCLIPS(logicalName,"(obj-ptr ");
      PrintLongInteger(logicalName,(long) hack->whichPattern);
     }
   else if (hack->allFields)
     {
      PrintCLIPS(logicalName,"(obj-slot-contents ");
      PrintLongInteger(logicalName,(long) hack->whichPattern);
      PrintCLIPS(logicalName," ");
      PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->whichSlot)));
     }
   else
     {
      PrintCLIPS(logicalName,"(obj-slot-var ");
      PrintLongInteger(logicalName,(long) hack->whichPattern);
      PrintCLIPS(logicalName," ");
      PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->whichSlot)));
      PrintCLIPS(logicalName," ");
      PrintLongInteger(logicalName,(long) hack->whichField);
     }
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
static BOOLEAN ObjectGetVarJNFunction1(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   struct ObjectMatchVar1 *hack;
   INSTANCE_TYPE *theInstance;
   struct multifieldMarker *theMarks;
   
   hack = (struct ObjectMatchVar1 *) ValueToBitMap(theValue);
   GetPatternObjectAndMarks(((int) hack->whichPattern) - 1,&theInstance,&theMarks);
   GetObjectValueGeneral(theResult,theInstance,theMarks,hack);
   return(CLIPS_TRUE);
  }
  
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
static VOID PrintObjectGetVarJN2(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct ObjectMatchVar2 *hack;
   
   hack = (struct ObjectMatchVar2 *) ValueToBitMap(theValue);
   PrintCLIPS(logicalName,"(obj-slot-quick-var ");
   PrintLongInteger(logicalName,(long) hack->whichPattern);
   PrintCLIPS(logicalName," ");
   PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->whichSlot)));
   if (hack->fromBeginning)
     {
      PrintCLIPS(logicalName," B");
      PrintLongInteger(logicalName,(long) (hack->beginningOffset + 1));
     }
   if (hack->fromEnd)
     {
      PrintCLIPS(logicalName," E");
      PrintLongInteger(logicalName,(long) (hack->endOffset + 1));
     }
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
static BOOLEAN ObjectGetVarJNFunction2(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   struct ObjectMatchVar2 *hack;
   INSTANCE_TYPE *theInstance;
   struct multifieldMarker *theMarks;
   
   hack = (struct ObjectMatchVar2 *) ValueToBitMap(theValue);
   GetPatternObjectAndMarks(((int) hack->whichPattern) - 1,&theInstance,&theMarks);
   GetObjectValueSimple(theResult,theInstance,hack);
   return(CLIPS_TRUE);
  }
  
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
static VOID PrintObjectGetVarPN1(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct ObjectMatchVar1 *hack;

   hack = (struct ObjectMatchVar1 *) ValueToBitMap(theValue);
   
   if (hack->objectAddress)
     PrintCLIPS(logicalName,"(ptn-obj-ptr ");
   else if (hack->allFields)
     {
      PrintCLIPS(logicalName,"(ptn-obj-slot-contents ");
      PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->whichSlot)));
     }
   else
     {
      PrintCLIPS(logicalName,"(ptn-obj-slot-var ");
      PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->whichSlot)));
      PrintCLIPS(logicalName," ");
      PrintLongInteger(logicalName,(long) hack->whichField);
     }
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
static BOOLEAN ObjectGetVarPNFunction1(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   struct ObjectMatchVar1 *hack;
   
   hack = (struct ObjectMatchVar1 *) ValueToBitMap(theValue);
   GetObjectValueGeneral(theResult,CurrentPatternObject,CurrentPatternObjectMarks,hack);
   return(CLIPS_TRUE);
  }
  
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
static VOID PrintObjectGetVarPN2(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct ObjectMatchVar2 *hack;
   
   hack = (struct ObjectMatchVar2 *) ValueToBitMap(theValue);
   PrintCLIPS(logicalName,"(ptn-obj-slot-quick-var ");
   PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->whichSlot)));
   if (hack->fromBeginning)
     {
      PrintCLIPS(logicalName," B");
      PrintLongInteger(logicalName,(long) (hack->beginningOffset + 1));
     }
   if (hack->fromEnd)
     {
      PrintCLIPS(logicalName," E");
      PrintLongInteger(logicalName,(long) (hack->endOffset + 1));
     }
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
static BOOLEAN ObjectGetVarPNFunction2(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   struct ObjectMatchVar2 *hack;
   
   hack = (struct ObjectMatchVar2 *) ValueToBitMap(theValue);
   GetObjectValueSimple(theResult,CurrentPatternObject,hack);
   return(CLIPS_TRUE);
  }
  
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
static VOID PrintObjectCmpConstant(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct ObjectCmpPNConstant *hack;
   
   hack = (struct ObjectCmpPNConstant *) ValueToBitMap(theValue);

   PrintCLIPS(logicalName,"(obj-const ");
   PrintCLIPS(logicalName,hack->pass ? "p " : "n ");
   if (hack->general)
     PrintExpression(logicalName,GetFirstArgument());
   else
     {
      PrintCLIPS(logicalName,hack->fromBeginning ? "B" : "E");
      PrintLongInteger(logicalName,(long) hack->offset);
      PrintCLIPS(logicalName," ");
      PrintExpression(logicalName,GetFirstArgument());
     }
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }

#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
static VOID PrintSlotLengthTest(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct ObjectMatchLength *hack;

   hack = (struct ObjectMatchLength *) ValueToBitMap(theValue);
   
   PrintCLIPS(logicalName,"(obj-slot-len ");
   if (hack->exactly)
     PrintCLIPS(logicalName,"= ");
   else
     PrintCLIPS(logicalName,">= ");
   PrintLongInteger(logicalName,(long) hack->minLength);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
static BOOLEAN SlotLengthTestFunction(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   struct ObjectMatchLength *hack;
   
   theResult->type = SYMBOL;
   theResult->value = CLIPSFalseSymbol;
   hack = (struct ObjectMatchLength *) ValueToBitMap(theValue);
   if (CurrentObjectSlotLength < hack->minLength)
     return(CLIPS_FALSE);
   if (hack->exactly && (CurrentObjectSlotLength > hack->minLength))
     return(CLIPS_FALSE);
   theResult->value = CLIPSTrueSymbol;
   return(CLIPS_TRUE);
  }

#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
static VOID PrintPNSimpleCompareFunction1(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct ObjectCmpPNSingleSlotVars1 *hack;

   hack = (struct ObjectCmpPNSingleSlotVars1 *) ValueToBitMap(theValue);
   
   PrintCLIPS(logicalName,"(pslot-cmp1 ");
   PrintCLIPS(logicalName,hack->pass ? "p " : "n ");
   PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->firstSlot)));
   PrintCLIPS(logicalName," ");
   PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->secondSlot)));
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
static BOOLEAN PNSimpleCompareFunction1(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   struct ObjectCmpPNSingleSlotVars1 *hack;
   INSTANCE_SLOT *is1,*is2;
   int rv;
   
   hack = (struct ObjectCmpPNSingleSlotVars1 *) ValueToBitMap(theValue);
   is1 = GetInsSlot(CurrentPatternObject,hack->firstSlot);
   is2 = GetInsSlot(CurrentPatternObject,hack->secondSlot);
   if (is1->type != is2->type)
     rv = hack->fail;
   else if (is1->value != is2->value)
     rv = hack->fail;
   else
     rv = hack->pass;
   theResult->type = SYMBOL;
   theResult->value = rv ? CLIPSTrueSymbol : CLIPSFalseSymbol;
   return(rv);
  }

#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
static VOID PrintPNSimpleCompareFunction2(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct ObjectCmpPNSingleSlotVars2 *hack;

   hack = (struct ObjectCmpPNSingleSlotVars2 *) ValueToBitMap(theValue);
   
   PrintCLIPS(logicalName,"(pslot-cmp2 ");
   PrintCLIPS(logicalName,hack->pass ? "p " : "n ");
   PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->firstSlot)));
   PrintCLIPS(logicalName,hack->fromBeginning ? " B" : " E");
   PrintLongInteger(logicalName,(long) hack->offset);
   PrintCLIPS(logicalName," ");
   PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->secondSlot)));
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
static BOOLEAN PNSimpleCompareFunction2(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   struct ObjectCmpPNSingleSlotVars2 *hack;
   int rv;
   FIELD *f1;
   INSTANCE_SLOT *is2;
   
   hack = (struct ObjectCmpPNSingleSlotVars2 *) ValueToBitMap(theValue);
   f1 = GetInsMultiSlotField(CurrentPatternObject,(unsigned) hack->firstSlot,
                             (unsigned) hack->fromBeginning,(unsigned) hack->offset);
   is2 = GetInsSlot(CurrentPatternObject,hack->secondSlot);
   if (f1->type != is2->type)
     rv = hack->fail;
   else if (f1->value != is2->value)
     rv = hack->fail;
   else
     rv = hack->pass;
   theResult->type = SYMBOL;
   theResult->value = rv ? CLIPSTrueSymbol : CLIPSFalseSymbol;
   return(rv);
  }

#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
static VOID PrintPNSimpleCompareFunction3(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct ObjectCmpPNSingleSlotVars3 *hack;

   hack = (struct ObjectCmpPNSingleSlotVars3 *) ValueToBitMap(theValue);
   
   PrintCLIPS(logicalName,"(pslot-cmp3 ");
   PrintCLIPS(logicalName,hack->pass ? "p " : "n ");
   PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->firstSlot)));
   PrintCLIPS(logicalName,hack->firstFromBeginning ? " B" : " E");
   PrintLongInteger(logicalName,(long) hack->firstOffset);
   PrintCLIPS(logicalName," ");
   PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->secondSlot)));
   PrintCLIPS(logicalName,hack->secondFromBeginning ? " B" : " E");
   PrintLongInteger(logicalName,(long) hack->secondOffset);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
static BOOLEAN PNSimpleCompareFunction3(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   struct ObjectCmpPNSingleSlotVars3 *hack;
   int rv;
   FIELD *f1,*f2;
   
   hack = (struct ObjectCmpPNSingleSlotVars3 *) ValueToBitMap(theValue);
   f1 = GetInsMultiSlotField(CurrentPatternObject,(unsigned) hack->firstSlot,
                             (unsigned) hack->firstFromBeginning,(unsigned) hack->firstOffset);
   f2 = GetInsMultiSlotField(CurrentPatternObject,(unsigned) hack->secondSlot,
                             (unsigned) hack->secondFromBeginning,(unsigned) hack->secondOffset);
   if (f1->type != f2->type)
     rv = hack->fail;
   else if (f1->value != f2->value)
     rv = hack->fail;
   else
     rv = hack->pass;
   theResult->type = SYMBOL;
   theResult->value = rv ? CLIPSTrueSymbol : CLIPSFalseSymbol;
   return(rv);
  }

#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
static VOID PrintJNSimpleCompareFunction1(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct ObjectCmpJoinSingleSlotVars1 *hack;

   hack = (struct ObjectCmpJoinSingleSlotVars1 *) ValueToBitMap(theValue);
   
   PrintCLIPS(logicalName,"(jslot-cmp1 ");
   PrintCLIPS(logicalName,hack->pass ? "p " : "n ");
   PrintLongInteger(logicalName,(long) hack->firstPattern);
   PrintCLIPS(logicalName," ");
   PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->firstSlot)));
   PrintCLIPS(logicalName," ");
   PrintLongInteger(logicalName,(long) hack->secondPattern);
   PrintCLIPS(logicalName," ");
   PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->secondSlot)));
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
static BOOLEAN JNSimpleCompareFunction1(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   INSTANCE_TYPE *ins1,*ins2;
   struct multifieldMarker *theMarks;
   struct ObjectCmpJoinSingleSlotVars1 *hack;
   int rv;
   INSTANCE_SLOT *is1,*is2;
   
   hack = (struct ObjectCmpJoinSingleSlotVars1 *) ValueToBitMap(theValue);
   GetPatternObjectAndMarks(((int) hack->firstPattern) - 1,&ins1,&theMarks);
   is1 = GetInsSlot(ins1,hack->firstSlot);
   GetPatternObjectAndMarks(((int) hack->secondPattern) - 1,&ins2,&theMarks);
   is2 = GetInsSlot(ins2,hack->secondSlot);
   if (is1->type != is2->type)
     rv = hack->fail;
   if (is1->value != is2->value)
     rv = hack->fail;
   else
     rv = hack->pass;
   theResult->type = SYMBOL;
   theResult->value = rv ? CLIPSTrueSymbol : CLIPSFalseSymbol;
   return(rv);
  }

#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
static VOID PrintJNSimpleCompareFunction2(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct ObjectCmpJoinSingleSlotVars2 *hack;

   hack = (struct ObjectCmpJoinSingleSlotVars2 *) ValueToBitMap(theValue);
   
   PrintCLIPS(logicalName,"(jslot-cmp2 ");
   PrintCLIPS(logicalName,hack->pass ? "p " : "n ");
   PrintLongInteger(logicalName,(long) hack->firstPattern);
   PrintCLIPS(logicalName," ");
   PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->firstSlot)));
   PrintCLIPS(logicalName,hack->fromBeginning ? " B" : " E");
   PrintLongInteger(logicalName,(long) hack->offset);
   PrintCLIPS(logicalName," ");
   PrintLongInteger(logicalName,(long) hack->secondPattern);
   PrintCLIPS(logicalName," ");
   PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->secondSlot)));
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
static BOOLEAN JNSimpleCompareFunction2(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   INSTANCE_TYPE *ins1,*ins2;
   struct multifieldMarker *theMarks;
   struct ObjectCmpJoinSingleSlotVars2 *hack;
   int rv;
   FIELD *f1;
   INSTANCE_SLOT *is2;
   
   hack = (struct ObjectCmpJoinSingleSlotVars2 *) ValueToBitMap(theValue);
   GetPatternObjectAndMarks(((int) hack->firstPattern) - 1,&ins1,&theMarks);
   f1 = GetInsMultiSlotField(ins1,(unsigned) hack->firstSlot,
                             (unsigned) hack->fromBeginning,(unsigned) hack->offset);
   GetPatternObjectAndMarks(((int) hack->secondPattern) - 1,&ins2,&theMarks);
   is2 = GetInsSlot(ins2,hack->secondSlot);
   if (f1->type != is2->type)
     rv = hack->fail;
   else if (f1->value != is2->value)
     rv = hack->fail;
   else
     rv = hack->pass;
   theResult->type = SYMBOL;
   theResult->value = rv ? CLIPSTrueSymbol : CLIPSFalseSymbol;
   return(rv);
  }

#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
static VOID PrintJNSimpleCompareFunction3(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   struct ObjectCmpJoinSingleSlotVars3 *hack;

   hack = (struct ObjectCmpJoinSingleSlotVars3 *) ValueToBitMap(theValue);
   
   PrintCLIPS(logicalName,"(jslot-cmp3 ");
   PrintCLIPS(logicalName,hack->pass ? "p " : "n ");
   PrintLongInteger(logicalName,(long) hack->firstPattern);
   PrintCLIPS(logicalName," ");
   PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->firstSlot)));
   PrintCLIPS(logicalName,hack->firstFromBeginning ? " B" : " E");
   PrintLongInteger(logicalName,(long) hack->firstOffset);
   PrintCLIPS(logicalName," ");
   PrintLongInteger(logicalName,(long) hack->secondPattern);
   PrintCLIPS(logicalName," ");
   PrintCLIPS(logicalName,ValueToString(FindIDSlotName((unsigned) hack->secondSlot)));
   PrintCLIPS(logicalName,hack->secondFromBeginning ? " B" : " E");
   PrintLongInteger(logicalName,(long) hack->secondOffset);
   PrintCLIPS(logicalName,")");
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif
  }
  
static BOOLEAN JNSimpleCompareFunction3(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   INSTANCE_TYPE *ins1,*ins2;
   struct multifieldMarker *theMarks;
   struct ObjectCmpJoinSingleSlotVars3 *hack;
   int rv;
   FIELD *f1,*f2;
   
   hack = (struct ObjectCmpJoinSingleSlotVars3 *) ValueToBitMap(theValue);
   GetPatternObjectAndMarks(((int) hack->firstPattern) - 1,&ins1,&theMarks);
   f1 = GetInsMultiSlotField(ins1,(unsigned) hack->firstSlot,
                             (unsigned) hack->firstFromBeginning,
                             (unsigned) hack->firstOffset);
   GetPatternObjectAndMarks(((int) hack->secondPattern) - 1,&ins2,&theMarks);
   f2 = GetInsMultiSlotField(ins1,(unsigned) hack->secondSlot,
                             (unsigned) hack->secondFromBeginning,
                             (unsigned) hack->secondOffset);
   if (f1->type != f2->type)
     rv = hack->fail;
   else if (f1->value != f2->value)
     rv = hack->fail;
   else
     rv = hack->pass;
   theResult->type = SYMBOL;
   theResult->value = rv ? CLIPSTrueSymbol : CLIPSFalseSymbol;
   return(rv);
  }

/****************************************************
  NAME         : GetPatternObjectAndMarks
  DESCRIPTION  : Finds the instance and multfiield
                 markers corresponding to a specified
                 pattern in the join network
  INPUTS       : 1) The index of the desired pattern
                 2) A buffer to hold the instance
                    address
                 3) A buffer to hold the list of
                    multifield markers
  RETURNS      : Nothing useful
  SIDE EFFECTS : Buffers set 
  NOTES        : None
 ****************************************************/
static VOID GetPatternObjectAndMarks(pattern,theInstance,theMarkers)
  int pattern;
  INSTANCE_TYPE **theInstance;
  struct multifieldMarker **theMarkers;
  {
   if (GlobalRHSBinds == NULL)
     {
      *theInstance = (INSTANCE_TYPE *)
        get_nth_pm_match(GlobalLHSBinds,pattern)->matchingItem;
      *theMarkers = 
        get_nth_pm_match(GlobalLHSBinds,pattern)->markers;
     }
   else if ((GlobalJoin->depth - 1) == pattern)
     {
      *theInstance = (INSTANCE_TYPE *) get_nth_pm_match(GlobalRHSBinds,0)->matchingItem;
      *theMarkers = get_nth_pm_match(GlobalRHSBinds,0)->markers;
     }
   else
     {
      *theInstance = (INSTANCE_TYPE *) 
        get_nth_pm_match(GlobalLHSBinds,pattern)->matchingItem;
      *theMarkers =  
        get_nth_pm_match(GlobalLHSBinds,pattern)->markers;
     }
  }

/***************************************************
  NAME         : GetObjectValueGeneral
  DESCRIPTION  : Access function for getting
                 pattern variable values within the
                 object pattern and join networks
  INPUTS       : 1) The result data object buffer
                 2) The instance to access
                 3) The list of multifield markers
                    for the pattern
                 4) Data for variable reference
  RETURNS      : Nothing useful
  SIDE EFFECTS : Data object is filled with the
                 values of the pattern variable
  NOTES        : None
 ***************************************************/
static VOID GetObjectValueGeneral(result,theInstance,theMarks,matchVar)
  DATA_OBJECT *result;
  INSTANCE_TYPE *theInstance;
  struct multifieldMarker *theMarks;
  struct ObjectMatchVar1 *matchVar;
  {
   int field,extent;
   INSTANCE_SLOT **insSlot,*basisSlot;
   
   if (matchVar->objectAddress)
     {
      result->type = INSTANCE_ADDRESS;
      result->value = (VOID *) theInstance;
      return;
     }
   if (matchVar->whichSlot == ISA_ID)
     {
      result->type = SYMBOL;
      result->value = (VOID *) GetDefclassNamePointer((VOID *) theInstance->cls);
      return;
     }
   if (matchVar->whichSlot == NAME_ID)
     {
      result->type = INSTANCE_NAME;
      result->value = (VOID *) theInstance->name;
      return;
     }
   insSlot = 
     &theInstance->slotAddresses
     [theInstance->cls->slotNameMap[matchVar->whichSlot] - 1];
   
   /* =========================================
      We need to reference the basis slots if
      the slot of this object has changed while
      the RHS was executing
      
      However, if the reference is being done
      by the LHS of a rule (as a consequence of
      an RHS action), give the pattern matcher
      the real value of the slot
      ========================================= */
   if ((theInstance->basisSlots != NULL) && (JoinOperationInProgress == CLIPS_FALSE))
     {
      basisSlot = theInstance->basisSlots + (insSlot - theInstance->slotAddresses);
      if (basisSlot->value != NULL)
        insSlot = &basisSlot;
     }
   
   /* ==================================================
      If we know we are accessing the entire slot,
      the don't bother with searching multifield markers
      or calculating offsets
      ================================================== */
   if (matchVar->allFields)
     {
      result->type = (int) (*insSlot)->type;
      result->value = (*insSlot)->value;
      if (result->type == MULTIFIELD)
        {
         result->begin = 0;
         result->end = GetMFLength((*insSlot)->value) - 1;
        }
      return;
     }
   
   /* =============================================
      Access a general field in a slot pattern with
      two or more multifield variables
      ============================================= */
   field = CalculateSlotField(theMarks,*insSlot,(int) matchVar->whichField,&extent);
   if (extent == -1)
     {
      if ((*insSlot)->desc->multiple)
        {
         result->type = GetMFType((*insSlot)->value,field);
         result->value = GetMFValue((*insSlot)->value,field);
        }
      else
        {
         result->type = (*insSlot)->type;
         result->value = (*insSlot)->value;
        }
     }
   else
     {
      result->type = MULTIFIELD;
      result->value = (*insSlot)->value;
      result->begin = field - 1;
      result->end = field + extent - 2; 
     }
  }
  
/***************************************************
  NAME         : GetObjectValueSimple
  DESCRIPTION  : Access function for getting
                 pattern variable values within the
                 object pattern and join networks
  INPUTS       : 1) The result data object buffer
                 2) The instance to access
                 3) Data for variable reference
  RETURNS      : Nothing useful
  SIDE EFFECTS : Data object is filled with the
                 values of the pattern variable
  NOTES        : None
 ***************************************************/
static VOID GetObjectValueSimple(result,theInstance,matchVar)
  DATA_OBJECT *result;
  INSTANCE_TYPE *theInstance;
  struct ObjectMatchVar2 *matchVar;
  {
   INSTANCE_SLOT **insSlot,*basisSlot;
   SEGMENT *segmentPtr;
   FIELD *fieldPtr;
   
   insSlot = 
     &theInstance->slotAddresses
     [theInstance->cls->slotNameMap[matchVar->whichSlot] - 1];
   
   /* =========================================
      We need to reference the basis slots if
      the slot of this object has changed while
      the RHS was executing
      
      However, if the reference is being done
      by the LHS of a rule (as a consequence of
      an RHS action), give the pattern matcher
      the real value of the slot
      ========================================= */
   if ((theInstance->basisSlots != NULL) && (JoinOperationInProgress == CLIPS_FALSE))
     {
      basisSlot = theInstance->basisSlots + (insSlot - theInstance->slotAddresses);
      if (basisSlot->value != NULL)
        insSlot = &basisSlot;
     }
   
   segmentPtr = (SEGMENT *) (*insSlot)->value;
   if (matchVar->fromBeginning)
     {
      if (matchVar->fromEnd)
        {
         result->type = MULTIFIELD;
         result->value = (VOID *) segmentPtr;
         result->begin = matchVar->beginningOffset;
         result->end = GetMFLength(segmentPtr) - (matchVar->endOffset + 1);
        }
      else
        {
         fieldPtr = &segmentPtr->theFields[matchVar->beginningOffset];
         result->type = fieldPtr->type;
         result->value = fieldPtr->value;
        }
     }
   else
     {
      fieldPtr = &segmentPtr->theFields[segmentPtr->multifieldLength - 
                                        (matchVar->endOffset + 1)];
      result->type = fieldPtr->type;
      result->value = fieldPtr->value;
     }
  }
  
/****************************************************
  NAME         : CalculateSlotField
  DESCRIPTION  : Determines the actual index into the
                 an object slot for a given pattern
                 variable
  INPUTS       : 1) The list of markers to examine
                 2) The instance slot (can be NULL)
                 3) The pattern index of the variable
                 4) A buffer in which to store the
                    extent of the pattern variable
                    (-1 for single-field vars)
  RETURNS      : The actual index
  SIDE EFFECTS : None
  NOTES        : None
 ****************************************************/
static int CalculateSlotField(theMarkers,theSlot,theIndex,extent)
  struct multifieldMarker *theMarkers;
  INSTANCE_SLOT *theSlot;
  int theIndex,*extent;
  {
   register int actualIndex;
   VOID *theSlotName;
   
   actualIndex = theIndex;
   *extent = -1;
   if (theSlot == NULL)
     return(actualIndex);
   theSlotName = (VOID *) theSlot->desc->slotName->name;
   while (theMarkers != NULL)
     {
      if (theMarkers->where.whichSlot == theSlotName)
        break;
      theMarkers = theMarkers->next;
     }
   while ((theMarkers != NULL) ? (theMarkers->where.whichSlot == theSlotName) : CLIPS_FALSE)
     {
      if (theMarkers->whichField == theIndex)
        {
         *extent = theMarkers->endPosition - theMarkers->startPosition + 1;
         return(actualIndex);
        }
      if (theMarkers->whichField > theIndex)
        return(actualIndex);
      actualIndex += theMarkers->endPosition - theMarkers->startPosition;
      theMarkers = theMarkers->next;
     }
   return(actualIndex);
  }

/****************************************************
  NAME         : GetInsMultiSlotField
  DESCRIPTION  : Gets the values of simple single
                 field references in multifield
                 slots for Rete comparisons
  INPUTS       : 1) The instance
                 2) The id of the slot
                 3) A flag indicating if offset is
                    from beginning or end of
                    multifield slot
                 4) The offset
  RETURNS      : The multifield field
  SIDE EFFECTS : None
  NOTES        : Should only be used to access
                 single-field reference in multifield
                 slots for pattern and join network
                 comparisons
 ****************************************************/
static FIELD *GetInsMultiSlotField(theInstance,theSlotID,fromBeginning,offset)
  INSTANCE_TYPE *theInstance;
  unsigned theSlotID,fromBeginning,offset;
  {
   register SEGMENT *theSegment;

   theSegment = (SEGMENT *)
     theInstance->slotAddresses
     [theInstance->cls->slotNameMap[theSlotID] - 1]->value;
   if (fromBeginning)
     return(&theSegment->theFields[offset]);
   return(&theSegment->theFields[theSegment->multifieldLength - offset - 1]);
  }
                                 
#endif

/***************************************************
  NAME         : 
  DESCRIPTION  : 
  INPUTS       : 
  RETURNS      : 
  SIDE EFFECTS : 
  NOTES        : 
 ***************************************************/
