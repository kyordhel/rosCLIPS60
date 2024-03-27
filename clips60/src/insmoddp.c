   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*              CLIPS Version 6.00  05/12/93           */
   /*                                                     */
   /*           INSTANCE MODIFY AND DUPLICATE MODULE      */
   /*******************************************************/

/*************************************************************/
/* Purpose:  Instance modify and duplicate support routines  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/
   
/* =========================================
   *****************************************
               EXTERNAL DEFINITIONS
   =========================================
   ***************************************** */
#include "setup.h"

#if OBJECT_SYSTEM

#if INSTANCE_PATTERN_MATCHING
#include "network.h"
#include "objrtmch.h"
#endif

#include "argacces.h"
#include "clipsmem.h"
#include "extnfunc.h"
#include "insfun.h"
#include "insmngr.h"
#include "inspsr.h"
#include "msgfun.h"
#include "msgpass.h"
#include "router.h"

#define _INSMODDP_SOURCE_
#include "insmoddp.h"

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
   
/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER

static DATA_OBJECT *EvaluateSlotOverrides(EXPRESSION *,int *,int *);
static VOID DeleteSlotOverrideEvaluations(DATA_OBJECT *,int);
static VOID ModifyMsgHandlerSupport(DATA_OBJECT *,int);
static VOID DuplicateMsgHandlerSupport(DATA_OBJECT *,int);

#else

static DATA_OBJECT *EvaluateSlotOverrides();
static VOID DeleteSlotOverrideEvaluations();
static VOID ModifyMsgHandlerSupport();
static VOID DuplicateMsgHandlerSupport();

#endif

/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */

/* =========================================
   *****************************************
      INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
static BOOLEAN ObjectModDupMsgValid = CLIPS_FALSE;

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

#if (! RUN_TIME)

/***************************************************
  NAME         : SetupInstanceModDupCommands
  DESCRIPTION  : Defines function interfaces for
                 modify- and duplicate- instance
                 functions
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Functions defined to CLIPS
  NOTES        : None
 ***************************************************/
globle VOID SetupInstanceModDupCommands()
  {
#if INSTANCE_PATTERN_MATCHING
   DefineFunction2("modify-instance",'u',PTIF InactiveModifyInstance,"InactiveModifyInstance",NULL);
   DefineFunction2("active-modify-instance",'u',PTIF ModifyInstance,"ModifyInstance",NULL);
   AddFunctionParser("active-modify-instance",ParseInitializeInstance);
   DefineFunction2("message-modify-instance",'u',PTIF InactiveMsgModifyInstance,
                   "InactiveMsgModifyInstance",NULL);
   DefineFunction2("active-message-modify-instance",'u',PTIF MsgModifyInstance,
                   "MsgModifyInstance",NULL);
   AddFunctionParser("active-message-modify-instance",ParseInitializeInstance);
   
   DefineFunction2("duplicate-instance",'u',
                    PTIF InactiveDuplicateInstance,"InactiveDuplicateInstance",NULL);
   DefineFunction2("active-duplicate-instance",'u',PTIF DuplicateInstance,"DuplicateInstance",NULL);
   AddFunctionParser("active-duplicate-instance",ParseInitializeInstance);
   DefineFunction2("message-duplicate-instance",'u',PTIF InactiveMsgDuplicateInstance,
                   "InactiveMsgDuplicateInstance",NULL);
   DefineFunction2("active-message-duplicate-instance",'u',PTIF MsgDuplicateInstance,
                   "MsgDuplicateInstance",NULL);
   AddFunctionParser("active-message-duplicate-instance",ParseInitializeInstance);
#else
   DefineFunction2("modify-instance",'u',PTIF ModifyInstance,"ModifyInstance",NULL);
   DefineFunction2("message-modify-instance",'u',PTIF MsgModifyInstance,
                   "MsgModifyInstance",NULL);
   DefineFunction2("duplicate-instance",'u',PTIF DuplicateInstance,"DuplicateInstance",NULL);
   DefineFunction2("message-duplicate-instance",'u',PTIF MsgDuplicateInstance,
                   "MsgDuplicateInstance",NULL);
#endif

   DefineFunction2("(direct-modify)",'u',PTIF DirectModifyMsgHandler,"DirectModifyMsgHandler",NULL);
   DefineFunction2("(message-modify)",'u',PTIF MsgModifyMsgHandler,"MsgModifyMsgHandler",NULL);
   DefineFunction2("(direct-duplicate)",'u',PTIF DirectDuplicateMsgHandler,"DirectDuplicateMsgHandler",NULL);
   DefineFunction2("(message-duplicate)",'u',PTIF MsgDuplicateMsgHandler,"MsgDuplicateMsgHandler",NULL);

   AddFunctionParser("modify-instance",ParseInitializeInstance);
   AddFunctionParser("message-modify-instance",ParseInitializeInstance);
   AddFunctionParser("duplicate-instance",ParseInitializeInstance);
   AddFunctionParser("message-duplicate-instance",ParseInitializeInstance);
  }

#endif

/*************************************************************
  NAME         : ModifyInstance
  DESCRIPTION  : Modifies slots of an instance via the
                 direct-modify message
  INPUTS       : The address of the result value
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slot updates performed directly
  NOTES        : CLIPS Syntax:
                 (modify-instance <instance> <slot-override>*)
 *************************************************************/
globle VOID ModifyInstance(result)
  DATA_OBJECT *result;
  {
   INSTANCE_TYPE *ins;
   EXPRESSION exp;
   DATA_OBJECT *overrides;
   int oldOMDMV,overrideCount,error;
   
   /* ===========================================
      The slot-overrides need to be evaluated now
      to resolve any variable references before a
      new frame is pushed for message-handler
      execution
      =========================================== */

   overrides = EvaluateSlotOverrides(GetFirstArgument()->nextArg,
                                     &overrideCount,&error);
   if (error)
     {
      SetpType(result,SYMBOL);
      SetpValue(result,CLIPSFalseSymbol);
      return;
     }
     
   /* ==================================
      Find the instance and make sure it
      wasn't deleted by the overrides
      ================================== */
   ins = CheckInstance(ValueToString(ExpressionFunctionCallName(CurrentExpression)));
   if (ins == NULL)
     {
      SetpType(result,SYMBOL);
      SetpValue(result,CLIPSFalseSymbol);
      DeleteSlotOverrideEvaluations(overrides,overrideCount);
      return;
     }
     
   /* ======================================
      We are passing the slot override
      expression information along
      to whatever message-handler implements
      the modify
      ====================================== */
   exp.type = EXTERNAL_ADDRESS;
   exp.value = (VOID *) overrides;
   exp.argList = NULL;
   exp.nextArg = NULL;
   
   oldOMDMV = ObjectModDupMsgValid;
   ObjectModDupMsgValid = CLIPS_TRUE;
   DirectMessage(FindSymbol(DIRECT_MODIFY_STRING),ins,result,&exp);
   ObjectModDupMsgValid = oldOMDMV;
   
   DeleteSlotOverrideEvaluations(overrides,overrideCount);
  }
  
/*************************************************************
  NAME         : MsgModifyInstance
  DESCRIPTION  : Modifies slots of an instance via the
                 direct-modify message
  INPUTS       : The address of the result value
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slot updates performed with put- messages
  NOTES        : CLIPS Syntax:
                 (message-modify-instance <instance>
                    <slot-override>*)
 *************************************************************/
globle VOID MsgModifyInstance(result)
  DATA_OBJECT *result;
  {
   INSTANCE_TYPE *ins;
   EXPRESSION exp;
   DATA_OBJECT *overrides;
   int oldOMDMV,overrideCount,error;
   
   /* ===========================================
      The slot-overrides need to be evaluated now
      to resolve any variable references before a
      new frame is pushed for message-handler
      execution
      =========================================== */
   overrides = EvaluateSlotOverrides(GetFirstArgument()->nextArg,
                                     &overrideCount,&error);
   if (error)
     {
      SetpType(result,SYMBOL);
      SetpValue(result,CLIPSFalseSymbol);
      return;
     }
     
   /* ==================================
      Find the instance and make sure it
      wasn't deleted by the overrides
      ================================== */
   ins = CheckInstance(ValueToString(ExpressionFunctionCallName(CurrentExpression)));
   if (ins == NULL)
     {
      SetpType(result,SYMBOL);
      SetpValue(result,CLIPSFalseSymbol);
      DeleteSlotOverrideEvaluations(overrides,overrideCount);
      return;
     }
     
   /* ======================================
      We are passing the slot override
      expression information along
      to whatever message-handler implements
      the modify
      ====================================== */
   exp.type = EXTERNAL_ADDRESS;
   exp.value = (VOID *) overrides;
   exp.argList = NULL;
   exp.nextArg = NULL;
   
   oldOMDMV = ObjectModDupMsgValid;
   ObjectModDupMsgValid = CLIPS_TRUE;
   DirectMessage(FindSymbol(MSG_MODIFY_STRING),ins,result,&exp);
   ObjectModDupMsgValid = oldOMDMV;
   
   DeleteSlotOverrideEvaluations(overrides,overrideCount);
  }
  
/*************************************************************
  NAME         : DuplicateInstance
  DESCRIPTION  : Duplicates an instance via the
                 direct-duplicate message
  INPUTS       : The address of the result value
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slot updates performed directly
  NOTES        : CLIPS Syntax:
                 (duplicate-instance <instance>
                   [to <instance-name>] <slot-override>*)
 *************************************************************/
globle VOID DuplicateInstance(result)
  DATA_OBJECT *result;
  {
   INSTANCE_TYPE *ins;
   DATA_OBJECT newName;
   EXPRESSION exp[2];
   DATA_OBJECT *overrides;
   int oldOMDMV,overrideCount,error;
   
   /* ===========================================
      The slot-overrides need to be evaluated now
      to resolve any variable references before a
      new frame is pushed for message-handler
      execution
      =========================================== */
   overrides = EvaluateSlotOverrides(GetFirstArgument()->nextArg->nextArg,
                                     &overrideCount,&error);
   if (error)
     {
      SetpType(result,SYMBOL);
      SetpValue(result,CLIPSFalseSymbol);
      return;
     }
   
   /* ==================================
      Find the instance and make sure it
      wasn't deleted by the overrides
      ================================== */
   ins = CheckInstance(ValueToString(ExpressionFunctionCallName(CurrentExpression)));
   if (ins == NULL)
     {
      SetpType(result,SYMBOL);
      SetpValue(result,CLIPSFalseSymbol);
      DeleteSlotOverrideEvaluations(overrides,overrideCount);
      return;
     }
   if (ArgTypeCheck(ValueToString(ExpressionFunctionCallName(CurrentExpression)),
                    2,INSTANCE_NAME,&newName) == CLIPS_FALSE)
     {
      SetpType(result,SYMBOL);
      SetpValue(result,CLIPSFalseSymbol);
      DeleteSlotOverrideEvaluations(overrides,overrideCount);
      return;
     }
     
   /* ======================================
      We are passing the slot override
      expression information along
      to whatever message-handler implements
      the duplicate
      ====================================== */
   exp[0].type = INSTANCE_NAME;
   exp[0].value = newName.value;
   exp[0].argList = NULL;
   exp[0].nextArg = &exp[1];
   exp[1].type = EXTERNAL_ADDRESS;
   exp[1].value = (VOID *) overrides;
   exp[1].argList = NULL;
   exp[1].nextArg = NULL;
   
   oldOMDMV = ObjectModDupMsgValid;
   ObjectModDupMsgValid = CLIPS_TRUE;
   DirectMessage(FindSymbol(DIRECT_DUPLICATE_STRING),ins,result,&exp[0]);
   ObjectModDupMsgValid = oldOMDMV;
   
   DeleteSlotOverrideEvaluations(overrides,overrideCount);
  }
  
/*************************************************************
  NAME         : MsgDuplicateInstance
  DESCRIPTION  : Duplicates an instance via the
                 message-duplicate message
  INPUTS       : The address of the result value
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slot updates performed w/ int & put- messages
  NOTES        : CLIPS Syntax:
                 (duplicate-instance <instance>
                   [to <instance-name>] <slot-override>*)
 *************************************************************/
globle VOID MsgDuplicateInstance(result)
  DATA_OBJECT *result;
  {
   INSTANCE_TYPE *ins;
   DATA_OBJECT newName;
   EXPRESSION exp[2];
   DATA_OBJECT *overrides;
   int oldOMDMV,overrideCount,error;
   
   /* ===========================================
      The slot-overrides need to be evaluated now
      to resolve any variable references before a
      new frame is pushed for message-handler
      execution
      =========================================== */
   overrides = EvaluateSlotOverrides(GetFirstArgument()->nextArg->nextArg,
                                     &overrideCount,&error);
   if (error)
     {
      SetpType(result,SYMBOL);
      SetpValue(result,CLIPSFalseSymbol);
      return;
     }
     
   /* ==================================
      Find the instance and make sure it
      wasn't deleted by the overrides
      ================================== */
   ins = CheckInstance(ValueToString(ExpressionFunctionCallName(CurrentExpression)));
   if (ins == NULL)
     {
      SetpType(result,SYMBOL);
      SetpValue(result,CLIPSFalseSymbol);
      DeleteSlotOverrideEvaluations(overrides,overrideCount);
      return;
     }
   if (ArgTypeCheck(ValueToString(ExpressionFunctionCallName(CurrentExpression)),
                    2,INSTANCE_NAME,&newName) == CLIPS_FALSE)
     {
      SetpType(result,SYMBOL);
      SetpValue(result,CLIPSFalseSymbol);
      DeleteSlotOverrideEvaluations(overrides,overrideCount);
      return;
     }

   /* ======================================
      We are passing the slot override
      expression information along
      to whatever message-handler implements
      the duplicate
      ====================================== */
   exp[0].type = INSTANCE_NAME;
   exp[0].value = newName.value;
   exp[0].argList = NULL;
   exp[0].nextArg = &exp[1];
   exp[1].type = EXTERNAL_ADDRESS;
   exp[1].value = (VOID *) overrides;
   exp[1].argList = NULL;
   exp[1].nextArg = NULL;
   
   oldOMDMV = ObjectModDupMsgValid;
   ObjectModDupMsgValid = CLIPS_TRUE;
   DirectMessage(FindSymbol(MSG_DUPLICATE_STRING),ins,result,&exp[0]);
   ObjectModDupMsgValid = oldOMDMV;
   
   DeleteSlotOverrideEvaluations(overrides,overrideCount);
  }

#if INSTANCE_PATTERN_MATCHING

/**************************************************************
  NAME         : InactiveModifyInstance
  DESCRIPTION  : Modifies slots of an instance of a class
                 Pattern-matching is automatically
                 delayed until the slot updates are done
  INPUTS       : The address of the result value
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slot updates performed directly
  NOTES        : CLIPS Syntax:
                 (modify-instance <instance-name>
                   <slot-override>*)
 **************************************************************/
globle VOID InactiveModifyInstance(result)
  DATA_OBJECT *result;
  {
   int ov;
   
   ov = SetDelayObjectPatternMatching(CLIPS_TRUE);
   ModifyInstance(result);
   SetDelayObjectPatternMatching(ov);
  }
  
/**************************************************************
  NAME         : InactiveMsgModifyInstance
  DESCRIPTION  : Modifies slots of an instance of a class
                 Pattern-matching is automatically
                 delayed until the slot updates are done
  INPUTS       : The address of the result value
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slot updates performed with put- messages
  NOTES        : CLIPS Syntax:
                 (message-modify-instance <instance-name>
                   <slot-override>*)
 **************************************************************/
globle VOID InactiveMsgModifyInstance(result)
  DATA_OBJECT *result;
  {
   int ov;
   
   ov = SetDelayObjectPatternMatching(CLIPS_TRUE);
   MsgModifyInstance(result);
   SetDelayObjectPatternMatching(ov);
  }
  
/*******************************************************************
  NAME         : InactiveDuplicateInstance
  DESCRIPTION  : Duplicates an instance of a class
                 Pattern-matching is automatically
                 delayed until the slot updates are done
  INPUTS       : The address of the result value
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slot updates performed directly
  NOTES        : CLIPS Syntax:
                 (duplicate-instance <instance> [to <instance-name>]
                   <slot-override>*)
 *******************************************************************/
globle VOID InactiveDuplicateInstance(result)
  DATA_OBJECT *result;
  {
   int ov;
   
   ov = SetDelayObjectPatternMatching(CLIPS_TRUE);
   DuplicateInstance(result);
   SetDelayObjectPatternMatching(ov);
  }
  
/**************************************************************
  NAME         : InactiveMsgDuplicateInstance
  DESCRIPTION  : Duplicates an instance of a class
                 Pattern-matching is automatically
                 delayed until the slot updates are done
  INPUTS       : The address of the result value
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slot updates performed with put- messages
  NOTES        : CLIPS Syntax:
                 (message-duplicate-instance <instance>
                   [to <instance-name>]
                   <slot-override>*)
 **************************************************************/
globle VOID InactiveMsgDuplicateInstance(result)
  DATA_OBJECT *result;
  {
   int ov;
   
   ov = SetDelayObjectPatternMatching(CLIPS_TRUE);
   MsgDuplicateInstance(result);
   SetDelayObjectPatternMatching(ov);
  }
  
#endif
  
/*****************************************************
  NAME         : DirectDuplicateMsgHandler
  DESCRIPTION  : Implementation for the USER class
                 handler direct-duplicate
                 
                 Implements duplicate-instance message
                 with a series of direct slot
                 placements
  INPUTS       : A data object buffer to hold the
                 result
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slot values updated
  NOTES        : None
 *****************************************************/
globle VOID DirectDuplicateMsgHandler(result)
  DATA_OBJECT *result;
  {
   DuplicateMsgHandlerSupport(result,CLIPS_FALSE);
  }
  
/*****************************************************
  NAME         : MsgDuplicateMsgHandler
  DESCRIPTION  : Implementation for the USER class
                 handler message-duplicate
                 
                 Implements duplicate-instance message
                 with a series of put- messages
  INPUTS       : A data object buffer to hold the
                 result
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slot values updated
  NOTES        : None
 *****************************************************/
globle VOID MsgDuplicateMsgHandler(result)
  DATA_OBJECT *result;
  {
   DuplicateMsgHandlerSupport(result,CLIPS_TRUE);
  }

/***************************************************
  NAME         : DirectModifyMsgHandler
  DESCRIPTION  : Implementation for the USER class
                 handler direct-modify
                 
                 Implements modify-instance message
                 with a series of direct slot
                 placements
  INPUTS       : A data object buffer to hold the
                 result
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slot values updated
  NOTES        : None
 ***************************************************/
globle VOID DirectModifyMsgHandler(result)
  DATA_OBJECT *result;
  {
   ModifyMsgHandlerSupport(result,CLIPS_FALSE);
  }
  
/***************************************************
  NAME         : MsgModifyMsgHandler
  DESCRIPTION  : Implementation for the USER class
                 handler message-modify
                 
                 Implements modify-instance message
                 with a series of put- messages
  INPUTS       : A data object buffer to hold the
                 result
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slot values updated
  NOTES        : None
 ***************************************************/
globle VOID MsgModifyMsgHandler(result)
  DATA_OBJECT *result;
  {
   ModifyMsgHandlerSupport(result,CLIPS_TRUE);
  }

/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/***********************************************************
  NAME         : EvaluateSlotOverrides
  DESCRIPTION  : Evaluates the slot-override expressions
                 for modify-instance and duplicate-instance
                 Evaluations are stored in an array of
                 data objects, where the supplementalInfo
                 field points at the name of the slot
                 The data object next fields are used
                 to link the array as well.
  INPUTS       : 1) The slot override expressions
                 2) A buffer to hold the number
                    of slot overrides
                 3) A buffer to hold an error flag
  RETURNS      : The slot override data object array
  SIDE EFFECTS : Data object array allocated and initialized
                 override count and error buffers set
  NOTES        : Slot overrides must be evaluated before
                 calling supporting message-handlers for
                 modify- and duplicate-instance in the
                 event that the overrides contain variable
                 references to an outer frame
 ***********************************************************/
static DATA_OBJECT *EvaluateSlotOverrides(ovExprs,ovCnt,error)
  EXPRESSION *ovExprs;
  int *ovCnt,*error;
  {
   DATA_OBJECT *ovs;
   int ovi;
   VOID *slotName;
   
   *error = CLIPS_FALSE;

   /* ==========================================
      There are two expressions chains for every
      slot override: one for the slot name and
      one for the slot value
      ========================================== */
   *ovCnt = CountArguments(ovExprs) / 2;
   if (*ovCnt == 0)
     return(NULL);
   
   /* ===============================================
      Evaluate all the slot override names and values
      and store them in a contiguous array
      =============================================== */
   ovs = (DATA_OBJECT *) gm2((int) (sizeof(DATA_OBJECT) * (*ovCnt)));
   ovi = 0;
   while (ovExprs != NULL)
     {
      if (EvaluateExpression(ovExprs,&ovs[ovi]))
        goto EvaluateOverridesError;
      if (ovs[ovi].type != SYMBOL)
        {
         ExpectedTypeError1(ValueToString(ExpressionFunctionCallName(CurrentExpression)),
                            ovi+1,"slot name");
         SetEvaluationError(CLIPS_TRUE);
         goto EvaluateOverridesError;
        }
      slotName = ovs[ovi].value;
      if (EvaluateAndStoreInDataObject(CLIPS_FALSE,ovExprs->nextArg->argList,
                                            &ovs[ovi]) == CLIPS_FALSE)
        goto EvaluateOverridesError;
      ovs[ovi].supplementalInfo = slotName;
      ovExprs = ovExprs->nextArg->nextArg;
      ovs[ovi].next = (ovExprs != NULL) ? &ovs[ovi+1] : NULL;
      ovi++;
     }
   return(ovs);
   
EvaluateOverridesError:
   rm((VOID *) ovs,(int) (sizeof(DATA_OBJECT) * (*ovCnt)));
   *error = CLIPS_TRUE;
   return(NULL);
  }

/**********************************************************
  NAME         : DeleteSlotOverrideEvaluations
  DESCRIPTION  : Deallocates slot override evaluation array
  INPUTS       : 1) The data object array
                 2) The number of elements
  RETURNS      : Nothing useful
  SIDE EFFECTS : Deallocates slot override data object
                 array for modify- and duplicate- instance
  NOTES        : None
 **********************************************************/
static VOID DeleteSlotOverrideEvaluations(ovEvals,ovCnt)
  DATA_OBJECT *ovEvals;
  int ovCnt;
  {
   if (ovEvals != NULL)
     rm((VOID *) ovEvals,(int) (sizeof(DATA_OBJECT) * ovCnt));
  }
  
/**********************************************************
  NAME         : ModifyMsgHandlerSupport
  DESCRIPTION  : Support routine for DirectModifyMsgHandler
                 and MsgModifyMsgHandler
                 
                 Performs a series of slot updates
                 directly or with messages
  INPUTS       : 1) A data object buffer to hold the result
                 2) A flag indicating whether to use
                    put- messages or direct placement
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slots updated (messages sent)
  NOTES        : None
 **********************************************************/
static VOID ModifyMsgHandlerSupport(result,msgpass)
  DATA_OBJECT *result;
  int msgpass;
  {
   DATA_OBJECT *slotOverrides,*newval,temp;
   EXPRESSION msgExp;
   INSTANCE_TYPE *ins;
   INSTANCE_SLOT *insSlot;
   
   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   if (ObjectModDupMsgValid == CLIPS_FALSE)
     {
      PrintErrorID("INSMODDP",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Direct/message-modify message valid only in modify-instance.\n");
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
   ObjectModDupMsgValid = CLIPS_FALSE;
   
   ins = GetActiveInstance();
   if (ins->garbage)
     {
      StaleInstanceAddress("modify-instance");
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
   
   /* =======================================
      Retrieve the slot override data objects
      passed from ModifyInstance - the slot
      name is stored in the supplementalInfo
      field - and the next fields are links
      ======================================= */
   slotOverrides = (DATA_OBJECT *) GetNthMessageArgument(1)->value;
   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;

   while (slotOverrides != NULL)
     {
      /* ===========================================================
         No evaluation or error checking needs to be done
         since this has already been done by EvaluateSlotOverrides()
         =========================================================== */
      insSlot = FindInstanceSlot(ins,(SYMBOL_HN *) slotOverrides->supplementalInfo);
      if (insSlot == NULL)
        {
         SlotExistError(ValueToString(slotOverrides->supplementalInfo),"modify-instance");
         SetEvaluationError(CLIPS_TRUE);
         return;
        }
      if (msgpass)
        {
         msgExp.type = slotOverrides->type;
         if (msgExp.type != MULTIFIELD)
           msgExp.value = slotOverrides->value;
         else
           msgExp.value = (VOID *) slotOverrides;
         msgExp.argList = NULL;
         msgExp.nextArg = NULL;
         DirectMessage(insSlot->desc->overrideMessage,ins,&temp,&msgExp);
         if (EvaluationError ||
             ((temp.type == SYMBOL) && (temp.value == CLIPSFalseSymbol)))
           return;
        }
      else
        {
         if (insSlot->desc->multiple && (slotOverrides->type != MULTIFIELD))
           {
            temp.type = MULTIFIELD;
            temp.value = CreateMultifield(1);
            SetDOBegin(temp,1);
            SetDOEnd(temp,1);
            SetMFType(temp.value,1,slotOverrides->type);
            SetMFValue(temp.value,1,slotOverrides->value);
            newval = &temp;
           }
         else
           newval = slotOverrides;
         if (PutSlotValue(ins,insSlot,newval,"modify-instance") == CLIPS_FALSE)
           return;
        }
        
      slotOverrides = slotOverrides->next;
     }
   result->value = CLIPSTrueSymbol;
  }
    
/*************************************************************
  NAME         : DuplicateMsgHandlerSupport
  DESCRIPTION  : Support routine for DirectDuplicateMsgHandler
                 and MsgDuplicateMsgHandler
                 
                 Performs a series of slot updates
                 directly or with messages
  INPUTS       : 1) A data object buffer to hold the result
                 2) A flag indicating whether to use
                    put- messages or direct placement
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slots updated (messages sent)
  NOTES        : None
 *************************************************************/
static VOID DuplicateMsgHandlerSupport(result,msgpass)
  DATA_OBJECT *result;
  int msgpass;
  {
   INSTANCE_TYPE *srcins,*dstins;
   SYMBOL_HN *newName;
   DATA_OBJECT *slotOverrides;
   EXPRESSION *valArg,msgExp;
   int i,oldMkInsMsgPass;
   INSTANCE_SLOT *dstInsSlot;
   DATA_OBJECT temp,*newval;
   
   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   if (ObjectModDupMsgValid == CLIPS_FALSE)
     {
      PrintErrorID("INSMODDP",2,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Direct/message-duplicate message valid only in duplicate-instance.\n");
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
   ObjectModDupMsgValid = CLIPS_FALSE;
   
   /* ==================================
      Grab the slot override expressions
      and determine the source instance
      and the name of the new instance
      ================================== */
   srcins = GetActiveInstance();
   newName = (SYMBOL_HN *) GetNthMessageArgument(1)->value;
   slotOverrides = (DATA_OBJECT *) GetNthMessageArgument(2)->value;
   if (srcins->garbage)
     {
      StaleInstanceAddress("duplicate-instance");
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
   if (newName == srcins->name)
     {
      PrintErrorID("INSMODDP",3,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Instance copy must have a different name in duplicate-instance.\n");
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
     
   /* ==========================================
      Create an uninitialized new instance of
      the new name (delete old version - if any)
      ========================================== */
   oldMkInsMsgPass = MkInsMsgPass;
   MkInsMsgPass = msgpass;
   dstins = BuildInstance(newName,srcins->cls,CLIPS_TRUE);
   MkInsMsgPass = oldMkInsMsgPass;
   if (dstins == NULL)
     return;
   dstins->busy++;
   
   /* ================================
      Place slot overrides directly or
      with put- messages
      ================================ */ 
   while (slotOverrides != NULL)
     {
      /* ===========================================================
         No evaluation or error checking needs to be done
         since this has already been done by EvaluateSlotOverrides()
         =========================================================== */
      dstInsSlot = FindInstanceSlot(dstins,(SYMBOL_HN *) slotOverrides->supplementalInfo);
      if (dstInsSlot == NULL)
        {
         SlotExistError(ValueToString(temp.value),"duplicate-instance");
         goto DuplicateError;
        }
      if (msgpass)
        {
         msgExp.type = slotOverrides->type;
         if (msgExp.type != MULTIFIELD)
           msgExp.value = slotOverrides->value;
         else
           msgExp.value = (VOID *) slotOverrides;
         msgExp.argList = NULL;
         msgExp.nextArg = NULL;
         DirectMessage(dstInsSlot->desc->overrideMessage,dstins,&temp,&msgExp);
         if (EvaluationError ||
             ((temp.type == SYMBOL) && (temp.value == CLIPSFalseSymbol)))
           goto DuplicateError;
        }
      else
        {
         if (dstInsSlot->desc->multiple && (slotOverrides->type != MULTIFIELD))
           {
            temp.type = MULTIFIELD;
            temp.value = CreateMultifield(1);
            SetDOBegin(temp,1);
            SetDOEnd(temp,1);
            SetMFType(temp.value,1,slotOverrides->type);
            SetMFValue(temp.value,1,slotOverrides->value);
            newval = &temp;
           }
         else
           newval = slotOverrides;
         if (PutSlotValue(dstins,dstInsSlot,newval,"duplicate-instance") == CLIPS_FALSE)
           goto DuplicateError;
        }
      dstInsSlot->override = CLIPS_TRUE;
      slotOverrides = slotOverrides->next;
     }
   
   /* =======================================
      Copy values from source instance to new
      directly or with put- messages
      ======================================= */
   for (i = 0 ; i < dstins->cls->localInstanceSlotCount ; i++)
     {
      if (dstins->slots[i].override == CLIPS_FALSE)
        {
         if (msgpass)
           {
            temp.type = srcins->slots[i].type;
            temp.value = srcins->slots[i].value;
            if (temp.type == MULTIFIELD)
              {
               SetDOBegin(temp,1);
               SetDOEnd(temp,GetMFLength(temp.value));
              }
            valArg = ConvertValueToExpression(&temp);
            DirectMessage(dstins->slots[i].desc->overrideMessage,
                          dstins,&temp,valArg);
            ReturnExpression(valArg);
            if (EvaluationError ||
                ((temp.type == SYMBOL) && (temp.value == CLIPSFalseSymbol)))
              goto DuplicateError;
           }
         else
           {
            temp.type = srcins->slots[i].type;
            temp.value = srcins->slots[i].value;
            if (srcins->slots[i].type == MULTIFIELD)
              {
               SetDOBegin(temp,1);
               SetDOEnd(temp,GetMFLength(srcins->slots[i].value));
              }
            if (PutSlotValue(dstins,&dstins->slots[i],&temp,"duplicate-instance")
                 == CLIPS_FALSE)
              goto DuplicateError;
           }
        }
     }
      
   /* =======================================
      Send init message for message-duplicate
      ======================================= */
   if (msgpass)
     {
      for (i = 0 ; i < dstins->cls->instanceSlotCount ; i++)
        dstins->slotAddresses[i]->override = CLIPS_TRUE;
      dstins->initializeInProgress = 1;
      oldMkInsMsgPass = WithinInit;
      WithinInit = CLIPS_TRUE;
      DirectMessage(INIT_SYMBOL,dstins,result,NULL);
      WithinInit = oldMkInsMsgPass;
     }
   dstins->busy--;
   if (dstins->garbage)
     {
      result->type = SYMBOL;
      result->value = CLIPSFalseSymbol;
      SetEvaluationError(CLIPS_TRUE);
     }
   else
     {
      result->type = INSTANCE_NAME;
      result->value = (VOID *) GetFullInstanceName(dstins);
     }
   return;
   
DuplicateError:
   dstins->busy--;
   QuashInstance(dstins);
   SetEvaluationError(CLIPS_TRUE);
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


