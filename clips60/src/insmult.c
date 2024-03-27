   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*              CLIPS Version 6.00  05/12/93           */
   /*                                                     */
   /*           INSTANCE MULTIFIELD SLOT MODULE           */
   /*******************************************************/

/*************************************************************/
/* Purpose:  Access routines for Instance Multifield Slots   */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
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

#include "argacces.h"
#include "extnfunc.h"
#include "insfun.h"
#include "msgfun.h"
#include "msgpass.h"
#include "multifun.h"
#include "router.h"

#define _INSMULT_SOURCE_
#include "insmult.h"

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */
#define INSERT         0
#define REPLACE        1
#define DELETE         2

/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER

static INSTANCE_TYPE *CheckMultifieldSlotInstance(char *);
static INSTANCE_SLOT *CheckMultifieldSlotModify(int,char *,INSTANCE_TYPE *,
                                       EXPRESSION *,int *,int *,DATA_OBJECT *);
static VOID AssignSlotToDataObject(DATA_OBJECT *,INSTANCE_SLOT *);

#else

static INSTANCE_TYPE *CheckMultifieldSlotInstance();
static INSTANCE_SLOT *CheckMultifieldSlotModify();
static VOID AssignSlotToDataObject();

#endif

/* =========================================
   *****************************************
       EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

#if (! RUN_TIME)

/***************************************************
  NAME         : SetupInstanceMultifieldCommands
  DESCRIPTION  : Defines function interfaces for
                 manipulating instance multislots
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Functions defined to CLIPS
  NOTES        : None
 ***************************************************/
globle VOID SetupInstanceMultifieldCommands()
  {
   /* ===================================
      Old version 5.1 compatibility names
      =================================== */
   DefineFunction2("direct-mv-replace",'b',PTIF DirectMVReplaceCommand,
                   "DirectMVReplaceCommand","4**wii");
   DefineFunction2("direct-mv-insert",'b',PTIF DirectMVInsertCommand,
                   "DirectMVInsertCommand","3**wi");
   DefineFunction2("direct-mv-delete",'b',PTIF DirectMVDeleteCommand,
                   "DirectMVDeleteCommand","33iw");
   DefineFunction2("mv-slot-replace",'u',PTIF MVSlotReplaceCommand,
                   "MVSlotReplaceCommand","5*uewii");
   DefineFunction2("mv-slot-insert",'u',PTIF MVSlotInsertCommand,
                   "MVSlotInsertCommand","4*uewi");
   DefineFunction2("mv-slot-delete",'u',PTIF MVSlotDeleteCommand,
                   "MVSlotDeleteCommand","44iew");

   /* =====================
      New version 6.0 names
      ===================== */
   DefineFunction2("slot-direct-replace$",'b',PTIF DirectMVReplaceCommand,
                   "DirectMVReplaceCommand","4**wii");
   DefineFunction2("slot-direct-insert$",'b',PTIF DirectMVInsertCommand,
                   "DirectMVInsertCommand","3**wi");
   DefineFunction2("slot-direct-delete$",'b',PTIF DirectMVDeleteCommand,
                   "DirectMVDeleteCommand","33iw");
   DefineFunction2("slot-replace$",'u',PTIF MVSlotReplaceCommand,
                   "MVSlotReplaceCommand","5*uewii");
   DefineFunction2("slot-insert$",'u',PTIF MVSlotInsertCommand,
                   "MVSlotInsertCommand","4*uewi");
   DefineFunction2("slot-delete$",'u',PTIF MVSlotDeleteCommand,
                   "MVSlotDeleteCommand","44iew");
  }

#endif

/***********************************************************************************
  NAME         : MVSlotReplaceCommand
  DESCRIPTION  : Allows user to replace a specified field of a multi-value slot
                 The slot is directly read (w/o a get- message) and the new
                   slot-value is placed via a put- message.
                 This function is not valid for single-value slots.
  INPUTS       : Caller's result buffer
  RETURNS      : CLIPS_TRUE if multi-value slot successfully modified,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : Put messsage sent for slot
  NOTES        : CLIPS Syntax : (slot-replace$ <instance> <slot> 
                                 <range-begin> <range-end> <value>)
 ***********************************************************************************/
globle VOID MVSlotReplaceCommand(result)
  DATA_OBJECT *result;
  {
   DATA_OBJECT newval,newseg,oldseg;
   INSTANCE_TYPE *ins;
   INSTANCE_SLOT *sp;
   int rb,re;
   EXPRESSION arg;
   
   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   ins = CheckMultifieldSlotInstance("slot-replace$");
   if (ins == NULL)
     return;
   sp = CheckMultifieldSlotModify(REPLACE,"slot-replace$",ins,
                            GetFirstArgument()->nextArg,&rb,&re,&newval);
   if (sp == NULL)
     return;
   AssignSlotToDataObject(&oldseg,sp);
   if (ReplaceMultiValueField(&newseg,&oldseg,rb,re,&newval,"slot-replace$") == CLIPS_FALSE)
     return;
   arg.type = MULTIFIELD;
   arg.value = (VOID *) &newseg;
   arg.nextArg = NULL;
   arg.argList = NULL;
   DirectMessage(sp->desc->overrideMessage,ins,result,&arg);
  }
  
/***********************************************************************************
  NAME         : MVSlotInsertCommand
  DESCRIPTION  : Allows user to insert a specified field of a multi-value slot
                 The slot is directly read (w/o a get- message) and the new
                   slot-value is placed via a put- message.
                 This function is not valid for single-value slots.
  INPUTS       : Caller's result buffer
  RETURNS      : CLIPS_TRUE if multi-value slot successfully modified, CLIPS_FALSE otherwise
  SIDE EFFECTS : Put messsage sent for slot
  NOTES        : CLIPS Syntax : (slot-insert$ <instance> <slot> <index> <value>)
 ***********************************************************************************/
globle VOID MVSlotInsertCommand(result)
  DATA_OBJECT *result;
  {
   DATA_OBJECT newval,newseg,oldseg;
   INSTANCE_TYPE *ins;
   INSTANCE_SLOT *sp;
   int index;
   EXPRESSION arg;
   
   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   ins = CheckMultifieldSlotInstance("slot-insert$");
   if (ins == NULL)
     return;
   sp = CheckMultifieldSlotModify(INSERT,"slot-insert$",ins,
                            GetFirstArgument()->nextArg,&index,NULL,&newval);
   if (sp == NULL)
     return;
   AssignSlotToDataObject(&oldseg,sp);
   if (InsertMultiValueField(&newseg,&oldseg,index,&newval,"slot-insert$") == CLIPS_FALSE)
     return;
   arg.type = MULTIFIELD;
   arg.value = (VOID *) &newseg;
   arg.nextArg = NULL;
   arg.argList = NULL;
   DirectMessage(sp->desc->overrideMessage,ins,result,&arg);
  }
  
/***********************************************************************************
  NAME         : MVSlotDeleteCommand
  DESCRIPTION  : Allows user to delete a specified field of a multi-value slot
                 The slot is directly read (w/o a get- message) and the new
                   slot-value is placed via a put- message.
                 This function is not valid for single-value slots.
  INPUTS       : Caller's result buffer
  RETURNS      : CLIPS_TRUE if multi-value slot successfully modified, CLIPS_FALSE otherwise
  SIDE EFFECTS : Put message sent for slot
  NOTES        : CLIPS Syntax : (slot-delete$ <instance> <slot>
                                 <range-begin> <range-end>)
 ***********************************************************************************/
globle VOID MVSlotDeleteCommand(result)
  DATA_OBJECT *result;
  {
   DATA_OBJECT newseg,oldseg;
   INSTANCE_TYPE *ins;
   INSTANCE_SLOT *sp;
   int rb,re;
   EXPRESSION arg;
   
   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   ins = CheckMultifieldSlotInstance("slot-delete$");
   if (ins == NULL)
     return;
   sp = CheckMultifieldSlotModify(DELETE,"slot-delete$",ins,
                            GetFirstArgument()->nextArg,&rb,&re,NULL);
   if (sp == NULL)
     return;
   AssignSlotToDataObject(&oldseg,sp);
   if (DeleteMultiValueField(&newseg,&oldseg,rb,re,"slot-delete$") == CLIPS_FALSE)
     return;
   arg.type = MULTIFIELD;
   arg.value = (VOID *) &newseg;
   arg.nextArg = NULL;
   arg.argList = NULL;
   DirectMessage(sp->desc->overrideMessage,ins,result,&arg);
  }
  
/*****************************************************************
  NAME         : DirectMVReplaceCommand
  DESCRIPTION  : Directly replaces a slot's value
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if put OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : Slot modified
  NOTES        : CLIPS Syntax: (direct-slot-replace$ <slot> 
                                <range-begin> <range-end> <value>)
 *****************************************************************/
globle BOOLEAN DirectMVReplaceCommand()
  {
   INSTANCE_SLOT *sp;
   INSTANCE_TYPE *ins;
   int rb,re;
   DATA_OBJECT newval,newseg,oldseg;
   
   if (CheckCurrentMessage("direct-slot-replace$",CLIPS_TRUE) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   ins = GetActiveInstance();
   sp = CheckMultifieldSlotModify(REPLACE,"direct-slot-replace$",ins,
                            GetFirstArgument(),&rb,&re,&newval);
   if (sp == NULL)
     return(CLIPS_FALSE);
   AssignSlotToDataObject(&oldseg,sp);
   if (ReplaceMultiValueField(&newseg,&oldseg,rb,re,&newval,"direct-slot-replace$")
           == CLIPS_FALSE)
     return(CLIPS_FALSE);
   if (PutSlotValue(ins,sp,&newseg,"function direct-slot-replace$"))
     return(CLIPS_TRUE);
   return(CLIPS_FALSE);
  }

/************************************************************************
  NAME         : DirectMVInsertCommand
  DESCRIPTION  : Directly inserts a slot's value
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if put OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : Slot modified
  NOTES        : CLIPS Syntax: (direct-slot-insert$ <slot> <index> <value>)
 ************************************************************************/
globle BOOLEAN DirectMVInsertCommand()
  {
   INSTANCE_SLOT *sp;
   INSTANCE_TYPE *ins;
   int index;
   DATA_OBJECT newval,newseg,oldseg;
   
   if (CheckCurrentMessage("direct-slot-insert$",CLIPS_TRUE) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   ins = GetActiveInstance();
   sp = CheckMultifieldSlotModify(INSERT,"direct-slot-insert$",ins,
                            GetFirstArgument(),&index,NULL,&newval);
   if (sp == NULL)
     return(CLIPS_FALSE);
   AssignSlotToDataObject(&oldseg,sp);
   if (InsertMultiValueField(&newseg,&oldseg,index,&newval,"direct-slot-insert$")
          == CLIPS_FALSE)
     return(CLIPS_FALSE);
   if (PutSlotValue(ins,sp,&newseg,"function direct-slot-insert$"))
     return(CLIPS_TRUE);
   return(CLIPS_FALSE);
  }

/*****************************************************************
  NAME         : DirectMVDeleteCommand
  DESCRIPTION  : Directly deletes a slot's value
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if put OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : Slot modified
  NOTES        : CLIPS Syntax: (direct-slot-delete$ <slot> 
                                <range-begin> <range-end>)
 *****************************************************************/
globle BOOLEAN DirectMVDeleteCommand()
  {
   INSTANCE_SLOT *sp;
   INSTANCE_TYPE *ins;
   int rb,re;
   DATA_OBJECT newseg,oldseg;
   
   if (CheckCurrentMessage("direct-slot-delete$",CLIPS_TRUE) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   ins = GetActiveInstance();
   sp = CheckMultifieldSlotModify(DELETE,"direct-slot-delete$",ins,
                                  GetFirstArgument(),&rb,&re,NULL);
   if (sp == NULL)
     return(CLIPS_FALSE);
   AssignSlotToDataObject(&oldseg,sp);
   if (DeleteMultiValueField(&newseg,&oldseg,rb,re,"direct-slot-delete$")
         == CLIPS_FALSE)
     return(CLIPS_FALSE);
   if (PutSlotValue(ins,sp,&newseg,"function direct-slot-delete$"))
     return(CLIPS_TRUE);
   return(CLIPS_FALSE);
  }
  
/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

/**********************************************************************
  NAME         : CheckMultifieldSlotInstance
  DESCRIPTION  : Gets the instance for the functions slot-replace$,
                    insert and delete
  INPUTS       : The function name
  RETURNS      : The instance address, NULL on errors
  SIDE EFFECTS : None
  NOTES        : None
 **********************************************************************/
static INSTANCE_TYPE *CheckMultifieldSlotInstance(func)
  char *func;
  {
   INSTANCE_TYPE *ins;
   DATA_OBJECT temp;
   
   if (ArgTypeCheck(func,1,INSTANCE_OR_INSTANCE_NAME,&temp) == CLIPS_FALSE)
     {
      SetEvaluationError(CLIPS_TRUE);
      return(NULL);
     }
   if (temp.type == INSTANCE_ADDRESS)
     {
      ins = (INSTANCE_TYPE *) temp.value;
      if (ins->garbage == 1)
        {
         StaleInstanceAddress(func);
         SetEvaluationError(CLIPS_TRUE);
         return(NULL);
        }
     }
   else
     {
      ins = FindInstanceBySymbol((SYMBOL_HN *) temp.value);
      if (ins == NULL)
        NoInstanceError(ValueToString(temp.value),func);
     }
   return(ins);
  }
        
/*********************************************************************
  NAME         : CheckMultifieldSlotModify
  DESCRIPTION  : For the functions slot-replace$, insert, & delete
                    as well as direct-slot-replace$, insert, & delete
                    this function gets the slot, index, and optional
                    field-value for these functions
  INPUTS       : 1) A code indicating the type of operation 
                      INSERT  (0) : Requires one index
                      REPLACE (1) : Requires two indices
                      DELETE  (2) : Requires two indices
                 2) Function name-string
                 3) Instance address
                 4) Argument expression chain
                 5) Caller's buffer for index (or beginning of range)
                 6) Caller's buffer for end of range 
                     (can be NULL for INSERT)
                 7) Caller's new-field value buffer
                     (can be NULL for DELETE)
  RETURNS      : The address of the instance-slot,
                    NULL on errors
  SIDE EFFECTS : Caller's index buffer set
                 Caller's new-field value buffer set (if not NULL)
                   Will allocate an ephemeral segment to store more
                     than 1 new field value
                 EvaluationError set on errors
  NOTES        : Assume the argument chain is at least 2
                   expressions deep - slot, index, and optional values
 *********************************************************************/
static INSTANCE_SLOT *CheckMultifieldSlotModify(code,func,ins,args,rb,re,newval)
  int code;
  char *func;
  INSTANCE_TYPE *ins;
  EXPRESSION *args;
  int *rb,*re;
  DATA_OBJECT *newval;
  {
   DATA_OBJECT temp;
   INSTANCE_SLOT *sp;
   int start;
   
   start = (args == GetFirstArgument()) ? 1 : 2;
   EvaluationError = CLIPS_FALSE;
   EvaluateExpression(args,&temp);
   if (temp.type != SYMBOL)
     {
      ExpectedTypeError1(func,start,"symbol");
      SetEvaluationError(CLIPS_TRUE);
      return(NULL);
     }
   sp = FindInstanceSlot(ins,(SYMBOL_HN *) temp.value);
   if (sp == NULL)
     {
      SlotExistError(ValueToString(temp.value),func);
      return(NULL);
     }
   if (sp->desc->multiple == 0)
     {
      PrintErrorID("INSMULT",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Function ");
      PrintCLIPS(WERROR,func);
      PrintCLIPS(WERROR," cannot be used on single-field slot ");
      PrintCLIPS(WERROR,ValueToString(sp->desc->slotName->name));
      PrintCLIPS(WERROR," in instance ");
      PrintCLIPS(WERROR,ValueToString(ins->name));
      PrintCLIPS(WERROR,".\n");
      SetEvaluationError(CLIPS_TRUE);
      return(NULL);
     }
   EvaluateExpression(args->nextArg,&temp);
   if (temp.type != INTEGER)
     {
      ExpectedTypeError1(func,start+1,"integer");
      SetEvaluationError(CLIPS_TRUE);
      return(NULL);
     }
   args = args->nextArg->nextArg;
   *rb = ValueToInteger(temp.value);
   if ((code == REPLACE) || (code == DELETE))
     {
      EvaluateExpression(args,&temp);
      if (temp.type != INTEGER)
        {
         ExpectedTypeError1(func,start+2,"integer");
         SetEvaluationError(CLIPS_TRUE);
         return(NULL);
        }
      *re = ValueToInteger(temp.value);
      args = args->nextArg;
     }
   if ((code == INSERT) || (code == REPLACE))
     {
      if (EvaluateAndStoreInDataObject(1,args,newval) == CLIPS_FALSE)
        return(NULL);
     }
   return(sp);
  }

/***************************************************
  NAME         : AssignSlotToDataObject
  DESCRIPTION  : Assigns the value of a multifield
                 slot to a data object
  INPUTS       : 1) The data object buffer
                 2) The instance slot
  RETURNS      : Nothing useful
  SIDE EFFECTS : Data object fields set
  NOTES        : Assumes slot is a multislot
 ***************************************************/
static VOID AssignSlotToDataObject(theDataObject,theSlot)
  DATA_OBJECT *theDataObject;
  INSTANCE_SLOT *theSlot;
  {
   theDataObject->type = theSlot->type;
   theDataObject->value = theSlot->value;
   theDataObject->begin = 0;
   theDataObject->end = GetInstanceSlotLength(theSlot) - 1;
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


