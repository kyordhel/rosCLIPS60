   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*              OBJECT MESSAGE DISPATCH CODE           */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
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

#ifndef _CLIPS_STDIO_
#include <stdio.h>
#define _CLIPS_STDIO_
#endif

#include "argacces.h"
#include "classcom.h"
#include "classfun.h"
#include "clipsmem.h"
#include "constrct.h"
#include "exprnpsr.h"
#include "insfun.h"
#include "msgfun.h"
#include "multifld.h"
#include "prcdrfun.h"
#include "prccode.h"
#include "router.h"
#include "utility.h"

#define _MSGPASS_SOURCE_
#include "msgpass.h"

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */

/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER

static VOID PerformMessage(DATA_OBJECT *,EXPRESSION *,SYMBOL_HN *);
static HANDLER_LINK *FindApplicableHandlers(DEFCLASS *,SYMBOL_HN *);
static VOID CallHandlers(DATA_OBJECT *);
static VOID EarlySlotBindError(INSTANCE_TYPE *,DEFCLASS *,unsigned);

#else

static VOID PerformMessage();
static HANDLER_LINK *FindApplicableHandlers();
static VOID CallHandlers();
static VOID EarlySlotBindError();

#endif
      
/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
globle SYMBOL_HN *CurrentMessageName = NULL;
globle HANDLER_LINK *CurrentCore = NULL;

/* =========================================
   *****************************************
      INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
static HANDLER_LINK *TopOfCore = NULL, *NextInCore = NULL;

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/*****************************************************
  NAME         : DirectMessage
  DESCRIPTION  : Plugs in given instance and 
                  performs specified message
  INPUTS       : 1) Message symbolic name
                 2) The instance address
                 3) Address of DATA_OBJECT buffer
                    (NULL if don't care)
                 4) Message argument expressions
  RETURNS      : Nothing useful
  SIDE EFFECTS : Side effects of message execution
  NOTES        : None
 *****************************************************/
globle VOID DirectMessage(msg,ins,resultbuf,remargs)
  SYMBOL_HN *msg;
  INSTANCE_TYPE *ins;
  DATA_OBJECT *resultbuf;
  EXPRESSION *remargs;
  {
   EXPRESSION args;
   DATA_OBJECT temp;
      
   if (resultbuf == NULL)
     resultbuf = &temp;
   args.nextArg = remargs;
   args.argList = NULL;
   args.type = INSTANCE_ADDRESS;
   args.value = (VOID *) ins;
   PerformMessage(resultbuf,&args,msg);
  }
    
/***************************************************
  NAME         : Send
  DESCRIPTION  : C Interface for sending messages to
                  instances
  INPUTS       : 1) The data object of the instance
                 2) The message name-string
                 3) The message arguments string
                    (Constants only)
                 4) Caller's buffer for result
  RETURNS      : Nothing useful
  SIDE EFFECTS : Executes message and stores result
                   caller's buffer
  NOTES        : None
 ***************************************************/
globle VOID Send(idata,msg,args,result)
  DATA_OBJECT *idata,*result;
  char *msg,*args;
  {
   int error;
   EXPRESSION *iexp;
   SYMBOL_HN *msym;

   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   msym = FindSymbol(msg);
   if (msym == NULL)
     {
      PrintNoHandlerError(msg);
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
   iexp = GenConstant(idata->type,idata->value);
   iexp->nextArg = ParseConstantArguments(args,&error);
   if (error == CLIPS_TRUE)
     {
      ReturnExpression(iexp);
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
   PerformMessage(result,iexp,msym);
   ReturnExpression(iexp);
  }

/*****************************************************
  NAME         : DestroyHandlerLinks
  DESCRIPTION  : Iteratively deallocates handler-links
  INPUTS       : The handler-link list
  RETURNS      : Nothing useful
  SIDE EFFECTS : Deallocation of links
  NOTES        : None
 *****************************************************/
globle VOID DestroyHandlerLinks(mhead)
  HANDLER_LINK *mhead;
  {
   HANDLER_LINK *tmp;
   
   while (mhead != NULL)
     {
      tmp = mhead;
      mhead = mhead->nxt;
      tmp->hnd->busy--;
      DecrementDefclassBusyCount((VOID *) tmp->hnd->cls);
      rtn_struct(messageHandlerLink,tmp);
     }
  }
  
/***********************************************************************
  NAME         : SendCommand
  DESCRIPTION  : Determines the applicable handler(s) and sets up the
                   core calling frame.  Then calls the core frame.
  INPUTS       : Caller's space for storing the result of the handler(s)
  RETURNS      : Nothing useful
  SIDE EFFECTS : Any side-effects caused by the execution of handlers in
                   the core framework
  NOTES        : CLIPS Syntax : (send <instance> <hnd> <args>*)
 ***********************************************************************/
globle VOID SendCommand(result)
  DATA_OBJECT *result;
  {
   EXPRESSION args;
   SYMBOL_HN *msg;
   DATA_OBJECT temp;
   
   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   if (ArgTypeCheck("send",2,SYMBOL,&temp) == CLIPS_FALSE)
     return;
   msg = (SYMBOL_HN *) temp.value;
   
   /* ============================================= 
      Get the instance or primitive for the message
      ============================================= */
   args.type = GetFirstArgument()->type;
   args.value = GetFirstArgument()->value;
   args.argList = GetFirstArgument()->argList;
   args.nextArg = GetFirstArgument()->nextArg->nextArg;
   
   PerformMessage(result,&args,msg);
  }

/***************************************************
  NAME         : GetNthMessageArgument
  DESCRIPTION  : Returns the address of the nth
                 (starting at 1) which is an
                 argument of the current message
                 dispatch
  INPUTS       : None
  RETURNS      : The message argument
  SIDE EFFECTS : None
  NOTES        : The active instance is always
                 stored as the first argument (0) in
                 the call frame of the message
 ***************************************************/
globle DATA_OBJECT *GetNthMessageArgument(n)
  int n;
  {
   return(&ProcParamArray[n]);
  }
        
#if IMPERATIVE_MESSAGE_HANDLERS

/*****************************************************
  NAME         : NextHandlerAvailable
  DESCRIPTION  : Determines if there the currently
                   executing handler can call a
                   shadowed handler
                 Used before calling call-next-handler
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if shadow ready, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax: (next-handlerp)
 *****************************************************/
globle int NextHandlerAvailable()
  {
   if (CurrentCore == NULL)
     return(CLIPS_FALSE);
   if (CurrentCore->hnd->type == MAROUND)
     return((NextInCore != NULL) ? CLIPS_TRUE : CLIPS_FALSE);
   if ((CurrentCore->hnd->type == MPRIMARY) && (NextInCore != NULL))
     return((NextInCore->hnd->type == MPRIMARY) ? CLIPS_TRUE : CLIPS_FALSE);
   return(CLIPS_FALSE);
  }
  
/********************************************************
  NAME         : CallNextHandler
  DESCRIPTION  : This function allows around-handlers
                   to execute the rest of the core frame.
                 It also allows primary handlers
                   to execute shadowed primaries.
                   
                 The original handler arguments are
                   left intact.
  INPUTS       : The caller's result-value buffer
  RETURNS      : Nothing useful
  SIDE EFFECTS : The core frame is called and any
                   appropriate changes are made when
                   used in an around handler
                   See CallHandlers()
                 But when call-next-handler is called
                   from a primary, the same shadowed
                   primary is called over and over
                   again for repeated calls to
                   call-next-handler.
  NOTES        : CLIPS Syntax: (call-next-handler) OR
                    (override-next-handler <arg> ...)
 ********************************************************/
globle VOID CallNextHandler(result)
  DATA_OBJECT *result;
  {
   EXPRESSION args;
   int overridep;
   HANDLER_LINK *oldNext,*oldCurrent;
   
   SetpType(result,SYMBOL);
   SetpValue(result,CLIPSFalseSymbol);
   EvaluationError = CLIPS_FALSE;
   if (HaltExecution)
     return;
   if (NextHandlerAvailable() == CLIPS_FALSE)
     {
      PrintErrorID("MSGPASS",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Shadowed message-handlers not applicable in current context.\n");
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
   if (CurrentExpression->value == (VOID *) FindFunction("override-next-handler"))
     {
      overridep = 1;
      args.type = ProcParamArray[0].type;
      if (args.type != MULTIFIELD)
        args.value = (VOID *) ProcParamArray[0].value;
      else
        args.value = (VOID *) &ProcParamArray[0];
      args.nextArg = GetFirstArgument();
      args.argList = NULL;
      PushProcParameters(&args,CountArguments(&args),
                          ValueToString(CurrentMessageName),"message",
                          UnboundHandlerErr);
      if (EvaluationError)
        {
         ReturnFlag = CLIPS_FALSE;
         return;
        }
     }
   else
     overridep = 0;
   oldNext = NextInCore;
   oldCurrent = CurrentCore;
   if (CurrentCore->hnd->type == MAROUND)
     {
      if (NextInCore->hnd->type == MAROUND)
        {
         CurrentCore = NextInCore;
         NextInCore = NextInCore->nxt;
#if DEBUGGING_FUNCTIONS
         if (CurrentCore->hnd->trace)
           WatchHandler(WTRACE,CurrentCore,BEGIN_TRACE);
#endif
         if (CheckHandlerArgCount())
           EvaluateProcActions(CurrentCore->hnd->cls->header.whichModule->theModule,
                               CurrentCore->hnd->actions,
                               CurrentCore->hnd->localVarCount,
                               result,UnboundHandlerErr);
#if DEBUGGING_FUNCTIONS
         if (CurrentCore->hnd->trace)
           WatchHandler(WTRACE,CurrentCore,END_TRACE);
#endif
        }
      else
        CallHandlers(result);
     }
   else
     {
      CurrentCore = NextInCore;
      NextInCore = NextInCore->nxt;
#if DEBUGGING_FUNCTIONS
      if (CurrentCore->hnd->trace)
        WatchHandler(WTRACE,CurrentCore,BEGIN_TRACE);
#endif
      if (CheckHandlerArgCount())
        EvaluateProcActions(CurrentCore->hnd->cls->header.whichModule->theModule,
                            CurrentCore->hnd->actions,
                            CurrentCore->hnd->localVarCount,
                            result,UnboundHandlerErr);
#if DEBUGGING_FUNCTIONS
      if (CurrentCore->hnd->trace)
        WatchHandler(WTRACE,CurrentCore,END_TRACE);
#endif
     }
   NextInCore = oldNext;
   CurrentCore = oldCurrent;
   if (overridep)
     PopProcParameters();
   ReturnFlag = CLIPS_FALSE;
  }

#endif /* IMPERATIVE_MESSAGE_HANDLERS */

/*************************************************************************
  NAME         : FindApplicableOfName
  DESCRIPTION  : Groups all handlers of all types of the specified
                   class of the specified name into the applicable handler
                   list
  INPUTS       : 1) The class address
                 2-3) The tops and bottoms of the four handler type lists:
                      around, before, primary and after
                 4) The message name symbol
  RETURNS      : Nothing useful
  SIDE EFFECTS : Modifies the handler lists to include applicable handlers
  NOTES        : None
 *************************************************************************/
globle VOID FindApplicableOfName(cls,tops,bots,mname)
  DEFCLASS *cls;
  HANDLER_LINK *tops[4],*bots[4];
  SYMBOL_HN *mname;
  {
   register int i,e;
   HANDLER *hnd;
   unsigned *arr;
   HANDLER_LINK *tmp;
   
   i = FindHandlerNameGroup(cls,mname);
   if (i == -1)
     return;
   e = cls->handlerCount-1;
   hnd = cls->handlers;
   arr = cls->handlerOrderMap;
   for ( ; i <= e ; i++)
     {
      if (hnd[arr[i]].name != mname)
        break;
#if ! IMPERATIVE_MESSAGE_HANDLERS
      if ((hnd[arr[i]].type == MPRIMARY) && (tops[MPRIMARY] != NULL))
        continue;
#endif
      tmp = get_struct(messageHandlerLink);
      hnd[arr[i]].busy++;
      IncrementDefclassBusyCount((VOID *) hnd[arr[i]].cls);
      tmp->hnd = &hnd[arr[i]];
      if (tops[tmp->hnd->type] == NULL)
        {
         tmp->nxt = NULL;
         tops[tmp->hnd->type] = bots[tmp->hnd->type] = tmp;
        }
#if AUXILIARY_MESSAGE_HANDLERS
      else if (tmp->hnd->type == MAFTER)
        {
         tmp->nxt = tops[tmp->hnd->type];
         tops[tmp->hnd->type] = tmp;
        }
#endif
      else
        {
         bots[tmp->hnd->type]->nxt = tmp;
         bots[tmp->hnd->type] = tmp;
         tmp->nxt = NULL;
        }
     }
  }
    
/*************************************************************************
  NAME         : JoinHandlerLinks
  DESCRIPTION  : Joins the queues of different handlers together
  INPUTS       : 1-2) The tops and bottoms of the four handler type lists:
                      around, before, primary and after
                 3) The message name symbol
  RETURNS      : The top of the joined lists, NULL on errors
  SIDE EFFECTS : Links all the handler type lists together, or all the
                   lists are destroyed if there are no primary handlers
  NOTES        : None
 *************************************************************************/
globle HANDLER_LINK *JoinHandlerLinks(tops,bots,mname)
  HANDLER_LINK *tops[4],*bots[4];
  SYMBOL_HN *mname;
  {
   register int i;
   HANDLER_LINK *mlink;
   
   if (tops[MPRIMARY] == NULL)
    {
     PrintNoHandlerError(ValueToString(mname));
     for (i = MAROUND ; i <= MAFTER ; i++)
       DestroyHandlerLinks(tops[i]);
     SetEvaluationError(CLIPS_TRUE);
     return(NULL);
    }
   
   mlink = tops[MPRIMARY];
   
#if AUXILIARY_MESSAGE_HANDLERS
   if (tops[MBEFORE] != NULL)
     {
      bots[MBEFORE]->nxt = mlink;
      mlink = tops[MBEFORE];
     }
#endif

#if IMPERATIVE_MESSAGE_HANDLERS
   if (tops[MAROUND] != NULL)
     {
      bots[MAROUND]->nxt = mlink;
      mlink = tops[MAROUND];
     }
#endif

#if AUXILIARY_MESSAGE_HANDLERS
   bots[MPRIMARY]->nxt = tops[MAFTER];
#endif

   return(mlink);
  }
  
/***************************************************
  NAME         : PrintHandlerSlotGetFunction
  DESCRIPTION  : Developer access function for
                 printing direct slot references
                 in message-handlers
  INPUTS       : 1) The logical name of the output
                 2) The bitmap expression
  RETURNS      : Nothing useful
  SIDE EFFECTS : Expression printed
  NOTES        : None
 ***************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintHandlerSlotGetFunction(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   HANDLER_SLOT_REFERENCE *theReference;
   DEFCLASS *theDefclass;
   SLOT_DESC *sd;
   
   theReference = (HANDLER_SLOT_REFERENCE *) ValueToBitMap(theValue);
   PrintCLIPS(logicalName,"?self:[");
   theDefclass = ClassIDMap[theReference->classID];
   PrintCLIPS(logicalName,ValueToString(theDefclass->header.name));
   PrintCLIPS(logicalName,"]");
   sd = theDefclass->instanceTemplate[theDefclass->slotNameMap[theReference->slotID]];
   PrintCLIPS(logicalName,ValueToString(sd->slotName->name));
#else
#if MAC_MPW
#pragma unused(logicalName)
#pragma unused(theValue)
#endif
#endif   
  }
  
/***************************************************
  NAME         : HandlerSlotGetFunction
  DESCRIPTION  : Access function for handling the
                 statically-bound direct slot
                 references in message-handlers
  INPUTS       : 1) The bitmap expression
                 2) A data object buffer
  RETURNS      : CLIPS_TRUE if OK, CLIPS_FALSE
                 on errors
  SIDE EFFECTS : Data object buffer gets value of
                 slot. On errors, buffer gets
                 symbol FALSE, EvaluationError
                 is set and error messages are
                 printed
  NOTES        : It is possible for a handler
                 (attached to a superclass of
                  the currently active instance)
                 containing these static references
                 to be called for an instance
                 which does not contain the slots
                 (e.g., an instance of a subclass
                  where the original slot was
                  no-inherit or the subclass
                  overrode the original slot)
 ***************************************************/
globle BOOLEAN HandlerSlotGetFunction(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   HANDLER_SLOT_REFERENCE *theReference;
   DEFCLASS *theDefclass;
   INSTANCE_TYPE *theInstance;
   INSTANCE_SLOT *sp;
   unsigned instanceSlotIndex;
   
   theReference = (HANDLER_SLOT_REFERENCE *) ValueToBitMap(theValue);
   theInstance = (INSTANCE_TYPE *) ProcParamArray[0].value;
   theDefclass = ClassIDMap[theReference->classID];
   if (theInstance->cls == theDefclass)
     {
      instanceSlotIndex = theInstance->cls->slotNameMap[theReference->slotID];
      sp = theInstance->slotAddresses[instanceSlotIndex - 1];
     }
   else
     {
      if (theReference->slotID > theInstance->cls->maxSlotNameID)
        goto HandlerGetError;
      instanceSlotIndex = theInstance->cls->slotNameMap[theReference->slotID];
      if (instanceSlotIndex == 0)
        goto HandlerGetError;
      instanceSlotIndex--;
      sp = theInstance->slotAddresses[instanceSlotIndex];
      if (sp->desc->cls != theDefclass)
        goto HandlerGetError;
     }
   theResult->type = sp->type;
   theResult->value = sp->value;
   if (sp->type == MULTIFIELD)
     {
      theResult->begin = 0;
      theResult->end = GetInstanceSlotLength(sp) - 1;
     }
   return(CLIPS_TRUE);
   
HandlerGetError:
   EarlySlotBindError(theInstance,theDefclass,theReference->slotID);
   theResult->type = SYMBOL;
   theResult->value = CLIPSFalseSymbol;
   SetEvaluationError(CLIPS_TRUE);
   return(CLIPS_FALSE);
  }
  
/***************************************************
  NAME         : PrintHandlerSlotPutFunction
  DESCRIPTION  : Developer access function for
                 printing direct slot bindings
                 in message-handlers
  INPUTS       : 1) The logical name of the output
                 2) The bitmap expression
  RETURNS      : Nothing useful
  SIDE EFFECTS : Expression printed
  NOTES        : None
 ***************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
globle VOID PrintHandlerSlotPutFunction(logicalName,theValue)
  char *logicalName;
  VOID *theValue;
  {
#if DEVELOPER
   HANDLER_SLOT_REFERENCE *theReference;
   DEFCLASS *theDefclass;
   SLOT_DESC *sd;
   
   theReference = (HANDLER_SLOT_REFERENCE *) ValueToBitMap(theValue);
   PrintCLIPS(logicalName,"(bind ?self:[");
   theDefclass = ClassIDMap[theReference->classID];
   PrintCLIPS(logicalName,ValueToString(theDefclass->header.name));
   PrintCLIPS(logicalName,"]");
   sd = theDefclass->instanceTemplate[theDefclass->slotNameMap[theReference->slotID]];
   PrintCLIPS(logicalName,ValueToString(sd->slotName->name));
   if (GetFirstArgument() != NULL)
     {
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
  
/***************************************************
  NAME         : HandlerSlotPutFunction
  DESCRIPTION  : Access function for handling the
                 statically-bound direct slot
                 bindings in message-handlers
  INPUTS       : 1) The bitmap expression
                 2) A data object buffer
  RETURNS      : CLIPS_TRUE if OK, CLIPS_FALSE
                 on errors
  SIDE EFFECTS : Data object buffer gets symbol
                 TRUE and slot is set. On errors,
                 buffer gets symbol FALSE,
                 EvaluationError is set and error
                 messages are printed
  NOTES        : It is possible for a handler
                 (attached to a superclass of
                  the currently active instance)
                 containing these static references
                 to be called for an instance
                 which does not contain the slots
                 (e.g., an instance of a subclass
                  where the original slot was
                  no-inherit or the subclass
                  overrode the original slot)
 ***************************************************/
globle BOOLEAN HandlerSlotPutFunction(theValue,theResult)
  VOID *theValue;
  DATA_OBJECT *theResult;
  {
   HANDLER_SLOT_REFERENCE *theReference;
   DEFCLASS *theDefclass;
   INSTANCE_TYPE *theInstance;
   INSTANCE_SLOT *sp;
   unsigned instanceSlotIndex;
   
   theReference = (HANDLER_SLOT_REFERENCE *) ValueToBitMap(theValue);
   theInstance = (INSTANCE_TYPE *) ProcParamArray[0].value;
   theDefclass = ClassIDMap[theReference->classID];
   if (theInstance->cls == theDefclass)
     {
      instanceSlotIndex = theInstance->cls->slotNameMap[theReference->slotID];
      sp = theInstance->slotAddresses[instanceSlotIndex - 1];
     }
   else
     {
      if (theReference->slotID > theInstance->cls->maxSlotNameID)
        goto HandlerPutError;
      instanceSlotIndex = theInstance->cls->slotNameMap[theReference->slotID];
      if (instanceSlotIndex == 0)
        goto HandlerPutError;
      instanceSlotIndex--;
      sp = theInstance->slotAddresses[instanceSlotIndex];
      if (sp->desc->cls != theDefclass)
        goto HandlerPutError;
     }
     
   /* =======================================================
      The slot has already been verified not to be read-only.
      However, if it is initialize-only, we need to make sure
      that we are initializing the instance (something we
      could not verify at parse-time)
      ======================================================= */
   if (sp->desc->initializeOnly && (WithinInit == CLIPS_FALSE))
     {
      SlotAccessViolationError(ValueToString(sp->desc->slotName->name),
                               CLIPS_TRUE,(VOID *) theInstance);
      goto HandlerPutError2;
     }
   if (EvaluateAndStoreInDataObject((int) sp->desc->multiple,
                                    GetFirstArgument(),theResult) == CLIPS_FALSE)
      goto HandlerPutError2;
   if (PutSlotValue(theInstance,sp,theResult,NULL) == CLIPS_FALSE)
      goto HandlerPutError2;
   return(CLIPS_TRUE);
   
HandlerPutError:
   EarlySlotBindError(theInstance,theDefclass,theReference->slotID);

HandlerPutError2:
   theResult->type = SYMBOL;
   theResult->value = CLIPSFalseSymbol;
   SetEvaluationError(CLIPS_TRUE);
   return(CLIPS_FALSE);
  }

/*****************************************************
  NAME         : DynamicHandlerGetSlot
  DESCRIPTION  : Directly references a slot's value
                 (uses dynamic binding to lookup slot)
  INPUTS       : The caller's result buffer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Caller's result buffer set
  NOTES        : CLIPS Syntax: (get <slot>)
 *****************************************************/
globle VOID DynamicHandlerGetSlot(result)
  DATA_OBJECT *result;
  {
   INSTANCE_SLOT *sp;
   INSTANCE_TYPE *ins;
   DATA_OBJECT temp;
   
   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   if (CheckCurrentMessage("dynamic-get",CLIPS_TRUE) == CLIPS_FALSE)
     return;
   EvaluateExpression(GetFirstArgument(),&temp);
   if (temp.type != SYMBOL)
     {
      ExpectedTypeError1("dynamic-get",1,"symbol");
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
   ins = GetActiveInstance();
   sp = FindInstanceSlot(ins,(SYMBOL_HN *) temp.value);
   if (sp == NULL)
     {
      SlotExistError(ValueToString(temp.value),"dynamic-get");
      return;
     }
   if ((sp->desc->publicVisibility == 0) &&
       (CurrentCore->hnd->cls != sp->desc->cls))
     {
      SlotVisibilityViolationError(sp->desc,CurrentCore->hnd->cls);
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
   result->type = sp->type;
   result->value = sp->value;
   if (sp->type == MULTIFIELD)
     {
      result->begin = 0;
      result->end = GetInstanceSlotLength(sp) - 1;
     }
  }
  
/***********************************************************
  NAME         : DynamicHandlerPutSlot
  DESCRIPTION  : Directly puts a slot's value
                 (uses dynamic binding to lookup slot)
  INPUTS       : Data obejct buffer for holding slot value
  RETURNS      : Nothing useful
  SIDE EFFECTS : Slot modified - and caller's buffer set
                 to value (or symbol FALSE on errors)
  NOTES        : CLIPS Syntax: (put <slot> <value>*)
 ***********************************************************/
globle VOID DynamicHandlerPutSlot(theResult)
  DATA_OBJECT *theResult;
  {
   INSTANCE_SLOT *sp;
   INSTANCE_TYPE *ins;
   DATA_OBJECT temp;
   
   theResult->type = SYMBOL;
   theResult->value = CLIPSFalseSymbol;
   if (CheckCurrentMessage("dynamic-put",CLIPS_TRUE) == CLIPS_FALSE)
     return;
   EvaluateExpression(GetFirstArgument(),&temp);
   if (temp.type != SYMBOL)
     {
      ExpectedTypeError1("dynamic-put",1,"symbol");
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
   ins = GetActiveInstance();
   sp = FindInstanceSlot(ins,(SYMBOL_HN *) temp.value);
   if (sp == NULL)
     {
      SlotExistError(ValueToString(temp.value),"dynamic-put");
      return;
     }
   if ((sp->desc->noWrite == 0) ? CLIPS_FALSE :
       ((sp->desc->initializeOnly == 0) || (WithinInit == CLIPS_FALSE)))
     {
      SlotAccessViolationError(ValueToString(sp->desc->slotName->name),
                               CLIPS_TRUE,(VOID *) ins);
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
   if ((sp->desc->publicVisibility == 0) &&
       (CurrentCore->hnd->cls != sp->desc->cls))
     {
      SlotVisibilityViolationError(sp->desc,CurrentCore->hnd->cls);
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
   if (EvaluateAndStoreInDataObject((int) sp->desc->multiple,
                     GetFirstArgument()->nextArg,&temp) == CLIPS_FALSE)
     return;
   if (PutSlotValue(ins,sp,&temp,NULL))
     {
      theResult->type = temp.type;
      theResult->value = temp.value;
      theResult->begin = temp.begin;
      theResult->end = temp.end;
     }
  }

/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/*****************************************************
  NAME         : PerformMessage
  DESCRIPTION  : Calls core framework for a message
  INPUTS       : 1) Caller's result buffer
                 2) Message argument expressions
                    (including implicit object)
                 3) Message name
  RETURNS      : Nothing useful
  SIDE EFFECTS : Any side-effects of message execution
                    and caller's result buffer set
  NOTES        : None
 *****************************************************/
static VOID PerformMessage(result,args,mname)
  DATA_OBJECT *result;
  EXPRESSION *args;
  SYMBOL_HN *mname;
  {
   int oldce;
   HANDLER_LINK *oldCore;
   DEFCLASS *cls = NULL;
   INSTANCE_TYPE *ins = NULL;
   SYMBOL_HN *oldName;
   
   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   EvaluationError = CLIPS_FALSE;
   if (HaltExecution)
     return;
   oldce = ExecutingConstruct();
   SetExecutingConstruct(CLIPS_TRUE);
   oldName = CurrentMessageName;
   CurrentMessageName = mname;
   CurrentEvaluationDepth++;
   PushProcParameters(args,CountArguments(args),
                        ValueToString(CurrentMessageName),"message",
                        UnboundHandlerErr);
   if (EvaluationError)
     {
      CurrentEvaluationDepth--;
      CurrentMessageName = oldName;
      PeriodicCleanup(CLIPS_FALSE,CLIPS_TRUE);
      SetExecutingConstruct(oldce);
      return;
     }
   if (ProcParamArray->type == INSTANCE_ADDRESS)
     {
      ins = (INSTANCE_TYPE *) ProcParamArray->value;
      if (ins->garbage == 1)
        {
         StaleInstanceAddress("send");
         SetEvaluationError(CLIPS_TRUE);
        }
      else
        {
         cls = ins->cls;
         ins->busy++;
        }
     }
   else if (ProcParamArray->type == INSTANCE_NAME)
     {
      ins = FindInstanceBySymbol((SYMBOL_HN *) ProcParamArray->value);
      if (ins == NULL)
        {
         PrintErrorID("MSGPASS",2,CLIPS_FALSE);
         PrintCLIPS(WERROR,"No such instance ");
         PrintCLIPS(WERROR,ValueToString((SYMBOL_HN *) ProcParamArray->value));
         PrintCLIPS(WERROR," in function send.\n");
         SetEvaluationError(CLIPS_TRUE);
        }
      else
        {
         ProcParamArray->value = (VOID *) ins;
         ProcParamArray->type = INSTANCE_ADDRESS;
         cls = ins->cls;
         ins->busy++;
        }
     }
   else if ((cls = PrimitiveClassMap[ProcParamArray->type]) == NULL)
     {
      CLIPSSystemError("MSGPASS",1);
      ExitCLIPS(2);
     }
   if (EvaluationError)
     {
      PopProcParameters();
      CurrentEvaluationDepth--;
      CurrentMessageName = oldName;
      PeriodicCleanup(CLIPS_FALSE,CLIPS_TRUE);
      SetExecutingConstruct(oldce);
      return;
     }
   oldCore = TopOfCore;
   TopOfCore = FindApplicableHandlers(cls,mname);
   if (TopOfCore != NULL)
     {
      HANDLER_LINK *oldCurrent,*oldNext;

      oldCurrent = CurrentCore;
      oldNext = NextInCore;
      
#if IMPERATIVE_MESSAGE_HANDLERS

      if (TopOfCore->hnd->type == MAROUND)
        {
         CurrentCore = TopOfCore;
         NextInCore = TopOfCore->nxt;
#if DEBUGGING_FUNCTIONS
         if (WatchMessages)
           WatchMessage(WTRACE,BEGIN_TRACE);
         if (CurrentCore->hnd->trace)
           WatchHandler(WTRACE,CurrentCore,BEGIN_TRACE);
#endif
         if (CheckHandlerArgCount())
           EvaluateProcActions(CurrentCore->hnd->cls->header.whichModule->theModule,
                               CurrentCore->hnd->actions,
                               CurrentCore->hnd->localVarCount,
                               result,UnboundHandlerErr);
#if DEBUGGING_FUNCTIONS
         if (CurrentCore->hnd->trace)
           WatchHandler(WTRACE,CurrentCore,END_TRACE);
         if (WatchMessages)
           WatchMessage(WTRACE,END_TRACE);
#endif
        }
      else
      
#endif  /* IMPERATIVE_MESSAGE_HANDLERS */

        {
         CurrentCore = NULL;
         NextInCore = TopOfCore;
#if DEBUGGING_FUNCTIONS
         if (WatchMessages)
           WatchMessage(WTRACE,BEGIN_TRACE);
#endif
         CallHandlers(result);
#if DEBUGGING_FUNCTIONS
         if (WatchMessages)
           WatchMessage(WTRACE,END_TRACE);
#endif
        }

      DestroyHandlerLinks(TopOfCore);
      CurrentCore = oldCurrent;
      NextInCore = oldNext;
     }
   TopOfCore = oldCore;
   ReturnFlag = CLIPS_FALSE;

   if (ins != NULL)
     ins->busy--;
     
   /* ==================================
      Restore the original calling frame
      ================================== */
   PopProcParameters();
   CurrentEvaluationDepth--;
   CurrentMessageName = oldName;
   PropagateReturnValue(result);
   PeriodicCleanup(CLIPS_FALSE,CLIPS_TRUE);
   SetExecutingConstruct(oldce);
   if (EvaluationError)
     {
      result->type = SYMBOL;
      result->value = CLIPSFalseSymbol;
     }
  }

/*****************************************************************************
  NAME         : FindApplicableHandlers
  DESCRIPTION  : Given a message name, this routine forms the "core frame"
                   for the message : a list of all applicable class handlers.
                   An applicable class handler is one whose name matches
                     the message and whose class matches the instance.
                     
                   The list is in the following order :
                   
                   All around handlers (from most specific to most general)
                   All before handlers (from most specific to most general)
                   All primary handlers (from most specific to most general)
                   All after handlers (from most general to most specific)
                   
  INPUTS       : 1) The class of the instance (or primitive) for the message
                 2) The message name
  RETURNS      : NULL if no applicable handlers or errors,
                   the list of handlers otherwise
  SIDE EFFECTS : Links are allocated for the list
  NOTES        : The instance is the first thing on the ProcParamArray
                 The number of arguments is in ProcParamArraySize
 *****************************************************************************/
static HANDLER_LINK *FindApplicableHandlers(cls,mname)
  DEFCLASS *cls;
  SYMBOL_HN *mname;
  {
   register int i;
   HANDLER_LINK *tops[4],*bots[4];
   
   for (i = MAROUND ; i <= MAFTER ; i++)
     tops[i] = bots[i] = NULL;
     
   for (i = 0 ; i < cls->allSuperclasses.classCount ; i++)
     FindApplicableOfName(cls->allSuperclasses.classArray[i],tops,bots,mname);
   return(JoinHandlerLinks(tops,bots,mname));
  }
  
/***************************************************************
  NAME         : CallHandlers
  DESCRIPTION  : Moves though the current message frame
                   for a send-message as follows :

                 Call all before handlers and ignore their
                   return values.
                 Call the first primary handler and
                   ignore the rest.  The return value
                   of the handler frame is this message's value.
                 Call all after handlers and ignore their
                   return values.
  INPUTS       : Caller's buffer for the return value of
                   the message
  RETURNS      : Nothing useful
  SIDE EFFECTS : The handlers are evaluated.
  NOTES        : IMPORTANT : The global NextInCore should be
                 pointing to the first handler to be executed.
 ***************************************************************/
static VOID CallHandlers(result)
  DATA_OBJECT *result;
  {
   HANDLER_LINK *oldCurrent,*oldNext;
#if AUXILIARY_MESSAGE_HANDLERS
   DATA_OBJECT temp;
#endif
   
   if (HaltExecution)
     return;
     
#if AUXILIARY_MESSAGE_HANDLERS
   oldCurrent = CurrentCore;
   oldNext = NextInCore;

   while (NextInCore->hnd->type == MBEFORE)
     {
      CurrentCore = NextInCore;
      NextInCore = NextInCore->nxt;
#if DEBUGGING_FUNCTIONS
      if (CurrentCore->hnd->trace)
        WatchHandler(WTRACE,CurrentCore,BEGIN_TRACE);
#endif
      if (CheckHandlerArgCount())
        EvaluateProcActions(CurrentCore->hnd->cls->header.whichModule->theModule,
                            CurrentCore->hnd->actions,
                            CurrentCore->hnd->localVarCount,
                            &temp,UnboundHandlerErr);
#if DEBUGGING_FUNCTIONS
      if (CurrentCore->hnd->trace)
        WatchHandler(WTRACE,CurrentCore,END_TRACE);
#endif
      ReturnFlag = CLIPS_FALSE;
      if ((NextInCore == NULL) || HaltExecution)
        {
         NextInCore = oldNext;
         CurrentCore = oldCurrent;
         return;
        }
     }
   if (NextInCore->hnd->type == MPRIMARY)
     {
#endif /* AUXILIARY_MESSAGE_HANDLERS */

      CurrentCore = NextInCore;
      NextInCore = NextInCore->nxt;
#if DEBUGGING_FUNCTIONS
      if (CurrentCore->hnd->trace)
        WatchHandler(WTRACE,CurrentCore,BEGIN_TRACE);
#endif
      if (CheckHandlerArgCount())
        EvaluateProcActions(CurrentCore->hnd->cls->header.whichModule->theModule,
                            CurrentCore->hnd->actions,
                            CurrentCore->hnd->localVarCount,
                            result,UnboundHandlerErr);
#if DEBUGGING_FUNCTIONS
      if (CurrentCore->hnd->trace)
        WatchHandler(WTRACE,CurrentCore,END_TRACE);
#endif
      ReturnFlag = CLIPS_FALSE;

#if AUXILIARY_MESSAGE_HANDLERS
      if ((NextInCore == NULL) || HaltExecution)
        {
         NextInCore = oldNext;
         CurrentCore = oldCurrent;
         return;
        }
      while (NextInCore->hnd->type == MPRIMARY)
        {
         NextInCore = NextInCore->nxt;
         if (NextInCore == NULL)
           {
            NextInCore = oldNext;
            CurrentCore = oldCurrent;
            return;
           }
        }
     }
   while (NextInCore->hnd->type == MAFTER)
     {
      CurrentCore = NextInCore;
      NextInCore = NextInCore->nxt;
#if DEBUGGING_FUNCTIONS
      if (CurrentCore->hnd->trace)
        WatchHandler(WTRACE,CurrentCore,BEGIN_TRACE);
#endif
      if (CheckHandlerArgCount())
        EvaluateProcActions(CurrentCore->hnd->cls->header.whichModule->theModule,
                            CurrentCore->hnd->actions,
                            CurrentCore->hnd->localVarCount,
                            &temp,UnboundHandlerErr);
#if DEBUGGING_FUNCTIONS
      if (CurrentCore->hnd->trace)
        WatchHandler(WTRACE,CurrentCore,END_TRACE);
#endif
      ReturnFlag = CLIPS_FALSE;
      if ((NextInCore == NULL) || HaltExecution)
        {
         NextInCore = oldNext;
         CurrentCore = oldCurrent;
         return;
        }
     }
#endif /*  AUXILIARY_MESSAGE_HANDLERS */

   NextInCore = oldNext;
   CurrentCore = oldCurrent;
  }
    
  
/********************************************************
  NAME         : EarlySlotBindError
  DESCRIPTION  : Prints out an error message when
                 a message-handler from a superclass
                 which contains a static-bind
                 slot access is not valid for the
                 currently active instance (i.e.
                 the instance is not using the
                 superclass's slot)
  INPUTS       : 1) The currently active instance
                 2) The defclass holding the invalid slot
                 3) The canonical id of the slot
  RETURNS      : Nothing useful
  SIDE EFFECTS : Error message printed
  NOTES        : None
 ********************************************************/
static VOID EarlySlotBindError(theInstance,theDefclass,slotID)
  INSTANCE_TYPE *theInstance;
  DEFCLASS *theDefclass;
  unsigned slotID;
  {
   SLOT_DESC *sd;
   
   sd = theDefclass->instanceTemplate[theDefclass->slotNameMap[slotID] - 1];
   PrintErrorID("MSGPASS",3,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Static reference to slot ");
   PrintCLIPS(WERROR,ValueToString(sd->slotName->name));
   PrintCLIPS(WERROR," of class ");
   PrintClassName(WERROR,theDefclass,CLIPS_FALSE);
   PrintCLIPS(WERROR," does not apply to ");
   PrintInstanceNameAndClass(WERROR,theInstance,CLIPS_TRUE);
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
