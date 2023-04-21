   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             PROCEDURAL FUNCTIONS MODULE             */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _PRCDRFUN_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_

#include "setup.h"

#include "clipsmem.h"
#include "router.h"
#include "argacces.h"
#include "constrnt.h"
#include "cstrnchk.h"
#include "cstrnops.h"
#include "multifld.h"
#include "exprnpsr.h"
#include "scanner.h"
#include "utility.h"
#include "prcdrpsr.h"
#include "prcdrfun.h"

#if DEFGLOBAL_CONSTRUCT 
#include "globldef.h" 
#endif

typedef struct loopCounterStack
  {
   long loopCounter;
   struct loopCounterStack *nxt;
  } LOOP_COUNTER_STACK;
  
/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle int                 ReturnFlag = CLIPS_FALSE;
   globle int                 BreakFlag = CLIPS_FALSE;

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static LOOP_COUNTER_STACK *LoopCounterStack = NULL;
   static struct dataObject  *BindList = NULL;

#if ! RUN_TIME
/*******************************************/
/* ProceduralFunctionDefinitions        */
/*******************************************/
globle VOID ProceduralFunctionDefinitions()
  {
   DefineFunction2("if",            'u', PTIF IfFunction,           "IfFunction", NULL);
   DefineFunction2("while",         'u', PTIF WhileFunction,        "WhileFunction", NULL);
   DefineFunction2("loop-for-count",'u', PTIF LoopForCountFunction, "LoopForCountFunction", NULL);
   DefineFunction2("(get-loop-count)",'l', PTIF GetLoopCount,       "GetLoopCount", NULL);
   DefineFunction2("bind",          'u', PTIF BindFunction,         "BindFunction", NULL);
   DefineFunction2("progn",         'u', PTIF PrognFunction,        "PrognFunction", NULL);
   DefineFunction2("return",        'u', PTIF ReturnFunction,       "ReturnFunction",NULL);
   DefineFunction2("break",         'v', PTIF BreakFunction,        "BreakFunction",NULL);
   DefineFunction2("switch",        'u', PTIF SwitchFunction,       "SwitchFunction",NULL);

   ProceduralFunctionParsers();

   FuncSeqOvlFlags("progn",CLIPS_FALSE,CLIPS_FALSE);
   FuncSeqOvlFlags("if",CLIPS_FALSE,CLIPS_FALSE);
   FuncSeqOvlFlags("while",CLIPS_FALSE,CLIPS_FALSE);
   FuncSeqOvlFlags("loop-for-count",CLIPS_FALSE,CLIPS_FALSE);
   FuncSeqOvlFlags("return",CLIPS_FALSE,CLIPS_FALSE);
   FuncSeqOvlFlags("switch",CLIPS_FALSE,CLIPS_FALSE);
  }
#endif

/********************************************************************/
/* WhileFunction:                                                        */
/********************************************************************/
globle VOID WhileFunction(while_result)
  DATA_OBJECT_PTR while_result;
  {
   DATA_OBJECT arg_ptr;

   CurrentEvaluationDepth++;
   RtnUnknown(1,&arg_ptr);
   while (((arg_ptr.value != CLIPSFalseSymbol) ||
           (arg_ptr.type != SYMBOL)) &&
           (HaltExecution != CLIPS_TRUE))
     {
      if ((BreakFlag == CLIPS_TRUE) || (ReturnFlag == CLIPS_TRUE))
        break;
      RtnUnknown(2,&arg_ptr);
      CurrentEvaluationDepth--;
      PeriodicCleanup(CLIPS_FALSE,CLIPS_TRUE);
      CurrentEvaluationDepth++;
      if ((BreakFlag == CLIPS_TRUE) || (ReturnFlag == CLIPS_TRUE))
        break;
      RtnUnknown(1,&arg_ptr);
     }
   CurrentEvaluationDepth--;

   BreakFlag = CLIPS_FALSE;
   if (ReturnFlag == CLIPS_TRUE)
     {
      while_result->type = arg_ptr.type;
      while_result->value = arg_ptr.value;
     }
   else
     {
      while_result->type = SYMBOL;
      while_result->value = CLIPSFalseSymbol;
     }
  }
  
/********************************************************************/
/* LoopForCountFunction:                                            */
/********************************************************************/
globle VOID LoopForCountFunction(loopResult)
  DATA_OBJECT_PTR loopResult;
  {
   DATA_OBJECT arg_ptr;
   long iterationEnd;
   LOOP_COUNTER_STACK *tmpCounter;

   tmpCounter = get_struct(loopCounterStack);
   tmpCounter->loopCounter = 0L;
   tmpCounter->nxt = LoopCounterStack;
   LoopCounterStack = tmpCounter;
   if (ArgTypeCheck("loop-for-count",1,INTEGER,&arg_ptr) == CLIPS_FALSE)
     {
      loopResult->type = SYMBOL;
      loopResult->value = CLIPSFalseSymbol;
      LoopCounterStack = tmpCounter->nxt;
      rtn_struct(loopCounterStack,tmpCounter);
      return;
     }
   tmpCounter->loopCounter = DOToLong(arg_ptr);
   if (ArgTypeCheck("loop-for-count",2,INTEGER,&arg_ptr) == CLIPS_FALSE)
     {
      loopResult->type = SYMBOL;
      loopResult->value = CLIPSFalseSymbol;
      LoopCounterStack = tmpCounter->nxt;
      rtn_struct(loopCounterStack,tmpCounter);
      return;
     }
   iterationEnd = DOToLong(arg_ptr);
   while ((tmpCounter->loopCounter <= iterationEnd) &&
          (HaltExecution != CLIPS_TRUE))
     {
      if ((BreakFlag == CLIPS_TRUE) || (ReturnFlag == CLIPS_TRUE))
        break;
      CurrentEvaluationDepth++;
      RtnUnknown(3,&arg_ptr);
      CurrentEvaluationDepth--;
      PeriodicCleanup(CLIPS_FALSE,CLIPS_TRUE);
      if ((BreakFlag == CLIPS_TRUE) || (ReturnFlag == CLIPS_TRUE))
        break;
      tmpCounter->loopCounter++;
     }

   BreakFlag = CLIPS_FALSE;
   if (ReturnFlag == CLIPS_TRUE)
     {
      loopResult->type = arg_ptr.type;
      loopResult->value = arg_ptr.value;
     }
   else
     {
      loopResult->type = SYMBOL;
      loopResult->value = CLIPSFalseSymbol;
     }
   LoopCounterStack = tmpCounter->nxt;
   rtn_struct(loopCounterStack,tmpCounter);
  }

/************************************************/
/* GetLoopCount                                 */
/************************************************/
globle long GetLoopCount()
  {
   int depth;
   LOOP_COUNTER_STACK *tmpCounter;
   
   depth = ValueToInteger(GetFirstArgument()->value);
   tmpCounter = LoopCounterStack;
   while (depth > 0)
     {
      tmpCounter = tmpCounter->nxt;
      depth--;
     }
   return(tmpCounter->loopCounter);
  }

/********************************************************************/
/* IfFunction:                                                      */
/********************************************************************/
globle VOID IfFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   int num_a;

   if ((num_a = ArgRangeCheck("if",2,3)) == -1)
     {
      returnValue->type = SYMBOL;
      returnValue->value = CLIPSFalseSymbol;
      return;
     }

   RtnUnknown(1,returnValue);
   if ((BreakFlag == CLIPS_TRUE) || (ReturnFlag == CLIPS_TRUE))
     return;

   /* IMPORTANT: Check neccessary tests */

   if ((returnValue->value == CLIPSFalseSymbol) &&
       (returnValue->type == SYMBOL) &&
       (num_a == 3))
     {
      RtnUnknown(3,returnValue);
      return;
     }
   else if ((returnValue->value != CLIPSFalseSymbol) ||
            (returnValue->type != SYMBOL))
     {
      RtnUnknown(2,returnValue);
      return;
     }

   returnValue->type = SYMBOL;
   returnValue->value = CLIPSFalseSymbol;
   return;
  }

/*******************************************/
/* BindFunction:  Performs bind operation. */
/*******************************************/
globle VOID BindFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DATA_OBJECT *bind_ptr, *last_bind;
   int found = CLIPS_FALSE;
   SYMBOL_HN *variableName = NULL;
#if DEFGLOBAL_CONSTRUCT
   struct defglobal *theGlobal = NULL;
#endif
   
   /*===============================================*/
   /* Determine the name of the variable to be set. */
   /*===============================================*/
   
#if DEFGLOBAL_CONSTRUCT
   if (GetFirstArgument()->type == DEFGLOBAL_PTR)
     { theGlobal = (struct defglobal *) GetFirstArgument()->value; }
   else 
#endif
     {
      EvaluateExpression(GetFirstArgument(),returnValue);
      variableName = (SYMBOL_HN *) DOPToPointer(returnValue);
     }
      
   /*===========================================*/
   /* Determine the new value for the variable. */
   /*===========================================*/

   if (GetFirstArgument()->nextArg == NULL)
     returnValue->value = NULL;
   else if (GetFirstArgument()->nextArg->nextArg == NULL)
     EvaluateExpression(GetFirstArgument()->nextArg,returnValue);
    else
     StoreInMultifield(returnValue,GetFirstArgument()->nextArg,CLIPS_TRUE);
     
   /*==================================*/
   /* Bind a defglobal if appropriate. */
   /*==================================*/

#if DEFGLOBAL_CONSTRUCT
   if (theGlobal != NULL)
     {
      QSetDefglobalValue(theGlobal,returnValue);
      return;
     }
#endif

   /*===============================================*/
   /* Search for the variable in the list of binds. */
   /*===============================================*/

   bind_ptr = BindList;
   last_bind = NULL;

   while ((bind_ptr != NULL) && (found == CLIPS_FALSE))
     {
      if (bind_ptr->supplementalInfo == (VOID *) variableName)
        { found = CLIPS_TRUE; }
      else
        {
         last_bind = bind_ptr;
         bind_ptr = bind_ptr->next;
        }
     }

   /*========================================================*/
   /* If variable was not in the list of binds, then add it. */
   /* Make sure that this operation preserves the bind list  */
   /* as a stack.                                            */
   /*========================================================*/

   if (found == CLIPS_FALSE)
     {
      if (returnValue->value != NULL)
        {
         bind_ptr = get_struct(dataObject);
         bind_ptr->supplementalInfo = (VOID *) variableName;
         bind_ptr->next = NULL;
         if (last_bind == NULL)
           { BindList = bind_ptr; }
         else
           { last_bind->next = bind_ptr; }
        }
      else
        {
         returnValue->type = SYMBOL;
         returnValue->value = CLIPSFalseSymbol;
         return;
        }
     }
   else
     { ValueDeinstall(bind_ptr); }

   /*================================*/
   /* Set the value of the variable. */
   /*================================*/
   if (returnValue->value != NULL)
     {
      bind_ptr->type = returnValue->type;
      bind_ptr->value = returnValue->value;
      bind_ptr->begin = returnValue->begin;
      bind_ptr->end = returnValue->end;
      ValueInstall(returnValue);
     }
   else
     {
      if (last_bind == NULL)
        BindList = bind_ptr->next;
      else
        last_bind->next = bind_ptr->next;
      rtn_struct(dataObject,bind_ptr);
      returnValue->type = SYMBOL;
      returnValue->value = CLIPSFalseSymbol;
     }
  }
  
/*********************************************************************/
/* GetBoundVariable: Searches the BindList for a specified variable. */
/*********************************************************************/
globle BOOLEAN GetBoundVariable(vPtr,varName)
  DATA_OBJECT_PTR vPtr;
  SYMBOL_HN *varName;
  {
   DATA_OBJECT_PTR bindPtr;

   bindPtr = BindList;
   while (bindPtr != NULL)
     {
      if (bindPtr->supplementalInfo == (VOID *) varName)
        {
         vPtr->type = bindPtr->type;
         vPtr->value = bindPtr->value;
         vPtr->begin = bindPtr->begin;
         vPtr->end = bindPtr->end;
         return(CLIPS_TRUE);
        }

      bindPtr = bindPtr->next;
     }
   return(CLIPS_FALSE);
  }
  
/*************************************************/
/* FlushBindList: Removes all variables from the */
/*   list of currently bound local variables.    */
/*************************************************/
globle VOID FlushBindList()
  {
   ReturnValues(BindList);
   BindList = NULL;
  }
  
/****************************************/
/* PrognFunction                                */
/****************************************/
globle VOID PrognFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   int numa, i;

   numa = RtnArgCount();

   if (numa == 0)
     {
      returnValue->type = SYMBOL;
      returnValue->value = CLIPSFalseSymbol;
      return;
     }

   i = 1;
   while ((i <= numa) && (GetHaltExecution() != CLIPS_TRUE))
     {
      RtnUnknown(i,returnValue);
      if ((BreakFlag == CLIPS_TRUE) || (ReturnFlag == CLIPS_TRUE))
        break;
      i++;
     }

   if (GetHaltExecution() == CLIPS_TRUE)
     {
      returnValue->type = SYMBOL;
      returnValue->value = CLIPSFalseSymbol;
      return;
     }

   return;
  }

/****************************************/
/* ReturnFunction                       */
/****************************************/
globle VOID ReturnFunction(result)
  DATA_OBJECT_PTR result;
  {
   if (RtnArgCount() == 0)
     {
      result->type = RVOID;
      result->value = CLIPSFalseSymbol;
     }
   else
     RtnUnknown(1,result);
   ReturnFlag = CLIPS_TRUE;
  }

/****************************************/
/* BreakFunction                        */
/****************************************/
globle VOID BreakFunction()
  {
   BreakFlag = CLIPS_TRUE;
  }

/****************************************/
/* SwitchFunction                       */
/****************************************/
globle VOID SwitchFunction(result)
  DATA_OBJECT_PTR result;
  {
   DATA_OBJECT switch_val,case_val;
   EXPRESSION *exp;
   
   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   
   /* ==========================
      Get the value to switch on
      ========================== */
   EvaluateExpression(GetFirstArgument(),&switch_val);
   if (EvaluationError)
     return;
   for (exp = GetFirstArgument()->nextArg ; exp != NULL ; exp = exp->nextArg->nextArg)
     {
      /* =================================================
         RVOID is the default case (if any) for the switch
         ================================================= */
      if (exp->type == RVOID)
        {
         EvaluateExpression(exp->nextArg,result);
         return;
        }
        
      /* ====================================================
         If the case matches, evaluate the actions and return
         ==================================================== */
      EvaluateExpression(exp,&case_val);
      if (EvaluationError)
        return;
      if (switch_val.type == case_val.type)
        {
         if ((case_val.type == MULTIFIELD) ? MultifieldDOsEqual(&switch_val,&case_val) :
             (switch_val.value == case_val.value))
           {
            EvaluateExpression(exp->nextArg,result);
            return;
           }
        }
     }
  }


  


