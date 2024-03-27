   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             DEFTEMPLATE COMMANDS MODULE             */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define  _TMPLTCOM_SOURCE_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT

#include <stdio.h>

#define _CLIPS_STDIO_

#include <string.h>

#include "extnfunc.h"
#include "constrct.h"
#include "router.h"
#include "argacces.h"
#include "tmpltfun.h"
#include "tmpltpsr.h"
#include "tmpltdef.h"

#if (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE) && (! RUN_TIME)
#include "bload.h"
#include "tmpltbin.h"
#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
#include "tmpltcmp.h"
#endif

#include "tmpltcom.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                    DuplicateModifyCommand(int,DATA_OBJECT_PTR);
   static struct templateSlot    *GetNthSlot(struct deftemplate *,int);
#else
   static VOID                    DuplicateModifyCommand();
   static struct templateSlot    *GetNthSlot();
#endif

/*********************************************************/
/* DeftemplateCommands: Set up the deftemplate commands. */
/*********************************************************/
globle VOID DeftemplateCommands()
  {    
#if ! RUN_TIME          
   DefineFunction("modify",'u', PTIF ModifyCommand,"ModifyCommand");
   DefineFunction("duplicate",'u', PTIF DuplicateCommand,"DuplicateCommand");

#if (! BLOAD_ONLY)
   AddFunctionParser("modify",ModifyParse);
   AddFunctionParser("duplicate",DuplicateParse);
#endif
   FuncSeqOvlFlags("modify",CLIPS_FALSE,CLIPS_FALSE);
   FuncSeqOvlFlags("duplicate",CLIPS_FALSE,CLIPS_FALSE);
#endif
  }
  
/********************************************************************/
/* ModifyCommand: High level stub for the modify command. Calls the */
/*   DuplicateModifyCommand function to perform the actual work.    */
/********************************************************************/
globle VOID ModifyCommand(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DuplicateModifyCommand(CLIPS_TRUE,returnValue);
  }

/**********************************************************************/
/* DuplicateCommand: High level stub for the duplicate command. Calls */
/*   the DuplicateModifyCommand function to perform the actual work.  */
/**********************************************************************/
globle VOID DuplicateCommand(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DuplicateModifyCommand(CLIPS_FALSE,returnValue);
  }

/***************************************************************/
/* DuplicateModifyCommand: Implements the duplicate and modify */
/*   commands. The fact being duplicated or modified is first  */
/*   copied to a new fact. Replacements to the fields of the   */
/*   new fact are then made. If a modify command is being      */
/*   performed, the original fact is retracted. Lastly, the    */
/*   new fact is asserted.                                     */
/***************************************************************/
static VOID DuplicateModifyCommand(retractIt,returnValue)
  int retractIt;
  DATA_OBJECT_PTR returnValue;
  {
   long int factNum;
   struct fact *oldFact, *newFact, *theFact;
   struct expr *testPtr;
   DATA_OBJECT computeResult;
   struct deftemplate *templatePtr;
   struct templateSlot *slotPtr;
   int i, position, found;
   
   /*===================================================*/
   /* Set the default return value to the symbol FALSE. */
   /*===================================================*/

   SetpType(returnValue,SYMBOL);
   SetpValue(returnValue,CLIPSFalseSymbol);
   
   /*=============================================*/
   /* Get a pointer to the fact to be duplicated. */
   /*=============================================*/

   testPtr = GetFirstArgument();

   EvaluateExpression(testPtr,&computeResult);
   if (computeResult.type == INTEGER)
     {
      factNum = ValueToLong(computeResult.value);
      if (factNum < 0)
        {
         if (retractIt) ExpectedTypeError2("modify",1);
         else ExpectedTypeError2("duplicate",1);
         SetEvaluationError(CLIPS_TRUE);
         return;
        }

      oldFact = (struct fact *) GetNextFact(NULL);
      while (oldFact != NULL)
        {
         if (oldFact->factIndex == factNum)
           { break; }
         else
           { oldFact = oldFact->nextFact; }
        }

      if (oldFact == NULL)
        {
         char tempBuffer[20];
         sprintf(tempBuffer,"f-%ld",factNum);
         CantFindItemErrorMessage("fact",tempBuffer);
         return;
        }
     }
   else if (computeResult.type == FACT_ADDRESS)
     { oldFact = (struct fact *) computeResult.value; }
   else
     {
      if (retractIt) ExpectedTypeError2("modify",1);
      else ExpectedTypeError2("duplicate",1);
      SetEvaluationError(CLIPS_TRUE);
      return;
     }

   testPtr = testPtr->nextArg;

   /*===============================*/
   /* See if it is a template fact. */
   /*===============================*/

   templatePtr = oldFact->whichDeftemplate;

   if (templatePtr->implied) return;
   
   /*================================================================*/
   /* Duplicate the values from the old fact (skipping multifields). */
   /*================================================================*/

   newFact = (struct fact *) CreateFactBySize((int) oldFact->theProposition.multifieldLength);
   newFact->whichDeftemplate = templatePtr;
   for (i = 0; i < oldFact->theProposition.multifieldLength; i++)
     {            
      newFact->theProposition.theFields[i].type = oldFact->theProposition.theFields[i].type;
      if (newFact->theProposition.theFields[i].type != MULTIFIELD)
        { newFact->theProposition.theFields[i].value = oldFact->theProposition.theFields[i].value; }
      else
        { newFact->theProposition.theFields[i].value = NULL; }
     }

   /*========================*/
   /* Start replacing slots. */
   /*========================*/

   while (testPtr != NULL)
     {
      /*============================================================*/
      /* If the slot identifier is an integer, then the slot was    */
      /* previously identified and its position within the template */
      /* was stored. Otherwise, the position of the slot within the */
      /* deftemplate has to be determined by comparing the name of  */
      /* the slot against the list of slots for the deftemplate.    */
      /*============================================================*/

      if (testPtr->type == INTEGER)
        { position = (int) ValueToLong(testPtr->value); }
      else
        {
         found = CLIPS_FALSE;
         position = 0;
         slotPtr = templatePtr->slotList;
         while (slotPtr != NULL)
           {
            if (slotPtr->slotName == (SYMBOL_HN *) testPtr->value)
              {
               found = CLIPS_TRUE;
               slotPtr = NULL;
              }
            else
              {
               slotPtr = slotPtr->next;
               position++;
              }
           }

         if (! found)
           {
            CLIPSSystemError("TMPLTCOM",1);
            SetEvaluationError(CLIPS_TRUE);
            ReturnFact(newFact);
            return;
           }
        }

      /*===================================================*/
      /* If a single field slot is being replaced, then... */
      /*===================================================*/

      if (newFact->theProposition.theFields[position].type != MULTIFIELD)
        {
         if ((testPtr->argList == NULL) ? TRUE : (testPtr->argList->nextArg != NULL))
           {
            MultiIntoSingleFieldSlotError(GetNthSlot(templatePtr,position),templatePtr);
            ReturnFact(newFact);
            return;
           }
           
         EvaluateExpression(testPtr->argList,&computeResult);
         SetEvaluationError(CLIPS_FALSE);

         /* Multifield values cannot be placed in single field slots. */
         if (computeResult.type == MULTIFIELD)
           {
            ReturnFact(newFact);
            MultiIntoSingleFieldSlotError(GetNthSlot(templatePtr,position),templatePtr);
            return;
           }

         newFact->theProposition.theFields[position].type = computeResult.type;
         newFact->theProposition.theFields[position].value = computeResult.value;
        }

      /*=================================*/
      /* Else replace a multifield slot. */
      /*=================================*/

      else
        {
         StoreInMultifield(&computeResult,testPtr->argList,CLIPS_FALSE);
         SetEvaluationError(CLIPS_FALSE);
         
         newFact->theProposition.theFields[position].type = computeResult.type;
         newFact->theProposition.theFields[position].value = computeResult.value;
        }

      testPtr = testPtr->nextArg;
     }
     
   /*=========================================================*/
   /* Copy old fact multifield values that were not replaced. */
   /*=========================================================*/
     
   for (i = 0; i < oldFact->theProposition.multifieldLength; i++)
     {            
      if ((newFact->theProposition.theFields[i].type == MULTIFIELD) &&
          (newFact->theProposition.theFields[i].value == NULL))
          
        { 
         newFact->theProposition.theFields[i].value = CopyMultifield(oldFact->theProposition.theFields[i].value);
        }
     }

   /*======================================*/
   /* Perform the duplicate/modify action. */
   /*======================================*/
   
   if (retractIt) Retract(oldFact);
   theFact = (struct fact *) Assert(newFact);
   
   /*========================================*/
   /* The asserted fact is the return value. */
   /*========================================*/

   if (theFact != NULL)
     {
      SetpDOBegin(returnValue,1);
      SetpDOEnd(returnValue,theFact->theProposition.multifieldLength);
      SetpType(returnValue,FACT_ADDRESS);
      SetpValue(returnValue,(VOID *) theFact);
     }
     
   return;
  }

/*******************************************************/
/* GetNthSlot                  */
/*******************************************************/
static struct templateSlot *GetNthSlot(theDeftemplate,position)
  struct deftemplate *theDeftemplate;
  int position;
  {
   struct templateSlot *slotPtr;
   int i = 0;
   
   slotPtr = theDeftemplate->slotList;
   while (slotPtr != NULL)
     {
      if (i == position) return(slotPtr);
      slotPtr = slotPtr->next;
      i++;
     }

   return(NULL);
  }

  
#endif
