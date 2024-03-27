   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             DEFTEMPLATE FUNCTIONS MODULE            */
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

#define _TMPLTFUN_SOURCE_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "constant.h"
#include "clipsmem.h"
#include "symbol.h"
#include "scanner.h"
#include "exprnpsr.h"
#include "argacces.h"
#include "router.h"
#include "cstrnchk.h"
#include "default.h"
#include "factmngr.h"
#include "commline.h"
#include "factrhs.h"
#include "modulutl.h"
#include "reorder.h"
#include "tmpltdef.h"
#include "tmpltlhs.h"
#include "tmpltrhs.h"

#include "tmpltfun.h"

#if ANSI_COMPILER
#if (! RUN_TIME) && (! BLOAD_ONLY)
   static struct expr            *ModAndDupParse(struct expr *,char *,char *);
   static SYMBOL_HN              *FindTemplateForFactAddress(SYMBOL_HN *,struct lhsParseNode *);
   static int                     FindSlotPosition(struct deftemplate *,struct symbolHashNode *);
#endif
#else
#if (! RUN_TIME) && (! BLOAD_ONLY)
   static struct expr            *ModAndDupParse();
   static SYMBOL_HN              *FindTemplateForFactAddress();
   static int                     FindSlotPosition();
#endif
#endif

/***********************************************************************/
/* CheckRHSSlotTypes: Checks the validity of a change to a slot as the */
/*   result of an assert, modify, or duplicate command. This checking  */
/*   is performed statically (i.e. when the command is being parsed).  */
/***********************************************************************/
globle BOOLEAN CheckRHSSlotTypes(rhsSlots,slotPtr,thePlace)
  struct expr *rhsSlots;
  struct templateSlot *slotPtr;
  char *thePlace;
  {
   int rv;
   char *theName;

   if (GetStaticConstraintChecking() == CLIPS_FALSE) return(CLIPS_TRUE);
      rv = ConstraintCheckExpressionChain(rhsSlots,slotPtr->constraints);
      if (rv != NO_VIOLATION)
        {
         if (rv != CARDINALITY_VIOLATION) theName = "A literal slot value";
         else theName = "Literal slot values";
         ConstraintViolationErrorMessage(theName,thePlace,CLIPS_TRUE,0,
                                         slotPtr->slotName,0,rv,slotPtr->constraints,CLIPS_TRUE);
         return(0);
        }
        
   return(1);
  }
  
#if (! RUN_TIME) && (! BLOAD_ONLY)

/***************************************************************/
/* UpdateModifyDuplicate: Changes the modify/duplicate command */
/*   found on the RHS of a rule such that the positions of the */
/*   slots for replacement are stored rather than the slot     */
/*   name which allows quicker replacement of slots. This      */
/*   substitution can only take place when the deftemplate     */
/*   type is known (i.e. if a fact-index is used you don't     */
/*   know which type of deftemplate is going to be replaced    */
/*   until you actually do the replacement of slots).          */
/***************************************************************/
globle BOOLEAN UpdateModifyDuplicate(top,name,vTheLHS)
  struct expr *top;
  char *name;
  VOID *vTheLHS;
  {
   struct expr *functionArgs, *tempArg;
   SYMBOL_HN *templateName = NULL;
   struct deftemplate *theDeftemplate;
   struct templateSlot *slotPtr;
   int position;

   /*========================================*/
   /* Determine the fact-address or index to */
   /* be retracted by the modify command.    */
   /*========================================*/

   functionArgs = top->argList;
   if (functionArgs->type == SF_VARIABLE)
     {
      templateName = FindTemplateForFactAddress((SYMBOL_HN *) functionArgs->value,vTheLHS);
      if (templateName == NULL) return(CLIPS_TRUE);
     }
   else
     { return(CLIPS_TRUE); }

   /*========================================*/
   /* Make sure that the fact being modified */
   /* has a corresponding deftemplate.       */
   /*========================================*/

   theDeftemplate = (struct deftemplate *)
                    LookupConstruct(DeftemplateConstruct,
                                    ValueToString(templateName),
                                    CLIPS_FALSE);
                                    
   if (theDeftemplate == NULL) return(CLIPS_TRUE);
     
   if (theDeftemplate->implied) return(CLIPS_TRUE);

   /*=============================================================*/
   /* Make sure all the slot names are valid for the deftemplate. */
   /*=============================================================*/

   tempArg = functionArgs->nextArg;
   while (tempArg != NULL)
     {
      /*======================*/
      /* Does the slot exist? */
      /*======================*/

      if ((slotPtr = FindSlot(theDeftemplate,tempArg->value,&position)) == NULL)
        {
         InvalidDeftemplateSlotMessage(ValueToString(tempArg->value),
                                       ValueToString(theDeftemplate->header.name));
         return(CLIPS_FALSE);
        }

      /*=========================================================*/
      /* Is a multifield value being put in a single field slot? */
      /*=========================================================*/

      if (slotPtr->multislot == CLIPS_FALSE)
        {
         if (tempArg->argList == NULL)
           {         
            SingleFieldSlotCardinalityError(slotPtr->slotName->contents);
            return(CLIPS_FALSE);
           }
         else if (tempArg->argList->nextArg != NULL)
           {
            SingleFieldSlotCardinalityError(slotPtr->slotName->contents);
            return(CLIPS_FALSE);
           }
         else if ((tempArg->argList->type == MF_VARIABLE) ||
                  ((tempArg->argList->type == FCALL) ?
                   (((struct FunctionDefinition *) tempArg->argList->value)->returnValueType == 'm') :
                      CLIPS_FALSE))
           {
            SingleFieldSlotCardinalityError(slotPtr->slotName->contents);
            return(CLIPS_FALSE);
           }
        }

      /*======================================*/
      /* Are the slot restrictions satisfied? */
      /*======================================*/

      if (CheckRHSSlotTypes(tempArg->argList,slotPtr,name) == 0)
        return(CLIPS_FALSE);

      /*=============================================*/
      /* Replace the slot with the integer position. */
      /*=============================================*/

      tempArg->type = INTEGER;
      tempArg->value = (VOID *) AddLong((long) (FindSlotPosition(theDeftemplate,tempArg->value) - 1));

      tempArg = tempArg->nextArg;
     }

   return(CLIPS_TRUE);
  }
  
/*******************************************************/
/* FindSlotPosition: Finds the position of a specified */
/*   slot in a deftemplate structure.                  */
/*******************************************************/
static int FindSlotPosition(theDeftemplate,name)
  struct deftemplate *theDeftemplate;
  SYMBOL_HN *name;
  {
   struct templateSlot *slotPtr;
   int position = 1;

   slotPtr = theDeftemplate->slotList;
   while (slotPtr != NULL)
     {
      if (slotPtr->slotName == name)
        { return(position); }
      slotPtr = slotPtr->next;
      position++;
     }

   return(0);
  }
 
/*****************************************************/
/* FindTemplateForFactAddress:    */
/*****************************************************/
static SYMBOL_HN *FindTemplateForFactAddress(factAddress,theLHS)
  SYMBOL_HN *factAddress;
  struct lhsParseNode *theLHS;
  {
   struct lhsParseNode *thePattern = NULL;
   
   while (theLHS != NULL)
     {
      if (theLHS->value == (VOID *) factAddress) 
        {
         thePattern = theLHS;
         theLHS = NULL;
        }
      else
        { theLHS = theLHS->bottom; }
     }
    
   if (thePattern == NULL) return(NULL);
    
   /*=====================================*/
   /* Verify that just a symbol is stored */
   /* as the first field of the pattern.  */
   /*=====================================*/
   
   thePattern = thePattern->right;   
   if ((thePattern->type != SF_WILDCARD) || (thePattern->bottom == NULL))
     { return(NULL); }
    
   thePattern = thePattern->bottom;
   if ((thePattern->type != SYMBOL) ||
            (thePattern->right != NULL) ||
            (thePattern->bottom != NULL))
    { return(NULL); }
  
   /*==================================================================*/
   /* Determine if there is a deftemplate corresponding to the symbol. */
   /*==================================================================*/
   
   return((SYMBOL_HN *) thePattern->value);
  }
  
/*******************************************/
/* ModifyParse: Parses the modify command. */
/*******************************************/
globle struct expr *ModifyParse(top,logicalName)
  struct expr *top;
  char *logicalName;
  {
   return(ModAndDupParse(top,logicalName,"modify"));
  }

/*************************************************/
/* DuplicateParse: Parses the duplicate command. */
/*************************************************/
globle struct expr *DuplicateParse(top,logicalName)
  struct expr *top;
  char *logicalName;
  {
   return(ModAndDupParse(top,logicalName,"duplicate"));
  }

/*************************************************************/
/* ModAndDupParse: Parses the modify and duplicate commands. */
/*************************************************************/
static struct expr *ModAndDupParse(top,logicalName,name)
  struct expr *top;
  char *logicalName;
  char *name;
  {
   int error = CLIPS_FALSE;
   struct token theToken;
   struct expr *next_one, *tempSlot;
   struct expr *new_test, *multi_test, *last_test;
   int printError;

   /*==================================================================*/
   /* Parse the fact-address or index to the modify/duplicate command. */
   /*==================================================================*/

   SavePPBuffer(" ");
   GetToken(logicalName,&theToken);

   if ((theToken.type == SF_VARIABLE) || (theToken.type == GBL_VARIABLE))
     { next_one = GenConstant(theToken.type,theToken.value); }
   else if (theToken.type == INTEGER)
     {
      if (! TopLevelCommand())
        {
         PrintErrorID("TMPLTFUN",1,CLIPS_TRUE);
         PrintCLIPS(WERROR,"Fact-indexes can only be used by ");
         PrintCLIPS(WERROR,name);
         PrintCLIPS(WERROR," as a top level command.\n");
         ReturnExpression(top);
         return(NULL);
        }

      next_one = GenConstant(INTEGER,theToken.value);
     }
   else
     {
      ExpectedTypeError2(name,1);
      ReturnExpression(top);
      return(NULL);
     }

   next_one->nextArg = NULL;
   next_one->argList = NULL;
   top->argList = next_one;
   next_one = top->argList;

   /*==============================================*/
   /* Parse the remaining modify/duplicate fields. */
   /*==============================================*/

   GetToken(logicalName,&theToken);
   while (theToken.type != RPAREN)
     {
      PPBackup();
      SavePPBuffer(" ");
      SavePPBuffer(theToken.printForm);

      /*=================================================*/
      /* Slot definition begins with a left parenthesis. */
      /*=================================================*/

      if (theToken.type != LPAREN)
        {
         SyntaxErrorMessage("duplicate/modify function");
         ReturnExpression(top);
         return(NULL);
        }

      /*=================================*/
      /* The slot name must be a symbol. */
      /*=================================*/

      GetToken(logicalName,&theToken);
      if (theToken.type != SYMBOL)
        {
         SyntaxErrorMessage("duplicate/modify function");
         ReturnExpression(top);
         return(NULL);
        }
        
      /*=================================*/
      /* Check for duplicate slot names. */
      /*=================================*/
      
      for (tempSlot = top->argList->nextArg; 
           tempSlot != NULL;
           tempSlot = tempSlot->nextArg)
        {
         if (tempSlot->value == theToken.value)
           {
            AlreadyParsedErrorMessage("slot ",ValueToString(theToken.value));
            ReturnExpression(top);
            return(NULL);
           }
        }

      /*=========================================*/
      /* Add the slot name to the list of slots. */
      /*=========================================*/
      
      next_one->nextArg = GenConstant(SYMBOL,theToken.value);
      next_one = next_one->nextArg;

      /*====================================================*/
      /* Get the values to be stored in the specified slot. */
      /*====================================================*/

      multi_test = NULL;
      last_test = NULL;
      while (theToken.type != RPAREN)
        {
         SavePPBuffer(" ");
         new_test = GetAssertArgument(logicalName,&theToken,&error,
                                      RPAREN,CLIPS_FALSE,&printError);

         if (error)
           {
            if (printError) SyntaxErrorMessage("deftemplate pattern");
            ReturnExpression(top);
            return(NULL);
           }

         if (last_test == NULL)
           { multi_test = new_test; }
         else
           { last_test->nextArg = new_test; }
         last_test = new_test;
        }

      /*==================================================*/
      /* Slot definition begins with a right parenthesis. */
      /*==================================================*/

      if (theToken.type != RPAREN)
        {
         SyntaxErrorMessage("duplicate/modify function");
         ReturnExpression(top);
         ReturnExpression(multi_test);
         return(NULL);
        }
      else
        {
         PPBackup();
         PPBackup();
         SavePPBuffer(")");
        }

      next_one->argList = multi_test;

      GetToken(logicalName,&theToken);
     }

   return(top);
  }

#endif

/*##############################################*/
/*#                                            #*/
/*# DEFTEMPLATE FUNCTIONS FOR DYNAMIC CHECKING #*/
/*#                                            #*/
/*##############################################*/

/**********************************************************************/
/* MultiIntoSingleFieldSlotError: Determines if a multifield value is */
/*   being placed into a single field slot of a deftemplate fact.     */
/**********************************************************************/
globle VOID MultiIntoSingleFieldSlotError(theSlot,theDeftemplate)
  struct templateSlot *theSlot;
  struct deftemplate *theDeftemplate;
  {
   PrintErrorID("TMPLTFUN",2,CLIPS_TRUE);
   PrintCLIPS(WERROR,"Attempted to assert a multifield value \n");
   PrintCLIPS(WERROR,"into the single field slot ");
   if (theSlot != NULL) PrintCLIPS(WERROR,theSlot->slotName->contents);
   else PrintCLIPS(WERROR,"<<unknown>>");
   PrintCLIPS(WERROR," of deftemplate ");
   if (theDeftemplate != NULL) PrintCLIPS(WERROR,theDeftemplate->header.name->contents);
   else PrintCLIPS(WERROR,"<<unknown>>");
   PrintCLIPS(WERROR,".\n");
   
   SetEvaluationError(CLIPS_TRUE);
  }

/**************************************************************/
/* CheckTemplateFact: Checks a fact to see if it violates any */
/*   deftemplate type, allowed-..., or range specifications.  */
/**************************************************************/
globle VOID CheckTemplateFact(theFact)
  struct fact *theFact;
  {
   struct field *sublist;
   int length, i;
   struct deftemplate *theDeftemplate;
   struct templateSlot *slotPtr;
   DATA_OBJECT theData;
   char thePlace[20];
   int rv;

   if (! GetDynamicConstraintChecking()) return;

   sublist = theFact->theProposition.theFields;
   length = theFact->theProposition.multifieldLength;

   /*========================================================*/
   /* If the deftemplate corresponding to the first field of */
   /* of the fact cannot be found, then the fact cannot be   */
   /* checked against the deftemplate format.                */
   /*========================================================*/

   theDeftemplate = theFact->whichDeftemplate;
   if (theDeftemplate == NULL) return;
   if (theDeftemplate->implied) return;

   /*===================================================*/
   /* Check each of the field slots of the deftemplate. */
   /*===================================================*/

   slotPtr = theDeftemplate->slotList;

   i = 0;
   while (slotPtr != NULL)
     {
      if (slotPtr->multislot == CLIPS_FALSE)
        {
         theData.type = sublist[i].type;
         theData.value = sublist[i].value;
         i++;
        }
      else
        {
         theData.type = MULTIFIELD;
         theData.value = (VOID *) &theFact->theProposition;
         theData.begin = i;
         theData.end = length-1;
        }
        
      rv = ConstraintCheckDataObject(&theData,slotPtr->constraints);
      if (rv != NO_VIOLATION)
        {
         sprintf(thePlace,"fact f-%-5ld ",theFact->factIndex);
      
         PrintErrorID("CSTRNCHK",1,CLIPS_TRUE);
         PrintCLIPS(WERROR,"Slot value ");
         PrintDataObject(WERROR,&theData);
         PrintCLIPS(WERROR," ");
         ConstraintViolationErrorMessage(NULL,thePlace,CLIPS_FALSE,0,slotPtr->slotName,
                                         0,rv,slotPtr->constraints,CLIPS_TRUE);
         SetHaltExecution(CLIPS_TRUE);
         return;
        }
        
      slotPtr = slotPtr->next;
     }

   return;
  }

#endif

