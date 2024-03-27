   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00 05/12/93             */
   /*                                                     */
   /*                DEFTEMPLATE LHS MODULE               */
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

#define _TMPLTLHS_SOURCE_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT && DEFRULE_CONSTRUCT && (! RUN_TIME) && (! BLOAD_ONLY)

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "constant.h"
#include "clipsmem.h"
#include "symbol.h"
#include "scanner.h"
#include "exprnpsr.h"
#include "router.h"
#include "constrnt.h"
#include "constrct.h"
#include "reorder.h"
#include "pattern.h"
#include "factrhs.h"
#include "modulutl.h"
#include "tmpltdef.h"

#include "tmpltlhs.h"

#if ANSI_COMPILER
   static struct lhsParseNode    *GetLHSSlots(char *,struct token *,struct deftemplate *,int *);
   static struct lhsParseNode    *GetSingleLHSSlot(char *,struct token *,
                                                   struct templateSlot *,int *,int);
   static BOOLEAN                 MultiplyDefinedLHSSlots(struct lhsParseNode *,SYMBOL_HN *);
#else
   static struct lhsParseNode    *GetLHSSlots();
   static struct lhsParseNode    *GetSingleLHSSlot();
   static BOOLEAN                 MultiplyDefinedLHSSlots();
#endif

/**************************************************/
/* DeftemplateLHSParse: Parses a LHS pattern that */
/*   uses the deftemplate format.                 */
/**************************************************/
globle struct lhsParseNode *DeftemplateLHSParse(readSource,theDeftemplate)
  char *readSource;
  struct deftemplate *theDeftemplate;
  {
   struct lhsParseNode *head = NULL, *firstSlot;
   struct token theToken;
   int error;

   /*===============================================================*/
   /* Make sure the deftemplate name is not connected to subfields. */
   /*===============================================================*/

   GetToken(readSource,&theToken);
   if ((theToken.type == OR_CONSTRAINT) || (theToken.type == AND_CONSTRAINT))
     {
      SyntaxErrorMessage("deftemplate patterns");
      return(NULL);
     }

   /*===============================================*/
   /* Create pattern node for the deftemplate name. */
   /*===============================================*/

   head = GetLHSParseNode();
   head->type = SF_WILDCARD;
   head->negated = CLIPS_FALSE;
   head->index = 0;
   head->slotNumber = 1;
   head->bottom = GetLHSParseNode();
   head->bottom->type = SYMBOL;
   head->bottom->negated = CLIPS_FALSE;
   head->bottom->value = (VOID *) theDeftemplate->header.name;

   /*======================================*/
   /* Get other fields in the deftemplate. */
   /*======================================*/

   error = CLIPS_FALSE;
   firstSlot = GetLHSSlots(readSource,&theToken,theDeftemplate,&error);
   if (error)
     {
      ReturnLHSParseNodes(firstSlot);
      ReturnLHSParseNodes(head);
      return(NULL);
     }

   /*=========================*/
   /* Return the LHS pattern. */
   /*=========================*/

   head->right = firstSlot;
   return(head);
  }

/******************************************/
/* GetLHSSlots: Retrieves all of the slot */
/*   values used in a LHS pattern.        */
/******************************************/
static struct lhsParseNode *GetLHSSlots(readSource,tempToken,theDeftemplate,error)
  char *readSource;
  struct token *tempToken;
  struct deftemplate *theDeftemplate;
  int *error;
  {
   struct lhsParseNode *firstSlot = NULL, *nextSlot, *lastSlot = NULL;
   struct templateSlot *slotPtr;
   int position;

   /*=======================================================*/
   /* Continue parsing slot definitions until the pattern's */
   /* closing right parenthesis is encountered.             */
   /*=======================================================*/

   while (tempToken->type != RPAREN)
     {
      PPBackup();
      SavePPBuffer(" ");
      SavePPBuffer(tempToken->printForm);

      /*=================================================*/
      /* Slot definitions begin with a left parenthesis. */
      /*=================================================*/

      if (tempToken->type != LPAREN)
        {
         *error = CLIPS_TRUE;
         SyntaxErrorMessage("deftemplate patterns");
         ReturnLHSParseNodes(firstSlot);
         return(NULL);
        }

      /*====================*/
      /* Get the slot name. */
      /*====================*/

      GetToken(readSource,tempToken);
      if (tempToken->type != SYMBOL)
        {
         *error = CLIPS_TRUE;
         SyntaxErrorMessage("deftemplate patterns");
         ReturnLHSParseNodes(firstSlot);
         return(NULL);
        }

      /*==========================================================*/
      /* Determine if the slot name is valid for the deftemplate. */
      /*==========================================================*/

      if ((slotPtr = FindSlot(theDeftemplate,tempToken->value,&position)) == NULL)
        {
         *error = CLIPS_TRUE;
         InvalidDeftemplateSlotMessage(ValueToString(tempToken->value),
                                       ValueToString(theDeftemplate->header.name));
         ReturnLHSParseNodes(firstSlot);
         return(NULL);
        }

      /*============================================*/
      /* Determine if the slot is multiply defined. */
      /*============================================*/
      
      if (MultiplyDefinedLHSSlots(firstSlot,tempToken->value) == CLIPS_TRUE)
        {
         *error = CLIPS_TRUE;
         ReturnLHSParseNodes(firstSlot);
         return(NULL);
        }
        
      /*==============================================================*/
      /* Get the pattern matching values used in the slot definition. */
      /*==============================================================*/

      nextSlot = GetSingleLHSSlot(readSource,tempToken,slotPtr,error,position+1);
      if (*error)
        {
         ReturnLHSParseNodes(firstSlot);
         ReturnLHSParseNodes(nextSlot);
         return(NULL);
        }

      /*=====================================*/
      /* Add the slot definition to the list */
      /* of slot definitions already parsed. */
      /*=====================================*/

      if (lastSlot == NULL)
        { firstSlot = nextSlot; }
      else
        { lastSlot->right = nextSlot; }
        
      while (nextSlot->right != NULL) nextSlot = nextSlot->right;
      lastSlot = nextSlot;

      /*==============================*/
      /* Begin parsing the next slot. */
      /*==============================*/

      GetToken(readSource,tempToken);
     }

   /*===========================================================*/
   /* Return all the slot definitions found in the lhs pattern. */
   /*===========================================================*/

   return(firstSlot);
  }

/*****************************************************/
/* GetSingleLHSSlot: Get the pattern matching values */
/*   to be associated with a slot name.              */
/*****************************************************/
static struct lhsParseNode *GetSingleLHSSlot(readSource,tempToken,slotPtr,error,position)
  char *readSource;
  struct token *tempToken;
  struct templateSlot *slotPtr;
  int *error;
  int position;
  {
   struct lhsParseNode *nextSlot;
   SYMBOL_HN *slotName;

   slotName = (SYMBOL_HN *) tempToken->value;

   SavePPBuffer(" ");
   GetToken(readSource,tempToken);

   /*====================================*/
   /* Get value for a single field slot. */
   /*====================================*/

   if (slotPtr->multislot == CLIPS_FALSE)
     {
      /*=======================*/
      /* Get the single value. */
      /*=======================*/

      nextSlot = RestrictionParse(readSource,tempToken,CLIPS_FALSE,
                                  slotPtr->slotName,position - 1,slotPtr->constraints,0);
      if (nextSlot == NULL)
        {
         *error = CLIPS_TRUE;
         return(NULL);
        }
                     
      /*======================================*/
      /* Multi field wildcards and variables  */
      /* not allowed in a single field slot.  */
      /*======================================*/

      if ((nextSlot->type == MF_VARIABLE) ||
          (nextSlot->type == MULTIFIELD))
        {
         SingleFieldSlotCardinalityError(slotPtr->slotName->contents);
         *error = CLIPS_TRUE;
         ReturnLHSParseNodes(nextSlot);
         return(NULL);
        }
     }

   /*====================================*/
   /* Get values for a multi field slot. */
   /*====================================*/

   else
     {
      nextSlot = RestrictionParse(readSource,tempToken,CLIPS_TRUE,slotName,position - 1,
                                  slotPtr->constraints,1);
      if (nextSlot == NULL)
        {
         *error = CLIPS_TRUE;
         return(NULL);
        }
     }

   /*========================================================*/
   /* The slot definition must end with a right parenthesis. */
   /*========================================================*/

   if (tempToken->type != RPAREN)
     {
      PPBackup();                        
      SavePPBuffer(" ");                  
      SavePPBuffer(tempToken->printForm); 
      SyntaxErrorMessage("deftemplate patterns");
      *error = CLIPS_TRUE;
      ReturnLHSParseNodes(nextSlot);
      return(NULL);
     }
   
   /*===============================================*/
   /* Fix the pretty print output if the multifield */
   /* slot contained no restrictions.               */
   /*===============================================*/
   
   if ((nextSlot->bottom == NULL) && slotPtr->multislot)
     {
      PPBackup();  
      PPBackup();                        
      SavePPBuffer(")");  
     } 

   /*=================================*/
   /* Add the slot values to the slot */
   /* structure and return it.        */
   /*=================================*/

   return(nextSlot);
  }

/******************************************************/
/* MultiplyDefinedLHSSlots: Determines if a slot name */
/*   was used more than once in a LHS pattern.        */
/******************************************************/
static BOOLEAN MultiplyDefinedLHSSlots(theSlots,slotName)
  struct lhsParseNode *theSlots;
  SYMBOL_HN *slotName;
  {
   while (theSlots != NULL)
     {
      if (theSlots->slot == slotName)
        {         
         AlreadyParsedErrorMessage("slot ",ValueToString(slotName));
         return(CLIPS_TRUE);
        }
        
      theSlots = theSlots->right;
     }

   return(CLIPS_FALSE);
  }

#endif


