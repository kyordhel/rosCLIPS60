   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 FACT MATCH MODULE                   */
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

#define _FACTMCH_SOURCE_

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
#include "factrete.h"
#include "tmpltdef.h"

#include "factmch.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static BOOLEAN                  EvaluatePatternExpression(struct factPatternNode *,struct expr *,int);
   static VOID                     TraceErrorToPattern(struct factPatternNode *,int);
   static VOID                     ProcessFactAlphaMatch(struct fact *,struct multifieldMarker *,struct factPatternNode *);
   static struct factPatternNode  *GetNextFactPatternNode(int,struct factPatternNode *);
   static int                      SkipFactPatternNode(struct factPatternNode *);
   static VOID                     ProcessMultifieldNode(struct factPatternNode *,
                                                         struct multifieldMarker *,
                                                         struct multifieldMarker *,int);
#else
   static BOOLEAN                  EvaluatePatternExpression();
   static VOID                     TraceErrorToPattern();
   static VOID                     ProcessFactAlphaMatch();
   static struct factPatternNode  *GetNextFactPatternNode();
   static int                      SkipFactPatternNode();
   static VOID                     ProcessMultifieldNode();
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct fact             *CurrentPatternFact;
   globle struct multifield       *CurrentPatternSegment;
   globle struct multifieldMarker *CurrentPatternMarks;
  
/*****************************************************************/
/* FactPatternMatch:  Compares the fields of a fact to the nodes in the */
/*   pattern network.  Arguments are:                        */
/*****************************************************************/
globle VOID FactPatternMatch(theFact,patternPtr,offset,markers,endMark)
  struct fact *theFact;
  struct factPatternNode *patternPtr;
  int offset;
  struct multifieldMarker *markers, *endMark;
  {
   int theFactField;
   int thePatternField;
   int offsetSlot;

   if (patternPtr == NULL) return;

   /* The offset only applies to the slot being */
   /* matching when this routine is entered. Once */
   /* a new slot starts being matched an offset of */
   /* 0 should be used.                         */
   offsetSlot = patternPtr->whichSlot;
   CurrentPatternFact = theFact;
   CurrentPatternMarks = markers;

   while (patternPtr != NULL)
     {
      thePatternField = patternPtr->whichField;
      if (offsetSlot == patternPtr->whichSlot) theFactField = thePatternField + offset;
      else theFactField = thePatternField;
      
      if (theFactField > 0) 
        { 
         CurrentPatternSegment = (struct multifield *) 
              theFact->theProposition.theFields[patternPtr->whichSlot].value;
        }
      else CurrentPatternSegment = NULL;
            
      /*====================================================*/
      /* Determine if we want to skip this node altogether. */
      /*====================================================*/
      
      if (SkipFactPatternNode(patternPtr))
        {
         patternPtr = GetNextFactPatternNode(CLIPS_TRUE,patternPtr);
        }
        
      /*=============================================================*/
      /* Determine if the test is satisfied for a single field node. */
      /*=============================================================*/
      
      else if (patternPtr->header.singlefieldNode)
        { 
         if (EvaluatePatternExpression(patternPtr,patternPtr->networkTest,theFactField))
           {
            if (patternPtr->header.stopNode) ProcessFactAlphaMatch(theFact,markers,patternPtr);
            patternPtr = GetNextFactPatternNode(CLIPS_FALSE,patternPtr);
           }
         else
           { patternPtr = GetNextFactPatternNode(CLIPS_TRUE,patternPtr); }
        }
        
      /*===========================================================*/
      /* Determine if the test is satisfied for a multifield node. */
      /*===========================================================*/
      
      else if (patternPtr->header.multifieldNode)
        { 
         if (offsetSlot == patternPtr->whichSlot)
           { ProcessMultifieldNode(patternPtr,markers,endMark,offset); }
         else
           { ProcessMultifieldNode(patternPtr,markers,endMark,0); }
         patternPtr = GetNextFactPatternNode(CLIPS_TRUE,patternPtr);
        }
     }
  }
  
/***************************************************************/
/* ProcessMultifieldNode: */
/***************************************************************/
static VOID ProcessMultifieldNode(thePattern,markers,endMark,offset)
  struct factPatternNode *thePattern;
  struct multifieldMarker *markers, *endMark;
  int offset;
  {
   struct multifieldMarker *newMark, *oldMark;
   int repeatCount;
     
   /*===============================================*/
   /* Save the value of the markers already stored. */
   /*===============================================*/

   oldMark = markers;

   /*===========================================*/
   /* Create a new multifield marker and append */
   /* it to the end of the current list.        */
   /*===========================================*/
   
   newMark = get_struct(multifieldMarker);
   newMark->whichField = thePattern->whichField - 1;
   newMark->where.whichSlotNumber = thePattern->whichSlot;
   newMark->startPosition = (thePattern->whichField - 1) + offset;
   newMark->next = NULL;
   
   if (endMark == NULL)
     {
      markers = newMark;
      CurrentPatternMarks = markers;
     }
   else
     { endMark->next = newMark; }   
    
   /*=============================================*/
   /* Handle the end slot node as a special case. */
   /*=============================================*/
   
   if (thePattern->header.endSlot)
     {
      newMark->endPosition = CurrentPatternSegment->multifieldLength - 
                             (thePattern->leaveFields + 1);
                             
      /* Make sure the endPosition is never less than one */
      /* below the startPosition (a multifield bound to   */
      /* no fields.                                       */
      if (newMark->endPosition < newMark->startPosition)
        { newMark->endPosition = newMark->startPosition - 1; }
      
      if ((thePattern->networkTest == NULL) ? 
          CLIPS_TRUE : 
          (EvaluatePatternExpression(thePattern,thePattern->networkTest,
                                     (int) thePattern->whichField + offset)))
        {
         if (thePattern->header.stopNode) 
           { ProcessFactAlphaMatch(CurrentPatternFact,CurrentPatternMarks,thePattern); }
         
         FactPatternMatch(CurrentPatternFact,
                          thePattern->nextLevel,0,CurrentPatternMarks,newMark);
        }
       
      rtn_struct(multifieldMarker,newMark);
      if (endMark != NULL) endMark->next = NULL;
      markers = oldMark;
      CurrentPatternMarks = oldMark;
      
      return;
     }
      
   /*==============================================*/
   /* Perform matching for nodes beneath this one. */
   /*==============================================*/
   
   for (repeatCount = CurrentPatternSegment->multifieldLength - 
                      (newMark->startPosition + thePattern->leaveFields);
        repeatCount >= 0;
        repeatCount--)
     {      
      newMark->endPosition = newMark->startPosition + (repeatCount - 1);
      
      if ((thePattern->networkTest == NULL) ? 
          CLIPS_TRUE : 
          (EvaluatePatternExpression(thePattern,thePattern->networkTest,
                                     (int) thePattern->whichField + offset)))
        {
         FactPatternMatch(CurrentPatternFact,
                          thePattern->nextLevel,offset + repeatCount - 1,
                          CurrentPatternMarks,newMark);
        }
     }

    /*======================================================*/
    /* Get rid of the marker created for a multifield node. */
    /*======================================================*/

    rtn_struct(multifieldMarker,newMark);
    if (endMark != NULL) endMark->next = NULL;
    markers = oldMark;
    CurrentPatternMarks = oldMark;
   }
        
/***************************************************************/
/* GetNextFactPatternNode                                      */
/***************************************************************/
static struct factPatternNode *GetNextFactPatternNode(finishedMatching,thePattern)
  int finishedMatching;
  struct factPatternNode *thePattern;
  {
   EvaluationError = CLIPS_FALSE;
   
   if (finishedMatching == CLIPS_FALSE) 
     { if (thePattern->nextLevel != NULL) return(thePattern->nextLevel); }
        
   while (thePattern->rightNode == NULL)
     {
      thePattern = thePattern->lastLevel;
      if (thePattern == NULL) return(NULL);
      if (thePattern->header.multifieldNode) return(NULL);
     }
     
   return(thePattern->rightNode);
  }
           
/***************************************************************/
/* ProcessFactAlphaMatch                                              */
/***************************************************************/
static VOID ProcessFactAlphaMatch(theFact,theMarks,thePattern)
  struct fact *theFact;
  struct multifieldMarker *theMarks;
  struct factPatternNode *thePattern;
  {
   struct partialMatch *theMatch;
   struct patternMatch *listOfMatches;
   struct joinNode *listOfJoins;
   
  /*===========================================*/
  /* Create the partial match for the pattern. */
  /*===========================================*/
         
  theMatch = CreateAlphaMatch(theFact,theMarks,(struct patternNodeHeader *) &thePattern->header);
         
  /*=======================================================*/
  /* Add the pattern to the list of matches for this fact. */
  /*=======================================================*/

  listOfMatches = (struct patternMatch *) theFact->list;
  theFact->list = (VOID *) get_struct(patternMatch);
  ((struct patternMatch *) theFact->list)->next = listOfMatches;
  ((struct patternMatch *) theFact->list)->matchingPattern = (struct patternNodeHeader *) thePattern;
  ((struct patternMatch *) theFact->list)->theMatch = theMatch;

  /*================================================================*/
  /* Send the partial match to the joins connected to this pattern. */
  /*================================================================*/

  listOfJoins = thePattern->header.entryJoin;
   while (listOfJoins != NULL)
     {
      Drive(theMatch,listOfJoins,RHS);
      listOfJoins = listOfJoins->rightMatchNode;
     }
  }
  
/***************************************************************/
/* EvaluatePatternExpression: Performs a faster evaluation for */
/*   pattern expressions than if EvaluateExpression were used  */
/*   directly. This function evaluates calls to function       */
/*   constant, function notconstant, function and, and         */
/*   function or. No other function calls are made in pattern  */
/*   expressions.                                              */
/***************************************************************/
static int EvaluatePatternExpression(patternPtr,theTest,thePosition)
  struct factPatternNode *patternPtr;
  struct expr *theTest;
  int thePosition;
  {
   DATA_OBJECT vresult;
   struct expr *oldArgument;
   int rv;
    
   if (theTest == NULL) return(CLIPS_TRUE);
   
   switch(theTest->type)
     {
      case SCALL_PN_CONSTANT2:
        oldArgument = CurrentExpression;
        CurrentExpression = theTest;
        rv = FactConstantPNFunction2(theTest->value,&vresult);
        CurrentExpression = oldArgument;
        return(rv);
        
      case SCALL_PN_CONSTANT4:
        oldArgument = CurrentExpression;
        CurrentExpression = theTest;
        rv = FactConstantPNFunction4(theTest->value,&vresult);
        CurrentExpression = oldArgument;
        return(rv);
        
      case SCALL_LENGTH_TEST:
        oldArgument = CurrentExpression;
        CurrentExpression = theTest;
        rv = FactSlotLengthTestFunction(theTest->value,&vresult);
        CurrentExpression = oldArgument;
        return(rv);
     }
      
   /*=========================================================*/
   /* Evaluate or expressions expressed in the format:        */
   /*   (or <expression 1> <expression 2> ... <expression n>) */
   /* Returns true (1.0) if any of the expression are true,   */
   /* otherwise returns false (0.0).                          */
   /*=========================================================*/

   if (theTest->value == PTR_OR)
     {
      theTest = theTest->argList;
      while (theTest != NULL)
        {
         if (EvaluatePatternExpression(patternPtr,theTest,thePosition) == CLIPS_TRUE)
           {
            if (EvaluationError) return(CLIPS_FALSE);
            return(CLIPS_TRUE);
           }
         if (EvaluationError) return(CLIPS_FALSE);
         theTest = theTest->nextArg;
        }
        
      return(CLIPS_FALSE);
     }

   /*==========================================================*/
   /* Evaluate and expressions expressed in the format:        */
   /*   (and <expression 1> <expression 2> ... <expression n>) */
   /* Returns false (0.0) if any of the expression are false,  */
   /* otherwise returns true (1.0).                            */
   /*==========================================================*/

   else if (theTest->value == PTR_AND)
     {
      theTest = theTest->argList;
      while (theTest != NULL)
        {
         if (EvaluatePatternExpression(patternPtr,theTest,thePosition) == CLIPS_FALSE)
           { return(CLIPS_FALSE); }
         if (EvaluationError) return(CLIPS_FALSE);
         theTest = theTest->nextArg;
        }
        
      return(CLIPS_TRUE);
     }

   /*==========================================================*/
   /* Evaluate all other expressions using EvaluateExpression. */
   /*==========================================================*/

   if (EvaluateExpression(theTest,&vresult))
     {
      PatternNetErrorMessage(patternPtr);
      return(CLIPS_FALSE);
     }

   if ((vresult.value == CLIPSFalseSymbol) && (vresult.type == SYMBOL))
     { return(CLIPS_FALSE); }
   
   return(CLIPS_TRUE);
  }
    
/*****************************************/
/* PatternNetErrorMessage:                     */
/*****************************************/
globle VOID PatternNetErrorMessage(patternPtr)
  struct factPatternNode *patternPtr;
  {
   char buffer[60];
   struct templateSlot *theSlots;
   int i;

   PrintErrorID("FACTMCH",1,CLIPS_TRUE);
   PrintCLIPS(WERROR,"This error occurred in the fact pattern network\n");
   PrintCLIPS(WERROR,"   Currently active fact: ");
   PrintFact(WERROR,CurrentPatternFact);
   PrintCLIPS(WERROR,"\n");
   
   if (CurrentPatternFact->whichDeftemplate->implied)
     { sprintf(buffer,"   Problem resides in field #%d\n",patternPtr->whichField); }
   else
     { 
      theSlots = CurrentPatternFact->whichDeftemplate->slotList;
      for (i = 0; i < patternPtr->whichSlot; i++) theSlots = theSlots->next;
      sprintf(buffer,"   Problem resides in slot %s\n",ValueToString(theSlots->slotName)); 
     }

   PrintCLIPS(WERROR,buffer);
   
   TraceErrorToPattern(patternPtr,CLIPS_FALSE);
   PrintCLIPS(WERROR,"\n");
  }

/***************************************************************************/
/* TraceErrorToPattern: Prints an error message when a error occurs as the */
/*   result of evaluating an expression in the pattern network. Prints the */
/*   fact currently being matched against and the field in the pattern     */
/*   which caused the problem, then calls the function TraceErrorToPattern */
/*   to further isolate the error by printing the rule(s) which contain    */
/*   the pattern.                                                          */
/***************************************************************************/
static VOID TraceErrorToPattern(patternPtr,traceRight)
  struct factPatternNode *patternPtr;
  int traceRight;
  {
   struct joinNode *joinPtr;
   char buffer[60];

   while (patternPtr != NULL)
     {
      if (patternPtr->header.stopNode)
        {
         joinPtr = patternPtr->header.entryJoin;
         while (joinPtr != NULL)
           {
            sprintf(buffer,"      Of pattern #%d in rule(s):\n",joinPtr->depth);
            PrintCLIPS(WERROR,buffer);
            TraceErrorToRule(joinPtr,"         ");
            joinPtr = joinPtr->rightMatchNode;
           }
        }
      else
        { TraceErrorToPattern(patternPtr->nextLevel,CLIPS_TRUE); }
        
      if (traceRight) patternPtr = patternPtr->rightNode;
      else patternPtr = NULL;
     }

  }
  
/**************************************************/
/* SkipFactPatternNode:                           */
/**************************************************/
static int SkipFactPatternNode(thePattern)
  struct factPatternNode *thePattern;
  {
   /*==========================================================*/
   /* If an incremental reset is being performed and the node  */
   /* is not part of the network to be reset, then the pattern */
   /* matching attempt for this pattern has failed.            */
   /*==========================================================*/

#if INCREMENTAL_RESET && (! RUN_TIME) && (! BLOAD_ONLY)
   if (IncrementalResetInProgress && (thePattern->header.initialize == CLIPS_FALSE))
     { return(CLIPS_TRUE); } 
#endif
    
   return(CLIPS_FALSE);   
  }

#if INCREMENTAL_RESET
/*************************************************************/
/* MarkFactPtnForIncrementalReset:       */
/*************************************************************/
globle VOID MarkFactPtnForIncrementalReset(thePattern,value)
  struct patternNodeHeader *thePattern;
  int value;
  {
   struct factPatternNode *patternPtr;
   
   if (thePattern == NULL) return;
   
   patternPtr = (struct factPatternNode *) thePattern;
   
   if (patternPtr->header.initialize == CLIPS_FALSE) return;
   
   while (patternPtr != NULL)
       {
        patternPtr->header.initialize = value;
        patternPtr = patternPtr->lastLevel;
       }
  }
  
/*************************************************************/
/* FactsIncrementalReset:       */
/*************************************************************/
globle VOID FactsIncrementalReset()
  {
   struct fact *factPtr;
   
   factPtr = (struct fact *) GetNextFact(NULL);

   while (factPtr != NULL)
     {
      FactPatternMatch(factPtr,factPtr->whichDeftemplate->patternNetwork,0,NULL,NULL);
      factPtr = (struct fact *) GetNextFact(factPtr);
     }
  }
#endif

#endif

