   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */ 
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 RULE PATTERN MODULE                 */
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

#define _PATTERN_SOURCE_

#include "setup.h"

#include <stdio.h>
#define _CLIPS_STDIO_

#if DEFRULE_CONSTRUCT

#include "constant.h"
#include "clipsmem.h"
#include "match.h"
#include "reteutil.h"
#include "constrnt.h"
#include "exprnpsr.h"
#include "router.h"
#include "cstrnchk.h"
#include "cstrnutl.h"
#include "rulecmp.h"

#include "pattern.h"

#define MAX_POSITIONS 8

/**************/
/* STRUCTURES */
/**************/

struct reservedSymbol
  {
   char *theSymbol;
   char *reservedBy;
   struct reservedSymbol *next;
  };

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if (! RUN_TIME) && (! BLOAD_ONLY)
#if ANSI_COMPILER
   static struct lhsParseNode    *ConjuctiveRestrictionParse(char *,struct token *,int *);
   static struct lhsParseNode    *LiteralRestrictionParse(char *,struct token *,int *);
   static int                     CheckForVariableMixing(struct lhsParseNode *);
   static VOID                    TallyFieldTypes(struct lhsParseNode *);
#else
   static struct lhsParseNode    *ConjuctiveRestrictionParse();
   static struct lhsParseNode    *LiteralRestrictionParse();
   static int                     CheckForVariableMixing();
   static VOID                    TallyFieldTypes();
#endif
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct patternParser *ListOfPatternParsers = NULL;

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static struct patternParser  *PatternParserArray[MAX_POSITIONS];
   static int                    nextPosition = 0;
   static struct reservedSymbol *ListOfReservedPatternSymbols = NULL;
  
/******************************************************************/
/* AddReservedPatternSymbol: Adds a symbol to the list of symbols */
/*  that are restricted for use in patterns. For example, the     */
/*  deftemplate construct cannot use the symbol "object", since   */
/*  this needs to be reserved for object patterns. Some symbols,  */
/*  such as "exists" are completely reserved and can not be used  */
/*  to start any type of pattern CE.                              */
/******************************************************************/
VOID AddReservedPatternSymbol(theSymbol,reservedBy)
  char *theSymbol;
  char *reservedBy;
  {
   struct reservedSymbol *newSymbol;
   
   newSymbol = get_struct(reservedSymbol);
   newSymbol->theSymbol = theSymbol;
   newSymbol->reservedBy = reservedBy;
   newSymbol->next = ListOfReservedPatternSymbols;
   ListOfReservedPatternSymbols = newSymbol;
  }
  
/******************************************************************/
/* ReservedPatternSymbol:                            */
/******************************************************************/
BOOLEAN ReservedPatternSymbol(theSymbol,checkedBy)
  char *theSymbol;
  char *checkedBy;
  {
   struct reservedSymbol *currentSymbol;
   
   currentSymbol = ListOfReservedPatternSymbols;
   while (currentSymbol != NULL)
     {
      if (strcmp(theSymbol,currentSymbol->theSymbol) == 0)
        {
         if ((currentSymbol->reservedBy == NULL) || (checkedBy ==  NULL))
           { return(CLIPS_TRUE); }
         if (strcmp(checkedBy,currentSymbol->reservedBy) == 0) return(CLIPS_FALSE);
         return(CLIPS_TRUE);
        }
        
      currentSymbol = currentSymbol->next;
     }
   
   return(CLIPS_FALSE);
  }

/******************************************************************/
/* ReservedPatternSymbolErrorMsg:                            */
/******************************************************************/
VOID ReservedPatternSymbolErrorMsg(theSymbol,usedFor)
  char *theSymbol;
  char *usedFor;
  {
   PrintErrorID("PATTERN",1,CLIPS_TRUE);
   PrintCLIPS(WERROR,"The symbol ");
   PrintCLIPS(WERROR,theSymbol);
   PrintCLIPS(WERROR," has special meaning\n");
   PrintCLIPS(WERROR,"and may not be used as ");
   PrintCLIPS(WERROR,usedFor);
   PrintCLIPS(WERROR,".\n");
  }
  
/***************************************************************************/
/* GetNextEntity:  */
/***************************************************************************/
globle VOID GetNextPatternEntity(theParser,theEntity)
  struct patternParser **theParser;
  struct patternEntity **theEntity;
  {
   if (*theParser == NULL) 
     { *theParser = ListOfPatternParsers; }
   else if (theEntity != NULL)
     { 
      if ((*theParser)->entityType->base.getNextFunction == NULL)
        { theEntity = NULL; }
      else
        { *theEntity = (struct patternEntity *) (*(*theParser)->entityType->base.getNextFunction)(*theEntity); }
      if ((*theEntity) == NULL) *theParser = (*theParser)->next;
     } 
   else 
     { *theParser = (*theParser)->next; }
   
   while ((*theEntity == NULL) && (*theParser != NULL))
     {
      if ((*theParser)->entityType->base.getNextFunction == NULL)
        { theEntity = NULL; }
      else
        { *theEntity = (struct patternEntity *) (*(*theParser)->entityType->base.getNextFunction)(*theEntity); }
      if (*theEntity == NULL) *theParser = (*theParser)->next;
      else return;
     }
     
   return;
  }
  
/***********************************************************/
/* DetachPattern:           */
/***********************************************************/
VOID DetachPattern(rhsType,theHeader)
  int rhsType;
  struct patternNodeHeader *theHeader;
  {
   struct patternParser *tempParser;

   tempParser = PatternParserArray[rhsType];
   
   if (tempParser != NULL)
     {
      FlushAlphaBetaMemory(theHeader->alphaMemory);
      (*tempParser->removePatternFunction)(theHeader);
     }
  }
   
/***********************/
/* AddPatternParser:   */
/***********************/
globle BOOLEAN AddPatternParser(name,priority,entityType,
                      recognizeFunction,parseFunction,
                      postAnalysisFunction,addPatternFunction,removePatternFunction,
                      genJNConstantFunction,replaceGetJNValueFunction,
                      genGetJNValueFunction,genCompareJNValuesFunction,
                      genPNConstantFunction,replaceGetPNValueFunction,
                      genGetPNValueFunction,genComparePNValuesFunction,
                      returnUserDataFunction,copyUserDataFunction,
                      markIRPatternFunction,incrementalResetFunction,
                      initialPatternFunction,
                      codeReferenceFunction)
  char *name;
  int priority;
  struct patternEntityRecord *entityType;
#if ANSI_COMPILER
  int (*recognizeFunction)(SYMBOL_HN *);
  struct lhsParseNode *(*parseFunction)(char *,struct token *);
  int (*postAnalysisFunction)(struct lhsParseNode *);
  struct patternNodeHeader *(*addPatternFunction)(struct lhsParseNode *);
  VOID (*removePatternFunction)(struct patternNodeHeader *);
  struct expr *(*genJNConstantFunction)(struct lhsParseNode *);
  VOID (*replaceGetJNValueFunction)(struct expr *,struct lhsParseNode *); /* 10 */
  struct expr *(*genGetJNValueFunction)(struct lhsParseNode *);
  struct expr *(*genCompareJNValuesFunction)(struct lhsParseNode *,struct lhsParseNode *);
  struct expr *(*genPNConstantFunction)(struct lhsParseNode *);
  VOID (*replaceGetPNValueFunction)(struct expr *,struct lhsParseNode *);
  struct expr *(*genGetPNValueFunction)(struct lhsParseNode *);
  struct expr *(*genComparePNValuesFunction)(struct lhsParseNode *,struct lhsParseNode *);
  VOID (*returnUserDataFunction)(VOID *);
  VOID *(*copyUserDataFunction)(VOID *);
  VOID (*markIRPatternFunction)(struct patternNodeHeader *,int);
  VOID (*incrementalResetFunction)(VOID);
  struct lhsParseNode *(*initialPatternFunction)(VOID);
  VOID (*codeReferenceFunction)(VOID *,FILE *,int,int);
#else
  int (*recognizeFunction)();
  struct lhsParseNode *(*parseFunction)();
  int (*postAnalysisFunction)();
  struct patternNodeHeader *(*addPatternFunction)();
  VOID (*removePatternFunction)();
  struct expr *(*genJNConstantFunction)();
  VOID (*replaceGetJNValueFunction)();
  struct expr *(*genGetJNValueFunction)();
  struct expr *(*genCompareJNValuesFunction)();
  struct expr *(*genPNConstantFunction)();
  VOID (*replaceGetPNValueFunction)();
  struct expr *(*genGetPNValueFunction)();
  struct expr *(*genComparePNValuesFunction)();
  VOID (*returnUserDataFunction)();
  VOID *(*copyUserDataFunction)();
  VOID (*markIRPatternFunction)();
  VOID (*incrementalResetFunction)();
  struct lhsParseNode *(*initialPatternFunction)();
  VOID (*codeReferenceFunction)();
#endif
  {
   struct patternParser *newPtr, *currentPtr, *lastPtr = NULL;

   if (nextPosition >= MAX_POSITIONS) return(CLIPS_FALSE);
   
   newPtr = get_struct(patternParser);

   newPtr->name = name;
   newPtr->entityType = entityType;
   newPtr->recognizeFunction = recognizeFunction;
   newPtr->parseFunction = parseFunction;
   newPtr->postAnalysisFunction = postAnalysisFunction;
   newPtr->addPatternFunction = addPatternFunction;
   newPtr->removePatternFunction = removePatternFunction;
   newPtr->genJNConstantFunction = genJNConstantFunction;
   newPtr->replaceGetJNValueFunction = replaceGetJNValueFunction;
   newPtr->genGetJNValueFunction = genGetJNValueFunction;
   newPtr->genCompareJNValuesFunction = genCompareJNValuesFunction;
   newPtr->genPNConstantFunction = genPNConstantFunction;
   newPtr->replaceGetPNValueFunction = replaceGetPNValueFunction;
   newPtr->genGetPNValueFunction = genGetPNValueFunction;
   newPtr->genComparePNValuesFunction = genComparePNValuesFunction;
   newPtr->returnUserDataFunction = returnUserDataFunction;
   newPtr->copyUserDataFunction = copyUserDataFunction;
   newPtr->markIRPatternFunction = markIRPatternFunction;
   newPtr->incrementalResetFunction = incrementalResetFunction;
   newPtr->initialPatternFunction = initialPatternFunction;
   newPtr->codeReferenceFunction = codeReferenceFunction;
   newPtr->priority = priority;
   newPtr->positionInArray = nextPosition;
   PatternParserArray[nextPosition] = newPtr;
   nextPosition++;

   if (ListOfPatternParsers == NULL)
     {
      newPtr->next = NULL;
      ListOfPatternParsers = newPtr;
      return(CLIPS_TRUE);
     }

   currentPtr = ListOfPatternParsers;
   while ((currentPtr != NULL) ? (priority < currentPtr->priority) : CLIPS_FALSE)
     {
      lastPtr = currentPtr;
      currentPtr = currentPtr->next;
     }

   if (lastPtr == NULL)
     {
      newPtr->next = ListOfPatternParsers;
      ListOfPatternParsers = newPtr;
     }
   else
     {
      newPtr->next = currentPtr;
      lastPtr->next = newPtr;
     }

   return(CLIPS_TRUE);
  }
  
/***********************/
/* FindPatternParser:   */
/***********************/
globle struct patternParser *FindPatternParser(name)
  char *name;
  {
   struct patternParser *tempParser;

   tempParser = ListOfPatternParsers;
   while (tempParser != NULL)
     {
      if (strcmp(tempParser->name,name) == 0) return(tempParser);
      tempParser = tempParser->next;
     }
     
   return(NULL);
  }

/***********************************************************/
/* GetPatternParser:           */
/***********************************************************/
struct patternParser *GetPatternParser(rhsType)
  int rhsType;
  {
   return(PatternParserArray[rhsType]);
  }

#if CONSTRUCT_COMPILER && (! RUN_TIME)

/******************************************************************/
/* PatternNodeHeaderToCode:                    */
/******************************************************************/
globle VOID PatternNodeHeaderToCode(fp,theHeader,imageID,maxIndices)
  FILE *fp;
  struct patternNodeHeader *theHeader;
  int imageID;
  int maxIndices;
  {
   fprintf(fp,"{NULL,NULL,");
   
   if (theHeader->entryJoin == NULL)
     { fprintf(fp,"NULL,"); }
   else
     {
      fprintf(fp,"&%s%d_%d[%d],",
                 JoinPrefix(),imageID,
                 (((int) theHeader->entryJoin->bsaveID) / maxIndices) + 1,
                 ((int) theHeader->entryJoin->bsaveID) % maxIndices);
     }
   
   fprintf(fp,"%d,%d,%d,0,0,%d,%d}",theHeader->singlefieldNode,
                                     theHeader->multifieldNode,
                                     theHeader->stopNode,
                                     theHeader->beginSlot,
                                     theHeader->endSlot);
  }

#endif

#if (! RUN_TIME) && (! BLOAD_ONLY)

/******************************************************************/
/* PostPatternAnalysis:                    */
/******************************************************************/
globle BOOLEAN PostPatternAnalysis(theLHS)
  struct lhsParseNode *theLHS;
  {
   struct lhsParseNode *patternPtr;
   struct patternParser *tempParser;
   
   patternPtr = theLHS;
   while (patternPtr != NULL)
     {
      if ((patternPtr->type == PATTERN_CE) && (patternPtr->patternType != NULL))
        {
         tempParser = patternPtr->patternType;
         if (tempParser->postAnalysisFunction != NULL)
           { if ((*tempParser->postAnalysisFunction)(patternPtr)) return(CLIPS_TRUE); }
        }
        
      patternPtr = patternPtr->bottom;
     }  
      
   return(CLIPS_FALSE);
  }

/******************************************************************/
/* RestrictionParse: Parses a single field within a pattern. This */
/*    field may either be a single field wildcard, a multifield   */
/*    wildcard, a single field variable, a multifield variable,   */
/*    or a series of connected constraints.                       */
/*                                                                */
/* <constraint> ::= ? |                                           */
/*                  $? |                                          */
/*                  <connected-constraint>                        */
/******************************************************************/
struct lhsParseNode *RestrictionParse(readSource,theToken,multifieldSlot,
                                      theSlot,slotNumber,theConstraints,
                                      position)
  char *readSource;
  struct token *theToken;
  int multifieldSlot;
  struct symbolHashNode *theSlot;
  int slotNumber;
  CONSTRAINT_RECORD *theConstraints;
  int position;
  {
   struct lhsParseNode *topNode = NULL, *lastNode = NULL, *nextNode;
   int numberOfSingleFields = 0;
   int numberOfMultifields = 0;
   int startPosition;
   int error = CLIPS_FALSE;
   
   startPosition = position;

   while (theToken->type != RPAREN)
     {
      if ((theToken->type == SF_WILDCARD) ||
          (theToken->type == MF_WILDCARD))
        {
         nextNode = GetLHSParseNode();
         nextNode->type = theToken->type;
         nextNode->negated = CLIPS_FALSE;
         GetToken(readSource,theToken);
        }
      else
        { 
         nextNode = ConjuctiveRestrictionParse(readSource,theToken,&error);
        }

      if (nextNode == NULL)
        {
         ReturnLHSParseNodes(topNode);
         return(NULL);
        }
              
      if ((theToken->type != RPAREN) && (multifieldSlot == CLIPS_TRUE))
        {
         PPBackup();
         SavePPBuffer(" ");
         SavePPBuffer(theToken->printForm);
        }
      
      if ((nextNode->type == SF_WILDCARD) || (nextNode->type == SF_VARIABLE))
        { numberOfSingleFields++; }
      else
        { numberOfMultifields++; }
        
      nextNode->slot = theSlot;
      nextNode->slotNumber = slotNumber;
      nextNode->index = position++;
      if (! multifieldSlot)
        {
         nextNode->constraints = theConstraints;
         return(nextNode);
        }
        
      if (lastNode == NULL) topNode = nextNode;
      else lastNode->right = nextNode;
      
      lastNode = nextNode;
     }

    if ((topNode == NULL) && (! multifieldSlot))
      {
       SyntaxErrorMessage("defrule");
       return(NULL);
      }
      
    nextNode = topNode;
    while (nextNode != NULL)
      {
       if (theConstraints != NULL)
         {
          nextNode->constraints = CopyConstraintRecord(theConstraints);
          ReturnExpression(nextNode->constraints->minFields);
          ReturnExpression(nextNode->constraints->maxFields);
          nextNode->constraints->minFields = GenConstant(SYMBOL,NegativeInfinity);
          nextNode->constraints->maxFields = GenConstant(SYMBOL,PositiveInfinity);
          nextNode->derivedConstraints = CLIPS_TRUE;
          
          if ((nextNode->type == MF_WILDCARD) || (nextNode->type == MF_VARIABLE))
            {
             if (theConstraints->maxFields->value != PositiveInfinity)
               {
                ReturnExpression(nextNode->constraints->maxFields);
                nextNode->constraints->maxFields = GenConstant(INTEGER,AddLong(ValueToLong(theConstraints->maxFields->value) - numberOfSingleFields));
               }
               
             if ((numberOfMultifields == 1) && (theConstraints->minFields->value != NegativeInfinity))
               {
                ReturnExpression(nextNode->constraints->minFields);
                nextNode->constraints->minFields = GenConstant(INTEGER,AddLong(ValueToLong(theConstraints->minFields->value) - numberOfSingleFields));
               }
            }
         }
       nextNode = nextNode->right;
      }
      
   if (multifieldSlot)
     {
      nextNode = GetLHSParseNode();
      nextNode->type = MF_WILDCARD;
      nextNode->multifieldSlot = CLIPS_TRUE;
      nextNode->bottom = topNode;
      nextNode->slot = theSlot;
      nextNode->slotNumber = slotNumber;
      nextNode->index = startPosition;
      nextNode->constraints = theConstraints;
      topNode = nextNode;
      TallyFieldTypes(topNode->bottom);
     }
     
   return(topNode);
  }

/***************************************************************/
/* TallyFieldTypes: Determines the number of single field and  */
/*   multifield variables and wildcards that appear before and */
/*   after each restriction found in a multifield slot.        */
/***************************************************************/
static VOID TallyFieldTypes(theRestrictions)
  struct lhsParseNode *theRestrictions;
  {
   struct lhsParseNode *tempNode1, *tempNode2, *tempNode3;
   int totalSingleFields = 0, totalMultiFields = 0;
   int runningSingleFields = 0, runningMultiFields = 0;
   
   for (tempNode1 = theRestrictions; tempNode1 != NULL; tempNode1 = tempNode1->right)
     {
      if ((tempNode1->type == SF_VARIABLE) || (tempNode1->type == SF_WILDCARD))
        { totalSingleFields++; }
      else
        { totalMultiFields++; }
     }
     
   for (tempNode1 = theRestrictions; tempNode1 != NULL; tempNode1 = tempNode1->right)
     {
      tempNode1->singleFieldsBefore = runningSingleFields;
      tempNode1->multiFieldsBefore = runningMultiFields;
      tempNode1->withinMultifieldSlot = CLIPS_TRUE;
      
      if ((tempNode1->type == SF_VARIABLE) || (tempNode1->type == SF_WILDCARD))
        { 
         tempNode1->singleFieldsAfter = totalSingleFields - (runningSingleFields + 1);
         tempNode1->multiFieldsAfter = totalMultiFields - runningMultiFields;
        }
      else
        { 
         tempNode1->singleFieldsAfter = totalSingleFields - runningSingleFields;
         tempNode1->multiFieldsAfter = totalMultiFields - (runningMultiFields + 1);
        }
         
      for (tempNode2 = tempNode1->bottom; tempNode2 != NULL; tempNode2 = tempNode2->bottom)
        {
         for (tempNode3 = tempNode2; tempNode3 != NULL; tempNode3 = tempNode3->right)
           {
            tempNode3->singleFieldsBefore = tempNode1->singleFieldsBefore;
            tempNode3->singleFieldsAfter = tempNode1->singleFieldsAfter;
            tempNode3->multiFieldsBefore = tempNode1->multiFieldsBefore;
            tempNode3->multiFieldsAfter = tempNode1->multiFieldsAfter;
            tempNode3->withinMultifieldSlot = CLIPS_TRUE;
           }
        }
        
      if ((tempNode1->type == SF_VARIABLE) || (tempNode1->type == SF_WILDCARD))
        { runningSingleFields++; }
      else
        { runningMultiFields++; }
     }
  }
  
/*******************************************************************/
/* ConjuctiveRestrictionParse: Parses a single constraint field in */
/*   a pattern that is not a single field wildcard, multifield     */
/*   wildcard, or multifield variable. The field may consist of a  */
/*   number of subfields tied together using the & connective      */
/*   constraint and/or the | connective constraint.                */
/*                                                                 */
/* <connected-constraint>                                          */
/*            ::= <single-constraint> |                            */
/*                <single-constraint> & <connected-constraint> |   */
/*                <single-constraint> | <connected-constraint>     */
/*******************************************************************/
static struct lhsParseNode *ConjuctiveRestrictionParse(readSource,theToken,error)
  char *readSource;
  struct token *theToken;
  int *error;
  {
   struct lhsParseNode *bindNode;
   struct lhsParseNode *theNode, *nextOr, *nextAnd;
   int connectorType;

   /*=====================================*/
   /* Get the first node and determine if */
   /* it is a binding variable.           */
   /*=====================================*/

   theNode = LiteralRestrictionParse(readSource,theToken,error);

   if (*error == CLIPS_TRUE)
     { return(NULL); }

   GetToken(readSource,theToken);
   
   if (((theNode->type == SF_VARIABLE) || (theNode->type == MF_VARIABLE)) &&
       (theNode->negated == CLIPS_FALSE) &&
       (theToken->type != OR_CONSTRAINT))
     {
      theNode->bindingVariable = CLIPS_TRUE;
      bindNode = theNode;
      nextOr = NULL;
      nextAnd = NULL;
     }
   else
     {
      bindNode = GetLHSParseNode();
      if (theNode->type == MF_VARIABLE) bindNode->type = MF_WILDCARD;
      else bindNode->type = SF_WILDCARD;
      bindNode->negated = CLIPS_FALSE;
      bindNode->bottom = theNode;
      nextOr = theNode;
      nextAnd = theNode;
     }

   /*=====================================*/
   /* Get the first node and determine if */
   /* it is a binding variable.           */
   /*=====================================*/

   while ((theToken->type == OR_CONSTRAINT) || (theToken->type == AND_CONSTRAINT))
     {
      /*==========================*/
      /* Get the next constraint. */
      /*==========================*/

      connectorType = theToken->type;

      GetToken(readSource,theToken);
      theNode = LiteralRestrictionParse(readSource,theToken,error);

      if (*error == CLIPS_TRUE)
        {
         ReturnLHSParseNodes(bindNode);
         return(NULL);
        }

      /*=======================================*/
      /* Attach the new constraint to the list */
      /* of constraints for this field.        */
      /*=======================================*/

      if (connectorType == OR_CONSTRAINT)
        {
         if (nextOr == NULL)
           { bindNode->bottom = theNode; }
         else
           { nextOr->bottom = theNode; }
         nextOr = theNode;
         nextAnd = theNode;
        }
      else if (connectorType == AND_CONSTRAINT)
        {
         if (nextAnd == NULL)
           {
            bindNode->bottom = theNode;
            nextOr = theNode;
           }
         else
           { nextAnd->right = theNode; }
         nextAnd = theNode;
        }
      else
        {
         CLIPSSystemError("RULEPSR",1);
         ExitCLIPS(4);
        }

      /*==================================================*/
      /* Determine if any more restrictions are connected */
      /* to the current list of restrictions.             */
      /*==================================================*/

      GetToken(readSource,theToken);
     }

   if (CheckForVariableMixing(bindNode))
     { 
      *error = CLIPS_TRUE;
      ReturnLHSParseNodes(bindNode);
      return(NULL);
     }
     
   return(bindNode);
  }
  
/***********************************************************/
/* CheckForVariableMixing: */
/***********************************************************/
static int CheckForVariableMixing(theRestriction)
  struct lhsParseNode *theRestriction;
  {
   struct lhsParseNode *tempRestriction;
   CONSTRAINT_RECORD *theConstraint;
   int multifield = CLIPS_FALSE;
   int singlefield = CLIPS_FALSE;
   int constant = CLIPS_FALSE;
   int singleReturnValue = CLIPS_FALSE;
   int multiReturnValue = CLIPS_FALSE;
   
   if (theRestriction->type == SF_VARIABLE) singlefield = CLIPS_TRUE;
   else if (theRestriction->type == MF_VARIABLE) multifield = CLIPS_TRUE;
   
   theRestriction = theRestriction->bottom;
   while (theRestriction != NULL)
     {
      tempRestriction = theRestriction;
      while (tempRestriction != NULL)
        {
         if (tempRestriction->type == SF_VARIABLE) singlefield = CLIPS_TRUE;
         else if (tempRestriction->type == MF_VARIABLE) multifield = CLIPS_TRUE;
         else if (ConstantType(tempRestriction->type)) constant = CLIPS_TRUE;
         else if (tempRestriction->type == RETURN_VALUE_CONSTRAINT)
           {
            theConstraint = FunctionCallToConstraintRecord(tempRestriction->expression->value);
            if (theConstraint->anyAllowed) { /* Do nothing. */ }
            else if (theConstraint->multifieldsAllowed) multiReturnValue = CLIPS_TRUE;
            else singleReturnValue = CLIPS_TRUE;
            RemoveConstraint(theConstraint);
           }
         
         tempRestriction = tempRestriction->right;
        }
      theRestriction = theRestriction->bottom;
     }
     
   if ((singlefield || constant || singleReturnValue) &&
       (multifield || multiReturnValue))
       
     {
      PrintErrorID("PATTERN",2,CLIPS_TRUE);
      PrintCLIPS(WERROR,"Single and multifield constraints cannot be mixed in a field constraint\n");
      return(CLIPS_TRUE);
     }
     
   return(CLIPS_FALSE);
  }

/***********************************************************/
/* LiteralRestrictionParse: Parses a subfield of a field.  */
/*   The subfield may be a literal constraint, a predicate */
/*   constraint, a return value constraint, or a variable  */
/*   constraint. The constraints may also be negated using */
/*   the ~ connective constraint.                          */
/*                                                         */
/* <single-constraint>     ::= <term> | ~<term>            */
/*                                                         */
/*  <term>                 ::= <constant> |                */
/*                             <single-field-variable> |   */
/*                             <multi-field-variable> |    */
/*                             :<function-call> |          */
/*                             =<function-call>            */
/***********************************************************/
static struct lhsParseNode *LiteralRestrictionParse(readSource,theToken,error)
  char *readSource;
  struct token *theToken;
  int *error;
  {
   struct lhsParseNode *topNode;
   struct expr *theExpression;

   topNode = GetLHSParseNode();

   /*====================================================*/
   /* Determine if the field has a '~' preceding it. If  */
   /* it  does, then the field has negative state logic. */
   /* Otherwise the field has positive state logic.      */
   /*====================================================*/

   if (theToken->type == NOT_CONSTRAINT)
     {
      GetToken(readSource,theToken);
      topNode->negated = CLIPS_TRUE;
     }
   else
     { topNode->negated = CLIPS_FALSE; }

   /*================================================================*/
   /* Determine if the field is valid.  Valid fields are ?variables, */
   /* words, strings, numbers, :(expression), and =(expression).     */
   /*================================================================*/

   topNode->type = theToken->type;

   if (theToken->type == SYMBOL)
     {
      if (strcmp(ValueToString(theToken->value),"=") == 0)
        {
         theExpression = Function0Parse(readSource);
         if (theExpression == NULL)
           {
            *error = CLIPS_TRUE;
            ReturnLHSParseNodes(topNode);
            return(NULL);
           }
         topNode->type = RETURN_VALUE_CONSTRAINT;
         topNode->expression = ExpressionToLHSParseNodes(theExpression);
         ReturnExpression(theExpression);
        }
      else if (strcmp(ValueToString(theToken->value),":") == 0)
        {
         theExpression = Function0Parse(readSource);
         if (theExpression == NULL)
           {
            *error = CLIPS_TRUE;
            ReturnLHSParseNodes(topNode);
            return(NULL);
           }
         topNode->type = PREDICATE_CONSTRAINT;
         topNode->expression = ExpressionToLHSParseNodes(theExpression);
         ReturnExpression(theExpression);
        }
      else
        { topNode->value = theToken->value; }
     }
   else if ((theToken->type == SF_VARIABLE)  ||
            (theToken->type == MF_VARIABLE)  ||
            (theToken->type == FLOAT) ||
            (theToken->type == INTEGER) ||
            (theToken->type == STRING) ||
            (theToken->type == INSTANCE_NAME))
     { topNode->value = theToken->value; }
   else
     {
      SyntaxErrorMessage("defrule");
      *error = CLIPS_TRUE;
      ReturnLHSParseNodes(topNode);
      return(NULL);
     }

   return(topNode);
  }

#endif

#endif




