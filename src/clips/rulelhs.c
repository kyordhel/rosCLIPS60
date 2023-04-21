   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             DEFRULE LHS PARSING MODULE              */
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

#define _RULELHS_SOURCE_

#include "setup.h"

#if (! RUN_TIME) && (! BLOAD_ONLY) &&  DEFRULE_CONSTRUCT

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "constant.h"
#include "symbol.h"
#include "clipsmem.h"
#include "exprnpsr.h"
#include "argacces.h"
#include "scanner.h"
#include "reorder.h"
#include "router.h"
#include "ruledef.h"
#include "constrct.h"
#include "cstrnchk.h"
#include "constrnt.h"
#include "pattern.h"
#include "agenda.h"
#include "rulelhs.h"

#if ANSI_COMPILER
   static struct lhsParseNode    *RuleBodyParse(char *,struct token *,char *,int *);
   static VOID                    DeclarationParse(char *,char *,int *);
   static struct lhsParseNode    *LHSPattern(char *,int,char *,int *);
   static struct lhsParseNode    *ConnectedPatternParse(char *,struct token *,int *);
   static struct lhsParseNode    *GroupPatterns(char *,int,char *,int *);
   static struct lhsParseNode    *TestPattern(char *,int *);
   static struct lhsParseNode    *AssignmentParse(char *,SYMBOL_HN *,int *);
   static VOID                    TagLHSLogicalNodes(struct lhsParseNode *);
   static struct lhsParseNode    *SimplePatternParse(char *,struct token *,int *);
   static VOID                    ParseSalience(char *,char *,int *);
   static VOID                    ParseAutoFocus(char *,int *);
#else
   static struct lhsParseNode    *RuleBodyParse();
   static VOID                    DeclarationParse();
   static struct lhsParseNode    *LHSPattern();
   static struct lhsParseNode    *ConnectedPatternParse();
   static struct lhsParseNode    *GroupPatterns();
   static struct lhsParseNode    *TestPattern();
   static struct lhsParseNode    *AssignmentParse();
   static VOID                    TagLHSLogicalNodes();
   static struct lhsParseNode    *SimplePatternParse();
   static VOID                    ParseSalience();
   static VOID                    ParseAutoFocus();
#endif

/****************************/
/* LOCAL INTERNAL VARIABLES */
/****************************/

   static int                    WithinNotCE = CLIPS_FALSE;

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle int                GlobalSalience;
   globle int                GlobalAutoFocus;
   globle struct expr       *SalienceExpression = NULL;

/*******************************************************************/
/* ParseRuleLHS: Coordinates all the actions necessary for parsing */
/*   the LHS of a rule including the reordering of pattern         */
/*   conditional elements to conform with the CLIPS Rete topology. */
/*******************************************************************/
globle struct lhsParseNode *ParseRuleLHS(readSource,theToken,ruleName)
  char *readSource;
  struct token *theToken;
  char *ruleName;
  {
   struct lhsParseNode *theLHS;
   int result;
   int error = CLIPS_FALSE;
   
   /*=====================================================*/
   /* Get the raw representation for the LHS of the rule. */
   /*=====================================================*/

   GlobalSalience = 0;
   GlobalAutoFocus = CLIPS_FALSE;
   SalienceExpression = NULL;

   SetIndentDepth(3);
   
   theLHS = RuleBodyParse(readSource,theToken,ruleName,&error);

   if (error) return(NULL);
   
   /*====================================================*/
   /* Reorder the raw representation so that it consists */
   /* of at most a single top level OR CE containing one */
   /* or more AND CEs.                                   */
   /*====================================================*/

   theLHS = ReorderPatterns(theLHS,&result);
   
   /*================================*/
   /* Return the LHS representation. */
   /*================================*/

   return(theLHS);
  }

/*********************************************************/
/* RuleBodyParse: Parses the LHS of a rule, but does not */
/*   reorder any of the LHS patterns to conform with the */
/*   CLIPS Rete Topology.                                */
/*                                                       */
/* <rule-body> ::= [<declaration>]                       */
/*                 <conditional-element>*                */
/*                 =>                                    */
/*********************************************************/
static struct lhsParseNode *RuleBodyParse(readSource,theToken,ruleName,error)
  char *readSource;
  struct token *theToken;
  char *ruleName;
  int *error;
  {
   struct lhsParseNode *theNode, *otherNodes;

   *error = CLIPS_FALSE;

   /*============================================*/
   /* Parse the first pattern as a special case. */
   /*============================================*/

   if ((theToken->type == SYMBOL) ? (strcmp(ValueToString(theToken->value),"=>") == 0) : CLIPS_FALSE)
     { return(NULL); }
   else if (theToken->type == LPAREN)
     {
      GetToken(readSource,theToken);
      if (theToken->type == SYMBOL)
        {
         if (strcmp(ValueToString(theToken->value),"declare") == 0)
           {
            DeclarationParse(readSource,ruleName,error);
            theNode = NULL;
           }
         else if (strcmp(ValueToString(theToken->value),"test") == 0)
           { theNode = TestPattern(readSource,error); }
         else if ((strcmp(ValueToString(theToken->value),"and") == 0) ||
                  (strcmp(ValueToString(theToken->value),"logical") == 0) ||
                  (strcmp(ValueToString(theToken->value),"not") == 0) ||
                  (strcmp(ValueToString(theToken->value),"exists") == 0) ||
                  (strcmp(ValueToString(theToken->value),"forall") == 0) ||
                  (strcmp(ValueToString(theToken->value),"or") == 0))
           { theNode = ConnectedPatternParse(readSource,theToken,error); }
         else
           { theNode = SimplePatternParse(readSource,theToken,error); }
        }
      else
        { theNode = SimplePatternParse(readSource,theToken,error); }
     }
   else if (theToken->type == SF_VARIABLE)
     { theNode = AssignmentParse(readSource,theToken->value,error); }
   else
     {
      SyntaxErrorMessage("defrule");
      *error = CLIPS_TRUE;
      return(NULL);
     }

   if (*error == CLIPS_TRUE)
     {
      ReturnLHSParseNodes(theNode);
      return(NULL);
     }

   PPCRAndIndent();

   /*======================================*/
   /* Parse the other patterns in the LHS. */
   /*======================================*/

   otherNodes = GroupPatterns(readSource,SYMBOL,"=>",error);

   if (*error == CLIPS_TRUE)
     {
      ReturnLHSParseNodes(theNode);
      return(NULL);
     }

   if (theNode == NULL)
     { theNode = otherNodes; }
   else
     { theNode->bottom = otherNodes; }

   return(theNode);
  }

/********************************************************/
/* DeclarationParse: Parses a defrule declaration. Only */
/*   salience declarations are currently allowed.       */
/*                                                      */
/* <declaration> ::= (declare <rule-property>+)         */
/*                                                      */
/* <rule-property> ::= (salience <integer-expression>)  */
/********************************************************/
static VOID DeclarationParse(readSource,ruleName,error)
  char *readSource;
  char *ruleName;
  int *error;
  {
   struct token theToken;
   struct expr *packPtr;
   int notDone = CLIPS_TRUE;
   int salienceParsed = CLIPS_FALSE, autoFocusParsed = CLIPS_FALSE;
   
   /*===========================*/
   /* Next token must be a '('. */
   /*===========================*/

   SavePPBuffer(" ");
   
   GetToken(readSource,&theToken);
   if (theToken.type != LPAREN)
     {
      SyntaxErrorMessage("declare statement");
      *error = CLIPS_TRUE;
      return;
     }

   while (notDone)
     {
      /*===========================================================*/
      /* Next token must be the symbol "salience" or "auto-focus". */
      /*===========================================================*/

      GetToken(readSource,&theToken);
      if (strcmp(ValueToString(theToken.value),"salience") == 0)
        { 
         if (salienceParsed)
           {
            AlreadyParsedErrorMessage("salience declaration",NULL);
            *error = CLIPS_TRUE;
           }
         else
           {    
            ParseSalience(readSource,ruleName,error); 
            salienceParsed = CLIPS_TRUE;
           }
        }
      else if (strcmp(ValueToString(theToken.value),"auto-focus") == 0)
        { 
         if (autoFocusParsed)
           {            
            AlreadyParsedErrorMessage("auto-focus declaration",NULL);
            *error = CLIPS_TRUE;
           }
         else
           {
            ParseAutoFocus(readSource,error);
            autoFocusParsed = CLIPS_TRUE;
           }
         }
       else
        {
         SyntaxErrorMessage("declare statement");
         *error = CLIPS_TRUE;
        }

      if (*error)
        {
         ReturnExpression(SalienceExpression);
         SalienceExpression = NULL;
         return;
        }
     
      /*=====================================*/
      /* Declarations are closed with a ')'. */
      /*=====================================*/

      GetToken(readSource,&theToken);
      if (theToken.type != RPAREN)
        {
         PPBackup();
         SavePPBuffer(" ");
         SavePPBuffer(theToken.printForm);
         ReturnExpression(SalienceExpression);
         SalienceExpression = NULL;
         SyntaxErrorMessage("declare statement");
         *error = CLIPS_TRUE;
         return;
        }

      /*===================================*/
      /* Declaration is closed with a ')'. */
      /*===================================*/

      GetToken(readSource,&theToken);
      if (theToken.type == RPAREN) notDone = CLIPS_FALSE;
      else if (theToken.type != LPAREN)
        {
         ReturnExpression(SalienceExpression);
         SalienceExpression = NULL;
         SyntaxErrorMessage("declare statement");
         *error = CLIPS_TRUE;
         return;
        }
      else
        {
         PPBackup();
         SavePPBuffer(" (");
        }
     }
   

   /*==============================================*/
   /* Return the value of the salience through the */
   /* global variable salience_value.              */
   /*==============================================*/

   packPtr = PackExpression(SalienceExpression);
   ReturnExpression(SalienceExpression);
   SalienceExpression = packPtr;
   return;
  }

/********************************************************/
/* ParseSalience:   */
/********************************************************/
static VOID ParseSalience(readSource,ruleName,error)
  char *readSource;
  char *ruleName;
  int *error;
  {
   int salience;
   DATA_OBJECT salienceValue;

   /*==============================*/
   /* Get the salience expression. */
   /*==============================*/

   SavePPBuffer(" ");

   SalienceExpression = ParseAtomOrExpression(readSource,NULL);
   if (SalienceExpression == NULL)
     {
      *error = CLIPS_TRUE;
      return;
     }

   /*============================================================*/
   /* Evaluate the expression and determine if it is an integer. */
   /*============================================================*/

   SetEvaluationError(CLIPS_FALSE);
   if (EvaluateExpression(SalienceExpression,&salienceValue))
     {
      SalienceInformationError(ruleName);
      *error = CLIPS_TRUE;
      return;
     }

   if (salienceValue.type != INTEGER)
     {
      SalienceNonIntegerError();
      *error = CLIPS_TRUE;
      return;
     }

   /*=======================================================*/
   /* Salience number must be in the range -10000 to 10000. */
   /*=======================================================*/

   salience = (int) ValueToLong(salienceValue.value);

   if ((salience > MAX_SALIENCE) || (salience < MIN_SALIENCE))
     {
      SalienceRangeError();
      *error = CLIPS_TRUE;
      return;
     }
     
   /*==========================================*/
   /* If the expression is a constant integer, */
   /* don't bother storing the expression.     */
   /*==========================================*/

   if (SalienceExpression->type == INTEGER)
     {
      ReturnExpression(SalienceExpression);
      SalienceExpression = NULL;
     }
   
   GlobalSalience = salience;
  }

/**************************************************************/
/* ParseAutoFocus: Parses the rest of a defrule auto-focus    */
/*   declaration once the auto-focus keyword has been parsed. */
/**************************************************************/
static VOID ParseAutoFocus(readSource,error)
  char *readSource;
  int *error;
  {
   struct token theToken;

   /*========================================*/
   /* The auto-focus value must be a symbol. */
   /*========================================*/
   
   SavePPBuffer(" ");
   
   GetToken(readSource,&theToken);
   if (theToken.type != SYMBOL)
     {
      SyntaxErrorMessage("auto-focus statement");
      *error = CLIPS_TRUE;
      return;
     }
     
   /*====================================================*/
   /* The auto-focus value must be either TRUE or FALSE. */
   /*====================================================*/
   
   if (strcmp(ValueToString(theToken.value),"TRUE") == 0) 
     { GlobalAutoFocus = CLIPS_TRUE; }
   else if (strcmp(ValueToString(theToken.value),"FALSE") == 0) 
     { GlobalAutoFocus = CLIPS_FALSE; }
   else
     {
      SyntaxErrorMessage("auto-focus statement");
      *error = CLIPS_TRUE;
     }
  }
  
/*****************************************************************/
/* LHSPattern: Parses a single conditional element found on the  */
/*   LHS of a rule. Conditonal element types include pattern CEs */
/*   (which may be assigned to a variable), test CEs, not CEs,   */
/*   logical CEs, and CEs, and or CEs.                           */
/*                                                               */
/* <conditional-element> ::= <pattern-CE> |                      */
/*                           <assigned-pattern-CE> |             */
/*                           <not-CE> | <and-CE> | <or-CE> |     */
/*                           <logical-CE> | <test-CE> |          */
/*                           <forall-CE> | <exists-CE>           */
/*****************************************************************/
static struct lhsParseNode *LHSPattern(readSource,terminator,terminatorString,error)
  char *readSource;
  int terminator;
  char *terminatorString;
  int *error;
  {
   struct token theToken;
   struct lhsParseNode *theNode;

   GetToken(readSource,&theToken);
   if (theToken.type == LPAREN)
     {
      GetToken(readSource,&theToken);
      if (theToken.type == SYMBOL)
        {
         if (strcmp(ValueToString(theToken.value),"test") == 0)
           { theNode = TestPattern(readSource,error); }
         else if ((strcmp(ValueToString(theToken.value),"and") == 0) ||
                  (strcmp(ValueToString(theToken.value),"logical") == 0) ||
                  (strcmp(ValueToString(theToken.value),"not") == 0) ||
                  (strcmp(ValueToString(theToken.value),"exists") == 0) ||
                  (strcmp(ValueToString(theToken.value),"forall") == 0) ||
                  (strcmp(ValueToString(theToken.value),"or") == 0))
           { theNode = ConnectedPatternParse(readSource,&theToken,error); }
         else
           { theNode = SimplePatternParse(readSource,&theToken,error); }
        }
      else
        { theNode = SimplePatternParse(readSource,&theToken,error); }
     }
   else if (theToken.type == SF_VARIABLE)
     { theNode = AssignmentParse(readSource,theToken.value,error); }
   else if ((theToken.type == terminator) ?
            (strcmp(theToken.printForm,terminatorString) == 0) : CLIPS_FALSE)
     { return(NULL);  }
   else
     {
      SyntaxErrorMessage("defrule");
      *error = CLIPS_TRUE;
      return(NULL);
     }

   if (*error == CLIPS_TRUE)
     {
      ReturnLHSParseNodes(theNode);
      return(NULL);
     }

   return(theNode);
  }

/*********************************************************************/
/* ConnectedPatternParse:  Handles parsing of the connected          */
/*   conditional elements (i.e. those conditional elements that may  */
/*   contain one or more other conditional elements).  The connected */
/*   conditional elements include the and CE, the or CE, the         */
/*   logical CE, and the not CE.  This routine is entered with the   */
/*   parsing pointing to the name of the connected CE. It is exited  */
/*   with the parser pointing to the closing right parenthesis of    */
/*   the connected CE.                                               */
/*                                                                   */
/* <and-CE>      ::= (and <conditional-element>+)                    */
/*                                                                   */
/* <or-CE>       ::= (or <conditional-element>+)                     */
/*                                                                   */
/* <logical-CE>  ::= (logical <conditional-element>+)                */
/*                                                                   */
/* <not-CE>      ::= (not <conditional-element>)                     */
/*                                                                   */
/* <exists-CE>   ::= (exists <conditional-element>+)                 */
/*                                                                   */
/* <forall-CE>   ::= (forall <conditional-element>                   */
/*                           <conditional-element>+)                 */
/*********************************************************************/
static struct lhsParseNode *ConnectedPatternParse(readSource,theToken,error)
  char *readSource;
  struct token *theToken;
  int *error;
  {
   int connectorValue = 0;
   struct lhsParseNode *theNode, *tempNode, *theGroup;
   char *errorCE;
   int logical = CLIPS_FALSE;
   int tempValue;

   /*==========================================================*/
   /* Use appropriate spacing for pretty printing of the rule. */
   /*==========================================================*/

   IncrementIndentDepth(5);
   if (strcmp(ValueToString(theToken->value),"or") == 0)
     {
      connectorValue = OR_CE;
      errorCE = "the or conditional element";
      SavePPBuffer("  ");
     }
   else if (strcmp(ValueToString(theToken->value),"and") == 0)
     {
      connectorValue = AND_CE;
      errorCE = "the and conditional element";
      SavePPBuffer(" ");
     }
   else if (strcmp(ValueToString(theToken->value),"not") == 0)
     {
      connectorValue = NOT_CE;
      errorCE = "the not conditional element";
      SavePPBuffer(" ");
     }
   else if (strcmp(ValueToString(theToken->value),"exists") == 0)
     {      
      connectorValue = EXISTS_CE;
      errorCE = "the exists conditional element";
      PPCRAndIndent();
     }
   else if (strcmp(ValueToString(theToken->value),"forall") == 0)
     {
      connectorValue = FORALL_CE;
      errorCE = "the forall conditional element";
      PPCRAndIndent();
     }
   else if (strcmp(ValueToString(theToken->value),"logical") == 0)
     {
      connectorValue = AND_CE;
      errorCE = "the logical conditional element";
      logical = CLIPS_TRUE;
      PPCRAndIndent();
     }
   
   /*=====================================================*/
   /* The logical CE cannot be contained within a not CE. */
   /*=====================================================*/
   
   if (WithinNotCE && logical)
     {
      PrintErrorID("RULELHS",1,CLIPS_TRUE);
      PrintCLIPS(WERROR,"The logical CE cannot be used within a not/exists/forall CE.\n");
      *error = CLIPS_TRUE;
      return(NULL);
     }
  
   /*========================================================*/
   /* Advance the scanner to the first point of interest in  */
   /* pattern that follows the logical connector: Either the */
   /* fact binder for the pattern or the pattern's first     */
   /* element slot.                                          */
   /*========================================================*/
   
   tempValue = WithinNotCE;
   if ((connectorValue == NOT_CE) || 
       (connectorValue == EXISTS_CE) ||
       (connectorValue == FORALL_CE))
     { WithinNotCE = CLIPS_TRUE; }
     
   theGroup = GroupPatterns(readSource,RPAREN,")",error);

   WithinNotCE = tempValue;
   
   DecrementIndentDepth(5);

   if (*error == CLIPS_TRUE)
     {
      ReturnLHSParseNodes(theGroup);
      return(NULL);
     }
   
   if (logical) TagLHSLogicalNodes(theGroup);
   
   /*=====================================================*/
   /* All the connected CEs must contain at least one CE. */
   /*=====================================================*/
    
   if (theGroup == NULL)
     {
      SyntaxErrorMessage(errorCE);
      *error = CLIPS_TRUE;
      return(NULL);
     }  
   
   /*============================================*/
   /* A not CE may not contain more than one CE. */
   /*============================================*/
   
   if ((connectorValue == NOT_CE) && (theGroup->bottom != NULL))
     {
      SyntaxErrorMessage(errorCE);
      ReturnLHSParseNodes(theGroup);
      *error = CLIPS_TRUE;
      return(NULL);
     }   
     
   /*============================================*/
   /* A forall CE must contain at least two CEs. */
   /*============================================*/
   
   if ((connectorValue == FORALL_CE) && (theGroup->bottom == NULL))
     {
      SyntaxErrorMessage(errorCE);
      ReturnLHSParseNodes(theGroup);
      *error = CLIPS_TRUE;
      return(NULL);
     }
   
   /*========================================================*/
   /* Remove an "and" and "or" CE that only contains one CE. */
   /*========================================================*/
   /*
   if (((connectorValue == AND_CE) || (connectorValue == OR_CE)) && 
       (theGroup->bottom == NULL))
     { return(theGroup); }
     */
   /*===========================================================*/
   /* Create the top most node which connects the CEs together. */
   /*===========================================================*/
   
   theNode = GetLHSParseNode();
   theNode->logical = logical;
      
   /*======================================================*/
   /* Attach and/or/not CEs directly to the top most node. */
   /*======================================================*/
   
   if ((connectorValue == AND_CE) || 
       (connectorValue == OR_CE) ||
       (connectorValue == NOT_CE))
     {
      theNode->type = connectorValue;
      theNode->right = theGroup;
     }
   
   /*=================================================================*/
   /* Wrap two not CEs around the patterns contained in an exists CE. */
   /*=================================================================*/
   
   else if (connectorValue == EXISTS_CE)
     {
      theNode->type = NOT_CE;
      
      theNode->right = GetLHSParseNode();
      theNode->right->type = NOT_CE;
      theNode->right->logical = logical;
      
      if (theGroup->bottom != NULL)
        {
         theNode->right->right = GetLHSParseNode();
         theNode->right->right->type = AND_CE;
         theNode->right->right->logical = logical;
         theNode->right->right->right = theGroup;
        }
      else
        { theNode->right->right = theGroup; }
     }
   
   /*==================================================*/
   /* For a forall CE, wrap a not CE around all of the */
   /* CEs and a not CE around the 2nd through nth CEs. */
   /*==================================================*/
   
   else if (connectorValue == FORALL_CE)
     {
      theNode->type = NOT_CE;
      
      tempNode = theGroup->bottom;
      theGroup->bottom = NULL;
      
      theNode->right = GetLHSParseNode();
      theNode->right->type = AND_CE;
      theNode->right->logical = logical;
      theNode->right->right = theGroup;
      
      theGroup = tempNode;
      
      theNode->right->right->bottom = GetLHSParseNode();
      theNode->right->right->bottom->type = NOT_CE;
      theNode->right->right->bottom->logical = logical;
      
      tempNode = theNode->right->right->bottom;
      
      if (theGroup->bottom == NULL) 
        { tempNode->right = theGroup; }
      else
        {
         tempNode->right = GetLHSParseNode();
         tempNode->right->type = AND_CE;
         tempNode->right->logical = logical;
         tempNode->right->right = theGroup;
        }
     }
     
   /*================*/
   /* Return the CE. */
   /*================*/
   
   return(theNode);
  }

/***********************************************************/
/* GroupPatterns: Groups a series of connected conditional */
/*   elements together.                                    */
/*                                                         */
/* <and-CE>      ::= (and <conditional-element>+)          */
/*                                                         */
/* <or-CE>       ::= (or <conditional-element>+)           */
/*                                                         */
/* <logical-CE>  ::= (logical <conditional-element>+)      */
/***********************************************************/
static struct lhsParseNode *GroupPatterns(readSource,terminator,terminatorString,error)
  int terminator;
  char *readSource, *terminatorString;
  int *error;
  {
   struct lhsParseNode *lastNode, *newNode, *theNode;

   lastNode = theNode = NULL;

   while (CLIPS_TRUE)
     {
      newNode = LHSPattern(readSource,terminator,terminatorString,error);

      if (*error)
        {
         ReturnLHSParseNodes(theNode);
         return(NULL);
        }

      if (newNode == NULL)
        {
         PPBackup();
         PPBackup();
         if (terminator == RPAREN)
           { SavePPBuffer(terminatorString); }
         else
           {
            PPCRAndIndent();
            SavePPBuffer(terminatorString);
           }

         return(theNode);
        }

      if (lastNode == NULL)
        { theNode = newNode; }
      else
        { lastNode->bottom = newNode; }

      lastNode = newNode;
      PPCRAndIndent();
     }
  }

/**************************************************************/
/* TestPattern: Handles parsing of test conditional elements. */
/*                                                            */
/* <test-CE> ::= (test <function-call>)                       */
/**************************************************************/
static struct lhsParseNode *TestPattern(readSource,error)
  char *readSource;
  int *error;
  {
   struct lhsParseNode *theNode;
   struct token theToken;
   struct expr *theExpression;

   /*================================================*/
   /* Create the data specification for the test CE. */
   /*================================================*/

   SavePPBuffer(" ");
   theNode = GetLHSParseNode();
   theNode->type = TEST_CE;
   theExpression = Function0Parse(readSource);
   theNode->expression = ExpressionToLHSParseNodes(theExpression);
   ReturnExpression(theExpression);

   if (theNode->expression == NULL)
     {
      *error = CLIPS_TRUE;
      ReturnLHSParseNodes(theNode);
      return(NULL);
     }

   /*=========================================================*/
   /* Check for the closing right parenthesis of the test CE. */
   /*=========================================================*/

   GetToken(readSource,&theToken);
   if (theToken.type != RPAREN)
     {
      SyntaxErrorMessage("test conditional element");
      *error = CLIPS_TRUE;
      ReturnLHSParseNodes(theNode);
      return(NULL);
     }

   return(theNode);
  }

/****************************************************************/
/* AssignmentParse: Finishes the parsing of pattern conditional */
/*   elements that have been bound to a  variable.              */
/*                                                              */
/* <assigned-pattern-CE> ::= ?<variable-symbol> <- <pattern-CE> */
/****************************************************************/
static struct lhsParseNode *AssignmentParse(readSource,factAddress,error)
  char *readSource;
  SYMBOL_HN *factAddress;
  int *error;
  {
   struct lhsParseNode *theNode;
   struct token theToken;
   
   /*=====================================================*/
   /* Patterns cannot be bound if they are with a not CE. */
   /*=====================================================*/
   
   if (WithinNotCE)
     {
      PrintErrorID("RULELHS",2,CLIPS_TRUE);
      PrintCLIPS(WERROR,"A pattern CE cannot be bound to a pattern-address within a not CE\n");
      *error = CLIPS_TRUE;
      return(NULL);
     }

   /*===============================================*/
   /* Check for binder token, "<-", after variable. */
   /*===============================================*/

   SavePPBuffer(" ");
   
   GetToken(readSource,&theToken);

   if ((theToken.type == SYMBOL) ? (strcmp(ValueToString(theToken.value),"<-") != 0) :
                                   CLIPS_TRUE)
     {
      SyntaxErrorMessage("binding patterns"); 
      *error = CLIPS_TRUE;
      return(NULL);
     }

   SavePPBuffer(" ");

   /*================================================*/
   /* Check for opening left parenthesis of pattern. */
   /*================================================*/

   GetToken(readSource,&theToken);
   if (theToken.type != LPAREN)
     {
      SyntaxErrorMessage("binding patterns");
      *error = CLIPS_TRUE;
      return(NULL);
     }

   /*======================================================*/
   /* Parse the pattern and return the data specification. */
   /*======================================================*/

   GetToken(readSource,&theToken);

   theNode = SimplePatternParse(readSource,&theToken,error);
   
   if (*error == CLIPS_TRUE)
     {
      ReturnLHSParseNodes(theNode);
      return(NULL);
     }

   /*=============================================*/
   /* Store the name of the variable to which the */
   /* pattern is bound and return the pattern.    */
   /*=============================================*/
   
   theNode->value = (VOID *) factAddress;
   return(theNode);
  }
  
/*****************************************************************/
/* TagLHSLogicalNodes: Marks all and and or conditional elements */
/*   contained within a logical conditional element as having    */
/*   the properties associated with a logical CE.                */
/*****************************************************************/
static VOID TagLHSLogicalNodes(nodePtr)
  struct lhsParseNode *nodePtr;
  {
   while (nodePtr != NULL)
     {
      nodePtr->logical = CLIPS_TRUE;
      if ((nodePtr->type == AND_CE) || 
          (nodePtr->type == OR_CE) ||
          (nodePtr->type == NOT_CE))
        { TagLHSLogicalNodes(nodePtr->right); }
      nodePtr = nodePtr->bottom;
     }
  }
  
/***********************************************************/
/* SimplePatternParse: Parses a simple pattern (an opening */
/*   parenthesis followed by one or more fields followed   */
/*   by a closing parenthesis).                            */
/*                                                         */
/* <pattern-CE> ::= <ordered-pattern-CE> |                 */
/*                  <template-pattern-CE>                  */
/***********************************************************/
static struct lhsParseNode *SimplePatternParse(readSource,theToken,error)
  char *readSource;
  struct token *theToken;
  int *error;
  {
   struct lhsParseNode *theNode;
   struct patternParser *tempParser;

   if (theToken->type != SYMBOL)
     {
      SyntaxErrorMessage("the first field of a pattern");
      *error = CLIPS_TRUE;
      return(NULL);
     }
   else if ((strcmp(ValueToString(theToken->value),"=") == 0) ||
            (strcmp(ValueToString(theToken->value),":") == 0))
     {
      SyntaxErrorMessage("the field field of a pattern");
      *error = CLIPS_TRUE;
      return(NULL);
     }
     
   theNode = GetLHSParseNode();
   theNode->type = PATTERN_CE;
   theNode->negated = CLIPS_FALSE;
   
   /*=================================================*/
   /* Check for types of patterns other than ordered. */
   /*=================================================*/

   tempParser = ListOfPatternParsers;
   while (tempParser != NULL)
     {
      if ((*tempParser->recognizeFunction)(theToken->value))
        {
         theNode->patternType = tempParser;
         theNode->right = (*tempParser->parseFunction)(readSource,theToken);
         if (theNode->right == NULL)
           {
            *error = CLIPS_TRUE;
            ReturnLHSParseNodes(theNode);
            return(NULL);
           }
    
         PropagatePatternType(theNode,tempParser);
         return(theNode);
        }
      tempParser = tempParser->next;
     }
     
   *error = CLIPS_TRUE;
   SyntaxErrorMessage("the field field of a pattern");
   ReturnLHSParseNodes(theNode);
   return(NULL);
  }
  
/**************************************************************/
/* PropagatePatternType: Sets the selfPattern field for all   */
/*   lhsParseNodes in a linked list of those data structures. */
/**************************************************************/
globle VOID PropagatePatternType(theLHS,theParser)
  struct lhsParseNode *theLHS;
  struct patternParser *theParser;
  {
   while (theLHS != NULL)
     {
      theLHS->patternType = theParser;
      if (theLHS->right != NULL) PropagatePatternType(theLHS->right,theParser);
      if (theLHS->expression != NULL) PropagatePatternType(theLHS->expression,theParser);
      theLHS = theLHS->bottom;
     }
  } 
  
#endif


