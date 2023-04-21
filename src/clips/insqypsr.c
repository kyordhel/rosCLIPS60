   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*          INSTANCE-SET QUERIES PARSER MODULE         */
   /*******************************************************/

/*************************************************************/
/* Purpose: Instance_set Queries Parsing Routines            */
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

#if INSTANCE_SET_QUERIES && (! RUN_TIME)

#if ANSI_COMPILER
#include <string.h>
#endif

#include "classcom.h"
#include "exprnpsr.h"
#include "extnfunc.h"
#include "insquery.h"
#include "prcdrpsr.h"
#include "prntutil.h"
#include "router.h"
#include "scanner.h"
#include "strngrtr.h"

#define _INSQYPSR_SOURCE_
#include "insqypsr.h"

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */
#define INSTANCE_SLOT_REF ':'

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

static EXPRESSION *ParseQueryRestrictions(EXPRESSION *,char *,struct token *);
static BOOLEAN ReplaceClassNameWithReference(EXPRESSION *);
static int ParseQueryTestExpression(EXPRESSION *,char *);
static int ParseQueryActionExpression(EXPRESSION *,char *,EXPRESSION *);
static VOID ReplaceInstanceVariables(EXPRESSION *,EXPRESSION *,int,int);
static VOID ReplaceSlotReference(EXPRESSION *,EXPRESSION *,
                                 struct FunctionDefinition *,int);
static int IsQueryFunction(EXPRESSION *);

#else

static EXPRESSION *ParseQueryRestrictions();
static BOOLEAN ReplaceClassNameWithReference();
static int ParseQueryTestExpression();
static int ParseQueryActionExpression();
static VOID ReplaceInstanceVariables();
static VOID ReplaceSlotReference();
static int IsQueryFunction();

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

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

/***********************************************************************
  NAME         : ParseQueryNoAction
  DESCRIPTION  : Parses the following functions :
                   (any-instancep)
                   (find-first-instance)
                   (find-all-instances)
  INPUTS       : 1) The address of the top node of the query function
                 2) The logical name of the input
  RETURNS      : The completed expression chain, or NULL on errors
  SIDE EFFECTS : The expression chain is extended, or the "top" node
                 is deleted on errors
  NOTES        : CLIPS Syntax : 
  
                 (<function> <query-block>)
                 
                 <query-block>  :== (<instance-var>+) <query-expression>
                 <instance-var> :== (<var-name> <class-name>+)
                 
                 Parses into following form :
                 
                 <query-function>
                      |
                      V
                 <query-expression>  ->
                              
                 <class-1a> -> <class-1b> -> (QDS) ->
                 
                 <class-2a> -> <class-2b> -> (QDS) -> ... 
 ***********************************************************************/
globle EXPRESSION *ParseQueryNoAction(top,readSource)
  EXPRESSION *top;
  char *readSource;
  {
   EXPRESSION *insQuerySetVars;
   struct token queryInputToken;

   insQuerySetVars = ParseQueryRestrictions(top,readSource,&queryInputToken);
   if (insQuerySetVars == NULL)
     return(NULL);
   IncrementIndentDepth(3);
   PPCRAndIndent();
   if (ParseQueryTestExpression(top,readSource) == CLIPS_FALSE)
     {
      DecrementIndentDepth(3);
      ReturnExpression(insQuerySetVars);
      return(NULL);
     }
   DecrementIndentDepth(3);
   GetToken(readSource,&queryInputToken);
   if (GetType(queryInputToken) != RPAREN)      
     {
      SyntaxErrorMessage("instance-set query function");
      ReturnExpression(top);
      ReturnExpression(insQuerySetVars);
      return(NULL);
     }
   ReplaceInstanceVariables(insQuerySetVars,top->argList,CLIPS_TRUE,0);
   ReturnExpression(insQuerySetVars);
   return(top);
  }
  
/***********************************************************************
  NAME         : ParseQueryAction
  DESCRIPTION  : Parses the following functions :
                   (do-for-instance)
                   (do-for-all-instances)
                   (delayed-do-for-all-instances)
  INPUTS       : 1) The address of the top node of the query function
                 2) The logical name of the input
  RETURNS      : The completed expression chain, or NULL on errors
  SIDE EFFECTS : The expression chain is extended, or the "top" node
                 is deleted on errors
  NOTES        : CLIPS Syntax : 
  
                 (<function> <query-block> <query-action>)
                 
                 <query-block>  :== (<instance-var>+) <query-expression>
                 <instance-var> :== (<var-name> <class-name>+)
                 
                 Parses into following form :
                 
                 <query-function>
                      |
                      V
                 <query-expression> -> <query-action>  ->
                              
                 <class-1a> -> <class-1b> -> (QDS) ->
                 
                 <class-2a> -> <class-2b> -> (QDS) -> ... 
 ***********************************************************************/
globle EXPRESSION *ParseQueryAction(top,readSource)
  EXPRESSION *top;
  char *readSource;
  {
   EXPRESSION *insQuerySetVars;
   struct token queryInputToken;
   
   insQuerySetVars = ParseQueryRestrictions(top,readSource,&queryInputToken);
   if (insQuerySetVars == NULL)
     return(NULL);
   IncrementIndentDepth(3);
   PPCRAndIndent();
   if (ParseQueryTestExpression(top,readSource) == CLIPS_FALSE)
     {
      DecrementIndentDepth(3);
      ReturnExpression(insQuerySetVars);
      return(NULL);
     }
   PPCRAndIndent();
   if (ParseQueryActionExpression(top,readSource,insQuerySetVars) == CLIPS_FALSE)
     {
      DecrementIndentDepth(3);
      ReturnExpression(insQuerySetVars);
      return(NULL);
     }
   DecrementIndentDepth(3);
   GetToken(readSource,&queryInputToken);
   if (GetType(queryInputToken) != RPAREN)      
     {
      SyntaxErrorMessage("instance-set query function");
      ReturnExpression(top);
      ReturnExpression(insQuerySetVars);
      return(NULL);
     }
   ReplaceInstanceVariables(insQuerySetVars,top->argList,CLIPS_TRUE,0);
   ReplaceInstanceVariables(insQuerySetVars,top->argList->nextArg,CLIPS_FALSE,0);
   ReturnExpression(insQuerySetVars);
   return(top);
  }

/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/***************************************************************
  NAME         : ParseQueryRestrictions
  DESCRIPTION  : Parses the class restrictions for a query
  INPUTS       : 1) The top node of the query expression
                 2) The logical name of the input
                 3) Caller's token buffer
  RETURNS      : The instance-variable expressions
  SIDE EFFECTS : Entire query expression deleted on errors
                 Nodes allocated for restrictions and instance
                   variable expressions
                 Class restrictions attached to query-expression
                   as arguments
  NOTES        : Expects top != NULL
 ***************************************************************/
static EXPRESSION *ParseQueryRestrictions(top,readSource,queryInputToken)
  EXPRESSION *top;
  char *readSource;
  struct token *queryInputToken;
  {
   EXPRESSION *insQuerySetVars = NULL,*lastInsQuerySetVars = NULL,
              *classExp = NULL,*lastClassExp,
              *tmp,*lastOne = NULL;
   int error = CLIPS_FALSE;

   SavePPBuffer(" ");
   GetToken(readSource,queryInputToken);
   if (queryInputToken->type != LPAREN)
     goto ParseQueryRestrictionsError1;
   GetToken(readSource,queryInputToken);
   if (queryInputToken->type != LPAREN)
     goto ParseQueryRestrictionsError1;
   while (queryInputToken->type == LPAREN)
     {
      GetToken(readSource,queryInputToken);
      if (queryInputToken->type != SF_VARIABLE)
        goto ParseQueryRestrictionsError1;
      tmp = insQuerySetVars;
      while (tmp != NULL)
        {
         if (tmp->value == queryInputToken->value)
           {
            PrintErrorID("INSQYPSR",1,CLIPS_FALSE);
            PrintCLIPS(WERROR,"Duplicate instance member variable name in function ");
            PrintCLIPS(WERROR,ValueToString(ExpressionFunctionCallName(top)));
            PrintCLIPS(WERROR,".\n");
            goto ParseQueryRestrictionsError2;
           }
         tmp = tmp->nextArg;
        }
      tmp = GenConstant(SF_VARIABLE,queryInputToken->value);
      if (insQuerySetVars == NULL)
        insQuerySetVars = tmp;
      else
        lastInsQuerySetVars->nextArg = tmp;
      lastInsQuerySetVars = tmp;
      SavePPBuffer(" ");
      classExp = ArgumentParse(readSource,&error);
      if (error)
        goto ParseQueryRestrictionsError2;
      if (classExp == NULL)
        goto ParseQueryRestrictionsError1;
      if (ReplaceClassNameWithReference(classExp) == CLIPS_FALSE)
        goto ParseQueryRestrictionsError2;
      lastClassExp = classExp;
      SavePPBuffer(" ");
      while ((tmp = ArgumentParse(readSource,&error)) != NULL)
        {
         if (ReplaceClassNameWithReference(tmp) == CLIPS_FALSE)
           goto ParseQueryRestrictionsError2;
         lastClassExp->nextArg = tmp;
         lastClassExp = tmp;
         SavePPBuffer(" ");
        }
      if (error)
        goto ParseQueryRestrictionsError2;
      PPBackup();
      PPBackup();
      SavePPBuffer(")");
      tmp = GenConstant(SYMBOL,(VOID *) QUERY_DELIMETER_SYMBOL);
      lastClassExp->nextArg = tmp;
      lastClassExp = tmp;
      if (top->argList == NULL)
        top->argList = classExp;
      else
        lastOne->nextArg = classExp;
      lastOne = lastClassExp;
      classExp = NULL;
      SavePPBuffer(" ");
      GetToken(readSource,queryInputToken);
     }
   if (queryInputToken->type != RPAREN)
     goto ParseQueryRestrictionsError1;
   PPBackup();
   PPBackup();
   SavePPBuffer(")");
   return(insQuerySetVars);
   
ParseQueryRestrictionsError1:
   SyntaxErrorMessage("instance-set query function");

ParseQueryRestrictionsError2:
   ReturnExpression(classExp);
   ReturnExpression(top);
   ReturnExpression(insQuerySetVars);
   return(NULL);
  }
  
/***************************************************
  NAME         : ReplaceClassNameWithReference
  DESCRIPTION  : In parsing an instance-set query,
                 this function replaces a constant
                 class name with an actual pointer
                 to the class
  INPUTS       : The expression
  RETURNS      : CLIPS_TRUE if all OK, CLIPS_FALSE
                 if class cannot be found
  SIDE EFFECTS : The expression type and value are
                 modified if class is found
  NOTES        : Searches current and imported
                 modules for reference
 ***************************************************/
static BOOLEAN ReplaceClassNameWithReference(exp)
  EXPRESSION *exp;
  {
   char *theClassName;
   VOID *theDefclass;
   
   if (exp->type == SYMBOL)
     {
      theClassName = ValueToString(exp->value);
      theDefclass = (VOID *) LookupDefclassByMdlOrScope(theClassName);
      if (theDefclass == NULL)
        {
         CantFindItemErrorMessage("class",theClassName);
         return(CLIPS_FALSE);
        }
      exp->type = DEFCLASS_PTR;
      exp->value = theDefclass;
     }
   return(CLIPS_TRUE);
  }

/*************************************************************
  NAME         : ParseQueryTestExpression
  DESCRIPTION  : Parses the test-expression for a query
  INPUTS       : 1) The top node of the query expression
                 2) The logical name of the input
  RETURNS      : CLIPS_TRUE if all OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : Entire query-expression deleted on errors
                 Nodes allocated for new expression
                 Test shoved in front of class-restrictions on
                    query argument list
  NOTES        : Expects top != NULL
 *************************************************************/
static int ParseQueryTestExpression(top,readSource)
  EXPRESSION *top;
  char *readSource;
  {
   EXPRESSION *qtest;
   int error;
   struct BindInfo *oldBindList;
   
   error = CLIPS_FALSE;
   oldBindList = GetParsedBindNames();
   SetParsedBindNames(NULL);
   qtest = ArgumentParse(readSource,&error);
   if (error == CLIPS_TRUE)
     {
      SetParsedBindNames(oldBindList);
      ReturnExpression(top);
      return(CLIPS_FALSE);
     }
   if (qtest == NULL)
     {
      SetParsedBindNames(oldBindList);
      SyntaxErrorMessage("instance-set query function");
      ReturnExpression(top);
      return(CLIPS_FALSE);
     }
   qtest->nextArg = top->argList;
   top->argList = qtest;
   if (ParsedBindNamesEmpty() == CLIPS_FALSE)
     {
      ClearParsedBindNames();
      SetParsedBindNames(oldBindList);
      PrintErrorID("INSQYPSR",2,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Binds are not allowed in instance-set query in function ");
      PrintCLIPS(WERROR,ValueToString(ExpressionFunctionCallName(top)));
      PrintCLIPS(WERROR,".\n");
      ReturnExpression(top);
      return(CLIPS_FALSE);
     }
   SetParsedBindNames(oldBindList);
   return(CLIPS_TRUE);
  }
  
/*************************************************************
  NAME         : ParseQueryActionExpression
  DESCRIPTION  : Parses the action-expression for a query
  INPUTS       : 1) The top node of the query expression
                 2) The logical name of the input
                 3) List of query parameters
  RETURNS      : CLIPS_TRUE if all OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : Entire query-expression deleted on errors
                 Nodes allocated for new expression
                 Action shoved in front of class-restrictions
                    and in back of test-expression on query
                    argument list
  NOTES        : Expects top != NULL && top->argList != NULL
 *************************************************************/
static int ParseQueryActionExpression(top,readSource,insQuerySetVars)
  EXPRESSION *top;
  char *readSource;
  EXPRESSION *insQuerySetVars;
  {
   EXPRESSION *qaction,*tmpInsSetVars;
   int error;
   struct BindInfo *oldBindList,*newBindList,*prev;
   
   error = CLIPS_FALSE;
   oldBindList = GetParsedBindNames();
   SetParsedBindNames(NULL);
   BreakContext = CLIPS_TRUE;
   ReturnContext = svContexts->rtn;
   qaction = ArgumentParse(readSource,&error);
   BreakContext = CLIPS_FALSE;
   if (error == CLIPS_TRUE)
     {
      SetParsedBindNames(oldBindList);
      ReturnExpression(top);
      return(CLIPS_FALSE);
     }
   if (qaction == NULL)
     {
      SetParsedBindNames(oldBindList);
      SyntaxErrorMessage("instance-set query function");
      ReturnExpression(top);
      return(CLIPS_FALSE);
     }
   qaction->nextArg = top->argList->nextArg;
   top->argList->nextArg = qaction;
   newBindList = GetParsedBindNames();
   prev = NULL;
   while (newBindList != NULL)
     {
      tmpInsSetVars = insQuerySetVars;
      while (tmpInsSetVars != NULL)
        {
         if (tmpInsSetVars->value == (VOID *) newBindList->name)
           {
            ClearParsedBindNames();
            SetParsedBindNames(oldBindList);
            PrintErrorID("INSQYPSR",3,CLIPS_FALSE);
            PrintCLIPS(WERROR,"Cannot rebind instance-set member variable ");
            PrintCLIPS(WERROR,ValueToString(tmpInsSetVars->value));
            PrintCLIPS(WERROR," in function ");
            PrintCLIPS(WERROR,ValueToString(ExpressionFunctionCallName(top)));
            PrintCLIPS(WERROR,".\n");
            ReturnExpression(top);
            return(CLIPS_FALSE);
           }
         tmpInsSetVars = tmpInsSetVars->nextArg;
        }
      prev = newBindList;
      newBindList = newBindList->next;
     }
   if (prev == NULL)
     SetParsedBindNames(oldBindList);
   else
     prev->next = oldBindList;
   return(CLIPS_TRUE);
  }
  
/***********************************************************************************
  NAME         : ReplaceInstanceVariables
  DESCRIPTION  : Replaces all references to instance-variables within an
                   instance query-function with function calls to query-instance
                   (which references the instance array at run-time)
  INPUTS       : 1) The instance-variable list
                 2) A boolean expression containing variable references
                 3) A flag indicating whether to allow slot references of the type
                    <instance-query-variable>:<slot-name> for direct slot access
                    or not
                 4) Nesting depth of query functions
  RETURNS      : Nothing useful
  SIDE EFFECTS : If a SF_VARIABLE node is found and is on the list of instance
                   variables, it is replaced with a query-instance function call.
  NOTES        : Other SF_VARIABLE(S) are left alone for replacement by other
                   parsers.  This implies that a user may use defgeneric,
                   defrule, and defmessage-handler variables within a query-function
                   where they do not conflict with instance-variable names.
 ***********************************************************************************/
static VOID ReplaceInstanceVariables(vlist,bexp,sdirect,ndepth)
  EXPRESSION *vlist,*bexp;
  int sdirect,ndepth;
  {
   EXPRESSION *eptr;
   struct FunctionDefinition *rindx_func,*rslot_func;
   int posn;
   
   rindx_func = FindFunction("(query-instance)");
   rslot_func = FindFunction("(query-instance-slot)");
   while (bexp != NULL)
     {
      if (bexp->type == SF_VARIABLE)
        {
         eptr = vlist;
         posn = 0;
         while ((eptr != NULL) ? (eptr->value != bexp->value) : CLIPS_FALSE)
           {
            eptr = eptr->nextArg;
            posn++;
           }
         if (eptr != NULL)
           {
            bexp->type = FCALL;
            bexp->value = (VOID *) rindx_func;
            eptr = GenConstant(INTEGER,(VOID *) AddLong((long) ndepth));
            eptr->nextArg = GenConstant(INTEGER,(VOID *) AddLong((long) posn));
            bexp->argList = eptr;
           }
         else if (sdirect == CLIPS_TRUE)
           ReplaceSlotReference(vlist,bexp,rslot_func,ndepth);
        }
      if (bexp->argList != NULL)
        {
         if (IsQueryFunction(bexp))
           ReplaceInstanceVariables(vlist,bexp->argList,sdirect,ndepth+1);
         else
           ReplaceInstanceVariables(vlist,bexp->argList,sdirect,ndepth);
        }
      bexp = bexp->nextArg;
     }
  }

/*************************************************************************
  NAME         : ReplaceSlotReference
  DESCRIPTION  : Replaces instance-set query function variable
                   references of the form: <instance-variable>:<slot-name>
                   with function calls to get these instance-slots at run
                   time
  INPUTS       : 1) The instance-set variable list
                 2) The expression containing the variable
                 3) The address of the instance slot access function
                 4) Nesting depth of query functions
  RETURNS      : Nothing useful
  SIDE EFFECTS : If the variable is a slot reference, then it is replaced
                   with the appropriate function-call.
  NOTES        : None
 *************************************************************************/
static VOID ReplaceSlotReference(vlist,exp,func,ndepth)
  EXPRESSION *vlist,*exp;
  struct FunctionDefinition *func;
  int ndepth;
  {
   int len,posn,oldpp;
   register int i;
   register char *str;
   EXPRESSION *eptr;
   struct token itkn;
   
   str = ValueToString(exp->value);
   len =  strlen(str);
   if (len < 3)
     return;
   for (i = len-1 ; i >= 1 ; i--)
     {
      if ((str[i] == INSTANCE_SLOT_REF) ? (i >= 1) : CLIPS_FALSE)
        {
         eptr = vlist;
         posn = 0;
         while ((eptr == NULL) ? CLIPS_FALSE : 
                (strncmp(ValueToString(eptr->value),str,i) != 0))
           {
            eptr = eptr->nextArg;
            posn++;
           }
         if (eptr != NULL)
           {
            OpenStringSource("query-var",str+i+1,0);
            oldpp = GetPPBufferStatus();
            SetPPBufferStatus(OFF);
            GetToken("query-var",&itkn);
            SetPPBufferStatus(oldpp);
            CloseStringSource("query-var");
            exp->type = FCALL;
            exp->value = (VOID *) func;
            exp->argList = GenConstant(INTEGER,(VOID *) AddLong((long) ndepth));
            exp->argList->nextArg = 
              GenConstant(INTEGER,(VOID *) AddLong((long) posn));
            exp->argList->nextArg->nextArg = GenConstant(itkn.type,itkn.value);
            break;
           }
        }
     }
  }

/********************************************************************
  NAME         : IsQueryFunction
  DESCRIPTION  : Determines if an expression is a query function call
  INPUTS       : The expression
  RETURNS      : CLIPS_TRUE if query function call, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ********************************************************************/
static int IsQueryFunction(exp)
  EXPRESSION *exp;
  {
   int (*fptr)(VOID_ARG);
   
   if (exp->type != FCALL)
     return(CLIPS_FALSE);
   fptr = (int (*)(VOID_ARG)) ExpressionFunctionPointer(exp);
   if (fptr == (int (*)(VOID_ARG)) AnyInstances)
     return(CLIPS_TRUE);
   if (fptr == (int (*)(VOID_ARG)) QueryFindInstance)
     return(CLIPS_TRUE);
   if (fptr == (int (*)(VOID_ARG)) QueryFindAllInstances)
     return(CLIPS_TRUE);
   if (fptr == (int (*)(VOID_ARG)) QueryDoForInstance)
     return(CLIPS_TRUE);
   if (fptr == (int (*)(VOID_ARG)) QueryDoForAllInstances)
     return(CLIPS_TRUE);
   if (fptr == (int (*)(VOID_ARG)) DelayedQueryDoForAllInstances)
     return(CLIPS_TRUE);
   return(CLIPS_FALSE);
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


