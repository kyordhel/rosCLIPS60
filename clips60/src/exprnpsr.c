   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*              EXPRESSION PARSER MODULE               */
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

#define _EXPRNPSR_SOURCE_

#include "setup.h"

#include <stdio.h>
#define _CLIPS_STDIO_
#if ANSI_COMPILER
#include <stdlib.h>
#endif
#include <string.h>
#include <ctype.h>

#include "constant.h"
#include "router.h"
#include "strngrtr.h"
#include "scanner.h"
#include "clipsmem.h"
#include "argacces.h"
#include "prntutil.h"
#include "cstrnchk.h"
#include "extnfunc.h"
#include "exprnpsr.h"
#include "modulutl.h"

#if DEFRULE_CONSTRUCT
#include "network.h"
#endif

#if DEFGENERIC_CONSTRUCT
#include "genrccom.h"
#endif

#if DEFFUNCTION_CONSTRUCT
#include "dffnxfun.h"
#endif

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
#if (! RUN_TIME)
   static int                     CheckExpressionAgainstRestrictions(struct expr *,char *,char *);
#endif
#else
#if (! RUN_TIME)
   static int                     CheckExpressionAgainstRestrictions();
#endif
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

#if (! RUN_TIME)
   globle SAVED_CONTEXTS      *svContexts = NULL;
   globle int                  ReturnContext;
   globle int                  BreakContext;
#endif
   globle BOOLEAN              SequenceOpMode = CLIPS_FALSE;

#if (! RUN_TIME)

/***************************************************************/
/* Function0Parse: Parses a function. Assumes that none of the */
/*   function has been parsed yet.                             */
/***************************************************************/
globle struct expr *Function0Parse(logicalName)
  char *logicalName;
  {
   struct token theToken;
   struct expr *top;

   /*=================================*/
   /* All functions begin with a '('. */
   /*=================================*/

   GetToken(logicalName,&theToken);
   if (theToken.type != LPAREN)
     {
      SyntaxErrorMessage("function calls");
      return(NULL);
     }

   /*=================================*/
   /* Parse the rest of the function. */
   /*=================================*/

   top = Function1Parse(logicalName);
   return(top);
  }

/*******************************************************/
/* Function1Parse: Parses a function. Assumes that the */
/*   opening left parenthesis has already been parsed. */
/*******************************************************/
globle struct expr *Function1Parse(logicalName)
  char *logicalName;
  {
   struct token theToken;
   struct expr *top;

   /*========================*/
   /* Get the function name. */
   /*========================*/

   GetToken(logicalName,&theToken);
   if (theToken.type != SYMBOL)
     {
      PrintErrorID("EXPRNPSR",1,CLIPS_TRUE);
      PrintCLIPS(WERROR,"A function name must be a symbol\n");
      return(NULL);
     }

   /*=================================*/
   /* Parse the rest of the function. */
   /*=================================*/

   top = Function2Parse(logicalName,ValueToString(theToken.value));
   return(top);
  }

/****************************************************/
/* Function2Parse: Parses a function. Assumes that  */
/*   the opening left parenthesis and function name */
/*   have already been parsed.                      */
/****************************************************/
globle struct expr *Function2Parse(logicalName,name)
  char *logicalName, *name;
  {
   struct FunctionDefinition *theFunction;
   struct expr *top;
#if DEFGENERIC_CONSTRUCT
   VOID *gfunc;
#endif
#if DEFFUNCTION_CONSTRUCT
   VOID *dptr;
#endif

   /*=========================================================*/
   /* Module specification cannot be used in a function call. */
   /*=========================================================*/
   
   if (FindModuleSeparator(name))
     {
      IllegalModuleSpecifierMessage();
      return(NULL);
     }
   
   /*================================*/
   /* Has the function been defined? */
   /*================================*/

   theFunction = FindFunction(name);

#if DEFGENERIC_CONSTRUCT
   gfunc = (VOID *) LookupDefgenericInScope(name);
#endif

#if DEFFUNCTION_CONSTRUCT
   if ((theFunction == NULL)
#if DEFGENERIC_CONSTRUCT
        && (gfunc == NULL)
#endif
     )
     dptr = (VOID *) LookupDeffunctionInScope(name);
   else
     dptr = NULL;
#endif

   /*=============================*/
   /* Define top level structure. */
   /*=============================*/

#if DEFFUNCTION_CONSTRUCT
   if (dptr != NULL)
     top = GenConstant(PCALL,dptr);
   else
#endif
#if DEFGENERIC_CONSTRUCT
   if (gfunc != NULL)
     top = GenConstant(GCALL,gfunc);
   else
#endif
   if (theFunction != NULL)
     top = GenConstant(FCALL,theFunction);
   else
     {      
      PrintErrorID("EXPRNPSR",3,CLIPS_TRUE);
      PrintCLIPS(WERROR,"Missing function declaration for ");
      PrintCLIPS(WERROR,name);
      PrintCLIPS(WERROR,".\n");
      return(NULL);
     }
      
   /*=======================================================*/
   /* Check to see if function has its own parsing routine. */
   /*=======================================================*/
   
   PushRtnBrkContexts();
   ReturnContext = CLIPS_FALSE;
   BreakContext = CLIPS_FALSE;
   
#if DEFGENERIC_CONSTRUCT || DEFFUNCTION_CONSTRUCT
   if (top->type == FCALL)
#endif
     {
      if (theFunction->parser != NULL)
        {
         top = (*theFunction->parser)(top,logicalName);
         PopRtnBrkContexts();
         if (top == NULL) return(NULL);
         if (ReplaceSequenceExpansionOps(top->argList,top,FindFunction("(expansion-call)"),
                                         FindFunction("expand$")))
           {
            ReturnExpression(top);
            return(NULL);
           }
         return(top);
        }
     }

   /*========================================*/
   /* Default parsing routine for functions. */
   /*========================================*/
   
   top = CollectArguments(top,logicalName);
   PopRtnBrkContexts();
   if (top == NULL) return(NULL);

   if (ReplaceSequenceExpansionOps(top->argList,top,FindFunction("(expansion-call)"),
                                    FindFunction("expand$")))
     {
      ReturnExpression(top);
      return(NULL);
     }
     
   /*============================================================*/
   /* If the function call uses the sequence expansion operator, */
   /* its arguments cannot be checked until runtime.             */
   /*============================================================*/
   
   if (top->value == (VOID *) FindFunction("(expansion-call)"))
     { return(top); }
     
   /*============================*/
   /* Check for argument errors. */
   /*============================*/

   if ((top->type == FCALL) && GetStaticConstraintChecking())
     {
      if (CheckExpressionAgainstRestrictions(top,theFunction->restrictions,name))
        {
         ReturnExpression(top);
         return(NULL);
        }
     }
     
#if DEFFUNCTION_CONSTRUCT 
   else if (top->type == PCALL)
     {
      if (CheckDeffunctionCall(top->value,CountArguments(top->argList)) == CLIPS_FALSE)
        {
         ReturnExpression(top);
         return(NULL);
        }
     }
#endif
        
   return(top);
  }

/***********************************************************************
  NAME         : ReplaceSequenceExpansionOps
  DESCRIPTION  : Replaces function calls which have multifield
                   references as arguments into a call to a 
                   special function which expands the multifield
                   into single arguments at run-time.
                 Multifield references which are not function
                   arguments are errors
  INPUTS       : 1) The expression
                 2) The current function call
                 3) The address of the internal CLIPS function
                    (expansion-call)
                 4) The address of the CLIPS function expand$
  RETURNS      : FALSE if OK, TRUE on errors
  SIDE EFFECTS : Function call expressions modified, if necessary
  NOTES        : Function calls which truly want a multifield
                   to be passed need use only a single-field
                   refernce (i.e. ? instead of $? - the $ is
                   being treated as a special expansion operator)
 **********************************************************************/
globle BOOLEAN ReplaceSequenceExpansionOps(actions,fcallexp,expcall,expmult)
  EXPRESSION *actions,*fcallexp;
  VOID *expcall,*expmult;
  {
   EXPRESSION *exp;
   
   while (actions != NULL)
     {
      if ((SequenceOpMode == CLIPS_FALSE) && (actions->type == MF_VARIABLE))
        actions->type = SF_VARIABLE;
      if ((actions->type == MF_VARIABLE) || (actions->type == MF_GBL_VARIABLE) ||
          (actions->value == expmult))
        {
         if ((fcallexp->type != FCALL) ? CLIPS_FALSE :
             (((struct FunctionDefinition *) fcallexp->value)->sequenceuseok == CLIPS_FALSE))
           {       
            PrintErrorID("EXPRNPSR",4,CLIPS_FALSE);
            PrintCLIPS(WERROR,"$ Sequence operator not a valid argument for ");
            PrintCLIPS(WERROR,ValueToString(((struct FunctionDefinition *) 
                              fcallexp->value)->callFunctionName));
            PrintCLIPS(WERROR,".\n");
            return(TRUE);
           }
         if (fcallexp->value != expcall)
           {
            exp = GenConstant(fcallexp->type,fcallexp->value);
            exp->argList = fcallexp->argList;
            exp->nextArg = NULL;
            fcallexp->type = FCALL;
            fcallexp->value = expcall;
            fcallexp->argList = exp;
           }
         if (actions->value != expmult)
           {
            exp = GenConstant(SF_VARIABLE,actions->value);
            if (actions->type == MF_GBL_VARIABLE)
              exp->type = GBL_VARIABLE;
            actions->argList = exp;
            actions->type = FCALL;
            actions->value = expmult;
           }
        }
      if (actions->argList != NULL)
        {
         if ((actions->type == GCALL) ||
             (actions->type == PCALL) ||
             (actions->type == FCALL))
           exp = actions;
         else
           exp = fcallexp;
         if (ReplaceSequenceExpansionOps(actions->argList,exp,expcall,expmult))
           return(TRUE);
        }
      actions = actions->nextArg;
     }
   return(FALSE);
  }
  
/*************************************/
/* PushRtnBrkContexts:               */
/*************************************/
globle VOID PushRtnBrkContexts()
  {
   SAVED_CONTEXTS *svtmp;
   
   svtmp = get_struct(saved_contexts);
   svtmp->rtn = ReturnContext;
   svtmp->brk = BreakContext;
   svtmp->nxt = svContexts;
   svContexts = svtmp;
  }
  
/*************************************/
/* PopRtnBrkContexts:                */
/*************************************/
globle VOID PopRtnBrkContexts()
  {
   SAVED_CONTEXTS *svtmp;
   
   ReturnContext = svContexts->rtn;
   BreakContext = svContexts->brk;
   svtmp = svContexts;
   svContexts = svContexts->nxt;
   rtn_struct(saved_contexts,svtmp);
  }

/*****************************************************************/
/* CheckExpressionAgainstRestrictions: Compares the arguments to */
/*   a function to the set of restrictions for that function to  */
/*   determine if any incompatibilities exist. If so, the value  */
/*   TRUE is returned, otherwise FALSE is returned.              */
/*****************************************************************/
static int CheckExpressionAgainstRestrictions(theExpression,restrictions,functionName)
  struct expr *theExpression;
  char *restrictions;
  char *functionName;
  {
   char theChar[2];
   int i = 0, j = 1;
   int number1, number2;
   int argCount;
   char defaultRestriction, argRestriction;
   struct expr *argPtr;
   int theRestriction;

   theChar[0] = '0';
   theChar[1] = '\0';
   
   if (restrictions == NULL) return(CLIPS_FALSE);

   argCount = CountArguments(theExpression->argList);

   /*======================================*/
   /* Get the minimum number of arguments. */
   /*======================================*/

   theChar[0] = restrictions[i++];

   if (isdigit(theChar[0]))
     { number1 = atoi(theChar); }
   else if (theChar[0] == '*')
     { number1 = -1; }
   else
     { return(CLIPS_FALSE); }

   /*======================================*/
   /* Get the maximum number of arguments. */
   /*======================================*/

   theChar[0] = restrictions[i++];
   if (isdigit(theChar[0]))
     { number2 = atoi(theChar); }
   else if (theChar[0] == '*')
     { number2 = 10000; }
   else
     { return(CLIPS_FALSE); }

   /*============================================*/
   /* Check for the correct number of arguments. */
   /*============================================*/

   if (number1 == number2)
     {
      if (argCount != number1)
        {
         ExpectedCountError(functionName,EXACTLY,number1);
         return(CLIPS_TRUE);
        }
     }
   else if (argCount < number1)
     {
      ExpectedCountError(functionName,AT_LEAST,number1);
      return(CLIPS_TRUE);
     }
   else if (argCount > number2)
     {
      ExpectedCountError(functionName,NO_MORE_THAN,number2);
      return(CLIPS_TRUE);
     }

   /*=======================================*/
   /* Check for the default argument types. */
   /*=======================================*/

   defaultRestriction = restrictions[i];
   if (defaultRestriction == '\0')
     { defaultRestriction = 'u'; }
   else if (defaultRestriction == '*')
     { 
      defaultRestriction = 'u';
      i++;
     }
   else
     { i++; }

   /*======================*/
   /* Check each argument. */
   /*======================*/

   argPtr = theExpression->argList;
   while (argPtr != NULL)
     {
      argRestriction = restrictions[i];
      if (argRestriction == '\0')
        { argRestriction = defaultRestriction; }
      else
        { i++; }
        
      if (argRestriction != '*')
        { theRestriction = (int) argRestriction; }
      else
        { theRestriction = (int) defaultRestriction; }

      if (CheckArgumentAgainstRestriction(argPtr,theRestriction))
        { 
         ExpectedTypeError1(functionName,j,GetArgumentTypeName(theRestriction)); 
         return(CLIPS_TRUE);
        }

      j++;
      argPtr = argPtr->nextArg;
     }

   return(CLIPS_FALSE);
  }
  
/*******************************************************/
/* CollectArguments: Parses and groups together all of */
/*   the arguments for a function call expression.     */
/*******************************************************/
globle struct expr *CollectArguments(top,logicalName)
  struct expr *top;
  char *logicalName;
  {
   int errorFlag;
   struct expr *lastOne, *nextOne;

   /*========================================*/
   /* Default parsing routine for functions. */
   /*========================================*/

   lastOne = NULL;

   while (CLIPS_TRUE)
     {
      SavePPBuffer(" ");

      errorFlag = CLIPS_FALSE;
      nextOne = ArgumentParse(logicalName,&errorFlag);

      if (errorFlag == CLIPS_TRUE)
        {
         ReturnExpression(top);
         return(NULL);
        }

      if (nextOne == NULL)
        {
         PPBackup();
         PPBackup();
         SavePPBuffer(")");
         return(top);
        }

      if (lastOne == NULL)
        { top->argList = nextOne; }
      else
        { lastOne->nextArg = nextOne; }

      lastOne = nextOne;
     }
  }

/************************************************************************/
/* ArgumentParse: Parses an argument within a function call expression. */
/************************************************************************/
globle struct expr *ArgumentParse(logicalName,errorFlag)
  char *logicalName;
  int *errorFlag;
  {
   struct expr *top;
   struct token theToken;

   GetToken(logicalName,&theToken);

   /*============================*/
   /* ')' counts as no argument. */
   /*============================*/

   if (theToken.type == RPAREN)
     { return(NULL); }

   /*================================*/
   /* Parse constants and variables. */
   /*================================*/

   if ((theToken.type == SF_VARIABLE) || (theToken.type == MF_VARIABLE) ||
       (theToken.type == SYMBOL) || (theToken.type == STRING) ||
#if DEFGLOBAL_CONSTRUCT
       (theToken.type == GBL_VARIABLE) ||
       (theToken.type == MF_GBL_VARIABLE) ||
#endif
#if OBJECT_SYSTEM
       (theToken.type == INSTANCE_NAME) ||
#endif
       (theToken.type == FLOAT) || (theToken.type == INTEGER))
     { return(GenConstant(theToken.type,theToken.value)); }

   /*======================*/
   /* Parse function call. */
   /*======================*/

   if (theToken.type != LPAREN)
     {   
      PrintErrorID("EXPRNPSR",2,CLIPS_TRUE);
      PrintCLIPS(WERROR,"Expected a constant, variable, or expression.\n");
      *errorFlag = CLIPS_TRUE;
      return(NULL);
     }

   top = Function1Parse(logicalName);
   if (top == NULL) *errorFlag = CLIPS_TRUE;
   return(top);
  }

/************************************************************/
/* ParseAtomOrExpression: Parses an expression which may be */
/*   a function call, atomic value (string, symbol, etc.),  */
/*   or variable (local or global).                         */
/************************************************************/
globle struct expr *ParseAtomOrExpression(logicalName,useToken)
  char *logicalName;
  struct token *useToken;
  {
   struct token theToken, *thisToken;
   struct expr *rv;
   
   if (useToken == NULL)
     {
      thisToken = &theToken;
      GetToken(logicalName,thisToken);
     }
   else thisToken = useToken;

   if ((thisToken->type == SYMBOL) || (thisToken->type == STRING) ||
       (thisToken->type == INTEGER) || (thisToken->type == FLOAT) ||
#if OBJECT_SYSTEM
       (thisToken->type == INSTANCE_NAME) ||
#endif
#if DEFGLOBAL_CONSTRUCT
       (thisToken->type == GBL_VARIABLE) ||
       (thisToken->type == MF_GBL_VARIABLE) ||
#endif
       (thisToken->type == SF_VARIABLE) || (thisToken->type == MF_VARIABLE))
     { rv = GenConstant(thisToken->type,thisToken->value); }
   else if (thisToken->type == LPAREN)
     {
      rv = Function1Parse(logicalName);
      if (rv == NULL) return(NULL);
     }
   else
     {
      PrintErrorID("EXPRNPSR",2,CLIPS_TRUE);
      PrintCLIPS(WERROR,"Expected a constant, variable, or expression.\n");
      return(NULL);
     }

   return(rv);
  }

/****************************************************************/
/* GroupActions: Returns progn test with actions if successful. */
/*   Returns Null if error detected.                            */
/****************************************************************/
globle struct expr *GroupActions(logicalName,rtn_tkn,readFirstToken,endWord)
  char *logicalName;
  struct token *rtn_tkn;
  int readFirstToken;
  char *endWord;
  {
   struct expr *top, *nextOne, *lastOne;

   /*========================================================*/
   /* Continue until all appropriate commands are processed. */
   /*========================================================*/

   top = GenConstant(FCALL,FindFunction("progn"));

   lastOne = NULL;

   while (CLIPS_TRUE)
     {
      if (readFirstToken)
        { GetToken(logicalName,rtn_tkn); }
      else
        { readFirstToken = CLIPS_TRUE; }

      if ((rtn_tkn->type == SYMBOL) && (endWord != NULL))
        {
         if (strcmp(ValueToString(rtn_tkn->value),endWord) == 0) return(top);
        }

      if ((rtn_tkn->type == SYMBOL) || (rtn_tkn->type == STRING) ||
          (rtn_tkn->type == INTEGER) || (rtn_tkn->type == FLOAT) ||
#if DEFGLOBAL_CONSTRUCT
          (rtn_tkn->type == GBL_VARIABLE) ||
          (rtn_tkn->type == MF_GBL_VARIABLE) ||
#endif
#if OBJECT_SYSTEM
          (rtn_tkn->type == INSTANCE_NAME) ||
#endif
          (rtn_tkn->type == SF_VARIABLE) || (rtn_tkn->type == MF_VARIABLE))
        { nextOne = GenConstant(rtn_tkn->type,rtn_tkn->value); }
      else if (rtn_tkn->type == LPAREN)
        { nextOne = Function1Parse(logicalName); }
      else
        {
         if (ReplaceSequenceExpansionOps(top,NULL,
                                         FindFunction("(expansion-call)"),
                                         FindFunction("expand$")))
           {
            ReturnExpression(top);
            return(NULL);
           }
         return(top);
        }

      if (nextOne == NULL)
        {
         rtn_tkn->type = UNKNOWN;
         ReturnExpression(top);
         return(NULL);
        }

      if (lastOne == NULL)
        { top->argList = nextOne; }
      else
        { lastOne->nextArg = nextOne; }

      lastOne = nextOne;

      PPCRAndIndent();
     }
  }
  
#endif

/*********************************************/
/* SetSequenceOperatorRecognition: Sets the  */
/*   value of the SequenceOpMode flag.       */
/*********************************************/
globle BOOLEAN SetSequenceOperatorRecognition(value)
  int value;
  {
   int ov;

   ov = SequenceOpMode;
   SequenceOpMode = value;
   return(ov);
  }
  
/*********************************************/
/* SetSequenceOperatorRecognition: Gets the  */
/*   value of the SequenceOpMode flag.       */
/*********************************************/
globle BOOLEAN GetSequenceOperatorRecognition()
  {
   return(SequenceOpMode);
  }
  
/******************************************************/
/* ParseConstantArguments: Parses a string into a set */
/*    of constant expressions.                        */
/******************************************************/
globle EXPRESSION *ParseConstantArguments(argstr,error)
  char *argstr;
  int *error;
  {
   EXPRESSION *top = NULL,*bot = NULL,*tmp;
   char *router = "***CLIPSFNXARGS***";
   struct token tkn;

   *error = CLIPS_FALSE;
   if (argstr != NULL)
     {
      if (OpenStringSource(router,argstr,0) == 0)
        {
         PrintErrorID("EXPRNPSR",6,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Cannot read arguments for external call.\n");
         *error = CLIPS_TRUE;
         return(NULL);
        }

      GetToken(router,&tkn);
      while (tkn.type != STOP)
        {
         if ((tkn.type != SYMBOL) && (tkn.type != STRING) &&
             (tkn.type != FLOAT) && (tkn.type != INTEGER) &&
             (tkn.type != INSTANCE_NAME))
           {         
            PrintErrorID("EXPRNPSR",7,CLIPS_FALSE);
            PrintCLIPS(WERROR,"Only constant arguments allowed for external CLIPS function call.\n");
            ReturnExpression(top);
            *error = CLIPS_TRUE;
            return(NULL);
           }
         tmp = GenConstant(tkn.type,tkn.value);
         if (top == NULL)
           top = tmp;
         else
           bot->nextArg = tmp;
         bot = tmp;
         GetToken(router,&tkn);
        }
      CloseStringSource(router);
     }
   return(top);
  }
  
