   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*              CLIPS Version 6.00  05/12/93           */
   /*                                                     */
   /*                INSTANCE PARSER MODULE               */
   /*******************************************************/

/*************************************************************/
/* Purpose:  Instance Function Parsing Routines              */
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
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#if ANSI_COMPILER
#include <string.h>
#endif

#include "classcom.h"
#include "classfun.h"
#include "classinf.h"
#include "constant.h"
#include "moduldef.h"
#include "evaluatn.h"
#include "exprnpsr.h"
#include "extnfunc.h"
#include "prntutil.h"
#include "router.h"

#define _INSPSR_SOURCE_
#include "inspsr.h"

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */
#define MAKE_TYPE       0
#define INITIALIZE_TYPE 1
#define MODIFY_TYPE     2
#define DUPLICATE_TYPE  3

#define CLASS_RLN          "of"
#define DUPLICATE_NAME_REF "to"

/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER

static BOOLEAN ReplaceClassNameWithReference(EXPRESSION *);

#else

static BOOLEAN ReplaceClassNameWithReference();

#endif

/* =========================================
   *****************************************
       EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
#if (! BLOAD_ONLY) && (! RUN_TIME)
extern struct token ObjectParseToken;
#else
globle struct token ObjectParseToken;
#endif

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
  
#if ! RUN_TIME

/*************************************************************************************
  NAME         : ParseInitializeInstance
  DESCRIPTION  : Parses initialize-instance and make-instance function
                   calls into an EXPRESSION form that
                   can later be evaluated with EvaluateExpression()
  INPUTS       : 1) The address of the top node of the expression
                    containing the initialize-instance function call
                 2) The logical name of the input source
  RETURNS      : The address of the modified expression, or NULL
                    if there is an error
  SIDE EFFECTS : The expression is enhanced to include all
                    aspects of the initialize-instance call
                    (slot-overrides etc.)
                 The "top" expression is deleted on errors.
  NOTES        : This function parses a initialize-instance call into
                 an expression of the following form :
                 
                 (initialize-instance <instance-name> <slot-override>*)
                  where <slot-override> ::= (<slot-name> <expression>+)
                  
                  goes to -->
                  
                  initialize-instance
                      |
                      V
                  <instance or name>-><slot-name>-><dummy-node>...
                                                      |
                                                      V
                                               <value-expression>...
                                             
                  (make-instance <instance> of <class> <slot-override>*)
                  goes to -->
                  
                  make-instance
                      |
                      V
                  <instance-name>-><class-name>-><slot-name>-><dummy-node>...
                                                                 |
                                                                 V
                                                          <value-expression>...

                  (make-instance of <class> <slot-override>*)
                  goes to -->
                  
                  make-instance
                      |
                      V
                  (gensym*)-><class-name>-><slot-name>-><dummy-node>...
                                                                 |
                                                                 V
                                                          <value-expression>...

                  (modify-instance <instance> <slot-override>*)
                  goes to -->
                  
                  modify-instance
                      |
                      V
                  <instance or name>-><slot-name>-><dummy-node>...
                                                      |
                                                      V
                                               <value-expression>...

                  (duplicate-instance <instance> [to <new-name>] <slot-override>*)
                  goes to -->
                  
                  duplicate-instance
                      |
                      V
                  <instance or name>-><new-name>-><slot-name>-><dummy-node>...
                                          OR                         |
                                      (gensym*)                      V
                                                           <value-expression>...

 *************************************************************************************/
globle EXPRESSION *ParseInitializeInstance(top,readSource)
  EXPRESSION *top;
  char *readSource;
  {
   int error,fcalltype,readclass;
   
   if ((top->value == (VOID *) FindFunction("make-instance")) ||
       (top->value == (VOID *) FindFunction("active-make-instance")))
     fcalltype = MAKE_TYPE;
   else if ((top->value == (VOID *) FindFunction("initialize-instance")) ||
            (top->value == (VOID *) FindFunction("active-initialize-instance")))
     fcalltype = INITIALIZE_TYPE;
   else if ((top->value == (VOID *) FindFunction("modify-instance")) ||
            (top->value == (VOID *) FindFunction("active-modify-instance")) ||
            (top->value == (VOID *) FindFunction("message-modify-instance")) ||
            (top->value == (VOID *) FindFunction("active-message-modify-instance")))
     fcalltype = MODIFY_TYPE;
   else
     fcalltype = DUPLICATE_TYPE;
   IncrementIndentDepth(3);
   error = CLIPS_FALSE;
   if (top->type == UNKNOWN)
     top->type = FCALL;
   else
     SavePPBuffer(" ");
   top->argList = ArgumentParse(readSource,&error);
   if (error)
     goto ParseInitializeInstanceError;
   else if (top->argList == NULL)
     {
      SyntaxErrorMessage("instance");
      goto ParseInitializeInstanceError;
     }
   SavePPBuffer(" ");
   
   if (fcalltype == MAKE_TYPE)
     {
      /* ======================================
         Handle the case of anonymous instances
         where the name was not specified
         ====================================== */
      if ((top->argList->type != SYMBOL) ? CLIPS_FALSE :
          (strcmp(ValueToString(top->argList->value),CLASS_RLN) == 0))
        {
         top->argList->nextArg = ArgumentParse(readSource,&error);
         if (error == CLIPS_TRUE)
           goto ParseInitializeInstanceError;
         if (top->argList->nextArg == NULL)
           {
            SyntaxErrorMessage("instance class");
            goto ParseInitializeInstanceError;
           }
         if ((top->argList->nextArg->type != SYMBOL) ? CLIPS_TRUE :
             (strcmp(ValueToString(top->argList->nextArg->value),CLASS_RLN) != 0))
           {
            top->argList->type = FCALL;
            top->argList->value = (VOID *) FindFunction("gensym*");
            readclass = CLIPS_FALSE;
           }
         else
           readclass = CLIPS_TRUE;
        }
      else
        {
         GetToken(readSource,&ObjectParseToken);
         if ((GetType(ObjectParseToken) != SYMBOL) ? CLIPS_TRUE :
             (strcmp(CLASS_RLN,DOToString(ObjectParseToken)) != 0))
           {
            SyntaxErrorMessage("make-instance");
            goto ParseInitializeInstanceError;
           }
         SavePPBuffer(" ");
         readclass = CLIPS_TRUE;
        }
      if (readclass)
        {
         top->argList->nextArg = ArgumentParse(readSource,&error);
         if (error)
           goto ParseInitializeInstanceError;
         if (top->argList->nextArg == NULL)
           {
            SyntaxErrorMessage("instance class");
            goto ParseInitializeInstanceError;
           }
        }
        
      /* ==============================================
         If the class name is a constant, go ahead and
         look it up now and replace it with the pointer
         ============================================== */
      if (ReplaceClassNameWithReference(top->argList->nextArg) == CLIPS_FALSE)
        goto ParseInitializeInstanceError;

      PPCRAndIndent();
      GetToken(readSource,&ObjectParseToken);
      top->argList->nextArg->nextArg = 
                  ParseSlotOverrides(readSource,&error);
     }
   else
     {
      PPCRAndIndent();
      GetToken(readSource,&ObjectParseToken);
      if (fcalltype == DUPLICATE_TYPE)
        {
         if ((ObjectParseToken.type != SYMBOL) ? CLIPS_FALSE :
             (strcmp(DOToString(ObjectParseToken),DUPLICATE_NAME_REF) == 0))
           {
            PPBackup();
            PPBackup();
            SavePPBuffer(ObjectParseToken.print_rep);
            SavePPBuffer(" ");
            top->argList->nextArg = ArgumentParse(readSource,&error);
            if (error)
              goto ParseInitializeInstanceError;
            if (top->argList->nextArg == NULL)
              {
               SyntaxErrorMessage("instance name");
               goto ParseInitializeInstanceError;
              }
            PPCRAndIndent();
            GetToken(readSource,&ObjectParseToken);
           }
         else
           top->argList->nextArg = GenConstant(FCALL,(VOID *) FindFunction("gensym*"));
         top->argList->nextArg->nextArg = ParseSlotOverrides(readSource,&error);
        }
      else
        top->argList->nextArg = ParseSlotOverrides(readSource,&error);
     }
   if (error)
      goto ParseInitializeInstanceError;
   if (GetType(ObjectParseToken) != RPAREN)
     {
      SyntaxErrorMessage("slot-override");
      goto ParseInitializeInstanceError;
     }
   DecrementIndentDepth(3);
   return(top);
   
ParseInitializeInstanceError:
   SetEvaluationError(CLIPS_TRUE);
   ReturnExpression(top);
   DecrementIndentDepth(3);
   return(NULL);
  }

/********************************************************************************
  NAME         : ParseSlotOverrides
  DESCRIPTION  : Forms expressions for slot-overrides
  INPUTS       : 1) The logical name of the input
                 2) Caller's buffer for error flkag
  RETURNS      : Address override expressions, NULL
                   if none or error.
  SIDE EFFECTS : Slot-expression built
                 Caller's error flag set
  NOTES        : <slot-override> ::= (<slot-name> <value>*)*
  
                 goes to
                 
                 <slot-name> --> <dummy-node> --> <slot-name> --> <dummy-node>...
                                       |
                                       V
                               <value-expression> --> <value-expression> --> ...
                               
                 Assumes first token has already been scanned
 ********************************************************************************/
globle EXPRESSION *ParseSlotOverrides(readSource,error)
  char *readSource;
  int *error;
  {
   EXPRESSION *top = NULL,*bot = NULL,*exp;
   
   while (GetType(ObjectParseToken) == LPAREN)
     {
      *error = CLIPS_FALSE;
      exp = ArgumentParse(readSource,error);
      if (*error == CLIPS_TRUE)
        {
         ReturnExpression(top);
         return(NULL);
        }
      else if (exp == NULL)
        {
         SyntaxErrorMessage("slot-override");
         *error = CLIPS_TRUE;
         ReturnExpression(top);
         SetEvaluationError(CLIPS_TRUE);
         return(NULL);
        }
      exp->nextArg = GenConstant(SYMBOL,CLIPSTrueSymbol);
      if (CollectArguments(exp->nextArg,readSource) == NULL)
        {
         *error = CLIPS_TRUE;
         ReturnExpression(top);
         return(NULL);
        }
      if (top == NULL)
        top = exp;
      else
        bot->nextArg = exp;
      bot = exp->nextArg;
      PPCRAndIndent();
      GetToken(readSource,&ObjectParseToken);
     }
   PPBackup();
   PPBackup();
   SavePPBuffer(ObjectParseToken.print_rep);
   return(top);
  }

#endif

/****************************************************************************
  NAME         : ParseSimpleInstance
  DESCRIPTION  : Parses instances from file for load-instances
                   into an EXPRESSION forms that
                   can later be evaluated with EvaluateExpression()
  INPUTS       : 1) The address of the top node of the expression
                    containing the make-instance function call
                 2) The logical name of the input source
  RETURNS      : The address of the modified expression, or NULL
                    if there is an error
  SIDE EFFECTS : The expression is enhanced to include all
                    aspects of the make-instance call
                    (slot-overrides etc.)
                 The "top" expression is deleted on errors.
  NOTES        : The name, class, values etc. must be constants.

                 This function parses a make-instance call into
                 an expression of the following form :
                                             
                  (make-instance <instance> of <class> <slot-override>*)
                  where <slot-override> ::= (<slot-name> <expression>+)
                  
                  goes to -->
                  
                  make-instance
                      |
                      V
                  <instance-name>-><class-name>-><slot-name>-><dummy-node>...
                                                                 |
                                                                 V
                                                          <value-expression>...

 ****************************************************************************/
globle EXPRESSION *ParseSimpleInstance(top,readSource)
  EXPRESSION *top;
  char *readSource;
  {
   EXPRESSION *exp,*vals = NULL,*vbot,*tval;
   int type;

   GetToken(readSource,&ObjectParseToken);
   if ((GetType(ObjectParseToken) != INSTANCE_NAME) &&
       (GetType(ObjectParseToken) != SYMBOL))
     goto MakeInstanceError;
   top->argList = GenConstant(INSTANCE_NAME,
                               (VOID *) GetValue(ObjectParseToken));
   GetToken(readSource,&ObjectParseToken);
   if ((GetType(ObjectParseToken) != SYMBOL) ? CLIPS_TRUE :
       (strcmp(CLASS_RLN,DOToString(ObjectParseToken)) != 0))
     goto MakeInstanceError;
   GetToken(readSource,&ObjectParseToken);
   if (GetType(ObjectParseToken) != SYMBOL)
     goto MakeInstanceError;
   top->argList->nextArg =
        GenConstant(SYMBOL,(VOID *) GetValue(ObjectParseToken));
   exp = top->argList->nextArg;
   if (ReplaceClassNameWithReference(exp) == CLIPS_FALSE)
     goto MakeInstanceError;
   GetToken(readSource,&ObjectParseToken);
   while (GetType(ObjectParseToken) == LPAREN)
     {
      GetToken(readSource,&ObjectParseToken);
      if (GetType(ObjectParseToken) != SYMBOL)
        goto SlotOverrideError;
      exp->nextArg = GenConstant(SYMBOL,(VOID *) GetValue(ObjectParseToken));
      exp->nextArg->nextArg = GenConstant(SYMBOL,CLIPSTrueSymbol);
      exp = exp->nextArg->nextArg;
      GetToken(readSource,&ObjectParseToken);
      vbot = NULL;
      while (GetType(ObjectParseToken) != RPAREN)
        {
         type = GetType(ObjectParseToken);
         if (type == LPAREN)
           {
            GetToken(readSource,&ObjectParseToken);
            if ((GetType(ObjectParseToken) != SYMBOL) ? CLIPS_TRUE :
                (strcmp(ValueToString(ObjectParseToken.value),"create$") != 0))
              goto SlotOverrideError;
            GetToken(readSource,&ObjectParseToken);
            if (GetType(ObjectParseToken) != RPAREN)
              goto SlotOverrideError;
            tval = GenConstant(FCALL,(VOID *) FindFunction("create$"));
           }
         else
           {
            if ((type != SYMBOL) && (type != STRING) &&
                (type != FLOAT) && (type != INTEGER) && (type != INSTANCE_NAME))
              goto SlotOverrideError;
            tval = GenConstant(type,(VOID *) GetValue(ObjectParseToken));
           }
         if (vals == NULL)
           vals = tval;
         else
           vbot->nextArg = tval;
         vbot = tval;
         GetToken(readSource,&ObjectParseToken);
        }
      exp->argList = vals;
      GetToken(readSource,&ObjectParseToken);
      vals = NULL;
     }
   if (GetType(ObjectParseToken) != RPAREN)
     goto SlotOverrideError;
   return(top);
   
MakeInstanceError:
   SyntaxErrorMessage("make-instance");
   SetEvaluationError(CLIPS_TRUE);
   ReturnExpression(top);
   return(NULL);
   
SlotOverrideError:
   SyntaxErrorMessage("slot-override");
   SetEvaluationError(CLIPS_TRUE);
   ReturnExpression(top);
   ReturnExpression(vals);
   return(NULL);
  }

/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

/***************************************************
  NAME         : ReplaceClassNameWithReference
  DESCRIPTION  : In parsing a make instance call,
                 this function replaces a constant
                 class name with an actual pointer
                 to the class
  INPUTS       : The expression
  RETURNS      : CLIPS_TRUE if all OK, CLIPS_FALSE
                 if class cannot be found
  SIDE EFFECTS : The expression type and value are
                 modified if class is found
  NOTES        : Searches current nd imported
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
      theDefclass = (VOID *) LookupDefclassInScope(theClassName);
      if (theDefclass == NULL)
        {
         CantFindItemErrorMessage("class",theClassName);
         return(CLIPS_FALSE);
        }
      if (ClassAbstractP(theDefclass))
        {
         PrintErrorID("INSMNGR",3,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Cannot create instances of abstract class ");
         PrintCLIPS(WERROR,theClassName);
         PrintCLIPS(WERROR,".\n");
         return(CLIPS_FALSE);
        }
      exp->type = DEFCLASS_PTR;
      exp->value = theDefclass;
     }
   return(CLIPS_TRUE);
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


