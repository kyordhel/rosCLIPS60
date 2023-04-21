   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            MISCELLANEOUS FUNCTIONS MODULE           */
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

#define _MISCFUN_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "setup.h"

#include "clipsmem.h"
#include "sysdep.h"
#include "multifld.h"
#include "exprnpsr.h"
#include "argacces.h"
#include "router.h"
#include "utility.h"

#if DEFFUNCTION_CONSTRUCT
#include "dffnxfun.h"
#endif

#include "miscfun.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                    ExpandFuncMultifield(DATA_OBJECT *,EXPRESSION *,
                                                       EXPRESSION **,VOID *);
#else
   static VOID                    ExpandFuncMultifield();
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static long int         GensymNumber = 1;

#if ! RUN_TIME
/*******************************************/
/* MiscFunctionDefinitions:           */
/*******************************************/
globle VOID MiscFunctionDefinitions()
  {
   DefineFunction2("gensym",           'w', PTIF GensymFunction,      "GensymFunction", "00");
   DefineFunction2("gensym*",          'w', PTIF GensymStarFunction,  "GensymStarFunction", "00");
   DefineFunction2("setgen",           'l', PTIF SetgenFunction,      "SetgenFunction", "11i");
   DefineFunction2("system",           'v', PTIF gensystem,           "gensystem", "1*k");
   DefineFunction2("length",           'l', PTIF LengthFunction,      "LengthFunction", "11q");
   DefineFunction2("length$",          'l', PTIF LengthFunction,      "LengthFunction", "11q");
   DefineFunction2("time",             'd', PTIF gentime,             "gentime", "00");
   DefineFunction2("random",           'l', PTIF RandomFunction,      "RandomFunction", "00");
   DefineFunction2("seed",             'v', PTIF SeedFunction,        "SeedFunction", "11i");
   DefineFunction2("conserve-mem",     'v', PTIF ConserveMemCommand,  "ConserveMemCommand", "11w");
   DefineFunction2("release-mem",      'l', PTIF ReleaseMemCommand,   "ReleaseMemCommand", "00");
#if DEBUGGING_FUNCTIONS
   DefineFunction2("mem-used",         'l', PTIF MemUsedCommand,      "MemUsedCommand", "00");
   DefineFunction2("mem-requests",     'l', PTIF MemRequestsCommand,  "MemRequestsCommand", "00");
#endif
   DefineFunction2("options",          'v', PTIF OptionsCommand,      "OptionsCommand", "00");
   DefineFunction2("(expansion-call)", 'u', PTIF ExpandFuncCall,      "ExpandFuncCall",NULL);
   DefineFunction2("expand$",'u', PTIF DummyExpandFuncMultifield,
                                           "DummyExpandFuncMultifield","11m");
   FuncSeqOvlFlags("expand$",CLIPS_FALSE,CLIPS_FALSE);
   DefineFunction2("(set-evaluation-error)", 
                                       'w', PTIF CauseEvaluationError,"CauseEvaluationError",NULL);
   DefineFunction2("set-sequence-operator-recognition",
                                       'b', PTIF SetSORCommand,"SetSORCommand","11w");
   DefineFunction2("get-sequence-operator-recognition",'b',
                    PTIF GetSequenceOperatorRecognition,"GetSequenceOperatorRecognition","00");
   DefineFunction2("get-function-restrictions",'s',
                   PTIF GetFunctionRestrictions,"GetFunctionRestrictions","11w");
   DefineFunction2("create$",     'm', PTIF CreateFunction,  "CreateFunction", NULL);
   DefineFunction2("mv-append",   'm', PTIF CreateFunction,  "CreateFunction", NULL);
   DefineFunction2("apropos",   'v', PTIF AproposCommand,  "AproposCommand", "11w");
  }
#endif

/***********************************************************/
/* CreateFunction: Create$ function for multifield values. */
/***********************************************************/
globle VOID CreateFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   StoreInMultifield(returnValue,GetFirstArgument(),CLIPS_TRUE);
  }  

/****************************************/
/* SetgenFunction:                      */
/****************************************/
globle long int SetgenFunction()
  {
   long lnum;
   DATA_OBJECT val_ptr;

   if (ArgCountCheck("setgen",EXACTLY,1) == -1) return(GensymNumber);
   if (ArgTypeCheck("setgen",1,INTEGER,&val_ptr) == CLIPS_FALSE) return(GensymNumber);

   lnum = CoerceToLongInteger(val_ptr.type,val_ptr.value);

   if (lnum < 1L)
     {
      ExpectedTypeError1("setgen",1,"number (greater than or equal to 1)");
      return(GensymNumber);
     }

   GensymNumber = lnum;
   return(lnum);
  }

/****************************************/
/* GensymFunction:                      */
/****************************************/
globle VOID *GensymFunction()
  {
   char genstring[15];

   ArgCountCheck("gensym",EXACTLY,0);

   sprintf(genstring,"gen%ld",GensymNumber);
   GensymNumber++;
   return(AddSymbol(genstring));
  }

/****************************************/
/* GensymStarFunction:                  */
/****************************************/
globle VOID *GensymStarFunction()
  {
   char genstring[15];

   ArgCountCheck("gensym*",EXACTLY,0);
   do
     {
      sprintf(genstring,"gen%ld",GensymNumber);
      GensymNumber++;
     } while (FindSymbol(genstring) != NULL);

   return(AddSymbol(genstring));
  }

/****************************************/
/* RandomFunction:                           */
/****************************************/
globle long RandomFunction()
  {
   ArgCountCheck("random",EXACTLY,0);

   return((long) genrand());
  }

/****************************************/
/* SeedFunction:                              */
/****************************************/
globle VOID SeedFunction()
  {
   int fnum;
   DATA_OBJECT val_ptr;

   if (ArgCountCheck("seed",EXACTLY,1) == -1) return;
   if (ArgTypeCheck("seed",1,INTEGER,&val_ptr) == CLIPS_FALSE) return;

   fnum = (int) DOToLong(val_ptr);

   genseed(fnum);
  }

/****************************************/
/* LengthFunction:                           */
/****************************************/
globle long int LengthFunction()
  {
   DATA_OBJECT item;

   if (ArgCountCheck("length$",EXACTLY,1) == -1) return(-1L);

   RtnUnknown(1,&item);

   if ((GetType(item) == STRING) || (GetType(item) == SYMBOL))
     {  return( (long) strlen(DOToString(item))); }

   else if (GetType(item) == MULTIFIELD)
     { return ( (long) GetDOLength(item)); }

   else
     {
      SetEvaluationError(CLIPS_TRUE);
      ExpectedTypeError2("length$",1);
      return(-1L);
     }
  }

/*******************************************/
/* ReleaseMemCommand:  Returns free memory */
/*   stored by CLIPS to the system.        */
/*******************************************/
globle long ReleaseMemCommand()
  {
   if (ArgCountCheck("release-mem",EXACTLY,0) == -1) return(0);
   return(ReleaseMem(-1L,CLIPS_FALSE));
  }

/**********************************************/
/* ConserveMemCommand:  Determines whether pretty print */
/*   representation for defrules and deffacts */
/*   will be stored.                          */
/**********************************************/
globle VOID ConserveMemCommand()
  {
   char *argument;
   DATA_OBJECT arg_ptr;

   if (ArgCountCheck("conserve-mem",EXACTLY,1) == -1) return;
   if (ArgTypeCheck("conserve-mem",1,SYMBOL,&arg_ptr) == CLIPS_FALSE) return;

   argument = DOToString(arg_ptr);

   if (strcmp(argument,"on") == 0)
     { SetConserveMemory(CLIPS_TRUE); }
   else if (strcmp(argument,"off") == 0)
     { SetConserveMemory(CLIPS_FALSE); }
   else
     {
      ExpectedTypeError1("conserve-mem",1,"symbol with value on or off");
      return;
     }

   return;
  }

#if DEBUGGING_FUNCTIONS

/***************************************/
/* MemUsedCommand:  Returns amount of  */
/*   memory allocated by CLIPS.        */
/***************************************/
globle long int MemUsedCommand()
  {
   if (ArgCountCheck("mem-used",EXACTLY,0) == -1) return(0);
   return(MemUsed());
  }

/**********************************************/
/* MemRequestsCommand:  Indicates number of memory */
/*   allocations made by CLIPS.               */
/**********************************************/
globle long int MemRequestsCommand()
  {

   if (ArgCountCheck("mem-requests",EXACTLY,0) == -1) return(0);
   return(MemRequests());
  }

#endif

/***************************************/
/* AproposCommand:         */
/***************************************/
globle VOID AproposCommand()
  {
   char *argument;
   DATA_OBJECT argPtr;
   struct symbolHashNode *hashPtr = NULL;
   int theLength;
   
   if (ArgCountCheck("apropos",EXACTLY,1) == -1) return;
   
   if (ArgTypeCheck("apropos",1,SYMBOL,&argPtr) == CLIPS_FALSE) return;

   argument = DOToString(argPtr);
   theLength = strlen(argument);
   
   while ((hashPtr = GetNextSymbolMatch(argument,theLength,hashPtr,CLIPS_TRUE)) != NULL)
     { 
      PrintCLIPS(WDISPLAY,ValueToString(hashPtr));
      PrintCLIPS(WDISPLAY,"\n");
     }
  }
  
/***************************************************************/
/* OptionsCommand: Lists the compiler flags that have been turned on. */
/***************************************************************/
globle VOID OptionsCommand()
  {
   if (ArgCountCheck("options",EXACTLY,0) == -1) return;

   PrintCLIPS(WDISPLAY,"Machine type: ");

#if GENERIC
   PrintCLIPS(WDISPLAY,"Generic ");
#endif
#if VAX_VMS
   PrintCLIPS(WDISPLAY,"VAX VMS ");
#endif
#if UNIX_V
   PrintCLIPS(WDISPLAY,"UNIX System V or 4.2BSD ");
#endif
#if UNIX_7
   PrintCLIPS(WDISPLAY,"UNIX System III Version 7 or Sun Unix ");
#endif
#if MAC_TC
   PrintCLIPS(WDISPLAY,"Apple Macintosh with Think C 5.0 ");
#endif
#if MAC_MPW
   PrintCLIPS(WDISPLAY,"Apple Macintosh with MPW C ");
#endif
#if IBM_MSC
   PrintCLIPS(WDISPLAY,"IBM PC with Microsoft C");
#endif
#if IBM_ZTC
   PrintCLIPS(WDISPLAY,"IBM PC with Zortech C");
#endif
#if IBM_ICB
   PrintCLIPS(WDISPLAY,"IBM PC with Intel C Code Builder");
#endif
#if IBM_TBC
   PrintCLIPS(WDISPLAY,"IBM PC with Turbo C");
#endif
PrintCLIPS(WDISPLAY,"\n");

PrintCLIPS(WDISPLAY,"ANSI Compiler is ");
#if ANSI_COMPILER
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Defrule construct is ");
#if DEFRULE_CONSTRUCT
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

#if DEFRULE_CONSTRUCT

PrintCLIPS(WDISPLAY,"  Conflict resolution strategies are ");
#if CONFLICT_RESOLUTION_STRATEGIES
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"  Dynamic salience is ");
#if DYNAMIC_SALIENCE
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"  Incremental reset is ");
#if INCREMENTAL_RESET
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"  Logical dependencies (truth maintenance) are ");
#if LOGICAL_DEPENDENCIES
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

#endif

PrintCLIPS(WDISPLAY,"Defmodule construct is ");
#if DEFMODULE_CONSTRUCT
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Deftemplate construct is ");
#if DEFTEMPLATE_CONSTRUCT
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

#if DEFTEMPLATE_CONSTRUCT

PrintCLIPS(WDISPLAY,"  Deffacts construct is ");
#if DEFFACTS_CONSTRUCT
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

#endif

PrintCLIPS(WDISPLAY,"Defglobal construct is ");
#if DEFGLOBAL_CONSTRUCT
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Deffunction construct is ");
#if DEFFUNCTION_CONSTRUCT
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Defgeneric/Defmethod constructs are ");
#if DEFGENERIC_CONSTRUCT
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

#if DEFGENERIC_CONSTRUCT

PrintCLIPS(WDISPLAY,"  Imperative methods are ");
#if IMPERATIVE_METHODS
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

#endif

PrintCLIPS(WDISPLAY,"Object System is ");
#if OBJECT_SYSTEM
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

#if OBJECT_SYSTEM

PrintCLIPS(WDISPLAY,"  Definstances construct is ");
#if DEFINSTANCES_CONSTRUCT
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"  Imperative (around/shadowed) message-handlers are ");
#if IMPERATIVE_MESSAGE_HANDLERS
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"  Auxiliary (before/after) message-handlers are ");
#if AUXILIARY_MESSAGE_HANDLERS
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"  Instance-set queries are ");
#if INSTANCE_SET_QUERIES
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"  Direct pattern-matching on instances is ");
#if INSTANCE_PATTERN_MATCHING
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"  Binary loading of instances is ");
#if BLOAD_INSTANCES
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"  Binary saving of instances is ");
#if BSAVE_INSTANCES
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

#endif

PrintCLIPS(WDISPLAY,"Extended math package is ");
#if EX_MATH
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Text processing package is ");
#if CLP_TEXTPRO
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Help system is ");
#if CLP_HELP
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Bload capability is ");
#if BLOAD_ONLY
  PrintCLIPS(WDISPLAY,"BLOAD ONLY");
#endif
#if BLOAD
  PrintCLIPS(WDISPLAY,"BLOAD");
#endif
#if BLOAD_AND_BSAVE
  PrintCLIPS(WDISPLAY,"BLOAD AND BSAVE");
#endif
#if (! BLOAD_ONLY) && (! BLOAD) && (! BLOAD_AND_BSAVE)
  PrintCLIPS(WDISPLAY,"OFF ");
#endif
PrintCLIPS(WDISPLAY,"\n");

PrintCLIPS(WDISPLAY,"EMACS Editor is ");
#if CLP_EDIT
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Construct compiler is ");
#if CONSTRUCT_COMPILER
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Basic I/O is ");
#if BASIC_IO
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Extended I/O is ");
#if EXT_IO
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"String function package is ");
#if STRING_FUNCTIONS
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Multifield function package is ");
#if MULTIFIELD_FUNCTIONS
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Debugging functions are ");
#if DEBUGGING_FUNCTIONS
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Block memory is ");
#if BLOCK_MEMORY
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Window Interface flag is ");
#if WINDOW_INTERFACE
   PrintCLIPS(WDISPLAY,"ON\n");
#else
   PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Short link names are ");
#if SHORT_LINK_NAMES
   PrintCLIPS(WDISPLAY,"ON\n");
#else
   PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Developer flag is ");
#if DEVELOPER
   PrintCLIPS(WDISPLAY,"ON\n");
#else
   PrintCLIPS(WDISPLAY,"OFF\n");
#endif

PrintCLIPS(WDISPLAY,"Run time module is ");
#if RUN_TIME
  PrintCLIPS(WDISPLAY,"ON\n");
#else
  PrintCLIPS(WDISPLAY,"OFF\n");
#endif
  }

/********************************************************************
  NAME         : ExpandFuncCall
  DESCRIPTION  : This function is a wrap-around for a normal
                   function call.  It preexamines the argument
                   expression list and expands any references to the
                   sequence operator.  It builds a copy of the 
                   function call expression with these new arguments
                   inserted and evaluates the function call.
  INPUTS       : A data object buffer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Expressions alloctaed/deallocated
                 Function called and arguments evaluated
                 EvaluationError set on errors
  NOTES        : None
 *******************************************************************/
globle VOID ExpandFuncCall(result)
  DATA_OBJECT *result;
  {
   EXPRESSION *newargexp,*fcallexp;
   struct FunctionDefinition *func;
   
   /* ======================================================================
      Copy the original function call's argument expression list.
      Look for expand$ function callsexpressions and replace those
        with the equivalent expressions of the expansions of evaluations
        of the arguments.
      ====================================================================== */
   newargexp = CopyExpression(GetFirstArgument()->argList);
   ExpandFuncMultifield(result,newargexp,&newargexp,
                        (VOID *) FindFunction("expand$"));
     
   /* ===================================================================
      Build the new function call expression with the expanded arguments.
      Check the number of arguments, if necessary, and call the thing.
      =================================================================== */
   fcallexp = get_struct(expr);
   fcallexp->type = GetFirstArgument()->type; 
   fcallexp->value = GetFirstArgument()->value;
   fcallexp->nextArg = NULL;
   fcallexp->argList = newargexp;
   if (fcallexp->type == FCALL)
     {
      func = (struct FunctionDefinition *) fcallexp->value;
      if (CheckFunctionArgCount(ValueToString(func->callFunctionName),
                                func->restrictions,CountArguments(newargexp)) == CLIPS_FALSE)
        {
         result->type = SYMBOL;
         result->value = CLIPSFalseSymbol;
         ReturnExpression(fcallexp);
         return;
        }
     }
#if DEFFUNCTION_CONSTRUCT 
   else if (fcallexp->type == PCALL)
     {
      if (CheckDeffunctionCall(fcallexp->value,
              CountArguments(fcallexp->argList)) == CLIPS_FALSE)
        {
         result->type = SYMBOL;
         result->value = CLIPSFalseSymbol;
         ReturnExpression(fcallexp);
         SetEvaluationError(TRUE);
         return;
        }
     }
#endif
     
   EvaluateExpression(fcallexp,result);
   ReturnExpression(fcallexp);
  }
  
/***********************************************************************
  NAME         : DummyExpandFuncMultifield
  DESCRIPTION  : The expansion of multifield arguments is valid only
                 when done for a function call.  All these expansions
                 are handled by the CLIPS wrpa-around function
                 (expansion-call) - see ExpandFuncCall.  If the CLIPS
                 function, epand-multifield is ever called directly,
                 it is an error.
  INPUTS       : Data object buffer
  RETURNS      : Nothing useful
  SIDE EFFECTS : EvaluationError set
  NOTES        : None
 **********************************************************************/
globle VOID DummyExpandFuncMultifield(result)
  DATA_OBJECT *result;
  {
   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   SetEvaluationError(TRUE);
   PrintErrorID("MISCFUN",1,CLIPS_FALSE);
   PrintCLIPS(WERROR,"expand$ must be used in the argument list of a function call.\n");
  }
  
/***********************************************************************
  NAME         : ExpandFuncMultifield
  DESCRIPTION  : Recursively examines an expression and replaces
                   PROC_EXPAND_MULTIFIELD expressions with the expanded
                   evaluation expression of its argument
  INPUTS       : 1) A data object result buffer
                 2) The expression to modify
                 3) The address of the expression, in case it is
                    deleted entirely
                 4) The address of the CLIPS function expand$
  RETURNS      : Nothing useful
  SIDE EFFECTS : Expressions allocated/deallocated as necessary
                 Evaluations performed
                 On errors, argument expression set to call a function
                   which causes an evaluation error when evaluated
                   a second time by actual caller.
  NOTES        : THIS ROUTINE MODIFIES EXPRESSIONS AT RUNTIME!!  MAKE
                 SURE THAT THE EXPRESSION PASSED IS SAFE TO CHANGE!!
 **********************************************************************/
static VOID ExpandFuncMultifield(result,exp,sto,expmult)
  DATA_OBJECT *result;
  EXPRESSION *exp,**sto;
  VOID *expmult;
  {
   EXPRESSION *newexp,*top,*bot;
   register int i;
   
   while (exp != NULL)
     {
      if (exp->value == expmult)
        {
         EvaluateExpression(exp->argList,result);
         ReturnExpression(exp->argList);
         if ((EvaluationError) || (result->type != MULTIFIELD))
           {
            exp->argList = NULL;
            if ((EvaluationError == CLIPS_FALSE) && (result->type != MULTIFIELD))
              ExpectedTypeError2("expand$",1);
            exp->value = (VOID *) FindFunction("(set-evaluation-error)");
            EvaluationError = CLIPS_FALSE;
            HaltExecution = CLIPS_FALSE;
            return;
           }
         top = bot = NULL;
         for (i = GetpDOBegin(result) ; i <= GetpDOEnd(result) ; i++)
           {
            newexp = get_struct(expr);
            newexp->type = GetMFType(result->value,i);
            newexp->value = GetMFValue(result->value,i);
            newexp->argList = NULL;
            newexp->nextArg = NULL;
            if (top == NULL)
              top = newexp;
            else
              bot->nextArg = newexp;
            bot = newexp;
           }
         if (top == NULL)
           {
            *sto = exp->nextArg;
            rtn_struct(expr,exp);
            exp = *sto;
           }
         else
           {
            bot->nextArg = exp->nextArg;
            *sto = top;
            rtn_struct(expr,exp);
            sto = &bot->nextArg;
            exp = bot->nextArg;
           }
        }
      else
        {
         if (exp->argList != NULL)
           ExpandFuncMultifield(result,exp->argList,&exp->argList,expmult);
         sto = &exp->nextArg;
         exp = exp->nextArg;
        }
     }
  }

/****************************************************************
  NAME         : CauseEvaluationError
  DESCRIPTION  : Dummy function use to cause evaluation errors on
                   a function call to generate error messages
  INPUTS       : None
  RETURNS      : A pointer to the CLIPSFalseSymbol
  SIDE EFFECTS : EvaluationError set
  NOTES        : None
 ****************************************************************/
globle SYMBOL_HN *CauseEvaluationError()
  {
   SetEvaluationError(TRUE);
   return((SYMBOL_HN *) CLIPSFalseSymbol);
  }
  
/****************************************************************
  NAME         : SetSORCommand
  DESCRIPTION  : Toggles SequenceOpMode - if TRUE, multifield
                   references are replaced with sequence
                   expansion operators
  INPUTS       : None
  RETURNS      : The old value of SequenceOpMode
  SIDE EFFECTS : SequenceOpMode toggled
  NOTES        : None
 ****************************************************************/
globle BOOLEAN SetSORCommand()
  {
#if (! RUN_TIME) && (! BLOAD_ONLY)
   DATA_OBJECT arg;
   
   if (ArgTypeCheck("set-sequence-operator-recognition",1,SYMBOL,&arg) == CLIPS_FALSE)
     return(SequenceOpMode);
   return(SetSequenceOperatorRecognition((arg.value == CLIPSFalseSymbol) ? 
                                         CLIPS_FALSE : CLIPS_TRUE));
#else 
     return(SequenceOpMode);
#endif
  }
  
/********************************************************************
  NAME         : GetFunctionRestrictions
  DESCRIPTION  : Gets DefineFunction2() restriction list for function
  INPUTS       : None
  RETURNS      : A string containing the function restriction codes
  SIDE EFFECTS : EvaluationError set on errors
  NOTES        : None
 ********************************************************************/
globle SYMBOL_HN *GetFunctionRestrictions()
  {
   DATA_OBJECT temp;
   struct FunctionDefinition *fptr;
   
   if (ArgTypeCheck("get-function-restrictions",1,SYMBOL,&temp) == CLIPS_FALSE)
     return((SYMBOL_HN *) AddSymbol(""));
   fptr = FindFunction(DOToString(temp));
   if (fptr == NULL)
     {
      CantFindItemErrorMessage("function",DOToString(temp));
      SetEvaluationError(TRUE);
      return((SYMBOL_HN *) AddSymbol(""));
     }
   if (fptr->restrictions == NULL)
     return((SYMBOL_HN *) AddSymbol("0**"));
   return((SYMBOL_HN *) AddSymbol(fptr->restrictions));
  }
  