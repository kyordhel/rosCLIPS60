   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*               STRING FUNCTIONS MODULE               */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Barry Cameron                                        */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _STRNGFUN_SOURCE_

#include "setup.h"

#if STRING_FUNCTIONS

#include <stdio.h>
#define _CLIPS_STDIO_
#include <ctype.h>
#include <string.h>

#include "clipsmem.h"
#include "extnfunc.h"
#include "argacces.h"
#include "router.h"
#include "strngrtr.h"
#include "scanner.h" 
#include "prcdrpsr.h"
#include "cstrcpsr.h"
#include "exprnpsr.h"
#include "constrct.h"

#if DEFRULE_CONSTRUCT
#include "drive.h"
#endif

#include "strngfun.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER   
   static VOID                    StrOrSymCatFunction(DATA_OBJECT_PTR,int);
#else
   static VOID                    StrOrSymCatFunction();
#endif

#if ! RUN_TIME
/*****************************************************************************/
/* StringFunctionDefinitions: Initializes the string manipulation functions. */
/*****************************************************************************/
globle VOID StringFunctionDefinitions()
  {
   DefineFunction2("str-cat",     'k', PTIF StrCatFunction,      "StrCatFunction", "1*");
   DefineFunction2("sym-cat",     'k', PTIF SymCatFunction,      "SymCatFunction", "1*");
   DefineFunction2("str-length",  'l', PTIF StrLengthFunction,   "StrLengthFunction", "11j");
   DefineFunction2("str-compare", 'l', PTIF StrCompareFunction,  "StrCompareFunction", "23*jji");
   DefineFunction2("upcase",      'j', PTIF UpcaseFunction,      "UpcaseFunction", "11j");
   DefineFunction2("lowcase",     'j', PTIF LowcaseFunction,     "LowcaseFunction", "11j");
   DefineFunction2("sub-string",  's', PTIF SubStringFunction,   "SubStringFunction", "33*iij");
   DefineFunction2("str-index",   'u', PTIF StrIndexFunction,    "StrIndexFunction", "22j");
   DefineFunction2("eval",        'u', PTIF EvalFunction,        "EvalFunction", "11k");
   DefineFunction2("build",       'b', PTIF BuildFunction,       "BuildFunction", "11k");
  }
#endif

/************************************************************/
/* StrCatFunction: CLIPS command for concatenating a series */
/*   of primitive data types together to yield a string.    */
/*   Syntax: (str-cat <value-1> ... <value-n>)              */
/************************************************************/
globle VOID StrCatFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  { StrOrSymCatFunction(returnValue,STRING); }

/************************************************************/
/* SymCatFunction: CLIPS command for concatenating a series */
/*   of primitive data types together to yield a symbol.    */
/*   Syntax: (sym-cat <value-1> ... <value-n>)              */
/************************************************************/
globle VOID SymCatFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   StrOrSymCatFunction(returnValue,SYMBOL);
  }

/********************************************************/
/* StrOrSymCatFunction: Driver routine for implementing */
/*   the str-cat and sym-cat functions.                 */
/********************************************************/
static VOID StrOrSymCatFunction(returnValue,returnType)
  DATA_OBJECT_PTR returnValue;
  int returnType;
  {
   DATA_OBJECT theArg;
   int numArgs, i, total, j;
   char *theString, **arrayOfStrings;
   VOID *hashPtr;
   char *functionName;

   /*============================================*/
   /* Determine the calling function name.       */
   /* Store the null string or the symbol nil as */
   /* the return value in the event of an error. */
   /*============================================*/
   
   SetpType(returnValue,returnType);
   if (returnType == STRING) 
     {
      functionName = "str-cat";
      SetpValue(returnValue,(VOID *) AddSymbol(""));
     }
   else 
     {
      functionName = "sym-cat";
      SetpValue(returnValue,(VOID *) AddSymbol("nil"));
     }

   /*===============================================*/
   /* Determine the number of arguments as create a */
   /* string array which is large enough to store   */
   /* the string representation of each argument.   */
   /*===============================================*/
   
   numArgs = RtnArgCount();
   arrayOfStrings = (char **) gm1((int) sizeof(char *) * numArgs);

   /*=============================================*/
   /* Evaluate each argument and store its string */
   /* representation in the string array.         */
   /*=============================================*/
   
   total = 1;
   for (i = 1 ; i <= numArgs ; i++)
     {
      RtnUnknown(i,&theArg);
        
      switch(GetType(theArg))
        {
         case STRING:
#if OBJECT_SYSTEM
         case INSTANCE_NAME:
#endif
         case SYMBOL:
           arrayOfStrings[i-1] = ValueToString(GetValue(theArg));
           break;
           
         case FLOAT:
           hashPtr = AddSymbol(FloatToString(ValueToDouble(GetValue(theArg))));
           arrayOfStrings[i-1] = ValueToString(hashPtr);
           break;
           
         case INTEGER:
           hashPtr = AddSymbol(LongIntegerToString(ValueToLong(GetValue(theArg))));
           arrayOfStrings[i-1] = ValueToString(hashPtr);
           break;
           
         default: 
           ExpectedTypeError1(functionName,i,"string, instance name, symbol, float, or integer");
           SetEvaluationError(CLIPS_TRUE);
           break;
        }

      if (EvaluationError)
        {
         rm(arrayOfStrings,(int) sizeof(char *) * numArgs);
         return;
        }
        
      total += strlen(arrayOfStrings[i - 1]);
     }
     
   /*=========================================================*/
   /* Allocate the memory to store the concatenated string or */
   /* symbol, then copy the values in the string array to the */
   /* memory just allocated.                                  */
   /*=========================================================*/
   
   theString = (char *) gm2 (((int) sizeof(char) * total));

   j = 0;
   for (i = 0 ; i < numArgs ; i++)
     {
      sprintf(&theString[j],"%s",arrayOfStrings[i]);
      j += strlen(arrayOfStrings[i]);
     }

   /*=========================================*/
   /* Return the concatenated value and clean */
   /* up the temporary memory used.           */
   /*=========================================*/
   
   SetpValue(returnValue,(VOID *) AddSymbol(theString));
   rm(theString,(int) sizeof(char) * total);
   rm(arrayOfStrings,(int) sizeof(char *) * numArgs);
  }

/***************************************************/
/* StrLengthFunction:  CLIPS command for returning */
/*   the length of a string or symbol.             */
/*   Syntax: (str-length <string-or-symbol>)       */
/***************************************************/
globle long int StrLengthFunction()
  {
   DATA_OBJECT theArg;

   /*===================================================*/
   /* Function str-length expects exactly one argument. */
   /*===================================================*/

   if (ArgCountCheck("str-length",EXACTLY,1) == -1)
     { return(-1L); }

   /*==================================================*/
   /* The argument should be of type symbol or string. */
   /*==================================================*/

   if (ArgTypeCheck("str-length",1,SYMBOL_OR_STRING,&theArg) == CLIPS_FALSE)
     { return(-1L); }

   /*============================================*/
   /* Return the length of the string or symbol. */
   /*============================================*/

   return( (long) strlen(DOToString(theArg)));
  }

/***************************************************/
/* UpcaseFunction: CLIPS command for converting a  */
/*   symbol or string to all uppercase characters. */
/*   Syntax: (upcase <string-or-symbol>)           */
/***************************************************/
globle VOID UpcaseFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DATA_OBJECT theArg;
   int i, slen;
   char *osptr, *nsptr;

   /*===============================================*/
   /* Function upcase expects exactly one argument. */
   /*===============================================*/

   if (ArgCountCheck("upcase",EXACTLY,1) == -1)
     {
      SetpType(returnValue,STRING);
      SetpValue(returnValue,(VOID *) AddSymbol(""));
      return;
     }

   /*==================================================*/
   /* The argument should be of type symbol or string. */
   /*==================================================*/

   if (ArgTypeCheck("upcase",1,SYMBOL_OR_STRING,&theArg) == CLIPS_FALSE)
     {
      SetpType(returnValue,STRING);
      SetpValue(returnValue,(VOID *) AddSymbol(""));
      return;
     }

   /*======================================================*/
   /* Allocate temporary memory and then copy the original */
   /* string or symbol to that memory, while uppercasing   */
   /* lower case alphabetic characters.                    */
   /*======================================================*/
    
   osptr = DOToString(theArg);
   slen = strlen(osptr) + 1;
   nsptr = (char *) gm2(slen);

   for (i = 0  ; i < slen ; i++)
     {
      if (islower(osptr[i]))
        { nsptr[i] = toupper(osptr[i]); }
      else
        { nsptr[i] = osptr[i]; }
     }

   /*========================================*/
   /* Return the uppercased string and clean */
   /* up the temporary memory used.          */
   /*========================================*/

   SetpType(returnValue,GetType(theArg));
   SetpValue(returnValue,(VOID *) AddSymbol(nsptr));
   rm(nsptr,slen);
  }

/****************************************************/
/* LowcaseFunction: CLIPS command for converting a  */
/*   symbol or string to all lowercase characters.  */
/*   Syntax: (lowcase <string-or-symbol>)           */
/****************************************************/
globle VOID LowcaseFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DATA_OBJECT theArg;
   int i, slen;
   char *osptr, *nsptr;

   /*================================================*/
   /* Function lowcase expects exactly one argument. */
   /*================================================*/

   if (ArgCountCheck("lowcase",EXACTLY,1) == -1)
     {
      SetpType(returnValue,STRING);
      SetpValue(returnValue,(VOID *) AddSymbol(""));
      return;
     }

   /*==================================================*/
   /* The argument should be of type symbol or string. */
   /*==================================================*/

   if (ArgTypeCheck("lowcase",1,SYMBOL_OR_STRING,&theArg) == CLIPS_FALSE)
     {
      SetpType(returnValue,STRING);
      SetpValue(returnValue,(VOID *) AddSymbol(""));
      return;
     }

   /*======================================================*/
   /* Allocate temporary memory and then copy the original */
   /* string or symbol to that memory, while lowercasing   */
   /* upper case alphabetic characters.                    */
   /*======================================================*/
   
   osptr = DOToString(theArg);
   slen = strlen(osptr) + 1;
   nsptr = (char *) gm2(slen);

   for (i = 0  ; i < slen ; i++)
     {
      if (isupper(osptr[i]))
        { nsptr[i] = tolower(osptr[i]); }
      else
        { nsptr[i] = osptr[i]; }
     }

   /*========================================*/
   /* Return the lowercased string and clean */
   /* up the temporary memory used.          */
   /*========================================*/

   SetpType(returnValue,GetType(theArg));
   SetpValue(returnValue,(VOID *) AddSymbol(nsptr));
   rm(nsptr,slen);
  }

/***************************************************************/
/* StrCompareFunction: CLIPS command for comparing two strings */
/*   or symbols.                                               */
/*   Syntax: (str-compare <string-1> <string-2> [<length>])    */
/***************************************************************/
globle long int StrCompareFunction()
  {
   int numArgs, length;
   DATA_OBJECT arg1, arg2, arg3;
   long returnValue;

   /*=======================================================*/
   /* Function str-compare expects either 2 or 3 arguments. */
   /*=======================================================*/

   if ((numArgs = ArgRangeCheck("str-compare",2,3)) == -1) return(0L);

   /*=============================================================*/
   /* The first two arguments should be of type symbol or string. */
   /*=============================================================*/

   if (ArgTypeCheck("str-compare",1,SYMBOL_OR_STRING,&arg1) == CLIPS_FALSE)
     { return(0L); }

   if (ArgTypeCheck("str-compare",2,SYMBOL_OR_STRING,&arg2) == CLIPS_FALSE)
     { return(0L); }

   /*===================================================*/
   /* Compare the strings. Use the 3rd argument for the */
   /* maximum length of comparison, if it is provided.  */
   /*===================================================*/

   if (numArgs == 3)
     {
      if (ArgTypeCheck("str-compare",3,INTEGER,&arg3) == CLIPS_FALSE)
        { return(0L); }

      length = CoerceToInteger(GetType(arg3),GetValue(arg3));
      returnValue = strncmp(DOToString(arg1),DOToString(arg2),length);
     }
   else
     { returnValue = strcmp(DOToString(arg1),DOToString(arg2)); }

   /*========================================================*/
   /* Return Values are as follows:                          */
   /* -1 is returned if <string-1> is less than <string-2>.  */
   /*  1 is return if <string-1> is greater than <string-2>. */
   /*  0 is returned if <string-1> is equal to <string-2>.   */
   /*========================================================*/
   
   if (returnValue < 0) returnValue = -1;
   else if (returnValue > 0) returnValue = 1;
   return(returnValue);
  }

/*****************************************************/
/* SubStringFunction:  Returns a portion of a string */
/*   and returns the pointer to a new string.        */
/*****************************************************/
globle VOID *SubStringFunction()
  {
   DATA_OBJECT val_ptr;
   char *tmp_str, *ret_str;
   int start, end, i, j;
   VOID *returnValue;

   /*===================================*/
   /* Check and retrieve the arguments. */
   /*===================================*/

   if (ArgCountCheck("sub-string",EXACTLY,3) == -1)
     { return((VOID *) AddSymbol("")); }

   if (ArgTypeCheck("sub-string",1,INTEGER,&val_ptr) == CLIPS_FALSE)
     { return((VOID *) AddSymbol("")); }

   start = CoerceToInteger(val_ptr.type,val_ptr.value) - 1;

   if (ArgTypeCheck("sub-string",2,INTEGER,&val_ptr) == CLIPS_FALSE)
     {  return((VOID *) AddSymbol("")); }

   end = CoerceToInteger(val_ptr.type,val_ptr.value) - 1;

   if (ArgTypeCheck("sub-string",3,SYMBOL_OR_STRING,&val_ptr) == CLIPS_FALSE)
     { return((VOID *) AddSymbol("")); }

   /*================================================*/
   /* If parameters are out of range return an error */
   /*================================================*/

   if (start < 0) start = 0;
   if (end > (int) strlen(DOToString(val_ptr)))
     { end = strlen(DOToString(val_ptr)); }

   /*==================================*/
   /* If the start is greater than the */
   /* end, return a null string.       */
   /*==================================*/

   if (start > end)
     { return((VOID *) AddSymbol("")); }

   /*=============================================*/
   /* Otherwise, allocate the string and copy the */
   /* designated portion of the old string to the */
   /* new string.                                 */
   /*=============================================*/

   else
     {
      ret_str = (char *) gm2(end - start +2);  /* (end - start) inclusive + EOS */
      tmp_str = DOToString(val_ptr);
      for(j=0, i=start;i <= end; i++, j++)
        { *(ret_str+j) = *(tmp_str+i); }
      *(ret_str+j) = '\0';
     }

   /*========================*/
   /* Return the new string. */
   /*========================*/

   returnValue = (VOID *) AddSymbol(ret_str);
   rm(ret_str,end - start + 2);
   return(returnValue);
  }

/**********************************************************/
/* StrIndexFunction: Returns the position of the first string in */
/*   the second. If string is not found, 0 is returned.   */
/**********************************************************/
globle VOID StrIndexFunction(result)
  DATA_OBJECT_PTR result;
  {
   DATA_OBJECT val_ptr1, val_ptr2;
   char *strg1, *strg2;
   int i, j;

   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;

   /*===================================*/
   /* Check and retrieve the arguments. */
   /*===================================*/

   if (ArgCountCheck("str-index",EXACTLY,2) == -1) return;

   if (ArgTypeCheck("str-index",1,SYMBOL_OR_STRING,&val_ptr1) == CLIPS_FALSE) return;

   if (ArgTypeCheck("str-index",2,SYMBOL_OR_STRING,&val_ptr2) == CLIPS_FALSE) return;

   strg1 = DOToString(val_ptr1);
   strg2 = DOToString(val_ptr2);

   /*=================================*/
   /* Find the position in string2 of */
   /* string1 (counting from 1).      */
   /*=================================*/

   if (strlen(strg1) == 0)
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong((long) strlen(strg2) + 1L);
      return;
     }

   for (i=1; *strg2; i++, strg2++)
     {
      for (j=0; *(strg1+j) && *(strg1+j) == *(strg2+j); j++);

      if (*(strg1+j) == '\0')
        {
         result->type = INTEGER;
         result->value = (VOID *) AddLong((long) i);
         return;
        }
     }

   return;
  }

/****************************************/
/* EVAL:  Evaluates a STRING or SYMBOL. */
/*   Syntax: (eval <string-or-symbol>)  */
/****************************************/
#if (! RUN_TIME) && (! BLOAD_ONLY)
globle VOID EvalFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DATA_OBJECT theArg;
   struct expr *top;
   int ov;
   static int depth = 0;
   char logicalNameBuffer[20];
   struct BindInfo *oldBinds;

   /*===================================================*/
   /* Function eval expects exactly one argument. */
   /*===================================================*/

   if (ArgCountCheck("eval",EXACTLY,1) == -1)
     {
      SetpType(returnValue,SYMBOL);
      SetpValue(returnValue,CLIPSFalseSymbol);
      return;
     }

   /*================================================*/
   /* The argument should be of type SYMBOL or STRING. */
   /*================================================*/

   if (ArgTypeCheck("eval",1,SYMBOL_OR_STRING,&theArg) == CLIPS_FALSE)
     {
      SetpType(returnValue,SYMBOL);
      SetpValue(returnValue,CLIPSFalseSymbol);
      return;
     }

   /*======================================================*/
   /* Evaluate the string. Create a different logical name */
   /* for use each time the eval function is called.       */
   /*======================================================*/

   depth++;
   sprintf(logicalNameBuffer,"Eval-%d",depth);
   if (OpenStringSource(logicalNameBuffer,DOToString(theArg),0) == 0)
     {
      SetpType(returnValue,SYMBOL);
      SetpValue(returnValue,CLIPSFalseSymbol);
      depth--;
      return;
     }

   ov = GetPPBufferStatus();
   SetPPBufferStatus(CLIPS_FALSE);
   oldBinds = GetParsedBindNames();
   SetParsedBindNames(NULL);

   top = ParseAtomOrExpression(logicalNameBuffer,NULL);

   SetPPBufferStatus(ov);
   ClearParsedBindNames();
   SetParsedBindNames(oldBinds);

   if (top == NULL)
     {
      SetEvaluationError(CLIPS_TRUE);
      CloseStringSource(logicalNameBuffer);
      SetpType(returnValue,SYMBOL);
      SetpValue(returnValue,CLIPSFalseSymbol);
      depth--;
      return;
     }
     
   if (top->type == MF_GBL_VARIABLE)
     {
      PrintErrorID("MISCFUN",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"expand$ must be used in the argument list of a function call.\n");
      SetEvaluationError(CLIPS_TRUE);
      CloseStringSource(logicalNameBuffer);
      SetpType(returnValue,SYMBOL);
      SetpValue(returnValue,CLIPSFalseSymbol);
      ReturnExpression(top);
      depth--;
      return;
     }
     
   if (ExpressionContainsVariables(top,CLIPS_FALSE))
     {      
      PrintErrorID("STRNGFUN",2,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Some variables could not be accessed by the eval function.\n");
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      SetpType(returnValue,SYMBOL);
      SetpValue(returnValue,CLIPSFalseSymbol);
     }
   else
     { EvaluateExpression(top,returnValue); }

   depth--;
   ReturnExpression(top);
   CloseStringSource(logicalNameBuffer);
  }
#else
globle VOID EvalFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   PrintErrorID("STRNGFUN",1,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Function eval does not work in run time modules.\n");
   SetpType(returnValue,SYMBOL);
   SetpValue(returnValue,CLIPSFalseSymbol);
  }
#endif

/*****************************************/
/* BUILD:  Evaluates a STRING or SYMBOL. */
/*   Syntax: (build <string-or-symbol>)  */
/*****************************************/
#if (! RUN_TIME) && (! BLOAD_ONLY)
globle int BuildFunction()
  {
   char *command_name;
   struct token com_tkn;
   DATA_OBJECT theArg;
   int errorFlag;

   /*==============================================*/
   /* Function build expects exactly one argument. */
   /*==============================================*/

   if (ArgCountCheck("build",EXACTLY,1) == -1) return(CLIPS_FALSE);

   /*==================================================*/
   /* The argument should be of type SYMBOL or STRING. */
   /*==================================================*/

   if (ArgTypeCheck("build",1,SYMBOL_OR_STRING,&theArg) == CLIPS_FALSE)
     { return(CLIPS_FALSE); }

   /*====================================================*/
   /* No additions during defrule join network activity. */
   /*====================================================*/

#if DEFRULE_CONSTRUCT
   if (JoinOperationInProgress) return(CLIPS_FALSE);
#endif

   /*======================*/
   /* Evaluate the string. */
   /*======================*/

   if (OpenStringSource("build",DOToString(theArg),0) == 0)
     { return(CLIPS_FALSE); }

   GetToken("build",&com_tkn);

   if (com_tkn.type != LPAREN)
     {
      CloseStringSource("build");
      return(CLIPS_FALSE);
     }

   GetToken("build",&com_tkn);
   if (com_tkn.type != SYMBOL)
     {
      CloseStringSource("build");
      return(CLIPS_FALSE);
     }

   command_name = ValueToString(com_tkn.value);

   errorFlag = ParseConstruct(command_name,"build");

   CloseStringSource("build");

   if (errorFlag == 1)
     {
      PrintCLIPS(WERROR,"\nERROR:\n");
      PrintInChunks(WERROR,GetPPBuffer());
      PrintCLIPS(WERROR,"\n");
     }

   DestroyPPBuffer();

   if (errorFlag == 0) return(CLIPS_TRUE);

   return(CLIPS_FALSE);
  }
#else
globle int BuildFunction()
  {
   PrintErrorID("STRNGFUN",1,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Function build does not work in run time modules.\n");
   return(CLIPS_FALSE);
  }
#endif
#endif
