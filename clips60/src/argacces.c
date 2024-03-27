   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*               ARGUMENT ACCESS MODULE                */
   /*******************************************************/

/*************************************************************/
/* Purpose: Provides access routines for accessing arguments */
/*   passed to user or system functions defined using the    */
/*   DefineFunction protocol for extending CLIPS.            */
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

#define _ARGACCES_SOURCE_

#include "setup.h"

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>
#include <ctype.h>
#if ANSI_COMPILER
#include <stdlib.h>
#endif

#include "extnfunc.h"
#include "router.h"
#include "cstrnchk.h"
#include "prntutil.h"

#include "argacces.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                    NonexistantError(char *,char *,int);
   static VOID                    ExpectedTypeError3(char *,char *,int,char *);
#else
   static VOID                    NonexistantError();
   static VOID                    ExpectedTypeError3();
#endif

/****************************************************************/
/* RtnLexeme: Access function to retrieve the nth argument from */
/*   a user or system function defined using the DefineFunction */
/*   protocol. The argument retrieved must be a symbol, string, */
/*   or instance name, otherwise an error is generated. Only    */
/*   the value of the argument is returned (i.e. the string "a" */
/*   would be returned for a, "a", and [a]).                    */
/****************************************************************/
globle char *RtnLexeme(argumentPosition)
  int argumentPosition;
  {
   int count = 1;
   DATA_OBJECT result;
   struct expr *argPtr;

   /*=====================================================*/
   /* Find the appropriate argument in the argument list. */
   /*=====================================================*/

   argPtr = CurrentExpression->argList;
   while ((argPtr != NULL) && (count < argumentPosition))
     {
      count++;
      argPtr = argPtr->nextArg;
     }

   if (argPtr == NULL)
     {
      NonexistantError("RtnLexeme",
                       ValueToString(ExpressionFunctionCallName(CurrentExpression)),
                       argumentPosition);
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      return(NULL);
     }

   /*============================================*/
   /* Return the value of the nth argument if it */
   /* is a symbol, string, or instance name.     */
   /*============================================*/

   EvaluateExpression(argPtr,&result);
   
   if ((result.type == SYMBOL) ||
#if OBJECT_SYSTEM
       (result.type == INSTANCE_NAME) ||
#endif
       (result.type == STRING))
     { return(ValueToString(result.value));}

   /*======================================================*/
   /* Generate an error if the argument is the wrong type. */
   /*======================================================*/
   
   ExpectedTypeError3("RtnLexeme",
                  ValueToString(ExpressionFunctionCallName(CurrentExpression)),
                  argumentPosition,"symbol, string, or instance name");
   SetHaltExecution(CLIPS_TRUE);
   SetEvaluationError(CLIPS_TRUE);
   return(NULL);
  }

/****************************************************************/
/* RtnDouble: Access function to retrieve the nth argument from */
/*   a user or system function defined using the DefineFunction */
/*   protocol. The argument retrieved must be a either a float  */
/*   or an integer (type conversion to a float is performed for */
/*   integers), otherwise an error is generated. Only the value */
/*   of the argument is returned (i.e. the float 3.0 would be   */
/*   returned for 3.0 and 3).                                   */
/****************************************************************/
globle double RtnDouble(argumentPosition)
  int argumentPosition;
  {
   int count = 1;
   DATA_OBJECT result;
   struct expr *argPtr;

   /*=====================================================*/
   /* Find the appropriate argument in the argument list. */
   /*=====================================================*/

   argPtr = CurrentExpression->argList;
   while ((argPtr != NULL) && (count < argumentPosition))
     {
      count++;
      argPtr = argPtr->nextArg;
     }

   if (argPtr == NULL)
     {
      NonexistantError("RtnDouble",
                       ValueToString(ExpressionFunctionCallName(CurrentExpression)),
                       argumentPosition);
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      return(1.0);
     }

   /*======================================*/
   /* Return the value of the nth argument */
   /* if it is a float or integer.         */
   /*======================================*/

   EvaluateExpression(argPtr,&result);

   if (result.type == FLOAT)
     { return(ValueToDouble(result.value)); }
   else if (result.type == INTEGER)
     { return((double) ValueToLong(result.value)); }
   
   /*======================================================*/
   /* Generate an error if the argument is the wrong type. */
   /*======================================================*/
   
   ExpectedTypeError3("RtnDouble",
                  ValueToString(ExpressionFunctionCallName(CurrentExpression)),
                  argumentPosition,"number");
   SetHaltExecution(CLIPS_TRUE);
   SetEvaluationError(CLIPS_TRUE);
   return(1.0);
  }

/****************************************************************/
/* RtnLong: Access function to retrieve the nth argument from   */
/*   a user or system function defined using the DefineFunction */
/*   protocol. The argument retrieved must be a either a float  */
/*   or an integer (type conversion to an integer is performed  */
/*   for floats), otherwise an error is generated. Only the     */
/*   value of the argument is returned (i.e. the integer 4      */
/*   would be returned for 4.3 and 4).                          */
/****************************************************************/
globle long RtnLong(argumentPosition)
  int argumentPosition;
  {
   int count = 1;
   DATA_OBJECT result;
   struct expr *argPtr;

   /*=====================================================*/
   /* Find the appropriate argument in the argument list. */
   /*=====================================================*/

   argPtr = CurrentExpression->argList;
   while ((argPtr != NULL) && (count < argumentPosition))
     {
      count++;
      argPtr = argPtr->nextArg;
     }

   if (argPtr == NULL)
     {
      NonexistantError("RtnLong",
                       ValueToString(ExpressionFunctionCallName(CurrentExpression)),
                       argumentPosition);
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      return(1L);
     }

   /*======================================*/
   /* Return the value of the nth argument */
   /* if it is a float or integer.         */
   /*======================================*/

   EvaluateExpression(argPtr,&result);

   if (result.type == FLOAT)
     { return((long) ValueToDouble(result.value)); }
   else if (result.type == INTEGER)
     { return(ValueToLong(result.value)); }
     
   /*======================================================*/
   /* Generate an error if the argument is the wrong type. */
   /*======================================================*/
   
   ExpectedTypeError3("RtnLong",
                  ValueToString(ExpressionFunctionCallName(CurrentExpression)),
                  argumentPosition,"number");
   SetHaltExecution(CLIPS_TRUE);
   SetEvaluationError(CLIPS_TRUE);
   return(1L);
  }

/******************************************************************/
/* RtnUnknown: Access function to retrieve the nth argument from  */
/*   a user or system function defined using the DefineFunction   */
/*   protocol. The argument retrieved can be of any type. The     */
/*   value and type of the argument are returned in a DATA_OBJECT */
/*   structure provided by the calling function.                  */
/******************************************************************/
globle DATA_OBJECT_PTR RtnUnknown(argumentPosition,returnValue)
  int argumentPosition;
  DATA_OBJECT_PTR returnValue;
  {
   static int count;
   static struct expr *argPtr;

   /*=====================================================*/
   /* Find the appropriate argument in the argument list. */
   /*=====================================================*/

   count = 1;
   argPtr = CurrentExpression->argList;
   while ((argPtr != NULL) && (count < argumentPosition))
     {
      count++;
      argPtr = argPtr->nextArg;
     }

   if (argPtr == NULL)
     {
      NonexistantError("RtnUnknown",
                       ValueToString(ExpressionFunctionCallName(CurrentExpression)),
                       argumentPosition);
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      return(NULL);
     }

   /*=======================================*/
   /* Return the value of the nth argument. */
   /*=======================================*/

   EvaluateExpression(argPtr,returnValue);
   return(returnValue);
  }

/********************************************************/
/* RtnArgCount: Returns the length of the argument list */
/*   for the function call currently being evaluated.   */
/********************************************************/
globle int RtnArgCount()
  {
   int count = 0;
   struct expr *argPtr;

   argPtr = CurrentExpression->argList;

   while (argPtr != NULL)
     {
      count++;
      argPtr = argPtr->nextArg;
     }

   return(count);
  }

/**********************************************************************/
/* ArgCountCheck: Given the expected number of arguments, determines  */
/*   if the function currently being evaluated has the correct number */
/*   of arguments. Three types of argument checking are provided by   */
/*   this function: 1) The function has exactly the expected number   */
/*   of arguments; 2) The function has at least the expected number   */
/*   of arguments; 3) The function has at most the expected number of */
/*   arguments. The number of arguments is returned if no error       */
/*   occurs, otherwise -1 is returned.                                */
/**********************************************************************/
globle int ArgCountCheck(functionName,countRelation,expectedNumber)
  char *functionName;
  int countRelation, expectedNumber;
  {
   int numberOfArguments;

   /*==============================================*/
   /* Get the number of arguments for the function */
   /* currently being evaluated.                   */
   /*==============================================*/
   
   numberOfArguments = RtnArgCount();
   
   /*=========================================================*/
   /* If the function satisfies expected number of arguments, */
   /* constraint, then return the number of arguments found.  */
   /*=========================================================*/
   
   if (countRelation == EXACTLY)
     { if (numberOfArguments == expectedNumber) return(numberOfArguments); }
   else if (countRelation == AT_LEAST)
     { if (numberOfArguments >= expectedNumber) return(numberOfArguments); }
   else if (countRelation == NO_MORE_THAN)
     { if (numberOfArguments <= expectedNumber) return(numberOfArguments); }

   /*================================================*/
   /* The correct number of arguments was not found. */
   /* Generate an error message and return -1.       */
   /*================================================*/
   
   ExpectedCountError(functionName,countRelation,expectedNumber);

   SetHaltExecution(CLIPS_TRUE);
   SetEvaluationError(CLIPS_TRUE);
   
   return(-1);
  }

/****************************************************************/
/* ArgRangeCheck: Checks that the number of arguments passed to */
/*   a function falls within a specified minimum and maximum    */
/*   range. The number of arguments passed to the function is   */
/*   returned if no error occurs, otherwise -1 is returned.     */
/****************************************************************/
globle int ArgRangeCheck(functionName,min,max)
  char *functionName;
  int min, max;
  {
   int numberOfArguments;

   numberOfArguments = RtnArgCount();
   if ((numberOfArguments < min) || (numberOfArguments > max))
     {
      PrintErrorID("ARGACCES",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Function ");
      PrintCLIPS(WERROR,functionName);
      PrintCLIPS(WERROR," expected at least ");
      PrintLongInteger(WERROR,(long) min);
      PrintCLIPS(WERROR," and no more than ");
      PrintLongInteger(WERROR,(long) max);
      PrintCLIPS(WERROR," arguments.\n");
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      return(-1);
     }

   return(numberOfArguments);
  }

/*******************************************************************/
/* ArgTypeCheck: Retrieves the nth argument passed to the function */
/*   call currently being evaluated and determines if it matches a */
/*   specified type. Returns TRUE if the argument was successfully */
/*   retrieved and is of the appropriate type, otherwise returns   */
/*   FALSE.                                                        */
/*******************************************************************/
globle int ArgTypeCheck(functionName,argumentPosition,expectedType,returnValue)
  char *functionName;
  int expectedType;
  int argumentPosition;
  DATA_OBJECT_PTR returnValue;
  {
   /*========================*/
   /* Retrieve the argument. */
   /*========================*/
   
   RtnUnknown(argumentPosition,returnValue);
   if (EvaluationError) return(CLIPS_FALSE);

   /*========================================*/
   /* If the argument's type exactly matches */
   /* the expected type, then return TRUE.   */
   /*========================================*/
   
   if (returnValue->type == expectedType) return (CLIPS_TRUE);
   
   /*=============================================================*/
   /* Some expected types encompass more than one primitive type. */
   /* If the argument's type matches one of the primitive types   */
   /* encompassed by the expected type, then return TRUE.         */
   /*=============================================================*/
   
   if ((expectedType == INTEGER_OR_FLOAT) &&
       ((returnValue->type == INTEGER) || (returnValue->type == FLOAT)))
     { return(CLIPS_TRUE); }

   if ((expectedType == SYMBOL_OR_STRING) &&
       ((returnValue->type == SYMBOL) || (returnValue->type == STRING)))
     { return(CLIPS_TRUE); }

#if OBJECT_SYSTEM
   if (((expectedType == SYMBOL_OR_STRING) || (expectedType == SYMBOL)) &&
       (returnValue->type == INSTANCE_NAME))
     { return(CLIPS_TRUE); }

   if ((expectedType == INSTANCE_NAME) &&
       ((returnValue->type == INSTANCE_NAME) || (returnValue->type == SYMBOL)))
     { return(CLIPS_TRUE); }

   if ((expectedType == INSTANCE_OR_INSTANCE_NAME) &&
       ((returnValue->type == INSTANCE_ADDRESS) || 
        (returnValue->type == INSTANCE_NAME) ||
        (returnValue->type == SYMBOL)))
     { return(CLIPS_TRUE); }
#endif

   /*===========================================================*/
   /* If the expected type is float and the argument's type is  */
   /* integer (or vice versa), then convert the argument's type */
   /* to match the expected type and then return TRUE.          */
   /*===========================================================*/
   
   if ((returnValue->type == INTEGER) && (expectedType == FLOAT))
     {
      returnValue->type = FLOAT;
      returnValue->value = (VOID *) AddDouble((double) ValueToLong(returnValue->value));
      return(CLIPS_TRUE);
     }

   if ((returnValue->type == FLOAT) && (expectedType == INTEGER))
     {
      returnValue->type = INTEGER;
      returnValue->value = (VOID *) AddLong((long) ValueToDouble(returnValue->value));
      return(CLIPS_TRUE);
     }

   /*=====================================================*/
   /* The argument's type didn't match the expected type. */
   /* Print an error message and return FALSE.            */
   /*=====================================================*/
   
   if (expectedType == FLOAT) ExpectedTypeError1(functionName,argumentPosition,"float");
   else if (expectedType == INTEGER) ExpectedTypeError1(functionName,argumentPosition,"integer");
   else if (expectedType == SYMBOL) ExpectedTypeError1(functionName,argumentPosition,"symbol");
   else if (expectedType == STRING) ExpectedTypeError1(functionName,argumentPosition,"string");
   else if (expectedType == MULTIFIELD) ExpectedTypeError1(functionName,argumentPosition,"multifield");
   else if (expectedType == INTEGER_OR_FLOAT)  ExpectedTypeError1(functionName,argumentPosition,"integer or float");
   else if (expectedType == SYMBOL_OR_STRING) ExpectedTypeError1(functionName,argumentPosition,"symbol or string");
#if OBJECT_SYSTEM
   else if (expectedType == INSTANCE_NAME) ExpectedTypeError1(functionName,argumentPosition,"instance name");
   else if (expectedType == INSTANCE_ADDRESS) ExpectedTypeError1(functionName,argumentPosition,"instance address");
   else if (expectedType == INSTANCE_OR_INSTANCE_NAME) ExpectedTypeError1(functionName,argumentPosition,"instance address or instance name");
#endif

   SetHaltExecution(CLIPS_TRUE);
   SetEvaluationError(CLIPS_TRUE);
   
   return(CLIPS_FALSE);
  }

/******************************************************************/
/* GetNumericArgument: Evaluates an expression to yield a numeric */
/*  argument. This provides quicker retrieval than using some of  */
/*  the other argument access routines. The numeric argument is   */
/*  returned in a DATA_OBJECT supplied by the calling function.   */
/*  TRUE is returned if a numeric argument was successfully       */
/*  retrieved, otherwise FALSE is returned.                       */
/******************************************************************/
globle BOOLEAN GetNumericArgument(theArgument,functionName,result,convertToFloat,whichArgument)
  struct expr *theArgument;
  char *functionName;
  DATA_OBJECT *result;
  BOOLEAN convertToFloat;
  int whichArgument;
  {
   int theType;
   VOID *theValue;

   /*==================================================================*/
   /* Evaluate the expression (don't bother calling EvaluateExpression */
   /* if the type is float or integer).                                */
   /*==================================================================*/
  
   switch(theArgument->type)
     {
      case FLOAT:
      case INTEGER:
        theType = theArgument->type;
        theValue = theArgument->value;
        break;
        
      default:
        EvaluateExpression(theArgument,result);
        theType = result->type;
        theValue = result->value;
        break;
     }

   /*==========================================*/
   /* If the argument is not float or integer, */
   /* print an error message and return FALSE. */
   /*==========================================*/
   
   if ((theType != FLOAT) && (theType != INTEGER))
     {
      ExpectedTypeError1(functionName,whichArgument,"integer or float");
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      result->type = INTEGER;
      result->value = (VOID *) AddLong(0L);
      return(CLIPS_FALSE);
     }

   /*==========================================================*/
   /* If the argument is an integer and the "convert to float" */
   /* flag is TRUE, then convert the integer to a float.       */
   /*==========================================================*/
   
   if ((convertToFloat) && (theType == INTEGER))
     {
      theType = FLOAT;
      theValue = (VOID *) AddDouble((double) ValueToLong(theValue));
     }

   /*============================================================*/
   /* The numeric argument was successfully retrieved. Store the */
   /* argument in the user supplied DATA_OBJECT and return TRUE. */
   /*============================================================*/
   
   result->type = theType;
   result->value = theValue;

   return(CLIPS_TRUE);
  }
  
/*********************************************************************/
/* GetLogicalName: Retrieves the nth argument passed to the function */
/*   call currently being evaluated and determines if it is a valid  */
/*   logical name. If valid, the logical name is returned, otherwise */
/*   NULL is returned.                                               */
/*********************************************************************/
globle char *GetLogicalName(whichArgument,defaultLogicalName)
  int whichArgument;
  char *defaultLogicalName;
  {
   char *logicalName;
   DATA_OBJECT result;

   RtnUnknown(whichArgument,&result);

   if ((GetType(result) == SYMBOL) ||
       (GetType(result) == STRING) ||
       (GetType(result) == INSTANCE_NAME))
     {
      logicalName = ValueToString(result.value);
      if ((strcmp(logicalName,"t") == 0) || (strcmp(logicalName,"T") == 0))
        { logicalName = defaultLogicalName; }
     }
   else if (GetType(result) == FLOAT)
     {
      logicalName = ValueToString(AddSymbol(FloatToString(DOToDouble(result))));
     }
   else if (GetType(result) == INTEGER)
     {
      logicalName = ValueToString(AddSymbol(LongIntegerToString(DOToLong(result))));
     }
   else
     { logicalName = NULL; }

   return(logicalName);
  }

/************************************************************/
/* GetFileName: Retrieves the nth argument passed to the    */
/*   function call currently being evaluated and determines */
/*   if it is a valid file name. If valid, the file name is */
/*   returned, otherwise NULL is returned.                  */
/************************************************************/
globle char *GetFileName(functionName,whichArgument)
  char *functionName;
  int whichArgument;
  {
   DATA_OBJECT result;

   RtnUnknown(whichArgument,&result);
   if ((GetType(result) != STRING) && (GetType(result) != SYMBOL))
     {
      ExpectedTypeError1(functionName,whichArgument,"file name");
      return(NULL);
     }

   return(DOToString(result));
  }  
  
/******************************************************************/
/* OpenErrorMessage: Generalized error message for opening files. */
/******************************************************************/
globle VOID OpenErrorMessage(functionName,fileName)
  char *functionName, *fileName;
  {
   PrintErrorID("ARGACCES",2,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Function ");
   PrintCLIPS(WERROR,functionName);
   PrintCLIPS(WERROR," was unable to open file ");
   PrintCLIPS(WERROR,fileName);
   PrintCLIPS(WERROR,".\n");
  }
  
/************************************************************/
/* GetModuleName: Retrieves the nth argument passed to the  */
/*   function call currently being evaluated and determines */
/*   if it is a valid module name. If valid, the module     */
/*   name is returned or NULL is returned to indicate all   */
/*   modules.                                               */
/************************************************************/
globle struct defmodule *GetModuleName(functionName,whichArgument,error)
  char *functionName;
  int whichArgument;
  int *error;
  {
   DATA_OBJECT result;
   struct defmodule *theModule;
   
   *error = CLIPS_FALSE;
   RtnUnknown(whichArgument,&result);

   if (GetType(result) != SYMBOL)
     {
      ExpectedTypeError1(functionName,whichArgument,"defmodule name");
      *error = CLIPS_TRUE;
      return(NULL);
     }
        
   if ((theModule = (struct defmodule *) FindDefmodule(DOToString(result))) == NULL)
     {
      if (strcmp("*",DOToString(result)) != 0) 
        {
         ExpectedTypeError1(functionName,1,"defmodule name");
         *error = CLIPS_TRUE;
        }
      return(NULL);
     }
     
   return(theModule);
  }
  
/****************************************************************/
/* GetConstructName: Retrieves the 1st argument passed to the   */
/*   function call currently being evaluated and determines if  */
/*   it is a valid name for a construct. Also checks that the   */
/*   function is only passed a single argument. This routine    */
/*   is used by functions such as ppdeftemplate, undefrule,     */
/*   etc... to retrieve the construct name on which to operate. */
/****************************************************************/
globle char *GetConstructName(functionName,constructType)
  char *functionName, *constructType;
  {
   DATA_OBJECT result;

   if (RtnArgCount() != 1)
     {
      ExpectedCountError(functionName,EXACTLY,1);
      return(NULL);
     }

   RtnUnknown(1,&result);

   if (GetType(result) != SYMBOL)
     {
      ExpectedTypeError1(functionName,1,constructType);
      return(NULL);
     }

   return(DOToString(result));
  }

/**************************************************************************/
/* NonexistantError: Prints the error message for a nonexistant argument. */
/**************************************************************************/
static VOID NonexistantError(accessFunction,functionName,argumentPosition)
  char *accessFunction, *functionName;
  int argumentPosition;
  {
   PrintErrorID("ARGACCES",3,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Function ");
   PrintCLIPS(WERROR,accessFunction);
   PrintCLIPS(WERROR," received a request from function ");
   PrintCLIPS(WERROR,functionName);
   PrintCLIPS(WERROR," for argument #");
   PrintLongInteger(WERROR,(long int) argumentPosition);
   PrintCLIPS(WERROR," which is non-existent\n");
  }
 
/*********************************************************/
/* ExpectedCountError: Prints the error message for an   */
/*   incorrect number of arguments passed to a function. */
/*********************************************************/
globle VOID ExpectedCountError(functionName,countRelation,expectedNumber)
  char *functionName;
  int countRelation, expectedNumber;
  {
   PrintErrorID("ARGACCES",4,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Function ");
   PrintCLIPS(WERROR,functionName);

   if (countRelation == EXACTLY)
     { PrintCLIPS(WERROR," expected exactly "); }
   else if (countRelation == AT_LEAST)
     { PrintCLIPS(WERROR," expected at least "); }
   else if (countRelation == NO_MORE_THAN)
     { PrintCLIPS(WERROR," expected no more than "); }
   else 
     { PrintCLIPS(WERROR," generated an illegal argument check for "); }

   PrintLongInteger(WERROR,(long int) expectedNumber);
   PrintCLIPS(WERROR," argument(s)\n");
  }
  
/*************************************************************/
/*  NAME         : CheckFunctionArgCount                     */
/*  DESCRIPTION  : Checks the number of arguments against    */
/*                 the system function restriction list      */
/*  INPUTS       : 1) Name of the calling function           */
/*                 2) The restriction list can be NULL       */
/*                 3) The number of arguments                */
/*  RETURNS      : TRUE if OK, FALSE otherwise               */
/*  SIDE EFFECTS : EvaluationError set on errrors            */
/*  NOTES        : Used to check generic function implicit   */
/*                 method (system function) calls and system */
/*                 function calls which have the sequence    */
/*                 expansion operator in their argument list */ 
/*************************************************************/
globle BOOLEAN CheckFunctionArgCount(functionName,restrictions,argumentCount)
  char *functionName, *restrictions;
  int argumentCount;
  {
   register int minArguments, maxArguments;
   char theChar[2];
   
   theChar[0] = '0';
   theChar[1] = EOS;
   
   /*=====================================================*/
   /* If there are no restrictions, then there is no need */
   /* to check for the correct number of arguments.       */
   /*=====================================================*/
   
   if (restrictions == NULL) return(TRUE);
   
   /*===========================================*/
   /* Determine the minimum number of arguments */
   /* required by the function.                 */
   /*===========================================*/
   
   if (isdigit(restrictions[0]))
     {
      theChar[0] = restrictions[0];
      minArguments = atoi(theChar);
     }
   else
     { minArguments = -1; }
     
   /*===========================================*/
   /* Determine the maximum number of arguments */
   /* required by the function.                 */
   /*===========================================*/
   
   if (isdigit(restrictions[1]))
     {
      theChar[0] = restrictions[1];
      maxArguments = atoi(theChar);
     }
   else
     { maxArguments = 10000; }
     
   /*==============================================*/
   /* If the function expects exactly N arguments, */
   /* then check to see if there are N arguments.  */
   /*==============================================*/
   
   if (minArguments == maxArguments)
     {
      if (argumentCount != minArguments)
        {
         ExpectedCountError(functionName,EXACTLY,minArguments);
         SetEvaluationError(CLIPS_TRUE);
         return(CLIPS_FALSE);
        }
      return(CLIPS_TRUE);
     }
     
   /*==================================*/
   /* Check to see if there were fewer */
   /* arguments passed than expected.  */
   /*==================================*/
     
   if (argumentCount < minArguments)
     {
      ExpectedCountError(functionName,AT_LEAST,minArguments);
      SetEvaluationError(TRUE);
      return(FALSE);
     }
     
   /*=================================*/
   /* Check to see if there were more */
   /* arguments passed than expected. */
   /*=================================*/
   
   if (argumentCount > maxArguments)
     {
      ExpectedCountError(functionName,NO_MORE_THAN,maxArguments);
      SetEvaluationError(TRUE);
      return(FALSE);
     }
     
   /*===============================*/
   /* The number of arguments falls */
   /* within the expected range.    */
   /*===============================*/
   
   return(TRUE);
  }
  
/*******************************************************************/
/* ExpectedTypeError1: Prints the error message for the wrong type */
/*   of argument passed to a user or system defined function. The  */
/*   expected type is passed as a string to this function.         */
/*******************************************************************/
globle VOID ExpectedTypeError1(functionName,whichArg,expectedType)
  char *functionName;
  int whichArg;
  char *expectedType;
  {
   PrintErrorID("ARGACCES",5,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Function ");
   PrintCLIPS(WERROR,functionName);
   PrintCLIPS(WERROR," expected argument #");
   PrintLongInteger(WERROR,(long int) whichArg);
   PrintCLIPS(WERROR," to be of type ");
   PrintCLIPS(WERROR,expectedType);
   PrintCLIPS(WERROR,"\n");
  }
  
/**************************************************************/
/* ExpectedTypeError2: Prints the error message for the wrong */
/*   type of argument passed to a user or system defined      */
/*   function. The expected type is derived by examining the  */
/*   function's argument restriction list.                    */
/**************************************************************/
globle VOID ExpectedTypeError2(functionName,whichArg)
  char *functionName;
  int whichArg;
  {
   struct FunctionDefinition *theFunction;
   char *theType;
   
   theFunction = FindFunction(functionName);
   
   if (theFunction == NULL) return;
   
   theType = GetArgumentTypeName(GetNthRestriction(theFunction,whichArg));
   
   ExpectedTypeError1(functionName,whichArg,theType);
  }
  
/*******************************************************************/
/* ExpectedTypeError3: Prints the error message for the wrong type */
/*   of argument passed to a user or system defined function when  */
/*   the argument was requested by calling RtnLexeme, RtnLong, or  */
/*   RtnDouble.                                                    */
/*******************************************************************/
static VOID ExpectedTypeError3(accessFunction,functionName,argumentPosition,type)
  char *accessFunction, *functionName, *type;
  int argumentPosition;
  {
   PrintErrorID("ARGACCES",6,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Function ");
   PrintCLIPS(WERROR,accessFunction);
   PrintCLIPS(WERROR," received a request from function ");
   PrintCLIPS(WERROR,functionName);
   PrintCLIPS(WERROR," for argument #");
   PrintLongInteger(WERROR,(long int) argumentPosition);
   PrintCLIPS(WERROR," which is not of type ");
   PrintCLIPS(WERROR,type);
   PrintCLIPS(WERROR,"\n");
  }

