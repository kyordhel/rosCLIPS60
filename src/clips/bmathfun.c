   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             BASIC MATH FUNCTIONS MODULE             */
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

#define _BMATHFUN_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_

#include "setup.h"

#include "exprnpsr.h"
#include "argacces.h"
#include "router.h"

#include "bmathfun.h"

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static BOOLEAN          AutoFloatDividend = CLIPS_TRUE;

/****************************************************************/
/* BasicMatchFunctionDefinitions: Defines basic math functions. */
/****************************************************************/
globle VOID BasicMathFunctionDefinitions()
  {   
#if ! RUN_TIME
   DefineFunction2("+", 'n',PTIF AdditionFunction, "AdditionFunction", "2*n");
   DefineFunction2("*", 'n', PTIF MultiplicationFunction, "MultiplicationFunction", "2*n");
   DefineFunction2("-", 'n', PTIF SubtractionFunction, "SubtractionFunction", "2*n");
   DefineFunction2("/", 'n', PTIF DivisionFunction, "DivisionFunction", "2*n");
   DefineFunction2("div", 'l', PTIF DivFunction, "DivFunction", "2*n");

   DefineFunction2("set-auto-float-dividend", 'b',
                   SetAutoFloatDividendCommand, "SetAutoFloatDividendCommand", "11");
   DefineFunction2("get-auto-float-dividend", 'b',
                  GetAutoFloatDividendCommand, "GetAutoFloatDividendCommand", "00");
                  
   DefineFunction2("integer", 'l', PTIF IntegerFunction, "IntegerFunction", "11n");
   DefineFunction2("float", 'd', PTIF FloatFunction, "FloatFunction", "11n");
   DefineFunction2("abs", 'n', PTIF AbsFunction, "AbsFunction", "11n");
   DefineFunction2("min", 'n', PTIF MinFunction, "MinFunction", "2*n");
   DefineFunction2("max", 'n', PTIF MaxFunction, "MaxFunction", "2*n");
#endif
  }

/**********************************/
/* AdditionFunction: CLIPS access */
/*   routine for the + function.  */
/**********************************/
globle VOID AdditionFunction(result)
  DATA_OBJECT_PTR result;
  {
   double ftotal = 0.0;
   long ltotal = 0L;
   BOOLEAN useFloatTotal = CLIPS_FALSE;
   EXPRESSION *test_ptr;
   DATA_OBJECT rv;
   int pos = 1;

   test_ptr = GetFirstArgument();

   while (test_ptr != NULL)
     {
      if (! GetNumericArgument(test_ptr,"+",&rv,useFloatTotal,pos)) test_ptr = NULL;
      else test_ptr = GetNextArgument(test_ptr);

      if (useFloatTotal)
        { ftotal += ValueToDouble(rv.value); }
      else
        {
         if (rv.type == INTEGER)
           { ltotal += ValueToLong(rv.value); }
         else
           {
            ftotal = (double) ltotal + ValueToDouble(rv.value);
            useFloatTotal = CLIPS_TRUE;
           }
        }

      pos++;
     }

   if (useFloatTotal)
     {
      result->type = FLOAT;
      result->value = (VOID *) AddDouble(ftotal);
     }
   else
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong(ltotal);
     }
  }

/****************************************/
/* MultiplicationFunction: CLIPS access */
/*   routine for the * function.        */
/****************************************/
globle VOID MultiplicationFunction(result)
  DATA_OBJECT_PTR result;
  {
   double ftotal = 1.0;
   long ltotal = 1L;
   BOOLEAN useFloatTotal = CLIPS_FALSE;
   EXPRESSION *test_ptr;
   DATA_OBJECT rv;
   int pos = 1;

   test_ptr = GetFirstArgument();

   while (test_ptr != NULL)
     {
      if (! GetNumericArgument(test_ptr,"*",&rv,useFloatTotal,pos)) test_ptr = NULL;
      else test_ptr = GetNextArgument(test_ptr);

      if (useFloatTotal)
        { ftotal *= ValueToDouble(rv.value); }
      else
        {
         if (rv.type == INTEGER)
           { ltotal *= ValueToLong(rv.value); }
         else
           {
            ftotal = (double) ltotal * ValueToDouble(rv.value);
            useFloatTotal = CLIPS_TRUE;
           }
        }
      pos++;
     }

   if (useFloatTotal)
     {
      result->type = FLOAT;
      result->value = (VOID *) AddDouble(ftotal);
     }
   else
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong(ltotal);
     }
  }

/*************************************/
/* SubtractionFunction: CLIPS access */
/*   routine for the - function.     */
/*************************************/
globle VOID SubtractionFunction(result)
  DATA_OBJECT_PTR result;
  {
   double ftotal = 0.0;
   long ltotal = 0L;
   BOOLEAN useFloatTotal = CLIPS_FALSE;
   EXPRESSION *test_ptr;
   DATA_OBJECT rv;
   int pos = 1;

   /*===============================================*/
   /* Get the first argument. The number which will */
   /* be divided by all subsequent numbers.         */
   /*===============================================*/

   test_ptr = GetFirstArgument();
   if (test_ptr != NULL)
     {
      if (! GetNumericArgument(test_ptr,"-",&rv,useFloatTotal,pos)) test_ptr = NULL;
      else test_ptr = GetNextArgument(test_ptr);

      if (rv.type == INTEGER)
        { ltotal = ValueToLong(rv.value); }
      else
        {
         ftotal = ValueToDouble(rv.value);
         useFloatTotal = CLIPS_TRUE;
        }
      pos++;
     }

   /*===================================*/
   /* Subtract the remaining arguments. */
   /*===================================*/

   while (test_ptr != NULL)
     {
      if (! GetNumericArgument(test_ptr,"-",&rv,useFloatTotal,pos)) test_ptr = NULL;
      else test_ptr = GetNextArgument(test_ptr);

      if (useFloatTotal)
        { ftotal -= ValueToDouble(rv.value); }
      else
        {
         if (rv.type == INTEGER)
           { ltotal -= ValueToLong(rv.value); }
         else
           {
            ftotal = (double) ltotal - ValueToDouble(rv.value);
            useFloatTotal = CLIPS_TRUE;
           }
        }
      pos++;
     }

   if (useFloatTotal)
     {
      result->type = FLOAT;
      result->value = (VOID *) AddDouble(ftotal);
     }
   else
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong(ltotal);
     }
  }

/***********************************/
/* DivisionFunction:  CLIPS access */
/*   routine for the / function.   */
/***********************************/
globle VOID DivisionFunction(result)
  DATA_OBJECT_PTR result;
  {
   double ftotal = 1.0;
   long ltotal = 1L;
   BOOLEAN useFloatTotal = AutoFloatDividend;
   EXPRESSION *test_ptr;
   DATA_OBJECT rv;
   int pos = 1;

   /*===============================================*/
   /* Get the first argument. The number which will */
   /* be divided by all subsequent numbers.         */
   /*===============================================*/

   test_ptr = GetFirstArgument();
   if (test_ptr != NULL)
     {
      if (! GetNumericArgument(test_ptr,"/",&rv,useFloatTotal,pos)) test_ptr = NULL;
      else test_ptr = GetNextArgument(test_ptr);

      if (rv.type == INTEGER)
        { ltotal = ValueToLong(rv.value); }
      else
        {
         ftotal = ValueToDouble(rv.value);
         useFloatTotal = CLIPS_TRUE;
        }
      pos++;
     }

   /*====================================*/
   /* Divide by the remaining arguments. */
   /*====================================*/

   while (test_ptr != NULL)
     {
      if (! GetNumericArgument(test_ptr,"/",&rv,useFloatTotal,pos)) test_ptr = NULL;
      else test_ptr = GetNextArgument(test_ptr);

      if ((rv.type == INTEGER) ? (ValueToLong(rv.value) == 0L) :
                                 ((rv.type == FLOAT) ? ValueToDouble(rv.value) == 0.0 : CLIPS_FALSE))
        {
         DivideByZeroErrorMessage("/");
         SetHaltExecution(CLIPS_TRUE);
         SetEvaluationError(CLIPS_TRUE);
         result->type = FLOAT;
         result->value = (VOID *) AddDouble(1.0);
         return;
        }

      if (useFloatTotal)
        { ftotal /= ValueToDouble(rv.value); }
      else
        {
         if (rv.type == INTEGER)
           { ltotal /= ValueToLong(rv.value); }
         else
           {
            ftotal = (double) ltotal / ValueToDouble(rv.value);
            useFloatTotal = CLIPS_TRUE;
           }
        }
      pos++;
     }

   if (useFloatTotal)
     {
      result->type = FLOAT;
      result->value = (VOID *) AddDouble(ftotal);
     }
   else
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong(ltotal);
     }
  }

/*************************************/
/* DivFunction: CLIPS access routine */
/*   for the div function.           */
/*************************************/
globle long DivFunction()
  {
   long total = 1L;
   EXPRESSION *test_ptr;
   DATA_OBJECT rv;
   int pos = 1;
   long theNumber;

   /*===============================================*/
   /* Get the first argument. The number which will */
   /* be divided by all subsequent numbers.         */
   /*===============================================*/

   test_ptr = GetFirstArgument();
   if (test_ptr != NULL)
     {
      if (! GetNumericArgument(test_ptr,"div",&rv,CLIPS_FALSE,pos)) test_ptr = NULL;
      else test_ptr = GetNextArgument(test_ptr);

      if (rv.type == INTEGER)
        { total = ValueToLong(rv.value); }
      else
        { total = (long) ValueToDouble(rv.value); }
      pos++;
     }

   /*====================================*/
   /* Divide by the remaining arguments. */
   /*====================================*/

   while (test_ptr != NULL)
     {
      if (! GetNumericArgument(test_ptr,"div",&rv,CLIPS_FALSE,pos)) test_ptr = NULL;
      else test_ptr = GetNextArgument(test_ptr);

      if (rv.type == INTEGER) theNumber = ValueToLong(rv.value);
      else if (rv.type == FLOAT) theNumber = (long) ValueToDouble(rv.value);
      else theNumber = 1;
      
      if (theNumber == 0L)
        {
         DivideByZeroErrorMessage("div");
         SetHaltExecution(CLIPS_TRUE);
         SetEvaluationError(CLIPS_TRUE);
         return(1L);
        }

      if (rv.type == INTEGER)
        { total /= ValueToLong(rv.value); }
      else
        { total = total / (long) ValueToDouble(rv.value); }

      pos++;
     }

   return(total);
  }

/*****************************************************/
/* SetAutoFloatDividendCommand: CLIPS access routine */
/*   for the set-auto-float-dividend command.        */
/*****************************************************/
globle int SetAutoFloatDividendCommand()
  {
   int oldValue;
   DATA_OBJECT arg_ptr;

   oldValue = AutoFloatDividend;

   if (ArgCountCheck("set-auto-float-dividend",EXACTLY,1) == -1)
     { return(oldValue); }

   RtnUnknown(1,&arg_ptr);

   if ((arg_ptr.value == CLIPSFalseSymbol) && (arg_ptr.type == SYMBOL))
     { AutoFloatDividend = CLIPS_FALSE; }
   else
     { AutoFloatDividend = CLIPS_TRUE; }

   return(oldValue);
  }

/*****************************************************/
/* GetAutoFloatDividendCommand: CLIPS access routine */
/*   for the get-auto-float-dividend command.        */
/*****************************************************/
globle int GetAutoFloatDividendCommand()
  {
   ArgCountCheck("get-auto-float-dividend",EXACTLY,0);

   return(AutoFloatDividend);
  }

/**********************************************/
/* GetAutoFloatDividend: C access routine for */
/*   the get-auto-float-dividend command.     */
/**********************************************/
globle BOOLEAN GetAutoFloatDividend()
  {
   return(AutoFloatDividend);
  }

/**********************************************/
/* SetAutoFloatDividend: C access routine for */
/*   the set-auto-float-dividend command.     */
/**********************************************/
globle BOOLEAN SetAutoFloatDividend(value)
  int value;
  {
   int ov;

   ov = AutoFloatDividend;
   AutoFloatDividend = value;
   return(ov);
  }
  
/*****************************************/
/* IntegerFunction: CLIPS access routine */
/*   for the integer function.           */
/*****************************************/
globle long int IntegerFunction()
  {
   DATA_OBJECT valstruct;

   if (ArgCountCheck("integer",EXACTLY,1) == -1) return(0L);
   if (ArgTypeCheck("integer",1,INTEGER,&valstruct) == CLIPS_FALSE) return(0L);

   return(ValueToLong(valstruct.value));
  }

/***************************************/
/* FloatFunction: CLIPS access routine */
/*   for the float function.           */
/***************************************/
globle double FloatFunction()
  {
   DATA_OBJECT valstruct;

   if (ArgCountCheck("float",EXACTLY,1) == -1) return(0.0);
   if (ArgTypeCheck("float",1,FLOAT,&valstruct) == CLIPS_FALSE) return(0.0);

   return(ValueToDouble(valstruct.value));
  }

/*************************************/
/* AbsFunction: CLIPS access routine */
/*   for the abs function.           */
/*************************************/
globle VOID AbsFunction(result)
  DATA_OBJECT_PTR result;
  {
   if (ArgCountCheck("abs",EXACTLY,1) == -1)
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong(0L);
      return;
     }

   if (ArgTypeCheck("abs",1,INTEGER_OR_FLOAT,result) == CLIPS_FALSE)
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong(0L);
      return;
     }

   if (result->type == INTEGER)
     {
      if (ValueToLong(result->value) < 0L)
        { result->value = (VOID *) AddLong(- ValueToLong(result->value)); }
     }
   else if (ValueToDouble(result->value) < 0.0)
     { result->value = (VOID *) AddDouble(- ValueToDouble(result->value)); }
  }

/*************************************/
/* MinFunction: CLIPS access routine */
/*   for the min function.           */
/*************************************/
globle VOID MinFunction(result)
  DATA_OBJECT_PTR result;
  {
   DATA_OBJECT arg_value;
   int num_a, i;

   if ((num_a = ArgCountCheck("min",AT_LEAST,1)) == -1)
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong(0L);
      return;
     }

   if (ArgTypeCheck("min",1,INTEGER_OR_FLOAT,result) == CLIPS_FALSE)
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong(0L);
      return;
     }

   for (i = 2 ; i <= num_a ; i++)
     {
      if (ArgTypeCheck("min",i,INTEGER_OR_FLOAT,&arg_value) == CLIPS_FALSE) return;

      if (result->type == INTEGER)
        {
         if (arg_value.type == INTEGER)
           {
            if (ValueToLong(result->value) > ValueToLong(arg_value.value))
              {
               result->type = arg_value.type;
               result->value = arg_value.value;
              }
           }
         else
           {
            if ((double) ValueToLong(result->value) >
                         ValueToDouble(arg_value.value))
              {
               result->type = arg_value.type;
               result->value = arg_value.value;
              }
           }
        }
      else
        {
         if (arg_value.type == INTEGER)
           {
            if (ValueToDouble(result->value) >
                (double) ValueToLong(arg_value.value))
              {
               result->type = arg_value.type;
               result->value = arg_value.value;
              }
           }
         else
           {
            if (ValueToDouble(result->value) > ValueToDouble(arg_value.value))
              {
               result->type = arg_value.type;
               result->value = arg_value.value;
              }
           }
        }
     }

   return;
  }

/*************************************/
/* MaxFunction: CLIPS access routine */
/*   for the max function.           */
/*************************************/
globle VOID MaxFunction(result)
  DATA_OBJECT_PTR result;
  {
   DATA_OBJECT arg_value;
   int num_a, i;

   if ((num_a = ArgCountCheck("max",AT_LEAST,1)) == -1)
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong(0L);
      return;
     }

   if (ArgTypeCheck("max",1,INTEGER_OR_FLOAT,result) == CLIPS_FALSE)
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong(0L);
      return;
     }

   for (i = 2 ; i <= num_a ; i++)
     {
      if (ArgTypeCheck("max",i,INTEGER_OR_FLOAT,&arg_value) == CLIPS_FALSE) return;

      if (result->type == INTEGER)
        {
         if (arg_value.type == INTEGER)
           {
            if (ValueToLong(result->value) < ValueToLong(arg_value.value))
              {
               result->type = arg_value.type;
               result->value = arg_value.value;
              }
           }
         else
           {
            if ((double) ValueToLong(result->value) <
                         ValueToDouble(arg_value.value))
              {
               result->type = arg_value.type;
               result->value = arg_value.value;
              }
           }
        }
      else
        {
         if (arg_value.type == INTEGER)
           {
            if (ValueToDouble(result->value) <
                (double) ValueToLong(arg_value.value))
              {
               result->type = arg_value.type;
               result->value = arg_value.value;
              }
           }
         else
           {
            if (ValueToDouble(result->value) < ValueToDouble(arg_value.value))
              {
               result->type = arg_value.type;
               result->value = arg_value.value;
              }
           }
        }
     }

   return;
  }

