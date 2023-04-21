   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*              PREDICATE FUNCTIONS MODULE             */
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

#define _PRDCTFUN_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_

#include "setup.h"

#include "exprnpsr.h"
#include "argacces.h"
#include "multifld.h"
#include "router.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   VOID                    PredicateFunctionDefinitions(void);
   BOOLEAN                 EqFunction(void);
   BOOLEAN                 NeqFunction(void);
   BOOLEAN                 StringpFunction(void);
   BOOLEAN                 WordpFunction(void);
   BOOLEAN                 LexemepFunction(void);
   BOOLEAN                 NumberpFunction(void);
   BOOLEAN                 FloatpFunction(void);
   BOOLEAN                 IntegerpFunction(void);
   BOOLEAN                 MultifieldpFunction(void);
   BOOLEAN                 PointerpFunction(void);
   BOOLEAN                 NotFunction(void);
   BOOLEAN                 AndFunction(void);
   BOOLEAN                 OrFunction(void);
   BOOLEAN                 LessThanOrEqualFunction(void);
   BOOLEAN                 GreaterThanOrEqualFunction(void);
   BOOLEAN                 LessThanFunction(void);
   BOOLEAN                 GreaterThanFunction(void);
   BOOLEAN                 NumericEqualFunction(void);
   BOOLEAN                 NumericNotEqualFunction(void);
   BOOLEAN                 OddpFunction(void);
   BOOLEAN                 EvenpFunction(void);
#else
   VOID                    PredicateFunctionDefinitions();
   BOOLEAN                 EqFunction();
   BOOLEAN                 NeqFunction();
   BOOLEAN                 StringpFunction();
   BOOLEAN                 WordpFunction();
   BOOLEAN                 LexemepFunction();
   BOOLEAN                 NumberpFunction();
   BOOLEAN                 FloatpFunction();
   BOOLEAN                 IntegerpFunction();
   BOOLEAN                 MultifieldpFunction();
   BOOLEAN                 PointerpFunction();
   BOOLEAN                 NotFunction();
   BOOLEAN                 AndFunction();
   BOOLEAN                 OrFunction();
   BOOLEAN                 LessThanOrEqualFunction();
   BOOLEAN                 GreaterThanOrEqualFunction();
   BOOLEAN                 LessThanFunction();
   BOOLEAN                 GreaterThanFunction();
   BOOLEAN                 NumericEqualFunction();
   BOOLEAN                 NumericNotEqualFunction();
   BOOLEAN                 OddpFunction();
   BOOLEAN                 EvenpFunction();
#endif

#if ! RUN_TIME
/**************************************************/
/* PredicateFunctionDefinitions: Defines standard */
/*   math and predicate functions.                */
/**************************************************/
globle VOID PredicateFunctionDefinitions()
  {
   DefineFunction2("not",      'b', NotFunction,                "NotFunction", "11");
   DefineFunction2("and",      'b', AndFunction,                "AndFunction", "2*");
   DefineFunction2("or",       'b', OrFunction,                 "OrFunction",  "2*");

   DefineFunction2("eq",       'b', EqFunction,                 "EqFunction",  "2*");
   DefineFunction2("neq",      'b', NeqFunction,                "NeqFunction", "2*");

   DefineFunction2("<=",       'b', LessThanOrEqualFunction,    "LessThanOrEqualFunction",  "2*n");
   DefineFunction2(">=",       'b', GreaterThanOrEqualFunction, "GreaterThanOrEqualFunction",  "2*n");
   DefineFunction2("<",        'b', LessThanFunction,           "LessThanFunction",  "2*n");
   DefineFunction2(">",        'b', GreaterThanFunction,        "GreaterThanFunction",  "2*n");
   DefineFunction2("=",        'b', NumericEqualFunction,       "NumericEqualFunction",  "2*n");
   DefineFunction2("!=",       'b', NumericNotEqualFunction,    "NumericNotEqualFunction",  "2*n");
   DefineFunction2("<>",       'b', NumericNotEqualFunction,    "NumericNotEqualFunction",  "2*n");

   DefineFunction2("wordp",    'b', WordpFunction,              "WordpFunction", "11");
   DefineFunction2("symbolp",  'b', WordpFunction,              "WordpFunction", "11");
   DefineFunction2("stringp",  'b', StringpFunction,            "StringpFunction", "11");
   DefineFunction2("lexemep",  'b', LexemepFunction,            "LexemepFunction", "11");
   DefineFunction2("numberp",  'b', NumberpFunction,            "NumberpFunction", "11");
   DefineFunction2("integerp", 'b', IntegerpFunction,           "IntegerpFunction", "11");
   DefineFunction2("floatp",   'b', FloatpFunction,             "FloatpFunction", "11");
   DefineFunction2("oddp",     'b', OddpFunction,               "OddpFunction", "11i");
   DefineFunction2("evenp",    'b', EvenpFunction,              "EvenpFunction", "11i");
   DefineFunction2("multifieldp",'b', MultifieldpFunction,      "MultifieldpFunction", "11");
   DefineFunction2("sequencep",'b', MultifieldpFunction,        "MultifieldpFunction", "11");
   DefineFunction2("pointerp", 'b', PointerpFunction,           "PointerpFunction", "11");
  }
#endif

/****************************************/
/* EqFunction:                               */
/****************************************/
globle BOOLEAN EqFunction()
  {
   DATA_OBJECT item, nextItem;
   int numArgs, i;
   struct expr *theExpression;

   numArgs = RtnArgCount();

   if (numArgs == 0)
     { return(CLIPS_FALSE); }

   theExpression = GetFirstArgument();
   EvaluateExpression(theExpression,&item);
   
   theExpression = GetNextArgument(theExpression);
   for (i = 2 ; i <= numArgs ; i++)
     {
      EvaluateExpression(theExpression,&nextItem);
      
      if (GetType(nextItem) != GetType(item))
        { return(CLIPS_FALSE); }

      if (GetType(nextItem) == MULTIFIELD)
        {
         if (MultifieldDOsEqual(&nextItem,&item) == CLIPS_FALSE)
           { return(CLIPS_FALSE); }
        }
      else if (nextItem.value != item.value)
        { return(CLIPS_FALSE); }
        
      theExpression = GetNextArgument(theExpression);
     }

   return(CLIPS_TRUE);
  }

/****************************************/
/* NeqFunction:                              */
/****************************************/
globle BOOLEAN NeqFunction()
  {
   DATA_OBJECT item, nextItem;
   int numArgs, i;
   struct expr *theExpression;

   numArgs = RtnArgCount();

   if (numArgs == 0)
     { return(CLIPS_FALSE); }

   theExpression = GetFirstArgument();
   EvaluateExpression(theExpression,&item);

   theExpression = GetNextArgument(theExpression);
   
   for (i = 2 ; i <= numArgs ; i++)
     {
      EvaluateExpression(theExpression,&nextItem);
      if (GetType(nextItem) != GetType(item))
        { /* skip this one */ }

      else if (nextItem.type == MULTIFIELD)
        {
         if (MultifieldDOsEqual(&nextItem,&item) == CLIPS_TRUE)
           { return(CLIPS_FALSE); }
        }
      else if (nextItem.value == item.value)
        { return(CLIPS_FALSE); }
        
      theExpression = GetNextArgument(theExpression);
     }

   return(CLIPS_TRUE);
  }

/****************************************/
/* StringpFunction:                             */
/****************************************/
globle BOOLEAN StringpFunction()
  {
   DATA_OBJECT item;

   if (ArgCountCheck("stringp",EXACTLY,1) == -1) return(CLIPS_FALSE);

   RtnUnknown(1,&item);

   if (GetType(item) == STRING)
     { return(CLIPS_TRUE); }
   else
     { return(CLIPS_FALSE); }
  }

/**************************************/
/* WordpFunction:                             */
/**************************************/
globle BOOLEAN WordpFunction()
  {
   DATA_OBJECT item;

   if (ArgCountCheck("symbolp",EXACTLY,1) == -1) return(CLIPS_FALSE);

   RtnUnknown(1,&item);

   if (GetType(item) == SYMBOL)
     { return(CLIPS_TRUE); }
   else
     { return(CLIPS_FALSE); }
  }

/****************************************/
/* LexemepFunction                      */
/****************************************/
globle BOOLEAN LexemepFunction()
  {
   DATA_OBJECT item;

   if (ArgCountCheck("lexemep",EXACTLY,1) == -1) return(CLIPS_FALSE);

   RtnUnknown(1,&item);

   if ((GetType(item) == SYMBOL) || (GetType(item) == STRING))
     { return(CLIPS_TRUE); }
   else
     { return(CLIPS_FALSE); }
  }


/****************************************/
/* NumberpFunction                              */
/****************************************/
globle BOOLEAN NumberpFunction()
  {
   DATA_OBJECT item;

   if (ArgCountCheck("numberp",EXACTLY,1) == -1) return(CLIPS_FALSE);

   RtnUnknown(1,&item);

   if ((GetType(item) == FLOAT) || (GetType(item) == INTEGER))
     { return(CLIPS_TRUE); }
   else
     { return(CLIPS_FALSE); }
  }

/****************************************/
/* FloatpFunction                       */
/****************************************/
globle BOOLEAN FloatpFunction()
  {
   DATA_OBJECT item;

   if (ArgCountCheck("floatp",EXACTLY,1) == -1) return(CLIPS_FALSE);

   RtnUnknown(1,&item);

   if (GetType(item) == FLOAT)
     { return(CLIPS_TRUE); }
   else
     { return(CLIPS_FALSE); }
  }


/****************************************/
/* IntegerpFunction:                               */
/****************************************/
globle BOOLEAN IntegerpFunction()
  {
   DATA_OBJECT valstruct;

   if (ArgCountCheck("integerp",EXACTLY,1) == -1) return(CLIPS_FALSE);

   RtnUnknown(1,&valstruct);

   if (GetType(valstruct) != INTEGER) return(CLIPS_FALSE);

   return(CLIPS_TRUE);
  }

/****************************************/
/* MultifieldpFunction:                           */
/****************************************/
globle BOOLEAN MultifieldpFunction()
  {
   DATA_OBJECT valstruct;

   if (ArgCountCheck("multifieldp",EXACTLY,1) == -1) return(CLIPS_FALSE);

   RtnUnknown(1,&valstruct);

   if (GetType(valstruct) != MULTIFIELD) return(CLIPS_FALSE);

   return(CLIPS_TRUE);
  }

/****************************************/
/* PointerpFunction:                           */
/****************************************/
globle BOOLEAN PointerpFunction()
  {
   DATA_OBJECT valstruct;

   if (ArgCountCheck("pointerp",EXACTLY,1) == -1) return(CLIPS_FALSE);

   RtnUnknown(1,&valstruct);

   if (GetType(valstruct) != EXTERNAL_ADDRESS) return(CLIPS_FALSE);

   return(CLIPS_TRUE);
  }

/****************************************/
/* NotFunction                          */
/****************************************/
globle BOOLEAN NotFunction()
  {
   EXPRESSION *test_ptr;
   DATA_OBJECT result;

   test_ptr = GetFirstArgument();
   if (test_ptr == NULL) { return(CLIPS_FALSE); }

   if (EvaluateExpression(test_ptr,&result)) return(CLIPS_FALSE);

   if ((result.value == CLIPSFalseSymbol) && (result.type == SYMBOL))
     { return(CLIPS_TRUE); }

   return(CLIPS_FALSE);
  }

/****************************************/
/* AndFunction                          */
/****************************************/
globle BOOLEAN AndFunction()
  {
   EXPRESSION *test_ptr;
   DATA_OBJECT result;

   test_ptr = GetFirstArgument();

   while (test_ptr != NULL)
     {
      if (EvaluateExpression(test_ptr,&result)) return(CLIPS_FALSE);
      if ((result.value == CLIPSFalseSymbol) && (result.type == SYMBOL))
        { return(CLIPS_FALSE); }

      test_ptr = GetNextArgument(test_ptr);
     }

   return(CLIPS_TRUE);
  }

/****************************************/
/* OrFunction                                   */
/****************************************/
globle BOOLEAN OrFunction()
  {
   EXPRESSION *test_ptr;
   DATA_OBJECT result;

   test_ptr = GetFirstArgument();

   while (test_ptr != NULL)
     {
      if (EvaluateExpression(test_ptr,&result)) return(CLIPS_FALSE);

      if ((result.value != CLIPSFalseSymbol) || (result.type != SYMBOL))
        { return(CLIPS_TRUE); }

      test_ptr = GetNextArgument(test_ptr);
     }

   return(CLIPS_FALSE);
  }

/****************************************/
/* LessThanOrEqualFunction              */
/****************************************/
globle BOOLEAN LessThanOrEqualFunction()
  {
   EXPRESSION *test_ptr;
   DATA_OBJECT rv1, rv2;
   int pos = 1;

   test_ptr = GetFirstArgument();

   if (test_ptr == NULL) { return(CLIPS_TRUE); }
   if (! GetNumericArgument(test_ptr,"<=",&rv1,CLIPS_FALSE,pos)) return(CLIPS_FALSE);
   test_ptr = GetNextArgument(test_ptr);
   if (test_ptr == NULL) { return(CLIPS_TRUE); }
   pos++;

   while (test_ptr != NULL)
     {
      if (! GetNumericArgument(test_ptr,"<=",&rv2,CLIPS_FALSE,pos)) return(CLIPS_FALSE);
      if (rv1.type == INTEGER)
        {
         if (rv2.type == INTEGER)
           {
            if (ValueToLong(rv1.value) > ValueToLong(rv2.value))
              { return(CLIPS_FALSE); }
           }
         else
           {
            if ((double) ValueToLong(rv1.value) > ValueToDouble(rv2.value))
              { return(CLIPS_FALSE); }
           }
        }
      else
        {
         if (rv2.type == INTEGER)
           {
            if (ValueToDouble(rv1.value) > (double) ValueToLong(rv2.value))
              { return(CLIPS_FALSE); }
           }
         else
           {
            if (ValueToDouble(rv1.value) > ValueToDouble(rv2.value))
              { return(CLIPS_FALSE); }
           }
        }

      rv1.type = rv2.type;
      rv1.value = rv2.value;
      test_ptr = GetNextArgument(test_ptr);
      pos++;
     }

   return(CLIPS_TRUE);
  }

/****************************************/
/* GreaterThanOrEqualFunction                */
/****************************************/
globle BOOLEAN GreaterThanOrEqualFunction()
  {
   EXPRESSION *test_ptr;
   DATA_OBJECT rv1, rv2;
   int pos = 1;

   test_ptr = GetFirstArgument();

   if (test_ptr == NULL) { return(CLIPS_TRUE); }
   if (! GetNumericArgument(test_ptr,">=",&rv1,CLIPS_FALSE,pos)) return(CLIPS_FALSE);
   test_ptr = GetNextArgument(test_ptr);
   if (test_ptr == NULL) { return(CLIPS_TRUE); }
   pos++;

   while (test_ptr != NULL)
     {
      if (! GetNumericArgument(test_ptr,">=",&rv2,CLIPS_FALSE,pos)) return(CLIPS_FALSE);
      if (rv1.type == INTEGER)
        {
         if (rv2.type == INTEGER)
           {
            if (ValueToLong(rv1.value) < ValueToLong(rv2.value))
              { return(CLIPS_FALSE); }
           }
         else
           {
            if ((double) ValueToLong(rv1.value) < ValueToDouble(rv2.value))
              { return(CLIPS_FALSE); }
           }
        }
      else
        {
         if (rv2.type == INTEGER)
           {
            if (ValueToDouble(rv1.value) < (double) ValueToLong(rv2.value))
              { return(CLIPS_FALSE); }
           }
         else
           {
            if (ValueToDouble(rv1.value) < ValueToDouble(rv2.value))
              { return(CLIPS_FALSE); }
           }
        }

      rv1.type = rv2.type;
      rv1.value = rv2.value;
      test_ptr = GetNextArgument(test_ptr);
      pos++;
     }

   return(CLIPS_TRUE);
  }

/****************************************/
/* LessThanFunction                            */
/****************************************/
globle BOOLEAN LessThanFunction()
  {
   EXPRESSION *test_ptr;
   DATA_OBJECT rv1, rv2;
   int pos = 1;

   test_ptr = GetFirstArgument();

   if (test_ptr == NULL) { return(CLIPS_TRUE); }
   if (! GetNumericArgument(test_ptr,"<",&rv1,CLIPS_FALSE,pos)) return(CLIPS_FALSE);
   test_ptr = GetNextArgument(test_ptr);
   if (test_ptr == NULL) { return(CLIPS_TRUE); }
   pos++;

   while (test_ptr != NULL)
     {
      if (! GetNumericArgument(test_ptr,"<",&rv2,CLIPS_FALSE,pos)) return(CLIPS_FALSE);
      if (rv1.type == INTEGER)
        {
         if (rv2.type == INTEGER)
           {
            if (ValueToLong(rv1.value) >= ValueToLong(rv2.value))
              { return(CLIPS_FALSE); }
           }
         else
           {
            if ((double) ValueToLong(rv1.value) >= ValueToDouble(rv2.value))
              { return(CLIPS_FALSE); }
           }
        }
      else
        {
         if (rv2.type == INTEGER)
           {
            if (ValueToDouble(rv1.value) >= (double) ValueToLong(rv2.value))
              { return(CLIPS_FALSE); }
           }
         else
           {
            if (ValueToDouble(rv1.value) >= ValueToDouble(rv2.value))
              { return(CLIPS_FALSE); }
           }
        }

      rv1.type = rv2.type;
      rv1.value = rv2.value;
      test_ptr = GetNextArgument(test_ptr);
      pos++;
     }

   return(CLIPS_TRUE);
  }

/*****************************************************************/
/* GreaterThanFunction: CLIPS access routine for the > function. */
/*****************************************************************/
globle BOOLEAN GreaterThanFunction()
  {
   EXPRESSION *test_ptr;
   DATA_OBJECT rv1, rv2;
   int pos = 1;

   test_ptr = GetFirstArgument();

   if (test_ptr == NULL) { return(CLIPS_TRUE); }
   if (! GetNumericArgument(test_ptr,">",&rv1,CLIPS_FALSE,pos)) return(CLIPS_FALSE);
   test_ptr = GetNextArgument(test_ptr);
   if (test_ptr == NULL) { return(CLIPS_TRUE); }
   pos++;

   while (test_ptr != NULL)
     {
      if (! GetNumericArgument(test_ptr,">",&rv2,CLIPS_FALSE,pos)) return(CLIPS_FALSE);
      if (rv1.type == INTEGER)
        {
         if (rv2.type == INTEGER)
           {
            if (ValueToLong(rv1.value) <= ValueToLong(rv2.value))
              { return(CLIPS_FALSE); }
           }
         else
           {
            if ((double) ValueToLong(rv1.value) <= ValueToDouble(rv2.value))
              { return(CLIPS_FALSE); }
           }
        }
      else
        {
         if (rv2.type == INTEGER)
           {
            if (ValueToDouble(rv1.value) <= (double) ValueToLong(rv2.value))
              { return(CLIPS_FALSE); }
           }
         else
           {
            if (ValueToDouble(rv1.value) <= ValueToDouble(rv2.value))
              { return(CLIPS_FALSE); }
           }
        }

      rv1.type = rv2.type;
      rv1.value = rv2.value;
      test_ptr = GetNextArgument(test_ptr);
      pos++;
     }

   return(CLIPS_TRUE);
  }

/****************************************/
/* NumericEqualFunction                      */
/****************************************/
globle BOOLEAN NumericEqualFunction()
  {
   EXPRESSION *test_ptr;
   DATA_OBJECT rv1, rv2;
   int pos = 1;

   test_ptr = GetFirstArgument();

   if (test_ptr == NULL) { return(CLIPS_TRUE); }
   if (! GetNumericArgument(test_ptr,"=",&rv1,CLIPS_FALSE,pos)) return(CLIPS_FALSE);
   test_ptr = GetNextArgument(test_ptr);
   if (test_ptr == NULL) { return(CLIPS_TRUE); }
   pos++;
   
   while (test_ptr != NULL)
     {
      if (! GetNumericArgument(test_ptr,"=",&rv2,CLIPS_FALSE,pos)) return(CLIPS_FALSE);
      if (rv1.type == INTEGER)
        {
         if (rv2.type == INTEGER)
           {
            if (ValueToLong(rv1.value) != ValueToLong(rv2.value))
              { return(CLIPS_FALSE); }
           }
         else
           {
            if ((double) ValueToLong(rv1.value) != ValueToDouble(rv2.value))
              { return(CLIPS_FALSE); }
           }
        }
      else
        {
         if (rv2.type == INTEGER)
           {
            if (ValueToDouble(rv1.value) != (double) ValueToLong(rv2.value))
              { return(CLIPS_FALSE); }
           }
         else
           {
            if (ValueToDouble(rv1.value) != ValueToDouble(rv2.value))
              { return(CLIPS_FALSE); }
           }
        }

      test_ptr = GetNextArgument(test_ptr);
      pos++;
     }

   return(CLIPS_TRUE);
  }


/****************************************/
/* NumericNotEqualFunction                    */
/****************************************/
globle BOOLEAN NumericNotEqualFunction()
  {
   EXPRESSION *test_ptr;
   DATA_OBJECT rv1, rv2;
   int pos = 1;

   test_ptr = GetFirstArgument();

   if (test_ptr == NULL) { return(CLIPS_TRUE); }
   if (! GetNumericArgument(test_ptr,"<>",&rv1,CLIPS_FALSE,pos)) return(CLIPS_FALSE);
   test_ptr = GetNextArgument(test_ptr);
   if (test_ptr == NULL) { return(CLIPS_TRUE); }
   pos++;

   while (test_ptr != NULL)
     {
      if (! GetNumericArgument(test_ptr,"<>",&rv2,CLIPS_FALSE,pos)) return(CLIPS_FALSE);
      if (rv1.type == INTEGER)
        {
         if (rv2.type == INTEGER)
           {
            if (ValueToLong(rv1.value) == ValueToLong(rv2.value))
              { return(CLIPS_FALSE); }
           }
         else
           {
            if ((double) ValueToLong(rv1.value) == ValueToDouble(rv2.value))
              { return(CLIPS_FALSE); }
           }
        }
      else
        {
         if (rv2.type == INTEGER)
           {
            if (ValueToDouble(rv1.value) == (double) ValueToLong(rv2.value))
              { return(CLIPS_FALSE); }
           }
         else
           {
            if (ValueToDouble(rv1.value) == ValueToDouble(rv2.value))
              { return(CLIPS_FALSE); }
           }
        }

      test_ptr = GetNextArgument(test_ptr);
      pos++;
     }

   return(CLIPS_TRUE);
  }

/****************************************/
/* OddpFunction:                                */
/****************************************/
globle BOOLEAN OddpFunction()
  {
   DATA_OBJECT valstruct;
   long num, halfnum;

   if (ArgCountCheck("oddp",EXACTLY,1) == -1) return(CLIPS_FALSE);
   if (ArgTypeCheck("oddp",1,INTEGER,&valstruct) == CLIPS_FALSE) return(CLIPS_FALSE);

   num = DOToLong(valstruct);

   halfnum = (num / 2) * 2;
   if (num == halfnum) return(CLIPS_FALSE);

   return(CLIPS_TRUE);
  }

/****************************************/
/* EvenpFunction:                               */
/****************************************/
globle BOOLEAN EvenpFunction()
  {
   DATA_OBJECT valstruct;
   long num, halfnum;

   if (ArgCountCheck("evenp",EXACTLY,1) == -1) return(CLIPS_FALSE);
   if (ArgTypeCheck("evenp",1,INTEGER,&valstruct) == CLIPS_FALSE) return(CLIPS_FALSE);

   num = DOToLong(valstruct);

   halfnum = (num / 2) * 2;
   if (num != halfnum) return(CLIPS_FALSE);

   return(CLIPS_TRUE);
  }



