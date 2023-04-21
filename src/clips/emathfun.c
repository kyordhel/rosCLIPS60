   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            EXTENDED MATH FUNCTIONS MODULE           */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Gary D. Riley                                        */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#include "setup.h"
#include "argacces.h"
#include "extnfunc.h"
#include "router.h"

#if EX_MATH

#include <math.h>

#ifndef PI
#define PI   3.14159265358979323846
#endif

#ifndef PID2
#define PID2 1.57079632679489661923 /* PI divided by 2 */
#endif

#define SMALLEST_ALLOWED_NUMBER 1e-15
#define dtrunc(x) (((x) < 0.0) ? ceil(x) : floor(x))
#endif

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if EX_MATH
#if ANSI_COMPILER
   static int                     SingleNumberCheck(char *,double *);
   static int                     TestProximity(double,double);
   static VOID                    DomainErrorMessage(char *);
   static VOID                    ArgumentOverflowErrorMessage(char *);
   static VOID                    SingularityErrorMessage(char *);
   static double                  clacosh(double);
   static double                  clasinh(double);
   static double                  clatanh(double);
   static double                  clasech(double);
   static double                  clacsch(double);
   static double                  clacoth(double);
#else
   static int                     SingleNumberCheck();
   static int                     TestProximity();
   static VOID                    DomainErrorMessage();
   static VOID                    ArgumentOverflowErrorMessage();
   static VOID                    SingularityErrorMessage();
   static double                  clacosh();
   static double                  clasinh();
   static double                  clatanh();
   static double                  clasech();
   static double                  clacsch();
   static double                  clacoth();
#endif
#endif

/****************************************/
/* GLOBAL INTERNAL FUNCTION DEFINITIONS */
/****************************************/

#if ANSI_COMPILER
   VOID                           ExtendedMathFunctionDefinitions(void);
#if EX_MATH
   double                         CosFunction(void);
   double                         SinFunction(void);
   double                         TanFunction(void);
   double                         SecFunction(void);
   double                         CscFunction(void);
   double                         CotFunction(void);
   double                         AcosFunction(void);
   double                         AsinFunction(void);
   double                         AtanFunction(void);
   double                         AsecFunction(void);
   double                         AcscFunction(void);
   double                         AcotFunction(void);
   double                         CoshFunction(void);
   double                         SinhFunction(void);
   double                         TanhFunction(void);
   double                         SechFunction(void);
   double                         CschFunction(void);
   double                         CothFunction(void);
   double                         AcoshFunction(void);
   double                         AsinhFunction(void);
   double                         AtanhFunction(void);
   double                         AsechFunction(void);
   double                         AcschFunction(void);
   double                         AcothFunction(void);
   long                           RoundFunction(void);
   VOID                           ModFunction(DATA_OBJECT_PTR);
   double                         ExpFunction(void);
   double                         LogFunction(void);
   double                         Log10Function(void);
   double                         SqrtFunction(void);
   double                         PiFunction(void);
   double                         DegToRadFunction(void);
   double                         RadToDegFunction(void);
   double                         DegToGradFunction(void);
   double                         GradToDegFunction(void);
   double                         PowFunction(void);
#endif
#else
   VOID                           ExtendedMathFunctionDefinitions();
#if EX_MATH
   double                         CosFunction();
   double                         SinFunction();
   double                         TanFunction();
   double                         SecFunction();
   double                         CscFunction();
   double                         CotFunction();
   double                         AcosFunction();
   double                         AsinFunction();
   double                         AtanFunction();
   double                         AsecFunction();
   double                         AcscFunction();
   double                         AcotFunction();
   double                         CoshFunction();
   double                         SinhFunction();
   double                         TanhFunction();
   double                         SechFunction();
   double                         CschFunction();
   double                         CothFunction();
   double                         AcoshFunction();
   double                         AsinhFunction();
   double                         AtanhFunction();
   double                         AsechFunction();
   double                         AcschFunction();
   double                         AcothFunction();
   long                           RoundFunction();
   VOID                           ModFunction();
   double                         ExpFunction();
   double                         LogFunction();
   double                         Log10Function();
   double                         SqrtFunction();
   double                         PiFunction();
   double                         DegToRadFunction();
   double                         RadToDegFunction();
   double                         DegToGradFunction();
   double                         GradToDegFunction();
   double                         PowFunction();
#endif
#endif

#if ! RUN_TIME
globle VOID ExtendedMathFunctionDefinitions()
  {
#if   EX_MATH
   DefineFunction2("cos",      'd', PTIF CosFunction,      "CosFunction", "11n");
   DefineFunction2("sin",      'd', PTIF SinFunction,      "SinFunction", "11n");
   DefineFunction2("tan",      'd', PTIF TanFunction,      "TanFunction", "11n");
   DefineFunction2("sec",      'd', PTIF SecFunction,      "SecFunction", "11n");
   DefineFunction2("csc",      'd', PTIF CscFunction,      "CscFunction", "11n");
   DefineFunction2("cot",      'd', PTIF CotFunction,      "CotFunction", "11n");
   DefineFunction2("acos",     'd', PTIF AcosFunction,     "AcosFunction", "11n");
   DefineFunction2("asin",     'd', PTIF AsinFunction,     "AsinFunction", "11n");
   DefineFunction2("atan",     'd', PTIF AtanFunction,     "AtanFunction", "11n");
   DefineFunction2("asec",     'd', PTIF AsecFunction,     "AsecFunction", "11n");
   DefineFunction2("acsc",     'd', PTIF AcscFunction,     "AcscFunction", "11n");
   DefineFunction2("acot",     'd', PTIF AcotFunction,     "AcotFunction", "11n");
   DefineFunction2("cosh",     'd', PTIF CoshFunction,     "CoshFunction", "11n");
   DefineFunction2("sinh",     'd', PTIF SinhFunction,     "SinhFunction", "11n");
   DefineFunction2("tanh",     'd', PTIF TanhFunction,     "TanhFunction", "11n");
   DefineFunction2("sech",     'd', PTIF SechFunction,     "SechFunction", "11n");
   DefineFunction2("csch",     'd', PTIF CschFunction,     "CschFunction", "11n");
   DefineFunction2("coth",     'd', PTIF CothFunction,     "CothFunction", "11n");
   DefineFunction2("acosh",    'd', PTIF AcoshFunction,    "AcoshFunction", "11n");
   DefineFunction2("asinh",    'd', PTIF AsinhFunction,    "AsinhFunction", "11n");
   DefineFunction2("atanh",    'd', PTIF AtanhFunction,    "AtanhFunction", "11n");
   DefineFunction2("asech",    'd', PTIF AsechFunction,    "AsechFunction", "11n");
   DefineFunction2("acsch",    'd', PTIF AcschFunction,    "AcschFunction", "11n");
   DefineFunction2("acoth",    'd', PTIF AcothFunction,    "AcothFunction", "11n");

   DefineFunction2("mod",      'n', PTIF ModFunction,      "ModFunction", "22n");
   DefineFunction2("exp",      'd', PTIF ExpFunction,      "ExpFunction", "11n");
   DefineFunction2("log",      'd', PTIF LogFunction,      "LogFunction", "11n");
   DefineFunction2("log10",    'd', PTIF Log10Function,    "Log10Function", "11n");
   DefineFunction2("sqrt",     'd', PTIF SqrtFunction,     "SqrtFunction", "11n");
   DefineFunction2("pi",       'd', PTIF PiFunction,       "PiFunction", "00");
   DefineFunction2("deg-rad",  'd', PTIF DegToRadFunction, "DegToRadFunction", "11n");
   DefineFunction2("rad-deg",  'd', PTIF RadToDegFunction, "RadToDegFunction", "11n");
   DefineFunction2("deg-grad", 'd', PTIF DegToGradFunction,"DegToGradFunction", "11n");
   DefineFunction2("grad-deg", 'd', PTIF GradToDegFunction,"GradToDegFunction", "11n");
   DefineFunction2("**",       'd', PTIF PowFunction,      "PowFunction", "22n");
   DefineFunction2("round",    'l', PTIF RoundFunction,    "RoundFunction", "11n");
#endif
  }
#endif

#if   EX_MATH

/*******************************************/
/*   CLIPS TRIGONOMETRIC FUNCTIONS         */
/*******************************************/

static int SingleNumberCheck(fnstr,num_ptr)
  char *fnstr;
  double *num_ptr;
  {
   DATA_OBJECT valstruct;

   if (ArgCountCheck(fnstr,EXACTLY,1) == -1) return(CLIPS_FALSE);
   if (ArgTypeCheck(fnstr,1,FLOAT,&valstruct) == CLIPS_FALSE) return(CLIPS_FALSE);

   *num_ptr = DOToDouble(valstruct);
   return(CLIPS_TRUE);
  }

static int TestProximity(nval,range)
  double nval, range;
  {
   if ((nval >= (- range)) && (nval <= range)) return CLIPS_TRUE;
   else return CLIPS_FALSE;
  }

static VOID DomainErrorMessage(functionName)
  char *functionName;
  {
   PrintErrorID("EMATHFUN",1,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Domain error for ");
   PrintCLIPS(WERROR,functionName);
   PrintCLIPS(WERROR," function.\n");
   SetHaltExecution(CLIPS_TRUE);
   SetEvaluationError(CLIPS_TRUE);
  }

static VOID ArgumentOverflowErrorMessage(functionName)
  char *functionName;
  {
   PrintErrorID("EMATHFUN",2,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Argument overflow for ");
   PrintCLIPS(WERROR,functionName);
   PrintCLIPS(WERROR," function.\n");
   SetHaltExecution(CLIPS_TRUE);
   SetEvaluationError(CLIPS_TRUE);
  }

static VOID SingularityErrorMessage(functionName)
  char *functionName;
  {
   PrintErrorID("EMATHFUN",3,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Singularity at asymptote in ");
   PrintCLIPS(WERROR,functionName);
   PrintCLIPS(WERROR," function.\n");
   SetHaltExecution(CLIPS_TRUE);
   SetEvaluationError(CLIPS_TRUE);
  }

globle double CosFunction()
  {
   double num;

   if (SingleNumberCheck("cos",&num) == CLIPS_FALSE) return(0.0);
   return(cos(num));
  }

globle double SinFunction()
  {
   double num;

   if (SingleNumberCheck("sin",&num) == CLIPS_FALSE) return(0.0);
   return(sin(num));
  }

globle double TanFunction()
  {
   double num, tv;

   if (SingleNumberCheck("tan",&num) == CLIPS_FALSE) return (0.0);
   tv = cos(num);
   if ((tv < SMALLEST_ALLOWED_NUMBER) && (tv > -SMALLEST_ALLOWED_NUMBER))
     {
      SingularityErrorMessage("tan");
      return(0.0);
     }

   return(sin(num) / tv);
  }

globle double SecFunction()
  {
   double num, tv;

   if (SingleNumberCheck("sec",&num) == CLIPS_FALSE) return(0.0);

   tv = cos(num);
   if ((tv < SMALLEST_ALLOWED_NUMBER) && (tv > -SMALLEST_ALLOWED_NUMBER))
     {
      SingularityErrorMessage("sec");
      return(0.0);
     }

   return(1.0 / tv);
  }

globle double CscFunction()
  {
   double num, tv;

   if (SingleNumberCheck("csc",&num) == CLIPS_FALSE) return(0.0);
   tv = sin(num);
   if ((tv < SMALLEST_ALLOWED_NUMBER) && (tv > -SMALLEST_ALLOWED_NUMBER))
     {
      SingularityErrorMessage("csc");
      return(0.0);
     }

   return(1.0 / tv);
  }

globle double CotFunction()
  {
    double num, tv;

    if (SingleNumberCheck("cot",&num) == CLIPS_FALSE) return(0.0);

    tv = sin(num);
    if ((tv < SMALLEST_ALLOWED_NUMBER) && (tv > -SMALLEST_ALLOWED_NUMBER))
      {
       SingularityErrorMessage("cot");
       return(0.0);
      }

    return(cos(num) / tv);
  }

globle double AcosFunction()
  {
   double num;

   if (SingleNumberCheck("acos",&num) == CLIPS_FALSE) return(0.0);
   if ((num > 1.0) || (num < -1.0))
     {
      DomainErrorMessage("acos");
      return(0.0);
     }
    return(acos(num));
  }

globle double AsinFunction()
  {
   double num;

   if (SingleNumberCheck("asin",&num) == CLIPS_FALSE) return(0.0);
   if ((num > 1.0) || (num < -1.0))
     {
      DomainErrorMessage("asin");
      return(0.0);
     }
   return(asin(num));
  }

globle double AtanFunction()
  {
    double num;

    if (SingleNumberCheck("atan",&num) == CLIPS_FALSE) return(0.0);
    return(atan(num));
  }

globle double AsecFunction()
  {
   double num;

   if (SingleNumberCheck("asec",&num) == CLIPS_FALSE) return(0.0);
   if ((num < 1.0) && (num > -1.0))
     {
      DomainErrorMessage("asec");
      return(0.0);
     }
    num = 1.0 / num;
    return(acos(num));
  }

globle double AcscFunction()
  {
   double num;

   if (SingleNumberCheck("acsc",&num) == CLIPS_FALSE) return(0.0);
   if ((num < 1.0) && (num > -1.0))
     {
      DomainErrorMessage("acsc");
      return(0.0);
     }
    num = 1.0 / num;
    return(asin(num));
  }

globle double AcotFunction()
  {
   double num;

   if (SingleNumberCheck("acot",&num) == CLIPS_FALSE) return(0.0);
   if (TestProximity(num,1e-25) == CLIPS_TRUE)  return(PID2);
   num = 1.0 / num;
   return(atan(num));
  }

globle double CoshFunction()
    {
    double num;

    if (SingleNumberCheck("cosh",&num) == CLIPS_FALSE) return(0.0);
    return(cosh(num));
    }


globle double SinhFunction()
    {
    double num;

    if (SingleNumberCheck("sinh",&num) == CLIPS_FALSE) return(0.0);
    return(sinh(num));
    }


globle double TanhFunction()
    {
    double num;

    if (SingleNumberCheck("tanh",&num) == CLIPS_FALSE) return(0.0);
    return(tanh(num));
    }

globle double SechFunction()
    {
     double num;

     if (SingleNumberCheck("sech",&num) == CLIPS_FALSE) return(0.0);
     return(1.0 / cosh(num));
    }

globle double CschFunction()
  {
   double num;

   if (SingleNumberCheck("csch",&num) == CLIPS_FALSE) return(0.0);
   if (num == 0.0)
     {
      SingularityErrorMessage("csch");
      return(0.0);
     }
   else if (TestProximity(num,1e-25) == CLIPS_TRUE)
     {
      ArgumentOverflowErrorMessage("csch");
      return(0.0);
     }
   return(1.0 / sinh(num));
  }

globle double CothFunction()
  {
   double num;

   if (SingleNumberCheck("coth",&num) == CLIPS_FALSE) return(0.0);
   if (num == 0.0)
     {
      SingularityErrorMessage("coth");
      return(0.0);
     }
   else if (TestProximity(num,1e-25) == CLIPS_TRUE)
     {
      ArgumentOverflowErrorMessage("coth");
      return(0.0);
     }
   return(1.0 / tanh(num));
  }

globle double AcoshFunction()
  {
   double num;

   if (SingleNumberCheck("acosh",&num) == CLIPS_FALSE) return(0.0);
   if (num < 1.0)
     {
      DomainErrorMessage("acosh");
      return(0.0);
     }
   return(clacosh(num));
  }

globle double AsinhFunction()
    {
    double num;

    if (SingleNumberCheck("asinh",&num) == CLIPS_FALSE) return(0.0);
    return(clasinh(num));
    }

globle double AtanhFunction()
  {
   double num;

   if (SingleNumberCheck("atanh",&num) == CLIPS_FALSE) return(0.0);
   if ((num >= 1.0) || (num <= -1.0))
     {
      DomainErrorMessage("atanh");
      return(0.0);
     }
   return(clatanh(num));
  }

globle double AsechFunction()
  {
   double num;

   if (SingleNumberCheck("asech",&num) == CLIPS_FALSE) return(0.0);
   if ((num > 1.0) || (num <= 0.0))
     {
      DomainErrorMessage("asech");
      return(0.0);
     }
   return(clasech(num));
  }

globle double AcschFunction()
  {
   double num;

   if (SingleNumberCheck("acsch",&num) == CLIPS_FALSE) return(0.0);
   if (num == 0.0)
     {
      DomainErrorMessage("acsch");
      return(0.0);
     }
   return(clacsch(num));
  }

globle double AcothFunction()
  {
   double num;

   if (SingleNumberCheck("acoth",&num) == CLIPS_FALSE) return(0.0);
   if ((num <= 1.0) && (num >= -1.0))
     {
      DomainErrorMessage("acoth");
      return(0.0);
     }
   return(clacoth(num));
  }

globle double ExpFunction()
  {
   double num;

   if (SingleNumberCheck("exp",&num) == CLIPS_FALSE) return(0.0);
   return(exp(num));
  }

globle double LogFunction()
  {
   double num;

   if (SingleNumberCheck("log",&num) == CLIPS_FALSE) return(0.0);
   if (num < 0.0)
     {
      DomainErrorMessage("log");
      return(0.0);
     }
   else if (num == 0.0)
     {
      ArgumentOverflowErrorMessage("log");
      return(0.0);
     }

   return(log(num));
  }

globle double Log10Function()
  {
   double num;

   if (SingleNumberCheck("log10",&num) == CLIPS_FALSE) return(0.0);
   if (num < 0.0)
     {
      DomainErrorMessage("log10");
      return(0.0);
     }
   else if (num == 0.0)
     {
      ArgumentOverflowErrorMessage("log10");
      return(0.0);
     }

    return(log10(num));
   }

globle double SqrtFunction()
  {
   double num;

   if (SingleNumberCheck("sqrt",&num) == CLIPS_FALSE) return(0.0);
   if (num < 0.00000)
     {
      DomainErrorMessage("sqrt");
      return(0.0);
     }
   return(sqrt(num));
  }

/****************************************/
/* PowFunction                       */
/****************************************/
globle double PowFunction()
  {
   DATA_OBJECT value1, value2;

   if (ArgCountCheck("**",EXACTLY,2) == -1) return(0.0);

   if (ArgTypeCheck("**",1,FLOAT,&value1) == CLIPS_FALSE) return(0.0);
   if (ArgTypeCheck("**",2,FLOAT,&value2) == CLIPS_FALSE) return(0.0);

    if (((DOToDouble(value1) == 0.0) &&
        (DOToDouble(value2) <= 0.0)) ||
       ((DOToDouble(value1) < 0.0) &&
        (dtrunc((double) DOToDouble(value2)) != DOToDouble(value2))))
     {
      DomainErrorMessage("**");
      SetHaltExecution(CLIPS_TRUE);
      SetEvaluationError(CLIPS_TRUE);
      return(0.0);
     }

   return (pow(DOToDouble(value1),DOToDouble(value2)));
  }

/*************************************/
/* ModFunction                               */
/*************************************/
globle VOID ModFunction(result)
  DATA_OBJECT_PTR result;
  {
   DATA_OBJECT item1, item2;
   double fnum1, fnum2;
   long lnum1, lnum2;

   if (ArgCountCheck("mod",EXACTLY,2) == -1)
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong(0L);
      return;
     }

   if (ArgTypeCheck("mod",1,INTEGER_OR_FLOAT,&item1) == CLIPS_FALSE)
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong(0L);
      return;
     }

   if (ArgTypeCheck("mod",2,INTEGER_OR_FLOAT,&item2) == CLIPS_FALSE)
     {
      result->type = INTEGER;
      result->value = (VOID *) AddLong(0L);
      return;
     }

   if (((item2.type == INTEGER) ? (ValueToLong(item2.value) == 0L) : CLIPS_FALSE) ||
       ((item2.type == FLOAT) ? ValueToDouble(item2.value) == 0.0 : CLIPS_FALSE))
     {
      DivideByZeroErrorMessage("mod");
      SetEvaluationError(CLIPS_TRUE);
      result->type = INTEGER;
      result->value = (VOID *) AddLong(0L);
      return;
     }

   if ((item1.type == FLOAT) || (item2.type == FLOAT))
     {
      fnum1 = CoerceToDouble(item1.type,item1.value);
      fnum2 = CoerceToDouble(item2.type,item2.value);
      result->type = FLOAT;
      result->value = (VOID *) AddDouble(fnum1 - (dtrunc(fnum1 / fnum2) * fnum2));
     }
   else
     {
      lnum1 = DOToLong(item1);
      lnum2 = DOToLong(item2);
      result->type = INTEGER;
      result->value = (VOID *) AddLong(lnum1 - (lnum1 / lnum2) * lnum2);
     }
  }

globle double PiFunction()
  {
   if (ArgCountCheck("pi",EXACTLY,0) == -1) return(acos(-1.0));
   return(acos(-1.0));
  }

globle double DegToRadFunction()
  {
   double num;

   if (SingleNumberCheck("deg-rad",&num) == CLIPS_FALSE) return(0.0);
   return(num * PI / 180.0);
  }

globle double RadToDegFunction()
  {
   double num;

   if (SingleNumberCheck("rad-deg",&num) == CLIPS_FALSE) return(0.0);
   return(num * 180.0 / PI);
  }

globle double DegToGradFunction()
  {
   double num;

   if (SingleNumberCheck("deg-grad",&num) == CLIPS_FALSE) return(0.0);
   return(num / 0.9);
  }

globle double GradToDegFunction()
  {
   double num;

   if (SingleNumberCheck("grad-deg",&num) == CLIPS_FALSE) return(0.0);
   return(num * 0.9);
  }

/****************************************/
/* RoundFunction:                            */
/****************************************/
globle long int RoundFunction()
  {
   DATA_OBJECT result;

   if (ArgCountCheck("round",EXACTLY,1) == -1)
     { return(0L); }

   if (ArgTypeCheck("round",1,INTEGER_OR_FLOAT,&result) == CLIPS_FALSE)
     { return(0L); }

   if (result.type == INTEGER)
     { return(ValueToLong(result.value)); }
   else
     { return((long) ceil(ValueToDouble(result.value) - 0.5)); }
  }

/***************************************************************************/
/*                                                                         */
/*                 SYSTEM DEPENDENT FUNCTIONS                              */
/*                                                                         */
/***************************************************************************/

static double clacosh(num)
  double num;
  {
   return(log(num + sqrt(num * num - 1.0)));
  }

static double clasinh(num)
  double num;
  {
   return(log(num + sqrt(num * num + 1.0)));
  }

static double clatanh(num)
  double num;
  {
   return((0.5) * log((1.0 + num) / (1.0 - num)));
  }

static double clasech(num)
  double num;
  {
   return(log(1.0 / num + sqrt(1.0 / (num * num) - 1.0)));
  }

static double clacsch(num)
  double num;
  {
   return(log(1.0 / num + sqrt(1.0 / (num * num) + 1.0)));
  }

static double clacoth(num)
  double num;
  {
   return((0.5) * log((num + 1.0)/(num - 1.0)));
  }


#endif
