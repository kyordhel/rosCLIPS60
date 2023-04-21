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

#ifndef _H_bmathfun

#define _H_bmathfun

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif
  
#ifdef _BMATHFUN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                    BasicMathFunctionDefinitions(VOID);
   LOCALE VOID                    AdditionFunction(DATA_OBJECT_PTR);
   LOCALE VOID                    MultiplicationFunction(DATA_OBJECT_PTR);
   LOCALE VOID                    SubtractionFunction(DATA_OBJECT_PTR);
   LOCALE VOID                    DivisionFunction(DATA_OBJECT_PTR);
   LOCALE long                    DivFunction(void);
   LOCALE BOOLEAN                 SetAutoFloatDividendCommand(void);
   LOCALE BOOLEAN                 GetAutoFloatDividendCommand(void);
   LOCALE BOOLEAN                 GetAutoFloatDividend(void);
   LOCALE BOOLEAN                 SetAutoFloatDividend(int);
   LOCALE long int                IntegerFunction(void);
   LOCALE double                  FloatFunction(void);
   LOCALE VOID                    AbsFunction(DATA_OBJECT_PTR);
   LOCALE VOID                    MinFunction(DATA_OBJECT_PTR);
   LOCALE VOID                    MaxFunction(DATA_OBJECT_PTR);
#else
   LOCALE VOID                    BasicMathFunctionDefinitions();
   LOCALE VOID                    AdditionFunction();
   LOCALE VOID                    MultiplicationFunction();
   LOCALE VOID                    SubtractionFunction();
   LOCALE VOID                    DivisionFunction();
   LOCALE long                    DivFunction();
   LOCALE BOOLEAN                 SetAutoFloatDividendCommand();
   LOCALE BOOLEAN                 GetAutoFloatDividendCommand();
   LOCALE BOOLEAN                 GetAutoFloatDividend();
   LOCALE BOOLEAN                 SetAutoFloatDividend();
   LOCALE long int                IntegerFunction();
   LOCALE double                  FloatFunction();
   LOCALE VOID                    AbsFunction();
   LOCALE VOID                    MinFunction();
   LOCALE VOID                    MaxFunction();
#endif

#endif


