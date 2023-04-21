   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00 05/12/93             */
   /*                                                     */
   /*             STRING FUNCTIONS HEADER FILE            */
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

#ifndef _H_strngfun

#define _H_strngfun

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _STRNGFUN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           StringFunctionDefinitions(void);
   LOCALE VOID                           StrCatFunction(DATA_OBJECT_PTR);
   LOCALE VOID                           SymCatFunction(DATA_OBJECT_PTR);
   LOCALE long int                       StrLengthFunction(void);
   LOCALE VOID                           UpcaseFunction(DATA_OBJECT_PTR);
   LOCALE VOID                           LowcaseFunction(DATA_OBJECT_PTR);
   LOCALE long int                       StrCompareFunction(void);
   LOCALE VOID                          *SubStringFunction(void);
   LOCALE VOID                           StrIndexFunction(DATA_OBJECT_PTR);
   LOCALE VOID                           EvalFunction(DATA_OBJECT_PTR);
   LOCALE int                            BuildFunction(void);
#else
   LOCALE VOID                           StringFunctionDefinitions();
   LOCALE VOID                           StrCatFunction();
   LOCALE VOID                           SymCatFunction();
   LOCALE long int                       StrLengthFunction();
   LOCALE VOID                           UpcaseFunction();
   LOCALE VOID                           LowcaseFunction();
   LOCALE long int                       StrCompareFunction();
   LOCALE VOID                          *SubStringFunction();
   LOCALE VOID                           StrIndexFunction();
   LOCALE VOID                           EvalFunction();
   LOCALE int                            BuildFunction();
#endif

#endif






