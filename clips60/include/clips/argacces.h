   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             ARGUMENT ACCESS HEADER FILE             */
   /*******************************************************/

#ifndef _H_argacces

#define _H_argacces

#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_moduldef
#include "moduldef.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _ARGACCES_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE char                          *RtnLexeme(int);
   LOCALE double                         RtnDouble(int);
   LOCALE long                           RtnLong(int);
   LOCALE struct dataObject             *RtnUnknown(int,struct dataObject *);
   LOCALE int                            RtnArgCount(void);
   LOCALE int                            ArgCountCheck(char *,int,int);
   LOCALE int                            ArgRangeCheck(char *,int,int);
   LOCALE int                            ArgTypeCheck(char *,int,int,struct dataObject *);
   LOCALE BOOLEAN                        GetNumericArgument(struct expr *,char *,struct dataObject *,int,int);
   LOCALE char                          *GetLogicalName(int,char *);
   LOCALE char                          *GetFileName(char *,int);
   LOCALE char                          *GetConstructName(char *,char *);
   LOCALE VOID                           ExpectedCountError(char *,int,int);
   LOCALE VOID                           OpenErrorMessage(char *,char *);
   LOCALE BOOLEAN                        CheckFunctionArgCount(char *,char *,int);
   LOCALE VOID                           ExpectedReturnTypeError(char *,char *);
   LOCALE VOID                           ExpectedTypeError1(char *,int,char *);
   LOCALE VOID                           ExpectedTypeError2(char *,int);
   LOCALE struct defmodule              *GetModuleName(char *,int,int *);
#else
   LOCALE char                          *RtnLexeme();
   LOCALE double                         RtnDouble();
   LOCALE long                           RtnLong();
   LOCALE struct dataObject             *RtnUnknown();
   LOCALE int                            RtnArgCount();
   LOCALE int                            ArgCountCheck();
   LOCALE int                            ArgRangeCheck();
   LOCALE int                            ArgTypeCheck();
   LOCALE BOOLEAN                        GetNumericArgument();
   LOCALE char                          *GetLogicalName();
   LOCALE char                          *GetFileName();
   LOCALE char                          *GetConstructName();
   LOCALE VOID                           ExpectedCountError();
   LOCALE VOID                           OpenErrorMessage();
   LOCALE BOOLEAN                        CheckFunctionArgCount();
   LOCALE VOID                           ExpectedReturnTypeError();  
   LOCALE VOID                           ExpectedTypeError1();
   LOCALE VOID                           ExpectedTypeError2();
   LOCALE struct defmodule              *GetModuleName();
#endif

#endif






