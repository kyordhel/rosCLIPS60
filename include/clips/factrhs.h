   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*        FACT RHS PATTERN PARSER HEADER MODULE        */
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

#ifndef _H_factrhs
#define _H_factrhs

#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_scanner
#include "scanner.h"
#endif
#ifndef _H_factmngr
#include "factmngr.h"
#endif
#ifndef _H_symbol
#include "symbol.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _FACTRHS_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE struct expr                   *BuildRHSAssert(char *,struct token *,int *,int,int,char *);
   LOCALE struct expr                   *GetAssertArgument(char *,struct token *,int *,int,int,int *);
   LOCALE struct expr                   *GetRHSPattern(char *,struct token *,int *,int,
                                                       int,int,int);
   LOCALE struct fact                   *StringToFact(char *);
#else
   LOCALE struct expr                   *BuildRHSAssert();
   LOCALE struct expr                   *GetAssertArgument();
   LOCALE struct expr                   *GetRHSPattern();
   LOCALE struct fact                   *StringToFact();
#endif

#endif



