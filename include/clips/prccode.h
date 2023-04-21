   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                                                     */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_prccode
#define _H_prccode

#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_moduldef
#include "moduldef.h"
#endif
#ifndef _H_scanner
#include "scanner.h"
#endif
#ifndef _H_symbol
#include "symbol.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _PRCCODE_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE VOID InstallProcedurePrimitives(VOID);

#if (! BLOAD_ONLY) && (! RUN_TIME)

#if DEFFUNCTION_CONSTRUCT || OBJECT_SYSTEM
LOCALE EXPRESSION *ParseProcParameters(char *,struct token *,EXPRESSION *,
                                       SYMBOL_HN **,int *,int *,int *,int (*)(char *));
#endif
LOCALE EXPRESSION *ParseProcActions(char *,char *,struct token *,EXPRESSION *,SYMBOL_HN *,
                                    int (*)(EXPRESSION *,VOID *),int (*)(EXPRESSION *,VOID *),
                                    int *,VOID *);
LOCALE BOOLEAN ReplaceProcVars(char *,EXPRESSION *,EXPRESSION *,SYMBOL_HN *,
                                     int (*)(EXPRESSION *,VOID *),VOID *);
#if DEFGENERIC_CONSTRUCT
LOCALE EXPRESSION *GenProcWildcardReference(int);
#endif
#endif

LOCALE VOID PushProcParameters(EXPRESSION *,int,char *,char *,VOID (*)(VOID_ARG));
LOCALE VOID PopProcParameters(void);

#if DEFGENERIC_CONSTRUCT
LOCALE EXPRESSION *GetProcParamExpressions(void);
#endif

LOCALE VOID EvaluateProcActions(struct defmodule *,EXPRESSION *,int,
                                DATA_OBJECT *,VOID (*)(VOID_ARG));
LOCALE VOID PrintProcParamArray(char *);
LOCALE VOID GrabProcWildargs(DATA_OBJECT *,int);

#else

LOCALE VOID InstallProcedurePrimitives();

#if (! BLOAD_ONLY) && (! RUN_TIME)
#if DEFFUNCTION_CONSTRUCT || OBJECT_SYSTEM
LOCALE EXPRESSION *ParseProcParameters();
#endif
LOCALE EXPRESSION *ParseProcActions();
LOCALE BOOLEAN ReplaceProcVars();
#if DEFGENERIC_CONSTRUCT
LOCALE EXPRESSION *GenProcWildcardReference();
#endif
#endif

LOCALE VOID PushProcParameters();
LOCALE VOID PopProcParameters();

#if DEFGENERIC_CONSTRUCT
LOCALE EXPRESSION *GetProcParamExpressions();
#endif

LOCALE VOID EvaluateProcActions();
LOCALE VOID PrintProcParamArray();
LOCALE VOID GrabProcWildargs();

#endif

#ifndef _GENRCFUN_SOURCE_
extern DATA_OBJECT *ProcParamArray;
extern int ProcParamArraySize;
#endif

#endif





