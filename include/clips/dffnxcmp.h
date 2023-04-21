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
/* Purpose: Deffunction Construct Compiler Code              */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_dffnxcmp
#define _H_dffnxcmp

#if DEFFUNCTION_CONSTRUCT && CONSTRUCT_COMPILER && (! RUN_TIME)

#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#include "dffnxfun.h"

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _DFFNXCMP_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE VOID SetupDeffunctionCompiler(void);
LOCALE VOID PrintDeffunctionReference(FILE *,DEFFUNCTION *,int,int);
LOCALE VOID DeffunctionCModuleReference(FILE *,int,int,int);

#else

LOCALE VOID SetupDeffunctionCompiler();
LOCALE VOID PrintDeffunctionReference();
LOCALE VOID DeffunctionCModuleReference();

#endif

#endif

#endif



