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
/* Purpose: Generic Function Construct Compiler Code         */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_genrccmp
#define _H_genrccmp

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _GENRCCMP_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#include "genrcfun.h"

#if ANSI_COMPILER
LOCALE VOID SetupGenericsCompiler(void);
LOCALE VOID PrintGenericFunctionReference(FILE *,DEFGENERIC *,int,int);
LOCALE VOID DefgenericCModuleReference(FILE *,int,int,int);
#else
LOCALE VOID SetupGenericsCompiler();
LOCALE VOID PrintGenericFunctionReference();
LOCALE VOID DefgenericCModuleReference();
#endif

#endif

