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
/* Purpose: Object System Construct Compiler Code            */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_objcmp
#define _H_objcmp

#ifndef _CLIPS_STDIO_
#include <stdio.h>
#define _CLIPS_STDIO_
#endif

#ifndef _H_object
#include "object.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _OBJCMP_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
LOCALE VOID SetupObjectsCompiler(void);
LOCALE VOID PrintClassReference(FILE *,DEFCLASS *,int,int);
LOCALE VOID DefclassCModuleReference(FILE *,int,int,int);
#else
LOCALE VOID SetupObjectsCompiler();
LOCALE VOID PrintClassReference();
LOCALE VOID DefclassCModuleReference();
#endif

#endif

