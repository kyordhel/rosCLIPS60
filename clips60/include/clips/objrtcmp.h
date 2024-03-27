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

#ifndef _H_objrtcmp
#define _H_objrtcmp

#if INSTANCE_PATTERN_MATCHING && (! RUN_TIME) && CONSTRUCT_COMPILER

#ifndef _CLIPS_STDIO_
#include <stdio.h>
#define _CLIPS_STDIO_
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _OBJRTCMP_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
LOCALE VOID ObjectPatternsCompilerSetup(void);
LOCALE VOID ObjectPatternNodeReference(VOID *,FILE *,int,int);
#else
LOCALE VOID ObjectPatternsCompilerSetup();
LOCALE VOID ObjectPatternNodeReference();
#endif

#endif

#endif






