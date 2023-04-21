   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*       DEFMODULE CONSTRUCT COMPILER HEADER FILE      */
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

#ifndef _H_modulcmp

#define _H_modulcmp

#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#ifndef _H_moduldef
#include "moduldef.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif
  
#ifdef _MODULCMP_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           DefmoduleCompilerSetup(void);
   LOCALE VOID                           PrintDefmoduleReference(FILE *,struct defmodule *);
#else
   LOCALE VOID                           DefmoduleCompilerSetup();
   LOCALE VOID                           PrintDefmoduleReference();
#endif

#endif
