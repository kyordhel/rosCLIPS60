   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*        SYMBOL CONSTRUCT COMPILER HEADER FILE        */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_symblcmp
#define _H_symblcmp

#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#ifndef _H_symbol
#include "symbol.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _SYMBLCMP_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif


               
#if ANSI_COMPILER
   LOCALE VOID                     PrintSymbolReference(FILE *,SYMBOL_HN *);
   LOCALE VOID                     PrintFloatReference(FILE *,FLOAT_HN *);
   LOCALE VOID                     PrintIntegerReference(FILE *,INTEGER_HN *);
   LOCALE VOID                     PrintBitMapReference(FILE *,BITMAP_HN *);
   LOCALE VOID                     AtomicValuesToCode(char *);
#else
   LOCALE VOID                     PrintSymbolReference();
   LOCALE VOID                     PrintFloatReference();
   LOCALE VOID                     PrintIntegerReference();
   LOCALE VOID                     PrintBitMapReference();
   LOCALE VOID                     AtomicValuesToCode();
#endif

#endif







