   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*           SYMBOL BINARY SAVE HEADER FILE            */
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

#ifndef _H_symblbin
#define _H_symblbin

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

#ifdef _SYMBLBIN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#define BitMapPointer(i) ((BITMAP_HN *) (BitMapArray[i]))
#define SymbolPointer(i) ((SYMBOL_HN *) (SymbolArray[i]))
#define FloatPointer(i) ((FLOAT_HN *) (FloatArray[i]))
#define IntegerPointer(i) ((INTEGER_HN *) (IntegerArray[i]))
               
#if ANSI_COMPILER
   LOCALE VOID                    MarkNeededAtomicValues(void);
   LOCALE VOID                    WriteNeededAtomicValues(FILE *);
   LOCALE VOID                    ReadNeededAtomicValues(void);
   LOCALE VOID                    InitAtomicValueNeededFlags(void);
   LOCALE VOID                    FreeAtomicValueStorage(void);
#else
   LOCALE VOID                    MarkNeededAtomicValues();
   LOCALE VOID                    WriteNeededAtomicValues();
   LOCALE VOID                    ReadNeededAtomicValues();
   LOCALE VOID                    InitAtomicValueNeededFlags();
   LOCALE VOID                    FreeAtomicValueStorage();
#endif

#ifndef _SYMBLBIN_SOURCE_
   extern struct symbolHashNode * HUGE_ADDR     *SymbolArray;
   extern struct floatHashNode * HUGE_ADDR      *FloatArray;
   extern struct integerHashNode * HUGE_ADDR    *IntegerArray;
   extern struct bitMapHashNode * HUGE_ADDR     *BitMapArray;
#endif

#endif







