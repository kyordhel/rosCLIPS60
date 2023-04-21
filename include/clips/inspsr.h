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

#ifndef _H_inspsr
#define _H_inspsr

#ifndef _H_expressn
#include "expressn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _INSPSR_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

#if ! RUN_TIME
LOCALE EXPRESSION *ParseInitializeInstance(EXPRESSION *,char *);
LOCALE EXPRESSION *ParseSlotOverrides(char *,int *);
#endif

LOCALE EXPRESSION *ParseSimpleInstance(EXPRESSION *,char *);

#else

#if ! RUN_TIME
LOCALE EXPRESSION *ParseInitializeInstance();
LOCALE EXPRESSION *ParseSlotOverrides();
#endif

LOCALE EXPRESSION *ParseSimpleInstance();

#endif

#ifndef _INSCOM_SOURCE_
#endif

#endif





