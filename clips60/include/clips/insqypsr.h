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

#ifndef _H_insqypsr
#define _H_insqypsr

#if INSTANCE_SET_QUERIES && (! RUN_TIME)

#ifndef _H_expressn
#include "expressn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _INSQYPSR_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE EXPRESSION *ParseQueryNoAction(EXPRESSION *,char *);
LOCALE EXPRESSION *ParseQueryAction(EXPRESSION *,char *);

#else

LOCALE EXPRESSION *ParseQueryNoAction();
LOCALE EXPRESSION *ParseQueryAction();

#endif

#ifndef _INSQYPSR_SOURCE_     
#endif

#endif

#endif





