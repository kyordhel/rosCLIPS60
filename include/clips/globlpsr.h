   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             DEFGLOBAL PARSER HEADER FILE            */
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

#ifndef _H_globlpsr

#define _H_globlpsr

#ifdef _DEFGLOBL_SOURCE_
struct defglobal;
#endif

#ifndef _H_expressn
#include "expressn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _GLOBLPSR_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE BOOLEAN                 ParseDefglobal(char *);
   LOCALE BOOLEAN                 ReplaceGlobalVariable(struct expr *);
   LOCALE VOID                    GlobalReferenceErrorMessage(char *);
#else
   LOCALE BOOLEAN                 ParseDefglobal();
   LOCALE BOOLEAN                 ReplaceGlobalVariable();
   LOCALE VOID                    GlobalReferenceErrorMessage();
#endif

#endif







