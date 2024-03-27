   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                GENERATE HEADER FILE                 */
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

#ifndef _H_generate

#define _H_generate

#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_reorder
#include "reorder.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _GENERATE_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER    
   LOCALE VOID                           FieldConversion(struct lhsParseNode *,struct lhsParseNode *);
   LOCALE struct expr                   *GetvarReplace(struct lhsParseNode *);
#else
   LOCALE VOID                           FieldConversion();
   LOCALE struct expr                   *GetvarReplace();
#endif 

#endif






