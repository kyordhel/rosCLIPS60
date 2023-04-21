   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*               FACT MATCH HEADER FILE                */
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

#ifndef _H_factmch

#define _H_factmch

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_factmngr
#include "factmngr.h"
#endif
#ifndef _H_factbld
#include "factbld.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _FACTMCH_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           FactPatternMatch(struct fact *,
                                               struct factPatternNode *,int,
                                               struct multifieldMarker *,
                                               struct multifieldMarker *);
   LOCALE VOID                           PatternNetErrorMessage(struct factPatternNode *);
   LOCALE VOID                           MarkFactPtnForIncrementalReset(struct patternNodeHeader *,int);
   LOCALE VOID                           FactsIncrementalReset(VOID);
#else
   LOCALE VOID                           FactPatternMatch();
   LOCALE VOID                           PatternNetErrorMessage();
   LOCALE VOID                           MarkFactPtnForIncrementalReset();
   LOCALE VOID                           FactsIncrementalReset();
#endif 

#ifndef _FACTMCH_SOURCE_
   extern struct fact             *CurrentPatternFact;
   extern struct multifield       *CurrentPatternSegment;
   extern struct multifieldMarker *CurrentPatternMarks;
#endif

#endif






