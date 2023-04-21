   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*              RETE UTILITY HEADER FILE               */
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

#ifndef _H_reteutil
#define _H_reteutil

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_match
#include "match.h"
#endif
#ifndef _H_network
#include "network.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _RETEUTIL_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER  
   LOCALE VOID                           PrintPartialMatch(char *,struct partialMatch *);
   LOCALE struct partialMatch           *CopyPartialMatch(struct partialMatch *,int,int);
   LOCALE struct partialMatch           *MergePartialMatches(struct partialMatch *,struct partialMatch *,int,int);
   LOCALE struct partialMatch           *AddSingleMatch(struct partialMatch *,struct alphaMatch *,int,int);
   LOCALE BOOLEAN                        GetNumericArgument(struct expr *,char *,struct dataObject *,BOOLEAN,int);
   LOCALE struct partialMatch           *NewPseudoFactPartialMatch(void);
   LOCALE long int                       IncrementPseudoFactIndex(void);
   LOCALE VOID                           FlushAlphaBetaMemory(struct partialMatch *);
   LOCALE VOID                           PrimeJoin(struct joinNode *);
   LOCALE struct multifieldMarker       *CopyMultifieldMarkers(struct multifieldMarker *);
   LOCALE struct partialMatch           *CreateAlphaMatch(VOID *,struct multifieldMarker *,
                                                       struct patternNodeHeader *);
   LOCALE VOID                           TraceErrorToRule(struct joinNode *,char *);
   LOCALE VOID                           InitializePatternHeader(struct patternNodeHeader *);
   LOCALE VOID                           MarkRuleNetwork(int);
   LOCALE VOID                           TagRuleNetwork(long *,long *,long *);
#else   
   LOCALE VOID                           PrintPartialMatch();
   LOCALE struct partialMatch           *CopyPartialMatch();
   LOCALE struct partialMatch           *MergePartialMatches();
   LOCALE struct partialMatch           *AddSingleMatch();
   LOCALE BOOLEAN                        GetNumericArgument();
   LOCALE struct partialMatch           *NewPseudoFactPartialMatch();
   LOCALE long int                       IncrementPseudoFactIndex();
   LOCALE VOID                           FlushAlphaBetaMemory();
   LOCALE VOID                           PrimeJoin();
   LOCALE struct multifieldMarker       *CopyMultifieldMarkers();
   LOCALE struct partialMatch           *CreateAlphaMatch();
   LOCALE VOID                           TraceErrorToRule();
   LOCALE VOID                           InitializePatternHeader();
   LOCALE VOID                           MarkRuleNetwork();
   LOCALE VOID                           TagRuleNetwork();
#endif 

#ifndef _RETEUTIL_SOURCE_
   extern struct partialMatch           *GlobalLHSBinds;
   extern struct partialMatch           *GlobalRHSBinds;
   extern struct joinNode               *GlobalJoin;
#endif

#endif




