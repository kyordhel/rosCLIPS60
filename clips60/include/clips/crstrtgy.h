   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*      CONFLICT RESOLUTION STRATEGY HEADER MODULE     */
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

#ifndef _H_crstrtgy

#define _H_crstrtgy

#include "agenda.h"
#include "symbol.h"

#define DEPTH_STRATEGY 0
#define BREADTH_STRATEGY 1
#define LEX_STRATEGY 2
#define MEA_STRATEGY 3
#define COMPLEXITY_STRATEGY 4
#define SIMPLICITY_STRATEGY 5
#define RANDOM_STRATEGY 6

#define DEFAULT_STRATEGY DEPTH_STRATEGY

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CRSTRTGY_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER   
   LOCALE VOID                           PlaceActivation(ACTIVATION **,ACTIVATION *);
#if CONFLICT_RESOLUTION_STRATEGIES
   LOCALE int                            SetStrategy(int);
   LOCALE int                            GetStrategy(void);
   LOCALE SYMBOL_HN                     *SetStrategyCommand(void);
   LOCALE SYMBOL_HN                     *GetStrategyCommand(void);
#endif
#else
   LOCALE VOID                           PlaceActivation();
#if CONFLICT_RESOLUTION_STRATEGIES
   LOCALE int                            SetStrategy();
   LOCALE int                            GetStrategy();
   LOCALE SYMBOL_HN                     *SetStrategyCommand();
   LOCALE SYMBOL_HN                     *GetStrategyCommand();
#endif
#endif

#endif






