   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*          LOGICAL DEPENDENCIES HEADER FILE           */
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

#ifndef _H_lgcldpnd

#define _H_lgcldpnd

#ifndef _H_match
#include "match.h"
#endif
#ifndef _H_pattern
#include "pattern.h"
#endif

struct dependency
  {
   VOID *dPtr;
   struct dependency *next;
  };
  
#ifdef LOCALE
#undef LOCALE
#endif
#ifdef _LGCLDPND_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE BOOLEAN                        AddLogicalDependencies(struct patternEntity *,int);
   LOCALE VOID                           RemoveEntityDependencies(struct patternEntity *);
   LOCALE VOID                           RemovePMDependencies(struct partialMatch *);
   LOCALE VOID                           AddToDependencyList(struct partialMatch *);
   LOCALE VOID                           ForceLogicalRetractions(void);
   LOCALE VOID                           ListDependencies(struct patternEntity *);
   LOCALE VOID                           ListDependents(struct patternEntity *);
   LOCALE VOID                           DependenciesCommand(void);
   LOCALE VOID                           DependentsCommand(void);
#else
   LOCALE BOOLEAN                        AddLogicalDependencies();
   LOCALE VOID                           RemoveEntityDependencies();
   LOCALE VOID                           RemovePMDependencies();
   LOCALE VOID                           AddToDependencyList();
   LOCALE VOID                           ForceLogicalRetractions();
   LOCALE VOID                           ListDependencies();
   LOCALE VOID                           ListDependents();
   LOCALE VOID                           DependenciesCommand();
   LOCALE VOID                           DependentsCommand();
#endif

#endif





