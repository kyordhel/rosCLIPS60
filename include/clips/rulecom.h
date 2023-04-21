   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             DEFRULE COMMANDS HEADER FILE            */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_rulecom
#define _H_rulecom

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
  
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _RULECOM_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER   
   LOCALE VOID                           DefruleConstructsToCSetup(void);
   LOCALE BOOLEAN                        Matches(VOID *);
   LOCALE VOID                           DefruleCommands(void);
   LOCALE VOID                           MatchesCommand(void);
#if DEVELOPER
   LOCALE VOID                           ShowJoinsCommand(void);
   LOCALE long                           RuleComplexityCommand(void);
#endif
#else  
   LOCALE VOID                           DefruleConstructsToCSetup();
   LOCALE BOOLEAN                        Matches();
   LOCALE VOID                           DefruleCommands();
   LOCALE VOID                           ListDefrules();
   LOCALE VOID                           PPRuleCommand();
   LOCALE VOID                           UndefruleCommand();
   LOCALE VOID                           MatchesCommand();
   LOCALE VOID                           GetDefruleListFunction();
#if DEVELOPER
   LOCALE VOID                           ShowJoinsCommand();
   LOCALE long                           RuleComplexityCommand();
#endif
#endif

#endif


