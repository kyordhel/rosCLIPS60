   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*           DEFRULE LHS PARSING HEADER FILE           */
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

#ifndef _H_rulelhs
#define _H_rulelhs

#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_reorder
#include "reorder.h"
#endif
#ifndef _H_scanner
#include "scanner.h"
#endif
#ifndef _H_pattern
#include "pattern.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _RULELHS_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif
  
#if ANSI_COMPILER
LOCALE struct lhsParseNode           *ParseRuleLHS(char *,struct token *,char *);
LOCALE VOID                           PropagatePatternType(struct lhsParseNode *,struct patternParser *);
#else
LOCALE struct lhsParseNode           *ParseRuleLHS();
LOCALE VOID                           PropagatePatternType();
#endif


#ifndef _RULELHS_SOURCE_
   extern int                     GlobalSalience;
   extern int                     GlobalAutoFocus;
   extern struct expr            *SalienceExpression;
#endif

#endif




