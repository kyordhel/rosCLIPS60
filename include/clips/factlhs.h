   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                FACT BUILD HEADER FILE               */
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

#ifndef _H_factlhs

#define _H_factlhs

#ifndef _H_symbol
#include "symbol.h"
#endif
#ifndef _H_scanner
#include "scanner.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _FACTLHS_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER 
   LOCALE int                            FactPatternParserFind(SYMBOL_HN *);
   LOCALE struct lhsParseNode           *FactPatternParse(char *,struct token *);
   LOCALE struct lhsParseNode           *SequenceRestrictionParse(char *,struct token *);
   LOCALE struct lhsParseNode           *CreateInitialFactPattern(void);
#else
   LOCALE int                            FactPatternParserFind();
   LOCALE struct lhsParseNode           *FactPatternParse();
   LOCALE struct lhsParseNode           *SequenceRestrictionParse();
   LOCALE struct lhsParseNode           *CreateInitialFactPattern();
#endif

#endif
