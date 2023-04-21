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

#ifndef _H_objrtgen
#define _H_objrtgen

#if INSTANCE_PATTERN_MATCHING && (! RUN_TIME) && (! BLOAD_ONLY)

#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_reorder
#include "reorder.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _OBJRTGEN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
LOCALE VOID ReplaceGetJNObjectValue(EXPRESSION *,struct lhsParseNode *);
LOCALE EXPRESSION *GenGetJNObjectValue(struct lhsParseNode *);
LOCALE EXPRESSION *ObjectJNVariableComparison(struct lhsParseNode *,struct lhsParseNode *);
LOCALE EXPRESSION *GenObjectPNConstantCompare(struct lhsParseNode *);
LOCALE VOID ReplaceGetPNObjectValue(EXPRESSION *,struct lhsParseNode *);
LOCALE EXPRESSION *GenGetPNObjectValue(struct lhsParseNode *);
LOCALE EXPRESSION *ObjectPNVariableComparison(struct lhsParseNode *,struct lhsParseNode *);
LOCALE VOID GenObjectLengthTest(struct lhsParseNode *);
LOCALE VOID GenObjectZeroLengthTest(struct lhsParseNode *);
#else
LOCALE VOID ReplaceGetJNObjectValue();
LOCALE EXPRESSION *GenGetJNObjectValue();
LOCALE EXPRESSION *ObjectJNVariableComparison();
LOCALE EXPRESSION *GenObjectPNConstantCompare();
LOCALE VOID ReplaceGetPNObjectValue();
LOCALE EXPRESSION *GenGetPNObjectValue();
LOCALE EXPRESSION *ObjectPNVariableComparison();
LOCALE VOID GenObjectLengthTest();
LOCALE VOID GenObjectZeroLengthTest();
#endif

#endif

#endif






