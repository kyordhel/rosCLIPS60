   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*          EXPRESSION OPERATIONS HEADER FILE          */
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

#ifndef _H_exprnops

#define _H_exprnops

#ifndef _H_expressn
#include "expressn.h"
#endif
   
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _EXPRNOPS_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE BOOLEAN                        ConstantExpression(struct expr *);
   LOCALE VOID                           PrintExpression(char *,struct expr *);
   LOCALE int                            ExpressionSize(struct expr *);
   LOCALE int                            CountArguments(struct expr *);
   LOCALE BOOLEAN                        ExpressionContainsVariables(struct expr *,int);
   LOCALE BOOLEAN                        IdenticalExpression(struct expr *,struct expr *);
   LOCALE struct expr                   *GenConstant(int,VOID *);
#if ! RUN_TIME
   LOCALE int                            CheckArgumentAgainstRestriction(struct expr *,int);
#endif
   LOCALE BOOLEAN                        ConstantType(int);
   LOCALE struct expr                   *CombineExpressions(struct expr *,struct expr *);
   LOCALE struct expr                   *AppendExpressions(struct expr *,struct expr *);
#else
   LOCALE BOOLEAN                        ConstantExpression();
   LOCALE VOID                           PrintExpression();
   LOCALE int                            ExpressionSize();
   LOCALE int                            CountArguments();
   LOCALE BOOLEAN                        ExpressionContainsVariables();
   LOCALE BOOLEAN                        IdenticalExpression();
   LOCALE struct expr                   *GenConstant();
#if ! RUN_TIME
   LOCALE int                            CheckArgumentAgainstRestriction();
#endif
   LOCALE BOOLEAN                        ConstantType();
   LOCALE struct expr                   *CombineExpressions();
   LOCALE struct expr                   *AppendExpressions();
#endif


#endif




