   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*           EXPRESSION BLOAD/BSAVE HEADER FILE        */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_exprnbin
#define _H_exprnbin

#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#ifdef LOCALE
#undef LOCALE
#endif
#ifdef _EXPRNBIN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#define ExpressionPointer(i) ((struct expr *) (((i) == -1L) ? NULL : &ExpressionArray[i]))
#define HashedExpressionPointer(i) ExpressionPointer(i)
                                   
#if ANSI_COMPILER 
   LOCALE VOID                        AllocateExpressions(void);
   LOCALE VOID                        RefreshExpressions(void);
   LOCALE VOID                        ClearBloadedExpressions(void);
   LOCALE VOID                        FindHashedExpressions(void);
   LOCALE VOID                        BsaveHashedExpressions(FILE *);
   LOCALE VOID                        BsaveConstructExpressions(FILE *);
   LOCALE VOID                        BsaveExpression(struct expr *,FILE *);
#else
   LOCALE VOID                        AllocateExpressions();
   LOCALE VOID                        RefreshExpressions();
   LOCALE VOID                        ClearBloadedExpressions();
   LOCALE VOID                        FindHashedExpressions();
   LOCALE VOID                        BsaveHashedExpressions();
   LOCALE VOID                        BsaveConstructExpressions();
   LOCALE VOID                        BsaveExpression();
#endif

#ifndef _EXPRNBIN_SOURCE_
   extern struct expr HUGE_ADDR                 *ExpressionArray;
   extern long int                               ExpressionCount;
#endif
   
#endif







