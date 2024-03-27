   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            CONSTRAINT CHECKING HEADER FILE          */
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

#ifndef _H_cstrnchk
#define _H_cstrnchk

#ifndef _H_constrnt
#include "constrnt.h"
#endif
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CSTRNCHK_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#define NO_VIOLATION                    0
#define TYPE_VIOLATION                  1
#define RANGE_VIOLATION                 2
#define ALLOWED_VALUES_VIOLATION        3
#define FUNCTION_RETURN_TYPE_VIOLATION  4
#define CARDINALITY_VIOLATION           5

#if ANSI_COMPILER  
   LOCALE BOOLEAN                        CheckAllowedValuesConstraint(int,VOID *,CONSTRAINT_RECORD *);
   LOCALE int                            ConstraintCheckExpressionChain(struct expr *,
                                                                     CONSTRAINT_RECORD *);
   LOCALE VOID                           ConstraintViolationErrorMessage(char *,char *,int,int,
                                                                      struct symbolHashNode *,
                                                                      int,int,CONSTRAINT_RECORD *,
                                                                      int);
   LOCALE int                            ConstraintCheckValue(int,VOID *,CONSTRAINT_RECORD *);
   LOCALE int                            ConstraintCheckDataObject(DATA_OBJECT *,CONSTRAINT_RECORD *);
#if (! BLOAD_ONLY) && (! RUN_TIME)
   LOCALE int                            ConstraintCheckExpression(struct expr *,
                                                                CONSTRAINT_RECORD *);
#endif
#if (! RUN_TIME)
   LOCALE BOOLEAN                        UnmatchableConstraint(struct constraintRecord *);
#endif
#else
   LOCALE BOOLEAN                        CheckAllowedValuesConstraint();
   LOCALE int                            ConstraintCheckExpressionChain();
   LOCALE VOID                           ConstraintViolationErrorMessage();
   LOCALE int                            ConstraintCheckValue();
   LOCALE int                            ConstraintCheckDataObject();
#if (! BLOAD_ONLY) && (! RUN_TIME)
   LOCALE int                            ConstraintCheckExpression();
#endif
#if (! RUN_TIME)
   LOCALE BOOLEAN                        UnmatchableConstraint();
#endif
#endif 

#endif




