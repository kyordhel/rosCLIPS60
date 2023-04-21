   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*           CONSTRAINT OPERATIONS HEADER FILE         */
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

#ifndef _H_cstrnops
#define _H_cstrnops

#if (! RUN_TIME)

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_constrnt
#include "constrnt.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CSTRNOPS_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER  
   LOCALE struct constraintRecord       *IntersectConstraints(struct constraintRecord *,struct constraintRecord *);
#if (! BLOAD_ONLY)
   LOCALE struct constraintRecord       *UnionConstraints(struct constraintRecord *,struct constraintRecord *);
   LOCALE VOID                           RemoveConstantFromConstraint(int,VOID *,CONSTRAINT_RECORD *);
#endif
#else
   LOCALE struct constraintRecord       *IntersectConstraints();
#if (! BLOAD_ONLY)
   LOCALE struct constraintRecord       *UnionConstraints();
   LOCALE VOID                           RemoveConstantFromConstraint();
#endif
#endif 

#endif

#endif


