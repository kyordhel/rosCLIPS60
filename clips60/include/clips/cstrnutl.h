   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            CONSTRAINT UTILITY HEADER FILE           */
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

#ifndef _H_cstrnutl
#define _H_cstrnutl

#ifndef _H_constrnt
#include "constrnt.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CSTRNUTL_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#if ANSI_COMPILER     
   LOCALE struct constraintRecord       *GetConstraintRecord(VOID);
   LOCALE int                            CompareNumbers(int,VOID *,int,VOID *);
   LOCALE struct constraintRecord       *CopyConstraintRecord(CONSTRAINT_RECORD *);
   LOCALE int                            SetConstraintType(int,CONSTRAINT_RECORD *);
   LOCALE VOID                           SetAnyAllowedFlags(CONSTRAINT_RECORD *,int);
   LOCALE VOID                           SetAnyRestrictionFlags(CONSTRAINT_RECORD *,int);
   LOCALE CONSTRAINT_RECORD             *ArgumentTypeToConstraintRecord(int);
   LOCALE CONSTRAINT_RECORD             *FunctionCallToConstraintRecord(VOID *);
   LOCALE CONSTRAINT_RECORD             *ExpressionToConstraintRecord(struct expr *);
#else
   LOCALE struct constraintRecord       *GetConstraintRecord();
   LOCALE int                            CompareNumbers();
   LOCALE struct constraintRecord       *CopyConstraintRecord();
   LOCALE int                            SetConstraintType();
   LOCALE VOID                           SetAnyAllowedFlags();
   LOCALE VOID                           SetAnyRestrictionFlags();
   LOCALE CONSTRAINT_RECORD             *ArgumentTypeToConstraintRecord();
   LOCALE CONSTRAINT_RECORD             *FunctionCallToConstraintRecord();
   LOCALE CONSTRAINT_RECORD             *ExpressionToConstraintRecord();
#endif 

#endif




