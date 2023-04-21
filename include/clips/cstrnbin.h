   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*    CONSTRAINT BLOAD/BSAVE/CONSTRUCTS-TO-C HEADER    */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*      Support functions for the binary save/load of        */
/*      constraint records used by various constructs.       */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_cstrnbin
#define _H_cstrnbin

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_constrnt
#include "constrnt.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CSTRNBIN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#define ConstraintIndex(theConstraint) (((! GetDynamicConstraintChecking()) || (theConstraint == NULL)) ? -1L : ((long) theConstraint->bsaveIndex))
#define ConstraintPointer(i) (((i) == -1L) ? NULL : (CONSTRAINT_RECORD *) &ConstraintArray[i])

#if ANSI_COMPILER  
#if BLOAD_AND_BSAVE
   LOCALE VOID                           WriteNeededConstraints(FILE *); 
#endif
   LOCALE VOID                           ReadNeededConstraints(VOID);
   LOCALE VOID                           ClearBloadedConstraints(VOID);
#else
#if BLOAD_AND_BSAVE
   LOCALE VOID                           WriteNeededConstraints(); 
#endif
   LOCALE VOID                           ReadNeededConstraints();
   LOCALE VOID                           ClearBloadedConstraints();
#endif 

#ifndef _CSTRNBIN_SOURCE_
   extern CONSTRAINT_RECORD * HUGE_ADDR   ConstraintArray;
#endif

#endif




