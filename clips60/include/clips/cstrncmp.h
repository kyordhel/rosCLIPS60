   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*          CONSTRAINT CONSTRUCTS-TO-C HEADER          */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*      Support functions for the constructs-to-c of         */
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

#ifndef _H_cstrncmp
#define _H_cstrncmp

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_constrnt
#include "constrnt.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CSTRNCMP_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#if ANSI_COMPILER  
   LOCALE VOID                           PrintConstraintReference(FILE *,CONSTRAINT_RECORD *,int,int);
   LOCALE VOID                           ConstraintRecordToCode(FILE *,CONSTRAINT_RECORD *);
   LOCALE int                            ConstraintsToCode(char *,int,FILE *,int,int);
#else
   LOCALE VOID                           PrintConstraintReference();
   LOCALE VOID                           ConstraintRecordToCode();
   LOCALE int                            ConstraintsToCode();
#endif 

#endif




