   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            DEFAULT ATTRIBUTE HEADER FILE            */
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

#ifndef _H_default
#define _H_default

#ifndef _H_constrnt
#include "constrnt.h"
#endif
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _DEFAULT_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER    
   LOCALE VOID                           DeriveDefaultFromConstraints(CONSTRAINT_RECORD *,DATA_OBJECT *,int);
   LOCALE struct expr                   *ParseDefault(char *,int,int,int,int *,int *,int *);
#else
   LOCALE VOID                           DeriveDefaultFromConstraints();
   LOCALE struct expr                   *ParseDefault();
#endif 

#endif




