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

#ifndef _H_genrcpsr
#define _H_genrcpsr

#if DEFGENERIC_CONSTRUCT && (! BLOAD_ONLY) && (! RUN_TIME)

#include "genrcfun.h"

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _GENRCPSR_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE BOOLEAN ParseDefgeneric(char *);
LOCALE BOOLEAN ParseDefmethod(char *);
LOCALE DEFMETHOD *AddMethod(DEFGENERIC *,DEFMETHOD *,int,unsigned,EXPRESSION *,
                            int,int,SYMBOL_HN *,EXPRESSION *,char *,int);
LOCALE VOID PackRestrictionTypes(RESTRICTION *,EXPRESSION *);
LOCALE VOID DeleteTempRestricts(EXPRESSION *);
LOCALE DEFMETHOD *FindMethodByRestrictions(DEFGENERIC *,EXPRESSION *,int,
                                           SYMBOL_HN *,int *);

#else

LOCALE BOOLEAN ParseDefgeneric();
LOCALE BOOLEAN ParseDefmethod();
LOCALE DEFMETHOD *AddMethod();
LOCALE VOID PackRestrictionTypes();
LOCALE VOID DeleteTempRestricts();
LOCALE DEFMETHOD *FindMethodByRestrictions();

#endif

#ifndef _GENRCPSR_SOURCE_
#endif

#endif

#endif




