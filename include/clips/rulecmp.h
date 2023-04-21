   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*        DEFRULE CONSTRUCT COMPILER HEADER FILE       */
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

#ifndef _H_rulecmp
#define _H_rulecmp

#include "conscomp.h"
#ifndef _H_extnfunc
#include "extnfunc.h"
#endif

#define JoinPrefix() ArbitraryPrefix(DefruleCodeItem,2)

#ifdef LOCALE
#undef LOCALE
#endif
  
#ifdef _RULECMP_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER  
   LOCALE VOID                     DefruleCompilerSetup(void);
   LOCALE VOID                     DefruleCModuleReference(FILE *,int,int,int);
#else
   LOCALE VOID                     DefruleCompilerSetup();
   LOCALE VOID                     DefruleCModuleReference();
#endif 

#ifndef _RULECMP_SOURCE_
extern struct CodeGeneratorItem *DefruleCodeItem;
#endif

#endif




