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

#ifndef _H_genrcexe
#define _H_genrcexe

#if DEFGENERIC_CONSTRUCT

#include "genrcfun.h"
#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _GENRCEXE_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE VOID GenericDispatch(DEFGENERIC *,DEFMETHOD *,DEFMETHOD *,EXPRESSION *,DATA_OBJECT *);
LOCALE VOID UnboundMethodErr(void);
LOCALE BOOLEAN IsMethodApplicable(DEFMETHOD *);

#if IMPERATIVE_METHODS
LOCALE int NextMethodP(void);
LOCALE VOID CallNextMethod(DATA_OBJECT *);
LOCALE VOID CallSpecificMethod(DATA_OBJECT *);
LOCALE VOID OverrideNextMethod(DATA_OBJECT *);
#endif

LOCALE VOID GetGenericCurrentArgument(DATA_OBJECT *); 

#else

LOCALE VOID GenericDispatch();
LOCALE VOID UnboundMethodErr();
LOCALE BOOLEAN IsMethodApplicable();

#if IMPERATIVE_METHODS
LOCALE int NextMethodP();
LOCALE VOID CallNextMethod();
LOCALE VOID CallSpecificMethod();
LOCALE VOID OverrideNextMethod();
#endif

LOCALE VOID GetGenericCurrentArgument(); 

#endif

#ifndef _GENRCEXE_SOURCE_
#endif

#endif

#endif




