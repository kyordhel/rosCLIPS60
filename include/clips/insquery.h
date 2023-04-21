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

#ifndef _H_insquery
#define _H_insquery

#if INSTANCE_SET_QUERIES

#ifndef _H_object
#include "object.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _INSQUERY_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#define QUERY_DELIMETER_STRING     "(QDS)"

#if ANSI_COMPILER
LOCALE VOID SetupQuery(void);
LOCALE SYMBOL_HN *GetQueryInstance(void);
LOCALE VOID GetQueryInstanceSlot(DATA_OBJECT *);
LOCALE BOOLEAN AnyInstances(void);
LOCALE VOID QueryFindInstance(DATA_OBJECT *);
LOCALE VOID QueryFindAllInstances(DATA_OBJECT *);
LOCALE VOID QueryDoForInstance(DATA_OBJECT *);
LOCALE VOID QueryDoForAllInstances(DATA_OBJECT *);
LOCALE VOID DelayedQueryDoForAllInstances(DATA_OBJECT *);
#else
LOCALE VOID SetupQuery();
LOCALE SYMBOL_HN *GetQueryInstance();
LOCALE VOID GetQueryInstanceSlot();
LOCALE BOOLEAN AnyInstances();
LOCALE VOID QueryFindInstance();
LOCALE VOID QueryFindAllInstances();
LOCALE VOID QueryDoForInstance();
LOCALE VOID QueryDoForAllInstances();
LOCALE VOID DelayedQueryDoForAllInstances();
#endif

#ifndef _INSQUERY_SOURCE_     
extern SYMBOL_HN *QUERY_DELIMETER_SYMBOL;
#endif

#endif

#endif





