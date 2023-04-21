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

#ifndef _H_classinf
#define _H_classinf

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CLASSINF_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE BOOLEAN ClassAbstractPCommand(void);
#if DEFRULE_CONSTRUCT
LOCALE BOOLEAN ClassReactivePCommand(void);
#endif
LOCALE VOID *ClassInfoFnxArgs(char *,int *);
LOCALE VOID ClassSlotsCommand(DATA_OBJECT *);
LOCALE VOID ClassSuperclassesCommand(DATA_OBJECT *);
LOCALE VOID ClassSubclassesCommand(DATA_OBJECT *);
LOCALE VOID GetDefmessageHandlersListCmd(DATA_OBJECT *);
LOCALE VOID SlotFacetsCommand(DATA_OBJECT *);
LOCALE VOID SlotSourcesCommand(DATA_OBJECT *);
LOCALE VOID SlotTypesCommand(DATA_OBJECT *);
LOCALE VOID SlotAllowedValuesCommand(DATA_OBJECT *);
LOCALE VOID SlotRangeCommand(DATA_OBJECT *);
LOCALE VOID SlotCardinalityCommand(DATA_OBJECT *);
LOCALE BOOLEAN ClassAbstractP(VOID *);
#if DEFRULE_CONSTRUCT
LOCALE BOOLEAN ClassReactiveP(VOID *);
#endif
LOCALE VOID ClassSlots(VOID *,DATA_OBJECT *,int);
LOCALE VOID GetDefmessageHandlerList(VOID *,DATA_OBJECT *,int);
LOCALE VOID ClassSuperclasses(VOID *,DATA_OBJECT *,int);
LOCALE VOID ClassSubclasses(VOID *,DATA_OBJECT *,int);
LOCALE VOID SlotFacets(VOID *,char *,DATA_OBJECT *);
LOCALE VOID SlotSources(VOID *,char *,DATA_OBJECT *);
LOCALE VOID SlotTypes(VOID *,char *,DATA_OBJECT *);
LOCALE VOID SlotAllowedValues(VOID *,char *,DATA_OBJECT *);
LOCALE VOID SlotRange(VOID *,char *,DATA_OBJECT *);
LOCALE VOID SlotCardinality(VOID *,char *,DATA_OBJECT *);

#else

LOCALE BOOLEAN ClassAbstractPCommand();
#if DEFRULE_CONSTRUCT
LOCALE BOOLEAN ClassReactivePCommand();
#endif
LOCALE VOID *ClassInfoFnxArgs();
LOCALE VOID ClassSlotsCommand();
LOCALE VOID ClassSuperclassesCommand();
LOCALE VOID ClassSubclassesCommand();
LOCALE VOID GetDefmessageHandlersListCmd();
LOCALE VOID SlotFacetsCommand();
LOCALE VOID SlotSourcesCommand();
LOCALE VOID SlotTypesCommand();
LOCALE VOID SlotAllowedValuesCommand();
LOCALE VOID SlotRangeCommand();
LOCALE VOID SlotCardinalityCommand();
LOCALE BOOLEAN ClassAbstractP();
#if DEFRULE_CONSTRUCT
LOCALE BOOLEAN ClassReactiveP();
#endif
LOCALE VOID ClassSlots();
LOCALE VOID GetDefmessageHandlerList();
LOCALE VOID ClassSuperclasses();
LOCALE VOID ClassSubclasses();
LOCALE VOID SlotFacets();
LOCALE VOID SlotSources();
LOCALE VOID SlotTypes();
LOCALE VOID SlotAllowedValues();
LOCALE VOID SlotRange();
LOCALE VOID SlotCardinality();

#endif

#endif





