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

#ifndef _H_insmngr
#define _H_insmngr

#ifndef _H_object
#include "object.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _INSMNGR_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE VOID InitializeInstanceCommand(DATA_OBJECT *);
LOCALE VOID MakeInstanceCommand(DATA_OBJECT *);
LOCALE SYMBOL_HN *GetFullInstanceName(INSTANCE_TYPE *);
LOCALE INSTANCE_TYPE *BuildInstance(SYMBOL_HN *,DEFCLASS *,BOOLEAN);
LOCALE VOID InitSlotsCommand(DATA_OBJECT *);
LOCALE BOOLEAN QuashInstance(INSTANCE_TYPE *);

#if INSTANCE_PATTERN_MATCHING
LOCALE VOID InactiveInitializeInstance(DATA_OBJECT *);
LOCALE VOID InactiveMakeInstance(DATA_OBJECT *);
#endif

#else

LOCALE VOID InitializeInstanceCommand();
LOCALE VOID MakeInstanceCommand();
LOCALE SYMBOL_HN *GetFullInstanceName();
LOCALE INSTANCE_TYPE *BuildInstance();
LOCALE VOID InitSlotsCommand();
LOCALE BOOLEAN QuashInstance();

#if INSTANCE_PATTERN_MATCHING
LOCALE VOID InactiveInitializeInstance();
LOCALE VOID InactiveMakeInstance();
#endif

#endif

#ifndef _INSMNGR_SOURCE_
extern INSTANCE_TYPE *InstanceList;
extern unsigned long GlobalNumberOfInstances;
#endif

#endif







