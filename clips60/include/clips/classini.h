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

#ifndef _H_classini
#define _H_classini

#ifndef _H_constrct
#include "constrct.h"
#endif
#ifndef _H_object
#include "object.h"
#endif

#if OBJECT_SYSTEM

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CLASSINI_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE VOID SetupObjectSystem(void);
#if RUN_TIME
LOCALE VOID ObjectsRunTimeInitialize(DEFCLASS *[],SLOT_NAME *[],DEFCLASS *[],unsigned);
#else
LOCALE VOID CreateSystemClasses(void);
#endif

#else

LOCALE VOID SetupObjectSystem();
#if RUN_TIME
LOCALE VOID ObjectsRunTimeInitialize();
#else
LOCALE VOID CreateSystemClasses();
#endif

#endif

#ifndef _CLASSINI_SOURCE_
extern int DefclassModuleIndex;
extern struct construct *DefclassConstruct;
#endif

#endif

#endif





