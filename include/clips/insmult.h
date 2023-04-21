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

#ifndef _H_insmult
#define _H_insmult

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _INSMULT_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

#if (! RUN_TIME)
LOCALE VOID SetupInstanceMultifieldCommands(void);
#endif

LOCALE VOID MVSlotReplaceCommand(DATA_OBJECT *);
LOCALE VOID MVSlotInsertCommand(DATA_OBJECT *);
LOCALE VOID MVSlotDeleteCommand(DATA_OBJECT *);
LOCALE BOOLEAN DirectMVReplaceCommand(void);
LOCALE BOOLEAN DirectMVInsertCommand(void);
LOCALE BOOLEAN DirectMVDeleteCommand(void);

#else

#if (! RUN_TIME)
LOCALE VOID SetupInstanceMultifieldCommands();
#endif

LOCALE VOID MVSlotReplaceCommand();
LOCALE VOID MVSlotInsertCommand();
LOCALE VOID MVSlotDeleteCommand();
LOCALE BOOLEAN DirectMVReplaceCommand();
LOCALE BOOLEAN DirectMVInsertCommand();
LOCALE BOOLEAN DirectMVDeleteCommand();

#endif

#ifndef _INSMULT_SOURCE_
#endif

#endif





