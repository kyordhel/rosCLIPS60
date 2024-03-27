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

#ifndef _H_insmoddp
#define _H_insmoddp

#define DIRECT_MODIFY_STRING    "direct-modify"
#define MSG_MODIFY_STRING       "message-modify"
#define DIRECT_DUPLICATE_STRING "direct-duplicate"
#define MSG_DUPLICATE_STRING    "message-duplicate"

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _INSMODDP_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

#if (! RUN_TIME)
LOCALE VOID SetupInstanceModDupCommands(void);
#endif

LOCALE VOID ModifyInstance(DATA_OBJECT *);
LOCALE VOID MsgModifyInstance(DATA_OBJECT *);
LOCALE VOID DuplicateInstance(DATA_OBJECT *);
LOCALE VOID MsgDuplicateInstance(DATA_OBJECT *);

#if INSTANCE_PATTERN_MATCHING
LOCALE VOID InactiveModifyInstance(DATA_OBJECT *);
LOCALE VOID InactiveMsgModifyInstance(DATA_OBJECT *);
LOCALE VOID InactiveDuplicateInstance(DATA_OBJECT *);
LOCALE VOID InactiveMsgDuplicateInstance(DATA_OBJECT *);
#endif

LOCALE VOID DirectModifyMsgHandler(DATA_OBJECT *);
LOCALE VOID MsgModifyMsgHandler(DATA_OBJECT *);
LOCALE VOID DirectDuplicateMsgHandler(DATA_OBJECT *);
LOCALE VOID MsgDuplicateMsgHandler(DATA_OBJECT *);

#else

#if (! RUN_TIME)
LOCALE VOID SetupInstanceModDupCommands();
#endif

LOCALE VOID ModifyInstance();
LOCALE VOID MsgModifyInstance();
LOCALE VOID DuplicateInstance();
LOCALE VOID MsgDuplicateInstance();

#if INSTANCE_PATTERN_MATCHING
LOCALE VOID InactiveModifyInstance();
LOCALE VOID InactiveMsgModifyInstance();
LOCALE VOID InactiveDuplicateInstance();
LOCALE VOID InactiveMsgDuplicateInstance();
#endif

LOCALE VOID DirectModifyMsgHandler();
LOCALE VOID MsgModifyMsgHandler();
LOCALE VOID DirectDuplicateMsgHandler();
LOCALE VOID MsgDuplicateMsgHandler();

#endif

#ifndef _INSMODDP_SOURCE_
#endif

#endif







