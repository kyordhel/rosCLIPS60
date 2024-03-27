   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                  DRIVE HEADER FILE                  */
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

#ifndef _H_drive

#define _H_drive

#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_match
#include "match.h"
#endif
#ifndef _H_network
#include "network.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _DRIVE_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   VOID                           Drive(struct partialMatch *,struct joinNode *,int);
   VOID                           PNLDrive(struct joinNode *,struct partialMatch *);
   BOOLEAN                        EvaluateJoinExpression(struct expr *,struct partialMatch *,struct partialMatch *,struct joinNode *);
#else
   VOID                           Drive();
   VOID                           PNLDrive();
   BOOLEAN                        EvaluateJoinExpression();
#endif

#ifndef _DRIVE_SOURCE_
   extern int                     JoinOperationInProgress;
#endif

#endif





