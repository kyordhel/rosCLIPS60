   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            INCREMENTAL RESET HEADER FILE            */
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

#ifndef _H_incrrset

#define _H_incrrset

#ifndef _H_ruledef
#include "ruledef.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _INCRRSET_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           IncrementalReset(struct defrule *);
   LOCALE BOOLEAN                        GetIncrementalReset(void);
   LOCALE BOOLEAN                        SetIncrementalReset(BOOLEAN);
   LOCALE int                            GetIncrementalResetCommand(void);
   LOCALE int                            SetIncrementalResetCommand(void);
#else
   LOCALE VOID                           IncrementalReset();
   LOCALE BOOLEAN                        GetIncrementalReset();
   LOCALE BOOLEAN                        SetIncrementalReset();
   LOCALE int                            GetIncrementalResetCommand();
   LOCALE int                            SetIncrementalResetCommand();
#endif

#if (! RUN_TIME) && (! BLOAD_ONLY)
#ifndef _INCRRSET_SOURCE_
   extern int                            IncrementalResetInProgress;
#endif
#endif

#endif









