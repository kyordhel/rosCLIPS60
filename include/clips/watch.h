   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                  WATCH HEADER FILE                  */
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

#ifndef _H_watch
#define _H_watch

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _WATCH_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#ifndef _H_expressn
#include "expressn.h"
#endif

#if ANSI_COMPILER 
   LOCALE BOOLEAN                        SetWatchItem(char *,int,struct expr *);
   LOCALE int                            GetWatchItem(char *);
   LOCALE BOOLEAN                        AddWatchItem(char *,int,int *,int,
                                                      BOOLEAN (*)(int,int,struct expr *),
                                                      BOOLEAN (*)(char *,int,struct expr *));
   LOCALE char                          *GetNthWatchName(int);
   LOCALE int                            GetNthWatchValue(int);
   LOCALE VOID                           WatchCommand(void);
   LOCALE VOID                           UnwatchCommand(void);
   LOCALE VOID                           ListWatchItemsCommand(void);
   LOCALE VOID                           WatchFunctionDefinitions(void);
   LOCALE BOOLEAN                        Watch(char *);
   LOCALE BOOLEAN                        Unwatch(char *);
#else
   LOCALE BOOLEAN                        SetWatchItem();
   LOCALE int                            GetWatchItem();
   LOCALE BOOLEAN                        AddWatchItem();
   LOCALE char                          *GetNthWatchName();
   LOCALE int                            GetNthWatchValue();
   LOCALE VOID                           WatchCommand();
   LOCALE VOID                           UnwatchCommand();
   LOCALE VOID                           ListWatchItemsCommand();
   LOCALE VOID                           WatchFunctionDefinitions();
   LOCALE BOOLEAN                        Watch();
   LOCALE BOOLEAN                        Unwatch();
#endif 

#endif




