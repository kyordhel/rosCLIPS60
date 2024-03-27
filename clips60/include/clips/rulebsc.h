   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*       DEFRULE BASIC COMMANDS HEADER FILE        */
   /*******************************************************/

/*************************************************************/
/* Purpose: Implements core commands for the defrule         */
/*   construct such as clear, reset, save, undefrule,        */
/*   ppdefrule, list-defrules, and                           */
/*   get-defrule-list.                                       */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_rulebsc
#define _H_rulebsc

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
  
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _RULEBSC_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           DefruleBasicCommands(void);
   LOCALE VOID                           UndefruleCommand(void);
   LOCALE BOOLEAN                        Undefrule(VOID *);
   LOCALE VOID                           GetDefruleListFunction(DATA_OBJECT_PTR);
   LOCALE VOID                           GetDefruleList(DATA_OBJECT_PTR,VOID *);
   LOCALE SYMBOL_HN                     *DefruleModuleFunction(void);
#if DEBUGGING_FUNCTIONS
   LOCALE VOID                           PPDefruleCommand(void);
   LOCALE int                            PPDefrule(char *,char *);
   LOCALE VOID                           ListDefrulesCommand(void);
   LOCALE VOID                           ListDefrules(char *,VOID *);
   LOCALE BOOLEAN                        GetDefruleWatchFirings(VOID *);
   LOCALE BOOLEAN                        GetDefruleWatchActivations(VOID *);
   LOCALE VOID                           SetDefruleWatchFirings(int,VOID *);
   LOCALE VOID                           SetDefruleWatchActivations(int,VOID *);
   LOCALE BOOLEAN                        DefruleWatchAccess(int,int,struct expr *);
   LOCALE BOOLEAN                        DefruleWatchPrint(char *,int,struct expr *);
#endif
#else
   LOCALE VOID                           DefruleBasicCommands();
   LOCALE VOID                           UndefruleCommand();
   LOCALE BOOLEAN                        Undefrule();
   LOCALE VOID                           GetDefruleListFunction();
   LOCALE VOID                           GetDefruleList();
   LOCALE SYMBOL_HN                     *DefruleModuleFunction();
#if DEBUGGING_FUNCTIONS
   LOCALE VOID                           PPDefruleCommand();
   LOCALE int                            PPDefrule();
   LOCALE VOID                           ListDefrulesCommand();
   LOCALE VOID                           ListDefrules();
   LOCALE BOOLEAN                        GetDefruleWatchFirings();
   LOCALE BOOLEAN                        GetDefruleWatchActivations();
   LOCALE VOID                           SetDefruleWatchFirings();
   LOCALE VOID                           SetDefruleWatchActivations();
   LOCALE BOOLEAN                        DefruleWatchAccess();
   LOCALE BOOLEAN                        DefruleWatchPrint();
#endif
#endif

#ifndef _RULEBSC_SOURCE_
   extern BOOLEAN                        WatchRules;
#endif

#endif


