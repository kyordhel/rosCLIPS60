   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*         DEFFACTS BASIC COMMANDS HEADER FILE         */
   /*******************************************************/

/*************************************************************/
/* Purpose: Implements core commands for the deffacts        */
/*   construct such as clear, reset, save, undeffacts,       */
/*   ppdeffacts, list-deffacts, and get-deffacts-list.       */
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

#ifndef _H_dffctbsc
#define _H_dffctbsc

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
  
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _DFFCTBSC_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           DeffactsBasicCommands(void);
   LOCALE VOID                           UndeffactsCommand(void);
   LOCALE BOOLEAN                        Undeffacts(VOID *);
   LOCALE VOID                           GetDeffactsListFunction(DATA_OBJECT_PTR);
   LOCALE VOID                           GetDeffactsList(DATA_OBJECT_PTR,VOID *);
   LOCALE SYMBOL_HN                     *DeffactsModuleFunction(void);
   LOCALE VOID                           PPDeffactsCommand(void);
   LOCALE int                            PPDeffacts(char *,char *);
   LOCALE VOID                           ListDeffactsCommand(void);
   LOCALE VOID                           ListDeffacts(char *,VOID *);
#else
   LOCALE VOID                           DeffactsBasicCommands();
   LOCALE VOID                           UndeffactsCommand();
   LOCALE BOOLEAN                        Undeffacts();
   LOCALE VOID                           GetDeffactsListFunction();
   LOCALE VOID                           GetDeffactsList();
   LOCALE SYMBOL_HN                     *DeffactsModuleFunction();
   LOCALE VOID                           PPDeffactsCommand();
   LOCALE int                            PPDeffacts();
   LOCALE VOID                           ListDeffactsCommand();
   LOCALE VOID                           ListDeffacts();
#endif

#endif


