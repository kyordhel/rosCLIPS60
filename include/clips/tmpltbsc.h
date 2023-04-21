   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*       DEFTEMPLATE BASIC COMMANDS HEADER FILE        */
   /*******************************************************/

/*************************************************************/
/* Purpose: Implements core commands for the deftemplate     */
/*   construct such as clear, reset, save, undeftemplate,    */
/*   ppdeftemplate, list-deftemplates, and                   */
/*   get-deftemplate-list.                                   */
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

#ifndef _H_tmpltbsc
#define _H_tmpltbsc

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
  
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _TMPLTBSC_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           DeftemplateBasicCommands(void);
   LOCALE VOID                           UndeftemplateCommand(void);
   LOCALE BOOLEAN                        Undeftemplate(VOID *);
   LOCALE VOID                           GetDeftemplateListFunction(DATA_OBJECT_PTR);
   LOCALE VOID                           GetDeftemplateList(DATA_OBJECT_PTR,VOID *);
   LOCALE SYMBOL_HN                     *DeftemplateModuleFunction(void);
#if DEBUGGING_FUNCTIONS
   LOCALE VOID                           PPDeftemplateCommand(void);
   LOCALE int                            PPDeftemplate(char *,char *);
   LOCALE VOID                           ListDeftemplatesCommand(void);
   LOCALE VOID                           ListDeftemplates(char *,VOID *);
   LOCALE BOOLEAN                        GetDeftemplateWatch(VOID *);
   LOCALE VOID                           SetDeftemplateWatch(int,VOID *);
   LOCALE BOOLEAN                        DeftemplateWatchAccess(int,int,struct expr *);
   LOCALE BOOLEAN                        DeftemplateWatchPrint(char *,int,struct expr *);
#endif
#else
   LOCALE VOID                           DeftemplateBasicCommands();
   LOCALE VOID                           UndeftemplateCommand();
   LOCALE BOOLEAN                        Undeftemplate();
   LOCALE VOID                           GetDeftemplateListFunction();
   LOCALE VOID                           GetDeftemplateList();
   LOCALE SYMBOL_HN                     *DeftemplateModuleFunction();
#if DEBUGGING_FUNCTIONS
   LOCALE VOID                           PPDeftemplateCommand();
   LOCALE int                            PPDeftemplate();
   LOCALE VOID                           ListDeftemplatesCommand();
   LOCALE VOID                           ListDeftemplates();
   LOCALE BOOLEAN                        GetDeftemplateWatch();
   LOCALE VOID                           SetDeftemplateWatch();
   LOCALE BOOLEAN                        DeftemplateWatchAccess();
   LOCALE BOOLEAN                        DeftemplateWatchPrint();
#endif
#endif

#ifndef _TMPLTBSC_SOURCE_
#if (! RUN_TIME) && (! BLOAD_ONLY) && DEBUGGING_FUNCTIONS
   extern int                            DeletedTemplateDebugFlags;
#endif
#endif

#endif


