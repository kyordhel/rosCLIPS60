   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*         DEFGLOBAL BASIC COMMANDS HEADER FILE        */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_globlbsc
#define _H_globlbsc

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
  
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _GLOBLBSC_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           DefglobalBasicCommands(void);
   LOCALE VOID                           UndefglobalCommand(void);
   LOCALE BOOLEAN                        Undefglobal(VOID *);
   LOCALE VOID                           GetDefglobalListFunction(DATA_OBJECT_PTR);
   LOCALE VOID                           GetDefglobalList(DATA_OBJECT_PTR,VOID *);
   LOCALE SYMBOL_HN                     *DefglobalModuleFunction(void);
   LOCALE VOID                           PPDefglobalCommand(void);
   LOCALE int                            PPDefglobal(char *,char *);
   LOCALE VOID                           ListDefglobalsCommand(void);
   LOCALE VOID                           ListDefglobals(char *,VOID *);
   LOCALE BOOLEAN                        GetDefglobalWatch(VOID *);
   LOCALE VOID                           SetDefglobalWatch(int,VOID *);
   LOCALE VOID                           ResetDefglobals(void);
#else
   LOCALE VOID                           DefglobalBasicCommands();
   LOCALE VOID                           UndefglobalCommand();
   LOCALE BOOLEAN                        Undefglobal();
   LOCALE VOID                           GetDefglobalListFunction();
   LOCALE VOID                           GetDefglobalList();
   LOCALE SYMBOL_HN                     *DefglobalModuleFunction();
   LOCALE VOID                           PPDefglobalCommand();
   LOCALE int                            PPDefglobal();
   LOCALE VOID                           ListDefglobalsCommand();
   LOCALE VOID                           ListDefglobals();
   LOCALE BOOLEAN                        GetDefglobalWatch();
   LOCALE VOID                           SetDefglobalWatch();
   LOCALE VOID                           ResetDefglobals();
#endif

#ifndef _GLOBLBSC_SOURCE_
   extern int                            WatchGlobals;
#endif

#endif


