   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*         DEFMODULE BASIC COMMANDS HEADER FILE        */
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

#ifndef _H_modulbsc
#define _H_modulbsc

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
  
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _MODULBSC_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           DefmoduleBasicCommands(void);
   LOCALE VOID                           GetDefmoduleList(DATA_OBJECT_PTR);
   LOCALE VOID                           PPDefmoduleCommand(void);
   LOCALE int                            PPDefmodule(char *,char *);
   LOCALE VOID                           ListDefmodulesCommand(void);
   LOCALE VOID                           ListDefmodules(char *);
#else
   LOCALE VOID                           DefmoduleBasicCommands();
   LOCALE VOID                           GetDefmoduleList();
   LOCALE VOID                           PPDefmoduleCommand();
   LOCALE int                            PPDefmodule();
   LOCALE VOID                           ListDefmodulesCommand();
   LOCALE VOID                           ListDefmodules();
#endif

#endif


