   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 ENGINE HEADER FILE                  */
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

#ifndef _H_engine

#define _H_engine

#ifndef _H_ruledef
#include "ruledef.h"
#endif
#ifndef _H_network
#include "network.h"
#endif
#ifndef _H_moduldef
#include "moduldef.h"
#endif

struct focus
  {
   struct defmodule *theModule;
   struct defruleModule *theDefruleModule;
   struct focus *next;
  };

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _ENGINE_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

/**************************************************************/
/* The GetFocus function is remapped under certain conditions */
/* because it conflicts with a Windows 3.1 function.          */
/**************************************************************/
#if ! ((GENERIC || IBM_TBC || IBM_MSC || IBM_ICB || IBM_ZTC) && WINDOW_INTERFACE)
#define WRGetFocus GetFocus 
#endif

#define MAX_PATTERNS_CHECKED 64

#if ANSI_COMPILER
   LOCALE long                    Run(long);
   LOCALE BOOLEAN                 AddRunFunction(char *,VOID (*)(void),int);
   LOCALE BOOLEAN                 RemoveRunFunction(char *);
   LOCALE VOID                    InitializeEngine(void);
   LOCALE VOID                    SetBreak(void *);
   LOCALE BOOLEAN                 RemoveBreak(void *);
   LOCALE VOID                    RemoveAllBreakpoints(void);
   LOCALE VOID                    ShowBreaks(char *,VOID *);
   LOCALE BOOLEAN                 DefruleHasBreakpoint(void *);
   LOCALE VOID                    RunCommand(void);
   LOCALE VOID                    SetBreakCommand(void);
   LOCALE VOID                    RemoveBreakCommand(void);
   LOCALE VOID                    ShowBreaksCommand(void);
   LOCALE VOID                    HaltCommand(void);
   LOCALE int                     FocusCommand(void);
   LOCALE VOID                    ClearFocusStackCommand(void);
   LOCALE VOID                    ClearFocusStack(void);
   LOCALE VOID                   *GetNextFocus(VOID *);
   LOCALE VOID                    Focus(VOID *);
   LOCALE int                     GetFocusChanged(void);
   LOCALE VOID                    SetFocusChanged(int);
   LOCALE VOID                    ListFocusStackCommand(void);
   LOCALE VOID                    ListFocusStack(char *);
   LOCALE VOID                    GetFocusStackFunction(DATA_OBJECT_PTR);
   LOCALE VOID                    GetFocusStack(DATA_OBJECT_PTR);
   LOCALE SYMBOL_HN              *PopFocusFunction(void);
   LOCALE SYMBOL_HN              *GetFocusFunction(void);
   LOCALE VOID                   *PopFocus(void);
   LOCALE VOID                   *WRGetFocus(void);
#else
   LOCALE long                    Run();
   LOCALE BOOLEAN                 AddRunFunction();
   LOCALE BOOLEAN                 RemoveRunFunction();
   LOCALE VOID                    InitializeEngine();
   LOCALE VOID                    SetBreak();
   LOCALE BOOLEAN                 RemoveBreak();
   LOCALE VOID                    RemoveAllBreakpoints();
   LOCALE VOID                    ShowBreaks();
   LOCALE BOOLEAN                 DefruleHasBreakpoint();
   LOCALE VOID                    RunCommand();
   LOCALE VOID                    SetBreakCommand();
   LOCALE VOID                    RemoveBreakCommand();
   LOCALE VOID                    ShowBreaksCommand();
   LOCALE VOID                    HaltCommand();
   LOCALE int                     FocusCommand();
   LOCALE VOID                    ClearFocusStackCommand();
   LOCALE VOID                    ClearFocusStack();
   LOCALE VOID                   *GetNextFocus();
   LOCALE VOID                    Focus();
   LOCALE int                     GetFocusChanged();
   LOCALE VOID                    SetFocusChanged();
   LOCALE VOID                    ListFocusStackCommand();
   LOCALE VOID                    ListFocusStack();
   LOCALE VOID                    GetFocusStackFunction();
   LOCALE VOID                    GetFocusStack();
   LOCALE SYMBOL_HN              *PopFocusFunction();
   LOCALE SYMBOL_HN              *GetFocusFunction();
   LOCALE VOID                   *PopFocus();
   LOCALE VOID                   *WRGetFocus();
#endif

#ifndef _ENGINE_SOURCE_
   extern struct defrule      *ExecutingRule;
   extern BOOLEAN              HaltRules;
   extern BOOLEAN              DeletedFiringRule;
   extern struct joinNode     *TheLogicalJoin;
#endif

#endif






