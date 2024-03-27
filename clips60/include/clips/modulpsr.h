   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             DEFMODULE PARSER HEADER FILE            */
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

#ifndef _H_modulpsr
#define _H_modulpsr

#ifndef _H_symbol
#include "symbol.h"
#endif
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_moduldef
#include "moduldef.h"
#endif

struct portConstructItem
  {
   char *constructName;
   int typeExpected;
   struct portConstructItem *next; 
  };

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _MODULPSR_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE long                           GetNumberOfDefmodules(void);
   LOCALE VOID                           SetNumberOfDefmodules(long);
   LOCALE VOID                           AddAfterModuleDefinedFunction(char *,VOID (*)(void),int);
   LOCALE int                            ParseDefmodule(char *);
   LOCALE VOID                           AddPortConstructItem(char *,int);
   LOCALE struct portConstructItem      *ValidPortConstructItem(char *);
   LOCALE int                            FindImportExportConflict(char *,struct defmodule *,char *);
#else
   LOCALE long                           GetNumberOfDefmodules();
   LOCALE VOID                           SetNumberOfDefmodules();
   LOCALE VOID                           AddAfterModuleDefinedFunction();
   LOCALE int                            ParseDefmodule();
   LOCALE VOID                           AddPortConstructItem();
   LOCALE struct portConstructItem      *ValidPortConstructItem();
   LOCALE int                            FindImportExportConflict();
#endif

#endif



