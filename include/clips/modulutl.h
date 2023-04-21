   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            DEFMODULE UTILITY HEADER FILE            */
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

#ifndef _H_modulutl
#define _H_modulutl

#ifndef _H_symbol
#include "symbol.h"
#endif
#ifndef _H_moduldef
#include "moduldef.h"
#endif
  
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _MODULUTL_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE int                            FindModuleSeparator(char *);
   LOCALE SYMBOL_HN                     *ExtractModuleName(int,char *);
   LOCALE SYMBOL_HN                     *ExtractConstructName(int,char *);
   LOCALE char                          *ExtractModuleAndConstructName(char *);
   LOCALE VOID                          *FindImportedConstruct(char *,struct defmodule *,
                                                               char *,int *,int,struct defmodule *);
   LOCALE VOID                           AmbiguousReferenceErrorMessage(char *,char *);
   LOCALE VOID                           MarkModulesAsUnvisited(void);
#else
   LOCALE int                            FindModuleSeparator();
   LOCALE SYMBOL_HN                     *ExtractModuleName();
   LOCALE SYMBOL_HN                     *ExtractConstructName();
   LOCALE char                          *ExtractModuleAndConstructName();
   LOCALE VOID                          *FindImportedConstruct();
   LOCALE VOID                           AmbiguousReferenceErrorMessage();
   LOCALE VOID                           MarkModulesAsUnvisited();
#endif
   
#endif



