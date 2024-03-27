   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00 05/12/93             */
   /*                                                     */
   /*           CONSTRUCT COMMAND HEADER MODULE           */
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
 
#ifndef _H_cstrccom

#define _H_cstrccom

#ifndef _H_moduldef
#include "moduldef.h"
#endif
#ifndef _H_constrct
#include "constrct.h"
#endif
        
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CSTRCCOM_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif
   
#if ANSI_COMPILER
#if (! RUN_TIME)
   LOCALE VOID                           AddConstructToModule(struct constructHeader *);
#endif
   LOCALE BOOLEAN                        DeleteNamedConstruct(char *,struct construct *);
   LOCALE VOID                          *FindNamedConstruct(char *,struct construct *);
   LOCALE VOID                           UndefconstructCommand(char *,struct construct *);
   LOCALE int                            PPConstruct(char *,char *,struct construct *);
   LOCALE SYMBOL_HN                     *GetConstructModuleCommand(char *,struct construct *);
   LOCALE struct defmodule              *GetConstructModule(char *,struct construct *);
   LOCALE BOOLEAN                        Undefconstruct(VOID *,struct construct *);
   LOCALE VOID                           SaveConstruct(char *,struct construct *);
   LOCALE char                          *GetConstructNameString(struct constructHeader *);
   LOCALE char                          *GetConstructModuleName(struct constructHeader *);
   LOCALE SYMBOL_HN                     *GetConstructNamePointer(struct constructHeader *);
   LOCALE VOID                           GetConstructListFunction(char *,DATA_OBJECT_PTR,
                                                                  struct construct *);
   LOCALE VOID                           GetConstructList(DATA_OBJECT_PTR,struct construct *,
                                                          struct defmodule *);
   LOCALE VOID                           ListConstructCommand(char *,struct construct *);
   LOCALE VOID                           ListConstruct(struct construct *,char *,struct defmodule *);
   LOCALE VOID                           SetNextConstruct(struct constructHeader *,struct constructHeader *);
   LOCALE struct defmoduleItemHeader    *GetConstructModuleItem(struct constructHeader *);
   LOCALE char                          *GetConstructPPForm(struct constructHeader *);
   LOCALE VOID                           PPConstructCommand(char *,struct construct *);
   LOCALE struct constructHeader        *GetNextConstructItem(struct constructHeader *,int);
   LOCALE struct defmoduleItemHeader    *GetConstructModuleItemByIndex(struct defmodule *,int);
   LOCALE VOID                           FreeConstructHeaderModule(struct defmoduleItemHeader *,
                                                                   struct construct *);
   LOCALE long                           DoForAllConstructs(VOID (*)(struct constructHeader *,VOID *),int,int,VOID *);
   LOCALE VOID                           InitializeConstructHeader(char *,struct constructHeader *,SYMBOL_HN *);
   LOCALE VOID                           SetConstructPPForm(struct constructHeader *,char *);
   LOCALE VOID                          *LookupConstruct(struct construct *,char *,BOOLEAN);
#if DEBUGGING_FUNCTIONS
   LOCALE BOOLEAN                        ConstructPrintWatchAccess(struct construct *,char *,
                                            EXPRESSION *,BOOLEAN (*)(VOID *),
                                            VOID (*)(BOOLEAN,VOID *));
   LOCALE BOOLEAN                        ConstructSetWatchAccess(struct construct *,BOOLEAN,
                                            EXPRESSION *,BOOLEAN (*)(VOID *),
                                            VOID (*)(BOOLEAN,VOID *));
#endif
#else
#if (! RUN_TIME)
   LOCALE VOID                           AddConstructToModule();
#endif
   LOCALE BOOLEAN                        DeleteNamedConstruct();
   LOCALE VOID                          *FindNamedConstruct();
   LOCALE VOID                           UndefconstructCommand();
   LOCALE int                            PPConstruct();
   LOCALE SYMBOL_HN                     *GetConstructModuleCommand();
   LOCALE struct defmodule              *GetConstructModule();
   LOCALE BOOLEAN                        Undefconstruct();
   LOCALE VOID                           SaveConstruct();
   LOCALE char                          *GetConstructNameString();
   LOCALE char                          *GetConstructModuleName();
   LOCALE SYMBOL_HN                     *GetConstructNamePointer();
   LOCALE VOID                           GetConstructListFunction();
   LOCALE VOID                           GetConstructList();
   LOCALE VOID                           ListConstructCommand();
   LOCALE VOID                           ListConstruct();
   LOCALE VOID                           SetNextConstruct();
   LOCALE struct defmoduleItemHeader    *GetConstructModuleItem();
   LOCALE char                          *GetConstructPPForm();
   LOCALE VOID                           PPConstructCommand();
   LOCALE struct constructHeader        *GetNextConstructItem();
   LOCALE struct defmoduleItemHeader    *GetConstructModuleItemByIndex();
   LOCALE VOID                           FreeConstructHeaderModule();
   LOCALE long                           DoForAllConstructs();
   LOCALE VOID                           InitializeConstructHeader();
   LOCALE VOID                           SetConstructPPForm();
   LOCALE VOID                          *LookupConstruct();
#if DEBUGGING_FUNCTIONS
   LOCALE BOOLEAN                        ConstructPrintWatchAccess();
   LOCALE BOOLEAN                        ConstructSetWatchAccess();
#endif

#endif 

#endif







