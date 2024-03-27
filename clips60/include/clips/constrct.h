   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00 05/12/93             */
   /*                                                     */
   /*                  CONSTRUCT MODULE                   */
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
 
#ifndef _H_constrct

#define _H_constrct

struct constructHeader;
struct construct;

#ifndef _H_moduldef
#include "moduldef.h"
#endif
#ifndef _H_symbol
#include "symbol.h"
#endif

struct constructHeader
  {
   struct symbolHashNode *name;
   char *ppForm;
   struct defmoduleItemHeader *whichModule;
   long bsaveID;
   struct constructHeader *next;
  };

#define CHS (struct constructHeader *)
  
struct construct
  {
   char *constructName;
   char *pluralName;
#if ANSI_COMPILER
   int (*parseFunction)(char *);
   VOID *(*findFunction)(char *);
   struct symbolHashNode *(*getConstructNameFunction)(struct constructHeader *);
   char *(*getPPFormFunction)(struct constructHeader *);
   struct defmoduleItemHeader *(*getModuleItemFunction)(struct constructHeader *);
   VOID *(*getNextItemFunction)(VOID *);
   VOID (*setNextItemFunction)(struct constructHeader *,struct constructHeader *);
   BOOLEAN (*isConstructDeletableFunction)(VOID *);
   int (*deleteFunction)(VOID *);
   VOID (*freeFunction)(VOID *);
#else
   int (*parseFunction)();
   VOID *(*findFunction)();
   struct symbolHashNode *(*getConstructNameFunction)();
   char *(*getPPFormFunction)();
   struct defmoduleItemHeader *(*getModuleItemFunction)();
   VOID *(*getNextItemFunction)();
   VOID (*setNextItemFunction)();
   BOOLEAN (*isConstructDeletableFunction)();
   int (*deleteFunction)();
   VOID (*freeFunction)();
#endif
   struct construct *next;
  };
  
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_scanner
#include "scanner.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CONSTRCT_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif
   
#if ANSI_COMPILER
   LOCALE int                            Save(char *);
   LOCALE BOOLEAN                        AddSaveFunction(char *,VOID (*)(char *),int);
   LOCALE BOOLEAN                        RemoveSaveFunction(char *);
   LOCALE VOID                           Reset(void);
   LOCALE BOOLEAN                        AddResetFunction(char *,VOID (*)(void),int);
   LOCALE BOOLEAN                        RemoveResetFunction(char *);
   LOCALE VOID                           Clear(void);
   LOCALE BOOLEAN                        AddClearReadyFunction(char *,int (*)(void),int);
   LOCALE BOOLEAN                        RemoveClearReadyFunction(char *);
   LOCALE BOOLEAN                        AddClearFunction(char *,VOID (*)(void),int);
   LOCALE BOOLEAN                        RemoveClearFunction(char *);
   LOCALE struct construct              *AddConstruct(char *,char *,int (*)(char *),
                                                      VOID *(*)(char *),
                                                      SYMBOL_HN *(*)(struct constructHeader *),
                                                      char *(*)(struct constructHeader *),
                                                      struct defmoduleItemHeader *(*)(struct constructHeader *),
                                                      VOID *(*)(VOID *),
                                                      VOID (*)(struct constructHeader *,struct constructHeader *),
                                                      BOOLEAN (*)(VOID *),
                                                      int (*)(VOID *),
                                                      VOID (*)(VOID *));
   LOCALE int                            RemoveConstruct(char *);
   LOCALE VOID                           SetCompilationsWatch(int);
   LOCALE BOOLEAN                        GetCompilationsWatch(void);
   LOCALE VOID                           SetPrintWhileLoading(BOOLEAN);
   LOCALE BOOLEAN                        GetPrintWhileLoading(void);
   LOCALE int                            ExecutingConstruct(void);
   LOCALE VOID                           SetExecutingConstruct(int);
   LOCALE VOID                           InitializeConstructs(void);
   LOCALE int                          (*SetBeforeResetFunction(int (*)(void)))(void);
   LOCALE VOID                           OldGetConstructList(DATA_OBJECT *,
                                                          VOID *(*)(VOID *), 
                                                          char *(*)(VOID *));
   LOCALE VOID                           ResetCommand(void);
   LOCALE VOID                           ClearCommand(void);
   LOCALE BOOLEAN                        ClearReady(void);
   LOCALE struct construct              *FindConstruct(char *);
   LOCALE VOID                           DeinstallConstructHeader(struct constructHeader *);
#else
   LOCALE int                            Save();
   LOCALE BOOLEAN                        AddSaveFunction();
   LOCALE BOOLEAN                        RemoveSaveFunction();
   LOCALE VOID                           Reset();
   LOCALE BOOLEAN                        AddResetFunction();
   LOCALE BOOLEAN                        RemoveResetFunction();
   LOCALE VOID                           Clear();
   LOCALE BOOLEAN                        AddClearReadyFunction();
   LOCALE BOOLEAN                        RemoveClearReadyFunction();
   LOCALE BOOLEAN                        AddClearFunction();
   LOCALE BOOLEAN                        RemoveClearFunction();
   LOCALE struct construct              *AddConstruct();
   LOCALE int                            RemoveConstruct();
   LOCALE VOID                           SetCompilationsWatch();
   LOCALE BOOLEAN                        GetCompilationsWatch();
   LOCALE VOID                           SetPrintWhileLoading();
   LOCALE BOOLEAN                        GetPrintWhileLoading();
   LOCALE int                            ExecutingConstruct();
   LOCALE VOID                           SetExecutingConstruct();
   LOCALE VOID                           InitializeConstructs();
   LOCALE int                          (*SetBeforeResetFunction())();
   LOCALE VOID                           OldGetConstructList();
   LOCALE VOID                           ResetCommand();
   LOCALE VOID                           ClearCommand();
   LOCALE BOOLEAN                        ClearReady();
   LOCALE struct construct              *FindConstruct();
   LOCALE VOID                           DeinstallConstructHeader();
#endif 

#ifndef _CONSTRCT_SOURCE_
   extern int                            ClearInProgress;
   extern int                            ResetInProgress;
#endif

#endif







