   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*              DEFFUNCTION HEADER FILE                */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_dffnxfun
#define _H_dffnxfun

#define GetDeffunctionName(x) GetConstructNameString((struct constructHeader *) x)
#define GetDeffunctionPPForm(x) GetConstructPPForm((struct constructHeader *) x)

#define GetDeffunctionNamePointer(x) GetConstructNamePointer((struct constructHeader *) x)
#define SetDeffunctionPPForm(d,ppf) SetConstructPPForm((struct constructHeader *) d,ppf)

#define DeffunctionModule(x) GetConstructModuleName((struct constructHeader *) x)

typedef struct deffunctionStruct DEFFUNCTION;
typedef struct deffunctionModule DEFFUNCTION_MODULE;

#ifndef _H_cstrccom
#include "cstrccom.h"
#endif
#ifndef _H_moduldef
#include "moduldef.h"
#endif
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_symbol
#include "symbol.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif
#ifdef _DFFNXFUN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

struct deffunctionModule
  {   
   struct defmoduleItemHeader header;
  };

struct deffunctionStruct
  {
   struct constructHeader header;
   unsigned busy,
            executing;
   unsigned short trace;
   EXPRESSION *code;
   int minNumberOfParameters,
       maxNumberOfParameters,
       numberOfLocalVars;
  };

#if ANSI_COMPILER
LOCALE VOID SetupDeffunctions(void);
LOCALE VOID *FindDeffunction(char *);
LOCALE DEFFUNCTION *LookupDeffunctionByMdlOrScope(char *);
LOCALE DEFFUNCTION *LookupDeffunctionInScope(char *);
LOCALE BOOLEAN Undeffunction(VOID *);
LOCALE VOID *GetNextDeffunction(VOID *);
LOCALE int IsDeffunctionDeletable(VOID *);
LOCALE VOID UndeffunctionCommand(void);
LOCALE SYMBOL_HN *GetDeffunctionModuleCommand(void);
LOCALE VOID DeffunctionGetBind(DATA_OBJECT *);
LOCALE VOID DFRtnUnknown(DATA_OBJECT *);
LOCALE VOID DFWildargs(DATA_OBJECT *);
LOCALE int CheckDeffunctionCall(VOID *,int);
#if DEBUGGING_FUNCTIONS
LOCALE VOID PPDeffunctionCommand(void);
LOCALE VOID ListDeffunctionsCommand(void);
LOCALE VOID ListDeffunctions(char *,struct defmodule *);
LOCALE VOID SetDeffunctionWatch(int,VOID *);
LOCALE int GetDeffunctionWatch(VOID *);
#endif
#if (! BLOAD_ONLY) && (! RUN_TIME)
LOCALE VOID RemoveDeffunction(VOID *);
#endif

LOCALE VOID GetDeffunctionListFunction(DATA_OBJECT *);
globle VOID GetDeffunctionList(DATA_OBJECT *,struct defmodule *);

#else
LOCALE VOID SetupDeffunctions();
LOCALE VOID *FindDeffunction();
LOCALE DEFFUNCTION *LookupDeffunctionByMdlOrScope();
LOCALE DEFFUNCTION *LookupDeffunctionInScope();
LOCALE BOOLEAN Undeffunction();
LOCALE VOID *GetNextDeffunction();
LOCALE int IsDeffunctionDeletable();
LOCALE VOID UndeffunctionCommand();
LOCALE SYMBOL_HN *GetDeffunctionModuleCommand();
LOCALE VOID DeffunctionGetBind();
LOCALE VOID DFRtnUnknown();
LOCALE VOID DFWildargs();
LOCALE int CheckDeffunctionCall();

#if DEBUGGING_FUNCTIONS
LOCALE VOID PPDeffunctionCommand();
LOCALE VOID ListDeffunctionsCommand();
LOCALE VOID ListDeffunctions();
LOCALE VOID SetDeffunctionWatch();
LOCALE int GetDeffunctionWatch();
#endif
#if (! BLOAD_ONLY) && (! RUN_TIME)
LOCALE VOID RemoveDeffunction();
#endif

LOCALE VOID GetDeffunctionListFunction();
globle VOID GetDeffunctionList();

#endif

#ifndef _DFFNXFUN_SOURCE_
extern struct construct *DeffunctionConstruct;
extern int DeffunctionModuleIndex;

#if DEBUGGING_FUNCTIONS
extern BOOLEAN WatchDeffunctions;
#endif

#endif

#endif






