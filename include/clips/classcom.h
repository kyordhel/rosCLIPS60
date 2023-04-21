   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                                                     */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_classcom
#define _H_classcom

#define GetDefclassName(x) GetConstructNameString((struct constructHeader *) x)
#define GetDefclassPPForm(x) GetConstructPPForm((struct constructHeader *) x)

#define GetDefclassNamePointer(x) GetConstructNamePointer((struct constructHeader *) x)
#define GetDefclassModule(x) GetConstructModuleItem((struct constructHeader *) x)

#define SetNextDefclass(c,t) SetNextConstruct((struct constructHeader *) c, \
                                              (struct constructHeader *) t)

#define SetDefclassPPForm(c,ppf) SetConstructPPForm((struct constructHeader *) c,ppf)

#define DefclassModule(x) GetConstructModuleName((struct constructHeader *) x)

#ifndef _H_cstrccom
#include "cstrccom.h"
#endif
#ifndef _H_moduldef
#include "moduldef.h"
#endif
#ifndef _H_object
#include "object.h"
#endif
#ifndef _H_symbol
#include "symbol.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CLASSCOM_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
LOCALE VOID *FindDefclass(char *);
LOCALE DEFCLASS *LookupDefclassByMdlOrScope(char *);
LOCALE DEFCLASS *LookupDefclassInScope(char *);
LOCALE DEFCLASS *LookupDefclassAnywhere(struct defmodule *,char *);
LOCALE BOOLEAN DefclassInScope(DEFCLASS *,struct defmodule *);
LOCALE VOID *GetNextDefclass(VOID *);
LOCALE BOOLEAN IsDefclassDeletable(VOID *);

LOCALE VOID UndefclassCommand(void);

#if DEBUGGING_FUNCTIONS
LOCALE VOID PPDefclassCommand(void);
LOCALE VOID ListDefclassesCommand(void);
LOCALE VOID ListDefclasses(char *,struct defmodule *);
LOCALE BOOLEAN GetDefclassWatchInstances(VOID *);
LOCALE VOID SetDefclassWatchInstances(int,VOID *);
LOCALE BOOLEAN GetDefclassWatchSlots(VOID *);
LOCALE VOID SetDefclassWatchSlots(int,VOID *);
LOCALE BOOLEAN DefclassWatchAccess(int,int,EXPRESSION *);
LOCALE BOOLEAN DefclassWatchPrint(char *,int,EXPRESSION *);
#endif

LOCALE VOID GetDefclassListFunction(DATA_OBJECT *);
LOCALE VOID GetDefclassList(DATA_OBJECT *,struct defmodule *);
LOCALE BOOLEAN Undefclass(VOID *);
LOCALE BOOLEAN HasSuperclass(DEFCLASS *,DEFCLASS *);

LOCALE SYMBOL_HN *CheckClassAndSlot(char *,DEFCLASS **);

#if (! BLOAD_ONLY) && (! RUN_TIME)
LOCALE VOID SaveDefclasses(char *);
#endif

#else
LOCALE VOID *FindDefclass();
LOCALE DEFCLASS *LookupDefclassByMdlOrScope();
LOCALE DEFCLASS *LookupDefclassInScope();
LOCALE DEFCLASS *LookupDefclassAnywhere();
LOCALE BOOLEAN DefclassInScope();
LOCALE VOID *GetNextDefclass();
LOCALE BOOLEAN IsDefclassDeletable();

LOCALE VOID UndefclassCommand();

#if DEBUGGING_FUNCTIONS
LOCALE VOID PPDefclassCommand();
LOCALE VOID ListDefclassesCommand();
LOCALE VOID ListDefclasses();
LOCALE BOOLEAN GetDefclassWatchInstances();
LOCALE VOID SetDefclassWatchInstances();
LOCALE BOOLEAN GetDefclassWatchSlots();
LOCALE VOID SetDefclassWatchSlots();
LOCALE BOOLEAN DefclassWatchAccess();
LOCALE BOOLEAN DefclassWatchPrint();
#endif

LOCALE VOID GetDefclassListFunction();
LOCALE VOID GetDefclassList();
LOCALE BOOLEAN Undefclass();
LOCALE BOOLEAN HasSuperclass();

LOCALE SYMBOL_HN *CheckClassAndSlot();

#if (! BLOAD_ONLY) && (! RUN_TIME)
LOCALE VOID SaveDefclasses();
#endif

#endif

#endif





