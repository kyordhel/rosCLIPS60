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

#ifndef _H_genrccom
#define _H_genrccom

#define GetDefgenericName(x) GetConstructNameString((struct constructHeader *) x)
#define GetDefgenericPPForm(x) GetConstructPPForm((struct constructHeader *) x)

#define SetNextDefgeneric(g,t) SetNextConstruct((struct constructHeader *) g, \
                                                (struct constructHeader *) t)
#define GetDefgenericNamePointer(x) GetConstructNamePointer((struct constructHeader *) x)
#define SetDefgenericPPForm(g,ppf) SetConstructPPForm((struct constructHeader *) g,ppf)

#define DefgenericModule(x) GetConstructModuleName((struct constructHeader *) x)

#ifndef _H_constrct
#include "constrct.h"
#endif
#ifndef _H_cstrccom
#include "cstrccom.h"
#endif
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_moduldef
#include "moduldef.h"
#endif
#ifndef _H_genrcfun
#include "genrcfun.h"
#endif
#ifndef _H_symbol
#include "symbol.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _GENRCCOM_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE VOID SetupGenericFunctions(void);
LOCALE VOID *FindDefgeneric(char *);
LOCALE DEFGENERIC *LookupDefgenericByMdlOrScope(char *);
LOCALE DEFGENERIC *LookupDefgenericInScope(char *);
LOCALE VOID *GetNextDefgeneric(VOID *);
LOCALE unsigned GetNextDefmethod(VOID *,unsigned);
LOCALE int IsDefgenericDeletable(VOID *);
LOCALE int IsDefmethodDeletable(VOID *,unsigned);
LOCALE VOID UndefgenericCommand(void);
LOCALE SYMBOL_HN *GetDefgenericModuleCommand(void);
LOCALE VOID UndefmethodCommand(void);

LOCALE BOOLEAN Undefgeneric(VOID *);
LOCALE BOOLEAN Undefmethod(VOID *,unsigned);

#if ! OBJECT_SYSTEM
LOCALE VOID TypeCommand(DATA_OBJECT *);
#endif

#if DEBUGGING_FUNCTIONS
LOCALE VOID GetDefmethodDescription(char *,int,VOID *,unsigned);
LOCALE BOOLEAN GetDefgenericWatch(VOID *);
LOCALE VOID SetDefgenericWatch(int,VOID *);
LOCALE BOOLEAN GetDefmethodWatch(VOID *,unsigned);
LOCALE VOID SetDefmethodWatch(int,VOID *,unsigned);
LOCALE VOID PPDefgenericCommand(void);
LOCALE VOID PPDefmethodCommand(void);
LOCALE VOID ListDefmethodsCommand(void);
LOCALE char *GetDefmethodPPForm(VOID *,unsigned);
LOCALE VOID ListDefgenericsCommand(void);
LOCALE VOID ListDefgenerics(char *,struct defmodule *);
LOCALE VOID ListDefmethods(char *,VOID *);
#endif

LOCALE VOID GetDefgenericListFunction(DATA_OBJECT *);
globle VOID GetDefgenericList(DATA_OBJECT *,struct defmodule *);
LOCALE VOID GetDefmethodListCommand(DATA_OBJECT *);
LOCALE VOID GetDefmethodList(VOID *,DATA_OBJECT *);
LOCALE VOID GetMethodRestrictionsCommand(DATA_OBJECT *);
LOCALE VOID GetMethodRestrictions(VOID *,unsigned,DATA_OBJECT *);

#else

LOCALE VOID SetupGenericFunctions();
LOCALE VOID *FindDefgeneric();
LOCALE DEFGENERIC *LookupDefgenericByMdlOrScope();
LOCALE DEFGENERIC *LookupDefgenericInScope();
LOCALE VOID *GetNextDefgeneric();
LOCALE unsigned GetNextDefmethod();
LOCALE int IsDefgenericDeletable();
LOCALE int IsDefmethodDeletable();
LOCALE VOID UndefgenericCommand();
LOCALE SYMBOL_HN *GetDefgenericModuleCommand();
LOCALE VOID UndefmethodCommand();

LOCALE BOOLEAN Undefgeneric();
LOCALE BOOLEAN Undefmethod();

#if ! OBJECT_SYSTEM
LOCALE VOID TypeCommand();
#endif

#if DEBUGGING_FUNCTIONS
LOCALE VOID GetDefmethodDescription();
LOCALE BOOLEAN GetDefgenericWatch();
LOCALE VOID SetDefgenericWatch();
LOCALE BOOLEAN GetDefmethodWatch();
LOCALE VOID SetDefmethodWatch();
LOCALE VOID PPDefgenericCommand();
LOCALE VOID PPDefmethodCommand();
LOCALE VOID ListDefmethodsCommand();
LOCALE char *GetDefmethodPPForm();
LOCALE VOID ListDefgenericsCommand();
LOCALE VOID ListDefgenerics();
LOCALE VOID ListDefmethods();
#endif

LOCALE VOID GetDefgenericListFunction();
globle VOID GetDefgenericList();
LOCALE VOID GetDefmethodListCommand();
LOCALE VOID GetDefmethodList();
LOCALE VOID GetMethodRestrictionsCommand();
LOCALE VOID GetMethodRestrictions();

#endif

#ifndef _GENRCCOM_SOURCE_
extern struct construct *DefgenericConstruct;
extern int DefgenericModuleIndex;
#endif

#endif





