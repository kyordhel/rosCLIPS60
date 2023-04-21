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

#ifndef _H_genrcfun
#define _H_genrcfun

#define SaveBusyCount(gfunc)    (OldGenericBusySave = gfunc->busy)
#define RestoreBusyCount(gfunc) (gfunc->busy = OldGenericBusySave)

typedef struct defgenericModule DEFGENERIC_MODULE;
typedef struct restriction RESTRICTION;
typedef struct method DEFMETHOD;
typedef struct defgeneric DEFGENERIC;

#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#ifndef _H_constrct
#include "constrct.h"
#endif
#ifndef _H_moduldef
#include "moduldef.h"
#endif
#ifndef _H_symbol
#include "symbol.h"
#endif
#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

#if OBJECT_SYSTEM
#ifndef _H_object
#include "object.h"
#endif
#endif

struct defgenericModule
  {
   struct defmoduleItemHeader header;
  };

struct restriction
  {
   VOID **types;
   EXPRESSION *query;
   unsigned tcnt;
  };

struct method
  {
   unsigned index,busy;
   int restrictionCount,
       minRestrictions,maxRestrictions,
       localVarCount;
   unsigned system : 1;
   unsigned trace : 1;
   RESTRICTION *restrictions;
   EXPRESSION *actions;
   char *ppForm;
  };

struct defgeneric
  {
   struct constructHeader header;
   unsigned busy,trace;
   DEFMETHOD *methods;
   unsigned mcnt,new_index;
  };

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _GENRCFUN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

#if ! RUN_TIME
LOCALE BOOLEAN ClearDefgenericsReady(void);
LOCALE VOID *AllocateDefgenericModule(void);
LOCALE VOID FreeDefgenericModule(VOID *);
#endif

#if (! BLOAD_ONLY) && (! RUN_TIME)

LOCALE int ClearDefmethods(void);
LOCALE int RemoveAllExplicitMethods(DEFGENERIC *);
LOCALE VOID RemoveDefgeneric(VOID *);
LOCALE int ClearDefgenerics(void);
LOCALE VOID MethodAlterError(DEFGENERIC *);
LOCALE VOID DeleteMethodInfo(DEFGENERIC *,DEFMETHOD *);
LOCALE int MethodsExecuting(DEFGENERIC *);
#if ! OBJECT_SYSTEM
LOCALE BOOLEAN SubsumeType(int,int);
#endif
#endif

LOCALE int FindMethodByIndex(DEFGENERIC *,unsigned);
#if DEBUGGING_FUNCTIONS
LOCALE VOID PreviewGeneric(void);
LOCALE VOID PrintMethod(char *,int,DEFMETHOD *);
#endif
LOCALE DEFGENERIC *CheckGenericExists(char *,char *);
LOCALE int CheckMethodExists(char *,DEFGENERIC *,int);

#if ! OBJECT_SYSTEM
LOCALE char *TypeName(int);
#endif

LOCALE VOID PrintGenericName(char *,DEFGENERIC *);

#else

#if ! RUN_TIME
LOCALE BOOLEAN ClearDefgenericsReady();
LOCALE VOID *AllocateDefgenericModule();
LOCALE VOID FreeDefgenericModule();
#endif

#if (! BLOAD_ONLY) && (! RUN_TIME)

LOCALE int ClearDefmethods();
LOCALE int RemoveAllExplicitMethods();
LOCALE VOID RemoveDefgeneric();
LOCALE int ClearDefgenerics();
LOCALE VOID MethodAlterError();
LOCALE VOID DeleteMethodInfo();
LOCALE int MethodsExecuting();
#if ! OBJECT_SYSTEM
LOCALE BOOLEAN SubsumeType();
#endif

#endif

LOCALE int FindMethodByIndex();
#if DEBUGGING_FUNCTIONS
LOCALE VOID PreviewGeneric();
LOCALE VOID PrintMethod();
#endif
LOCALE DEFGENERIC *CheckGenericExists();
LOCALE int CheckMethodExists();

#if ! OBJECT_SYSTEM
LOCALE char *TypeName();
#endif

LOCALE VOID PrintGenericName();

#endif

#ifndef _GENRCFUN_SOURCE_
extern DEFGENERIC *CurrentGeneric;
extern DEFMETHOD *CurrentMethod;
extern DATA_OBJECT *GenericCurrentArgument;

#if DEBUGGING_FUNCTIONS
extern int WatchGenerics,WatchMethods;
#endif

#if (! RUN_TIME) && (! BLOAD_ONLY)
extern int OldGenericBusySave;
#endif

#endif

#endif





