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

#ifndef _H_insfun
#define _H_insfun

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_moduldef
#include "moduldef.h"
#endif
#ifndef _H_object
#include "object.h"
#endif

#ifndef _H_pattern
#include "pattern.h"
#endif

typedef struct igarbage
  {
   INSTANCE_TYPE *ins;
   struct igarbage *nxt;
  } IGARBAGE;

#define INSTANCE_TABLE_HASH_SIZE 683
#define InstanceSizeHeuristic(ins)      sizeof(INSTANCE_TYPE)

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _INSFUN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
LOCALE VOID IncrementInstanceCount(VOID *);
LOCALE VOID DecrementInstanceCount(VOID *);
LOCALE VOID InitializeInstanceTable(void);
LOCALE VOID CleanupInstances(void);
LOCALE unsigned HashInstance(SYMBOL_HN *);
LOCALE VOID DestroyAllInstances(void);
LOCALE VOID RemoveInstanceData(INSTANCE_TYPE *);
LOCALE INSTANCE_TYPE *FindInstanceBySymbol(SYMBOL_HN *);
globle INSTANCE_TYPE *FindInstanceInModule(SYMBOL_HN *,struct defmodule *,
                                           struct defmodule *,BOOLEAN);
LOCALE INSTANCE_SLOT *FindInstanceSlot(INSTANCE_TYPE *,SYMBOL_HN *);
LOCALE int FindInstanceTemplateSlot(DEFCLASS *,SYMBOL_HN *);
LOCALE int EvaluateAndStoreInDataObject(int,EXPRESSION *,DATA_OBJECT *);
LOCALE int PutSlotValue(INSTANCE_TYPE *,INSTANCE_SLOT *,DATA_OBJECT *,char *);
LOCALE int DirectPutSlotValue(INSTANCE_TYPE *,INSTANCE_SLOT *,DATA_OBJECT *);
LOCALE BOOLEAN ValidSlotValue(DATA_OBJECT *,SLOT_DESC *,INSTANCE_TYPE *,char *);
LOCALE INSTANCE_TYPE *CheckInstance(char *);
LOCALE VOID NoInstanceError(char *,char *);
LOCALE VOID SlotExistError(char *,char *);
LOCALE VOID StaleInstanceAddress(char *);
LOCALE int GetInstancesChanged(void);
LOCALE VOID SetInstancesChanged(int);
LOCALE VOID PrintSlot(char *,SLOT_DESC *,INSTANCE_TYPE *,char *);
LOCALE VOID PrintInstanceNameAndClass(char *,INSTANCE_TYPE *,BOOLEAN);
#else
LOCALE VOID IncrementInstanceCount();
LOCALE VOID DecrementInstanceCount();
LOCALE VOID InitializeInstanceTable();
LOCALE VOID CleanupInstances();
LOCALE unsigned HashInstance();
LOCALE VOID DestroyAllInstances();
LOCALE VOID RemoveInstanceData();
LOCALE INSTANCE_TYPE *FindInstanceBySymbol();
globle INSTANCE_TYPE *FindInstanceInModule();
LOCALE INSTANCE_SLOT *FindInstanceSlot();
LOCALE int FindInstanceTemplateSlot();
LOCALE int EvaluateAndStoreInDataObject();
LOCALE int PutSlotValue();
LOCALE int DirectPutSlotValue();
LOCALE BOOLEAN ValidSlotValue();
LOCALE INSTANCE_TYPE *CheckInstance();
LOCALE VOID NoInstanceError();
LOCALE VOID SlotExistError();
LOCALE VOID StaleInstanceAddress();
LOCALE int GetInstancesChanged();
LOCALE VOID SetInstancesChanged();
LOCALE VOID PrintSlot();
LOCALE VOID PrintInstanceNameAndClass();
#endif

#ifndef _INSFUN_SOURCE_
extern INSTANCE_TYPE **InstanceTable;
extern int WithinInit;
extern int MaintainGarbageInstances;
extern int MkInsMsgPass;
extern int ChangesToInstances;
extern IGARBAGE *InstanceGarbageList;
extern struct patternEntityRecord InstanceInfo;
#endif

#endif







