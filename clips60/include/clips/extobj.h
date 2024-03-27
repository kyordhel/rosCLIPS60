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
/* Purpose      External Function Definitions for COOL       */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_extobj
#define _H_extobj

#ifndef _H_moduldef
#include "moduldef.h"
#endif
#ifndef _H_constrct
#include "constrct.h"
#endif
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_expressn
#include "expressn.h"
#endif

#define GetDefclassName(x) GetConstructNameString((struct constructHeader *) x)
#define GetDefclassPPForm(x) GetConstructPPForm((struct constructHeader *) x)
#define GetDefinstancesName(x) GetConstructNameString((struct constructHeader *) x)
#define GetDefinstancesPPForm(x) GetConstructPPForm((struct constructHeader *) x)

#define DefclassModule(x) GetConstructModuleName((struct constructHeader *) x)
#define DefinstancesModule(x) GetConstructModuleName((struct constructHeader *) x)

#if ANSI_COMPILER

extern VOID SetupObjectSystem(void);

extern VOID IncrementInstanceCount(VOID *);
extern VOID DecrementInstanceCount(VOID *);
extern int GetInstancesChanged(void);
extern VOID SetInstancesChanged(int);

extern VOID *GetNextDefclass(VOID *);
extern VOID *FindDefclass(char *);
extern BOOLEAN IsDefclassDeletable(VOID *);
extern BOOLEAN Undefclass(VOID *);

#if DEFINSTANCES_CONSTRUCT
extern VOID *GetNextDefinstances(VOID *);
extern VOID *FindDefinstances(char *);
extern int IsDefinstancesDeletable(VOID *);
extern BOOLEAN Undefinstances(VOID *);
extern VOID GetDefinstancesList(DATA_OBJECT *,struct defmodule *);
#endif

extern long SaveInstances(char *,int,EXPRESSION *,BOOLEAN);

#if BSAVE_INSTANCES
extern long BinarySaveInstances(char *,int,EXPRESSION *,BOOLEAN);
#endif

#if BLOAD_INSTANCES
extern long BinaryLoadInstances(char *);
#endif

extern long LoadInstances(char *);
extern long RestoreInstances(char *);
extern VOID *MakeInstance(char *);
extern BOOLEAN DeleteInstance(VOID *);
extern BOOLEAN UnmakeInstance(VOID *);
extern VOID *CreateRawInstance(VOID *,char *);
extern VOID *FindInstance(VOID *,char *,BOOLEAN);
extern int ValidInstanceAddress(VOID *);
extern VOID DirectGetSlot(VOID *,char *,DATA_OBJECT *);
extern int DirectPutSlot(VOID *,char *,DATA_OBJECT *);
extern char *GetInstanceName(VOID *);
extern VOID *GetInstanceClass(VOID *);
extern VOID *GetNextInstance(VOID *);
extern unsigned long GetGlobalNumberOfInstances(void);
extern VOID *GetNextInstanceInScope(VOID *);
extern VOID *GetNextInstanceInClass(VOID *,VOID *);
extern VOID GetInstancePPForm(char *,int,VOID *);

extern char *GetDefmessageHandlerName(VOID *,unsigned);
extern char *GetDefmessageHandlerType(VOID *,unsigned);
extern unsigned GetNextDefmessageHandler(VOID *,unsigned);
extern unsigned FindDefmessageHandler(VOID *,char *,char *);
extern int IsDefmessageHandlerDeletable(VOID *,unsigned);
extern int UndefmessageHandler(VOID *,unsigned);
extern int WildDeleteHandler(VOID *,char *,char *);
extern VOID Send(DATA_OBJECT *,char *,char *,DATA_OBJECT *);

#if DEBUGGING_FUNCTIONS

extern VOID DescribeClass(char *,VOID *);
extern VOID BrowseClasses(char *,VOID *);

extern BOOLEAN GetDefclassWatchInstances(VOID *);
extern VOID SetDefclassWatchInstances(int,VOID *);
extern BOOLEAN GetDefclassWatchSlots(VOID *);
extern VOID SetDefclassWatchSlots(int,VOID *);

extern char *GetDefmessageHandlerPPForm(VOID *,unsigned);

extern VOID ListDefclasses(char *,struct defmodule *);
extern VOID ListDefinstances(char *,struct defmodule *);
extern VOID Instances(char *,VOID *,char *,int);
extern VOID ListDefmessageHandlers(char *,VOID *,int);

extern VOID PreviewSend(char *,VOID *,char *);

extern BOOLEAN GetDefmessageHandlerWatch(VOID *,unsigned);
extern VOID SetDefmessageHandlerWatch(int,VOID *,unsigned);

#endif

extern BOOLEAN SuperclassP(VOID *,VOID *);
extern BOOLEAN SubclassP(VOID *,VOID *);
extern BOOLEAN ClassAbstractP(VOID *);
#if DEFRULE_CONSTRUCT
extern BOOLEAN ClassReactiveP(VOID *);
#endif
extern BOOLEAN SlotExistP(VOID *,char *,BOOLEAN);
extern BOOLEAN SlotWritableP(VOID *,char *);
extern BOOLEAN SlotInitableP(VOID *,char *);
extern BOOLEAN SlotPublicP(VOID *,char *);
extern BOOLEAN SlotDirectAccessP(VOID *,char *);
extern VOID ClassSlots(VOID *,DATA_OBJECT *,int);
extern VOID GetDefmessageHandlerList(VOID *,DATA_OBJECT *,int);
extern VOID ClassSuperclasses(VOID *,DATA_OBJECT *,int);
extern VOID ClassSubclasses(VOID *,DATA_OBJECT *,int);
extern VOID SlotFacets(VOID *,char *,DATA_OBJECT *);
extern VOID SlotSources(VOID *,char *,DATA_OBJECT *);
extern VOID SlotTypes(VOID *,char *,DATA_OBJECT *);
extern VOID SlotAllowedValues(VOID *,char *,DATA_OBJECT *);
extern VOID SlotRange(VOID *,char *,DATA_OBJECT *);
extern VOID SlotCardinality(VOID *,char *,DATA_OBJECT *);
extern VOID GetDefclassList(DATA_OBJECT *,struct defmodule *);

#if INSTANCE_PATTERN_MATCHING
extern BOOLEAN SetDelayObjectPatternMatching(int);
extern BOOLEAN GetDelayObjectPatternMatching(void);
#endif

#else

extern VOID SetupObjectSystem();

extern VOID IncrementInstanceCount();
extern VOID DecrementInstanceCount();
extern int GetInstancesChanged();
extern VOID SetInstancesChanged();

extern VOID *GetNextDefclass();
extern VOID *FindDefclass();
extern BOOLEAN IsDefclassDeletable();
extern BOOLEAN Undefclass();

#if DEFINSTANCES_CONSTRUCT
extern VOID *GetNextDefinstances();
extern VOID *FindDefinstances();
extern int IsDefinstancesDeletable();
extern BOOLEAN Undefinstances();
extern VOID GetDefinstancesList();
#endif

extern long SaveInstances();

#if BSAVE_INSTANCES
extern long BinarySaveInstances();
#endif

#if BLOAD_INSTANCES
extern long BinaryLoadInstances();
#endif

extern long LoadInstances();
extern long RestoreInstances();
extern VOID *MakeInstance();
extern BOOLEAN DeleteInstance();
extern BOOLEAN UnmakeInstance();
extern VOID *CreateRawInstance();
extern VOID *FindInstance();
extern int ValidInstanceAddress();
extern VOID DirectGetSlot();
extern int DirectPutSlot();
extern char *GetInstanceName();
extern VOID *GetInstanceClass();
extern unsigned long GetGlobalNumberOfInstances();
extern VOID *GetNextInstance();
extern VOID *GetNextInstanceInScope();
extern VOID *GetNextInstanceInClass();
extern VOID GetInstancePPForm();

extern char *GetDefmessageHandlerName();
extern char *GetDefmessageHandlerType();
extern unsigned GetNextDefmessageHandler();
extern unsigned FindDefmessageHandler();
extern int IsDefmessageHandlerDeletable();
extern int UndefmessageHandler();
extern int WildDeleteHandler();
extern VOID Send();

#if DEBUGGING_FUNCTIONS

extern VOID DescribeClass();
extern VOID BrowseClasses();

extern BOOLEAN GetDefclassWatchInstances();
extern VOID SetDefclassWatchInstances();
extern BOOLEAN GetDefclassWatchSlots();
extern VOID SetDefclassWatchSlots();

extern char *GetDefmessageHandlerPPForm();

extern VOID ListDefclasses();

#if DEFINSTANCES_CONSTRUCT
extern VOID ListDefinstances();
#endif

extern VOID Instances();
extern VOID ListDefmessageHandlers();

extern VOID PreviewSend();

extern BOOLEAN GetDefmessageHandlerWatch();
extern VOID SetDefmessageHandlerWatch();

#endif

extern BOOLEAN SuperclassP();
extern BOOLEAN SubclassP();
extern BOOLEAN ClassAbstractP();
#if DEFRULE_CONSTRUCT
extern BOOLEAN ClassReactiveP();
#endif
extern BOOLEAN SlotExistP();
extern BOOLEAN SlotWritableP();
extern BOOLEAN SlotInitableP();
extern BOOLEAN SlotPublicP();
extern BOOLEAN SlotDirectAccessP();
extern VOID ClassSlots();
extern VOID GetDefmessageHandlerList();
extern VOID ClassSuperclasses();
extern VOID ClassSubclasses();
extern VOID SlotFacets();
extern VOID SlotSources();
extern VOID SlotTypes();
extern VOID SlotAllowedValues();
extern VOID SlotRange();
extern VOID SlotCardinality();
extern VOID GetDefclassList();

#if INSTANCE_PATTERN_MATCHING
extern BOOLEAN SetDelayObjectPatternMatching();
extern BOOLEAN GetDelayObjectPatternMatching();
#endif

#endif

extern int ChangesToInstances;
extern VOID *DummyInstance;

#endif



