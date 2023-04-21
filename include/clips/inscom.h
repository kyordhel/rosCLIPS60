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

#ifndef _H_inscom
#define _H_inscom

#ifndef _H_object
#include "object.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _INSCOM_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
LOCALE VOID SetupInstances(void);
LOCALE BOOLEAN DeleteInstance(VOID *);
LOCALE BOOLEAN UnmakeInstance(VOID *);
#if DEBUGGING_FUNCTIONS
LOCALE VOID InstancesCommand(void);
LOCALE VOID PPInstanceCommand(void);
LOCALE VOID Instances(char *,VOID *,char *,int);
#endif
LOCALE VOID *MakeInstance(char *);
LOCALE VOID *CreateRawInstance(VOID *,char *);
LOCALE VOID *FindInstance(VOID *,char *,BOOLEAN);
LOCALE int ValidInstanceAddress(VOID *);
LOCALE VOID DirectGetSlot(VOID *,char *,DATA_OBJECT *);
LOCALE int DirectPutSlot(VOID *,char *,DATA_OBJECT *);
LOCALE char *GetInstanceName(VOID *);
LOCALE VOID *GetInstanceClass(VOID *);
LOCALE unsigned long GetGlobalNumberOfInstances(void);
LOCALE VOID *GetNextInstance(VOID *);
LOCALE VOID *GetNextInstanceInScope(VOID *);
LOCALE VOID *GetNextInstanceInClass(VOID *,VOID *);
LOCALE VOID GetInstancePPForm(char *,int,VOID *);
LOCALE VOID ClassCommand(DATA_OBJECT *);
LOCALE BOOLEAN DeleteInstanceCommand(void);
LOCALE BOOLEAN UnmakeInstanceCommand(void);
LOCALE VOID SymbolToInstanceName(DATA_OBJECT *);
LOCALE VOID *InstanceNameToSymbol(void);
LOCALE VOID InstanceAddressCommand(DATA_OBJECT *);
LOCALE VOID InstanceNameCommand(DATA_OBJECT *);
LOCALE BOOLEAN InstanceAddressPCommand(void);
LOCALE BOOLEAN InstanceNamePCommand(void);
LOCALE BOOLEAN InstancePCommand(void);
LOCALE BOOLEAN InstanceExistPCommand(void);
#else
LOCALE VOID SetupInstances();
LOCALE BOOLEAN DeleteInstance();
LOCALE BOOLEAN UnmakeInstance();
#if DEBUGGING_FUNCTIONS
LOCALE VOID InstancesCommand();
LOCALE VOID PPInstanceCommand();
LOCALE VOID Instances();
#endif
LOCALE VOID *MakeInstance();
LOCALE VOID *CreateRawInstance();
LOCALE VOID *FindInstance();
LOCALE int ValidInstanceAddress();
LOCALE VOID DirectGetSlot();
LOCALE int DirectPutSlot();
LOCALE char *GetInstanceName();
LOCALE VOID *GetInstanceClass();
LOCALE unsigned long GetGlobalNumberOfInstances();
LOCALE VOID *GetNextInstance();
LOCALE VOID *GetNextInstanceInScope();
LOCALE VOID *GetNextInstanceInClass();
LOCALE VOID GetInstancePPForm();
LOCALE VOID ClassCommand();
LOCALE BOOLEAN DeleteInstanceCommand();
LOCALE BOOLEAN UnmakeInstanceCommand();
LOCALE VOID SymbolToInstanceName();
LOCALE VOID *InstanceNameToSymbol();
LOCALE VOID InstanceAddressCommand();
LOCALE VOID InstanceNameCommand();
LOCALE BOOLEAN InstanceAddressPCommand();
LOCALE BOOLEAN InstanceNamePCommand();
LOCALE BOOLEAN InstancePCommand();
LOCALE BOOLEAN InstanceExistPCommand();
#endif

#ifndef _INSCOM_SOURCE_
extern INSTANCE_TYPE DummyInstance;
#endif

#endif





