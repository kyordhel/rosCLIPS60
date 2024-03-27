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

#ifndef _H_classfun
#define _H_classfun

#ifndef _H_object
#include "object.h"
#endif

#define TestTraversalID(traversalRecord,id) TestBitMap(traversalRecord,id)
#define SetTraversalID(traversalRecord,id) SetBitMap(traversalRecord,id)
#define ClearTraversalID(traversalRecord,id) ClearBitMap(traversalRecord,id)

#define CLASS_TABLE_HASH_SIZE     167
#define SLOT_NAME_TABLE_HASH_SIZE 167

#define INITIAL_OBJECT_CLASS_NAME "INITIAL-OBJECT"

#define ISA_ID  0
#define NAME_ID 1

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CLASSFUN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
LOCALE VOID IncrementDefclassBusyCount(VOID *);
LOCALE VOID DecrementDefclassBusyCount(VOID *);
LOCALE BOOLEAN InstancesPurge(void);

#if ! RUN_TIME
LOCALE VOID InitializeClasses(void);
#endif
LOCALE SLOT_DESC *FindClassSlot(DEFCLASS *,SYMBOL_HN *);
LOCALE VOID ClassExistError(char *,char *);
LOCALE VOID DeleteClassLinks(CLASS_LINK *);
LOCALE VOID PrintClassName(char *,DEFCLASS *,BOOLEAN);

#if DEBUGGING_FUNCTIONS || ((! BLOAD_ONLY) && (! RUN_TIME))
LOCALE VOID PrintPackedClassLinks(char *,char *,PACKED_CLASS_LINKS *);
#endif

#if ! RUN_TIME
LOCALE VOID PutClassInTable(DEFCLASS *);
LOCALE VOID RemoveClassFromTable(DEFCLASS *);
LOCALE VOID AddClassLink(PACKED_CLASS_LINKS *,DEFCLASS *,int);
LOCALE VOID DeleteSubclassLink(DEFCLASS *,DEFCLASS *);
LOCALE DEFCLASS *NewClass(SYMBOL_HN *);
LOCALE VOID DeletePackedClassLinks(PACKED_CLASS_LINKS *,int);
LOCALE VOID AssignClassID(DEFCLASS *);
LOCALE SLOT_NAME *AddSlotName(SYMBOL_HN *,unsigned,int);
LOCALE VOID DeleteSlotName(SLOT_NAME *);
LOCALE VOID RemoveDefclass(VOID *);
LOCALE VOID InstallClass(DEFCLASS *,int);
#endif
  
#if (! BLOAD_ONLY) && (! RUN_TIME)
LOCALE int IsClassBeingUsed(DEFCLASS *);
LOCALE int RemoveAllUserClasses(void);
LOCALE int DeleteClassUAG(DEFCLASS *);
LOCALE VOID MarkBitMapSubclasses(char *,DEFCLASS *,int);
#endif

LOCALE int FindSlotNameID(SYMBOL_HN *);
LOCALE SYMBOL_HN *FindIDSlotName(unsigned);
LOCALE SLOT_NAME *FindIDSlotNameHash(unsigned);
LOCALE int GetTraversalID(void);
LOCALE VOID ReleaseTraversalID(void);
LOCALE unsigned HashClass(SYMBOL_HN *);

#else
LOCALE VOID IncrementDefclassBusyCount();
LOCALE VOID DecrementDefclassBusyCount();
LOCALE BOOLEAN InstancesPurge();

#if ! RUN_TIME
LOCALE VOID InitializeClasses();
#endif
LOCALE SLOT_DESC *FindClassSlot();
LOCALE VOID ClassExistError();
LOCALE VOID DeleteClassLinks();
LOCALE VOID PrintClassName();

#if DEBUGGING_FUNCTIONS || ((! BLOAD_ONLY) && (! RUN_TIME))
LOCALE VOID PrintPackedClassLinks();
#endif

#if ! RUN_TIME
LOCALE VOID PutClassInTable();
LOCALE VOID RemoveClassFromTable();
LOCALE VOID AddClassLink();
LOCALE VOID DeleteSubclassLink();
LOCALE DEFCLASS *NewClass();
LOCALE VOID DeletePackedClassLinks();
LOCALE VOID AssignClassID();
LOCALE SLOT_NAME *AddSlotName();
LOCALE VOID DeleteSlotName();
LOCALE VOID RemoveDefclass();
LOCALE VOID InstallClass();
#endif

#if (! BLOAD_ONLY) && (! RUN_TIME)
LOCALE int IsClassBeingUsed();
LOCALE int RemoveAllUserClasses();
LOCALE int DeleteClassUAG();
LOCALE VOID MarkBitMapSubclasses();
#endif

LOCALE int FindSlotNameID();
LOCALE SYMBOL_HN *FindIDSlotName();
LOCALE SLOT_NAME *FindIDSlotNameHash();
LOCALE int GetTraversalID();
LOCALE VOID ReleaseTraversalID();
LOCALE unsigned HashClass();

#endif

#ifndef _CLASSFUN_SOURCE_
extern DEFCLASS **ClassIDMap;
extern DEFCLASS **ClassTable;
extern SLOT_NAME **SlotNameTable;
extern DEFCLASS *PrimitiveClassMap[];
extern unsigned short MaxClassID;
extern SYMBOL_HN *ISA_SYMBOL,*NAME_SYMBOL;
#if DEFRULE_CONSTRUCT
extern SYMBOL_HN *INITIAL_OBJECT_SYMBOL;
#endif
#if DEBUGGING_FUNCTIONS
extern int WatchInstances,WatchSlots;
#endif
#endif

#endif









