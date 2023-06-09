   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                OBJECT SYSTEM DEFINITIONS            */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_object
#define _H_object

typedef struct defclassModule DEFCLASS_MODULE;
typedef struct defclass DEFCLASS;
typedef struct packedClassLinks PACKED_CLASS_LINKS;
typedef struct classLink CLASS_LINK;
typedef struct slotName SLOT_NAME;
typedef struct slotDescriptor SLOT_DESC;
typedef struct messageHandler HANDLER;
typedef struct instance INSTANCE_TYPE;
typedef struct instanceSlot INSTANCE_SLOT;

/* Maximum # of simultaneous class hierarchy traversals
   should be a multiple of BITS_PER_BYTE and less than MAX_INT      */

#define MAX_TRAVERSALS  128
#define TRAVERSAL_BYTES 16       /* (MAX_TRAVERSALS / BITS_PER_BYTE) */

#define VALUE_REQUIRED     0
#define VALUE_PROHIBITED   1
#define VALUE_NOT_REQUIRED 2

#ifndef _H_constrct
#include "constrct.h"
#endif
#ifndef _H_constrnt
#include "constrnt.h"
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
#ifndef _H_multifld
#include "multifld.h"
#endif
#ifndef _H_symbol
#include "symbol.h"
#endif

#ifndef _H_match
#include "match.h"
#endif
#ifndef _H_pattern
#include "pattern.h"
#endif

#define GetInstanceSlotLength(sp) GetMFLength(sp->value)

struct packedClassLinks
  {
   unsigned short classCount;
   DEFCLASS **classArray;
  };

struct defclassModule
  {
   struct defmoduleItemHeader header;
  };

struct defclass
  {
   struct constructHeader header;
   unsigned installed      : 1;
   unsigned system         : 1;
   unsigned abstract       : 1;
   unsigned reactive       : 1;
   unsigned traceInstances : 1;
   unsigned traceSlots     : 1;
   unsigned short id;
   unsigned busy,
            hashTableIndex;
   PACKED_CLASS_LINKS directSuperclasses,
                      directSubclasses,
                      allSuperclasses;
   SLOT_DESC *slots,
             **instanceTemplate;
   unsigned *slotNameMap;
   unsigned slotCount,
            localInstanceSlotCount,
            instanceSlotCount,
            maxSlotNameID;
   INSTANCE_TYPE *instanceList,
                 *instanceListBottom;
   HANDLER *handlers;
   unsigned *handlerOrderMap;
   unsigned handlerCount;
   DEFCLASS *nxtHash;
   BITMAP_HN *scopeMap;
   char traversalRecord[TRAVERSAL_BYTES];
  };
  
struct classLink
  {
   DEFCLASS *cls;
   struct classLink *nxt;
  };

struct slotName
  {
   unsigned hashTableIndex,
            use,
            id;
   SYMBOL_HN *name,
             *putHandlerName;
   struct slotName *nxt;
   long bsaveIndex;
  };

struct instanceSlot
  {
   SLOT_DESC *desc;
   unsigned valueRequired : 1;
   unsigned override      : 1;
   unsigned type          : 6;
   VOID *value;
  };
  
struct slotDescriptor
  {
   unsigned shared                   : 1;
   unsigned multiple                 : 1;
   unsigned composite                : 1;
   unsigned noInherit                : 1;
   unsigned noWrite                  : 1;
   unsigned initializeOnly           : 1;
   unsigned dynamicDefault           : 1;
   unsigned defaultSpecified         : 1;
   unsigned noDefault                : 1;
   unsigned reactive                 : 1;
   unsigned publicVisibility         : 1;
   unsigned createReadAccessor       : 1;
   unsigned createWriteAccessor      : 1;
   unsigned overrideMessageSpecified : 1;
   DEFCLASS *cls;
   SLOT_NAME *slotName;
   SYMBOL_HN *overrideMessage;
   VOID *defaultValue;
   CONSTRAINT_RECORD *constraint;
   unsigned sharedCount;
   long bsaveIndex;
   INSTANCE_SLOT sharedValue;
  };

struct instance
  {
   struct patternEntity header;
   VOID *partialMatchList;
   INSTANCE_SLOT *basisSlots;
   unsigned installed            : 1;
   unsigned garbage              : 1;
   unsigned initializeInProgress : 1;
   SYMBOL_HN *name;
   int depth;
   unsigned hashTableIndex;
   unsigned busy;
   DEFCLASS *cls;
   INSTANCE_TYPE *prvClass,*nxtClass,
                 *prvHash,*nxtHash,
                 *prvList,*nxtList;
   INSTANCE_SLOT **slotAddresses,
                 *slots;
  };
 
struct messageHandler
  {
   unsigned system         : 1;
   unsigned type           : 2;
   unsigned mark           : 1;
   unsigned trace          : 1;
   unsigned busy;
   SYMBOL_HN *name;
   DEFCLASS *cls;
   int minParams,
       maxParams,
       localVarCount;
   EXPRESSION *actions;
   char *ppForm;
  };

#endif






