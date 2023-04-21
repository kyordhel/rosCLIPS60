   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*               EVALUATION HEADER FILE                */
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

#ifndef _H_evaluatn

#define _H_evaluatn

struct entityRecord;
struct dataObject;

#ifndef _H_constant
#include "constant.h"
#endif
#ifndef _H_symbol
#include "symbol.h"
#endif
#ifndef _H_expressn
#include "expressn.h"
#endif

struct dataObject
  {
   VOID *supplementalInfo;
   int type;
   VOID *value;
   int begin;
   int end;
   struct dataObject *next;
  };

typedef struct dataObject DATA_OBJECT;
typedef struct dataObject * DATA_OBJECT_PTR;

#if ANSI_COMPILER
#define DATA_OBJECT_PTR_ARG DATA_OBJECT_PTR
#else
#define DATA_OBJECT_PTR_ARG
#endif

struct entityRecord
  {
   unsigned int type : 13;
   unsigned int copyToEvaluate : 1;
   unsigned int bitMap : 1;
   unsigned int addsToRuleComplexity : 1;
#if ANSI_COMPILER
   VOID (*shortPrintFunction)(char *,VOID *);
   VOID (*longPrintFunction)(char *,VOID *);
   BOOLEAN (*deleteFunction)(VOID *);
   BOOLEAN (*evaluateFunction)(VOID *,DATA_OBJECT *);
   VOID *(*getNextFunction)(VOID *);
   VOID (*decrementBusyCount)(VOID *);
   VOID (*incrementBusyCount)(VOID *);
   VOID (*propagateDepth)(VOID *);
   VOID (*markNeeded)(VOID *);
   VOID (*install)(VOID *);
   VOID (*deinstall)(VOID *);
#else
   VOID (*shortPrintFunction)();
   VOID (*longPrintFunction)();
   BOOLEAN (*deleteFunction)();
   BOOLEAN (*evaluateFunction)();
   VOID *(*getNextFunction)();
   VOID (*decrementBusyCount)();
   VOID (*incrementBusyCount)();
   VOID (*propagateDepth)();
   VOID (*markNeeded)();
   VOID (*install)();
   VOID (*deinstall)();
#endif
  };
  
typedef struct entityRecord ENTITY_RECORD;
typedef struct entityRecord * ENTITY_RECORD_PTR;

#define GetDOLength(target)       (((target).end - (target).begin) + 1)
#define GetpDOLength(target)      (((target)->end - (target)->begin) + 1)
#define GetDOBegin(target)        ((target).begin + 1)
#define GetDOEnd(target)          ((target).end + 1)
#define GetpDOBegin(target)       ((target)->begin + 1)
#define GetpDOEnd(target)         ((target)->end + 1)
#define SetDOBegin(target,val)   ((target).begin = (val) - 1) 
#define SetDOEnd(target,val)     ((target).end = (val) - 1)
#define SetpDOBegin(target,val)   ((target)->begin = (val) - 1) 
#define SetpDOEnd(target,val)     ((target)->end = (val) - 1)

#define DOPToString(target) (((struct symbolHashNode *) (target->value))->contents)
#define DOPToDouble(target) (((struct floatHashNode *) (target->value))->contents)
#define DOPToFloat(target) ((float) (((struct floatHashNode *) (target->value))->contents))
#define DOPToLong(target) (((struct integerHashNode *) (target->value))->contents)
#define DOPToInteger(target) ((int) (((struct integerHashNode *) (target->value))->contents))
#define DOPToPointer(target)       ((target)->value) 

#define DOToString(target) (((struct symbolHashNode *) (target.value))->contents)
#define DOToDouble(target) (((struct floatHashNode *) (target.value))->contents)
#define DOToFloat(target) ((float) (((struct floatHashNode *) (target.value))->contents))
#define DOToLong(target) (((struct integerHashNode *) (target.value))->contents)
#define DOToInteger(target) ((int) (((struct integerHashNode *) (target.value))->contents))
#define DOToPointer(target)        ((target).value) 

#define CoerceToLongInteger(t,v) ((t == INTEGER) ? ValueToLong(v) : (long int) ValueToDouble(v))
#define CoerceToInteger(t,v) ((t == INTEGER) ? (int) ValueToLong(v) : (int) ValueToDouble(v))
#define CoerceToDouble(t,v) ((t == INTEGER) ? (double) ValueToLong(v) : ValueToDouble(v))
  
#define GetFirstArgument()           (CurrentExpression->argList)
#define GetNextArgument(ep)          (ep->nextArg)

#define BITS_PER_BYTE    8

#define BitwiseTest(n,b)   ((n) & (char) (1 << (b)))
#define BitwiseSet(n,b)    (n |= (char) (1 << (b)))
#define BitwiseClear(n,b)  (n &= (char) ~(1 << (b)))

#define TestBitMap(map,id)  BitwiseTest(map[id / BITS_PER_BYTE],id % BITS_PER_BYTE)
#define SetBitMap(map,id)   BitwiseSet(map[id / BITS_PER_BYTE],id % BITS_PER_BYTE)
#define ClearBitMap(map,id) BitwiseClear(map[id / BITS_PER_BYTE],id % BITS_PER_BYTE)

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _EVALUATN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE int                            EvaluateExpression(struct expr *,struct dataObject *);
   LOCALE VOID                           SetEvaluationError(BOOLEAN);
   LOCALE int                            GetEvaluationError(void);
   LOCALE VOID                           SetHaltExecution(int);
   LOCALE int                            GetHaltExecution(void);
   LOCALE VOID                           ReturnValues(struct dataObject *);
   LOCALE VOID                           PrintDataObject(char *,struct dataObject *);
   LOCALE VOID                           SetMultifieldErrorValue(struct dataObject *);
   LOCALE VOID                           ValueInstall(struct dataObject *);
   LOCALE VOID                           ValueDeinstall(struct dataObject *);
   LOCALE VOID                           PropagateReturnValue(struct dataObject *);
#if DEFFUNCTION_CONSTRUCT || DEFGENERIC_CONSTRUCT
   LOCALE int                            CLIPSFunctionCall(char *,char *,DATA_OBJECT *);
#endif
   LOCALE VOID                           CopyDataObject(DATA_OBJECT *,DATA_OBJECT *);
   LOCALE VOID                           AtomInstall(int,VOID *);
   LOCALE VOID                           AtomDeinstall(int,VOID *);
   LOCALE struct expr                   *ConvertValueToExpression(DATA_OBJECT *);
   LOCALE unsigned int                   GetAtomicHashValue(int,VOID *,int);
   LOCALE VOID                           InstallPrimitive(struct entityRecord *,int);
#else
   LOCALE int                            EvaluateExpression();
   LOCALE VOID                           SetEvaluationError();
   LOCALE int                            GetEvaluationError();
   LOCALE VOID                           SetHaltExecution();
   LOCALE int                            GetHaltExecution();
   LOCALE VOID                           ReturnValues();
   LOCALE VOID                           PrintDataObject();
   LOCALE VOID                           SetMultifieldErrorValue();
   LOCALE VOID                           ValueInstall();
   LOCALE VOID                           ValueDeinstall();
   LOCALE VOID                           PropagateReturnValue();
#if DEFFUNCTION_CONSTRUCT || DEFGENERIC_CONSTRUCT
   LOCALE int                            CLIPSFunctionCall();
#endif
   LOCALE VOID                           CopyDataObject();
   LOCALE VOID                           AtomInstall();
   LOCALE VOID                           AtomDeinstall();
   LOCALE struct expr                   *ConvertValueToExpression();
   LOCALE unsigned int                   GetAtomicHashValue();
   LOCALE VOID                           InstallPrimitive();
#endif

#ifndef _EVALUATN_SOURCE_
   extern struct expr            *CurrentExpression;
   extern int                     EvaluationError;
   extern int                     HaltExecution;
   extern int                     CurrentEvaluationDepth;
   extern struct entityRecord    *PrimitivesArray[];
#endif

#endif






