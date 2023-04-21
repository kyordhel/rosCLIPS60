   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*               EXPRESSION HEADER FILE                */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_expressn

#define _H_expressn

struct expr;
struct exprHashNode;

#ifndef _H_exprnops
#include "exprnops.h"
#endif

/******************************/
/* Expression Data Structures */
/******************************/

struct expr 
   {
    short type;
    VOID *value;
    struct expr *argList;
    struct expr *nextArg;
   };
   
#define arg_list argList
#define next_arg nextArg

typedef struct expr EXPRESSION;

typedef struct exprHashNode
  {
   unsigned hashval;
   unsigned count;
   struct expr *exp;
   struct exprHashNode *nxt;
   long bsaveID;
  } EXPRESSION_HN;

#define EXPRESSION_HASH_SIZE 503

/*************************/
/* Type and Value Macros */
/*************************/

#define GetType(target)         ((target).type) 
#define GetpType(target)        ((target)->type) 
#define SetType(target,val)     ((target).type = (val)) 
#define SetpType(target,val)    ((target)->type = (val)) 
#define GetValue(target)        ((target).value) 
#define GetpValue(target)       ((target)->value) 
#define SetValue(target,val)    ((target).value = (VOID *) (val)) 
#define SetpValue(target,val)   ((target)->value = (VOID *) (val)) 

/********************/
/* Global Functions */
/********************/
   
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _EXPRESSN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           ReturnExpression(struct expr *);
   LOCALE VOID                           ExpressionInstall(struct expr *);
   LOCALE VOID                           ExpressionDeinstall(struct expr *);
   LOCALE struct expr                   *CopyExpression(struct expr *);
   LOCALE struct expr                   *PackExpression(struct expr *);
   LOCALE int                            ListToPacked(struct expr *,struct expr *,int);
   LOCALE VOID                           ReturnPackedExpression(struct expr *);
   LOCALE VOID                           InitExpressionData(void);
   LOCALE VOID                           InitExpressionPointers(void);
   LOCALE BOOLEAN                        ConstantType(int);
#if (! BLOAD_ONLY) && (! RUN_TIME)
   LOCALE EXPRESSION                    *AddHashedExpression(EXPRESSION *);
#endif
#if (! RUN_TIME)
   LOCALE VOID                           RemoveHashedExpression(EXPRESSION *);
#endif
#if BLOAD_AND_BSAVE || BLOAD_ONLY || BLOAD || CONSTRUCT_COMPILER
   LOCALE long                           HashedExpressionIndex(EXPRESSION *);
#endif
#else
   LOCALE VOID                           ReturnExpression();
   LOCALE VOID                           ExpressionInstall();
   LOCALE VOID                           ExpressionDeinstall();
   LOCALE struct expr                   *CopyExpression();
   LOCALE struct expr                   *PackExpression();
   LOCALE int                            ListToPacked();
   LOCALE VOID                           ReturnPackedExpression();
   LOCALE VOID                           InitExpressionData();
   LOCALE VOID                           InitExpressionPointers();
   LOCALE BOOLEAN                        ConstantType();
#if (! BLOAD_ONLY) && (! RUN_TIME)
   LOCALE EXPRESSION                    *AddHashedExpression();
#endif
#if (! RUN_TIME)
   LOCALE VOID                           RemoveHashedExpression();
#endif
#if BLOAD_AND_BSAVE || BLOAD_ONLY || BLOAD || CONSTRUCT_COMPILER
   LOCALE long                           HashedExpressionIndex();
#endif
#endif
   
/********************/
/* Global Variables */
/********************/

#ifndef _EXPRESSN_SOURCE_
   extern VOID                          *PTR_AND; 
   extern VOID                          *PTR_OR; 
   extern VOID                          *PTR_EQ;
   extern VOID                          *PTR_NEQ; 
   extern VOID                          *PTR_NOT;
   extern EXPRESSION_HN                **ExpressionHashTable;
#endif

#endif




