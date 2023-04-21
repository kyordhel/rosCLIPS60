   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            EXTERNAL FUNCTIONS HEADER FILE           */
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

#ifndef _H_extnfunc

#define _H_extnfunc

#ifndef _H_symbol
#include "symbol.h"
#endif
#ifndef _H_expressn
#include "expressn.h"
#endif

struct FunctionDefinition
  {
   struct symbolHashNode *callFunctionName;
   char *actualFunctionName; 
   char returnValueType;  
   int (*functionPointer)(VOID_ARG);
#if ANSI_COMPILER
   struct expr *(*parser)(struct expr *,char *);
#else
   struct expr *(*parser)();
#endif
   char *restrictions;
   short int overloadable; 
   short int sequenceuseok; 
   short int bsaveIndex;
   struct FunctionDefinition *next;
  };

#define ValueFunctionType(target) (((struct FunctionDefinition *) target)->returnValueType)
#define ExpressionFunctionType(target) (((struct FunctionDefinition *) ((target)->value))->returnValueType)
#define ExpressionFunctionPointer(target) (((struct FunctionDefinition *) ((target)->value))->functionPointer)
#define ExpressionFunctionCallName(target) (((struct FunctionDefinition *) ((target)->value))->callFunctionName)
#define ExpressionFunctionRealName(target) (((struct FunctionDefinition *) ((target)->value))->actualFunctionName)

#define PTIF (int (*)(VOID_ARG))

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _EXTNFUNC_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#ifdef LOCALE
struct FunctionHash
  {
   struct FunctionDefinition *fdPtr;
   struct FunctionHash *next;
  };

#define SIZE_FUNCTION_HASH 51
#endif

#if ANSI_COMPILER
   LOCALE int                            DefineFunction(char *,int,int (*)(void),char *);
   LOCALE int                            DefineFunction2(char *,int,int (*)(void),char *,char *);
   LOCALE int                            AddFunctionParser(char *,struct expr *(*)(struct expr *,char *));
   LOCALE int                            RemoveFunctionParser(char *);
   LOCALE int                            FuncSeqOvlFlags(char *,int,int);
   LOCALE struct FunctionDefinition     *GetFunctionList(void);
   LOCALE VOID                           InstallFunctionList(struct FunctionDefinition *);
   LOCALE struct FunctionDefinition     *FindFunction(char *);
   LOCALE int                            GetNthRestriction(struct FunctionDefinition *,int);
   LOCALE char                          *GetArgumentTypeName(int);
#else
   LOCALE int                            DefineFunction();
   LOCALE int                            DefineFunction2();
   LOCALE int                            AddFunctionParser();
   LOCALE int                            RemoveFunctionParser();
   LOCALE int                            FuncSeqOvlFlags();
   LOCALE struct FunctionDefinition     *GetFunctionList();
   LOCALE VOID                           InstallFunctionList();
   LOCALE struct FunctionDefinition     *FindFunction();
   LOCALE int                            GetNthRestriction();
   LOCALE char                          *GetArgumentTypeName();
#endif

#endif




