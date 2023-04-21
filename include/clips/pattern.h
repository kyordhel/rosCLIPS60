   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                PATTERN HEADER FILE                  */
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

#ifndef _H_pattern

#define _H_pattern

#ifndef _CLIPS_STDIO_
#include <stdio.h>
#define _CLIPS_STDIO_
#endif

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

struct patternEntityRecord
  {
   struct entityRecord base;
#if ANSI_COMPILER
   VOID (*decrementBasisCount)(VOID *);
   VOID (*incrementBasisCount)(VOID *);
   VOID (*matchFunction)(VOID *);
#else
   VOID (*decrementBasisCount)();
   VOID (*incrementBasisCount)();
   VOID (*matchFunction)();
#endif
  };

typedef struct patternEntityRecord PTRN_ENTITY_RECORD;
typedef struct patternEntityRecord *PTRN_ENTITY_RECORD_PTR; 
  
struct patternEntity
  {
   struct patternEntityRecord *theInfo;
#if LOGICAL_DEPENDENCIES
   VOID *dependents;
#endif
   unsigned busyCount;
   long int timeTag;
  };
  
typedef struct patternEntity PATTERN_ENTITY;
typedef struct patternEntity * PATTERN_ENTITY_PTR;

struct patternParser;

#ifndef _H_symbol
#include "symbol.h"
#endif
#ifndef _H_scanner
#include "scanner.h"
#endif
#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_match
#include "match.h"
#endif
#ifndef _H_reorder
#include "reorder.h"
#endif
#ifndef _H_constrnt
#include "constrnt.h"
#endif

#define MAXIMUM_NUMBER_OF_PATTERNS 128

struct patternParser
  {
   char *name; 
   struct patternEntityRecord *entityType;
   int positionInArray;
#if ANSI_COMPILER
   int (*recognizeFunction)(SYMBOL_HN *);
   struct lhsParseNode *(*parseFunction)(char *,struct token *);
   int (*postAnalysisFunction)(struct lhsParseNode *);
   struct patternNodeHeader *(*addPatternFunction)(struct lhsParseNode *);
   VOID (*removePatternFunction)(struct patternNodeHeader *);
   struct expr *(*genJNConstantFunction)(struct lhsParseNode *);
   VOID (*replaceGetJNValueFunction)(struct expr *,struct lhsParseNode *);
   struct expr *(*genGetJNValueFunction)(struct lhsParseNode *);
   struct expr *(*genCompareJNValuesFunction)(struct lhsParseNode *,struct lhsParseNode *);
   struct expr *(*genPNConstantFunction)(struct lhsParseNode *);
   VOID (*replaceGetPNValueFunction)(struct expr *,struct lhsParseNode *);
   struct expr *(*genGetPNValueFunction)(struct lhsParseNode *);
   struct expr *(*genComparePNValuesFunction)(struct lhsParseNode *,struct lhsParseNode *);
   VOID (*returnUserDataFunction)(VOID *);
   VOID *(*copyUserDataFunction)(VOID *);
   VOID (*markIRPatternFunction)(struct patternNodeHeader *,int);
   VOID (*incrementalResetFunction)(VOID);
   struct lhsParseNode *(*initialPatternFunction)(VOID);
   VOID (*codeReferenceFunction)(VOID *,FILE *,int,int);
#else
   int (*recognizeFunction)();
   struct lhsParseNode *(*parseFunction)();
   int (*postAnalysisFunction)();
   struct patternNodeHeader *(*addPatternFunction)();
   VOID (*removePatternFunction)();
   struct expr *(*genJNConstantFunction)();
   VOID (*replaceGetJNValueFunction)();
   struct expr *(*genGetJNValueFunction)();
   struct expr *(*genCompareJNValuesFunction)();
   struct expr *(*genPNConstantFunction)();
   VOID (*replaceGetPNValueFunction)();
   struct expr *(*genGetPNValueFunction)();
   struct expr *(*genComparePNValuesFunction)();
   VOID (*returnUserDataFunction)();
   VOID *(*copyUserDataFunction)();
   VOID (*markIRPatternFunction)();
   VOID (*incrementalResetFunction)();
   struct lhsParseNode *(*initialPatternFunction)();
   VOID (*codeReferenceFunction)();
#endif
   int priority;
   struct patternParser *next;
  };
  



#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _PATTERN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE int                            AddPatternParser(char *,int,
                                                          struct patternEntityRecord *,
                                                          int (*)(SYMBOL_HN *),
                                                          struct lhsParseNode *(*)(char *,struct token *),
                                                          int (*)(struct lhsParseNode *),
                                                          struct patternNodeHeader *(*)(struct lhsParseNode *),
                                                          VOID (*)(struct patternNodeHeader *),
                                                          struct expr *(*)(struct lhsParseNode *),
                                                          VOID (*)(struct expr *,struct lhsParseNode *),
                                                          struct expr *(*)(struct lhsParseNode *),
                                                          struct expr *(*)(struct lhsParseNode *,struct lhsParseNode *),
                                                          struct expr *(*)(struct lhsParseNode *),
                                                          VOID (*)(struct expr *,struct lhsParseNode *),
                                                          struct expr *(*)(struct lhsParseNode *),
                                                          struct expr *(*)(struct lhsParseNode *,struct lhsParseNode *),
                                                          VOID (*)(VOID *),
                                                          VOID *(*)(VOID *),
                                                          VOID (*)(struct patternNodeHeader *,int),
                                                          VOID (*)(VOID),
                                                          struct lhsParseNode *(*)(VOID),
                                                          VOID (*)(VOID *,FILE *,int,int));
   LOCALE struct patternParser          *FindPatternParser(char *);
   LOCALE VOID                           DetachPattern(int,struct patternNodeHeader *);
   LOCALE VOID                           GetNextPatternEntity(struct patternParser **,
                                                              struct patternEntity **);
   LOCALE struct patternParser          *GetPatternParser(int);
   LOCALE struct lhsParseNode           *RestrictionParse(char *,struct token *,int,
                                                       struct symbolHashNode *,int,
                                                       struct constraintRecord *,int);
   LOCALE int                            PostPatternAnalysis(struct lhsParseNode *);
   LOCALE VOID                           PatternNodeHeaderToCode(FILE *,struct patternNodeHeader *,int,int);
   LOCALE VOID                           AddReservedPatternSymbol(char *,char *);
   LOCALE BOOLEAN                        ReservedPatternSymbol(char *,char *);
   LOCALE VOID                           ReservedPatternSymbolErrorMsg(char *,char *);
#else
   LOCALE int                            AddPatternParser();
   LOCALE struct patternParser          *FindPatternParser();
   LOCALE VOID                           DetachPattern();
   LOCALE VOID                           GetNextPatternEntity();
   LOCALE struct patternParser          *GetPatternParser();
   LOCALE struct lhsParseNode           *RestrictionParse();
   LOCALE int                            PostPatternAnalysis();
   LOCALE VOID                           PatternNodeHeaderToCode();
   LOCALE VOID                           AddReservedPatternSymbol();
   LOCALE BOOLEAN                        ReservedPatternSymbol();
   LOCALE VOID                           ReservedPatternSymbolErrorMsg();
#endif

#ifndef _PATTERN_SOURCE_
   extern struct patternParser          *ListOfPatternParsers;
#endif

#endif









