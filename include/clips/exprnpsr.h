   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            EXPRESSION PARSER HEADER FILE            */
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

#ifndef _H_exprnpsr

#define _H_exprnpsr

#ifndef _H_extnfunc
#include "extnfunc.h"
#endif
#ifndef _H_scanner
#include "scanner.h"
#endif

#if (! RUN_TIME)

typedef struct saved_contexts
  {
   int rtn;
   int brk;
   struct saved_contexts *nxt;
  } SAVED_CONTEXTS;
  
#endif
  
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _EXPRNPSR_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE struct expr                   *Function0Parse(char *);
   LOCALE struct expr                   *Function1Parse(char *);
   LOCALE struct expr                   *Function2Parse(char *,char *);
   LOCALE VOID                           PushRtnBrkContexts(void);
   LOCALE VOID                           PopRtnBrkContexts(void);
   LOCALE BOOLEAN                        ReplaceSequenceExpansionOps(struct expr *,struct expr *,
                                                                     VOID *,VOID *);
   LOCALE struct expr                   *CollectArguments(struct expr *,char *);
   LOCALE struct expr                   *ArgumentParse(char *,int *);
   LOCALE struct expr                   *ParseAtomOrExpression(char *,struct token *);
   LOCALE EXPRESSION                    *ParseConstantArguments(char *,int *);
   LOCALE BOOLEAN                        SetSequenceOperatorRecognition(int);
   LOCALE BOOLEAN                        GetSequenceOperatorRecognition(void);
   LOCALE struct expr                   *GroupActions(char *,struct token *,int,char *);
#else

   LOCALE struct expr                   *Function0Parse();
   LOCALE struct expr                   *Function1Parse();
   LOCALE struct expr                   *Function2Parse();
   LOCALE VOID                           PushRtnBrkContexts();
   LOCALE VOID                           PopRtnBrkContexts();
   LOCALE BOOLEAN                        ReplaceSequenceExpansionOps();
   LOCALE struct expr                   *CollectArguments();
   LOCALE struct expr                   *ArgumentParse();
   LOCALE struct expr                   *ParseAtomOrExpression();
   LOCALE EXPRESSION                    *ParseConstantArguments();
   LOCALE BOOLEAN                        SetSequenceOperatorRecognition();
   LOCALE BOOLEAN                        GetSequenceOperatorRecognition();
   LOCALE struct expr                   *GroupActions();
#endif

#ifndef _EXPRNPSR_SOURCE_
#if (! RUN_TIME)
extern SAVED_CONTEXTS *svContexts;
extern int ReturnContext,BreakContext;
#endif
extern int SequenceOpMode;
#endif

#endif




