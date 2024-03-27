   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                 BSAVE HEADER FILE                   */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_bsave
#define _H_bsave

struct BinaryItem;

#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#ifndef _H_expressn
#include "expressn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _BSAVE_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

struct BinaryItem
  {
   char *name; 
   VOID (*findFunction)(VOID_ARG);
   VOID (*bloadStorageFunction)(VOID_ARG);
   VOID (*bloadFunction)(VOID_ARG);
   VOID (*clearFunction)(VOID_ARG);
#if ANSI_COMPILER
   VOID (*expressionFunction)(FILE *);
   VOID (*bsaveStorageFunction)(FILE *);
   VOID (*bsaveFunction)(FILE *);
#else
   VOID (*expressionFunction)();
   VOID (*bsaveStorageFunction)();
   VOID (*bsaveFunction)();
#endif
   int priority;
   struct BinaryItem *next;
  };
  
  
typedef struct bsave_expr 
  {
   int type;
   long value,arg_list,next_arg;
  } BSAVE_EXPRESSION;
  
#define CONSTRUCT_HEADER_SIZE 20
                                   
#if ANSI_COMPILER 
   LOCALE int                     BsaveCommand(void);
#if BLOAD_AND_BSAVE
   LOCALE BOOLEAN                 Bsave(char *);
   LOCALE VOID                    MarkNeededItems(struct expr *);
   LOCALE VOID                    SaveBloadCount(long);
   LOCALE VOID                    RestoreBloadCount(long *);
#endif
#if BLOAD_AND_BSAVE || BSAVE_INSTANCES
   LOCALE VOID                    GenWrite(VOID *,unsigned long,FILE *);
#endif
   LOCALE BOOLEAN                 AddBinaryItem(char *,int ,VOID (*)(void),
                                                VOID (*)(FILE *),VOID (*)(FILE *),
                                                VOID (*)(FILE *),VOID (*)(void),
                                                VOID (*)(void),VOID (*)(void));
#else
   LOCALE int                     BsaveCommand();
#if BLOAD_AND_BSAVE
   LOCALE BOOLEAN                 Bsave();
   LOCALE VOID                    MarkNeededItems();
   LOCALE VOID                    SaveBloadCount();
   LOCALE VOID                    RestoreBloadCount();
#endif
#if BLOAD_AND_BSAVE || BSAVE_INSTANCES
   LOCALE VOID                    GenWrite();
#endif
   LOCALE BOOLEAN                 AddBinaryItem();
#endif

#ifndef _BSAVE_SOURCE_
   extern struct BinaryItem      *ListOfBinaryItems;
#endif

#endif







