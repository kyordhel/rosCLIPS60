   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*           CONSTRUCT COMPILER HEADER FILE            */
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

#ifndef _H_conscomp
#define _H_conscomp

#define ArbitraryPrefix(codeItem,i)    (codeItem)->arrayNames[(i)]

#define ModulePrefix(codeItem)         (codeItem)->arrayNames[0]
#define ConstructPrefix(codeItem)      (codeItem)->arrayNames[1]

#ifndef _H_extnfunc
#include "extnfunc.h"
#endif
#ifndef _H_symblcmp
#include "symblcmp.h"
#endif
#ifndef _H_moduldef
#include "moduldef.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

struct CodeGeneratorItem
  {
   char *name; 
   VOID (*beforeFunction)(VOID_ARG);
#if ANSI_COMPILER
   VOID (*initFunction)(FILE *,int,int);
   int (*generateFunction)(char *,int,FILE *,int,int);
#else
   VOID (*initFunction)();
   int (*generateFunction)();
#endif
   int priority;
   char **arrayNames;
   struct CodeGeneratorItem *next;
  };

struct CodeGeneratorFile
 {
  char *filePrefix;
  int id,version;
 };
   
#ifdef _CONSCOMP_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER  
   LOCALE VOID                      ConstructsToCCommandDefinition(void);
   LOCALE FILE                     *NewCFile(char *,int,int,int);
   LOCALE int                       ExpressionToCode(FILE *,struct expr *);
   LOCALE VOID                      PrintFunctionReference(FILE *,struct FunctionDefinition *);
   LOCALE struct CodeGeneratorItem *AddCodeGeneratorItem(char *,int,VOID (*)(void),
                                                         VOID (*)(FILE *,int,int),
                                                         int (*)(char *,int,FILE *,int,int),int);
   LOCALE FILE                     *CloseFileIfNeeded(FILE *,int *,int *,int,int *,struct CodeGeneratorFile *);
   LOCALE FILE                     *OpenFileIfNeeded(FILE *,char *,int,int,int *,int,FILE *,
                                                     char *,char *,int,struct CodeGeneratorFile *);
   LOCALE VOID                      MarkConstructBsaveIDs(int);
   LOCALE VOID                      ConstructHeaderToCode(FILE *,struct constructHeader *,int,int,
                                                         int,char *,char *);
   LOCALE VOID                      ConstructModuleToCode(FILE *,struct defmodule *,int,int,
                                                         int,char *);
   LOCALE VOID                      PrintHashedExpressionReference(FILE *,struct expr *,int,int);
#else
   LOCALE VOID                      ConstructsToCCommandDefinition();
   LOCALE FILE                     *NewCFile();
   LOCALE int                       ExpressionToCode();
   LOCALE VOID                      PrintFunctionReference();
   LOCALE struct CodeGeneratorItem *AddCodeGeneratorItem();
   LOCALE FILE                     *CloseFileIfNeeded();
   LOCALE FILE                     *OpenFileIfNeeded();
   LOCALE VOID                      MarkConstructBsaveIDs();
   LOCALE VOID                      ConstructHeaderToCode();
   LOCALE VOID                      ConstructModuleToCode();
   LOCALE VOID                      PrintHashedExpressionReference();
#endif 

#ifndef _SYMBLBIN_SOURCE_
   extern globle int                       ImageID;
   extern globle FILE                     *HeaderFP;
   extern globle int                       MaxIndices;
#endif
   
#endif




