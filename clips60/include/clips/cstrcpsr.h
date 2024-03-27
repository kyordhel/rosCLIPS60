   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00 05/12/93             */
   /*                                                     */
   /*              CONSTRUCT PARSER MODULE                */
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
 
#ifndef _H_cstrcpsr

#define _H_cstrcpsr

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_scanner
#include "scanner.h"
#endif
#ifndef _H_constrct
#include "constrct.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CSTRCPSR_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif
   
#if ANSI_COMPILER
   LOCALE int                            Load(char *);
   LOCALE int                            LoadConstructsFromLogicalName(char *); 
   LOCALE int                            ParseConstruct(char *,char *);
   LOCALE VOID                           RemoveConstructFromModule(struct constructHeader *);
   LOCALE struct symbolHashNode         *GetConstructNameAndComment(char *,
                                         struct token *,char *,void *(*)(char *),
                                         int (*)(void *),char *,int,int,int);
   LOCALE VOID                           ImportExportConflictMessage(char *,char *,char *,char *);
#else
   LOCALE int                            Load();
   LOCALE int                            LoadConstructsFromLogicalName(); 
   LOCALE int                            ParseConstruct();
   LOCALE VOID                           RemoveConstructFromModule();
   LOCALE struct symbolHashNode         *GetConstructNameAndComment();
   LOCALE VOID                           ImportExportConflictMessage();
#endif 

#endif







