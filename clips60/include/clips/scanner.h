   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 SCANNER HEADER FILE                 */
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

#ifndef _H_scanner
#define _H_scanner

struct token;

#ifndef _H_pprint
#include "pprint.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _SCANNER_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

struct token
  {
   int type;
   VOID *value;
   char *printForm;
  };

#define print_rep printForm

#if ANSI_COMPILER
   LOCALE VOID                           GetToken(char *,struct token *);
   LOCALE VOID                           CopyToken(struct token *,struct token *); 
#else
   LOCALE VOID                           GetToken();
   LOCALE VOID                           CopyToken(); 
#endif

#ifndef _SCANNER_SOURCE_
   extern int                            IgnoreCompletionErrors;
#endif
#endif




