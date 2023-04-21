   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*               DEFTEMPLATE HEADER FILE               */
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

#ifndef _H_tmpltcom

#define _H_tmpltcom

#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_factmngr
#include "factmngr.h"
#endif
#ifndef _H_constrnt
#include "constrnt.h"
#endif
#ifndef _H_symbol
#include "symbol.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _TMPLTCOM_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           DeftemplateCommands(void);
   LOCALE VOID                           ModifyCommand(DATA_OBJECT_PTR);
   LOCALE VOID                           DuplicateCommand(DATA_OBJECT_PTR);
#else
   LOCALE VOID                           DeftemplateCommands();
   LOCALE VOID                           ModifyCommand();
   LOCALE VOID                           DuplicateCommand();
#endif

#endif





