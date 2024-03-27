   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*          DEFTEMPLATE FUNCTION HEADER FILE           */
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

#ifndef _H_tmpltfun

#define _H_tmpltfun

#ifndef _H_symbol
#include "symbol.h"
#endif
#ifndef _H_scanner
#include "scanner.h"
#endif
#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_factmngr
#include "factmngr.h"
#endif
#ifndef _H_tmpltdef
#include "tmpltdef.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif
  
#ifdef _TMPLTFUN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE BOOLEAN                        UpdateModifyDuplicate(struct expr *,char *,VOID *);
   LOCALE struct expr                   *ModifyParse(struct expr *,char *);
   LOCALE struct expr                   *DuplicateParse(struct expr *,char *);
   LOCALE VOID                           CheckTemplateFact(struct fact *);
   LOCALE VOID                           MultiIntoSingleFieldSlotError(struct templateSlot *,struct deftemplate *);
   LOCALE BOOLEAN                        CheckRHSSlotTypes(struct expr *,struct templateSlot *,char *);
#else
   LOCALE BOOLEAN                        UpdateModifyDuplicate();
   LOCALE struct expr                   *ModifyParse();
   LOCALE struct expr                   *DuplicateParse();
   LOCALE VOID                           CheckTemplateFact();
   LOCALE VOID                           MultiIntoSingleFieldSlotError();
   LOCALE BOOLEAN                        CheckRHSSlotTypes();
#endif

#endif









