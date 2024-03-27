   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                DEFFACTS HEADER FILE                 */
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

#ifndef _H_dffctdef
#define _H_dffctdef

#ifndef _H_symbol
#include "symbol.h"
#endif
#ifndef _H_expressn
#include "expressn.h"
#endif
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_constrct
#include "constrct.h"
#endif
#ifndef _H_moduldef
#include "moduldef.h"
#endif
#ifndef _H_cstrccom
#include "cstrccom.h"
#endif

struct deffacts
  {
   struct constructHeader header;
   struct expr *assertList;
  };
  
struct deffactsModule
  {
   struct defmoduleItemHeader header;
  };

#define GetDeffactsName(x) GetConstructNameString((struct constructHeader *) x)
#define GetDeffactsPPForm(x) GetConstructPPForm((struct constructHeader *) x)
#define DeffactsModule(x) GetConstructModuleName((struct constructHeader *) x)

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _DFFCTDEF_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           InitializeDeffacts(void);
   LOCALE VOID                          *FindDeffacts(char *);
   LOCALE VOID                          *GetNextDeffacts(VOID *);
   LOCALE VOID                           CreateInitialFactDeffacts(void);
   LOCALE BOOLEAN                        IsDeffactsDeletable(VOID *);
   LOCALE struct deffactsModule         *GetDeffactsModuleItem(struct defmodule *);
#else
   LOCALE VOID                           InitializeDeffacts();
   LOCALE VOID                          *FindDeffacts();
   LOCALE VOID                          *GetNextDeffacts();
   LOCALE VOID                           CreateInitialFactDeffacts();
   LOCALE BOOLEAN                        IsDeffactsDeletable();
   LOCALE struct deffactsModule         *GetDeffactsModuleItem();
#endif

#ifndef _DFFCTDEF_SOURCE_
   extern struct construct              *DeffactsConstruct;
   extern int                            DeffactsModuleIndex;
#endif


#endif


