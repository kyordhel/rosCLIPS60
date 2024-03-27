   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                                                     */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_defins
#define _H_defins

#if DEFINSTANCES_CONSTRUCT

#define GetDefinstancesName(x) GetConstructNameString((struct constructHeader *) x)
#define GetDefinstancesPPForm(x) GetConstructPPForm((struct constructHeader *) x)

#define GetDefinstancesNamePointer(x) GetConstructNamePointer((struct constructHeader *) x)
#define SetDefinstancesPPForm(d,ppf) SetConstructPPForm((struct constructHeader *) d,ppf)

#define GetDefinstancesModuleName(x) GetConstructModuleName((struct constructHeader *) x)

struct definstances;

#ifndef _H_constrct
#include "constrct.h"
#endif
#ifndef _H_cstrccom
#include "cstrccom.h"
#endif
#ifndef _H_moduldef
#include "moduldef.h"
#endif
#ifndef _H_object
#include "object.h"
#endif

typedef struct definstancesModule
  {
   struct defmoduleItemHeader header;
  } DEFINSTANCES_MODULE;
     
typedef struct definstances
  {
   struct constructHeader header;
   unsigned busy;
   EXPRESSION *mkinstance;
  } DEFINSTANCES;

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _DEFINS_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE VOID SetupDefinstances(void);
LOCALE VOID *GetNextDefinstances(VOID *);
LOCALE VOID *FindDefinstances(char *);
LOCALE int IsDefinstancesDeletable(VOID *);
LOCALE VOID UndefinstancesCommand(void);
LOCALE SYMBOL_HN *GetDefinstancesModuleCommand(void);
LOCALE BOOLEAN Undefinstances(VOID *);

#if DEBUGGING_FUNCTIONS
LOCALE VOID PPDefinstancesCommand(void);
LOCALE VOID ListDefinstancesCommand(void);
LOCALE VOID ListDefinstances(char *,struct defmodule *);
#endif

LOCALE VOID GetDefinstancesListFunction(DATA_OBJECT *);
LOCALE VOID GetDefinstancesList(DATA_OBJECT *,struct defmodule *);

#else

LOCALE VOID SetupDefinstances();
LOCALE VOID *GetNextDefinstances();
LOCALE VOID *FindDefinstances();
LOCALE int IsDefinstancesDeletable();
LOCALE SYMBOL_HN *GetDefinstancesModuleCommand();
LOCALE VOID UndefinstancesCommand();
LOCALE BOOLEAN Undefinstances();

#if DEBUGGING_FUNCTIONS
LOCALE VOID PPDefinstancesCommand();
LOCALE VOID ListDefinstancesCommand();
LOCALE VOID ListDefinstances();
#endif

LOCALE VOID GetDefinstancesListFunction();
LOCALE VOID GetDefinstancesList();

#endif

#ifndef _DEFINS_SOURCE_
extern int DefinstancesModuleIndex;
#endif

#endif

#endif





