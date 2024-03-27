   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                DEFGLOBAL HEADER FILE                */
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

#ifndef _H_globldef
#define _H_globldef

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

struct defglobal
  {
   struct constructHeader header;
   unsigned int watch   : 1;
   unsigned int inScope : 1;
   long busyCount;
   DATA_OBJECT current;
   struct expr *initial;
  };

struct defglobalModule
  {
   struct defmoduleItemHeader header;
  };

#define GetDefglobalName(x) GetConstructNameString(x)
#define GetDefglobalPPForm(x) GetConstructPPForm(x)
#define DefglobalModule(x) GetConstructModuleName((struct constructHeader *) x)

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _GLOBLDEF_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           InitializeDefglobals(void);
   LOCALE VOID                          *FindDefglobal(char *);
   LOCALE VOID                          *GetNextDefglobal(VOID *);
   LOCALE VOID                           CreateInitialFactDefglobal(void);
   LOCALE BOOLEAN                        IsDefglobalDeletable(VOID *);
   LOCALE struct defglobalModule        *GetDefglobalModuleItem(struct defmodule *);
   LOCALE VOID                           QSetDefglobalValue(struct defglobal *,DATA_OBJECT_PTR);
   LOCALE struct defglobal              *QFindDefglobal(struct symbolHashNode *);
   LOCALE VOID                           GetDefglobalValueForm(char *,int,VOID *);
   LOCALE int                            GetGlobalsChanged(void);
   LOCALE VOID                           SetGlobalsChanged(int);
   LOCALE BOOLEAN                        GetDefglobalValue(char *,DATA_OBJECT_PTR);
   LOCALE BOOLEAN                        SetDefglobalValue(char *,DATA_OBJECT_PTR);
   LOCALE VOID                           UpdateDefglobalScope(void);
   LOCALE VOID                          *GetNextDefglobalInScope(void *);
#else
   LOCALE VOID                           InitializeDefglobal();
   LOCALE VOID                          *FindDefglobal();
   LOCALE VOID                          *GetNextDefglobal();
   LOCALE VOID                           CreateInitialFactDefglobal();
   LOCALE BOOLEAN                        IsDefglobalDeletable();
   LOCALE struct defglobalModule        *GetDefglobalModuleItem();
   LOCALE VOID                           QSetDefglobalValue();
   LOCALE struct defglobal              *QFindDefglobal();
   LOCALE VOID                           GetDefglobalValueForm();
   LOCALE int                            GetGlobalsChanged();
   LOCALE VOID                           SetGlobalsChanged();
   LOCALE BOOLEAN                        GetDefglobalValue();
   LOCALE BOOLEAN                        SetDefglobalValue();
   LOCALE VOID                           UpdateDefglobalScope();
   LOCALE VOID                          *GetNextDefglobalInScope();
#endif

#ifndef _GLOBLDEF_SOURCE_
   extern struct construct              *DefglobalConstruct;
   extern int                            DefglobalModuleIndex;
   extern int                            ChangeToGlobals;
#endif


#endif


