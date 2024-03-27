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
/*      Brian L. Donnell                                     */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_tmpltdef
#define _H_tmpltdef

struct deftemplate;
struct templateSlot;
struct deftemplateModule;

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
#ifndef _H_constrnt
#include "constrnt.h"
#endif
#include "factbld.h"
#ifndef _H_factmngr
#include "factmngr.h"
#endif
#ifndef _H_cstrccom
#include "cstrccom.h"
#endif
  
struct deftemplate
  {
   struct constructHeader header;
   struct templateSlot *slotList;  
   unsigned int implied       : 1;
   unsigned int watch         : 1;
   unsigned int inScope       : 1;
   unsigned int numberOfSlots : 13;  
   long busyCount;
   struct factPatternNode *patternNetwork;
  };
  
struct templateSlot
  {
   struct symbolHashNode *slotName;
   unsigned int multislot : 1;
   unsigned int noDefault : 1;
   unsigned int defaultPresent : 1;
   unsigned int defaultDynamic : 1;
   CONSTRAINT_RECORD *constraints;
   struct expr *defaultList;
   struct templateSlot *next;
  };

struct deftemplateModule
  {
   struct defmoduleItemHeader header;
  };

#define GetDeftemplateName(x) GetConstructNameString(x)
#define GetDeftemplatePPForm(x) GetConstructPPForm(x)
#define DeftemplateModule(x) GetConstructModuleName((struct constructHeader *) x)

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _TMPLTDEF_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           InitializeDeftemplates(void);
   LOCALE VOID                          *FindDeftemplate(char *);
   LOCALE VOID                          *GetNextDeftemplate(VOID *);
   LOCALE BOOLEAN                        IsDeftemplateDeletable(VOID *);
   LOCALE struct deftemplateModule      *GetDeftemplateModuleItem(struct defmodule *);
   LOCALE VOID                           PrintTemplateFact(char *,struct fact *);
   LOCALE VOID                           UpdateDeftemplateScope(void);
   LOCALE struct templateSlot           *FindSlot(struct deftemplate *,struct symbolHashNode *,int *);
   LOCALE VOID                           InvalidDeftemplateSlotMessage(char *,char *);
   LOCALE VOID                           SingleFieldSlotCardinalityError(char *);
#else
   LOCALE VOID                           InitializeDeftemplates();
   LOCALE VOID                          *FindDeftemplate();
   LOCALE VOID                          *GetNextDeftemplate();
   LOCALE BOOLEAN                        IsDeftemplateDeletable();
   LOCALE struct deftemplateModule      *GetDeftemplateModuleItem();
   LOCALE VOID                           PrintTemplateFact();
   LOCALE VOID                           UpdateDeftemplateScope();
   LOCALE struct templateSlot           *FindSlot();
   LOCALE VOID                           InvalidDeftemplateSlotMessage();
   LOCALE VOID                           SingleFieldSlotCardinalityError();
#endif

#ifndef _TMPLTDEF_SOURCE_
   extern struct construct              *DeftemplateConstruct;
   extern int                            DeftemplateModuleIndex;
#endif


#endif


