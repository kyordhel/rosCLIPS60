   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 DEFRULE HEADER FILE                 */
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

#ifndef _H_ruledef
#define _H_ruledef

#define GetDisjunctIndex(r) ((struct constructHeader *) r)->bsaveID

struct defrule;
struct defruleModule;

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
#ifndef _H_cstrccom
#include "cstrccom.h"
#endif
#ifndef _H_agenda
#include "agenda.h"
#endif
#ifndef _H_network
#include "network.h"
#endif

    
struct defrule
  {
   struct constructHeader header;
   int salience;
   int localVarCnt;
   unsigned int complexity      : 11;
   unsigned int afterBreakpoint :  1;
   unsigned int watchActivation :  1;
   unsigned int watchFiring     :  1;
   unsigned int autoFocus       :  1;
   unsigned int executing       :  1;
#if DYNAMIC_SALIENCE
   struct expr *dynamicSalience;
#endif
   struct expr *actions;
#if LOGICAL_DEPENDENCIES
   struct joinNode *logicalJoin;
#endif
   struct joinNode *lastJoin;
   struct defrule *disjunct;
  };
  
struct defruleModule
  {
   struct defmoduleItemHeader header;
   struct activation *agenda;
  };

#define GetDefruleName(x) GetConstructNameString(x)
#define GetDefrulePPForm(x) GetConstructPPForm(x)
#define DefruleModule(x) GetConstructModuleName((struct constructHeader *) x)

#define GetPreviousJoin(theJoin) \
   (((theJoin)->joinFromTheRight) ? \
    ((struct joinNode *) (theJoin)->rightSideEntryStructure) : \
    ((theJoin)->lastLevel))
#define GetPatternForJoin(theJoin) \
   (((theJoin)->joinFromTheRight) ? \
    NULL : \
    ((theJoin)->rightSideEntryStructure))

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _RULEDEF_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           InitializeDefrules(void);
   LOCALE VOID                          *FindDefrule(char *);
   LOCALE VOID                          *GetNextDefrule(VOID *);
   LOCALE struct defruleModule          *GetDefruleModuleItem(struct defmodule *);
   LOCALE BOOLEAN                        IsDefruleDeletable(VOID *);
#else
   LOCALE VOID                           InitializeDefrules();
   LOCALE VOID                          *FindDefrule();
   LOCALE VOID                          *GetNextDefrule();
   LOCALE struct defruleModule          *GetDefruleModuleItem();
   LOCALE BOOLEAN                        IsDefruleDeletable();
#endif

#ifndef _RULEDEF_SOURCE_
   extern struct construct              *DefruleConstruct;
   extern int                            DefruleModuleIndex;
   extern long                           CurrentEntityTimeTag;
#endif

#endif


