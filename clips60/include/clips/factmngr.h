   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*              FACTS MANAGER HEADER FILE              */
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

#ifndef _H_factmngr

#define _H_factmngr

struct fact;

#ifndef _H_pattern
#include "pattern.h"
#endif
#include "multifld.h"
#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_tmpltdef
#include "tmpltdef.h"
#endif

struct fact
  {
   struct patternEntity factHeader;
   struct deftemplate *whichDeftemplate;
   VOID *list;
   long int factIndex;
   unsigned int depth : 15;
   unsigned int garbage : 1;
   struct fact *previousFact;  
   struct fact *nextFact;  
   struct multifield theProposition;
  };

#ifdef LOCALE
#undef LOCALE
#endif
#ifdef _FACTMNGR_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           PrintFactWithIdentifier(char *,struct fact *);
   LOCALE VOID                           PrintFact(char *,struct fact *);
   LOCALE VOID                           PrintFactInLongForm(char *,VOID *);
   LOCALE BOOLEAN                        Retract(VOID *);
   LOCALE VOID                           RemoveOldFacts(void);
   LOCALE VOID                          *Assert(VOID *);
   LOCALE VOID                           RemoveAllFacts(void);
   LOCALE struct fact                   *CreateFactBySize(int);
   LOCALE VOID                           FactInstall(struct fact *);
   LOCALE VOID                           FactDeinstall(struct fact *);
   LOCALE VOID                           SetFactID(long);
   LOCALE VOID                          *GetNextFact(VOID *);
   LOCALE VOID                          *GetNextFactInScope(VOID *);
   LOCALE VOID                           GetFactPPForm(char *,int,VOID *);
   LOCALE long int                       FactIndex(VOID *);
   LOCALE VOID                          *AssertString(char *);
   LOCALE int                            GetFactListChanged(void);
   LOCALE VOID                           SetFactListChanged(int);
   LOCALE long int                       GetNumberOfFacts(void);
   LOCALE VOID                           InitializeFacts(void);
   LOCALE struct fact                   *FindIndexedFact(long);
   LOCALE VOID                           IncrementFactCount(VOID *);
   LOCALE VOID                           DecrementFactCount(VOID *);
   LOCALE VOID                           PrintFactIdentifier(char *,VOID *);
   LOCALE VOID                           DecrementFactBasisCount(VOID *);
   LOCALE VOID                           IncrementFactBasisCount(VOID *);
   LOCALE VOID                           ReturnFact(struct fact *);   
   LOCALE VOID                           MatchFactFunction(VOID *);
   LOCALE struct fact                   *CreateFact(VOID *);
   LOCALE BOOLEAN                        PutFactSlot(VOID *,char *,DATA_OBJECT *);
   LOCALE BOOLEAN                        GetFactSlot(VOID *,char *,DATA_OBJECT *);
   LOCALE BOOLEAN                        AssignFactSlotDefaults(VOID *);
#else
   LOCALE VOID                           PrintFactWithIdentifier();
   LOCALE VOID                           PrintFact();
   LOCALE VOID                           PrintFactInLongForm();
   LOCALE BOOLEAN                        Retract();
   LOCALE VOID                           RemoveOldFacts();
   LOCALE VOID                          *Assert();
   LOCALE VOID                           RemoveAllFacts();
   LOCALE struct fact                   *CreateFactBySize();
   LOCALE VOID                           FactInstall();
   LOCALE VOID                           FactDeinstall();
   LOCALE VOID                           SetFactID();
   LOCALE VOID                          *GetNextFact();
   LOCALE VOID                          *GetNextFactInScope();
   LOCALE VOID                           GetFactPPForm();
   LOCALE long int                       FactIndex();
   LOCALE VOID                          *AssertString();
   LOCALE int                            GetFactlistChanged();
   LOCALE VOID                           SetFactlistChanged();
   LOCALE long int                       GetNumberOfFacts();
   LOCALE VOID                           InitializeFacts();
   LOCALE struct fact                   *FindIndexedFact();
   LOCALE VOID                           IncrementFactCount();
   LOCALE VOID                           DecrementFactCount();
   LOCALE VOID                           PrintFactIdentifier();
   LOCALE VOID                           DecrementFactBasisCount();
   LOCALE VOID                           IncrementFactBasisCount();
   LOCALE VOID                           ReturnFact();
   LOCALE VOID                           MatchFactFunction();
   LOCALE struct fact                   *CreateFact();
   LOCALE BOOLEAN                        PutFactSlot();
   LOCALE BOOLEAN                        GetFactSlot();
   LOCALE BOOLEAN                        AssignFactSlotDefaults();
#endif

#ifndef _FACTMNGR_SOURCE_
   extern int                            ChangeToFactList;
   extern struct fact                    DummyFact;
#if DEBUGGING_FUNCTIONS
   extern int                            WatchFacts;
#endif
#endif

#endif





