   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 AGENDA HEADER FILE                  */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*   Provides functionality for examining, manipulating,     */
/*   adding, and removing activations from the agenda.       */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_agenda

#define _H_agenda

#ifndef _H_ruledef
#include "ruledef.h"
#endif
#ifndef _H_symbol
#include "symbol.h"
#endif
#ifndef _H_match
#include "match.h"
#endif

#define WHEN_DEFINED 0
#define WHEN_ACTIVATED 1
#define EVERY_CYCLE 2

#define MAX_SALIENCE  10000
#define MIN_SALIENCE -10000

/*******************/
/* DATA STRUCTURES */
/*******************/

struct activation
  { 
   struct defrule *theRule;
   struct partialMatch *basis; 
   int salience;
   unsigned long int timetag; 
#if CONFLICT_RESOLUTION_STRATEGIES 
   struct partialMatch *sortedBasis;
   int randomID;
#endif     
   struct activation *prev;             
   struct activation *next;       
  };

typedef struct activation ACTIVATION;

#define GetActivationSalience(actPtr) (((struct activation *) actPtr)->salience)
#define GetActivationRule(actPtr) (((struct activation *) actPtr)->theRule)
#define GetActivationBasis(actPtr) (((struct activation *) actPtr)->basis)
#define GetActivationSortedBasis(actPtr) (((struct activation *) actPtr)->sortedBasis)

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _AGENDA_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

/****************************************/
/* GLOBAL EXTERNAL FUNCTION DEFINITIONS */
/****************************************/

#if ANSI_COMPILER   
   LOCALE VOID                    SalienceInformationError(char *);
   LOCALE VOID                    AddActivation(void *,VOID *);
   LOCALE VOID                    ClearRuleFromAgenda(VOID *);
   LOCALE VOID                   *GetNextActivation(VOID *);
   LOCALE char                   *GetActivationName(VOID *);
   LOCALE int                     SetActivationSalience(VOID *,int);
   LOCALE VOID                    GetActivationPPForm(char *,int,VOID *);
   LOCALE BOOLEAN                 MoveActivationToTop(VOID *);
   LOCALE BOOLEAN                 DeleteActivation(VOID *);
   LOCALE BOOLEAN                 DetachActivation(VOID *);
   LOCALE VOID                    Agenda(char *,VOID *);
   LOCALE VOID                    RemoveActivation(VOID *,int,int);
   LOCALE VOID                    RemoveAllActivations(void);
   LOCALE int                     GetAgendaChanged(void);
   LOCALE VOID                    SetAgendaChanged(int);
   LOCALE long int                GetNumberOfActivations(void);
   LOCALE BOOLEAN                 GetSalienceEvaluation(void);
   LOCALE BOOLEAN                 SetSalienceEvaluation(BOOLEAN);
   LOCALE VOID                    RefreshAgenda(VOID *);
   LOCALE VOID                    ReorderAgenda(VOID *);
   LOCALE VOID                    InitializeAgenda(void);
   LOCALE SYMBOL_HN              *SetSalienceEvaluationCommand(void);
   LOCALE SYMBOL_HN              *GetSalienceEvaluationCommand(void);
   LOCALE VOID                    RefreshAgendaCommand(void);
   LOCALE VOID                    RefreshCommand(void);
   LOCALE BOOLEAN                 Refresh(VOID *);
   LOCALE VOID                    SalienceRangeError(void);
   LOCALE VOID                    SalienceNonIntegerError(void);
#if DEBUGGING_FUNCTIONS
   LOCALE VOID                    AgendaCommand(void);
#endif
#else
   LOCALE VOID                    SalienceInformationError();
   LOCALE VOID                    AddActivation();
   LOCALE VOID                    ClearRuleFromAgenda();
   LOCALE VOID                   *GetNextActivation();
   LOCALE char                   *GetActivationName();
   LOCALE VOID                   *ActivationBasis();
   LOCALE int                     SetActivationSalience();
   LOCALE VOID                    GetActivationPPForm();
   LOCALE BOOLEAN                 MoveActivationToTop();
   LOCALE BOOLEAN                 DeleteActivation();
   LOCALE BOOLEAN                 DetachActivation();
   LOCALE VOID                    Agenda();
   LOCALE VOID                    RemoveActivation();
   LOCALE VOID                    RemoveAllActivations();
   LOCALE int                     GetAgendaChanged();
   LOCALE VOID                    SetAgendaChanged();
   LOCALE long int                GetNumberOfActivations();
   LOCALE BOOLEAN                 GetSalienceEvaluation();
   LOCALE BOOLEAN                 SetSalienceEvaluation();
   LOCALE VOID                    RefreshAgenda();
   LOCALE VOID                    ReorderAgenda();
   LOCALE VOID                    InitializeAgenda();
   LOCALE SYMBOL_HN              *SetSalienceEvaluationCommand();
   LOCALE SYMBOL_HN              *GetSalienceEvaluationCommand();
   LOCALE VOID                    RefreshAgendaCommand();
   LOCALE VOID                    RefreshCommand();
   LOCALE BOOLEAN                 Refresh();
   LOCALE VOID                    SalienceRangeError();
   LOCALE VOID                    SalienceNonIntegerError();
#if DEBUGGING_FUNCTIONS
   LOCALE VOID                    AgendaCommand();
#endif
#endif

#ifndef _AGENDA_SOURCE_
#if DEBUGGING_FUNCTIONS
   extern BOOLEAN              WatchActivations;
#endif
#endif

#endif






