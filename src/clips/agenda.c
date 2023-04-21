   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                    AGENDA MODULE                    */
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
/*      Brian L. Donnell                                     */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _AGENDA_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "setup.h"

#if DEFRULE_CONSTRUCT

#include "constant.h"
#include "clipsmem.h"
#include "sysdep.h"
#include "router.h"
#include "strngrtr.h"
#include "reteutil.h"
#include "retract.h"
#include "extnfunc.h"
#include "argacces.h"
#include "crstrtgy.h"
#include "watch.h"
#include "moduldef.h"
#include "engine.h"
#include "rulebsc.h"
#include "ruledef.h"
#include "agenda.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                    PrintActivation(char *,VOID *);
   static VOID                    AgendaClearFunction(VOID);
#if DYNAMIC_SALIENCE
   static char                   *SalienceEvaluationName(int);
   static int                     EvaluateSalience(VOID *);
#endif
#else
   static VOID                    PrintActivation();
   static VOID                    AgendaClearFunction();
#if DYNAMIC_SALIENCE
   static char                   *SalienceEvaluationName();
   static int                     EvaluateSalience();
#endif
#endif
   
/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

#if DEBUGGING_FUNCTIONS
   globle BOOLEAN              WatchActivations = OFF;
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static long int             NumberOfActivations = 0;
   static unsigned long int    CurrentTimetag = 0;
   static int                  AgendaChanged = CLIPS_FALSE;
#if DYNAMIC_SALIENCE
   static BOOLEAN              SalienceEvaluation = WHEN_DEFINED;
#endif

/************************************************************/
/* InitializeAgenda: Initializes the activations watch item */
/*   and the CLIPS commands for manipulating the agena.     */
/************************************************************/
globle VOID InitializeAgenda()
  {
   AddClearFunction("agenda",AgendaClearFunction,0);
#if DEBUGGING_FUNCTIONS
   AddWatchItem("activations",1,&WatchActivations,40,DefruleWatchAccess,DefruleWatchPrint);
#endif
#if ! RUN_TIME
   DefineFunction2("refresh", 'v', PTIF RefreshCommand, "RefreshCommand", "11w");
#if DYNAMIC_SALIENCE
   DefineFunction2("refresh-agenda",'v',
                   PTIF RefreshAgendaCommand,"RefreshAgendaCommand", "01w");
   DefineFunction2("get-salience-evaluation",'w',
                   PTIF GetSalienceEvaluationCommand,"GetSalienceEvaluationCommand", "00");
   DefineFunction2("set-salience-evaluation",'w',
                   PTIF SetSalienceEvaluationCommand,"SetSalienceEvaluationCommand", "11w");
#endif
#if DEBUGGING_FUNCTIONS   
   DefineFunction2("agenda", 'v', PTIF AgendaCommand, "AgendaCommand", "01w");
#endif
#endif
  }

/*****************************************************************/
/* AddActivation: Creates a rule activation to be added to the   */
/*   Agenda and links the activation with its associated partial */
/*   match. The function PlaceActivation is then called to place */
/*   the activation on the Agenda. Typically called when all     */
/*   patterns on the LHS of a rule have been satisfied.          */
/*****************************************************************/
globle VOID AddActivation(vTheRule,vBinds)
  VOID *vTheRule;
  VOID *vBinds;
  {
   struct activation *newActivation;
   struct defrule *theRule = (struct defrule *) vTheRule;
   struct partialMatch *binds = (struct partialMatch *) vBinds;
   struct defruleModule *theModuleItem;
   
   /*=======================================*/
   /* Focus on the module if the activation */
   /* is from an auto-focus rule.           */
   /*=======================================*/
   
   if (theRule->autoFocus)
     { Focus((VOID *) theRule->header.whichModule->theModule); }
     
   /*======================================================*/
   /* Create the activation.  Store the join connected to  */
   /* the actions in the rhs and the list of fact bindings */
   /* in the lhs.                                          */
   /*======================================================*/

   newActivation = get_struct(activation);
   newActivation->theRule = theRule;
   newActivation->basis = binds;
   newActivation->timetag = CurrentTimetag++;
#if DYNAMIC_SALIENCE
   newActivation->salience = EvaluateSalience(theRule);
#else
   newActivation->salience = theRule->salience;
#endif
#if CONFLICT_RESOLUTION_STRATEGIES
   newActivation->sortedBasis = NULL;
   newActivation->randomID = genrand();
#endif
   newActivation->prev = NULL;
   newActivation->next = NULL;

   NumberOfActivations++;

   /*=======================================================*/
   /* Point the partial match to the activation to complete */
   /* the link between the join network and the agenda.     */
   /*=======================================================*/

   binds->binds[binds->bcount].gm.theValue = (VOID *) newActivation;

   /*====================================================*/
   /* If activations are being watch, display a message. */
   /*====================================================*/

#if DEBUGGING_FUNCTIONS
   if (newActivation->theRule->watchActivation)
     {
      PrintCLIPS(WTRACE,"==> Activation ");
      PrintActivation(WTRACE,(VOID *) newActivation);
      PrintCLIPS(WTRACE,"\n");
     }
#endif

    /*=====================================*/
    /* Place the activation in the agenda. */
    /*=====================================*/

    theModuleItem = (struct defruleModule *) theRule->header.whichModule;
    PlaceActivation(&(theModuleItem->agenda),newActivation);
   }

/***************************************************************/
/* ClearRuleFromAgenda: Clears the agenda of a specified rule. */
/***************************************************************/
globle VOID ClearRuleFromAgenda(vTheRule)
  VOID *vTheRule;
  {
   struct defrule *theRule = (struct defrule *) vTheRule;
   struct defrule *tempRule;
   struct activation *agendaPtr, *agendaNext;

   agendaPtr = ((struct defruleModule *) theRule->header.whichModule)->agenda;

   /*==============================================*/
   /* Loop through every activation on the agenda. */
   /*==============================================*/

   while (agendaPtr != NULL)
     {
      agendaNext = agendaPtr->next;
      tempRule = theRule;
      while (tempRule != NULL)
        {
         if (agendaPtr->theRule == tempRule)
           { 
            RemoveActivation(agendaPtr,CLIPS_TRUE,CLIPS_TRUE);
            tempRule = NULL;
           }
         else 
           { tempRule = tempRule->disjunct; }
        }
      agendaPtr = agendaNext;
     }
  }

/********************************************************************/
/* GetNextActivation: Returns an activation from the Agenda. If its */
/*   argument is NULL, then the first activation on the Agenda      */
/*   is returned. If its argument is not NULL, the next activation  */
/*   after the argument is returned.                                */
/********************************************************************/
globle VOID *GetNextActivation(actPtr)
  VOID *actPtr;
  {
   struct defruleModule *theModuleItem;
   
   if (actPtr == NULL)
     {
      theModuleItem = (struct defruleModule *) GetModuleItem(NULL,DefruleModuleIndex);
      if (theModuleItem == NULL) return(NULL);
      return((VOID *) theModuleItem->agenda);
     }
   else
     { return((VOID *) (((struct activation *) actPtr)->next)); }
  }

/**********************************************************************************/
/* GetActivationName: Returns the name of the rule associated with an activation. */
/**********************************************************************************/
globle char *GetActivationName(actPtr)
  VOID *actPtr;
  { return(ValueToString(((struct activation *) actPtr)->theRule->header.name)); }

/********************************************************************/
/* SetActivationSalience: Sets the salience value of an activation. */
/********************************************************************/
globle int SetActivationSalience(actPtr,value)
  VOID *actPtr;
  int value;
  {
   int temp;

   temp = ((struct activation *) actPtr)->salience;
   ((struct activation *) actPtr)->salience = value;
   return(temp);
  }

/**********************************************************************************/
/* GetActivationPPForm: Returns the pretty print representation of an activation. */
/**********************************************************************************/
globle VOID GetActivationPPForm(buffer,bufferLength,theActivation)
  char *buffer;
  int bufferLength;
  VOID *theActivation;
  {
   OpenStringDestination("ActPPForm",buffer,bufferLength);
   PrintActivation("ActPPForm",(VOID *) theActivation);
   CloseStringDestination("ActPPForm");
  }

/********************************************/
/* MoveActivationToTop: Moves the specified */
/*   activation to the top of the agenda.   */
/********************************************/
globle BOOLEAN MoveActivationToTop(vtheActivation)
  VOID *vtheActivation;
  {
   struct activation *prevPtr;
   struct activation *theActivation = (struct activation *) vtheActivation;
   struct defruleModule *theModuleItem;

   theModuleItem = (struct defruleModule *) theActivation->theRule->header.whichModule;
   
   prevPtr = theActivation->prev;

   if (theActivation != theModuleItem->agenda)
     {
      prevPtr->next = theActivation->next;
      if (theActivation->next != NULL) theActivation->next->prev = prevPtr;

      theActivation->next = theModuleItem->agenda;
      theModuleItem->agenda->prev = theActivation;
      theActivation->prev = NULL;
      theModuleItem->agenda = theActivation;
     }

   AgendaChanged = CLIPS_TRUE;
   return(CLIPS_TRUE);
  }

/******************************************************/
/* DeleteActivation: Removes the specified activation */
/*   from the agenda.                                 */
/******************************************************/
globle BOOLEAN DeleteActivation(theActivation)
  VOID *theActivation;
  {
   if (theActivation == NULL) RemoveAllActivations();
   else RemoveActivation((struct activation *) theActivation,CLIPS_TRUE,CLIPS_TRUE);

   return(CLIPS_TRUE);
  }
  
/*******************************************************/
/* DetachActivation: Detaches the specified activation */
/*   from the list of activations on the Agenda.       */
/*******************************************************/
globle BOOLEAN DetachActivation(vTheActivation)
  VOID *vTheActivation;
  {
   struct defruleModule *theModuleItem;
   struct activation *theActivation = (struct activation *) vTheActivation;
   
   if (theActivation == NULL) return(CLIPS_FALSE);
   
   theModuleItem = (struct defruleModule *) theActivation->theRule->header.whichModule;
   
   if (theActivation->prev == NULL)
     {
      if (theActivation != theModuleItem->agenda) return(CLIPS_FALSE);
      
      theModuleItem->agenda = theActivation->next;
      if (theModuleItem->agenda != NULL) theModuleItem->agenda->prev = NULL;
      theActivation->next = NULL;
      AgendaChanged = CLIPS_TRUE;
      return(CLIPS_TRUE);
     }
     
   theActivation->prev->next = theActivation->next;
   if (theActivation->next != NULL) 
     { theActivation->next->prev = theActivation->prev; }
     
   theActivation->prev = NULL;
   theActivation->next = NULL;

   AgendaChanged = CLIPS_TRUE;
   return(CLIPS_TRUE);
  }

/****************************************************************************/
/* PrintActivation: Prints an activation in a �pretty� format. Salience,    */
/*   rule name, and the partial match which activated the rule are printed. */
/****************************************************************************/
static VOID PrintActivation(logicalName,vTheActivation)
  char *logicalName;
  VOID *vTheActivation;
  {
   struct activation *theActivation = (struct activation *) vTheActivation;
   char printSpace[20];

   sprintf(printSpace,"%-6d ",theActivation->salience);
   PrintCLIPS(logicalName,printSpace);
   PrintCLIPS(logicalName,ValueToString(theActivation->theRule->header.name));
   PrintCLIPS(logicalName,": ");
   PrintPartialMatch(logicalName,theActivation->basis);
  }

/****************************************************/
/* Agenda: C access routine for the agenda command. */
/****************************************************/
globle VOID Agenda(logicalName,vTheModule)
  char *logicalName;
  VOID *vTheModule;
  {
   struct defmodule *theModule = (struct defmodule *) vTheModule;

   ListItemsDriver(logicalName,theModule,"activation","activations",
                   GetNextActivation,NULL,PrintActivation,NULL);
  }

/*******************************************************************/
/* RemoveActivation: Returns an activation and its associated data */
/*   structures to the Memory Manager. Links to other activations  */
/*   and partial matches may also be updated.                      */
/*******************************************************************/
globle VOID RemoveActivation(vTheActivation,updateAgenda,updateLinks)
  VOID *vTheActivation;
  int updateAgenda, updateLinks;
  {
   struct defruleModule *theModuleItem;
   struct activation *theActivation = (struct activation *) vTheActivation;
   
   theModuleItem = (struct defruleModule *) theActivation->theRule->header.whichModule;

   /*=================================*/
   /* Update the agenda if necessary. */
   /*=================================*/

   if (updateAgenda == CLIPS_TRUE)
     {
      if (theActivation->prev == NULL)
        {
         theModuleItem->agenda = theModuleItem->agenda->next;
         if (theModuleItem->agenda != NULL) theModuleItem->agenda->prev = NULL;
        }
      else
        {
         theActivation->prev->next = theActivation->next;
         if (theActivation->next != NULL)
           { theActivation->next->prev = theActivation->prev; }
        }

      /*===============================================*/
      /* Indicate removal of activation if activations */
      /* are being watched.                            */
      /*===============================================*/

#if DEBUGGING_FUNCTIONS
      if (theActivation->theRule->watchActivation)
        {
         PrintCLIPS(WTRACE,"<== Activation ");
         PrintActivation(WTRACE,(VOID *) theActivation);
         PrintCLIPS(WTRACE,"\n");
        }
#endif

      AgendaChanged = CLIPS_TRUE;
     }

   /*============================================*/
   /* Update join and agenda links if necessary. */
   /*============================================*/

   if ((updateLinks == CLIPS_TRUE) && (theActivation->basis != NULL))
     { theActivation->basis->binds[theActivation->basis->bcount].gm.theValue = NULL; }

   /*========================*/
   /* Return the activation. */
   /*========================*/

   NumberOfActivations--;

#if CONFLICT_RESOLUTION_STRATEGIES
   if (theActivation->sortedBasis != NULL)
     { ReturnPartialMatch(theActivation->sortedBasis); }
#endif

   rtn_struct(activation,theActivation);
  }

/******************************************************************/
/* AgendaClearFunction: */
/******************************************************************/
static VOID AgendaClearFunction()
  { 
   CurrentTimetag = 0; 
  }

/******************************************************************/
/* RemoveAllActivations: Removes all activations from the agenda. */
/******************************************************************/
globle VOID RemoveAllActivations()
  {
   struct activation *tempPtr, *theActivation;

   theActivation = GetDefruleModuleItem(NULL)->agenda;
   while (theActivation != NULL)
     {
      tempPtr = theActivation->next;
      RemoveActivation(theActivation,CLIPS_TRUE,CLIPS_TRUE);
      theActivation = tempPtr;
     }
  }

/**********************************************************************/
/* GetAgendaChanged: Returns the value of the variable AgendaChanged. */
/**********************************************************************/
globle int GetAgendaChanged()
  { 
   return(AgendaChanged);
  }

/*******************************************************************/
/* SetAgendaChanged: Sets the value of the variable AgendaChanged. */
/*******************************************************************/
globle VOID SetAgendaChanged(value)
  int value;
  {
   AgendaChanged = value;
  }

/*******************************************************/
/* ReorderAgenda: Completely reorders the agenda based */
/*   on the current conflict resolution strategy.      */
/*******************************************************/
globle VOID ReorderAgenda(vTheModule)
  VOID *vTheModule;
  {
   struct activation *theActivation, *tempPtr;
   struct defmodule *theModule = (struct defmodule *) vTheModule;
   int allModules = CLIPS_FALSE;
   struct defruleModule *theModuleItem;

   if (theModule == NULL)
     {
      allModules = CLIPS_TRUE;
      theModule = (struct defmodule *) GetNextDefmodule(NULL);
     }

   while (theModule != NULL)
     { 
      theModuleItem = GetDefruleModuleItem(theModule);

      theActivation = theModuleItem->agenda;
      theModuleItem->agenda = NULL;

      while (theActivation != NULL)
        {
         tempPtr = theActivation->next;
         theActivation->next = NULL;
         theActivation->prev = NULL;
         PlaceActivation(&(theModuleItem->agenda),theActivation);
         theActivation = tempPtr;
        }
      
      if (allModules) theModule = (struct defmodule *) GetNextDefmodule(theModule);
      else theModule = NULL;
     }
  }
  
/**********************************************************************************/
/* GetNumberOfActivations: Returns the value of the variable NumberOfActivations. */
/**********************************************************************************/
globle long int GetNumberOfActivations()
  { return(NumberOfActivations); }
  
/********************************************************/
/* RefreshCommand: CLIPS command for refreshing a rule. */
/*   Syntax: (refresh <defrule-name>)                   */
/********************************************************/
globle VOID RefreshCommand()
  {
   char *ruleName;
   VOID *rulePtr;

   ruleName = GetConstructName("refresh","rule name");
   if (ruleName == NULL) return;

   rulePtr = FindDefrule(ruleName);
   if (rulePtr == NULL)
     {
      CantFindItemErrorMessage("defrule",ruleName);
      return;
     }

   Refresh(rulePtr);
  }

/****************************************************************/
/* Refresh: Refreshes a defrule. Activations of the rule */
/*   that have already been fired are added to the agenda.      */
/****************************************************************/
globle BOOLEAN Refresh(theRule)
  VOID *theRule;
  {
   struct defrule *rulePtr;
   struct partialMatch *listOfMatches;

   rulePtr = (struct defrule *) theRule;
   
   while (rulePtr != NULL)
     {
      listOfMatches = rulePtr->lastJoin->beta;
      while (listOfMatches != NULL)
        {
         if ((listOfMatches->activationf) && (! listOfMatches->counterf))
           {
            if (listOfMatches->binds[listOfMatches->bcount].gm.theValue == NULL)
              { AddActivation(rulePtr,listOfMatches); }
           }

         listOfMatches = listOfMatches->next;
        }
      rulePtr = rulePtr->disjunct;
     }

   return(CLIPS_TRUE);
  }
  
/*************************************************************/
/* SalienceInformationError:                                  */
/*************************************************************/
globle VOID SalienceInformationError(ruleName)
  char *ruleName;
  {
   PrintErrorID("AGENDA",3,CLIPS_TRUE);
   PrintCLIPS(WERROR,"This error occurred while evaluating the salience");
   if (ruleName != NULL)
     {
      PrintCLIPS(WERROR," for rule ");
      PrintCLIPS(WERROR,ruleName);
     }
   PrintCLIPS(WERROR,".\n");
  }

/*************************************************************/
/* SalienceRangeError:                                  */
/*************************************************************/
globle VOID SalienceRangeError()
  {
   PrintErrorID("AGENDA",2,CLIPS_TRUE);
   PrintCLIPS(WERROR,"Salience value out of range ");
   PrintLongInteger(WERROR,(long int) MIN_SALIENCE);
   PrintCLIPS(WERROR," to ");
   PrintLongInteger(WERROR,(long int) MAX_SALIENCE);
   PrintCLIPS(WERROR,".\n");
  }
  
/*************************************************************/
/* SalienceNonIntegerError:                                  */
/*************************************************************/
globle VOID SalienceNonIntegerError()
  {
   PrintErrorID("AGENDA",1,CLIPS_TRUE);
   PrintCLIPS(WERROR,"Salience value must be an integer value.\n");
  }
         
#if DYNAMIC_SALIENCE
/**********************************************/
/* RefreshAgendaCommand: CLIPS access routine */
/*   for the refresh-agenda command.          */
/**********************************************/
globle VOID RefreshAgendaCommand()
  {
   int numArgs, error;
   struct defmodule *theModule;
   
   if ((numArgs = ArgCountCheck("refresh-agenda",NO_MORE_THAN,1)) == -1) return;

   if (numArgs == 1)
     {
      theModule = GetModuleName("refresh-agenda",1,&error);
      if (error) return;
     }
   else
     { theModule = ((struct defmodule *) GetCurrentModule()); }

   RefreshAgenda(theModule);
  }

/***************************************/
/* RefreshAgenda: C access routine for */
/*   the refresh-agenda command.       */
/***************************************/
globle VOID RefreshAgenda(vTheModule)
  VOID *vTheModule;
  {
   struct activation *theActivation;
   struct defmodule *theModule = (struct defmodule *) vTheModule;
   BOOLEAN oldValue;
   int allModules = CLIPS_FALSE;
   
   SaveCurrentModule();
   
   if (theModule == NULL)
     {
      allModules = CLIPS_TRUE;
      theModule = (struct defmodule *) GetNextDefmodule(NULL);
     }

   while (theModule != NULL)
     {
      SetCurrentModule((VOID *) theModule);
      theActivation = (struct activation *) GetNextActivation(NULL);

      oldValue = GetSalienceEvaluation();
      SetSalienceEvaluation(WHEN_ACTIVATED);

      while (theActivation != NULL)
        {
         theActivation->salience = EvaluateSalience(theActivation->theRule);
         theActivation = (struct activation *) GetNextActivation(theActivation);
        }

      ReorderAgenda(theModule);
      
      if (allModules) theModule = (struct defmodule *) GetNextDefmodule(theModule);
      else theModule = NULL;
     }

   SetSalienceEvaluation(oldValue);
   RestoreCurrentModule();
  }
 
/************************************************************/
/* SetSalienceEvaluationCommand: CLIPS command for setting  */
/*   the salience evaluation behavior.                      */
/*   Syntax: (set-salience-evaluation-behavior <symbol>)    */
/************************************************************/
globle SYMBOL_HN *SetSalienceEvaluationCommand()
  {
   DATA_OBJECT argPtr;
   char *argument;

   if (ArgCountCheck("set-salience-evaluation",EXACTLY,1) == -1)
     { return((SYMBOL_HN *) AddSymbol(SalienceEvaluationName(GetSalienceEvaluation()))); }

   if (ArgTypeCheck("set-salience-evaluation",1,SYMBOL,&argPtr) == CLIPS_FALSE)
     { return((SYMBOL_HN *) AddSymbol(SalienceEvaluationName(GetSalienceEvaluation()))); }

   argument = DOToString(argPtr);

   if (strcmp(argument,"when-defined") == 0)
     { SetSalienceEvaluation(WHEN_DEFINED); }
   else if (strcmp(argument,"when-activated") == 0)
     { SetSalienceEvaluation(WHEN_ACTIVATED); }
   else if (strcmp(argument,"every-cycle") == 0)
     { SetSalienceEvaluation(EVERY_CYCLE); }
   else
     {
      ExpectedTypeError1("set-salience-evaluation",1,
      "symbol with value when-defined, when-activated, or every-cycle");
      return((SYMBOL_HN *) AddSymbol(SalienceEvaluationName(GetSalienceEvaluation())));
     }

   return((SYMBOL_HN *) AddSymbol(argument));
  }
  
/************************************************************/
/* GetSalienceEvaluationCommand: CLIPS command for getting  */
/*   the salience evaluation behavior.                      */
/*   Syntax: (get-salience-evaluation-behavior)             */
/************************************************************/
globle SYMBOL_HN *GetSalienceEvaluationCommand()
  {
   ArgCountCheck("get-salience-evaluation",EXACTLY,0);

   return((SYMBOL_HN *) AddSymbol(SalienceEvaluationName(GetSalienceEvaluation())));
  }

/*****************************************************************/
/* SalienceEvaluationName: Given the integer value corresponding */
/*   to a specified salience evaluation behavior, returns a      */
/*   character stringof the behavior's name.                     */
/*****************************************************************/
static char *SalienceEvaluationName(strategy)
  int strategy;
  {
   char *sname;

   switch (strategy)
     {
      case WHEN_DEFINED:
        sname = "when-defined";
        break;
      case WHEN_ACTIVATED:
        sname = "when-activated";
        break;
      case EVERY_CYCLE:
        sname = "every-cycle";
        break;
      default:
        sname = "unknown";
        break;
     }

   return(sname);
  }
  
/********************************************************************************/
/* GetSalienceEvaluation: Returns the value of the variable SalienceEvaluation. */
/********************************************************************************/
globle BOOLEAN GetSalienceEvaluation()
  { return(SalienceEvaluation); }

/*****************************************************************************/
/* SetSalienceEvaluation: Sets the value of the variable SalienceEvaluation. */
/*****************************************************************************/
globle BOOLEAN SetSalienceEvaluation(value)
  int value;
  {
   int ov;

   ov = SalienceEvaluation;
   SalienceEvaluation = value;
   return(ov);
  } 

/*****************************************************************/
/* EvaluateSalience: Returns the salience value of the specified */
/*   defrule. If salience evaluation is currently set to         */
/*   when-defined, then the current value of the rule's salience */
/*   is returned. Otherwise the salience expression associated   */
/*   with the rule is reevaluated, the value is stored as the    */
/*   rule's current salience, and it is then returned.           */
/*****************************************************************/
static int EvaluateSalience(vPtr)
 VOID *vPtr;
 {
  struct defrule *rPtr;
  DATA_OBJECT salienceValue;
  int salience;

  rPtr = (struct defrule *) vPtr;

  if (GetSalienceEvaluation() != WHEN_DEFINED)
    {
     if (rPtr->dynamicSalience == NULL) return(rPtr->salience);

     SetEvaluationError(CLIPS_FALSE);
     if (EvaluateExpression(rPtr->dynamicSalience,&salienceValue))
       {
        SalienceInformationError(ValueToString(rPtr->header.name));
        return(rPtr->salience);
       }

     if (salienceValue.type != INTEGER)
       {
        SalienceNonIntegerError();
        SalienceInformationError(ValueToString(rPtr->header.name));
        SetEvaluationError(CLIPS_TRUE);
        return(rPtr->salience);
       }

     salience = (int) ValueToLong(salienceValue.value);

     if ((salience > MAX_SALIENCE) || (salience < MIN_SALIENCE))
        {
         SalienceRangeError();
         SetEvaluationError(CLIPS_TRUE);
         SalienceInformationError(ValueToString(((struct defrule *) rPtr)->header.name));
         return(rPtr->salience);
        }
     rPtr->salience = salience;
    }

  return(rPtr->salience);
 }
#endif
 
#if DEBUGGING_FUNCTIONS

/************************************************/
/* AgendaCommand: Prints out the agenda of the */
/*   rules that are ready to fire.              */
/*   Syntax: (agenda)                           */
/************************************************/
globle VOID AgendaCommand()
  {
   int numArgs, error;
   struct defmodule *theModule;
   
   if ((numArgs = ArgCountCheck("agenda",NO_MORE_THAN,1)) == -1) return;

   if (numArgs == 1)
     {
      theModule = GetModuleName("agenda",1,&error);
      if (error) return;
     }
   else
     { theModule = ((struct defmodule *) GetCurrentModule()); }
     
   Agenda(WDISPLAY,theModule);
  }
  
#endif

#endif

