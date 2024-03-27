   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                RULE COMMANDS MODULE                 */
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

#define _RULECOM_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "setup.h"

#if DEFRULE_CONSTRUCT

#include "constant.h"
#include "clipsmem.h"
#include "evaluatn.h"
#include "extnfunc.h"
#include "engine.h"
#include "watch.h"
#include "crstrtgy.h"
#include "constrct.h"
#include "argacces.h"
#include "router.h"
#include "reteutil.h"
#include "ruledlt.h"
#include "pattern.h"

#if BLOAD || BLOAD_AND_BSAVE || BLOAD_ONLY
#include "rulebin.h"
#endif

#if LOGICAL_DEPENDENCIES
#include "lgcldpnd.h"
#endif

#if INCREMENTAL_RESET
#include "incrrset.h"
#endif

#include "rulecom.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
#if DEVELOPER
   static VOID                    ShowJoins(VOID *);
#endif
#else
#if DEVELOPER
   static VOID                    ShowJoins();
#endif
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

#if DEVELOPER && (! RUN_TIME) && (! BLOAD_ONLY)
   static int                  WatchRuleAnalysis = OFF;
#endif

/************************************************************/
/* DefruleCommands:                                         */
/************************************************************/
globle VOID DefruleCommands()
  {
#if ! RUN_TIME
   DefineFunction2("run",          'v', PTIF RunCommand,          "RunCommand", "*1i");
   DefineFunction2("halt",         'v', PTIF HaltCommand,         "HaltCommand", "00");
   DefineFunction2("focus",        'b', PTIF FocusCommand,        "FocusCommand", "1*w");
   DefineFunction2("clear-focus-stack", 'v', PTIF ClearFocusStackCommand, "ClearFocusStackCommand", "00");
   DefineFunction2("get-focus-stack",'m',PTIF GetFocusStackFunction,"GetFocusStackFunction","00");
   DefineFunction2("pop-focus",'w',PTIF PopFocusFunction,"PopFocusFunction","00");
   DefineFunction2("get-focus",'w',PTIF GetFocusFunction,"GetFocusFunction","00");
#if DEBUGGING_FUNCTIONS
   DefineFunction2("set-break",    'v', PTIF SetBreakCommand,     "SetBreakCommand", "11w");
   DefineFunction2("remove-break", 'v', PTIF RemoveBreakCommand,  "RemoveBreakCommand", "*1w");
   DefineFunction2("show-breaks",  'v', PTIF ShowBreaksCommand,   "ShowBreaksCommand", "01w");
   DefineFunction2("matches",      'v', PTIF MatchesCommand,      "MatchesCommand", "11w");
   DefineFunction2("list-focus-stack", 'v', PTIF ListFocusStackCommand, "ListFocusStackCommand", "00");
#if LOGICAL_DEPENDENCIES
   DefineFunction2("dependencies", 'v', PTIF DependenciesCommand, "DependenciesCommand", "11h");
   DefineFunction2("dependents",   'v', PTIF DependentsCommand,   "DependentsCommand", "11h");
#endif
#endif

#if INCREMENTAL_RESET
   DefineFunction2("get-incremental-reset",'b',
                   GetIncrementalResetCommand,"GetIncrementalResetCommand","00");
   DefineFunction2("set-incremental-reset",'b',
                   SetIncrementalResetCommand,"SetIncrementalResetCommand","11");
#endif

#if CONFLICT_RESOLUTION_STRATEGIES
   DefineFunction2("get-strategy", 'w', PTIF GetStrategyCommand,  "GetStrategyCommand", "00");
   DefineFunction2("set-strategy", 'w', PTIF SetStrategyCommand,  "SetStrategyCommand", "11w");
#endif

#if DEVELOPER && (! BLOAD_ONLY)
   DefineFunction2("rule-complexity",'l', PTIF RuleComplexityCommand,"RuleComplexityCommand", "11w");
   DefineFunction2("show-joins",   'v', PTIF ShowJoinsCommand,    "ShowJoinsCommand", "11w");
   AddWatchItem("rule-analysis",0,&WatchRuleAnalysis,0,NULL,NULL);
#endif

#endif
  }

#if DEBUGGING_FUNCTIONS

/*****************************************************************/
/* MatchesCommand: CLIPS access routine for the matches command. */
/*****************************************************************/
globle VOID MatchesCommand()
  {
   char *ruleName;
   VOID *rulePtr;

   ruleName = GetConstructName("matches","rule name");
   if (ruleName == NULL) return;

   rulePtr = FindDefrule(ruleName);
   if (rulePtr == NULL)
     {
      CantFindItemErrorMessage("defrule",ruleName);
      return;
     }

   Matches(rulePtr);
  }

/******************************************************/
/* Matches: C access routine for the matches command. */
/******************************************************/
globle BOOLEAN Matches(theRule)
  VOID *theRule;
  {
   struct defrule *rulePtr, *tmpPtr;
   struct partialMatch *listOfMatches, **theStorage;
   struct joinNode *theJoin, *lastJoin;
   int i, depth;
   ACTIVATION *agendaPtr;
   int flag;
   int matchesDisplayed;

   rulePtr = (struct defrule *) theRule;

   /*=================================================*/
   /* Loop through each of the disjuncts for the rule */
   /*=================================================*/
   
   tmpPtr = rulePtr;
   while (rulePtr != NULL)
     {
      /*======================================*/
      /* Determine the last join in the rule. */
      /*======================================*/
      
      lastJoin = rulePtr->lastJoin;
      
      /*===================================*/
      /* Determine the number of patterns. */
      /*===================================*/
       
      theJoin = lastJoin;
      depth = 0;
      while (theJoin != NULL)
        {
         if (theJoin->joinFromTheRight)
           { theJoin = (struct joinNode *) theJoin->rightSideEntryStructure; }
         else
           {
            depth++;
            theJoin = theJoin->lastLevel;
           }
        }
        
      /*=========================================*/
      /* Store the alpha memory partial matches. */
      /*=========================================*/
      
      theStorage = (struct partialMatch **) genalloc((unsigned) (depth * sizeof(struct partialMatch)));
      
      theJoin = lastJoin;
      i = depth - 1;
      while (theJoin != NULL)
        {
         if (theJoin->joinFromTheRight)
           { theJoin = (struct joinNode *) theJoin->rightSideEntryStructure; }
         else
           {
            theStorage[i] = ((struct patternNodeHeader *) theJoin->rightSideEntryStructure)->alphaMemory;
            i--;
            theJoin = theJoin->lastLevel;
           }
        }
         
      /*=======================================*/
      /* List individual matches for patterns. */
      /*=======================================*/

      for (i = 0; i < depth; i++)
        {
         if (GetHaltExecution() == CLIPS_TRUE) return(CLIPS_TRUE);

         PrintCLIPS(WDISPLAY,"Matches for Pattern ");
         PrintLongInteger(WDISPLAY,(long int) i + 1);
         PrintCLIPS(WDISPLAY,"\n");
         
         listOfMatches = theStorage[i];
         if (listOfMatches == NULL) PrintCLIPS(WDISPLAY," None\n");
           
         while (listOfMatches != NULL)
           {
            if (GetHaltExecution() == CLIPS_TRUE) return(CLIPS_TRUE);
            PrintPartialMatch(WDISPLAY,listOfMatches);
            PrintCLIPS(WDISPLAY,"\n");
            listOfMatches = listOfMatches->next;
           }
        }
        
      genfree(theStorage,(unsigned) (depth * sizeof(struct partialMatch)));

      /*=======================*/
      /* List partial matches. */
      /*=======================*/

      depth = lastJoin->depth;
      theStorage = (struct partialMatch **) genalloc((unsigned) (depth * sizeof(struct partialMatch)));
      
      theJoin = lastJoin;
      for (i = depth - 1; i >= 0; i--)
        {
         theStorage[i] = theJoin->beta;
         theJoin = theJoin->lastLevel;
        }
      
      for (i = 1; i < depth; i++)
        {
         if (GetHaltExecution() == CLIPS_TRUE) return(CLIPS_TRUE);

         matchesDisplayed = 0;
         PrintCLIPS(WDISPLAY,"Partial matches for CEs 1 - ");
         PrintLongInteger(WDISPLAY,(long int) i + 1);
         PrintCLIPS(WDISPLAY,"\n");
         listOfMatches = theStorage[i];
            
         while (listOfMatches != NULL)
           {
            if (GetHaltExecution() == CLIPS_TRUE) return(CLIPS_TRUE);

            if (listOfMatches->counterf == CLIPS_FALSE)
              {
               matchesDisplayed++;
               PrintPartialMatch(WDISPLAY,listOfMatches);
               PrintCLIPS(WDISPLAY,"\n");
              }
            listOfMatches = listOfMatches->next;
           }
              
         if (matchesDisplayed == 0) { PrintCLIPS(WDISPLAY," None\n"); }
        }
        
      genfree(theStorage,(unsigned) (depth * sizeof(struct partialMatch)));

      rulePtr = rulePtr->disjunct;
     }
   
   /*===================*/
   /* List activations. */
   /*===================*/
   
   rulePtr = tmpPtr;
   PrintCLIPS(WDISPLAY,"Activations\n");
   agendaPtr = (struct activation *) GetNextActivation(NULL);
   flag = 1;
   while (agendaPtr != NULL)
     {
      if (GetHaltExecution() == CLIPS_TRUE) return(CLIPS_TRUE);

      if (((struct activation *) agendaPtr)->theRule->header.name == rulePtr->header.name)
        {
         flag = 0;
         PrintPartialMatch(WDISPLAY,GetActivationBasis(agendaPtr));
         PrintCLIPS(WDISPLAY,"\n");
        }
      agendaPtr = (struct activation *) GetNextActivation(agendaPtr);
     }
   if (flag) PrintCLIPS(WDISPLAY," None\n");

   return(CLIPS_TRUE);
  }
  
#endif
  
#if DEVELOPER
/*************************************************************/
/* RuleComplexityCommand: Displays the complexity of a rule. */
/*************************************************************/
globle long RuleComplexityCommand()
  {
   char *ruleName;
   struct defrule *rulePtr;

   ruleName = GetConstructName("rule-complexity","rule name");
   if (ruleName == NULL) return(-1);

   rulePtr = (struct defrule *) FindDefrule(ruleName);
   if (rulePtr == NULL)
     {      
      CantFindItemErrorMessage("defrule",ruleName);
      return(-1);
     }

   return(rulePtr->complexity);
  }
  
/********************************************************************/
/* ShowJoinsCommand: Shows the join network expressions for a rule. */
/*   Syntax: (show-joins <rule name>)                               */
/********************************************************************/
globle VOID ShowJoinsCommand()
  {
   char *ruleName;
   VOID *rulePtr;

   ruleName = GetConstructName("show-joins","rule name");
   if (ruleName == NULL) return;

   rulePtr = FindDefrule(ruleName);
   if (rulePtr == NULL)
     {
      CantFindItemErrorMessage("defrule",ruleName);
      return;
     }

   ShowJoins(rulePtr);

   return;
  }
  
/****************************************************/
/* ShowJoins: Displays the join network for a rule. */
/****************************************************/
static VOID ShowJoins(theRule)
  VOID *theRule;
  {
   struct defrule *rulePtr, *tmpPtr;
   struct joinNode *theJoin;
   struct joinNode *joinList[MAXIMUM_NUMBER_OF_PATTERNS];
   int numberOfJoins;

   rulePtr = (struct defrule *) theRule;

   /*=================================================*/
   /* Loop through each of the disjuncts for the rule */
   /*=================================================*/
   
   tmpPtr = rulePtr;
   while (rulePtr != NULL)
     {
      /*=====================================*/
      /* Determine the number of join nodes. */
      /*=====================================*/
       
      numberOfJoins = -1;
      theJoin = rulePtr->lastJoin;
      while (theJoin != NULL)
        {
         if (theJoin->joinFromTheRight)
           { theJoin = (struct joinNode *) theJoin->rightSideEntryStructure; }
         else
           {
            numberOfJoins++;
            joinList[numberOfJoins] = theJoin;
            theJoin = theJoin->lastLevel;
           }
        }
        
      /*====================*/
      /* Display the joins. */
      /*====================*/
       
      while (numberOfJoins >= 0)
        {
         char buffer[20];
         sprintf(buffer,"%2d%c%c: ",(int) joinList[numberOfJoins]->depth,
                                     (joinList[numberOfJoins]->patternIsNegated) ? 'n' : ' ',
                                     (joinList[numberOfJoins]->logicalJoin) ? 'l' : ' ');
         PrintCLIPS(WDISPLAY,buffer);
         PrintExpression(WDISPLAY,joinList[numberOfJoins]->networkTest);
         PrintCLIPS(WDISPLAY,"\n");
         numberOfJoins--;
        };
       
      rulePtr = rulePtr->disjunct;
      if (rulePtr != NULL) PrintCLIPS(WDISPLAY,"\n");
     }
  }
  
#endif

#endif /* DEFRULE_CONSTRUCT */

