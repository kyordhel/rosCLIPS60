   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                    ENGINE MODULE                    */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Bebe Ly                                              */
/*      Brian L. Donnell                                     */
/*      Salvador Marmol                                      */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _ENGINE_SOURCE_

#include <stdio.h>
//#include <tcl.h>
//#include <tk.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "setup.h"

#if DEFRULE_CONSTRUCT

#include "constant.h"
#include "clipsmem.h"
#include "watch.h"
#include "router.h"
#include "argacces.h"
#include "reteutil.h"
#include "utility.h"
#include "retract.h"
#include "ruledlt.h"
#include "prccode.h"
#include "prcdrfun.h"
#include "inscom.h"
#include "factmngr.h" 
#include "sysdep.h"
#include "agenda.h"

#include "engine.h"

/*************************/
/* STRUCTURE DEFINITIONS */
/*************************/

struct runFunction
  {
   char *name;
#if ANSI_COMPILER
   VOID (*ip)(void);
#else
   VOID (*ip)();
#endif
   int priority;
   struct runFunction *next;
  };
        
/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static struct activation      *NextActivationToFire(void);
#else
   static struct activation      *NextActivationToFire();
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct defrule      *ExecutingRule = NULL;
   globle BOOLEAN              HaltRules = CLIPS_FALSE;
#if LOGICAL_DEPENDENCIES
   globle struct joinNode     *TheLogicalJoin = NULL;
#endif

/****************************************/
/* GLOBAL EXTERNAL VARIABLE DEFINITIONS */
/****************************************/	

extern int TkClipsFlag;
extern char TkClipsCmd[1000][256];

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static struct runFunction  *ListOfRunFunctions = NULL;
   static struct focus        *CurrentFocus = NULL;
   static int                  FocusChanged = CLIPS_FALSE;
#if DEBUGGING_FUNCTIONS
   static int                  WatchStatistics = OFF;
   static int                  WatchFocus = OFF;
#endif
  
/**********************************************/
/* Run: C access routine for the run command. */
/**********************************************/
globle long int Run(runLimit)
  long int runLimit;
  {
   long int rulesFired = 0;
   DATA_OBJECT result;
   struct runFunction *execPtr;
#if DEBUGGING_FUNCTIONS
   long int maxActivations = 0, sumActivations = 0;
#if DEFTEMPLATE_CONSTRUCT
   long int maxFacts = 0, sumFacts = 0;
#endif
#if OBJECT_SYSTEM
   long int maxInstances = 0, sumInstances = 0;
#endif
   double endTime = 0.0, startTime = 0.0;
   long int tempValue;
#endif
   unsigned int i;
   static int alreadyRunning = CLIPS_FALSE;
   struct patternEntity *theMatchingItem;
   struct partialMatch *theBasis;
   ACTIVATION *theActivation;
   char *RuleFiring = NULL;

   /*=====================================================*/
   /* Make sure the run command is not already executing. */
   /*=====================================================*/

   if (alreadyRunning) return(0);
   alreadyRunning = CLIPS_TRUE;

   /*================================*/
   /* Set up statistics information. */
   /*================================*/

#if DEBUGGING_FUNCTIONS
   if (WatchStatistics)
     {
#if DEFTEMPLATE_CONSTRUCT
      maxFacts = GetNumberOfFacts();
      sumFacts = maxFacts;
#endif
#if OBJECT_SYSTEM
      maxInstances = GetGlobalNumberOfInstances();
      sumInstances = maxInstances;
#endif
      maxActivations = GetNumberOfActivations();
      sumActivations = maxActivations;
      startTime = gentime();
     }
#endif

   /*=============================*/
   /* Set up execution variables. */
   /*=============================*/

   if (CurrentEvaluationDepth == 0) SetHaltExecution(CLIPS_FALSE);
   HaltRules = CLIPS_FALSE;
     
   /*=====================================================*/
   /* Fire rules until the agenda is empty, the run limit */
   /* has been reached, or a rule execution error occurs. */
   /*=====================================================*/
   
   theActivation = NextActivationToFire();     
   while ((theActivation != NULL) &&
          (runLimit != 0) &&
          (HaltExecution == CLIPS_FALSE) &&
          (HaltRules == CLIPS_FALSE))
     {      
       /* Check for a possible Tk events*/

	//Tcl_DoOneEvent(TCL_DONT_WAIT);
/*	if (TkClipsFlag)	
	{
		TkClipsFlag--;
        	RouteCommand(TkClipsCmd[TkClipsFlag]);
	AssertString(TkClipsCmd[TkClipsFlag]);
	printf("Ejecutaria %s\n",TkClipsCmd[TkClipsFlag]);
	fflush(stdout);
      	}*/



      /*==========================*/
      /* Bookkeeping and Tracing. */
      /*==========================*/

      DetachActivation(theActivation);
      RuleFiring = GetActivationName(theActivation);
      theBasis = (struct partialMatch *) GetActivationBasis(theActivation);
      ExecutingRule = (struct defrule *) GetActivationRule(theActivation);
      
      rulesFired++;
      if (runLimit > 0) { runLimit--; }

#if DEBUGGING_FUNCTIONS
      if (ExecutingRule->watchFiring)
        {
         char printSpace[60];

         sprintf(printSpace,"FIRE %4ld ",rulesFired);
         PrintCLIPS(WTRACE,printSpace);
         PrintCLIPS(WTRACE,RuleFiring);
         PrintCLIPS(WTRACE,": ");
         PrintPartialMatch(WTRACE,theBasis);
         PrintCLIPS(WTRACE,"\n");
        }
#endif
      
      /*=================================================*/
      /* Remove the link between the activation and the  */
      /* completed match for the rule. Set the busy flag */
      /* for the completed match to TRUE (so the match   */
      /* upon which our RHS variables are dependent is   */
      /* not deleted while our rule is firing). Set up   */
      /* the global pointers to the completed match for  */
      /* routines which do variable extractions.         */
      /*=================================================*/

      theBasis->binds[theBasis->bcount].gm.theValue = NULL;
      theBasis->busy = CLIPS_TRUE;

      GlobalLHSBinds = theBasis;
      GlobalRHSBinds = NULL;

      /*===================================================================*/
      /* Increment the count for each of the facts/objects associated with */
      /* the rule activation so that the facts/objects cannot be deleted   */
      /* by garbage collection while the rule is executing.                */
      /* Also need to insure that a basis copy is made of an object        */
      /* if it changes on the RHS of a rule for which it is a basis        */
      /*===================================================================*/

      for (i = 0; i < theBasis->bcount; i++)
        {
         theMatchingItem = theBasis->binds[i].gm.theMatch->matchingItem;
         if (theMatchingItem != NULL)
           { (*theMatchingItem->theInfo->incrementBasisCount)(theMatchingItem); }
        }

      /*====================================================*/
      /* Execute the rule's right hand side actions. If the */
      /* rule has logical patterns, set up the pointer to   */
      /* the rules logical join so the assert command will  */
      /* attach the appropriate dependencies to the facts.  */
      /*====================================================*/

#if LOGICAL_DEPENDENCIES
      TheLogicalJoin = ExecutingRule->logicalJoin;
#endif
      CurrentEvaluationDepth++;
      SetEvaluationError(CLIPS_FALSE);
      ExecutingRule->executing = CLIPS_TRUE;
      EvaluateProcActions(ExecutingRule->header.whichModule->theModule,
                          ExecutingRule->actions,ExecutingRule->localVarCnt,
                          &result,NULL);
      ExecutingRule->executing = CLIPS_FALSE;
      SetEvaluationError(CLIPS_FALSE);
      CurrentEvaluationDepth--;
#if LOGICAL_DEPENDENCIES
      TheLogicalJoin = NULL;
#endif
        
#if DEBUGGING_FUNCTIONS
      if ((HaltExecution) || (HaltRules && ExecutingRule->watchFiring))
#else
      if ((HaltExecution) || (HaltRules))
#endif

        {
         PrintErrorID("PRCCODE",4,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Execution halted during the actions of defrule ");
         PrintCLIPS(WDIALOG,RuleFiring);
         PrintCLIPS(WDIALOG,".\n");
        }

      theBasis->busy = CLIPS_FALSE;

      /*===================================================================*/
      /* Decrement the count for each of the facts/objects associated with */
      /* the rule activation. If the last match for the activation         */
      /* is from a not CE, then we need to make sure that the last         */
      /* match is an actual match for the CE and not a counter.            */
      /* Also, when an object is no longer needed by an RHS, we need       */
      /* to insure that any basis copies are removed                       */
      /*===================================================================*/

      for (i = 0; i < (theBasis->bcount - 1); i++)
        {
         theMatchingItem = theBasis->binds[i].gm.theMatch->matchingItem;
         if (theMatchingItem != NULL)
           { (*theMatchingItem->theInfo->decrementBasisCount)(theMatchingItem); }
        }
        
      i = theBasis->bcount - 1;
      if (theBasis->counterf == CLIPS_FALSE)
        {
         theMatchingItem = theBasis->binds[i].gm.theMatch->matchingItem;
         if (theMatchingItem != NULL)
           { (*theMatchingItem->theInfo->decrementBasisCount)(theMatchingItem); }
        }

      /*========================================*/
      /* Return the agenda node to free memory. */
      /*========================================*/

      RemoveActivation(theActivation,CLIPS_FALSE,CLIPS_FALSE);

      /*======================================*/
      /* Get rid of partial matches discarded */
      /* while executing the rule's RHS.      */
      /*======================================*/
      
      FlushGarbagePartialMatches();

      /*==================================*/
      /* Get rid of other garbage created */
      /* while executing the rule's RHS.  */
      /*==================================*/
      
      PeriodicCleanup(CLIPS_FALSE,CLIPS_TRUE);

      /*==========================*/
      /* Keep up with statistics. */
      /*==========================*/
      
#if DEBUGGING_FUNCTIONS
      if (WatchStatistics)
        {
#if DEFTEMPLATE_CONSTRUCT
         tempValue = GetNumberOfFacts();
         if (tempValue > maxFacts) maxFacts = tempValue;
         sumFacts += tempValue;
#endif
#if OBJECT_SYSTEM
         tempValue = GetGlobalNumberOfInstances();
         if (tempValue > maxInstances) maxInstances = tempValue;
         sumInstances += tempValue;
#endif
         tempValue = GetNumberOfActivations();
         if (tempValue > maxActivations) maxActivations = tempValue;
         sumActivations += tempValue;
        }
#endif

      /*==================================*/
      /* Update saliences if appropriate. */
      /*==================================*/

#if DYNAMIC_SALIENCE
      if (GetSalienceEvaluation() == EVERY_CYCLE) RefreshAgenda(NULL);
#endif

      /*=============================================*/
      /* Execute exec list after performing actions. */
      /*=============================================*/

      execPtr = ListOfRunFunctions;
      while (execPtr != NULL)
        {
         (*execPtr->ip)();
         execPtr = execPtr->next;
        }

      /*==============================================*/
      /* If a return was issued on the RHS of a rule, */
      /* then pop the current focus.                  */
      /*==============================================*/
      
      if (ReturnFlag == CLIPS_TRUE) PopFocus();
      ReturnFlag = CLIPS_FALSE;
      
      /*========================*/
      /* Check for breakpoints. */
      /*========================*/

      theActivation = (struct activation *) NextActivationToFire();
      if (theActivation != NULL)
        {
         if (((struct defrule *) GetActivationRule(theActivation))->afterBreakpoint)
           {
            HaltRules = CLIPS_TRUE;
            PrintCLIPS(WDIALOG,"Breaking on rule ");
            PrintCLIPS(WDIALOG,GetActivationName(theActivation));
            PrintCLIPS(WDIALOG,".\n");
           }
        }
     }

   if (runLimit == rulesFired)
     { PrintCLIPS(WDIALOG,"rule firing limit reached\n"); }

   /*==============================*/
   /* Restore execution variables. */
   /*==============================*/

   ExecutingRule = NULL;
   HaltRules = CLIPS_FALSE;
   RuleFiring = NULL;

   /*=================================================*/
   /* Print out statistics if they are being watched. */
   /*=================================================*/
   
#if DEBUGGING_FUNCTIONS
   if (WatchStatistics)
     {
      char printSpace[60];

      endTime = gentime();

      PrintLongInteger(WDIALOG,rulesFired);
      PrintCLIPS(WDIALOG," rules fired");

#if (! GENERIC)
      if (startTime != endTime)
        {
         PrintCLIPS(WDIALOG,"        Run time is ");
         PrintFloat(WDIALOG,endTime - startTime);
         PrintCLIPS(WDIALOG," seconds.\n");
         PrintFloat(WDIALOG,(double) rulesFired / (endTime - startTime));
         PrintCLIPS(WDIALOG," rules per second.\n");
        }
      else
        { PrintCLIPS(WDIALOG,"\n"); }
#endif

#if DEFTEMPLATE_CONSTRUCT
      sprintf(printSpace,"%ld mean number of facts (%ld maximum).\n",
                          (long) (((double) sumFacts / (rulesFired + 1)) + 0.5),
                          maxFacts);
      PrintCLIPS(WDIALOG,printSpace);
#endif

#if OBJECT_SYSTEM
      sprintf(printSpace,"%ld mean number of instances (%ld maximum).\n",
                          (long) (((double) sumInstances / (rulesFired + 1)) + 0.5),
                          maxInstances);
      PrintCLIPS(WDIALOG,printSpace);
#endif

      sprintf(printSpace,"%ld mean number of activations (%ld maximum).\n",
                          (long) (((double) sumActivations / (rulesFired + 1)) + 0.5),
                          maxActivations);
      PrintCLIPS(WDIALOG,printSpace);
     }
#endif

   /*==========================================*/
   /* The current module should be the current */
   /* focus when the run finishes.             */
   /*==========================================*/
   
   if (CurrentFocus != NULL)
     {
      if (CurrentFocus->theModule != ((struct defmodule *) GetCurrentModule()))
        { SetCurrentModule((VOID *) CurrentFocus->theModule); }
     }
        
   /*===================================*/
   /* Return the number of rules fired. */
   /*===================================*/

   alreadyRunning = CLIPS_FALSE;
   return(rulesFired);
  }
  
/**************************************************************/
/* NextActivationToFire: */
/**************************************************************/
static struct activation *NextActivationToFire()
  {
   struct activation *theActivation;
   struct defmodule *theModule;
   
   if (CurrentFocus == NULL)
     { 
      theModule = (struct defmodule *) FindDefmodule("MAIN");
      Focus(theModule);
     }
     
   theActivation = CurrentFocus->theDefruleModule->agenda;
   while ((theActivation == NULL) && (CurrentFocus != NULL))
     {
      if (CurrentFocus != NULL) PopFocus(); 
      if (CurrentFocus != NULL) theActivation = CurrentFocus->theDefruleModule->agenda;
     }
     
   return(theActivation);
  }
  
/*********************************************************/
/* PopFocus: Removes the top focus from the focus stack. */
/*********************************************************/
globle VOID *PopFocus()
  {
   struct focus *tempFocus;
   VOID *rv;
   
   if (CurrentFocus == NULL) return(NULL);
   
   rv = (VOID *) CurrentFocus->theModule;
   
#if DEBUGGING_FUNCTIONS
   if (WatchFocus)
     {
      PrintCLIPS(WTRACE,"<== Focus ");
      PrintCLIPS(WTRACE,ValueToString(CurrentFocus->theModule->name));
      
      if (CurrentFocus->next != NULL)
        {
         PrintCLIPS(WTRACE," to ");
         PrintCLIPS(WTRACE,ValueToString(CurrentFocus->next->theModule->name));
        }
        
      PrintCLIPS(WTRACE,"\n");
     }
#endif
     
   tempFocus = CurrentFocus;
   CurrentFocus = CurrentFocus->next;
   rtn_struct(focus,tempFocus);
   
   if (CurrentFocus != NULL) SetCurrentModule((VOID *) CurrentFocus->theModule);
   FocusChanged = CLIPS_TRUE;
   
   return(rv);
  }
  
/**************************************************************/
/* GetNextFocus: */
/**************************************************************/
globle VOID *GetNextFocus(theFocus)
  VOID *theFocus;
  {
   if (theFocus == NULL) return((VOID *) CurrentFocus);
   
   return((VOID *) ((struct focus *) theFocus)->next);
  }
  
/**************************************************************/
/* Focus: */
/**************************************************************/
globle VOID Focus(vTheModule)
  VOID *vTheModule;
  {
   struct defmodule *theModule = (struct defmodule *) vTheModule;
   struct focus *tempFocus;
   
   SetCurrentModule((VOID *) theModule);
   if (CurrentFocus != NULL)
     { if (CurrentFocus->theModule == theModule) return; }
  
#if DEBUGGING_FUNCTIONS   
   if (WatchFocus)
     {
      PrintCLIPS(WTRACE,"==> Focus ");
      PrintCLIPS(WTRACE,ValueToString(theModule->name));
      if (CurrentFocus != NULL)
        {
         PrintCLIPS(WTRACE," from ");
         PrintCLIPS(WTRACE,ValueToString(CurrentFocus->theModule->name));
        }
      PrintCLIPS(WTRACE,"\n");
     }
#endif
   
   tempFocus = get_struct(focus);
   tempFocus->theModule = theModule;
   tempFocus->theDefruleModule = GetDefruleModuleItem(theModule);
   tempFocus->next = CurrentFocus;
   CurrentFocus = tempFocus;
   FocusChanged = CLIPS_TRUE;
  }

/************************************************/
/* ClearFocusStackCommand: CLIPS access routine */
/*   for the clear-focus-stack command.         */
/************************************************/
globle VOID ClearFocusStackCommand()
  {
   if (ArgCountCheck("list-focus-stack",EXACTLY,0) == -1) return;

   ClearFocusStack();
  }
  
/****************************************/
/* ClearFocusStack: C access routine    */
/*   for the clear-focus-stack command. */
/****************************************/
globle VOID ClearFocusStack()
  {
   while (CurrentFocus != NULL) PopFocus();
     
   FocusChanged = CLIPS_TRUE;
  }
  
/**************************************************************/
/* AddRunFunction: Adds a function to the ListOfRunFunctions. */
/**************************************************************/
globle BOOLEAN AddRunFunction(name,functionPtr,priority)
  char *name;
#if ANSI_COMPILER
   VOID (*functionPtr)(void);
#else
   VOID (*functionPtr)();
#endif
  int priority;
  {
   struct runFunction *newPtr, *currentPtr, *lastPtr = NULL;

   newPtr = get_struct(runFunction);

   newPtr->name = name;
   newPtr->ip = functionPtr;
   newPtr->priority = priority;

   if (ListOfRunFunctions == NULL)
     {
      newPtr->next = NULL;
      ListOfRunFunctions = newPtr;
      return(1);
     }

   currentPtr = ListOfRunFunctions;
   while ((currentPtr != NULL) ? (priority < currentPtr->priority) : CLIPS_FALSE)
     {
      lastPtr = currentPtr;
      currentPtr = currentPtr->next;
     }

   if (lastPtr == NULL)
     {
      newPtr->next = ListOfRunFunctions;
      ListOfRunFunctions = newPtr;
     }
   else
     {
      newPtr->next = currentPtr;
      lastPtr->next = newPtr;
     }

   return(1);
  }

/**********************************************************************/
/* RemoveRunFunction: Removes a function from the ListOfRunFunctions. */
/**********************************************************************/
globle BOOLEAN RemoveRunFunction(name)
  char *name;
  {
   struct runFunction *currentPtr, *lastPtr;

   lastPtr = NULL;
   currentPtr = ListOfRunFunctions;

   while (currentPtr != NULL)
     {
      if (strcmp(name,currentPtr->name) == 0)
        {
         if (lastPtr == NULL)
           { ListOfRunFunctions = currentPtr->next; }
         else
           { lastPtr->next = currentPtr->next; }
         rtn_struct(runFunction,currentPtr);
         return(1);
        }
      lastPtr = currentPtr;
      currentPtr = currentPtr->next;
     }

   return(0);
  }

/*****************************************************************************/
/* InitializeEngine: Initializes the activations and statistics watch items. */
/*****************************************************************************/
globle VOID InitializeEngine()
  {
#if DEBUGGING_FUNCTIONS
   AddWatchItem("statistics",0,&WatchStatistics,20,NULL,NULL);
   AddWatchItem("focus",0,&WatchFocus,0,NULL,NULL);
#endif
  }
  
/*********************************************************/
/* RunCommand: CLIPS access routine for the run command. */
/*********************************************************/
globle VOID RunCommand()
  {
   int numArgs;
   long int runLimit = -1;
   DATA_OBJECT argPtr;

   if ((numArgs = ArgCountCheck("run",NO_MORE_THAN,1)) == -1) return;

   if (numArgs == 0)
     { runLimit = -1; }
   else if (numArgs == 1)
     {
      if (ArgTypeCheck("run",1,INTEGER,&argPtr) == CLIPS_FALSE) return;
      runLimit = DOToLong(argPtr);
     }

   Run(runLimit);

   return;
  }
  
/***********************************************/
/* HaltCommand: Causes rule execution to halt. */
/***********************************************/
globle VOID HaltCommand()
  {
   ArgCountCheck("halt",EXACTLY,0);
   HaltRules = CLIPS_TRUE;
  }
  
#if DEBUGGING_FUNCTIONS

/*********************************************************/
/* SetBreak: C access routine for the set-break command. */
/*********************************************************/
globle VOID SetBreak(theRule)
  VOID *theRule;
  {
   struct defrule *thePtr;

   thePtr = (struct defrule *) theRule;
   while (thePtr != NULL)
     {
      thePtr->afterBreakpoint = 1;
      thePtr = thePtr->disjunct;
     }
  }

/****************************************************************/
/* RemoveBreak: C access routine for the remove-break command. */
/****************************************************************/
globle BOOLEAN RemoveBreak(theRule)
  VOID *theRule;
  {
   struct defrule *thePtr;
   int rv = CLIPS_FALSE;

   thePtr = (struct defrule *) theRule;
   while (thePtr != NULL)
     {
      if (thePtr->afterBreakpoint == 1)
        {
         thePtr->afterBreakpoint = 0;
         rv = CLIPS_TRUE;
        }
      thePtr = thePtr->disjunct;
     }

   return(rv);
  }

/**************************************************/
/* RemoveAllBreakpoints: Removes all breakpoints. */
/**************************************************/
globle VOID RemoveAllBreakpoints()
  {
   VOID *theRule;
   VOID *theDefmodule = NULL;
   
   while ((theDefmodule = GetNextDefmodule(theDefmodule)) != NULL)
     {
      theRule = NULL;
      while ((theRule = GetNextDefrule(theRule)) != NULL)
        { RemoveBreak(theRule); }
     }
  }

/*************************************************************/
/* ShowBreaks: C access routine for the show-breaks command. */
/*************************************************************/
globle VOID ShowBreaks(logicalName,vTheModule)
  char *logicalName;
  VOID *vTheModule;
  {  
   ListItemsDriver(logicalName,(struct defmodule *) vTheModule,
                   NULL,NULL,
#if ANSI_COMPILER
                   GetNextDefrule,(char *(*)(VOID *)) GetConstructNameString,
#else
                   GetNextDefrule,(char *(*)()) GetConstructNameString,
#endif
                   NULL,DefruleHasBreakpoint);
   }

/************************************************************************************/
/* DefruleHasBreakpoint: Indicates whether the specified rule has a breakpoint set. */
/************************************************************************************/
globle BOOLEAN DefruleHasBreakpoint(theRule)
  VOID *theRule;
  {
   return(((struct defrule *) theRule)->afterBreakpoint);
  }

/*****************************************/
/* SetBreakCommand: CLIPS access routine */
/*   for the set-break command.          */
/*****************************************/
globle VOID SetBreakCommand()
  {
   DATA_OBJECT argPtr;
   char *argument;
   VOID *defrulePtr;

   if (ArgCountCheck("set-break",EXACTLY,1) == -1) return;

   if (ArgTypeCheck("set-break",1,SYMBOL,&argPtr) == CLIPS_FALSE) return;

   argument = DOToString(argPtr);

   if ((defrulePtr = FindDefrule(argument)) == NULL)
     {
      CantFindItemErrorMessage("defrule",argument);
      return;
     }

   SetBreak(defrulePtr);
  }

/********************************************/
/* RemoveBreakCommand: CLIPS access routine */
/*   for the remove-break command.          */
/********************************************/
globle VOID RemoveBreakCommand()
  {
   DATA_OBJECT argPtr;
   char *argument;
   int nargs;
   VOID *defrulePtr;

   if ((nargs = ArgCountCheck("remove-break",NO_MORE_THAN,1)) == -1)
     { return; }

   if (nargs == 0)
     {
      RemoveAllBreakpoints();
      return;
     }

   if (ArgTypeCheck("remove-break",1,SYMBOL,&argPtr) == CLIPS_FALSE) return;

   argument = DOToString(argPtr);

   if ((defrulePtr = FindDefrule(argument)) == NULL)
     {
      CantFindItemErrorMessage("defrule",argument);
      return;
     }

   if (RemoveBreak(defrulePtr) == CLIPS_FALSE)
     {
      PrintCLIPS(WERROR,"Rule ");
      PrintCLIPS(WERROR,argument);
      PrintCLIPS(WERROR," does not have a breakpoint set.\n");
     }
  }

/*******************************************/
/* ShowBreaksCommand: CLIPS access routine */
/*   for the show-breaks command.          */
/*******************************************/
globle VOID ShowBreaksCommand()
  {
   int numArgs, error;
   struct defmodule *theModule;
   
   if ((numArgs = ArgCountCheck("show-breaks",NO_MORE_THAN,1)) == -1) return;

   if (numArgs == 1)
     {
      theModule = GetModuleName("show-breaks",1,&error);
      if (error) return;
     }
   else
     { theModule = ((struct defmodule *) GetCurrentModule()); }

   ShowBreaks(WDISPLAY,theModule);
  }
  
/***********************************************/
/* ListFocusStackCommand: CLIPS access routine */
/*   for the list-focus-stack command.         */
/***********************************************/
globle VOID ListFocusStackCommand()
  {
   if (ArgCountCheck("list-focus-stack",EXACTLY,0) == -1) return;

   ListFocusStack(WDISPLAY);
  }
  
/****************************************/
/* ListFocusStack: C access routine for */
/*   the list-focus-stack command.      */
/****************************************/
globle VOID ListFocusStack(logicalName)
  char *logicalName;
  {
   struct focus *theFocus;
        
   for (theFocus = CurrentFocus;
        theFocus != NULL;
        theFocus = theFocus->next)
     {
      PrintCLIPS(logicalName,GetDefmoduleName(theFocus->theModule));
      PrintCLIPS(logicalName,"\n");
     }
  }
  
#endif

/***********************************************/
/* GetFocusStackFunction: CLIPS access routine */
/*   for the get-focus-stack function.         */
/***********************************************/
globle VOID GetFocusStackFunction(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   if (ArgCountCheck("get-focus-stack",EXACTLY,0) == -1) return;

   GetFocusStack(returnValue);
  }
  
/***************************************/
/* GetFocusStack: C access routine for */
/*   the get-focus-stack function.     */
/***************************************/
globle VOID GetFocusStack(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   struct focus *theFocus;
   struct multifield *theList;
   int count = 0;
      
   if (CurrentFocus == NULL)
     {
      SetpType(returnValue,MULTIFIELD);
      SetpDOBegin(returnValue,1);
      SetpDOEnd(returnValue,0);
      SetpValue(returnValue,(VOID *) CreateMultifield(0));
      return;
     }
  
   for (theFocus = CurrentFocus; theFocus != NULL; theFocus = theFocus->next)
     { count++; }
   
   SetpType(returnValue,MULTIFIELD);
   SetpDOBegin(returnValue,1);
   SetpDOEnd(returnValue,count);
   theList = (struct multifield *) CreateMultifield((int) count);
   SetpValue(returnValue,(VOID *) theList);
   
   for (theFocus = CurrentFocus, count = 1; 
        theFocus != NULL; 
        theFocus = theFocus->next, count++)
     { 
      SetMFType(theList,count,SYMBOL);
      SetMFValue(theList,count,theFocus->theModule->name);
     }
  }
  
/******************************************/
/* PopFocusFunction: CLIPS access routine */
/*   for the pop-focus function.          */
/******************************************/
globle SYMBOL_HN *PopFocusFunction()
  {
   struct defmodule *theModule;
   
   ArgCountCheck("pop-focus",EXACTLY,0);
   
   theModule = (struct defmodule *) PopFocus();
   if (theModule == NULL) return((SYMBOL_HN *) CLIPSFalseSymbol);
   return(theModule->name);
  }

/******************************************/
/* GetFocusFunction: CLIPS access routine */
/*   for the get-focus function.          */
/******************************************/
globle SYMBOL_HN *GetFocusFunction()
  {
   struct defmodule *rv;
   
   ArgCountCheck("get-focus",EXACTLY,0);
   rv = (struct defmodule *) WRGetFocus(); 
   if (rv == NULL) return((SYMBOL_HN *) CLIPSFalseSymbol);  
   return(rv->name);
  }
  
/**********************************/
/* GetFocus: C access routine for */
/*   the get-focus function.      */
/**********************************/
globle VOID *WRGetFocus()
  {
   if (CurrentFocus == NULL) return(NULL);
   
   return((VOID *) CurrentFocus->theModule);
  }
  
/*****************************************************/
/* FocusCommand: Implements the CLIPS focus command. */
/*****************************************************/
globle int FocusCommand()
  {
   DATA_OBJECT argPtr;
   char *argument;
   struct defmodule *theModule;
   int argCount, i;

   /*=====================================================*/
   /* Check for the correct number and type of arguments. */
   /*=====================================================*/
      
   if ((argCount = ArgCountCheck("focus",AT_LEAST,1)) == -1)
     { return(CLIPS_FALSE); }

   /*===========================================*/
   /* Focus on the specified defrule module(s). */
   /*===========================================*/
   
   for (i = argCount; i > 0; i--)
     {
      if (ArgTypeCheck("focus",i,SYMBOL,&argPtr) == CLIPS_FALSE)
        { return(CLIPS_FALSE); }

      argument = DOToString(argPtr);
      theModule = (struct defmodule *) FindDefmodule(argument);
   
      if (theModule == NULL)
        {
         CantFindItemErrorMessage("defmodule",argument);
         return(CLIPS_FALSE);
        }
     
      Focus((VOID *) theModule);
     }
   
   /*===================================================*/
   /* Return TRUE to indicate success of focus command. */
   /*===================================================*/
   
   return(CLIPS_TRUE);
  }
  
/********************************************************************/
/* GetFocusChanged: Returns the value of the variable FocusChanged. */
/********************************************************************/
globle int GetFocusChanged()
  { 
   return(FocusChanged);
  }

/*****************************************************************/
/* SetFocusChanged: Sets the value of the variable FocusChanged. */
/*****************************************************************/
globle VOID SetFocusChanged(value)
  int value;
  {
   FocusChanged = value;
  }

#endif

