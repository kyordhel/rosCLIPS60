   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 RULE PARSING MODULE                 */
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

#define _RULEPSR_SOURCE_

#include "setup.h"

#if DEFRULE_CONSTRUCT

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "constant.h"
#include "clipsmem.h"
#include "symbol.h"
#include "scanner.h"
#include "router.h"
#include "engine.h"
#include "rulelhs.h"
#include "ruledef.h"
#include "rulebld.h"  
#include "pattern.h"
#include "cstrnchk.h"
#include "cstrnops.h"
#include "exprnpsr.h"
#include "analysis.h"
#include "prcdrpsr.h"
#include "constrct.h"
#include "prccode.h"
#include "incrrset.h"
#include "rulecstr.h"
#include "watch.h"
#include "ruledlt.h"
#include "cstrcpsr.h"
#include "rulebsc.h"

#if LOGICAL_DEPENDENCIES
#include "lgcldpnd.h"
#endif

#if DEFTEMPLATE_CONSTRUCT
#include "tmpltfun.h"
#endif

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
#include "bload.h"
#endif

#include "rulepsr.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if (! RUN_TIME) && (! BLOAD_ONLY)
#if ANSI_COMPILER
   static struct expr            *ParseRuleRHS(char *);
   static int                     ReplaceRHSVariable(struct expr *,VOID *);
   static struct defrule         *ProcessRuleLHS(struct lhsParseNode *,struct expr *,SYMBOL_HN *);
   static struct defrule         *CreateNewDisjunct(SYMBOL_HN *,int,struct expr *,
                                                    int,int,struct joinNode *);
   static int                     RuleComplexity(struct lhsParseNode *);
   static int                     ExpressionComplexity(struct expr *);
#if LOGICAL_DEPENDENCIES
   static int                     LogicalAnalysis(struct lhsParseNode *);
#endif
   static VOID                    AddToDefruleList(struct defrule *);
#else
   static struct expr            *ParseRuleRHS();
   static int                     ReplaceRHSVariable();
   static struct defrule         *ProcessRuleLHS();
   static struct defrule         *CreateNewDisjunct();
   static int                     RuleComplexity();
   static int                     ExpressionComplexity();
#if LOGICAL_DEPENDENCIES
   static int                     LogicalAnalysis();
#endif
   static VOID                    AddToDefruleList();
#endif
#endif

/*******************************************************************/
/* ParseDefrule: Coordinates all actions necessary for the parsing */
/*   and creation of a defrule into the current environment.       */
/*******************************************************************/
globle int ParseDefrule(readSource)
  char *readSource;
  {
#if (! RUN_TIME) && (! BLOAD_ONLY)
   SYMBOL_HN *ruleName;
   struct lhsParseNode *theLHS;
   struct expr *actions;
   struct token theToken;
   struct defrule *topDisjunct = NULL, *tempPtr;
   struct defruleModule *theModuleItem;

   /*================================================*/
   /* Flush the buffer which stores the pretty print */
   /* representation for a rule.  Add the already    */
   /* parsed keyword defrule to this buffer.         */
   /*================================================*/

   SetPPBufferStatus(ON);
   FlushPPBuffer();
   SavePPBuffer("(defrule ");

   /*=========================================================*/
   /* Rules cannot be loaded when a binary load is in effect. */
   /*=========================================================*/

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
   if (Bloaded() == CLIPS_TRUE)
     {      
      CannotLoadWithBloadMessage("defrule");
      return(CLIPS_TRUE);
     }
#endif

   /*================================================*/
   /* Parse the name and comment fields of the rule, */
   /* deleting the rule if it already exists.        */
   /*================================================*/

#if DEBUGGING_FUNCTIONS
   DeletedRuleDebugFlags = 0;
#endif 

   ruleName = GetConstructNameAndComment(readSource,&theToken,"defrule",
                                         FindDefrule,Undefrule,"*",CLIPS_FALSE,
                                         CLIPS_TRUE,CLIPS_TRUE);
                                         
   if (ruleName == NULL) return(CLIPS_TRUE);
   
   /*============================*/
   /* Parse the LHS of the rule. */
   /*============================*/

   theLHS = ParseRuleLHS(readSource,&theToken,ValueToString(ruleName));
   if (theLHS == NULL)
     {
      ReturnPackedExpression(SalienceExpression);
      SalienceExpression = NULL;
      return(CLIPS_TRUE);
     }
     
#if ! DYNAMIC_SALIENCE
   ReturnPackedExpression(SalienceExpression);
   SalienceExpression = NULL;
#endif

   /*============================*/
   /* Parse the RHS of the rule. */
   /*============================*/

   ClearParsedBindNames();
   ReturnContext = CLIPS_TRUE;
   actions = ParseRuleRHS(readSource);
   
   if (actions == NULL)
     {
      ReturnPackedExpression(SalienceExpression);
      SalienceExpression = NULL;
      ReturnLHSParseNodes(theLHS);
      return(CLIPS_TRUE);
     }

   /*=======================*/
   /* Process the rule LHS. */
   /*=======================*/

   topDisjunct = ProcessRuleLHS(theLHS,actions,ruleName);
   
   ReturnExpression(actions);
   ClearParsedBindNames();
   ReturnLHSParseNodes(theLHS);
   
   if (topDisjunct == NULL)
     {
      ReturnPackedExpression(SalienceExpression);
      SalienceExpression = NULL;
      return(CLIPS_TRUE);
     }
     
   SalienceExpression = NULL;
     
   /*======================================*/
   /* Save the nice printout of the rules. */
   /*======================================*/

   SavePPBuffer("\n");
   if (GetConserveMemory() == CLIPS_TRUE)
     { topDisjunct->header.ppForm = NULL; }
   else
     { topDisjunct->header.ppForm = CopyPPBuffer(); }

   /*=======================================*/
   /* Store a pointer to the rule's module. */
   /*=======================================*/
   
   theModuleItem = (struct defruleModule *)
                   GetModuleItem(NULL,FindModuleItem("defrule")->moduleIndex);
                   
   for (tempPtr = topDisjunct; tempPtr != NULL; tempPtr = tempPtr->disjunct)
     { tempPtr->header.whichModule = (struct defmoduleItemHeader *) theModuleItem; }
       
   /*===============================================*/
   /* Rule completely parsed. Add to list of rules. */
   /*===============================================*/

   AddToDefruleList(topDisjunct);

   /*========================================================================*/
   /* If a rule is redefined, then we want to restore its breakpoint status. */
   /*========================================================================*/

#if DEBUGGING_FUNCTIONS
   if (BitwiseTest(DeletedRuleDebugFlags,0)) SetBreak(topDisjunct);
   if (BitwiseTest(DeletedRuleDebugFlags,1) || GetWatchItem("rules"))
     SetDefruleWatchFirings(ON,(VOID *) topDisjunct);
     if (BitwiseTest(DeletedRuleDebugFlags,2) || GetWatchItem("activations"))
     SetDefruleWatchActivations(ON,(VOID *) topDisjunct);
#endif

   /*================================*/
   /* Perform the incremental reset. */
   /*================================*/

#if INCREMENTAL_RESET   
   IncrementalReset(topDisjunct);
#endif

   /*=============================================*/
   /* Return FALSE to indicate no errors occured. */
   /*=============================================*/

#endif   
   return(CLIPS_FALSE);
  }   
  
#if (! RUN_TIME) && (! BLOAD_ONLY)

/**************************************************************/
/* ProcessRuleLHS: Processes each of the disjuncts of a rule. */
/**************************************************************/
static struct defrule *ProcessRuleLHS(theLHS,actions,ruleName)
  struct lhsParseNode *theLHS;
  struct expr *actions;
  SYMBOL_HN *ruleName;
  {
   struct lhsParseNode *tempNode = NULL;
   struct defrule *topDisjunct = NULL, *currentDisjunct, *lastDisjunct = NULL;
   struct expr *newActions, *packPtr;
   int logicalJoin = 0;
   int localVarCnt;
   int complexity;
   struct joinNode *lastJoin;

   /*===========================================================*/
   /* The top level of the construct representing the LHS of a  */
   /* rule is assumed to be an OR.  If the implied OR is at the */
   /* top level of the pattern construct, then remove it.       */
   /*===========================================================*/

   if (theLHS->type == OR_CE) theLHS = theLHS->right;

   /*=========================================*/
   /* Loop through each disjunct of the rule. */
   /*=========================================*/
   
   localVarCnt = CountParsedBindNames();
   while (theLHS != NULL)
     {
      /*===================================*/
      /* Analyze the LHS of this disjunct. */
      /*===================================*/
      
      if (theLHS->type == AND_CE) tempNode = theLHS->right; 
      else if (theLHS->type == PATTERN_CE) tempNode = theLHS;

      if (VariableAnalysis(tempNode))
        {
         ReturnDefrule(topDisjunct);
         return(NULL);
        }
        
      /*=========================================*/
      /* Perform entity dependent post analysis. */
      /*=========================================*/
     
      if (PostPatternAnalysis(tempNode))
        {
         ReturnDefrule(topDisjunct);
         return(NULL);
        }
        
      /*========================================================*/
      /* Print out developer information if it's being watched. */
      /*========================================================*/

#if DEVELOPER
      if (GetWatchItem("rule-analysis"))
        {      
         struct lhsParseNode *traceNode;
         char buffer[20];
         
         PrintCLIPS(WDISPLAY,"\n");
         for (traceNode = tempNode; traceNode != NULL; traceNode = traceNode->bottom)
           {
            if (traceNode->userCE)
              {
               sprintf(buffer,"CE %2d: ",traceNode->whichCE);
               PrintCLIPS(WDISPLAY,buffer);
               PrintExpression(WDISPLAY,traceNode->networkTest);
               PrintCLIPS(WDISPLAY,"\n");
              }
           }
        }
#endif
    
      /*========================================*/
      /* Check to see that logical CEs are used */
      /* appropriately in the LHS of the rule.  */
      /*========================================*/

#if LOGICAL_DEPENDENCIES
      if ((logicalJoin = LogicalAnalysis(tempNode)) < 0) 
        {
         ReturnDefrule(topDisjunct);
         return(NULL);
        }
#endif
      
      /*======================================================*/
      /* Check to see if there are any RHS constraint errors. */
      /*======================================================*/
      
      if (CheckRHSForConstraintErrors(actions,tempNode))
        {
         ReturnDefrule(topDisjunct);
         return(NULL);
        }
      
      /*=================================================*/
      /* Replace variable references in the RHS with the */
      /* appropriate variable retrieval functions.       */
      /*=================================================*/
      
      newActions = CopyExpression(actions);
      if (ReplaceProcVars("RHS of defrule",newActions,NULL,NULL,
                          ReplaceRHSVariable,(VOID *) tempNode))
        {
         ReturnDefrule(topDisjunct);
         ReturnExpression(newActions);
         return(NULL);
        }
        
      ExpressionInstall(newActions);
      packPtr = PackExpression(newActions);
      ReturnExpression(newActions);
           
      /*===============================================================*/
      /* Create the pattern and join data structures for the new rule. */
      /*===============================================================*/
      
      lastJoin = ConstructJoins(logicalJoin,tempNode);
      
      /*===================================================================*/
      /* Determine the rule's complexity for use with conflict resolution. */
      /*===================================================================*/
      
      complexity = RuleComplexity(tempNode);
      
      /*=====================================================*/
      /* Create the defrule data structure for this disjunct */
      /* and put it in the list of disjuncts for this rule.  */
      /*=====================================================*/
      
      currentDisjunct = CreateNewDisjunct(ruleName,localVarCnt,packPtr,complexity,
                                          logicalJoin,lastJoin);
                                          
      /*============================================================*/
      /* Place the disjunct in the list of disjuncts for this rule. */
      /* If the disjunct is the first disjunct, then increment the  */
      /* reference counts for the dynamic salience (the expression  */
      /* for the dynamic salience is only stored with the first     */
      /* disjuncts and the other disjuncts refer back to the first  */
      /* disjunct for their dynamic salience value.                 */
      /*============================================================*/                                     

      if (topDisjunct == NULL) 
        {
         topDisjunct = currentDisjunct;
#if DYNAMIC_SALIENCE
         ExpressionInstall(topDisjunct->dynamicSalience);
#endif
        }
      else lastDisjunct->disjunct = currentDisjunct;

      /*===========================================*/
      /* Move on to the next disjunct of the rule. */
      /*===========================================*/
      
      theLHS = theLHS->bottom;
      lastDisjunct = currentDisjunct;
     }

   return(topDisjunct);
  }
  
/************************************************************************/
/* CreateNewDisjunct: Creates and initializes a defrule data structure. */
/************************************************************************/
static struct defrule *CreateNewDisjunct(ruleName,localVarCnt,theActions,complexity,
                                         logicalJoin,lastJoin)
  SYMBOL_HN *ruleName;
  int localVarCnt;
  struct expr *theActions;
  int complexity;
  int logicalJoin;
  struct joinNode *lastJoin;
  {
   struct defrule *newDisjunct;
#if LOGICAL_DEPENDENCIES
   struct joinNode *tempJoin;
#endif

   /*===================================================*/
   /* Create and initialize the defrule data structure. */
   /*===================================================*/
   
   newDisjunct = get_struct(defrule);
   newDisjunct->header.ppForm = NULL;
   newDisjunct->header.next = NULL;
#if LOGICAL_DEPENDENCIES
   newDisjunct->logicalJoin = NULL;
#endif
   newDisjunct->disjunct = NULL;
   newDisjunct->header.name = ruleName;
   IncrementSymbolCount(newDisjunct->header.name);
   newDisjunct->actions = theActions;
   newDisjunct->salience = GlobalSalience;
   newDisjunct->afterBreakpoint = 0;
   newDisjunct->watchActivation = 0;
   newDisjunct->watchFiring = 0;
   newDisjunct->executing = 0;
   newDisjunct->complexity = complexity;
   newDisjunct->autoFocus = GlobalAutoFocus;
#if DYNAMIC_SALIENCE
   newDisjunct->dynamicSalience = SalienceExpression;
#endif
   newDisjunct->localVarCnt = localVarCnt;

   /*============================================================*/
   /* Attach the rule's last join to the defrule data structure. */
   /*============================================================*/
   
   lastJoin->ruleToActivate = newDisjunct;
   newDisjunct->lastJoin = lastJoin;
   
   /*=================================================*/
   /* Determine the rule's logical join if it exists. */
   /*=================================================*/
   
#if LOGICAL_DEPENDENCIES
   tempJoin = lastJoin;
   while (tempJoin != NULL)
     {
      if (tempJoin->depth == logicalJoin)
        {
         newDisjunct->logicalJoin = tempJoin;
         tempJoin->logicalJoin = CLIPS_TRUE;
        }
      tempJoin = tempJoin->lastLevel;
     }
#endif

   /*==================================================*/
   /* Return the newly created defrule data structure. */
   /*==================================================*/
   
   return(newDisjunct);
  }

/*****************************************************************************/
/* ReplaceExpressionVariables: Replaces all symbolic references to variables */
/*   (local and global) found in an expression on the RHS of a rule with     */
/*   expressions containing function calls to retrieve the variable's value. */
/*   Makes the final modifications necessary for handling the modify and     */
/*   duplicate commands.                                                     */
/*****************************************************************************/
static int ReplaceRHSVariable(list,VtheLHS)
  struct expr *list;
  VOID *VtheLHS;
  {
   struct lhsParseNode *theVariable;

   /*=======================================*/
   /* Handle modify and duplicate commands. */
   /*=======================================*/
   
#if DEFTEMPLATE_CONSTRUCT
   if (list->type == FCALL)
     {
      if (list->value == (VOID *) FindFunction("modify"))
        {
         if (UpdateModifyDuplicate(list,"modify",VtheLHS) == CLIPS_FALSE)
           return(-1);
        }
      else if (list->value == (VOID *) FindFunction("duplicate"))
        {
         if (UpdateModifyDuplicate(list,"duplicate",VtheLHS) == CLIPS_FALSE)
           return(-1);
        }
        
      return(0);
     }
     
#endif

   if ((list->type != SF_VARIABLE) && (list->type != MF_VARIABLE))
     { return(CLIPS_FALSE); }
   
   /*===============================================================*/
   /* Check to see if the variable is bound on the LHS of the rule. */
   /*===============================================================*/

   theVariable = FindVariable((SYMBOL_HN *) list->value,(struct lhsParseNode *) VtheLHS);
   if (theVariable == NULL) return(CLIPS_FALSE);
     
   /*================================================*/
   /* Replace the variable reference with a function */
   /* call to retrieve the variable.                 */
   /*================================================*/
   
   if (theVariable->patternType != NULL) 
     { (*theVariable->patternType->replaceGetJNValueFunction)(list,theVariable); }
   else
     { return(CLIPS_FALSE); }
   
   /*=================================================================*/
   /* Return TRUE to indicate the variable was successfully replaced. */
   /*=================================================================*/
   
   return(CLIPS_TRUE);
  }
  
/*******************************************************/
/* ParseRuleRHS: Coordinates all the actions necessary */
/*   for parsing the RHS of a rule.                    */
/*******************************************************/
static struct expr *ParseRuleRHS(readSource)
  char *readSource;
  {
   struct expr *actions;
   struct token theToken;

   /*=========================================================*/
   /* Process the actions on the right hand side of the rule. */
   /*=========================================================*/

   SavePPBuffer("\n   ");
   SetIndentDepth(3);

   actions = GroupActions(readSource,&theToken,CLIPS_TRUE,NULL);

   if (actions == NULL) return(NULL);

   /*=============================*/
   /* Reformat the closing token. */
   /*=============================*/

   PPBackup();
   PPBackup();
   SavePPBuffer(theToken.printForm);

   /*======================================================*/
   /* Check for the closing right parenthesis of the rule. */
   /*======================================================*/

   if (theToken.type != RPAREN)
     {
      SyntaxErrorMessage("defrule");
      ReturnExpression(actions);
      return(NULL);
     }

   /*========================*/
   /* Return the rule's RHS. */
   /*========================*/
   
   return(actions);
  }
  
/************************************************************/
/* RuleComplexity: Returns the complexity of a rule for use */
/*   by the LEX and MEA conflict resolution strategies.     */
/************************************************************/
static int RuleComplexity(theLHS)
  struct lhsParseNode *theLHS;
  {
   struct lhsParseNode *thePattern, *tempPattern;
   int complexity = 0;

   while (theLHS != NULL)
     {
      complexity += 1; /* Add 1 for each pattern. */
      complexity += ExpressionComplexity(theLHS->networkTest);
      thePattern = theLHS->right;
      while (thePattern != NULL)
        {
         if (thePattern->multifieldSlot)
           {
            tempPattern = thePattern->bottom;
            while (tempPattern != NULL)
              {
               complexity += ExpressionComplexity(tempPattern->networkTest);
               tempPattern = tempPattern->right;
              }
           }
         else
           { complexity += ExpressionComplexity(thePattern->networkTest); }
         thePattern = thePattern->right;
        }
      theLHS = theLHS->bottom;
     }

   return(complexity);
  }
  
/********************************************************************/
/* ExpressionComplexity: Determines the complexity of a expression. */
/********************************************************************/
static int ExpressionComplexity(exprPtr)
  struct expr *exprPtr;
  {
   int complexity = 0;

   while (exprPtr != NULL)
     {
      if (exprPtr->type == FCALL)
        {
         /*=========================================*/
         /* Logical combinations do not add to the  */
         /* complexity, but their arguments do.     */
         /*=========================================*/

         if ((exprPtr->value == PTR_AND) ||
                  (exprPtr->value == PTR_NOT) ||
                  (exprPtr->value == PTR_OR))
           { complexity += ExpressionComplexity(exprPtr->argList); }

         /*=========================================*/
         /* else other function calls increase the  */
         /* complexity, but their arguments do not. */
         /*=========================================*/

         else
           { complexity++; }
        }
      else if ((PrimitivesArray[exprPtr->type] != NULL) ?
               PrimitivesArray[exprPtr->type]->addsToRuleComplexity : CLIPS_FALSE)
        { complexity++; }

      exprPtr = exprPtr->nextArg;
     }

   return(complexity);
  }
  
#if LOGICAL_DEPENDENCIES

/***************************************************************/
/* LogicalAnalysis:  Analyzes the use of the logical construct */
/*   within the LHS of a rule.                                 */
/***************************************************************/
static int LogicalAnalysis(patternList)
  struct lhsParseNode *patternList;
  {
   int firstLogical, logicalsFound = CLIPS_FALSE, logicalJoin = 0;
   int gap = CLIPS_FALSE;

   firstLogical = patternList->logical;

   while (patternList != NULL)
     {
      if ((patternList->type == PATTERN_CE) && (patternList->endNandDepth == 1))
        {
         if (patternList->logical)
           {
            if (! firstLogical)
              {
               PrintErrorID("RULEPSR",1,CLIPS_TRUE);
               PrintCLIPS(WERROR,"Logical CEs must be placed first in a rule\n");
               return(-1);
              }

            if (gap)
              {
               PrintErrorID("RULEPSR",2,CLIPS_TRUE);
               PrintCLIPS(WERROR,"Gaps may not exist between logical CEs\n");
               return(-1);
              }

            logicalJoin++;
            logicalsFound = CLIPS_TRUE;
           }
         else gap = CLIPS_TRUE;
        }

      patternList = patternList->bottom;
     }

   if (logicalsFound) return(logicalJoin);

   return(0);
  }

#endif

/*****************************************************************/
/* FindVariable: Searches for the last occurence of a variable   */
/*  in the LHS of a rule that is visible from the RHS of a rule. */
/*  The last occurence of the variable on the LHS side of the    */
/*  rule will have the strictest constraints (because it will    */
/*  have been intersected with all of the other constraints for  */
/*  the variable on the LHS of the rule). The strictest          */
/*  constraints are useful for performing type checking on the   */
/*  RHS of the rule.                                             */
/*****************************************************************/
globle struct lhsParseNode *FindVariable(name,theLHS)
  SYMBOL_HN *name;
  struct lhsParseNode *theLHS;
  {
   struct lhsParseNode *theFields, *tmpFields = NULL;
   struct lhsParseNode *theReturnValue = NULL;
      
   while (theLHS != NULL)
     {
      if ((theLHS->type == PATTERN_CE) && 
          (theLHS->negated == CLIPS_FALSE) && 
          (theLHS->beginNandDepth == 1))
        {
         if (theLHS->value == (VOID *) name)
           { theReturnValue = theLHS; }
           
         theFields = theLHS->right;
         while (theFields != NULL)
           {
            if (theFields->multifieldSlot)
              {
               tmpFields = theFields;
               theFields = theFields->bottom;
              }
              
            if (theFields == NULL)
              { /* Do Nothing */ }
            else if (((theFields->type == SF_VARIABLE) || (theFields->type == MF_VARIABLE)) &&
                (theFields->value == (VOID *) name))
              { theReturnValue = theFields; }
              
            if (theFields == NULL)
              {
               theFields = tmpFields;
               tmpFields = NULL;
              }
            else if ((theFields->right == NULL) && (tmpFields != NULL))
              {
               theFields = tmpFields;
               tmpFields = NULL;
              }
            theFields = theFields->right;
           }
        }
        
      theLHS = theLHS->bottom;
     }
     
   return(theReturnValue);
  }
  
/***********************************************************/
/* AddToDefruleList: Adds a defrule to the ListOfDefrules. */
/***********************************************************/
static VOID AddToDefruleList(rulePtr)
  struct defrule *rulePtr;
  {
   struct defrule *tempRule; 
   struct defruleModule *theModuleItem;
   
   theModuleItem = (struct defruleModule *) rulePtr->header.whichModule;

   if (theModuleItem->header.lastItem == NULL)
     { theModuleItem->header.firstItem = (struct constructHeader *) rulePtr; }
   else
     {
      tempRule = (struct defrule *) theModuleItem->header.lastItem;
      while (tempRule != NULL)
        {
         tempRule->header.next = (struct constructHeader *) rulePtr;
         tempRule = tempRule->disjunct;
        }
     }

   theModuleItem->header.lastItem = (struct constructHeader *) rulePtr;
  }

  
#endif

#endif
