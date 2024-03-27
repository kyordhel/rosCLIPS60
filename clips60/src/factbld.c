   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  08/18/92            */
   /*                                                     */
   /*                   FACT BUILD MODULE                 */
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

#define _FACTBLD_SOURCE_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT && DEFRULE_CONSTRUCT

#include <stdio.h>
#define _CLIPS_STDIO_

#include "clipsmem.h"
#include "reteutil.h"
#include "router.h"
#include "reorder.h"
#include "factcmp.h"
#include "factmch.h"
#include "factgen.h"
#include "factmngr.h"
#include "factlhs.h"
#include "argacces.h"
#include "modulutl.h"
#include "tmpltdef.h"

#include "factbld.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
#if (! RUN_TIME) && (! BLOAD_ONLY)
   static struct factPatternNode    *FindPatternNode(struct factPatternNode *,struct lhsParseNode *,
                                                  struct factPatternNode **,int);
   static struct factPatternNode    *CreateNewPatternNode(struct lhsParseNode *,struct factPatternNode *,
                                                       struct factPatternNode *,int);
   static VOID                       ClearPatternMatches(struct factPatternNode *);
   static VOID                       DetachFactPattern(struct patternNodeHeader *);
   static struct patternNodeHeader  *PlaceFactPattern(struct lhsParseNode *);
   static struct lhsParseNode       *RemoveUnneededSlots(struct lhsParseNode *);
   static VOID                       FindAndSetDeftemplatePatternNetwork(struct factPatternNode *,struct factPatternNode *);
#endif
#else
#if (! RUN_TIME) && (! BLOAD_ONLY)
   static struct factPatternNode    *FindPatternNode();
   static struct factPatternNode    *CreateNewPatternNode();
   static VOID                       ClearPatternMatches();
   static VOID                       DetachFactPattern();
   static struct patternNodeHeader  *PlaceFactPattern();
   static struct lhsParseNode       *RemoveUnneededSlots();
   static VOID                       FindAndSetDeftemplatePatternNetwork();
#endif
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle struct patternEntityRecord  FactInfo = { { FACT_ADDRESS,1,0,0,
                                                     PrintFactIdentifier, 
                                                     PrintFactInLongForm,
                                                     Retract,
                                                     NULL,
                                                     GetNextFact,
                                                     IncrementFactCount,
                                                     DecrementFactCount,NULL,NULL,NULL,NULL
                                                   },
                                                   DecrementFactBasisCount,
                                                   IncrementFactBasisCount,
                                                   MatchFactFunction
                                                 };

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

#if (! RUN_TIME) && (! BLOAD_ONLY)
   static struct deftemplate       *CurrentDeftemplate;
#endif

/*********************************************************/
/* InitializeFactPatterns: Adds fact patterns to the set */
/*   of patterns recognized by the rule pattern parsing  */
/*   and pattern/join network integration routines.      */
/*********************************************************/
globle VOID InitializeFactPatterns()
  {
#if DEFRULE_CONSTRUCT
   InitializeFactReteFunctions();
   
#if (! RUN_TIME) && (! BLOAD_ONLY)   
   AddPatternParser("facts",0,&FactInfo,
                    FactPatternParserFind,
                    FactPatternParse,
                    NULL,PlaceFactPattern,
                    DetachFactPattern,
                    NULL,FactReplaceGetvar,
                    FactGenGetvar,FactJNVariableComparison,
                    FactGenPNConstant,FactReplaceGetfield,
                    FactGenGetfield,FactPNVariableComparison,
                    NULL,NULL,
#if INCREMENTAL_RESET
                    MarkFactPtnForIncrementalReset,
                    FactsIncrementalReset,
#else
                    NULL,NULL,
#endif
                    CreateInitialFactPattern,
#if CONSTRUCT_COMPILER
                    FactPatternNodeReference);
#else
                    NULL);
#endif

#else
   AddPatternParser("facts",0,&FactInfo,
                    NULL,NULL,NULL,NULL,NULL,NULL,NULL,
                    NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
#if INCREMENTAL_RESET
                    MarkFactPtnForIncrementalReset,
                    FactsIncrementalReset,
#else
                    NULL,NULL,
#endif
                    NULL,NULL);  
#endif
 
#endif
  }

#if (! RUN_TIME) && (! BLOAD_ONLY)
  
/******************************************************************************/
/* PlaceFactPattern: Integrates a fact pattern into the fact pattern network. */
/******************************************************************************/
static struct patternNodeHeader *PlaceFactPattern(thePattern)
  struct lhsParseNode *thePattern;
  {
   struct lhsParseNode *tempPattern = NULL;
   struct factPatternNode *currentLevel, *lastLevel;
   struct factPatternNode *nodeBeforeMatch, *newNode;
   int endSlot, count;
   char *theDeftemplate;
   
   theDeftemplate = ValueToString(thePattern->right->bottom->value);
   
   /*=====================================================*/
   /* Remove any slot tests that test only for existance. */
   /*=====================================================*/
   
   thePattern->right = RemoveUnneededSlots(thePattern->right);
   
   /*========================================================*/
   /* If the constant test for the relation name is the only */
   /* pattern network test and there are no other network    */
   /* tests, then remove the test, but keep the node since   */
   /* there must be a link from the fact pattern network to  */
   /* the join network. Otherwise, remove the test for the   */
   /* relation name since this test has already been done    */
   /* before entering the pattern network (since each        */
   /* deftemplate has its own pattern network.               */
   /*========================================================*/
   
   if (thePattern->right->right == NULL)
     {   
      ReturnExpression(thePattern->right->networkTest);
      thePattern->right->networkTest = NULL;
     }
   else
     {
      tempPattern = thePattern->right;
      thePattern->right = thePattern->right->right;
      tempPattern->right = NULL;
      ReturnLHSParseNodes(tempPattern);
     }
     
   /*==================================================*/
   /* Get the deftemplate associated with the pattern. */
   /*==================================================*/
   
   CurrentDeftemplate = (struct deftemplate *) 
       FindImportedConstruct("deftemplate",NULL,theDeftemplate,&count,CLIPS_TRUE,NULL);
   
   /*================================================*/
   /* Initialize some pointers to indicate where the */
   /* pattern is being added to the pattern network. */
   /*================================================*/
   
   currentLevel = CurrentDeftemplate->patternNetwork;
   lastLevel = NULL;
   thePattern = thePattern->right;
   
   /*===========================================*/
   /* Loop until all fields in the pattern have */ 
   /* been added to the pattern network.        */
   /*===========================================*/
   
   while (thePattern != NULL)
     {
      /*===========================================================*/
      /* If a multifield slot is being processed, then process the */
      /* pattern nodes attached to the multifield pattern node.    */
      /*===========================================================*/
      
      if (thePattern->multifieldSlot)
        {
         tempPattern = thePattern;
         thePattern = thePattern->bottom;
        }
        
      /*============================================*/
      /* Determine if the last pattern field within */
      /* a multifield slot is being processed.      */
      /*============================================*/
      
      if ((thePattern->right == NULL) && (tempPattern != NULL)) 
        { endSlot = CLIPS_TRUE; }
      else 
        { endSlot = CLIPS_FALSE; }
        
      /*========================================*/
      /* Is there a node in the pattern network */
      /* that can be reused (shared)?           */
      /*========================================*/
      
      newNode = FindPatternNode(currentLevel,thePattern,&nodeBeforeMatch,endSlot);
   
      /*================================================*/
      /* If the pattern node cannot be shared, then add */
      /* a new pattern node to the pattern network.     */
      /*================================================*/
      
      if (newNode == NULL)
        { newNode = CreateNewPatternNode(thePattern,nodeBeforeMatch,lastLevel,endSlot); }
      
      /*===========================================================*/
      /* Move on to the next field in the new pattern to be added. */
      /*===========================================================*/

      if ((thePattern->right == NULL) && (tempPattern != NULL))
        {
         thePattern = tempPattern;
         tempPattern = NULL;
        }
        
      thePattern = thePattern->right;
      
      /*=========================================================*/
      /* If there are no more pattern nodes to be added to the   */
      /* pattern network, then mark the last pattern node added  */
      /* as a stop node (i.e. if you get to this node and the    */
      /* network test succeeds, then a pattern has been matched. */
      /*=========================================================*/
      
      if (thePattern == NULL) newNode->header.stopNode = CLIPS_TRUE;
        
      /*================================================*/
      /* Update the pointers which indicate where we're */
      /* trying to add the new pattern to the currently */
      /* existing pattern network.                      */
      /*================================================*/
      
      lastLevel = newNode;
      currentLevel = newNode->nextLevel;
     }
     
   /*==================================================*/
   /* Return the leaf node of the newly added pattern. */
   /*==================================================*/
   
   return((struct patternNodeHeader *) newNode);
  }

/*************************************************************/
/* FindPatternNode: Looks for a pattern node at a specified  */
/*  level in the pattern network that can be reused (shared) */
/*  with a pattern field being added to the pattern network. */
/*************************************************************/
static struct factPatternNode *FindPatternNode(listOfNodes,thePattern,nodeBeforeMatch,
                                               endSlot)
  struct factPatternNode *listOfNodes;
  struct lhsParseNode *thePattern;
  struct factPatternNode **nodeBeforeMatch;
  int endSlot;
  {
   *nodeBeforeMatch = NULL;
   
   /*==========================================================*/
   /* Loop through the nodes at the given level in the pattern */
   /* network looking for a node that can be reused (shared)?  */
   /*==========================================================*/
   
   while (listOfNodes != NULL)
     {
      /*==========================================================*/
      /* If the type of the pattern node and the expression being */
      /* tested by the pattern node are the same as the type and  */
      /* expression for the pattern field being added, then       */
      /* return the pattern node because it can be shared with    */
      /* the pattern field being added.                           */
      /*==========================================================*/
      
      if ((thePattern->type == SF_WILDCARD) || (thePattern->type == SF_VARIABLE))
        { 
         if ((listOfNodes->header.singlefieldNode) && 
             (listOfNodes->header.endSlot == endSlot) &&
             (listOfNodes->whichField == thePattern->index) &&
             (listOfNodes->whichSlot == (thePattern->slotNumber - 1)) &&
             IdenticalExpression(listOfNodes->networkTest,thePattern->networkTest))
           { return(listOfNodes); } 
        }
      else if ((thePattern->type == MF_WILDCARD) || (thePattern->type == MF_VARIABLE))
        { 
         if ((listOfNodes->header.multifieldNode) && 
             (listOfNodes->header.endSlot == endSlot) &&
             (listOfNodes->leaveFields == thePattern->singleFieldsAfter) &&
             (listOfNodes->whichField == thePattern->index) &&
             (listOfNodes->whichSlot == (thePattern->slotNumber - 1)) &&
             IdenticalExpression(listOfNodes->networkTest,thePattern->networkTest))
           { return(listOfNodes); } 
        }

      /*==================================*/
      /* Move on to the next node at this */
      /* level in the pattern network.    */
      /*==================================*/
      
      *nodeBeforeMatch = listOfNodes;
      listOfNodes = listOfNodes->rightNode;
     }

   /*==============================================*/
   /* A shareable pattern node could not be found. */
   /*==============================================*/
   
   return(NULL);
  }
  
/*************************************************************/
/* RemoveUnneededSlots: Removes fact pattern nodes that have */
/*   no effect on pattern matching. For example, given the   */
/*   following deftemplate and a pattern using it,           */
/*                                                           */
/*   (deftemplate foo (field x) (field y))                   */
/*                                                           */
/*   (foo (x ?x) (y ?y))                                     */
/*                                                           */
/*   The x and y slot pattern nodes can be discarded since   */
/*   all foo facts will have these two slots in the fact     */
/*   data structure used to store them.                      */
/*************************************************************/
static struct lhsParseNode *RemoveUnneededSlots(thePattern)
  struct lhsParseNode *thePattern;
  {
   struct lhsParseNode *tempPattern = thePattern;
   struct lhsParseNode *lastPattern = NULL, *head = thePattern;
   struct expr *theTest;
   
   while (tempPattern != NULL)
     {
      /*=============================================================*/
      /* A single field slot that has no pattern network expression  */
      /* associated with it can be removed (i.e. any value contained */
      /* in this slot will satisfy the pattern being matched).       */
      /*=============================================================*/
      
      if (((tempPattern->type == SF_WILDCARD) || (tempPattern->type == SF_VARIABLE)) && 
          (tempPattern->networkTest == NULL))
        {
         if (lastPattern != NULL) lastPattern->right = tempPattern->right;
         else head = tempPattern->right;
         
         tempPattern->right = NULL;
         ReturnLHSParseNodes(tempPattern);
         
         if (lastPattern != NULL) tempPattern = lastPattern->right;
         else tempPattern = head;
        }
        
      /*=======================================================*/
      /* A multifield variable or wildcard within a multifield */
      /* slot can be removed if there are no other multifield  */
      /* variables or wildcards contained in the same slot     */
      /* (and the multifield has no expressions which must be  */
      /* evaluated in the fact pattern network).               */
      /*=======================================================*/
      
      else if (((tempPattern->type == MF_WILDCARD) || (tempPattern->type == MF_VARIABLE)) && 
               (tempPattern->multifieldSlot == CLIPS_FALSE) &&
               (tempPattern->networkTest == NULL) &&
               (tempPattern->multiFieldsBefore == 0) &&
               (tempPattern->multiFieldsAfter == 0))
        {
         if (lastPattern != NULL) lastPattern->right = tempPattern->right;
         else head = tempPattern->right;
         
         tempPattern->right = NULL;
         ReturnLHSParseNodes(tempPattern);
         
         if (lastPattern != NULL) tempPattern = lastPattern->right;
         else tempPattern = head;
        }  
  
      /*==================================================================*/
      /* A multifield wildcard or variable contained in a multifield slot */
      /* that contains no other multifield wildcards or variables, but    */
      /* does have an expression that must be evaluated, can be changed   */
      /* to a single field pattern node with the same expression.         */
      /*==================================================================*/
      
      else if (((tempPattern->type == MF_WILDCARD) || (tempPattern->type == MF_VARIABLE)) && 
               (tempPattern->multifieldSlot == CLIPS_FALSE) &&
               (tempPattern->networkTest != NULL) &&
               (tempPattern->multiFieldsBefore == 0) &&
               (tempPattern->multiFieldsAfter == 0))
        {
         tempPattern->type = SF_WILDCARD;
         lastPattern = tempPattern;
         tempPattern = tempPattern->right;
        }
        
      /*=========================================================*/
      /* If we're dealing with a multifield slot with no slot    */
      /* restrictions, then treat the multfield slot as a single */
      /* field slot, but attach a test which verifies that the   */
      /* slot contains a zero length multifield value.           */
      /*=========================================================*/
      
      else if ((tempPattern->type == MF_WILDCARD) && 
               (tempPattern->multifieldSlot == CLIPS_TRUE) &&
               (tempPattern->bottom == NULL))
        {
         tempPattern->type = SF_WILDCARD;
         tempPattern->networkTest = FactGenCheckZeroLength(tempPattern->slotNumber);
         tempPattern->multifieldSlot = CLIPS_FALSE;
         lastPattern = tempPattern;
         tempPattern = tempPattern->right;
        }
      
      /*===================================================*/
      /* Recursively call RemoveUnneededSlots for the slot */
      /* restrictions contained within a multifield slot.  */
      /*===================================================*/
      
      else if ((tempPattern->type == MF_WILDCARD) && 
               (tempPattern->multifieldSlot == CLIPS_TRUE))
        {
         /*=======================================================*/
         /* Add an expression to the first pattern restriction in */
         /* the multifield slot that determines whether or not    */
         /* the fact's slot value contains the minimum number of  */
         /* required fields to satisfy the pattern restrictions   */
         /* for this slot. The length check is place before any   */
         /* other tests, so that preceeding checks do not have to */
         /* determine if there are enough fields in the slot to   */
         /* safely retrieve a value.                              */
         /*=======================================================*/
         
         theTest = FactGenCheckLength(tempPattern->bottom);
         theTest = CombineExpressions(theTest,tempPattern->bottom->networkTest);
         tempPattern->bottom->networkTest = theTest;
         
         /*=========================================================*/
         /* Remove any unneeded pattern restrictions from the slot. */
         /*=========================================================*/
         
         tempPattern->bottom = RemoveUnneededSlots(tempPattern->bottom);
         
         /*===========================================================*/
         /* If the slot no longer contains any restrictions, then the */
         /* multifield slot can be completely removed. In any case,   */
         /* move on to the next slot to be examined for removal.      */
         /*===========================================================*/  

         if (tempPattern->bottom == NULL)
           {
            if (lastPattern != NULL) lastPattern->right = tempPattern->right;
            else head = tempPattern->right;
         
            tempPattern->right = NULL;
            ReturnLHSParseNodes(tempPattern);
         
            if (lastPattern != NULL) tempPattern = lastPattern->right;
            else tempPattern = head;
           }
         else
           {
            lastPattern = tempPattern;
            tempPattern = tempPattern->right;
           }
        }
      
      /*=======================================================*/
      /* If none of the other tests for removing slots or slot */
      /* restrictions apply, then move on to the next slot or  */
      /* slot restriction to be tested.                        */
      /*=======================================================*/
      
      else
        {
         lastPattern = tempPattern;
         tempPattern = tempPattern->right;
        }
     }
     
   /*======================================*/
   /* Return the pattern with unused slots */
   /* and slot restrictions removed.       */
   /*======================================*/
   
   return(head);
  }

/****************************************************/
/* CreateNewPatternNode: Creates a new pattern node */
/*   and initializes all of its values.             */
/****************************************************/
static struct factPatternNode *CreateNewPatternNode(thePattern,nodeBeforeMatch,
                                                    upperLevel,endSlot)
  struct lhsParseNode *thePattern;
  struct factPatternNode *nodeBeforeMatch;
  struct factPatternNode *upperLevel;
  int endSlot;
  {
   struct factPatternNode *newNode;

   /*========================================*/
   /* Create the pattern node and initialize */
   /* its slots to the default values.       */
   /*========================================*/
   
   newNode = get_struct(factPatternNode);
   newNode->nextLevel = NULL;
   newNode->rightNode = NULL;
   newNode->leftNode = NULL;
   newNode->leaveFields = thePattern->singleFieldsAfter;
   InitializePatternHeader((struct patternNodeHeader *) &newNode->header);
   
   if (thePattern->index > 0) newNode->whichField = thePattern->index;
   else newNode->whichField = 0;
   
   if (thePattern->slotNumber >= 0) newNode->whichSlot = thePattern->slotNumber - 1;
   else newNode->whichSlot = newNode->whichField;
   
   /*=============================================================*/
   /* Set the slot values which indicate whether the pattern node */
   /* is a single-field, multifield, or end-of-pattern node.      */
   /*=============================================================*/
   
   if ((thePattern->type == SF_WILDCARD) || (thePattern->type == SF_VARIABLE))
     { newNode->header.singlefieldNode = CLIPS_TRUE; }
   else if ((thePattern->type == MF_WILDCARD) || (thePattern->type == MF_VARIABLE))
     { newNode->header.multifieldNode = CLIPS_TRUE; }
   newNode->header.endSlot = endSlot;
   
   /*===========================================================*/
   /* Install the expression associated with this pattern node. */
   /*===========================================================*/
   
   newNode->networkTest = AddHashedExpression(thePattern->networkTest);

   /*===============================================*/
   /* Set the upper level pointer for the new node. */
   /*===============================================*/
   
   newNode->lastLevel = upperLevel;
   
   /*======================================================*/
   /* If there are no nodes on this level, then attach the */
   /* new node to the child pointer of the upper level.    */
   /*======================================================*/
         
   if (nodeBeforeMatch == NULL)
     {
      if (upperLevel == NULL) CurrentDeftemplate->patternNetwork = newNode;
      else upperLevel->nextLevel = newNode;
      return(newNode);
     }
   
   /*=====================================================*/
   /* If there is an upper level above the new node, then */
   /* place the new node as the first child in the upper  */
   /* level's nextLevel (child) link.                     */
   /*=====================================================*/
   
   if (upperLevel != NULL)
     {
      newNode->rightNode = upperLevel->nextLevel;
      if (upperLevel->nextLevel != NULL)
        { upperLevel->nextLevel->leftNode = newNode; }
      upperLevel->nextLevel = newNode;
      return(newNode);
     }
   
   /*=====================================================*/
   /* Since there is no upper level above the new node,   */
   /* (i.e. the new node is being added to the highest    */
   /* level in the pattern network), the new node becomes */
   /* the first node visited in the pattern network.      */
   /*=====================================================*/
     
   newNode->rightNode = CurrentDeftemplate->patternNetwork;
   if (CurrentDeftemplate->patternNetwork != NULL)
     { CurrentDeftemplate->patternNetwork->leftNode = newNode; }
     
   CurrentDeftemplate->patternNetwork = newNode;
   return(newNode);
  }

/*************************************************************/
/* DetachFactPattern: Removes a pattern node and all of its  */
/*   parent nodes from the pattern network. Nodes are only   */
/*   removed if they are no longer shared (i.e. a pattern    */
/*   node that has more than one child node is shared). A    */
/*   pattern from a rule is typically removed by removing    */
/*   the bottom most pattern node used by the pattern and    */
/*   then removing any parent nodes which are not shared by  */
/*   other patterns.                                         */
/*                                                           */
/*   Example:                                                */
/*     Patterns (a b c d) and (a b e f) would be represented */
/*     by the pattern net shown on the left.  If (a b c d)   */
/*     was detached, the resultant pattern net would be the  */
/*     one shown on the right.                               */
/*                                                           */
/*           a                  a                            */
/*           |                  |                            */
/*           b                  b                            */
/*           |                  |                            */
/*           c--e               e                            */
/*           |  |               |                            */
/*           d  f               f                            */
/*                                                           */
/*************************************************************/
static VOID DetachFactPattern(thePattern)
  struct patternNodeHeader *thePattern;
  {
   struct factPatternNode *patternPtr;
   struct factPatternNode *upperLevel;
   
   /*=====================================================*/
   /* Get rid of any matches stored in the alpha memory.  */
   /*=====================================================*/
   
   patternPtr = (struct factPatternNode *) thePattern;
   ClearPatternMatches(patternPtr);

   /*=====================================================*/
   /* If there are no joins entered from this pattern, then */
   /* the pattern node is no longer a stop node. Also if  */
   /* the pattern has a next level pointer, then it can   */
   /* not be removed since other patterns make use of it. */
   /*=====================================================*/
    
   if (patternPtr->header.entryJoin == NULL) patternPtr->header.stopNode = CLIPS_FALSE;
   if (patternPtr->nextLevel != NULL) return;
   
   /*==============================================================*/
   /* Loop until all appropriate pattern nodes have been detached. */
   /*==============================================================*/

   upperLevel = patternPtr;
   while (upperLevel != NULL)
     {
      if ((upperLevel->leftNode == NULL) &&
          (upperLevel->rightNode == NULL))
        {
         /*===============================================*/
         /* Pattern node is the only node on this level.  */
         /* Remove it and continue detaching other nodes  */
         /* above this one, because no other patterns are */
         /* dependent upon this node.                     */
         /*===============================================*/

         patternPtr = upperLevel;
         upperLevel = patternPtr->lastLevel;
         
         if (upperLevel == NULL) 
           { FindAndSetDeftemplatePatternNetwork(patternPtr,NULL); }
         else 
           {
            upperLevel->nextLevel = NULL;
            if (upperLevel->header.stopNode) upperLevel = NULL;
           }

         RemoveHashedExpression(patternPtr->networkTest);
         rtn_struct(factPatternNode,patternPtr);
        }
      else if (upperLevel->leftNode != NULL)
        {
         /*====================================================*/
         /* Pattern node has another pattern node which must   */
         /* be checked preceding it.  Remove the pattern node, */
         /* but do not detach any nodes above this one.        */
         /*====================================================*/

         patternPtr = upperLevel;
         upperLevel->leftNode->rightNode = upperLevel->rightNode;
         if (upperLevel->rightNode != NULL)
           { upperLevel->rightNode->leftNode = upperLevel->leftNode; }

         RemoveHashedExpression(patternPtr->networkTest);
         rtn_struct(factPatternNode,patternPtr);
         upperLevel = NULL;
        }
      else
        {
         /*====================================================*/
         /* Pattern node has no pattern node preceding it, but */
         /* does have one succeeding it. Remove the pattern    */
         /* node, but do not detach any nodes above this one.  */
         /*====================================================*/

         patternPtr = upperLevel;
         upperLevel = upperLevel->lastLevel;
         if (upperLevel == NULL)
           { FindAndSetDeftemplatePatternNetwork(patternPtr,patternPtr->rightNode); }
         else
           { upperLevel->nextLevel = patternPtr->rightNode; }
         patternPtr->rightNode->leftNode = NULL;

         RemoveHashedExpression(patternPtr->networkTest);
         rtn_struct(factPatternNode,patternPtr);
         upperLevel = NULL;
        }
     }
  }
  
/***********************************************************/
/* FindAndSetDeftemplatePatternNetwork: When a deftemplate */
/*   pattern is detached from the fact pattern network, it */
/*   is not possible to directly detach the link from the  */
/*   deftemplate to the pattern network (it is a one way   */
/*   link). Therefore if the top most pointer to a         */
/*   deftemplates pattern network must be changed, it is   */
/*   necessary to search the list of deftemplates to find  */
/*   the appropriate one to modify.                        */
/***********************************************************/
static VOID FindAndSetDeftemplatePatternNetwork(whichOne,newValue)
  struct factPatternNode *whichOne;
  struct factPatternNode *newValue;
  {
   struct deftemplate *theDeftemplate;
   struct defmodule *theModule;
   
   SaveCurrentModule();
   
   for (theModule = (struct defmodule *) GetNextDefmodule(NULL);
        theModule != NULL;
        theModule = (struct defmodule *) GetNextDefmodule(theModule))
     {
      SetCurrentModule((VOID *) theModule);
      theDeftemplate = (struct deftemplate *) GetNextDeftemplate(NULL);
      while (theDeftemplate != NULL)
        {
         if (theDeftemplate->patternNetwork == whichOne)
           {
            RestoreCurrentModule();
            theDeftemplate->patternNetwork = newValue;
            return;
           }
         theDeftemplate = (struct deftemplate *) GetNextDeftemplate(theDeftemplate);
        }
     }
     
   RestoreCurrentModule();
  }
              
/***************************************************************/
/* ClearPatternMatches: Clears the fact list of all pointers   */
/*   which point to a specific pattern.  The pointers are used */
/*   to remember which patterns were matched by a fact to      */
/*   make retraction easier.  When a rule is excised, the      */
/*   pointers need to be removed.                              */
/***************************************************************/
static VOID ClearPatternMatches(patternPtr)
  struct factPatternNode *patternPtr;
  {
   struct fact *fact_ptr;
   struct patternMatch *match_before, *match_ptr;

   /*===========================================*/
   /* Loop through every fact in the fact list. */
   /*===========================================*/

   fact_ptr = (struct fact *) GetNextFact(NULL);
   while (fact_ptr != NULL)
     {
      match_before = NULL;
      match_ptr = (struct patternMatch *) fact_ptr->list;

      /*========================================*/
      /* Loop through every match for the fact. */
      /*========================================*/

      while (match_ptr != NULL)
        {
         if (match_ptr->matchingPattern == (struct patternNodeHeader *) patternPtr)
           {
            /* Remove the match. */
            if (match_before == NULL)
              {
               fact_ptr->list = (VOID *) match_ptr->next;
               rtn_struct(patternMatch,match_ptr);
               match_ptr = (struct patternMatch *) fact_ptr->list;
              }
            else
             {
              match_before->next = match_ptr->next;
              rtn_struct(patternMatch,match_ptr);
              match_ptr = match_before->next;
             }
           }
         else
          {
           /* Move on to the next match. */
           match_before = match_ptr;
           match_ptr = match_ptr->next;
          }
       }
     fact_ptr = (struct fact *) GetNextFact(fact_ptr);
    }
  }
#endif
  
#endif



