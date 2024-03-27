   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*          OBJECT PATTERN MATCHER MODULE              */
   /*******************************************************/

/**************************************************************/
/* Purpose: RETE Network Interface for Objects                */
/*                                                            */
/* Principal Programmer(s):                                   */
/*      Brian L. Donnell                                      */
/*                                                            */
/* Contributing Programmer(s):                                */
/*                                                            */
/* Revision History:                                          */
/*                                                            */
/**************************************************************/
/* =========================================
   *****************************************
               EXTERNAL DEFINITIONS
   =========================================
   ***************************************** */
#include "setup.h"

#if INSTANCE_PATTERN_MATCHING

#include "classfun.h"
#include "clipsmem.h"
#include "drive.h"
#include "multifld.h"

#if LOGICAL_DEPENDENCIES
#include "lgcldpnd.h"
#endif

#if INCREMENTAL_RESET && (! RUN_TIME) && (! BLOAD_ONLY)
#include "incrrset.h"
#endif

#include "reteutil.h"
#include "ruledlt.h"
#include "reorder.h"
#include "retract.h"
#include "router.h"

#include "objrtfnx.h"

#define _OBJRTMCH_SOURCE_
#include "objrtmch.h"

#include "insmngr.h"

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */

/* =========================================
   *****************************************
                 MACROS AND TYPES
   =========================================
   ***************************************** */
typedef struct objectMatchAction
  {
   int type;
   INSTANCE_TYPE *ins;
   int slotNameID;
   struct objectMatchAction *nxt;
  } OBJECT_MATCH_ACTION;

/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER
static VOID QueueObjectMatchAction(int,INSTANCE_TYPE *,int);
static VOID MarkObjectPatternNetwork(int);
static VOID ObjectPatternMatch(int,OBJECT_PATTERN_NODE *,struct multifieldMarker *);
static VOID ProcessPatternNode(int,OBJECT_PATTERN_NODE *,struct multifieldMarker *);
static VOID CreateObjectAlphaMatch(OBJECT_ALPHA_NODE *);
static BOOLEAN EvaluateObjectPatternTest(int,struct multifieldMarker *,EXPRESSION *,
                                         OBJECT_PATTERN_NODE *);
static VOID ObjectRetractAction(INSTANCE_TYPE *,int);
static VOID ObjectPatternNetErrorMessage(OBJECT_PATTERN_NODE *);
static VOID TraceErrorToObjectPattern(int,OBJECT_PATTERN_NODE *);
#else
static VOID QueueObjectMatchAction();
static VOID MarkObjectPatternNetwork();
static VOID ObjectPatternMatch();
static VOID ProcessPatternNode();
static VOID CreateObjectAlphaMatch();
static BOOLEAN EvaluateObjectPatternTest();
static VOID ObjectRetractAction();
static VOID ObjectPatternNetErrorMessage();
static VOID TraceErrorToObjectPattern();
#endif

/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */

/* =========================================
   *****************************************
      INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
static OBJECT_MATCH_ACTION *ObjectMatchActionQueue = NULL;
static OBJECT_PATTERN_NODE *ObjectPatternNetworkPointer = NULL;
static OBJECT_ALPHA_NODE *ObjectPatternNetworkTerminalPointer = NULL;
static BOOLEAN DelayObjectPatternMatching = CLIPS_FALSE;
static unsigned long CurrentObjectMatchTimeTag = 0L;

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/***************************************************
  NAME         : ResetObjectMatchTimeTags
  DESCRIPTION  : Resets the pattern network marks
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : CurrentObjectMatchTimeTag reset to
                 0, and all match time tags reset
                 These are used to recognize valid
                 pattern nodes on a match
  NOTES        : Done during a (reset) or (clear)
 ***************************************************/
globle VOID ResetObjectMatchTimeTags()
  {
   OBJECT_ALPHA_NODE *alphaPtr;
   OBJECT_PATTERN_NODE *lastLevel;

   CurrentObjectMatchTimeTag = 0L;   
   alphaPtr = ObjectNetworkTerminalPointer();
   while (alphaPtr != NULL)
     {
      alphaPtr->matchTimeTag = 0L;
      lastLevel = alphaPtr->patternNode;
      while (lastLevel != NULL)
        {
         if (lastLevel->matchTimeTag == 0L)
           break;
         lastLevel->matchTimeTag = 0L;
         lastLevel = lastLevel->lastLevel;
        }
      alphaPtr = alphaPtr->nxtTerminal;
     }
  }

/***************************************************************************
  NAME         : ObjectMatchDelay
  DESCRIPTION  : CLIPS interface for SetDelayObjectPatternMatching
  INPUTS       : None
  RETURNS      : The old value of DelayObjectPatternMatching
  SIDE EFFECTS : DelayObjectPatternMatching set and Rete network updates
                 delayed until pattern-matching is completed
  NOTES        : CLIPS Syntax: (object-pattern-match-delay <action>*)
 ***************************************************************************/
globle VOID ObjectMatchDelay(result)
  DATA_OBJECT *result;
  {
   register int ov;
   
   ov = SetDelayObjectPatternMatching(CLIPS_TRUE);
   EvaluateExpression(GetFirstArgument(),result);
   if (EvaluationError)
     {
      SetDelayObjectPatternMatching(ov);
      SetEvaluationError(CLIPS_TRUE);
     }
   else
     SetDelayObjectPatternMatching(ov);
  }
  
/***************************************************
  NAME         : SetDelayObjectPatternMatching
  DESCRIPTION  : Sets the flag determining if Rete
                 network activity is to be delayed
                 for objects or not
  INPUTS       : The value of the flag
  RETURNS      : The old value of the flag
  SIDE EFFECTS : DelayObjectPatternMatching set
  NOTES        : When the delay is set to CLIPS_FALSE,
                 all pending Rete network updates
                 are performed
 ***************************************************/
globle BOOLEAN SetDelayObjectPatternMatching(value)
  int value;
  {
   BOOLEAN oldval;
   
   oldval = DelayObjectPatternMatching;
   if (value)
     DelayObjectPatternMatching = CLIPS_TRUE;
   else
     {
      DelayObjectPatternMatching = CLIPS_FALSE;
      ObjectNetworkAction(0,NULL,-1);
     }
   return(oldval);
  }
  
/***************************************************
  NAME         : GetDelayObjectPatternMatching
  DESCRIPTION  : Gets the flag determining if Rete
                 network activity is to be delayed
                 for objects or not
  INPUTS       : None
  RETURNS      : The flag
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle BOOLEAN GetDelayObjectPatternMatching()
  {
   return(DelayObjectPatternMatching);
  }

/********************************************************
  NAME         : ObjectNetworkPointer
  DESCRIPTION  : Returns the first object network
                 pattern node
  INPUTS       : None
  RETURNS      : The top of the object pattern network
  SIDE EFFECTS : None
  NOTES        : None
 ********************************************************/
globle OBJECT_PATTERN_NODE *ObjectNetworkPointer()
  {
   return(ObjectPatternNetworkPointer);
  }

/********************************************************
  NAME         : ObjectNetworkTerminalPointer
  DESCRIPTION  : Returns the first terminal pattern node
  INPUTS       : None
  RETURNS      : The last node of a pattern
  SIDE EFFECTS : None
  NOTES        : None
 ********************************************************/
globle OBJECT_ALPHA_NODE *ObjectNetworkTerminalPointer()
  {
   return(ObjectPatternNetworkTerminalPointer);
  }

/***************************************************
  NAME         : SetObjectNetworkPointer
  DESCRIPTION  : Sets the object pattern network to
                  the given network
  INPUTS       : Top of the new pattern network
  RETURNS      : Nothing useful
  SIDE EFFECTS : ObjectPatternNetworkPointer set
  NOTES        : None
 ***************************************************/
globle VOID SetObjectNetworkPointer(value)
  OBJECT_PATTERN_NODE *value;
  {
   ObjectPatternNetworkPointer = value;
  }

/*******************************************************
  NAME         : SetObjectNetworkTerminalPointer
  DESCRIPTION  : Sets the global list of terminal
                 pattern nodes (the ones containing
                 the bitmaps) to the given node
  INPUTS       : The last node of a pattern
  RETURNS      : Nothing useful
  SIDE EFFECTS : ObjectPatternNetworkTerminalPointer set
  NOTES        : None
 *******************************************************/
globle VOID SetObjectNetworkTerminalPointer(value)
  OBJECT_ALPHA_NODE *value;
  {
   ObjectPatternNetworkTerminalPointer = value;
  }

/************************************************************************
  NAME         : ObjectNetworkAction
  DESCRIPTION  : Main driver for pattern-matching on objects
                 If the pattern-matching is current delayed or another
                 object is currently being pattern-matched, the requested
                 match action is queued for later processing.
                 Otherwise, the match action is performed and the
                 Rete network is updated.
  INPUTS       : 1) The match action type
                    OBJECT_ASSERT  (1)
                    OBJECT_RETRACT (2)
                    OBJECT_MODIFY  (3)
                 2) The instance to be matched (can be NULL if only
                    want pending actions to be performed)
                 3) The name id of the slot being updated (can be -1)
                    If this argument is -1, it is assumed that any
                    pattern which could match this instance must be
                    checked.  Otherwise, only the patterns which
                    explicitly match on the named slot will be checked.
  RETURNS      : Nothing useful
  SIDE EFFECTS : Action queued or Rete netrwork updated
  NOTES        : None
 ************************************************************************/
globle VOID ObjectNetworkAction(type,ins,slotNameID)
  int type;
  INSTANCE_TYPE *ins;
  int slotNameID;
  {
   OBJECT_MATCH_ACTION *cur;
   
   /* ============================================================================
      If pattern-matching is delayed (by use of the set-object-pattern-match-delay
      function), then the instance should be marked for later processing (when
      the delay is turned off).
      ============================================================================ */
   if (JoinOperationInProgress)
     return;
   
   JoinOperationInProgress = CLIPS_TRUE;

   if (ins != NULL)
     {
      if (DelayObjectPatternMatching == CLIPS_FALSE)
        {
         if (type == OBJECT_ASSERT)
           {
            ins->header.timeTag = CurrentEntityTimeTag++;
            CurrentPatternObject = ins;
            CurrentPatternObjectSlot = NULL;
            MarkObjectPatternNetwork(-1);
            ObjectPatternMatch(0,ObjectNetworkPointer(),NULL);
           }
         else if (type == OBJECT_RETRACT)
           ObjectRetractAction(ins,-1);
         else
           {
            ins->header.timeTag = CurrentEntityTimeTag++;
            ObjectRetractAction(ins,slotNameID);
            CurrentPatternObject = ins;
            CurrentPatternObjectSlot = NULL;
            MarkObjectPatternNetwork(slotNameID);
            ObjectPatternMatch(0,ObjectNetworkPointer(),NULL);
           }
        }
      else
        QueueObjectMatchAction(type,ins,slotNameID);
     }
     
   /* ========================================
      Process all pending actions in the queue
      ======================================== */
   while ((ObjectMatchActionQueue != NULL) && 
          (DelayObjectPatternMatching == CLIPS_FALSE))
     {
      cur = ObjectMatchActionQueue;
      ObjectMatchActionQueue = cur->nxt;
      
      if (cur->type == OBJECT_ASSERT)
        {
         cur->ins->header.timeTag = CurrentEntityTimeTag++;
         CurrentPatternObject = cur->ins;
         CurrentPatternObjectSlot = NULL;
         MarkObjectPatternNetwork(-1);
         ObjectPatternMatch(0,ObjectNetworkPointer(),NULL);
        }
      else if (cur->type == OBJECT_RETRACT)
        ObjectRetractAction(cur->ins,-1);
      else
        {
         cur->ins->header.timeTag = CurrentEntityTimeTag++;
         ObjectRetractAction(cur->ins,cur->slotNameID);
         CurrentPatternObject = cur->ins;
         CurrentPatternObjectSlot = NULL;
         MarkObjectPatternNetwork(cur->slotNameID);
         ObjectPatternMatch(0,ObjectNetworkPointer(),NULL);
        }
      cur->ins->busy--;
      rtn_struct(objectMatchAction,cur);
     }
   JoinOperationInProgress = CLIPS_FALSE;
#if LOGICAL_DEPENDENCIES
   ForceLogicalRetractions();
#endif
  }
  
/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
  
/***************************************************
  NAME         : QueueObjectMatchAction
  DESCRIPTION  : Posts a Rete network match event
                 for later processing
  INPUTS       : 1) The match action type
                    OBJECT_ASSERT  (1)
                    OBJECT_RETRACT (2)
                    OBJECT_MODIFY  (3)
                 2) The instance to be matched
                 3) The name id of the slot being
                    updated (can be -1)
  RETURNS      : Nothing useful
  SIDE EFFECTS : Queue updated
  NOTES        : None
 ***************************************************/
static VOID QueueObjectMatchAction(type,ins,slotNameID)
  int type;
  INSTANCE_TYPE *ins;
  int slotNameID;
  {
   OBJECT_MATCH_ACTION *prv,*cur,*new,*tmp1,*tmp2;

   prv = NULL;
   cur = ObjectMatchActionQueue;
   while (cur != NULL)
     {
      /* ===========================================================
         Here are the possibilities for the first Rete event already
         on the queue as compared with the new event for an object:
         
         Assert/Retract  -->  Delete assert event
                              Ignore retract event
         Assert/Modify   -->  Ignore modify event
         Modify/Modify   -->  Queue new modify event
         Modify/Retract  -->  Delete all modify events
                              Queue the retract event
         
         Regardless, a new event must be placed at the end
         of the queue so that conflict resolution strategies
         will work properly when the pattern-matching
         actually occurs
         =========================================================== */
      if (cur->ins == ins)
        {
         /* ===================================================
            An action for initially asserting the newly created
            object to all applicable patterns
            =================================================== */
         if (cur->type == OBJECT_ASSERT)
           {
            if (type == OBJECT_RETRACT)
              {
               /* ===================================================
                  If we are retracting the entire object, then we can
                  remove the assert action (and all modifies as well)
                  and ignore the retract action
                  (basically the object came and went before the Rete
                  network had a chance to see it)
                  =================================================== */
               if (prv == NULL)
                 ObjectMatchActionQueue = cur->nxt;
               else
                 prv->nxt = cur->nxt;
               cur->ins->busy--;
               rtn_struct(objectMatchAction,cur);
               return;
              }
            
            /* =================================================
               If this is a modify action, then we can ignore it
               since the assert action will encompass it
               ================================================= */
           }
         
         /* =====================================================
            If the object is being deleted after the slot modify,
            drop all modify events and stick the retract in their
            place.
            ===================================================== */
         else if (type == OBJECT_RETRACT)
           {
            cur->type = OBJECT_RETRACT;
            if (cur->slotNameID != -1)
              cur->slotNameID = -1;
            tmp1 = cur;
            tmp2 = cur->nxt;
            while (tmp2 != NULL)
              {
               if (tmp2->ins == cur->ins)
                 {
                  tmp1->nxt = tmp2->nxt;  
                  tmp2->ins->busy--;
                  rtn_struct(objectMatchAction,tmp2);
                  tmp2 = tmp1->nxt;
                 }
               else
                 {
                  tmp1 = tmp2;
                  tmp2 = tmp2->nxt;  
                 }
              }
           }
           
         /* ==================================================
            If a modify event for this slot is already on the
            queue move it to the end and ignore this one.
            Otherwise, queue this new modify event at the end.
            ================================================== */
         else
           {
            while (cur != NULL)
              {
               /* ==================================================
                  If the modify event already is on the queue,
                  breaking here will allow the if statement
                  below to move the event to the end of the queue
                  
                  If the loop finishes without finding the same
                  event, the outer searching loop will be terminated
                  and the new modify event will be appended to
                  the queue
                  ================================================== */
               if ((cur->ins == ins) && (cur->slotNameID == slotNameID))
                 break;
               prv = cur;
               cur = cur->nxt;
              }
            if (cur == NULL)
              break;
           }
         
         /* ===================================================
            Make sure the (changed) event gets placed at the
            end of the queue to insure that conflict resolution
            strategies work correctly
            =================================================== */
         if (cur->nxt != NULL)
           {
            if (prv == NULL)
              ObjectMatchActionQueue = cur->nxt;
            else
              prv->nxt = cur->nxt;
            new = cur;
            while (cur->nxt != NULL)
              cur = cur->nxt;
            cur->nxt = new;
            new->nxt = NULL;
           }
         return;
        }
      prv = cur;
      cur = cur->nxt;
     }
     
   /* ==============================================================
      If there are no actions for the instance already on the queue,
      the new action is simply appended.
      ============================================================== */
   new = get_struct(objectMatchAction);
   new->type = type;
   new->nxt = cur;
   new->slotNameID = slotNameID;
   new->ins = ins;
   new->ins->busy++;
   if (prv == NULL)
     ObjectMatchActionQueue = new;
   else
     prv->nxt = new;
  }

/******************************************************
  NAME         : MarkObjectPatternNetwork
  DESCRIPTION  : Iterates through all terminal
                 pattern nodes checking class and
                 slot bitmaps.  If a pattern is
                 applicable to the object/slot change,
                 then all the nodes belonging to
                 the pattern are marked as needing
                 to be examined by the pattern matcher.
  INPUTS       : The id of the slot being changed
                 (-1 if this is an assert for the
                  entire object)
  RETURNS      : Nothing useful
  SIDE EFFECTS : Applicable pattern nodes marked
  NOTES        : Incremental reset status is also
                 checked here
 ******************************************************/
static VOID MarkObjectPatternNetwork(slotNameID)
  int slotNameID;
  {
   OBJECT_ALPHA_NODE *alphaPtr;
   OBJECT_PATTERN_NODE *upper;
   CLASS_BITMAP *clsset;
   SLOT_BITMAP *sltset;
   unsigned id;

   CurrentObjectMatchTimeTag++;
   alphaPtr = ObjectNetworkTerminalPointer();
   id = CurrentPatternObject->cls->id;
   while (alphaPtr != NULL)
     {
      /* =============================================================
         If an incremental reset is in progress, make sure that the
         pattern has been marked for initialization before proceeding.
         ============================================================= */
#if INCREMENTAL_RESET && (! RUN_TIME) && (! BLOAD_ONLY)
      if (IncrementalResetInProgress && 
          (alphaPtr->header.initialize == CLIPS_FALSE))
        {
         alphaPtr = alphaPtr->nxtTerminal;
         continue;
        }
#endif
              
      /* ============================================
         Check the class bitmap to see if the pattern
         pattern is applicable to the object at all
         ============================================ */
      clsset = (CLASS_BITMAP *) ValueToBitMap(alphaPtr->classbmp);
        
      if ((id > clsset->maxid) ? CLIPS_FALSE : TestBitMap(clsset->map,id))
        {
         /* ===================================================
            If we are doing an assert, then we need to
            check all patterns which satsify the class bitmap
            (The retraction has already been done in this case)
            =================================================== */
         if (slotNameID == -1)
           {
            alphaPtr->matchTimeTag = CurrentObjectMatchTimeTag;
            for (upper = alphaPtr->patternNode ; upper != NULL ; upper = upper->lastLevel)
              {
               if (upper->matchTimeTag == CurrentObjectMatchTimeTag)
                 break;
               else
                 upper->matchTimeTag = CurrentObjectMatchTimeTag;
              }
           }
           
         /* ===================================================
            If we are doing a slot modify, then we need to
            check only the subset of patterns which satisfy the
            class bitmap AND actually match on the slot in
            question.
            =================================================== */
         else if (alphaPtr->slotbmp != NULL)
           {
            sltset = (SLOT_BITMAP *) ValueToBitMap(alphaPtr->slotbmp);
            
            if ((slotNameID > sltset->maxid) ? CLIPS_FALSE : 
                 TestBitMap(sltset->map,slotNameID))
              {
               alphaPtr->matchTimeTag = CurrentObjectMatchTimeTag;
               for (upper = alphaPtr->patternNode ; upper != NULL ; upper = upper->lastLevel)
                 {
                  if (upper->matchTimeTag == CurrentObjectMatchTimeTag)
                    break;
                  else
                    upper->matchTimeTag = CurrentObjectMatchTimeTag;
                 }
              }
           }
        }
      alphaPtr = alphaPtr->nxtTerminal;   
     }
  }

/**********************************************************************************
  NAME         : ObjectPatternMatch
  DESCRIPTION  : Iterates through all the pattern nodes on one level
                 in the pattern network.  A node is only processed
                 if it can lead to a terminating class bitmap node
                 which applies to the object being matched.  This
                 allows for a significant reduction in the number of
                 patterns considered.
  INPUTS       : 1) The offset of the slot position from the pattern index
                 2) The pattern node being examined
                 3) The end of the list of multifield markers for the pattern
  RETURNS      : Nothing useful
  SIDE EFFECTS : The pattern tests are evaluated and the child nodes may
                 be processed (which may cause a whole series of Rete network
                 updates).
  NOTES        : Several globals are used to keep track of the current
                 slot being examined:
                 CurrentPatternMarks - the series of multifield markers
                 CurrentPatternObject - the object being pattern-matched
                 CurrentPatternObjectSlot - the current slot being examined
                 CurrentObjectSlotLength - the cardinality of the slot value
                 
                 An optimization is performed when evaluating
                 constant tests on a slot value field.  All
                 pattern nodes on a level which restrict the same slot
                 are grouped together.  Those which are constant
                 tests are placed at the far right.  Thus, as soon
                 as one of these constant tests succeeds, the remaining
                 nodes for that slot on this level can be skipped
 **********************************************************************************/
static VOID ObjectPatternMatch(offset,patternTop,endMark)
  int offset;
  OBJECT_PATTERN_NODE *patternTop;
  struct multifieldMarker *endMark;
  {
   register int saveSlotLength;
   register INSTANCE_SLOT *saveSlot;
   OBJECT_PATTERN_NODE *blockedNode;
   
   while (patternTop != NULL)
     {
      /* ===========================================================
         MarkObjectPatternNetwork() has already marked pattern nodes
         which need processing according to the class bitmaps,
         slot updates and incremental reset status
         =========================================================== */
      if (patternTop->matchTimeTag == CurrentObjectMatchTimeTag)
        {
         /* ========================================
            Make sure we are examining the correct
            slot of the object for this pattern node
            ======================================== */
         if ((patternTop->slotNameID == ISA_ID) ||
             (patternTop->slotNameID == NAME_ID))
           {
            CurrentPatternObjectSlot = NULL;
            CurrentObjectSlotLength = 1;
            offset = 0;
           }
         else if ((CurrentPatternObjectSlot == NULL) ? CLIPS_TRUE :
                  (CurrentPatternObjectSlot->desc->slotName->id != patternTop->slotNameID))
           {
            /* ====================================================
               Need to reset the indices for the multifield
               markers now that we have moved onto a different slot
                  ==================================================== */
            CurrentPatternObjectSlot = 
             CurrentPatternObject->slotAddresses[CurrentPatternObject->cls->slotNameMap
                                             [patternTop->slotNameID] - 1];
            offset = 0;
            if (CurrentPatternObjectSlot->desc->multiple)
              CurrentObjectSlotLength = 
                GetInstanceSlotLength(CurrentPatternObjectSlot);
            else
              CurrentObjectSlotLength = 1;
           }
            
         /* ========================================================
            Process the pattern node.  If it is satisfied by the
            the instance, ProcessPatternNode() will recursively pass
            all of its children nodes through ObjectPatternMatch()
            ======================================================== */
         saveSlotLength = CurrentObjectSlotLength;
         saveSlot = CurrentPatternObjectSlot;
         ProcessPatternNode(offset,patternTop,endMark);
         CurrentObjectSlotLength = saveSlotLength;
         CurrentPatternObjectSlot = saveSlot;
        }
         
      /* ==============================================================
         Move on to the siblings of this node - if the current node was
         a constant test that succeeded, skip further sibling nodes
         (which test on the same field in the pattern)
         which match on the same slot since they are all constant tests
         as well and will, of course fail.
         ============================================================== */
      if (patternTop->blocked == CLIPS_TRUE)
        {
         patternTop->blocked = CLIPS_FALSE;
         blockedNode = patternTop;
         patternTop = patternTop->rightNode;
         while (patternTop != NULL)
           {
            if ((patternTop->slotNameID != blockedNode->slotNameID) ||
                (patternTop->whichField != blockedNode->whichField))
              break;
            patternTop = patternTop->rightNode;
           }
        }
      else
        patternTop = patternTop->rightNode;
     }
  }

/**********************************************************************************
  NAME         : ProcessPatternNode
  DESCRIPTION  : Determines if a pattern node satsifies the corresponding
                 slot value field(s) in an object.  If it does,
                 ObjectPatternMatch() is recursively called to process
                 the child nodes of this node.  In this mutual recursion
                 between ObjectPatternMatch() and ProcessPatternNode(),
                 the nodes of all applicable patterns are processed
                 to completion.  ObjectPatternMatch() enters an object
                 into a pattern's aplha memory when the traversal reaches
                 a terminal class bitmap node.
  INPUTS       : 1) The offset of the slot index from the pattern index
                 2) The pattern node being examined
                 3) The end of the list of multifield markers for the pattern
  RETURNS      : Nothing useful
  SIDE EFFECTS : The pattern tests are evaluated and the child nodes may
                 be processed (which may cause a whole series of Rete network
                 updates).
  NOTES        : Several globals are used to keep track of the current
                 slot being examined:
                 CurrentPatternMarks - the series of multifield markers
                 CurrentPatternObject - the object being pattern-matched
                 CurrentPatternObjectSlot - the current slot being examined
                 CurrentObjectSlotLength - the cardinality of the slot value
 **********************************************************************************/
static VOID ProcessPatternNode(offset,patternNode,endMark)
  int offset;
  OBJECT_PATTERN_NODE *patternNode;
  struct multifieldMarker *endMark;
  {
   int patternSlotField,objectSlotField,
       objectSlotLength,
       repeatCount;
   INSTANCE_SLOT *objectSlot;
   struct multifieldMarker *newMark;
   
   patternSlotField = patternNode->whichField;
   objectSlotField = patternSlotField + offset;
   
   /* ==========================================
      If this is a test on the class or the name
      of the object, process it separately.
      ========================================== */
   if (CurrentPatternObjectSlot == NULL)
     {
      if ((patternNode->networkTest == NULL) ? CLIPS_TRUE :
          (EvaluateObjectPatternTest(objectSlotField,NULL, 
                                     (EXPRESSION *) patternNode->networkTest,patternNode)))
        {
         if (patternNode->alphaNode != NULL)
           CreateObjectAlphaMatch(patternNode->alphaNode);
         ObjectPatternMatch(offset,patternNode->nextLevel,endMark);
        }
      return;
     }
     
   /* ================================
      Check a single-field restriction
      ================================ */
   if (patternNode->multifieldNode == 0)
     {
      if ((patternNode->networkTest == NULL) ? CLIPS_TRUE : 
          EvaluateObjectPatternTest(objectSlotField,NULL,
                                    (EXPRESSION *) patternNode->networkTest,patternNode))
        {
         if (patternNode->alphaNode != NULL)
           CreateObjectAlphaMatch(patternNode->alphaNode);
         ObjectPatternMatch(offset,patternNode->nextLevel,endMark);
        }
      return;
     }
     
   /* ==================================================================
      Check a multifield restriction.  Add a marker for this field which
      has indices indicating to which values in the object slot the
      multifield pattern node is bound
      ================================================================== */
   newMark = get_struct(multifieldMarker);
   newMark->whichField = patternSlotField;
   newMark->where.whichSlot = (VOID *) CurrentPatternObjectSlot->desc->slotName->name;
   newMark->startPosition = objectSlotField;
   newMark->next = NULL;
   if (CurrentPatternObjectMarks == NULL)
     CurrentPatternObjectMarks = newMark;
   else
     endMark->next = newMark;
   
   /* ==========================================================
      If there are further pattern restrictions on this slot,
      try pattern-matching for all possible bound values of the
      multifield pattern node: from no values to all values from
      the starting position of the multifield to the end of the
      object slot.  Otherwise, bind the multifield to all the
      remaining fields in the slot value and continue with
      pattern-matching
      ========================================================== */
   if (patternNode->endSlot == CLIPS_FALSE)
     {
      objectSlotLength = CurrentObjectSlotLength;
      objectSlot = CurrentPatternObjectSlot;
      newMark->endPosition = newMark->startPosition - 1;
      repeatCount = objectSlotLength - newMark->startPosition
                    - patternNode->leaveFields + 2;
      while (repeatCount > 0)
        {
         if ((patternNode->networkTest == NULL) ? CLIPS_TRUE :
              EvaluateObjectPatternTest(objectSlotField,newMark,
                        (EXPRESSION *) patternNode->networkTest,patternNode))
           {
            if (patternNode->alphaNode != NULL)
              CreateObjectAlphaMatch(patternNode->alphaNode);
            ObjectPatternMatch(offset + (newMark->endPosition - objectSlotField),
                               patternNode->nextLevel,newMark);
            CurrentObjectSlotLength = objectSlotLength;
            CurrentPatternObjectSlot = objectSlot;
           }
         newMark->endPosition++;
         repeatCount--;
        }
     }
   else
     {
      newMark->endPosition = CurrentObjectSlotLength;
      if ((patternNode->networkTest == NULL) ? CLIPS_TRUE :
          EvaluateObjectPatternTest(objectSlotField,newMark,
                                    (EXPRESSION *) patternNode->networkTest,patternNode))
        {
         if (patternNode->alphaNode != NULL)
           CreateObjectAlphaMatch(patternNode->alphaNode);
         ObjectPatternMatch(0,patternNode->nextLevel,newMark);
        }
     }
   
   /* ======================================
      Delete the temporary multifield marker
      ====================================== */
   if (CurrentPatternObjectMarks == newMark)
     CurrentPatternObjectMarks = NULL;
   rtn_struct(multifieldMarker,newMark);
  }

/***************************************************
  NAME         : CreateObjectAlphaMatch
  DESCRIPTION  : Places an instance in the alpha
                 memory of a pattern and drives the
                 partial match through the join
                 network
  INPUTS       : The alpha memory node
  RETURNS      : Nothing useful
  SIDE EFFECTS : Join network updated
  NOTES        : None
 ***************************************************/
static VOID CreateObjectAlphaMatch(alphaPtr)
  OBJECT_ALPHA_NODE *alphaPtr;
  {
   struct joinNode *listOfJoins;
   struct partialMatch *theMatch;
   struct patternMatch *newMatch;

   while (alphaPtr != NULL)
     {
      if (alphaPtr->matchTimeTag == CurrentObjectMatchTimeTag)
        {
         /* ===================================================
            If we have reached the class bitmap of the pattern,
            place the object in the alpha memory of each of
            the terminal nodes underneath and drive
            the partial matches through the join network.

            Insert the instance into the alpha memory
            of this pattern and mark it as busy
            =================================================== */
         CurrentPatternObject->busy++;
         theMatch = CreateAlphaMatch((VOID *) CurrentPatternObject,
                                     CurrentPatternObjectMarks,
                                     (struct patternNodeHeader *) alphaPtr);
            
         /* ======================================
            Attach the partial match to the object
            to ease later retraction
            ====================================== */
         newMatch = get_struct(patternMatch);
         newMatch->next = (struct patternMatch *) CurrentPatternObject->partialMatchList;
         newMatch->matchingPattern = (struct patternNodeHeader *) alphaPtr;
         newMatch->theMatch = theMatch;
         CurrentPatternObject->partialMatchList = (VOID *) newMatch;
            
         /* ================================================
            Drive the partial match through the join network
            ================================================ */
         listOfJoins = alphaPtr->header.entryJoin;
         while (listOfJoins != NULL)
           {
            Drive(theMatch,listOfJoins,RHS);
            listOfJoins = listOfJoins->rightMatchNode;
           }
        }
      alphaPtr = alphaPtr->nxtInGroup;
     }
  }
  
/******************************************************
  NAME         : EvaluateObjectPatternTest
  DESCRIPTION  : Evaluates the pattern network test
                 expression for a node
  INPUTS       : 1) The actual index of the slot value
                    field currently being examined
                 2) The multifield marker (if any)
                    for the pattern node being exmained
                 3) The pattern network test expression
                 4) The pattern node being examined
  RETURNS      : CLIPS_TRUE if the node passes the
                 test, CLIPS_FALSE otherwise
  SIDE EFFECTS : Evaluation of the test
                 EvaluationError and HaltExecution
                 are always set to CLIPS_FALSE
  NOTES        : Assumes networkTest != NULL
 ******************************************************/
static BOOLEAN EvaluateObjectPatternTest(objectSlotField,selfSlotMarker,networkTest,patternNode)
  int objectSlotField;
  struct multifieldMarker *selfSlotMarker;
  EXPRESSION *networkTest;
  OBJECT_PATTERN_NODE *patternNode;
  {
   DATA_OBJECT vresult;
   int rv;
   
   if (networkTest->type == OBJ_PN_CONSTANT)
     {
      struct expr *oldArgument;
         
      oldArgument = CurrentExpression;
      CurrentExpression = networkTest;
      rv = ObjectCmpConstantFunction(networkTest->value,&vresult);
      CurrentExpression = oldArgument;
      if (rv)
        {
         if (((struct ObjectCmpPNConstant *) 
                 ValueToBitMap(networkTest->value))->pass)
           patternNode->blocked = CLIPS_TRUE;
         return(CLIPS_TRUE);
        }
      return(CLIPS_FALSE);
     }
     
   /* =========================================================
      Evaluate or expressions expressed in the format:
         (or <expression 1> <expression 2> ... <expression n>)
       Returns true (1.0) if any of the expression are true,
       otherwise returns false (0.0).
      ========================================================= */
   if (networkTest->value == PTR_OR)
     {
      networkTest = networkTest->argList;
      while (networkTest != NULL)
        {
         if (EvaluateObjectPatternTest(objectSlotField,selfSlotMarker,networkTest,patternNode))
           {
            /* ============================================
               A node can be blocked ONLY if there were one
               positive constant test on that node
               ============================================ */
            patternNode->blocked = CLIPS_FALSE;
            return(CLIPS_TRUE);
           }
         patternNode->blocked = CLIPS_FALSE;
         networkTest = networkTest->nextArg;
        }
      return(CLIPS_FALSE);
     }

   /* ==========================================================
      Evaluate and expressions expressed in the format:
       (and <expression 1> <expression 2> ... <expression n>)
      Returns false (0.0) if any of the expression are false,
      otherwise returns true (1.0).
      ========================================================== */
   else if (networkTest->value == PTR_AND)
     {
      networkTest = networkTest->argList;
      while (networkTest != NULL)
        {
         if (EvaluateObjectPatternTest(objectSlotField,selfSlotMarker,networkTest,patternNode)
              == CLIPS_FALSE)
           {
            patternNode->blocked = CLIPS_FALSE;
            return(CLIPS_FALSE);
           }
         patternNode->blocked = CLIPS_FALSE;
         networkTest = networkTest->nextArg;
        }
      return(CLIPS_TRUE);
     }

   /* =======================================================
      Evaluate all other expressions using EvaluateExpression
      ======================================================= */
   else
     {
      HaltExecution = CLIPS_FALSE;
      if (EvaluateExpression(networkTest,&vresult))
        {
         ObjectPatternNetErrorMessage(patternNode);
         EvaluationError = CLIPS_FALSE;
         HaltExecution = CLIPS_FALSE;
         return(CLIPS_FALSE);
        }
      if ((vresult.value != CLIPSFalseSymbol) || (vresult.type != SYMBOL))
        return(CLIPS_TRUE);
     }
   return(CLIPS_FALSE);
  }

/****************************************************
  NAME         : ObjectRetractAction
  DESCRIPTION  : Retracts the instance from the
                 applicable patterns for the object
                 (if the slotNameID != -1, then the
                  instance is only retracted from
                  the alpha memories of the patterns
                  which actually match on that slot)
  INPUTS       : 1) The instance
                 2) The id of the slot being modified
                    (-1 if the instance is actually
                     being removed)
  RETURNS      : Nothing useful
  SIDE EFFECTS : Retractions performed
  NOTES        : None
 ****************************************************/
static VOID ObjectRetractAction(ins,slotNameID)
  INSTANCE_TYPE *ins;
  int slotNameID;
  {
   struct patternMatch *prvMatch,*tmpMatch,
                       *deleteMatch,*svMatch;
   OBJECT_ALPHA_NODE *alphaPtr;
   SLOT_BITMAP *sltset;
#if LOGICAL_DEPENDENCIES
   VOID *saveDependents;
#endif
   
   if (slotNameID == -1)
     {
      if (ins->partialMatchList != NULL)
        {
         tmpMatch = (struct patternMatch *) ins->partialMatchList;
         while (tmpMatch != NULL)
           {
            ins->busy--;
            tmpMatch = tmpMatch->next;
           }
         NetworkRetract((struct patternMatch *) ins->partialMatchList);
         ins->partialMatchList = NULL;
        }
     }
   else
     {
      deleteMatch = NULL;
      prvMatch = NULL;
      tmpMatch = (struct patternMatch *) ins->partialMatchList;
      while (tmpMatch != NULL)
        {
         alphaPtr = (OBJECT_ALPHA_NODE *) tmpMatch->matchingPattern;
         if (alphaPtr->slotbmp != NULL)
           {
            sltset = (SLOT_BITMAP *) ValueToBitMap(alphaPtr->slotbmp);
            if ((slotNameID > sltset->maxid) ? CLIPS_FALSE : 
                 TestBitMap(sltset->map,slotNameID))
              {
               ins->busy--;
               if (prvMatch == NULL)
                 ins->partialMatchList = (VOID *) tmpMatch->next;
               else
                 prvMatch->next = tmpMatch->next;
               svMatch = tmpMatch->next;
               tmpMatch->next = deleteMatch;
               deleteMatch = tmpMatch;
               tmpMatch = svMatch;
              }
            else
              {
               prvMatch = tmpMatch;
               tmpMatch = tmpMatch->next;
              }
           }
         else
           {
            prvMatch = tmpMatch;
            tmpMatch = tmpMatch->next;
           }
        }
        
      /* =============================================
         We need to preserve any logical dependencies
         of this object and reattach them after doing
         the retract.  Otherwise, the Rete network
         will believe the object is gone and remove
         the links from the partial matches upon which
         this object is logically dependent.
         ============================================= */
      if (deleteMatch != NULL)
        {
#if LOGICAL_DEPENDENCIES
         saveDependents = ins->header.dependents;
         ins->header.dependents = NULL;
         NetworkRetract(deleteMatch);
         ins->header.dependents = saveDependents;
#else
         NetworkRetract(deleteMatch);
#endif
        }
     }
  }
  
/*****************************************************
  NAME         : ObjectPatternNetErrorMessage
  DESCRIPTION  : Prints out a locational error message
                 when an evaluation error occurs
                 during object pattern-matching
  INPUTS       : The pattern node
  RETURNS      : Nothing useful
  SIDE EFFECTS : Error message displayed
  NOTES        : None
 *****************************************************/
static VOID ObjectPatternNetErrorMessage(patternPtr)
  OBJECT_PATTERN_NODE *patternPtr;
  {
   PrintErrorID("OBJRTMCH",1,CLIPS_TRUE);
   PrintCLIPS(WERROR,"This error occurred in the object pattern network\n");
   PrintCLIPS(WERROR,"   Currently active instance: [");
   PrintCLIPS(WERROR,ValueToString(CurrentPatternObject->name));
   PrintCLIPS(WERROR,"]\n");
   PrintCLIPS(WERROR,"   Problem resides in slot ");
   PrintCLIPS(WERROR,ValueToString(FindIDSlotName(patternPtr->slotNameID)));
   PrintCLIPS(WERROR," field #");
   PrintLongInteger(WERROR,(long) patternPtr->whichField);
   PrintCLIPS(WERROR,"\n");
   TraceErrorToObjectPattern(CLIPS_TRUE,patternPtr);
   PrintCLIPS(WERROR,"\n");
  }

/*********************************************************
  NAME         : TraceErrorToObjectPattern
  DESCRIPTION  : Used by ObjectPatternNetErrorMessage() to
                 print the rule(s) which contain an object
                 pattern.
  INPUTS       : 1) A flag indicating if this is the
                    node in which the error actually
                    occurred or not
                 2) The pattern node
  RETURNS      : Nothing useful
  SIDE EFFECTS : Error message displayed
  NOTES        : None
 *********************************************************/
static VOID TraceErrorToObjectPattern(errorNode,patternPtr)
  int errorNode;
  OBJECT_PATTERN_NODE *patternPtr;
  {
   struct joinNode *joinPtr;

   while (patternPtr != NULL)
     {
      if (patternPtr->alphaNode != NULL)
        {
         joinPtr = patternPtr->alphaNode->header.entryJoin;
         while (joinPtr != NULL)
           {
            PrintCLIPS(WERROR,"      Of pattern #");
            PrintLongInteger(WERROR,(long) joinPtr->depth);
            PrintCLIPS(WERROR," in rule(s):\n");
            TraceErrorToRule(joinPtr,"         ");
            joinPtr = joinPtr->rightMatchNode;
           }
        }
      TraceErrorToObjectPattern(CLIPS_FALSE,patternPtr->nextLevel);
      if (errorNode)
        break;
      patternPtr = patternPtr->rightNode;
     }

  }

#endif
   
/***************************************************
  NAME         : 
  DESCRIPTION  : 
  INPUTS       : 
  RETURNS      : 
  SIDE EFFECTS : 
  NOTES        : 
 ***************************************************/
