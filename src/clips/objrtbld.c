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
/* Purpose: RETE Network Parsing Interface for Objects        */
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

#if (! BLOAD_ONLY) && (! RUN_TIME)

#if ANSI_COMPILER
#include <string.h>
#endif

#include "classcom.h"
#include "classfun.h"
#include "clipsmem.h"
#include "cstrnutl.h"
#include "constrnt.h"
#include "cstrnchk.h"
#include "cstrnops.h"
#include "ruledef.h"
#include "drive.h"
#include "exprnpsr.h"
#include "insfun.h"
#include "insmngr.h"
#include "network.h"
#include "object.h"
#include "pattern.h"
#include "reteutil.h"
#include "rulepsr.h"
#include "scanner.h"
#include "symbol.h"
#include "utility.h"

#endif

#include "constrct.h"
#include "objrtmch.h"
#include "objrtgen.h"
#include "objrtfnx.h"
#include "reorder.h"
#include "router.h"

#if CONSTRUCT_COMPILER && (! RUN_TIME)
#include "objrtcmp.h"
#endif

#if BLOAD_AND_BSAVE || BLOAD || BLOAD_ONLY
#include "objrtbin.h"
#endif

#define _OBJRTBLD_SOURCE_
#include "objrtbld.h"

#if ! DEFINSTANCES_CONSTRUCT
#include "extnfunc.h"
#include "classfun.h"
#include "classcom.h"
#endif

#if (! BLOAD_ONLY) && (! RUN_TIME)

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */
#define OBJECT_PATTERN_INDICATOR "object"

/* =========================================
   *****************************************
                 MACROS AND TYPES
   =========================================
   ***************************************** */
#define ClassBitMapSize(bmp) ((int) (sizeof(CLASS_BITMAP) + \
                                     (sizeof(char) * (bmp->maxid / BITS_PER_BYTE))))
                                     
#define SlotBitMapSize(bmp) ((int) (sizeof(SLOT_BITMAP) + \
                                     (sizeof(char) * (bmp->maxid / BITS_PER_BYTE))))

/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER
static BOOLEAN PatternParserFind(SYMBOL_HN *);
static struct lhsParseNode *ObjectLHSParse(char *,struct token *);
static BOOLEAN ReorderAndAnalyzeObjectPattern(struct lhsParseNode *);
static struct patternNodeHeader *PlaceObjectPattern(struct lhsParseNode *);
static OBJECT_PATTERN_NODE *FindObjectPatternNode(OBJECT_PATTERN_NODE *,struct lhsParseNode *,
                                                  OBJECT_PATTERN_NODE **,unsigned);
static OBJECT_PATTERN_NODE *CreateNewObjectPatternNode(struct lhsParseNode *,OBJECT_PATTERN_NODE *,
                                                       OBJECT_PATTERN_NODE *,unsigned);
static VOID DetachObjectPattern(struct patternNodeHeader *);
static VOID ClearObjectPatternMatches(OBJECT_ALPHA_NODE *);
static VOID RemoveObjectPartialMatches(INSTANCE_TYPE *,struct patternNodeHeader *);
static BOOLEAN CheckDuplicateSlots(struct lhsParseNode *,SYMBOL_HN *);
static struct lhsParseNode *ParseClassRestriction(char *,struct token *);
static struct lhsParseNode *ParseNameRestriction(char *,struct token *);
static struct lhsParseNode *ParseSlotRestriction(char *,struct token *,CONSTRAINT_RECORD *,int);
static CLASS_BITMAP *NewClassBitMap(int,int);
static VOID InitializeClassBitMap(CLASS_BITMAP *,int);
static VOID DeleteIntermediateClassBitMap(CLASS_BITMAP *);
static VOID *CopyClassBitMap(VOID *);
static VOID DeleteClassBitMap(VOID *);
static VOID MarkBitMapClassesBusy(BITMAP_HN *,int);
static BOOLEAN EmptyClassBitMap(CLASS_BITMAP *);
static BOOLEAN IdenticalClassBitMap(CLASS_BITMAP *,CLASS_BITMAP *);
static BOOLEAN ProcessClassRestriction(CLASS_BITMAP *,struct lhsParseNode **,int);
static CONSTRAINT_RECORD *ProcessSlotRestriction(CLASS_BITMAP *,SYMBOL_HN *,int *);
static VOID IntersectClassBitMaps(CLASS_BITMAP *,CLASS_BITMAP *);
static VOID UnionClassBitMaps(CLASS_BITMAP *,CLASS_BITMAP *);
static CLASS_BITMAP *PackClassBitMap(CLASS_BITMAP *);
static struct lhsParseNode *FilterObjectPattern(struct patternParser *,
                                              struct lhsParseNode *,struct lhsParseNode **,
                                              struct lhsParseNode **,struct lhsParseNode **);
static BITMAP_HN *FormSlotBitMap(struct lhsParseNode *);
static struct lhsParseNode *RemoveSlotExistenceTests(struct lhsParseNode *,BITMAP_HN **);
static struct lhsParseNode *CreateInitialObjectPattern(void);
static EXPRESSION *ObjectMatchDelayParse(EXPRESSION *,char *);
#if INCREMENTAL_RESET
static VOID MarkObjectPtnIncrementalReset(struct patternNodeHeader *,int);
static VOID ObjectIncrementalReset(void);
#endif
#else
static BOOLEAN PatternParserFind();
static struct lhsParseNode *ObjectLHSParse();
static BOOLEAN ReorderAndAnalyzeObjectPattern();
static struct patternNodeHeader *PlaceObjectPattern();
static OBJECT_PATTERN_NODE *FindObjectPatternNode();
static OBJECT_PATTERN_NODE *CreateNewObjectPatternNode();
static VOID DetachObjectPattern();
static VOID ClearObjectPatternMatches();
static VOID RemoveObjectPartialMatches();
static BOOLEAN CheckDuplicateSlots();
static struct lhsParseNode *ParseClassRestriction();
static struct lhsParseNode *ParseNameRestriction();
static struct lhsParseNode *ParseSlotRestriction();
static CLASS_BITMAP *NewClassBitMap();
static VOID InitializeClassBitMap();
static VOID DeleteIntermediateClassBitMap();
static VOID *CopyClassBitMap();
static VOID DeleteClassBitMap();
static VOID MarkBitMapClassesBusy();
static BOOLEAN EmptyClassBitMap();
static BOOLEAN IdenticalClassBitMap();
static BOOLEAN ProcessClassRestriction();
static CONSTRAINT_RECORD *ProcessSlotRestriction();
static VOID IntersectClassBitMaps();
static VOID UnionClassBitMaps();
static CLASS_BITMAP *PackClassBitMap();
static struct lhsParseNode *FilterObjectPattern();
static BITMAP_HN *FormSlotBitMap();
static struct lhsParseNode *RemoveSlotExistenceTests();
static struct lhsParseNode *CreateInitialObjectPattern();
static EXPRESSION *ObjectMatchDelayParse();
#if INCREMENTAL_RESET
static VOID MarkObjectPtnIncrementalReset();
static VOID ObjectIncrementalReset();
#endif
#endif

#endif

#if ! DEFINSTANCES_CONSTRUCT
#if ANSI_COMPILER
static VOID ResetInitialObject(void);
#else
static VOID ResetInitialObject();
#endif
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

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

/********************************************************
  NAME         : SetupObjectPatternStuff
  DESCRIPTION  : Installs the parsers and other items
                 necessary for recognizing and processing
                 object patterns in defrules
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Rete network interfaces for objects
                 initialized
  NOTES        : None
 ********************************************************/
globle VOID SetupObjectPatternStuff()
  {
#if (! BLOAD_ONLY) && (! RUN_TIME)

   if (ReservedPatternSymbol("object",NULL) == CLIPS_TRUE)
     {
      CLIPSSystemError("OBJRTBLD",1);
      ExitCLIPS(5);
     }
   AddReservedPatternSymbol("object",NULL);

   /* ===========================================================================
      The object pattern parser needs to have a higher priority than deftemplates
      or regular facts so that the "object" keyword is always recognized first
      =========================================================================== */
   AddPatternParser("objects",20,&InstanceInfo,PatternParserFind,
                    ObjectLHSParse,ReorderAndAnalyzeObjectPattern,
                    PlaceObjectPattern,DetachObjectPattern,
                    NULL,ReplaceGetJNObjectValue,
                    GenGetJNObjectValue,ObjectJNVariableComparison,
                    GenObjectPNConstantCompare,ReplaceGetPNObjectValue,
                    GenGetPNObjectValue,ObjectPNVariableComparison,
                    DeleteClassBitMap,CopyClassBitMap,
#if INCREMENTAL_RESET
                    MarkObjectPtnIncrementalReset,ObjectIncrementalReset,
#else
                    NULL,NULL,
#endif
                    CreateInitialObjectPattern,
#if CONSTRUCT_COMPILER && (! RUN_TIME)
                    ObjectPatternNodeReference
#else
                    NULL
#endif
                    );
   DefineFunction2("object-pattern-match-delay",'u',
                   PTIF ObjectMatchDelay,"ObjectMatchDelay",NULL);

   AddFunctionParser("object-pattern-match-delay",ObjectMatchDelayParse);
   FuncSeqOvlFlags("object-pattern-match-delay",CLIPS_FALSE,CLIPS_FALSE);

   AddClearFunction("reset-object-match-times",ResetObjectMatchTimeTags,0);

#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
   ObjectPatternsCompilerSetup();
#endif

   AddResetFunction("reset-object-match-times",ResetObjectMatchTimeTags,0);
#if ! DEFINSTANCES_CONSTRUCT
   AddResetFunction("reset-initial-object",ResetInitialObject,0);
#endif
   InstallObjectPrimitives();

#if BLOAD_AND_BSAVE || BLOAD || BLOAD_ONLY
   SetupObjectPatternsBload();
#endif

  }

/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

#if ! DEFINSTANCES_CONSTRUCT

static VOID ResetInitialObject()
  {
   EXPRESSION *tmp;
   DATA_OBJECT rtn;

   tmp = GenConstant(FCALL,(VOID *) FindFunction("make-instance"));
   tmp->argList = GenConstant(INSTANCE_NAME,(VOID *) INITIAL_OBJECT_SYMBOL);
   tmp->argList->nextArg = 
       GenConstant(DEFCLASS_PTR,(VOID *) LookupDefclassInScope(INITIAL_OBJECT_CLASS_NAME));
   EvaluateExpression(tmp,&rtn);
   ReturnExpression(tmp);
  }

#endif
  
#if (! BLOAD_ONLY) && (! RUN_TIME)

/*****************************************************
  NAME         : PatternParserFind
  DESCRIPTION  : Determines if a pattern CE is an
                 object pattern (i.e. the first field
                 is the constant symbol "object")
  INPUTS       : 1) The type of the first field
                 2) The value of the first field
  RETURNS      : CLIPS_TRUE if it is an object pattern,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : Used by AddPatternParser()
 *****************************************************/
static BOOLEAN PatternParserFind(value)
  SYMBOL_HN *value;
  {
   if (strcmp(ValueToString(value),OBJECT_PATTERN_INDICATOR) == 0)
     return(CLIPS_TRUE);
     
   return(CLIPS_FALSE);
  }
    
/************************************************************************************
  NAME         : ObjectLHSParse
  DESCRIPTION  : Scans and parses an object pattern for a rule
  INPUTS       : 1) The logical name of the input source
                 2) A buffer holding the last token read
  RETURNS      : The address of struct lhsParseNodes, NULL on errors
  SIDE EFFECTS : A series of struct lhsParseNodes are created to represent
                 the intermediate parse of the pattern
                 Pretty-print form for the pattern is saved
  NOTES        : Object Pattern Syntax:
                 (object [<class-constraint>] [<name-constraint>] <slot-constraint>*)
                 <class-constraint> ::= (is-a <constraint>)
                 <name-constraint> ::= (name <constraint>)
                 <slot-constraint> ::= (<slot-name> <constraint>*)
 ************************************************************************************/
#if IBM_TBC
#pragma argsused
#endif
static struct lhsParseNode *ObjectLHSParse(readSource,lastToken)
  char *readSource;
  struct token *lastToken;
  {
#if MAC_MPW
#pragma unused(lastToken)
#endif
   struct token theToken;
   struct lhsParseNode *firstNode = NULL,*lastNode = NULL,*tmpNode;
   CLASS_BITMAP *clsset,*tmpset;
   CONSTRAINT_RECORD *slotConstraints;
   int ppbackupReqd = CLIPS_FALSE,multip;
   
   /* ========================================================
      Get a bitmap big enough to mark the ids of all currently
      existing classes - and set all bits, since the initial
      set of applicable classes is everything.
      ======================================================== */
   clsset = NewClassBitMap(((int) MaxClassID) - 1,1);
   if (EmptyClassBitMap(clsset))
     {
      PrintErrorID("OBJRTBLD",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"No objects of existing classes can satisfy pattern.\n");
      DeleteIntermediateClassBitMap(clsset);
      return(NULL);
     }
   tmpset = NewClassBitMap(((int) MaxClassID) - 1,1);

   IncrementIndentDepth(7);
   
   /* ===========================================
      Parse the class, name and slot restrictions
      =========================================== */
   GetToken(readSource,&theToken);
   while (theToken.type != RPAREN)
     {
      ppbackupReqd = CLIPS_TRUE;      
      PPBackup();
      SavePPBuffer(" ");
      SavePPBuffer(theToken.print_rep);
      if (theToken.type != LPAREN)
        {
         SyntaxErrorMessage("object pattern");
         goto ObjectLHSParseERROR;
        }
      GetToken(readSource,&theToken);
      if (theToken.type != SYMBOL)
        {
         SyntaxErrorMessage("object pattern");
         goto ObjectLHSParseERROR;
        }
      if (CheckDuplicateSlots(firstNode,(SYMBOL_HN *) theToken.value))
        goto ObjectLHSParseERROR;
      if (theToken.value == (VOID *) ISA_SYMBOL)
        {
         tmpNode = ParseClassRestriction(readSource,&theToken);
         if (tmpNode == NULL)
           goto ObjectLHSParseERROR;
         InitializeClassBitMap(tmpset,0);
         if (ProcessClassRestriction(tmpset,&tmpNode->bottom,CLIPS_TRUE) == CLIPS_FALSE)
           {
            ReturnLHSParseNodes(tmpNode);
            goto ObjectLHSParseERROR;
           }
         IntersectClassBitMaps(clsset,tmpset);
        }
      else if (theToken.value == (VOID *) NAME_SYMBOL)
        {
         tmpNode = ParseNameRestriction(readSource,&theToken);
         if (tmpNode == NULL)
           goto ObjectLHSParseERROR;
         InitializeClassBitMap(tmpset,1);
        }
      else
        {
         slotConstraints = ProcessSlotRestriction(clsset,(SYMBOL_HN *) theToken.value,&multip);
         if (slotConstraints != NULL)
           {
            InitializeClassBitMap(tmpset,1);
            tmpNode = ParseSlotRestriction(readSource,&theToken,slotConstraints,multip);
            if (tmpNode == NULL)
              goto ObjectLHSParseERROR;
           }
         else
           {
            InitializeClassBitMap(tmpset,0);
            tmpNode = GetLHSParseNode();
            tmpNode->slot = (SYMBOL_HN *) theToken.value;
           }
        }
      if (EmptyClassBitMap(tmpset))
        {
         PrintErrorID("OBJRTBLD",2,CLIPS_FALSE);
         PrintCLIPS(WERROR,"No objects of existing classes can satisfy ");
         PrintCLIPS(WERROR,ValueToString(tmpNode->slot));
         PrintCLIPS(WERROR," restriction in object pattern.\n");
         ReturnLHSParseNodes(tmpNode);
         goto ObjectLHSParseERROR;
        }
      if (EmptyClassBitMap(clsset))
        {
         PrintErrorID("OBJRTBLD",1,CLIPS_FALSE);
         PrintCLIPS(WERROR,"No objects of existing classes can satisfy pattern.\n");
         ReturnLHSParseNodes(tmpNode);
         goto ObjectLHSParseERROR;
        }
      if (tmpNode != NULL)
        {
         if (firstNode == NULL)
           firstNode = tmpNode;
         else
           lastNode->right = tmpNode;
         lastNode = tmpNode;
        }
      PPCRAndIndent();
      GetToken(readSource,&theToken);
     }
   if (firstNode == NULL)
     {
      if (EmptyClassBitMap(clsset))
        {
         PrintErrorID("OBJRTBLD",1,CLIPS_FALSE);
         PrintCLIPS(WERROR,"No objects of existing classes can satisfy pattern.\n");
         goto ObjectLHSParseERROR;
        }
      firstNode = GetLHSParseNode();
      firstNode->type = SF_WILDCARD;
      firstNode->slot = ISA_SYMBOL;
      firstNode->slotNumber = ISA_ID;
      firstNode->index = 1;
     }
   if (ppbackupReqd)
     {
      PPBackup();
      PPBackup();
      SavePPBuffer(theToken.print_rep);
     }
   DeleteIntermediateClassBitMap(tmpset);
   clsset = PackClassBitMap(clsset);
   firstNode->userData = AddBitMap((VOID *) clsset,ClassBitMapSize(clsset));
   IncrementBitMapCount(firstNode->userData);
   DeleteIntermediateClassBitMap(clsset);
   DecrementIndentDepth(7);
   return(firstNode);
   
ObjectLHSParseERROR:
   DeleteIntermediateClassBitMap(clsset);
   DeleteIntermediateClassBitMap(tmpset);
   ReturnLHSParseNodes(firstNode);
   DecrementIndentDepth(7);
   return(NULL);
  }
  
/**************************************************************
  NAME         : ReorderAndAnalyzeObjectPattern
  DESCRIPTION  : This function reexamines the object pattern
                 after constraint and variable analysis info
                 has been propagated from other patterns.
                   Any slots which are no longer applicable
                 to the pattern are eliminated from the
                 class set.
                   Also, the slot names are ordered according
                 to lexical value to aid in deteterming
                 sharing between object patterns.  (The is-a
                 and name restrictions are always placed
                 first regardless of symbolic hash value.)
  INPUTS       : The pattern CE lhsParseNode
  RETURNS      : CLIPS_FALSE if all OK, otherwise CLIPS_TRUE
                 (e.g. all classes are eliminated as potential
                  matching candidates for the pattern)
  SIDE EFFECTS : Slot restrictions are reordered (if necessary)
  NOTES        : Adds a default is-a slot if one does not
                 already exist
 **************************************************************/
static BOOLEAN ReorderAndAnalyzeObjectPattern(topNode)
  struct lhsParseNode *topNode;
  {
   CLASS_BITMAP *clsset,*tmpset;
   EXPRESSION *rexp,*tmpmin,*tmpmax;
   DEFCLASS *cls;
   struct lhsParseNode *tmpNode,*subNode,*bitmap_node,*isa_node,*name_node;
   register unsigned short i;
   SLOT_DESC *sd;
   CONSTRAINT_RECORD *crossConstraints;
   int incompatibleConstraint,clssetChanged = CLIPS_FALSE;
   
   /* ==========================================================
      Make sure that the bitmap marking which classes of object
      can match the pattern is attached to the class restriction
      (which will always be present and the last restriction
      after the sort)
      ========================================================== */
   topNode->right = FilterObjectPattern(topNode->patternType,topNode->right,
                                        &bitmap_node,&isa_node,&name_node);
   if (GetStaticConstraintChecking() == CLIPS_FALSE)
     return(CLIPS_FALSE);
      
   /* ============================================
      Allocate a temporary set for marking classes
      ============================================ */
   clsset = (CLASS_BITMAP *) ValueToBitMap(bitmap_node->userData);
   tmpset = NewClassBitMap((int) clsset->maxid,0);

   /* ==========================================================
      Check the allowed-values for the constraint on the is-a
      slot.  If there are any, make sure that only the classes
      with those values as names are marked in the bitmap.
      
      There will only be symbols in the list because the
      original constraint on the is-a slot allowed only symbols.
      ========================================================== */
   if ((isa_node == NULL) ? CLIPS_FALSE :
       ((isa_node->constraints == NULL) ? CLIPS_FALSE :
        (isa_node->constraints->restrictionList != NULL)))
     {
      rexp = isa_node->constraints->restrictionList;
      while (rexp != NULL)
        {
         cls = LookupDefclassInScope(ValueToString(rexp->value));
         if (cls != NULL)
           {
            if ((cls->id <= clsset->maxid) ? TestBitMap(clsset->map,cls->id) : CLIPS_FALSE)
              SetBitMap(tmpset->map,cls->id);
           }
         rexp = rexp->nextArg;
        }
      clssetChanged = IdenticalClassBitMap(tmpset,clsset) ? CLIPS_FALSE : CLIPS_TRUE;
     }
   else
     CopyMemory(char,tmpset->maxid / BITS_PER_BYTE + 1,tmpset->map,clsset->map);
     
   /* ================================================================
      For each of the slots (excluding name and is-a), check the total
      constraints for the slot against the individual constraints
      for each occurrence of the slot in the classes marked in
      the bitmap.  For any slot which is not compatible with
      the overall constraint, clear its class's bit in the bitmap.
      ================================================================ */
   tmpNode = topNode->right;
   while (tmpNode != bitmap_node)
     {
      if ((tmpNode == isa_node) || (tmpNode == name_node))
        {
         tmpNode = tmpNode->right;
         continue;
        }
      for (i = 0 ; i <= tmpset->maxid ; i++)
        if (TestBitMap(tmpset->map,i))
          {
           cls = ClassIDMap[i];
           sd =  cls->instanceTemplate[FindInstanceTemplateSlot(cls,tmpNode->slot)];
              
           /* =========================================
              Check the top-level lhsParseNode for type
              and cardinality compatibility
              ========================================= */
           crossConstraints = IntersectConstraints(tmpNode->constraints,sd->constraint);
           incompatibleConstraint = UnmatchableConstraint(crossConstraints);
           RemoveConstraint(crossConstraints);
           if (incompatibleConstraint)
             {
              ClearBitMap(tmpset->map,i);
              clssetChanged = CLIPS_TRUE;
             }
           else if (tmpNode->type == MF_WILDCARD)
             {
              /* ==========================================
                 Check the sub-nodes for type compatibility
                 ========================================== */
              for (subNode = tmpNode->bottom ; subNode != NULL ; subNode = subNode->right)
                {
                 /* ========================================================
                    Temporarily reset cardinality of variables to
                    match slot so that no cardinality errors will be flagged
                    ======================================================== */  
                 tmpmin = subNode->constraints->minFields;
                 subNode->constraints->minFields = sd->constraint->minFields;
                 tmpmax = subNode->constraints->maxFields;
                 subNode->constraints->maxFields = sd->constraint->maxFields;
                 crossConstraints = IntersectConstraints(subNode->constraints,
                                                         sd->constraint);
                 subNode->constraints->minFields = tmpmin;
                 subNode->constraints->maxFields = tmpmax;
                 incompatibleConstraint = UnmatchableConstraint(crossConstraints);
                 RemoveConstraint(crossConstraints);
                 if (incompatibleConstraint)
                   {
                    ClearBitMap(tmpset->map,i);
                    clssetChanged = CLIPS_TRUE;
                    break;
                   }
                }
             }
          }
      tmpNode = tmpNode->right;  
     }
     
   if (clssetChanged)
     {
      /* =======================================================
         Make sure that there are still classes of objects which
         can satisfy this pattern.  Otherwise, signal an error.
         ======================================================= */
      if (EmptyClassBitMap(tmpset))
        {
         PrintErrorID("OBJRTBLD",3,CLIPS_TRUE);
         DeleteIntermediateClassBitMap(tmpset);
         PrintCLIPS(WERROR,"No objects of existing classes can satisfy pattern #");
         PrintLongInteger(WERROR,(long) topNode->pattern);
         PrintCLIPS(WERROR,".\n");
         return(CLIPS_TRUE);
        }
      clsset = PackClassBitMap(tmpset);
      DeleteClassBitMap((VOID *) bitmap_node->userData);
      bitmap_node->userData = AddBitMap((VOID *) clsset,ClassBitMapSize(clsset));
      IncrementBitMapCount(bitmap_node->userData);
      DeleteIntermediateClassBitMap(clsset);
     }
   else
     DeleteIntermediateClassBitMap(tmpset);
   return(CLIPS_FALSE);
  }

/*****************************************************
  NAME         : PlaceObjectPattern
  DESCRIPTION  : Integrates an object pattern into the
                 object pattern network
  INPUTS       : The intermediate parse representation
                 of the pattern
  RETURNS      : The address of the new pattern
  SIDE EFFECTS : Object pattern network updated
  NOTES        : None
 *****************************************************/
static struct patternNodeHeader *PlaceObjectPattern(thePattern)
  struct lhsParseNode *thePattern;
  {
   OBJECT_PATTERN_NODE *currentLevel,*lastLevel;
   struct lhsParseNode *tempPattern = NULL;
   OBJECT_PATTERN_NODE *nodeSlotGroup, *newNode;
   OBJECT_ALPHA_NODE *newAlphaNode;
   unsigned endSlot;
   BITMAP_HN *newClassBitMap,*newSlotBitMap;
   
   /* =====================================================
      Get the top of the object pattern network and prepare
      for the traversal to look for shareable pattern nodes
      ===================================================== */
   currentLevel = ObjectNetworkPointer();
   lastLevel = NULL;
   
   /* ==================================================
      Remove slot existence tests from the pattern since
      these are accounted for by the class bitmap
      and find the class and slot bitmaps
      ================================================== */
   newSlotBitMap = FormSlotBitMap(thePattern->right);
   thePattern->right = RemoveSlotExistenceTests(thePattern->right,&newClassBitMap);
   thePattern = thePattern->right;
   
   /* =========================================
      Loop until all fields in the pattern have
      been added to the pattern network
      Process the bitmap node ONLY if it is
      the only node in the pattern
      ========================================= */
   do
     {
      if (thePattern->multifieldSlot)
        {
         tempPattern = thePattern;
         thePattern = thePattern->bottom;
        }
        
      /* ==========================================
         Determine if the last pattern field within
         a multifield slot is being processed.
         ========================================== */
      if ((thePattern->right == NULL) && (tempPattern != NULL)) 
        endSlot = CLIPS_TRUE;
      else 
        endSlot = CLIPS_FALSE;
        
      /* ======================================
         Is there a node in the pattern network
         that can be reused (shared)?
         ====================================== */
      newNode = FindObjectPatternNode(currentLevel,thePattern,&nodeSlotGroup,endSlot);
   
      /* ==============================================
         If the pattern node cannot be shared, then add
         a new pattern node to the pattern network
         ============================================== */
      if (newNode == NULL)
        newNode = CreateNewObjectPatternNode(thePattern,nodeSlotGroup,lastLevel,endSlot);
      
      /* ====================================================
         Move on to the next field in the pattern to be added
         ==================================================== */
      if ((thePattern->right == NULL) && (tempPattern != NULL))
        {
         thePattern = tempPattern;
         tempPattern = NULL;
        }
        
      lastLevel = newNode;
      currentLevel = newNode->nextLevel;
      thePattern = thePattern->right;
     }
   while ((thePattern != NULL) ? (thePattern->userData == NULL) : CLIPS_FALSE);
   
   /* ===============================================
      Return the leaf node of the newly added pattern
      =============================================== */
   newAlphaNode = lastLevel->alphaNode;
   while (newAlphaNode != NULL)
     {
      if ((newClassBitMap == newAlphaNode->classbmp) &&
          (newSlotBitMap == newAlphaNode->slotbmp))
        return((struct patternNodeHeader *) newAlphaNode);
      newAlphaNode = newAlphaNode->nxtInGroup;
     }
   newAlphaNode = get_struct(objectAlphaNode);
   InitializePatternHeader(&newAlphaNode->header);
   newAlphaNode->matchTimeTag = 0L;
   newAlphaNode->patternNode = lastLevel;
   newAlphaNode->classbmp = newClassBitMap;
   IncrementBitMapCount(newClassBitMap);
   MarkBitMapClassesBusy(newClassBitMap,1);
   newAlphaNode->slotbmp = newSlotBitMap;
   if (newSlotBitMap != NULL)
     IncrementBitMapCount(newSlotBitMap);
   newAlphaNode->bsaveID = 0L;
   newAlphaNode->nxtInGroup = lastLevel->alphaNode;
   lastLevel->alphaNode = newAlphaNode;
   newAlphaNode->nxtTerminal = ObjectNetworkTerminalPointer();
   SetObjectNetworkTerminalPointer(newAlphaNode);
   return((struct patternNodeHeader *) newAlphaNode);
  }
  
/************************************************************************
  NAME         : FindObjectPatternNode
  DESCRIPTION  : Looks for a pattern node at a specified
                 level in the pattern network that can be reused (shared)
                 with a pattern field being added to the pattern network.
  INPUTS       : 1) The current layer of nodes being examined in the
                    object pattern network
                 2) The intermediate parse representation of the pattern
                    being added
                 3) A buffer for holding the first node of a group
                    of slots with the same name as the new node
                 4) An integer code indicating if this is the last
                    fiedl in a slot pattern or not
  RETURNS      : The old pattern network node matching the new node, or
                 NULL if there is none (nodeSlotGroup will hold the
                 place where to attach a new node)
  SIDE EFFECTS : nodeSlotGroup set
  NOTES        : None
 ************************************************************************/
static OBJECT_PATTERN_NODE *FindObjectPatternNode(listOfNodes,thePattern,
                                                  nodeSlotGroup,endSlot)
  OBJECT_PATTERN_NODE *listOfNodes;
  struct lhsParseNode *thePattern;
  OBJECT_PATTERN_NODE **nodeSlotGroup;
  unsigned endSlot;
  {
   *nodeSlotGroup = NULL;

   /* ========================================================
      Loop through the nodes at the given level in the pattern
      network looking for a node that can be reused (shared)
      ======================================================== */
   while (listOfNodes != NULL)
     {
      /* =======================================================
         A object pattern node can be shared if the slot name is
         the same, the test is on the same field in the pattern,
         and the network test expressions are the same
         ======================================================= */
      if (((thePattern->type == MF_WILDCARD) || (thePattern->type == MF_VARIABLE)) ?
          listOfNodes->multifieldNode : (listOfNodes->multifieldNode == 0))
        {
         if ((thePattern->slotNumber == (int) listOfNodes->slotNameID) &&
             (thePattern->index == listOfNodes->whichField) && 
             (thePattern->singleFieldsAfter == listOfNodes->leaveFields) &&
             (endSlot == listOfNodes->endSlot) &&
             IdenticalExpression(listOfNodes->networkTest,thePattern->networkTest))
           return(listOfNodes);
        }
      
      /* ============================================
         Find the beginning of a group of nodes with
         the same slot name testing on the same field
         ============================================ */
      if ((*nodeSlotGroup == NULL) &&
          (thePattern->index == listOfNodes->whichField) &&
          (thePattern->slotNumber == (int) listOfNodes->slotNameID))
        *nodeSlotGroup = listOfNodes;
      listOfNodes = listOfNodes->rightNode;
     }

   /* ============================================
      A shareable pattern node could not be found.
      ============================================ */
   return(NULL);
  }

/*****************************************************************
  NAME         : CreateNewObjectPatternNode
  DESCRIPTION  : Creates a new pattern node and initializes
                   all of its values.
  INPUTS       : 1) The intermediate parse representation
                    of the new pattern node
                 2) A pointer to the network node after
                    which to add the new node
                 3) A pointer to the parent node on the
                    level above to link the new node
                 4) An integer code indicating if this is the last
                    fiedl in a slot pattern or not
  RETURNS      : A pointer to the new pattern node
  SIDE EFFECTS : Pattern node allocated, initialized and
                   attached
  NOTES        : None
 *****************************************************************/
static OBJECT_PATTERN_NODE *CreateNewObjectPatternNode(thePattern,nodeSlotGroup,
                                                       upperLevel,endSlot)
  struct lhsParseNode *thePattern;
  OBJECT_PATTERN_NODE *nodeSlotGroup;
  OBJECT_PATTERN_NODE *upperLevel;
  unsigned endSlot;
  {
   OBJECT_PATTERN_NODE *newNode,*prvNode,*curNode;
   
   newNode = get_struct(objectPatternNode);
   newNode->blocked = CLIPS_FALSE;
   newNode->multifieldNode = CLIPS_FALSE;
   newNode->alphaNode = NULL;
   newNode->matchTimeTag = 0L;
   newNode->nextLevel = NULL;
   newNode->rightNode = NULL;
   newNode->leftNode = NULL;
   newNode->bsaveID = 0L;
   
   /* ========================================================
      Install the expression associated with this pattern node
      ======================================================== */
   newNode->networkTest = AddHashedExpression((EXPRESSION *) thePattern->networkTest);
   newNode->whichField = thePattern->index;
   newNode->leaveFields = thePattern->singleFieldsAfter;
   
   /* ======================================
      Install the slot name for the new node
      ====================================== */
   newNode->slotNameID = (unsigned) thePattern->slotNumber;
   if ((thePattern->type == MF_WILDCARD) || (thePattern->type == MF_VARIABLE))
     newNode->multifieldNode = CLIPS_TRUE;
   newNode->endSlot = endSlot;
   
   /* ============================================
      Set the upper level pointer for the new node
      ============================================ */
   newNode->lastLevel = upperLevel;
   
   /* ============================================ 
      If there are no nodes with this slot name on
      this level, simply prepend it to the front
      ============================================ */
   if (nodeSlotGroup == NULL)
     {
      if (upperLevel == NULL)
        {
         newNode->rightNode = ObjectNetworkPointer();
         SetObjectNetworkPointer(newNode);
        }
      else
        {
         newNode->rightNode = upperLevel->nextLevel;
         upperLevel->nextLevel = newNode;
        }
      if (newNode->rightNode != NULL)
        newNode->rightNode->leftNode = newNode;
      return(newNode);
     }
     
   /* ===========================================================
      Group this node with other nodes of the same name
      testing on the same field in the pattern
      on this level.  This allows us to do some optimization
      with constant tests on a particular slots.  If we put
      all constant tests for a particular slot/field group at the
      end of that group, then when one of those test succeeds
      during pattern-matching, we don't have to test any
      more of the nodes with that slot/field name to the right.
      =========================================================== */
   prvNode = NULL;
   curNode = nodeSlotGroup;
   while ((curNode == NULL) ? CLIPS_FALSE :
          (curNode->slotNameID == nodeSlotGroup->slotNameID) &&
          (curNode->whichField == nodeSlotGroup->whichField))
     {
      if ((curNode->networkTest == NULL) ? CLIPS_FALSE :
          ((curNode->networkTest->type != OBJ_PN_CONSTANT) ? CLIPS_FALSE :
           ((struct ObjectCmpPNConstant *) ValueToBitMap(curNode->networkTest->value))->pass))
        break;
      prvNode = curNode;
      curNode = curNode->rightNode;
     }   
   if (curNode != NULL)
     {
      newNode->leftNode = curNode->leftNode;
      newNode->rightNode = curNode;
      if (curNode->leftNode != NULL)
        curNode->leftNode->rightNode = newNode;
      else if (curNode->lastLevel != NULL)
        curNode->lastLevel->nextLevel = newNode;
      else
        SetObjectNetworkPointer(newNode);
      curNode->leftNode = newNode;
     }
   else
     {
      newNode->leftNode = prvNode;
      prvNode->rightNode = newNode;
     }
   
   return(newNode);
  }

/********************************************************
  NAME         : DetachObjectPattern
  DESCRIPTION  : Removes a pattern node and all of its  
   parent nodes from the pattern network. Nodes are only   
   removed if they are no longer shared (i.e. a pattern    
   node that has more than one child node is shared). A    
   pattern from a rule is typically removed by removing    
   the bottom most pattern node used by the pattern and    
   then removing any parent nodes which are not shared by  
   other patterns.                                         
                                                           
   Example:                                                
     Patterns (a b c d) and (a b e f) would be represented 
     by the pattern net shown on the left.  If (a b c d)   
     was detached, the resultant pattern net would be the  
     one shown on the right. The '=' represents an         
     end-of-pattern node.                                  
                                                           
           a                  a                            
           |                  |                            
           b                  b                            
           |                  |                            
           c--e               e                            
           |  |               |                            
           d  f               f                            
           |  |               |                            
           =  =               =           
  INPUTS       : The pattern to be removed
  RETURNS      : Nothing useful
  SIDE EFFECTS : All non-shared nodes associated with the
                 pattern are removed
  NOTES        : None
 ********************************************************/                 
static VOID DetachObjectPattern(thePattern)
  struct patternNodeHeader *thePattern;
  {
   OBJECT_ALPHA_NODE *alphaPtr,*prv,*terminalPtr;
   OBJECT_PATTERN_NODE *patternPtr,*upperLevel;
   
   /* ==================================================
      Get rid of any matches stored in the alpha memory.
      ================================================== */
   alphaPtr = (OBJECT_ALPHA_NODE *) thePattern;
   ClearObjectPatternMatches(alphaPtr);
   
   /* =======================================
      Unmark the classes to which the pattern
      is applicable and unmark the class and
      slot id maps so that they can become
      ephemeral
      ======================================= */
   MarkBitMapClassesBusy(alphaPtr->classbmp,-1);
   DeleteClassBitMap(alphaPtr->classbmp);
   if (alphaPtr->slotbmp != NULL)
     DecrementBitMapCount(alphaPtr->slotbmp);
   
   /* ======================================
      Only continue deleting this pattern if
      this is the last alpha memory attached
      ====================================== */
   prv = NULL;
   terminalPtr = ObjectNetworkTerminalPointer();
   while (terminalPtr != alphaPtr)
     {
      prv = terminalPtr;
      terminalPtr = terminalPtr->nxtTerminal;
     }
   if (prv == NULL)
     SetObjectNetworkTerminalPointer(terminalPtr->nxtTerminal);
   else
     prv->nxtTerminal = terminalPtr->nxtTerminal;
   
   prv = NULL;
   terminalPtr = alphaPtr->patternNode->alphaNode;
   while (terminalPtr != alphaPtr)
     {
      prv = terminalPtr;
      terminalPtr = terminalPtr->nxtInGroup;
     }
   if (prv == NULL)
     {
      if (alphaPtr->nxtInGroup != NULL)
        {
         alphaPtr->patternNode->alphaNode = alphaPtr->nxtInGroup;
         rtn_struct(objectAlphaNode,alphaPtr);
         return;
        }
     }
   else
     {
      prv->nxtInGroup = alphaPtr->nxtInGroup;
      rtn_struct(objectAlphaNode,alphaPtr);
      return;
     }
   alphaPtr->patternNode->alphaNode = NULL;
   rtn_struct(objectAlphaNode,alphaPtr);
   
   upperLevel = alphaPtr->patternNode;
   if (upperLevel->nextLevel != NULL)
     return;
     
   /*==============================================================*/
   /* Loop until all appropriate pattern nodes have been detached. */
   /*==============================================================*/
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
           SetObjectNetworkPointer(NULL);
         else
           {
            upperLevel->nextLevel = NULL;
            if (upperLevel->alphaNode != NULL)
              upperLevel = NULL;
           }
         RemoveHashedExpression((EXPRESSION *) patternPtr->networkTest);
         rtn_struct(objectPatternNode,patternPtr);
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

         RemoveHashedExpression((EXPRESSION *) patternPtr->networkTest);
         rtn_struct(objectPatternNode,patternPtr);
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
           { SetObjectNetworkPointer(patternPtr->rightNode); }
         else
           { upperLevel->nextLevel = patternPtr->rightNode; }
         patternPtr->rightNode->leftNode = NULL;

         RemoveHashedExpression((EXPRESSION *) patternPtr->networkTest);
         rtn_struct(objectPatternNode,patternPtr);
         upperLevel = NULL;
        }
     }
  }

/***************************************************
  NAME         : ClearObjectPatternMatches
  DESCRIPTION  : Removes a pattern node alpha memory
                 from the list of partial matches
                 on all instances (active or
                 garbage collected)
  INPUTS       : The pattern node to remove
  RETURNS      : Nothing useful
  SIDE EFFECTS : Pattern alpha memory removed
                 from all object partial match lists
  NOTES        : Used when a pattern is removed
 ***************************************************/
static VOID ClearObjectPatternMatches(alphaPtr)
  OBJECT_ALPHA_NODE *alphaPtr;
  {
   INSTANCE_TYPE *ins;
   IGARBAGE *igrb;

   /* =============================================
      Loop through every active and queued instance
      ============================================= */   
   ins = InstanceList;
   while (ins != NULL)
     {
      RemoveObjectPartialMatches((VOID *) ins,(struct patternNodeHeader *) alphaPtr);
      ins = ins->nxtList;
     }
     
   /* ============================
      Check for garbaged instances
      ============================ */
   igrb = InstanceGarbageList;
   while (igrb != NULL)
     {
      RemoveObjectPartialMatches((VOID *) igrb->ins,(struct patternNodeHeader *) alphaPtr);
      igrb = igrb->nxt;
     }
  }
  
/***************************************************
  NAME         : RemoveObjectPartialMatches
  DESCRIPTION  : Removes a partial match from a
                 list of partial matches for
                 an instance
  INPUTS       : 1) The instance
                 2) The pattern node header
                    corresponding to the match
  RETURNS      : Nothing useful
  SIDE EFFECTS : Match removed
  NOTES        : None
 ***************************************************/
static VOID RemoveObjectPartialMatches(ins,phead)
  INSTANCE_TYPE *ins;
  struct patternNodeHeader *phead;
  {
   struct patternMatch *match_before, *match_ptr;
  
   match_before = NULL;
   match_ptr = (struct patternMatch *) ins->partialMatchList;

   /* ======================================= 
      Loop through every match for the object
      ======================================= */
   while (match_ptr != NULL)
     {
      if (match_ptr->matchingPattern == phead)
        {
         ins->busy--;
         if (match_before == NULL)
           {
            ins->partialMatchList = (VOID *) match_ptr->next;
            rtn_struct(patternMatch,match_ptr);
            match_ptr = (struct patternMatch *) ins->partialMatchList;
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
        match_before = match_ptr;
        match_ptr = match_ptr->next;
       }
     }
  }

/******************************************************
  NAME         : CheckDuplicateSlots
  DESCRIPTION  : Determines if a restriction has
                 already been defined in a pattern
  INPUTS       : The list of already built restrictions
  RETURNS      : CLIPS_TRUE if a definition already
                 exists, CLIPS_FALSE otherwise
  SIDE EFFECTS : An error message is printed if a
                 duplicate is found
  NOTES        : None
 ******************************************************/
static BOOLEAN CheckDuplicateSlots(nodeList,slotName)
  struct lhsParseNode *nodeList;
  SYMBOL_HN *slotName;
  {
   while (nodeList != NULL)
     {
      if (nodeList->slot == slotName)
        {
         PrintErrorID("OBJRTBLD",4,CLIPS_TRUE);
         PrintCLIPS(WERROR,"Multiple restrictions on attribute ");
         PrintCLIPS(WERROR,ValueToString(slotName));
         PrintCLIPS(WERROR," not allowed.\n");
         return(CLIPS_TRUE);
        }
      nodeList = nodeList->right;
     }
   return(CLIPS_FALSE);
  }
  
/**********************************************************
  NAME         : ParseClassRestriction
  DESCRIPTION  : Parses the single-field constraint
                  on the class an object pattern
  INPUTS       : 1) The logical input source
                 2) A buffer for tokens
  RETURNS      : The intermediate pattern nodes 
                 representing the class constraint
                 (NULL on errors)
  SIDE EFFECTS : Intermediate pattern nodes allocated
  NOTES        : None
 **********************************************************/
static struct lhsParseNode *ParseClassRestriction(readSource,theToken)
  char *readSource;
  struct token *theToken;
  {
   struct lhsParseNode *tmpNode;
   SYMBOL_HN *rln;
   CONSTRAINT_RECORD *rv;
   
   rv = GetConstraintRecord();
   rv->anyAllowed = 0;
   rv->symbolsAllowed = 1;
   rln = (SYMBOL_HN *) theToken->value;
   SavePPBuffer(" ");
   GetToken(readSource,theToken);
   tmpNode = RestrictionParse(readSource,theToken,CLIPS_FALSE,rln,ISA_ID,rv,0);
   if (tmpNode == NULL)
     {
      RemoveConstraint(rv);
      return(NULL);
     }
   if ((theToken->type != RPAREN) || 
       (tmpNode->type == MF_WILDCARD) || 
       (tmpNode->type == MF_VARIABLE))
     {
      PPBackup();
      if (theToken->type != RPAREN)
        {
         SavePPBuffer(" ");
         SavePPBuffer(theToken->print_rep);
        }
      SyntaxErrorMessage("class restriction in object pattern");
      ReturnLHSParseNodes(tmpNode);
      RemoveConstraint(rv);
      return(NULL);
     }
   tmpNode->derivedConstraints = 1;
   return(tmpNode);
  }
  
/**********************************************************
  NAME         : ParseNameRestriction
  DESCRIPTION  : Parses the single-field constraint
                  on the name of an object pattern
  INPUTS       : 1) The logical input source
                 2) A buffer for tokens
  RETURNS      : The intermediate pattern nodes 
                 representing the name constraint
                 (NULL on errors)
  SIDE EFFECTS : Intermediate pattern nodes allocated
  NOTES        : None
 **********************************************************/
static struct lhsParseNode *ParseNameRestriction(readSource,theToken)
  char *readSource;
  struct token *theToken;
  {
   struct lhsParseNode *tmpNode;
   SYMBOL_HN *rln;
   CONSTRAINT_RECORD *rv;
   
   rv = GetConstraintRecord();
   rv->anyAllowed = 0;
   rv->instanceNamesAllowed = 1;
   rln = (SYMBOL_HN *) theToken->value;
   SavePPBuffer(" ");
   GetToken(readSource,theToken);
   tmpNode = RestrictionParse(readSource,theToken,CLIPS_FALSE,rln,NAME_ID,rv,0);
   if (tmpNode == NULL)
     {
      RemoveConstraint(rv);
      return(NULL);
     }
   if ((theToken->type != RPAREN) || 
       (tmpNode->type == MF_WILDCARD) || 
       (tmpNode->type == MF_VARIABLE))
     {
      PPBackup();                        
      if (theToken->type != RPAREN)
        {
         SavePPBuffer(" ");
         SavePPBuffer(theToken->print_rep);
        }
      SyntaxErrorMessage("name restriction in object pattern");
      ReturnLHSParseNodes(tmpNode);
      RemoveConstraint(rv);
      return(NULL);
     }
   
   tmpNode->derivedConstraints = 1;
   return(tmpNode);
  }
  
/***************************************************
  NAME         : ParseSlotRestriction
  DESCRIPTION  : Parses the field constraint(s)
                  on a slot of an object pattern
  INPUTS       : 1) The logical input source
                 2) A buffer for tokens
                 3) Constraint record holding the
                    unioned constraints of all the
                    slots which could match the
                    slot pattern
                 4) A flag indicating if any
                    multifield slots match the name
  RETURNS      : The intermediate pattern nodes 
                 representing the slot constraint(s)
                 (NULL on errors)
  SIDE EFFECTS : Intermediate pattern nodes
                 allocated
  NOTES        : None
 ***************************************************/
static struct lhsParseNode *ParseSlotRestriction(readSource,theToken,slotConstraints,multip)
  char *readSource;
  struct token *theToken;
  CONSTRAINT_RECORD *slotConstraints;
  int multip;
  {
   struct lhsParseNode *tmpNode;
   SYMBOL_HN *slotName;
   
   slotName = (SYMBOL_HN *) theToken->value;
   SavePPBuffer(" ");
   GetToken(readSource,theToken);
   tmpNode = RestrictionParse(readSource,theToken,multip,slotName,FindSlotNameID(slotName),
                              slotConstraints,1);
   if (tmpNode == NULL)
     {
      RemoveConstraint(slotConstraints);
      return(NULL);
     }
   if (theToken->type != RPAREN)
     {
      PPBackup();                        
      SavePPBuffer(" ");                  
      SavePPBuffer(theToken->print_rep); 
      SyntaxErrorMessage("object slot pattern");
      ReturnLHSParseNodes(tmpNode);
      RemoveConstraint(slotConstraints);
      return(NULL);
     }
   if ((tmpNode->bottom == NULL) && (tmpNode->multifieldSlot))
     {
      PPBackup();
      PPBackup();
      SavePPBuffer(")");
     }
   tmpNode->derivedConstraints = 1;     
   return(tmpNode);
  }
  
/********************************************************
  NAME         : NewClassBitMap
  DESCRIPTION  : Creates a new bitmap large enough
                 to hold all ids of classes in the
                 system and initializes all the bits
                 to zero or one.
  INPUTS       : 1) The maximum id that will be set
                    in the bitmap
                 2) An integer code indicating if all
                    the bits are to be set to zero or one
  RETURNS      : The new bitmap
  SIDE EFFECTS : BitMap allocated and initialized
  NOTES        : None
 ********************************************************/
static CLASS_BITMAP *NewClassBitMap(maxid,set)
  int maxid,set;
  {
   register CLASS_BITMAP *bmp;
   int size;
   
   if (maxid == -1)
     maxid = 0;
   size = (int) (sizeof(CLASS_BITMAP) +
                (sizeof(char) * (maxid / BITS_PER_BYTE)));
   bmp = (CLASS_BITMAP *) gm2(size);
   ClearBitString((VOID *) bmp,size);
   bmp->maxid = (unsigned short) maxid;
   InitializeClassBitMap(bmp,set);
   return(bmp);
  }

/***********************************************************
  NAME         : InitializeClassBitMap
  DESCRIPTION  : Initializes a bitmap to all zeroes or ones.
  INPUTS       : 1) The bitmap
                 2) An integer code indicating if all
                    the bits are to be set to zero or one
  RETURNS      : Nothing useful
  SIDE EFFECTS : The bitmap is initialized
  NOTES        : None
 ***********************************************************/
static VOID InitializeClassBitMap(bmp,set)
  CLASS_BITMAP *bmp;
  int set;
  {
   register int i,bytes;
   DEFCLASS *cls;
   struct defmodule *currentModule;
   
   bytes = bmp->maxid / BITS_PER_BYTE + 1;
   while (bytes > 0)
     {
      bmp->map[bytes - 1] = (char) 0;
      bytes--;
     }
   if (set)
     {
      currentModule = ((struct defmodule *) GetCurrentModule());
      for (i = 0 ; i <= bmp->maxid ; i++)
        {
         cls = ClassIDMap[i];
         if ((cls != NULL) ? DefclassInScope(cls,currentModule) : CLIPS_FALSE)
           {
            if (cls->reactive && (cls->abstract == 0))
              SetBitMap(bmp->map,i);
           }
        }
     }
  }
  
/********************************************
  NAME         : DeleteIntermediateClassBitMap
  DESCRIPTION  : Deallocates a bitmap
  INPUTS       : The class set
  RETURNS      : Nothing useful
  SIDE EFFECTS : Class set deallocated
  NOTES        : None
 ********************************************/
static VOID DeleteIntermediateClassBitMap(bmp)
  CLASS_BITMAP *bmp;
  {
   rm((VOID *) bmp,ClassBitMapSize(bmp));
  }
  
/******************************************************
  NAME         : CopyClassBitMap
  DESCRIPTION  : Increments the in use count of a
                 bitmap and returns the same pointer
  INPUTS       : The bitmap
  RETURNS      : The bitmap
  SIDE EFFECTS : Increments the in use count
  NOTES        : Class sets are shared by multiple
                 copies of an object pattern within an
                 OR CE.  The use count prevents having
                 to make duplicate copies of the bitmap
 ******************************************************/
static VOID *CopyClassBitMap(gset)
  VOID *gset;
  {
   if (gset != NULL)
     IncrementBitMapCount(gset);
   return(gset);
  }
  
/**********************************************************
  NAME         : DeleteClassBitMap
  DESCRIPTION  : Deallocates a bitmap,
                 and decrements the busy flags of the
                 classes marked in the bitmap
  INPUTS       : The bitmap
  RETURNS      : Nothing useful
  SIDE EFFECTS : Class set deallocated and classes unmarked
  NOTES        : None
 **********************************************************/
static VOID DeleteClassBitMap(gset)
  VOID *gset;
  {
   
   if (gset == NULL)
     return;
   DecrementBitMapCount(gset);
  }

/***************************************************
  NAME         : MarkBitMapClassesBusy
  DESCRIPTION  : Increments/Decrements busy counts
                 of all classes marked in a bitmap
  INPUTS       : 1) The bitmap hash node
                 2) 1 or -1 (to increment or
                    decrement class busy counts)
  RETURNS      : Nothing useful
  SIDE EFFECTS : Bitmap class busy counts updated
  NOTES        : None
 ***************************************************/
static VOID MarkBitMapClassesBusy(bmphn,offset)
  BITMAP_HN *bmphn;
  int offset;
  {
   register CLASS_BITMAP *bmp;
   register unsigned short i;
   register DEFCLASS *cls;

   /* ====================================
      If a clear is in progress, we do not
      have to worry about busy counts
      ==================================== */
   if (ClearInProgress)
     return;
   bmp = (CLASS_BITMAP *) ValueToBitMap(bmphn);
   for (i = 0 ; i <= bmp->maxid ; i++)
     if (TestBitMap(bmp->map,i))
       {
        cls =  ClassIDMap[i];
        cls->busy += offset;
       }
  }
   
/****************************************************
  NAME         : EmptyClassBitMap
  DESCRIPTION  : Determines if one or more bits
                 are marked in a bitmap
  INPUTS       : The bitmap
  RETURNS      : CLIPS_TRUE if the set has no bits
                 marked, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ****************************************************/
static BOOLEAN EmptyClassBitMap(bmp)
  CLASS_BITMAP *bmp;
  {
   register unsigned short bytes;
   
   bytes = bmp->maxid / BITS_PER_BYTE + 1;
   while (bytes > 0)
     {
      if (bmp->map[bytes - 1] != (char) 0)
        return(CLIPS_FALSE);
      bytes--;
     }
   return(CLIPS_TRUE);
  }

/***************************************************
  NAME         : IdenticalClassBitMap
  DESCRIPTION  : Determines if two bitmaps
                 are identical
  INPUTS       : 1) First bitmap
                 2) Second bitmap
  RETURNS      : CLIPS_TRUE if bitmaps are the same,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
static BOOLEAN IdenticalClassBitMap(cs1,cs2)
  CLASS_BITMAP *cs1,*cs2;
  {
   register int i;
   
   if (cs1->maxid != cs2->maxid)
     return(CLIPS_FALSE);
   for (i = 0 ; i < (cs1->maxid / BITS_PER_BYTE + 1) ; i++)
     if (cs1->map[i] != cs2->map[i])
       return(CLIPS_FALSE);
   return(CLIPS_TRUE);
  }
  
/*****************************************************************
  NAME         : ProcessClassRestriction
  DESCRIPTION  : Examines a class restriction and forms a bitmap
                 corresponding to the maximal set of classes which
                 can satisfy a static analysis of the restriction
  INPUTS       : 1) The bitmap to mark classes in
                 2) The lhsParseNodes of the restriction
                 3) A flag indicating if this is the first
                    non-recursive call or not
  RETURNS      : CLIPS_TRUE if all OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : Class bitmap set and lhsParseNodes corressponding
                 to constant restrictions are removed
  NOTES        : None
 *****************************************************************/
static BOOLEAN ProcessClassRestriction(clsset,classRestrictions,recursiveCall)
  CLASS_BITMAP *clsset;
  struct lhsParseNode **classRestrictions;
  int recursiveCall;
  {
   register struct lhsParseNode *chk,**oraddr;
   CLASS_BITMAP *tmpset1,*tmpset2;
   int constant_restriction = CLIPS_TRUE;
   
   if (*classRestrictions == NULL)
     {
      if (recursiveCall)
        InitializeClassBitMap(clsset,1);
      return(CLIPS_TRUE);
     }

   /* ===============================================
      Determine the corresponding class set and union
      it with the current total class set.  If an AND
      restriction is comprised entirely of symbols,
      it can be removed
      =============================================== */
   tmpset1 = NewClassBitMap(((int) MaxClassID) - 1,1);
   tmpset2 = NewClassBitMap(((int) MaxClassID) - 1,0);
   for  (chk = *classRestrictions ; chk != NULL ; chk = chk->right)
     {
      if (chk->type == SYMBOL)
        {
         chk->value = (VOID *) LookupDefclassInScope(ValueToString(chk->value));
         if (chk->value == NULL)
           {
            PrintErrorID("OBJRTBLD",5,CLIPS_FALSE);
            PrintCLIPS(WERROR,"Undefined class in object pattern.\n");
            DeleteIntermediateClassBitMap(tmpset1);
            DeleteIntermediateClassBitMap(tmpset2);
            return(CLIPS_FALSE);
           }
         if (chk->negated)
           {
            InitializeClassBitMap(tmpset2,1);
            MarkBitMapSubclasses(tmpset2->map,(DEFCLASS *) chk->value,0);
           }
         else
           {
            InitializeClassBitMap(tmpset2,0);
            MarkBitMapSubclasses(tmpset2->map,(DEFCLASS *) chk->value,1);
           }
         IntersectClassBitMaps(tmpset1,tmpset2);
        }
      else
        constant_restriction = CLIPS_FALSE;
     }
   if (EmptyClassBitMap(tmpset1))
     {
      PrintErrorID("OBJRTBLD",2,CLIPS_FALSE);
      PrintCLIPS(WERROR,"No objects of existing classes can satisfy ");
      PrintCLIPS(WERROR,"is-a restriction in object pattern.\n");
      DeleteIntermediateClassBitMap(tmpset1);
      DeleteIntermediateClassBitMap(tmpset2);
      return(CLIPS_FALSE);
     }
   if (constant_restriction)
     {
      chk = *classRestrictions;
      *classRestrictions = chk->bottom;
      chk->bottom = NULL;
      ReturnLHSParseNodes(chk);
      oraddr = classRestrictions;
     }
   else
     oraddr = &(*classRestrictions)->bottom;
   UnionClassBitMaps(clsset,tmpset1);
   DeleteIntermediateClassBitMap(tmpset1);
   DeleteIntermediateClassBitMap(tmpset2);
     
   /* =====================================
      Process the next OR class restriction
      ===================================== */
   return(ProcessClassRestriction(clsset,oraddr,CLIPS_FALSE));
  }
  
/****************************************************************
  NAME         : ProcessSlotRestriction
  DESCRIPTION  : Determines which slots could match the slot
                 pattern and determines the union of all
                 constraints for the pattern
  INPUTS       : 1) The class set
                 2) The slot name
                 3) A buffer to hold a flag indicating if
                    any multifield slots are found w/ this name
  RETURNS      : A union of the constraints on all the slots
                 which could match the slots (NULL if no
                 slots found)
  SIDE EFFECTS : The class bitmap set is marked/cleared
  NOTES        : None
 ****************************************************************/
static CONSTRAINT_RECORD *ProcessSlotRestriction(clsset,slotName,multip)
  CLASS_BITMAP *clsset;
  SYMBOL_HN *slotName;
  int *multip;
  {
   register DEFCLASS *cls;
   register int si;
   CONSTRAINT_RECORD *totalConstraints = NULL,*tmpConstraints;
   register unsigned i;
   
   *multip = CLIPS_FALSE;
   for (i = 0 ; i < CLASS_TABLE_HASH_SIZE ; i++)
     for (cls = ClassTable[i] ; cls != NULL ; cls = cls->nxtHash)
       {
        if (TestBitMap(clsset->map,cls->id))
          {
           si = FindInstanceTemplateSlot(cls,slotName);
           if ((si != -1) ? cls->instanceTemplate[si]->reactive : CLIPS_FALSE)
             {
              if (cls->instanceTemplate[si]->multiple)
                *multip = CLIPS_TRUE;
              tmpConstraints = 
                 UnionConstraints(cls->instanceTemplate[si]->constraint,totalConstraints);
              RemoveConstraint(totalConstraints);
              totalConstraints = tmpConstraints;
             }
           else
             ClearBitMap(clsset->map,cls->id);
          }
       }
   return(totalConstraints);
  }
  
/****************************************************
  NAME         : IntersectClassBitMaps
  DESCRIPTION  : Bitwise-ands two bitmaps and stores
                 the result in the first
  INPUTS       : The two bitmaps
  RETURNS      : Nothing useful
  SIDE EFFECTS : ClassBitMaps anded
  NOTES        : Assumes the first bitmap is at least
                 as large as the second
 ****************************************************/
static VOID IntersectClassBitMaps(cs1,cs2)
  CLASS_BITMAP *cs1,*cs2;
  {
   register unsigned short bytes;
   
   bytes = cs2->maxid / BITS_PER_BYTE + 1;
   while (bytes > 0)
     {
      cs1->map[bytes - 1] &= cs2->map[bytes - 1];
      bytes--;
     }
  }
  
/****************************************************
  NAME         : UnionClassBitMaps
  DESCRIPTION  : Bitwise-ors two bitmaps and stores
                 the result in the first
  INPUTS       : The two bitmaps
  RETURNS      : Nothing useful
  SIDE EFFECTS : ClassBitMaps ored
  NOTES        : Assumes the first bitmap is at least
                 as large as the second
 ****************************************************/
static VOID UnionClassBitMaps(cs1,cs2)
  CLASS_BITMAP *cs1,*cs2;
  {
   register unsigned short bytes;
   
   bytes = cs2->maxid / BITS_PER_BYTE + 1;
   while (bytes > 0)
     {
      cs1->map[bytes - 1] |= cs2->map[bytes - 1];
      bytes--;
     }
  }
  
/*****************************************************
  NAME         : PackClassBitMap
  DESCRIPTION  : This routine packs a bitmap
                 bitmap such that at least one of
                 the bits in the rightmost byte is
                 set (i.e. the bitmap takes up
                 the smallest space possible).
  INPUTS       : The bitmap
  RETURNS      : The new (packed) bitmap
  SIDE EFFECTS : The oldset is deallocated
  NOTES        : None
 *****************************************************/
static CLASS_BITMAP *PackClassBitMap(oldset)
  CLASS_BITMAP *oldset;
  {
   register unsigned short newmaxid;
   CLASS_BITMAP *newset;
   
   for (newmaxid = oldset->maxid ; newmaxid > 0 ; newmaxid--)
     if (TestBitMap(oldset->map,newmaxid))
       break;
   if (newmaxid != oldset->maxid)
     {
      newset = NewClassBitMap((int) newmaxid,0);
      CopyMemory(char,newmaxid / BITS_PER_BYTE + 1,newset->map,oldset->map);
      DeleteIntermediateClassBitMap(oldset);
     }
   else
     newset = oldset;
   return(newset);
  }
    
/*****************************************************************
  NAME         : FilterObjectPattern
  DESCRIPTION  : Appends an extra node to hold the bitmap,
                 and finds is-a and name nodes
  INPUTS       : 1) The object pattern parser address
                    to give to a default is-a slot
                 2) The unfiltered slot list
                 3) A buffer to hold the address of
                    the class bitmap restriction node
                 4) A buffer to hold the address of
                    the is-a restriction node
                 4) A buffer to hold the address of
                    the name restriction node
  RETURNS      : The filtered slot list
  SIDE EFFECTS : clsset is attached to extra slot pattern
                 Pointers to the is-a and name slots are also
                 stored (if they exist) for easy reference
  NOTES        : None
 *****************************************************************/
static struct lhsParseNode *FilterObjectPattern(selfPatternType,unfilteredSlots,
                                                bitmap_slot,isa_slot,name_slot)
  struct patternParser *selfPatternType;
  struct lhsParseNode *unfilteredSlots,**bitmap_slot,**isa_slot,**name_slot;
  {
   struct lhsParseNode *prv,*cur;
   
   *isa_slot = NULL;
   *name_slot = NULL;
   
   /* ============================================
      Create a dummy node to attach to the end 
      of the pattern which holds the class bitmap.
      ============================================ */
   *bitmap_slot = GetLHSParseNode();
   (*bitmap_slot)->type = SF_WILDCARD;
   (*bitmap_slot)->slot = ISA_SYMBOL;
   (*bitmap_slot)->slotNumber = ISA_ID;
   (*bitmap_slot)->index = 1;
   (*bitmap_slot)->patternType = selfPatternType;
   (*bitmap_slot)->userData = unfilteredSlots->userData;
   unfilteredSlots->userData = NULL;
     
   /* ========================
      Find is-a and name nodes
      ======================== */
   prv = NULL;
   cur = unfilteredSlots;
   while (cur != NULL)
     {
      if (cur->slot == ISA_SYMBOL)
        *isa_slot = cur;
      else if (cur->slot == NAME_SYMBOL)
        *name_slot = cur;
      prv = cur;
      cur = cur->right;
     }
     
   /* ================================
      Add the class bitmap conditional
      element to end of pattern
      ================================ */
   if (prv == NULL)
     unfilteredSlots = *bitmap_slot;
   else
     prv->right = *bitmap_slot;
   return(unfilteredSlots);
  }
  
/***************************************************
  NAME         : FormSlotBitMap
  DESCRIPTION  : Examines an object pattern and
                 forms a minimal bitmap marking
                 the ids of the slots used in
                 the pattern
  INPUTS       : The intermediate parsed pattern
  RETURNS      : The new slot bitmap (can be NULL)
  SIDE EFFECTS : Bitmap created and added to hash
                 table - corresponding bits set
                 for ids of slots used in pattern
  NOTES        : None
 ***************************************************/
static BITMAP_HN *FormSlotBitMap(thePattern)
  struct lhsParseNode *thePattern;
  {
   struct lhsParseNode *node;
   int maxSlotID = -1,size;
   SLOT_BITMAP *bmp;
   BITMAP_HN *hshBmp;
   
   /* =======================================
      Find the largest slot id in the pattern
      ======================================= */
   for (node = thePattern ; node != NULL ; node = node->right)
     if (node->slotNumber > maxSlotID)
       maxSlotID = node->slotNumber;
       
   /* ===================================================
      If the pattern contains no slot tests or only tests
      on the class or name (which do not change) do not
      store a slot bitmap
      =================================================== */
   if ((maxSlotID == ISA_ID) || (maxSlotID == NAME_ID))
     return(NULL);
     
   /* ===================================
      Initialize the bitmap to all zeroes
      =================================== */
   size = (int) (sizeof(SLOT_BITMAP) +
                (sizeof(char) * (maxSlotID / BITS_PER_BYTE)));
   bmp = (SLOT_BITMAP *) gm2(size);
   ClearBitString((VOID *) bmp,size);
   bmp->maxid = (unsigned short) maxSlotID;

   /* ============================================
      Add (retrieve) a bitmap to (from) the bitmap
      hash table which has a corresponding bit set
      for the id of every slot used in the pattern
      ============================================ */
   for (node = thePattern ; node != NULL ; node = node->right)
     SetBitMap(bmp->map,node->slotNumber);
   hshBmp = (BITMAP_HN *) AddBitMap((VOID *) bmp,SlotBitMapSize(bmp));
   rm((VOID *) bmp,size);
   return(hshBmp);
  }

/****************************************************
  NAME         : RemoveSlotExistenceTests
  DESCRIPTION  : Removes slot existence test since
                 these are accounted for by class
                 bitmap or name slot.
  INPUTS       : 1) The intermediate pattern nodes
                 2) A buffer to hold the class bitmap
  RETURNS      : The filtered list
  SIDE EFFECTS : Slot existence tests removed
  NOTES        : None
 ****************************************************/
static struct lhsParseNode *RemoveSlotExistenceTests(thePattern,bmp)
  struct lhsParseNode *thePattern;
  BITMAP_HN **bmp;
  {
   struct lhsParseNode *tempPattern = thePattern;
   struct lhsParseNode *lastPattern = NULL, *head = thePattern;
   
   while (tempPattern != NULL)
     {
      /* ==========================================
         Remember the class bitmap for this pattern
         ========================================== */
      if (tempPattern->userData != NULL)
        {
         *bmp = (BITMAP_HN *) tempPattern->userData;
         lastPattern = tempPattern;
         tempPattern = tempPattern->right;
        }
        
      /* ===========================================================
         A single field slot that has no pattern network expression
         associated with it can be removed (i.e. any value contained
         in this slot will satisfy the pattern being matched).
         =========================================================== */
      else if (((tempPattern->type == SF_WILDCARD) ||
                (tempPattern->type == SF_VARIABLE)) && 
               (tempPattern->networkTest == NULL))
        {
         if (lastPattern != NULL) lastPattern->right = tempPattern->right;
         else head = tempPattern->right;
         
         tempPattern->right = NULL;
         ReturnLHSParseNodes(tempPattern);
         
         if (lastPattern != NULL) tempPattern = lastPattern->right;
         else tempPattern = head;
        }
        
      /* =====================================================
         A multifield variable or wildcard within a multifield
         slot can be removed if there are no other multifield
         variables or wildcards contained in the same slot
         (and the multifield has no expressions which must be
         evaluated in the fact pattern network).
         ===================================================== */
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
  
      /* ================================================================
         A multifield wildcard or variable contained in a multifield slot
         that contains no other multifield wildcards or variables, but
         does have an expression that must be evaluated, can be changed
         to a single field pattern node with the same expression.
         ================================================================ */
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
        
      /* =======================================================
         If we're dealing with a multifield slot with no slot
         restrictions, then treat the multfield slot as a single
         field slot, but attach a test which verifies that the
         slot contains a zero length multifield value.
         ======================================================= */
      else if ((tempPattern->type == MF_WILDCARD) && 
               (tempPattern->multifieldSlot == CLIPS_TRUE) &&
               (tempPattern->bottom == NULL))
        {
         tempPattern->type = SF_WILDCARD;
         GenObjectZeroLengthTest(tempPattern);
         tempPattern->multifieldSlot = CLIPS_FALSE;
         lastPattern = tempPattern;
         tempPattern = tempPattern->right;
        }
      
      /* ======================================================
         Recursively call RemoveSlotExistenceTests for the slot
         restrictions contained within a multifield slot.
         ====================================================== */
      else if ((tempPattern->type == MF_WILDCARD) && 
               (tempPattern->multifieldSlot == CLIPS_TRUE))
        {
         /* =====================================================
            Add an expression to the first pattern restriction in
            the multifield slot that determines whether or not
            the fact's slot value contains the minimum number of
            required fields to satisfy the pattern restrictions
            for this slot. The length check is place before any
            other tests, so that preceeding checks do not have to
            determine if there are enough fields in the slot to
            safely retrieve a value.
            ===================================================== */
         GenObjectLengthTest(tempPattern->bottom);
         
         /* =======================================================
            Remove any unneeded pattern restrictions from the slot.
            ======================================================= */
         tempPattern->bottom = RemoveSlotExistenceTests(tempPattern->bottom,bmp);
         
         /* =========================================================
            If the slot no longer contains any restrictions, then the
            multifield slot can be completely removed. In any case,
            move on to the next slot to be examined for removal.
            ========================================================= */  
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
      
      /* =====================================================
         If none of the other tests for removing slots or slot
         restrictions apply, then move on to the next slot or
         slot restriction to be tested.
         ===================================================== */
      else
        {
         lastPattern = tempPattern;
         tempPattern = tempPattern->right;
        }
     }
     
   /* ====================================
      Return the pattern with unused slots
      and slot restrictions removed.
      ==================================== */
   return(head);
  }

/***************************************************
  NAME         : CreateInitialObjectPattern
  DESCRIPTION  : Creates a default object pattern
                 for use in defrules
  INPUTS       : None
  RETURNS      : The default initial pattern
  SIDE EFFECTS : Pattern created
  NOTES        : The pattern created is:
                 (object (is-a INITIAL-OBJECT)
                         (name [initial-object]))
 ***************************************************/
static struct lhsParseNode *CreateInitialObjectPattern()
  {
   struct lhsParseNode *topNode;
   CLASS_BITMAP *clsset;
   int initialObjectClassID;
   
   initialObjectClassID = LookupDefclassInScope(INITIAL_OBJECT_CLASS_NAME)->id;
   clsset = NewClassBitMap(initialObjectClassID,0);
   SetBitMap(clsset->map,initialObjectClassID);
   clsset = PackClassBitMap(clsset);
   
   topNode = GetLHSParseNode();
   topNode->userData = AddBitMap((VOID *) clsset,ClassBitMapSize(clsset));
   IncrementBitMapCount(topNode->userData);
   DeleteIntermediateClassBitMap(clsset);
   topNode->type = SF_WILDCARD;
   topNode->index = 1;
   topNode->slot = NAME_SYMBOL;
   topNode->slotNumber = NAME_ID;
   
   topNode->bottom = GetLHSParseNode();
   topNode->bottom->type = INSTANCE_NAME;
   topNode->bottom->value = (VOID *) INITIAL_OBJECT_SYMBOL;
   
   return(topNode);
  }
  
/**************************************************************
  NAME         : ObjectMatchDelayParse
  DESCRIPTION  : Parses the object-pattern-match-delay function
  INPUTS       : 1) The function call expression
                 2) The logical name of the input source
  RETURNS      : The top expression with the other
                 action expressions attached
  SIDE EFFECTS : Parses the function call and attaches
                 the appropriate arguments to the
                 top node
  NOTES        : None
 **************************************************************/
static EXPRESSION *ObjectMatchDelayParse(top,infile)
  struct expr *top;
  char *infile;
  {
   struct token tkn;
   
   IncrementIndentDepth(3);
   PPCRAndIndent();
   top->argList = GroupActions(infile,&tkn,CLIPS_TRUE,NULL);
   PPBackup();
   PPBackup();
   SavePPBuffer(tkn.print_rep);
   DecrementIndentDepth(3);
   if (top->argList == NULL)
     {
      ReturnExpression(top);
      return(NULL);
     }
   return(top);
  }

#if INCREMENTAL_RESET

/***************************************************
  NAME         : MarkObjectPtnIncrementalReset
  DESCRIPTION  : Marks/unmarks an object pattern for
                 incremental reset
  INPUTS       : 1) The object pattern alpha node
                 2) The value to which to set the
                    incremental reset flag
  RETURNS      : Nothing useful
  SIDE EFFECTS : The pattern node is set/unset
  NOTES        : The pattern node can only be
                 set if it is a new node and
                 thus marked for initialization
                 by PlaceObjectPattern
 ***************************************************/
static VOID MarkObjectPtnIncrementalReset(thePattern,value)
  struct patternNodeHeader *thePattern;
  int value;
  {
   if (thePattern->initialize == CLIPS_FALSE)
     return;
   thePattern->initialize = value;
  }

/***********************************************************
  NAME         : ObjectIncrementalReset
  DESCRIPTION  : Performs an assert for all instances in the
                 system.  All new patterns in the pattern
                 network from the new rule have been marked
                 as needing processing.  Old patterns will
                 be ignored.
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : All objects driven through new patterns
  NOTES        : None
 ***********************************************************/
static VOID ObjectIncrementalReset()
  {
   INSTANCE_TYPE *ins;
   
   for (ins = InstanceList ; ins != NULL ; ins = ins->nxtList)
     ObjectNetworkAction(OBJECT_ASSERT,(VOID *) ins,-1);
  }

#endif

#endif

#endif

/***************************************************
  NAME         : 
  DESCRIPTION  : 
  INPUTS       : 
  RETURNS      : 
  SIDE EFFECTS : 
  NOTES        : 
 ***************************************************/
