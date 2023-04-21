   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                  CLASS PARSER MODULE                */
   /*******************************************************/

/**************************************************************/
/* Purpose: Parsing Routines for Defclass Construct           */
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

#if OBJECT_SYSTEM && (! BLOAD_ONLY) && (! RUN_TIME)

#if BLOAD || BLOAD_AND_BSAVE
#include "bload.h"
#endif

#include "classcom.h"
#include "classfun.h"
#include "clipsmem.h"
#include "cstrcpsr.h"
#include "inherpsr.h"
#include "modulpsr.h"
#include "modulutl.h"
#include "msgpsr.h"
#include "clsltpsr.h"
#include "router.h"
#include "scanner.h"

#define _CLASSPSR_SOURCE_
#include "classpsr.h"

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */
#define ROLE_RLN             "role"
#define ABSTRACT_RLN         "abstract"
#define CONCRETE_RLN         "concrete"

#define HANDLER_DECL         "message-handler"

#define SLOT_RLN             "slot"
#define SGL_SLOT_RLN         "single-slot"
#define MLT_SLOT_RLN         "multislot"

#define DIRECT               0
#define INHERIT              1

/* =========================================
   *****************************************
               MACROS AND TYPES
   =========================================
   ***************************************** */

/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER

static BOOLEAN ValidClassName(char *,DEFCLASS **);
static BOOLEAN ParseSimpleQualifier(char *,char *,char *,char *,BOOLEAN *,BOOLEAN *);
static BOOLEAN ReadUntilClosingParen(char *,struct token *);
static VOID AddClass(DEFCLASS *);
static VOID BuildSubclassLinks(DEFCLASS *);
static VOID FormInstanceTemplate(DEFCLASS *);
static VOID FormSlotNameMap(DEFCLASS *);
static TEMP_SLOT_LINK *MergeSlots(TEMP_SLOT_LINK *,DEFCLASS *,unsigned *,int);
static VOID PackSlots(DEFCLASS *,TEMP_SLOT_LINK *);
#if DEFMODULE_CONSTRUCT
static VOID CreateClassScopeMap(DEFCLASS *);
#endif
static VOID CreatePublicSlotMessageHandlers(DEFCLASS *);

#else

static BOOLEAN ValidClassName();
static BOOLEAN ParseSimpleQualifier();
static BOOLEAN ReadUntilClosingParen();
static VOID AddClass();
static VOID BuildSubclassLinks();
static VOID FormInstanceTemplate();
static VOID FormSlotNameMap();
static TEMP_SLOT_LINK *MergeSlots();
static VOID PackSlots();
#if DEFMODULE_CONSTRUCT
static VOID CreateClassScopeMap();
#endif
static VOID CreatePublicSlotMessageHandlers();

#endif

/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
globle struct token ObjectParseToken;

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

/***************************************************************************************
  NAME         : ParseDefclass
  DESCRIPTION  : (defclass ...) is A CLIPS construct (as
                 opposed to a function), thus no variables
                 may be used.  This means classes may only
                 be STATICALLY defined (like rules).
  INPUTS       : The logical name of the router
                    for the parser input
  RETURNS      : CLIPS_FALSE if successful parse, CLIPS_TRUE otherwise
  SIDE EFFECTS : Inserts valid class definition into
                 Class Table.
  NOTES        : CLIPS Syntax :
                 (defclass <name> [<comment>]
                    (is-a <superclass-name>+)
                    <class-descriptor>*)
                
                 <class-descriptor> :== (slot <name> <slot-descriptor>*) |
                                        (role abstract|concrete) |
                                        (pattern-match reactive|non-reactive)
                                        
                                        These are for documentation only:
                                        (message-handler <name> [<type>])
                                        
                 <slot-descriptor>  :== (default <default-expression>) |
                                        (default-dynamic <default-expression>) |
                                        (storage shared|local) |
                                        (access read-only|read-write|initialize-only) |
                                        (propagation no-inherit|inherit) |
                                        (source composite|exclusive)
                                        (pattern-match reactive|non-reactive)
                                        (visibility public|private)
                                        (override-message <message-name>)
                                        (type ...) |
                                        (cardinality ...) |
                                        (allowed-symbols ...) |
                                        (allowed-strings ...) |
                                        (allowed-numbers ...) |
                                        (allowed-integers ...) |
                                        (allowed-floats ...) |
                                        (allowed-values ...) |
                                        (allowed-instance-names ...) |
                                        (range ...)
                                        
               <default-expression> ::= ?NONE | ?VARIABLE | <expression>*
  ***************************************************************************************/
globle int ParseDefclass(readSource)
  char *readSource;
  {
   SYMBOL_HN *cname;
   DEFCLASS *cls;
   PACKED_CLASS_LINKS *sclasses,*preclist;
   TEMP_SLOT_LINK *slots = NULL;
   int roleSpecified = CLIPS_FALSE,
       abstract = CLIPS_FALSE,
       parseError;
#if INSTANCE_PATTERN_MATCHING
   int patternMatchSpecified = CLIPS_FALSE,
       reactive = CLIPS_TRUE;
#endif
   
   SetPPBufferStatus(ON);
   FlushPPBuffer();          
   SetIndentDepth(3);    
   SavePPBuffer("(defclass ");

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
   if (Bloaded())
     {      
      CannotLoadWithBloadMessage("defclass");
      return(CLIPS_TRUE);
     }
#endif

   cname = GetConstructNameAndComment(readSource,&ObjectParseToken,"defclass",
                                      FindDefclass,NULL,"#",CLIPS_TRUE,
                                      CLIPS_TRUE,CLIPS_TRUE);
   if (cname == NULL)
     return(CLIPS_TRUE);

   if (ValidClassName(ValueToString(cname),&cls) == CLIPS_FALSE)
     return(CLIPS_TRUE);
     
   sclasses = ParseSuperclasses(readSource,cname);
   if (sclasses == NULL)
     return(CLIPS_TRUE);
   preclist = FindPrecedenceList(cls,sclasses);
   if (preclist == NULL)
     {
      DeletePackedClassLinks(sclasses,CLIPS_TRUE);
      return(CLIPS_TRUE);
     }
   parseError = CLIPS_FALSE;
   GetToken(readSource,&ObjectParseToken);
   while (GetType(ObjectParseToken) != RPAREN)
     {
      if (GetType(ObjectParseToken) != LPAREN)
        {
         SyntaxErrorMessage("defclass");
         parseError = CLIPS_TRUE;
         break;
        }
      PPBackup();
      PPCRAndIndent();
      SavePPBuffer("(");
      GetToken(readSource,&ObjectParseToken);
      if (GetType(ObjectParseToken) != SYMBOL)
        {
         SyntaxErrorMessage("defclass");
         parseError = CLIPS_TRUE;
         break;
        }
      if (strcmp(DOToString(ObjectParseToken),ROLE_RLN) == 0)
        {
         if (ParseSimpleQualifier(readSource,ROLE_RLN,CONCRETE_RLN,ABSTRACT_RLN,
                                  &roleSpecified,&abstract) == CLIPS_FALSE)
           {
            parseError = CLIPS_TRUE;
            break;
           }
        }
#if INSTANCE_PATTERN_MATCHING
      else if (strcmp(DOToString(ObjectParseToken),MATCH_RLN) == 0)
        {
         if (ParseSimpleQualifier(readSource,MATCH_RLN,NONREACTIVE_RLN,REACTIVE_RLN,
                                  &patternMatchSpecified,&reactive) == CLIPS_FALSE)
           {
            parseError = CLIPS_TRUE;
            break;
           }
        }
#endif
      else if (strcmp(DOToString(ObjectParseToken),SLOT_RLN) == 0)
        {
         slots = ParseSlot(readSource,slots,preclist,CLIPS_FALSE,CLIPS_FALSE);
         if (slots == NULL)
           {
            parseError = CLIPS_TRUE;
            break;
           }
        }
      else if (strcmp(DOToString(ObjectParseToken),SGL_SLOT_RLN) == 0)
        {
         slots = ParseSlot(readSource,slots,preclist,CLIPS_FALSE,CLIPS_TRUE);
         if (slots == NULL)
           {
            parseError = CLIPS_TRUE;
            break;
           }
        }
      else if (strcmp(DOToString(ObjectParseToken),MLT_SLOT_RLN) == 0)
        {
         slots = ParseSlot(readSource,slots,preclist,CLIPS_TRUE,CLIPS_TRUE);
         if (slots == NULL)
           {
            parseError = CLIPS_TRUE;
            break;
           }
        }
      else if (strcmp(DOToString(ObjectParseToken),HANDLER_DECL) == 0)
        {
         if (ReadUntilClosingParen(readSource,&ObjectParseToken) == CLIPS_FALSE)
           {
            parseError = CLIPS_TRUE;
            break;
           }
        }
      else
        {
         SyntaxErrorMessage("defclass");
         parseError = CLIPS_TRUE;
         break;
        }
      GetToken(readSource,&ObjectParseToken);
     }
   if ((GetType(ObjectParseToken) != RPAREN) || (parseError == CLIPS_TRUE))
     {
      DeletePackedClassLinks(sclasses,CLIPS_TRUE);
      DeletePackedClassLinks(preclist,CLIPS_TRUE);
      DeleteSlots(slots);
      return(CLIPS_TRUE);
     }
   SavePPBuffer("\n");
   
   /* =========================================================================
      The abstract/reactive qualities of a class are inherited if not specified
      ========================================================================= */
   if (roleSpecified == CLIPS_FALSE)
     abstract = preclist->classArray[1]->abstract;
#if INSTANCE_PATTERN_MATCHING
   if (patternMatchSpecified == CLIPS_FALSE)
     reactive = preclist->classArray[1]->reactive;
     
   /* ================================================================
      An abstract class cannot have direct instances, thus it makes no
      sense for it to be reactive since it will have no objects to
      respond to pattern-matching
      ================================================================ */
   if (abstract && reactive)
     {
      PrintErrorID("CLASSPSR",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"An abstract class cannot be reactive.\n");
      DeletePackedClassLinks(sclasses,CLIPS_TRUE);
      DeletePackedClassLinks(preclist,CLIPS_TRUE);
      DeleteSlots(slots);
      return(CLIPS_TRUE);
     }
     
#endif

   cls = NewClass(cname);
   cls->abstract = abstract;
#if INSTANCE_PATTERN_MATCHING
   cls->reactive = reactive;
#endif
   cls->directSuperclasses.classCount = sclasses->classCount;
   cls->directSuperclasses.classArray = sclasses->classArray;
   
   /* =======================================================
      This is a hack to let functions which need to iterate
      over a class AND its superclasses to conveniently do so
      
      The real precedence list starts in position 1
      ======================================================= */
   preclist->classArray[0] = cls;
   cls->allSuperclasses.classCount = preclist->classCount;
   cls->allSuperclasses.classArray = preclist->classArray;
   rtn_struct(packedClassLinks,sclasses);
   rtn_struct(packedClassLinks,preclist);
   
   /* =================================
      Shove slots into contiguous array
      ================================= */
   if (slots != NULL)
     PackSlots(cls,slots);
   AddClass(cls);
      
   return(CLIPS_FALSE);
  }

/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

/***********************************************************
  NAME         : ValidClassName
  DESCRIPTION  : Determines if a new class of the given
                 name can be defined in the current module
  INPUTS       : 1) The new class name
                 2) Buffer to hold class address
  RETURNS      : CLIPS_TRUE if OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : Error message printed if not OK
  NOTES        : GetConstructNameAndComment() (called before
                 this function) ensures that the defclass
                 name does not conflict with one from
                 another module
 ***********************************************************/
static BOOLEAN ValidClassName(theClassName,theDefclass)
  char *theClassName;
  DEFCLASS **theDefclass;
  {
   *theDefclass = (DEFCLASS *) FindDefclass(theClassName);
   if (*theDefclass != NULL)
     {
      /* ===================================
         System classes (which are visible
         in all modules) cannot be redefined
         =================================== */
      if ((*theDefclass)->system)
        {
         PrintErrorID("CLASSPSR",2,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Cannot redefine a predefined system class.\n");
         return(CLIPS_FALSE);
        }
        
      /* ===============================================
         A class in the current module can only be
         redefined if it is not in use, e.g., instances,
         generic function method restrictions, etc.
         =============================================== */
      if (IsDefclassDeletable((VOID *) *theDefclass) == CLIPS_FALSE)
        {
         PrintErrorID("CLASSPSR",3,CLIPS_FALSE);
         PrintCLIPS(WERROR,GetDefclassName((VOID *) *theDefclass));
         PrintCLIPS(WERROR," class cannot be redefined while\n");
         PrintCLIPS(WERROR,"    outstanding references to it still exist.\n");
         return(CLIPS_FALSE);
        }
     }
   return(CLIPS_TRUE);
  }
  
/***************************************************************
  NAME         : ParseSimpleQualifier
  DESCRIPTION  : Parses abstract/concrete role and
                 pattern-matching reactivity for class
  INPUTS       : 1) The input logical name
                 2) The name of the qualifier being parsed
                 3) The qualifier value indicating that the
                    qualifier should be false
                 4) The qualifier value indicating that the
                    qualifier should be true
                 5) A pointer to a bitmap indicating
                    if the qualifier has already been parsed
                 6) A buffer to store the value of the qualifier
  RETURNS      : CLIPS_TRUE if all OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : Bitmap and qualifier buffers set
                 Messages printed on errors
  NOTES        : None
 ***************************************************************/
static BOOLEAN ParseSimpleQualifier(readSource,classQualifier,
                                    clearRelation,setRelation,
                                    alreadyTestedFlag,binaryFlag)
  char *readSource,*classQualifier,
       *clearRelation,*setRelation;
  BOOLEAN *alreadyTestedFlag,*binaryFlag;
  {
   if (*alreadyTestedFlag)
     {
      PrintErrorID("CLASSPSR",4,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Class ");
      PrintCLIPS(WERROR,classQualifier);
      PrintCLIPS(WERROR," already declared.\n");
      return(CLIPS_FALSE);
     }
   SavePPBuffer(" ");
   GetToken(readSource,&ObjectParseToken);
   if (GetType(ObjectParseToken) != SYMBOL)
     goto ParseSimpleQualifierError;
   if (strcmp(DOToString(ObjectParseToken),setRelation) == 0)
     *binaryFlag = CLIPS_TRUE;
   else if (strcmp(DOToString(ObjectParseToken),clearRelation) == 0)
     *binaryFlag = CLIPS_FALSE;
   else
     goto ParseSimpleQualifierError;
   GetToken(readSource,&ObjectParseToken);
   if (GetType(ObjectParseToken) != RPAREN)
     goto ParseSimpleQualifierError;
   *alreadyTestedFlag = CLIPS_TRUE;
   return(CLIPS_TRUE);
   
ParseSimpleQualifierError:
   SyntaxErrorMessage("defclass");
   return(CLIPS_FALSE);
  }
  
/***************************************************
  NAME         : ReadUntilClosingParen
  DESCRIPTION  : Skips over tokens until a ')' is
                 encountered.
  INPUTS       : 1) The logical input source
                 2) A buffer for scanned tokens
  RETURNS      : CLIPS_TRUE if ')' read, CLIPS_FALSE
                 otherwise
  SIDE EFFECTS : Tokens read
  NOTES        : Expects first token after opening
                 paren has already been scanned
 ***************************************************/
static BOOLEAN ReadUntilClosingParen(readSource,inputToken)
  char *readSource;
  struct token *inputToken;
  {
   int cnt = 1,lparen_read = CLIPS_FALSE;

   do
     {
      if (lparen_read == CLIPS_FALSE)
        SavePPBuffer(" ");
      GetToken(readSource,inputToken);
      if (inputToken->type == STOP)
        {
         SyntaxErrorMessage("message-handler declaration");
         return(CLIPS_FALSE);
        }
      else if (inputToken->type == LPAREN)
        {
         lparen_read = CLIPS_TRUE;
         cnt++;
        }
      else if (inputToken->type == RPAREN)
        {
         cnt--;
         if (lparen_read == CLIPS_FALSE)
           {
            PPBackup();
            PPBackup();
            SavePPBuffer(")");
           }
         lparen_read = CLIPS_FALSE;
        }
      else
        lparen_read = CLIPS_FALSE;
     }
   while (cnt > 0);

   return(CLIPS_TRUE);
  }

/*****************************************************************************
  NAME         : AddClass
  DESCRIPTION  : Determines the precedence list of the new class.
                 If it is valid, the routine checks to see if the class
                 already exists.  If it does not, all the subclass
                 links are made from the class's direct superclasses,
                 and the class is inserted in the hash table.  If it
                 does, all sublclasses are deleted. An error will occur
                 if any instances of the class (direct or indirect) exist.
                 If all checks out, the old definition is replaced by the new. 
  INPUTS       : The new class description
  RETURNS      : Nothing useful
  SIDE EFFECTS : The class is deleted if there is an error.
  NOTES        : No change in the class graph state will occur
                 if there were any errors.
                 Assumes class is not busy!!!
 *****************************************************************************/
static VOID AddClass(cls)
  DEFCLASS *cls;
  {
   DEFCLASS *ctmp;
#if DEBUGGING_FUNCTIONS
   int oldTraceInstances = CLIPS_FALSE,
       oldTraceSlots = CLIPS_FALSE;
#endif
   
   /* ===============================================
      If class does not already exist, insert and 
      form progeny links with all direct superclasses
      =============================================== */
   cls->hashTableIndex = HashClass(GetDefclassNamePointer((VOID *) cls));
   ctmp = (DEFCLASS *) FindDefclass(GetDefclassName((VOID *) cls));
   
   if (ctmp != NULL)
     {
#if DEBUGGING_FUNCTIONS
      oldTraceInstances = ctmp->traceInstances;
      oldTraceSlots = ctmp->traceSlots;
#endif
      DeleteClassUAG(ctmp);
     }
   PutClassInTable(cls);

   BuildSubclassLinks(cls);
   InstallClass(cls,CLIPS_TRUE);
   AddConstructToModule((struct constructHeader *) cls);
   
   FormInstanceTemplate(cls);
   FormSlotNameMap(cls);
   
   AssignClassID(cls);

#if DEBUGGING_FUNCTIONS
   if (cls->abstract)
     {
      cls->traceInstances = CLIPS_FALSE;
      cls->traceSlots = CLIPS_FALSE;
     }
   else
     {
      if (oldTraceInstances)
        cls->traceInstances = CLIPS_TRUE;
      if (oldTraceSlots)
        cls->traceSlots = CLIPS_TRUE;
     }
#endif

#if DEBUGGING_FUNCTIONS
   if (GetConserveMemory() == CLIPS_FALSE)
     SetDefclassPPForm((VOID *) cls,CopyPPBuffer());
#endif

#if DEFMODULE_CONSTRUCT

   /* =========================================
      Create a bitmap indicating whether this
      class is in scope or not for every module
      ========================================= */
   CreateClassScopeMap(cls);
   
#endif
   
   /* ==============================================
      Define get- and put- handlers for public slots
      ============================================== */
   CreatePublicSlotMessageHandlers(cls);   
  }
  
/*******************************************************
  NAME         : BuildSubclassLinks
  DESCRIPTION  : Follows the list of superclasses
                 for a class and puts the class in
                 each of the superclasses' subclass
                 list.
  INPUTS       : The address of the class
  RETURNS      : Nothing useful
  SIDE EFFECTS : The subclass lists for every superclass
                 are modified.
  NOTES        : Assumes the superclass list is formed.
 *******************************************************/
static VOID BuildSubclassLinks(cls)
  DEFCLASS *cls;
  {
   register unsigned i;
   
   for (i = 0 ; i < cls->directSuperclasses.classCount ; i++)
     AddClassLink(&cls->directSuperclasses.classArray[i]->directSubclasses,cls,-1);
  }

/**********************************************************
  NAME         : FormInstanceTemplate
  DESCRIPTION  : Forms a contiguous array of instance
                  slots for use in creating instances later
                 Also used in determining instance slot
                  indices a priori during handler defns
  INPUTS       : The class
  RETURNS      : Nothing useful
  SIDE EFFECTS : Contiguous array of instance slots formed
  NOTES        : None
 **********************************************************/
static VOID FormInstanceTemplate(cls)
  DEFCLASS *cls;
  {
   TEMP_SLOT_LINK *islots = NULL,*stmp;
   unsigned scnt = 0;
   register unsigned i;

   /* ========================
      Get direct class's slots
      ======================== */
   islots = MergeSlots(islots,cls,&scnt,DIRECT);
   
   /* ===================================================================
      Get all inherited slots - a more specific slot takes precedence
      over more general, i.e. the first class in the precedence list with
      a particular slot gets to specify its default value
      =================================================================== */
   for (i = 1 ; i < cls->allSuperclasses.classCount ; i++)
     islots = MergeSlots(islots,cls->allSuperclasses.classArray[i],&scnt,INHERIT);
    
   /* ===================================================
      Allocate a contiguous array to store all the slots.
      =================================================== */
   cls->instanceSlotCount = scnt;
   cls->localInstanceSlotCount = 0;
   if (scnt > 0)
     cls->instanceTemplate = (SLOT_DESC **) gm2((int) (scnt * sizeof(SLOT_DESC *)));
   for (i = 0 ; i < scnt ; i++)
     {
      stmp = islots;
      islots = islots->nxt;
      cls->instanceTemplate[i] = stmp->desc;
      if (stmp->desc->shared == 0)
        cls->localInstanceSlotCount++;
      rtn_struct(tempSlotLink,stmp);
     }
  }
   
/**********************************************************
  NAME         : FormSlotNameMap
  DESCRIPTION  : Forms a mapping of the slot name ids into
                 the instance template.  Given the slot
                 name id, this map provides a much faster
                 lookup of a slot.  The id is stored
                 statically in object patterns and can
                 be looked up via a hash table at runtime
                 as well.
  INPUTS       : The class
  RETURNS      : Nothing useful
  SIDE EFFECTS : Contiguous array of integers formed
                 The position in the array corresponding
                 to a slot name id holds an the index
                 into the instance template array holding
                 the slot
                 The max slot name id for the class is
                 also stored to make deletion of the slots
                 easier
  NOTES        : Assumes the instance template has already
                 been formed
 **********************************************************/
static VOID FormSlotNameMap(cls)
  DEFCLASS *cls;
  {
   register int i;
   
   cls->maxSlotNameID = 0;
   cls->slotNameMap = NULL;
   if (cls->instanceSlotCount == 0)
     return;
   for (i = 0 ; i < cls->instanceSlotCount ; i++)
     if (cls->instanceTemplate[i]->slotName->id > cls->maxSlotNameID)
       cls->maxSlotNameID = cls->instanceTemplate[i]->slotName->id;
   cls->slotNameMap = (unsigned *) gm2((int) (sizeof(unsigned) * (cls->maxSlotNameID + 1)));
   for (i = 0 ; i <= cls->maxSlotNameID ; i++)
     cls->slotNameMap[i] = 0;
   for (i = 0 ; i < cls->instanceSlotCount ; i++)
     cls->slotNameMap[cls->instanceTemplate[i]->slotName->id] = i + 1;
  }
  
/********************************************************************
  NAME         : MergeSlots
  DESCRIPTION  : Adds non-duplicate slots to list and increments
                   slot count for the class instance template
  INPUTS       : 1) The old slot list
                 2) The address of class containing new slots
                 3) Caller's buffer for # of slots
                 4) A flag indicating whether the new list of slots
                    is from the direct parent-class or not.
  RETURNS      : The address of the new expanded list, or NULL
                   for an empty list
  SIDE EFFECTS : The list is expanded
                 Caller's slot count is adjusted.
  NOTES        : Lists are assumed to contain no duplicates
 *******************************************************************/
static TEMP_SLOT_LINK *MergeSlots(old,cls,scnt,src)
  TEMP_SLOT_LINK *old;
  DEFCLASS *cls;
  unsigned *scnt;
  int src;
  {
   TEMP_SLOT_LINK *cur,*tmp;
   register int i;
   SLOT_DESC *new;
   
   /* ======================================
      Process the slots in reverse order
      since we are pushing them onto a stack
      ====================================== */
   for (i = cls->slotCount - 1 ; i >= 0 ; i--)
     {
      new = &cls->slots[i];
     
      /* ==========================================
         A class can prevent it slots from being
         propagated to all but its direct instances
         ========================================== */
      if ((new->noInherit == 0) ? CLIPS_TRUE : (src == DIRECT))
        {
         cur = old;
         while ((cur != NULL) ? (new->slotName != cur->desc->slotName) : CLIPS_FALSE)
           cur = cur->nxt;
         if (cur == NULL)
           {
            tmp = get_struct(tempSlotLink);
            tmp->desc = new;
            tmp->nxt = old;
            old = tmp;
            (*scnt)++;
           }
        }
     }
   return(old);
  }

/***********************************************************************
  NAME         : PackSlots
  DESCRIPTION  : Groups class-slots into a contiguous array
                  "slots" field points to array
                  "slotCount" field set
  INPUTS       : 1) The class
                 2) The list of slots
  RETURNS      : Nothing useful
  SIDE EFFECTS : Temporary list deallocated, contiguous array allocated,
                   and nxt pointers linked
                 Class pointer set for slots
  NOTES        : Assumes class->slotCount == 0 && class->slots == NULL
 ***********************************************************************/
static VOID PackSlots(cls,slots)
  DEFCLASS *cls;
  TEMP_SLOT_LINK *slots;
  {
   TEMP_SLOT_LINK *stmp,*sprv;
   register int i;
   
   stmp = slots;
   while  (stmp != NULL)
     {
      stmp->desc->cls = cls;
      cls->slotCount++;
      stmp = stmp->nxt;
     }
   cls->slots = (SLOT_DESC *) gm2((int) (sizeof(SLOT_DESC) * cls->slotCount));
   stmp = slots;
   for (i = 0 ; i < cls->slotCount ; i++)
     {
      sprv = stmp;
      stmp = stmp->nxt;
      CopyMemory(SLOT_DESC,1,&(cls->slots[i]),sprv->desc);
      cls->slots[i].sharedValue.desc = &(cls->slots[i]);
      cls->slots[i].sharedValue.value = NULL;
      rtn_struct(slotDescriptor,sprv->desc);
      rtn_struct(tempSlotLink,sprv);
     }
  }
  
#if DEFMODULE_CONSTRUCT

/********************************************************
  NAME         : CreateClassScopeMap
  DESCRIPTION  : Creates a bitmap where each bit position
                 corresponds to a module id. If the bit
                 is set, the class is in scope for that
                 module, otherwise it is not.
  INPUTS       : The class
  RETURNS      : Nothing useful
  SIDE EFFECTS : Scope bitmap created and attached
  NOTES        : Uses FindImportedConstruct()
 ********************************************************/
static VOID CreateClassScopeMap(theDefclass)
  DEFCLASS *theDefclass;
  {
   int scopeMapSize;
   char *scopeMap;
   char *className;
   struct defmodule *matchModule,
                    *theModule;
   int moduleID,count;
   
   className = ValueToString(theDefclass->header.name);
   matchModule = theDefclass->header.whichModule->theModule;
   
   scopeMapSize = (int) (sizeof(char) * ((GetNumberOfDefmodules() / BITS_PER_BYTE) + 1));
   scopeMap = (char *) gm2(scopeMapSize);
   
   ClearBitString((VOID *) scopeMap,scopeMapSize);
   SaveCurrentModule();
   for (theModule = (struct defmodule *) GetNextDefmodule(NULL) ;
        theModule != NULL ;
        theModule = (struct defmodule *) GetNextDefmodule((VOID *) theModule))
     {
      SetCurrentModule((VOID *) theModule);
      moduleID = (int) theModule->bsaveID;
      if (FindImportedConstruct("defclass",matchModule,
                                className,&count,CLIPS_TRUE,NULL) != NULL)
        SetBitMap(scopeMap,moduleID);
     }
   RestoreCurrentModule();
   theDefclass->scopeMap = (BITMAP_HN *) AddBitMap(scopeMap,scopeMapSize);
   IncrementBitMapCount(theDefclass->scopeMap);
   rm((VOID *) scopeMap,scopeMapSize);
  }

#endif
  
/*****************************************************************************
  NAME         : CreatePublicSlotMessageHandlers
  DESCRIPTION  : Creates a get-<slot-name> and
                 put-<slot-name> handler for every
                 public slot in a class.
                 
                 The syntax of the message-handlers
                 created are:
                 
                 (defmessage-handler <class> get-<slot-name> primary ()
                    ?self:<slot-name>)
                 
                 For single-field slots:
                 
                 (defmessage-handler <class> put-<slot-name> primary (?value)
                    (bind ?self:<slot-name> ?value))
                 
                 For multifield slots:
                 
                 (defmessage-handler <class> put-<slot-name> primary ($?value)
                    (bind ?self:<slot-name> ?value))
  INPUTS       : The defclass
  RETURNS      : Nothing useful
  SIDE EFFECTS : Message-handlers created
  NOTES        : None
 ******************************************************************************/
static VOID CreatePublicSlotMessageHandlers(theDefclass)
  DEFCLASS *theDefclass;
  {
   register unsigned i;
   register SLOT_DESC *sd;
   
   for (i = 0 ; i < theDefclass->slotCount ; i++)
     {
      sd = &theDefclass->slots[i];
        CreateGetAndPutHandlers(sd);
     }
   for (i = 0 ; i < theDefclass->handlerCount ; i++)
     theDefclass->handlers[i].system = CLIPS_TRUE;
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
