   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*              CLIPS Version 6.00  05/12/93           */
   /*                                                     */
   /*            INSTANCE PRIMITIVE SUPPORT MODULE        */
   /*******************************************************/

/*************************************************************/
/* Purpose:  Creation and Deletion of Instances Routines     */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/
   
/* =========================================
   *****************************************
               EXTERNAL DEFINITIONS
   =========================================
   ***************************************** */
#include "setup.h"

#if OBJECT_SYSTEM

#if INSTANCE_PATTERN_MATCHING
#include "network.h"
#include "drive.h"
#include "objrtmch.h"
#endif

#if LOGICAL_DEPENDENCIES
#include "lgcldpnd.h"
#endif

#include "classcom.h"
#include "classfun.h"
#include "clipsmem.h"
#include "extnfunc.h"
#include "insfun.h"
#include "modulutl.h"
#include "msgfun.h"
#include "router.h"
#include "utility.h"

#define _INSMNGR_SOURCE_
#include "insmngr.h"

#include "inscom.h"

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */
#define MAKE_TRACE   "==>"
#define UNMAKE_TRACE "<=="

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

static INSTANCE_TYPE *NewInstance(void);
static INSTANCE_TYPE *InstanceLocationInfo(DEFCLASS *,SYMBOL_HN *,INSTANCE_TYPE **,unsigned *);
static VOID InstallInstance(INSTANCE_TYPE *,int);
static VOID BuildDefaultSlots(BOOLEAN);
static int CoreInitializeInstance(INSTANCE_TYPE *,EXPRESSION *);
static int InsertSlotOverrides(INSTANCE_TYPE *,EXPRESSION *);
static VOID EvaluateClassDefaults(INSTANCE_TYPE *);

#if DEBUGGING_FUNCTIONS
static VOID PrintInstanceWatch(char *,INSTANCE_TYPE *);
#endif

#else

static INSTANCE_TYPE *NewInstance();
static INSTANCE_TYPE *InstanceLocationInfo();
static VOID InstallInstance();
static VOID BuildDefaultSlots();
static int CoreInitializeInstance();
static int InsertSlotOverrides();
static VOID EvaluateClassDefaults();

#if DEBUGGING_FUNCTIONS
static VOID PrintInstanceWatch();
#endif

#endif

/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
globle INSTANCE_TYPE *InstanceList = NULL;
globle unsigned long GlobalNumberOfInstances = 0L;

/* =========================================
   *****************************************
      INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
static INSTANCE_TYPE *CurrentInstance = NULL;
static INSTANCE_TYPE *InstanceListBottom = NULL;

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/***********************************************************
  NAME         : InitializeInstanceCommand
  DESCRIPTION  : Initializes an instance of a class
  INPUTS       : The address of the result value
  RETURNS      : Nothing useful
  SIDE EFFECTS : Instance intialized
  NOTES        : CLIPS Syntax:
                 (active-initialize-instance <instance-name>
                    <slot-override>*)
 ***********************************************************/
globle VOID InitializeInstanceCommand(result)
  DATA_OBJECT *result;
  {
   INSTANCE_TYPE *ins;
   
   SetpType(result,SYMBOL);
   SetpValue(result,CLIPSFalseSymbol);
   ins = CheckInstance("initialize-instance");
   if (ins == NULL)
     return;
   if (CoreInitializeInstance(ins,GetFirstArgument()->nextArg) == CLIPS_TRUE)
     {
      SetpType(result,INSTANCE_NAME);
      SetpValue(result,(VOID *) ins->name);
     }
  }
  
/****************************************************************
  NAME         : MakeInstanceCommand
  DESCRIPTION  : Creates and initializes an instance of a class
  INPUTS       : The address of the result value
  RETURNS      : Nothing useful
  SIDE EFFECTS : Instance intialized
  NOTES        : CLIPS Syntax:
                 (active-make-instance <instance-name> of <class>
                    <slot-override>*)
 ****************************************************************/
globle VOID MakeInstanceCommand(result)
  DATA_OBJECT *result;
  {
   SYMBOL_HN *iname;
   INSTANCE_TYPE *ins;
   DATA_OBJECT temp;
   DEFCLASS *cls;
   
   SetpType(result,SYMBOL);
   SetpValue(result,CLIPSFalseSymbol);
   EvaluateExpression(GetFirstArgument(),&temp);
   if ((GetType(temp) != SYMBOL) && 
       (GetType(temp) != INSTANCE_NAME))
     {
      PrintErrorID("INSMNGR",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Expected a valid name for new instance.\n");
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
   iname = (SYMBOL_HN *) GetValue(temp);
   if (GetFirstArgument()->nextArg->type == DEFCLASS_PTR)
     cls = (DEFCLASS *) GetFirstArgument()->nextArg->value;
   else
     {
      EvaluateExpression(GetFirstArgument()->nextArg,&temp);
      if (GetType(temp) != SYMBOL)
        {
         PrintErrorID("INSMNGR",2,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Expected a valid class name for new instance.\n");
         SetEvaluationError(CLIPS_TRUE);
         return;
        }
      cls = LookupDefclassInScope(DOToString(temp));
      if (cls == NULL)
        {
         ClassExistError(ValueToString(ExpressionFunctionCallName(CurrentExpression)),
                         DOToString(temp));
         SetEvaluationError(CLIPS_TRUE);
         return;
        }
     }
     
   ins = BuildInstance(iname,cls,CLIPS_TRUE);
   if (ins == NULL)
     return;
   if (CoreInitializeInstance(ins,GetFirstArgument()->nextArg->nextArg) == CLIPS_TRUE)
     {
      result->type = INSTANCE_NAME;
      result->value = (VOID *) GetFullInstanceName(ins);
     }
   else
     QuashInstance(ins);
  }

/***************************************************
  NAME         : GetFullInstanceName
  DESCRIPTION  : If this function is called while
                 the current module is other than
                 the one in which the instance
                 resides, then the module name is
                 prepended to the instance name.
                 Otherwise - the base name only is
                 returned.
  INPUTS       : The instance
  RETURNS      : The instance name symbol (with
                 module name and :: prepended)
  SIDE EFFECTS : Temporary buffer allocated possibly
                 and new symbol created
  NOTES        : Used to differentiate between
                 instances of the same name in
                 different modules
 ***************************************************/
globle SYMBOL_HN *GetFullInstanceName(ins)
  INSTANCE_TYPE *ins;
  {
   char *moduleName,*buffer;
   int bufsz;
   SYMBOL_HN *iname;
   
   if (ins == &DummyInstance)
     return((SYMBOL_HN *) AddSymbol("Dummy Instance"));
   if (ins->garbage)
     return(ins->name);
   if (ins->cls->header.whichModule->theModule == ((struct defmodule *) GetCurrentModule()))
     return(ins->name);
   moduleName = GetDefmoduleName((VOID *) ins->cls->header.whichModule->theModule);
   bufsz = (int) (sizeof(char) * (strlen(moduleName) + 
                                  strlen(ValueToString(ins->name)) + 3));
   buffer = (char *) gm2(bufsz);
   sprintf(buffer,"%s::%s",moduleName,ValueToString(ins->name));
   iname = (SYMBOL_HN *) AddSymbol(buffer);
   rm((VOID *) buffer,bufsz);
   return(iname);
  }

/***************************************************
  NAME         : BuildInstance
  DESCRIPTION  : Creates an uninitialized instance
  INPUTS       : 1) Name of the instance
                 2) Class pointer
                 3) Flag indicating whether init
                    message will be called for
                    this instance or not
  RETURNS      : The address of the new instance,
                   NULL on errors (or when a
                   a logical basis in a rule was
                   deleted int the same RHS in
                   which the instance creation
                   occurred)
  SIDE EFFECTS : Old definition (if any) is deleted
  NOTES        : None
 ***************************************************/
globle INSTANCE_TYPE *BuildInstance(iname,cls,initMessage)
  SYMBOL_HN *iname;
  DEFCLASS *cls;
  BOOLEAN initMessage;
  {
   INSTANCE_TYPE *ins,*iprv;
   unsigned hashTableIndex;
   int modulePosition;
   SYMBOL_HN *moduleName;
   
#if INSTANCE_PATTERN_MATCHING
   if (JoinOperationInProgress && cls->reactive)
     {
      PrintErrorID("INSMNGR",10,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Cannot create instances of reactive classes while\n");
      PrintCLIPS(WERROR,"  pattern-matching is in process.\n");
      SetEvaluationError(CLIPS_TRUE);
      return(NULL);
     }
#endif
   if (cls->abstract)
     {
      PrintErrorID("INSMNGR",3,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Cannot create instances of abstract class ");
      PrintCLIPS(WERROR,GetDefclassName((VOID *) cls));
      PrintCLIPS(WERROR,".\n");
      SetEvaluationError(CLIPS_TRUE);
      return(NULL);
     }
   modulePosition = FindModuleSeparator(ValueToString(iname));
   if (modulePosition)
     {
      moduleName = ExtractModuleName(modulePosition,ValueToString(iname));
      if ((moduleName == NULL) ||
          (moduleName != cls->header.whichModule->theModule->name))
        {
         PrintErrorID("INSMNGR11",11,CLIPS_TRUE);
         PrintCLIPS(WERROR,"Invalid module specifier in new instance name.\n");
         SetEvaluationError(CLIPS_TRUE);
         return(NULL);
        }
      iname = ExtractConstructName(modulePosition,ValueToString(iname));
     }
   ins = InstanceLocationInfo(cls,iname,&iprv,&hashTableIndex);
   if (ins != NULL)
     {
      if (ins->installed == 0)
        {
         PrintErrorID("INSMNGR",4,CLIPS_FALSE);
         PrintCLIPS(WERROR,"The instance ");
         PrintCLIPS(WERROR,ValueToString(iname));
         PrintCLIPS(WERROR," has a slot-value which depends on the instance definition.\n");
         SetEvaluationError(CLIPS_TRUE);
         return(NULL);
        }
      ins->busy++;
      IncrementSymbolCount(iname);
      if (ins->garbage == 0)
        {
         if (MkInsMsgPass)
           DirectMessage(DELETE_SYMBOL,ins,NULL,NULL);
         else
           QuashInstance(ins);
        }
      ins->busy--;
      DecrementSymbolCount(iname);
      if (ins->garbage == 0)
        {
         PrintErrorID("INSMNGR",5,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Unable to delete old instance ");
         PrintCLIPS(WERROR,ValueToString(iname));
         PrintCLIPS(WERROR,".\n");
         SetEvaluationError(CLIPS_TRUE);
         return(NULL);
        }
     }
     
   /* ============================================================= 
      Create the base instance from the defaults of the inheritance
      precedence list
      ============================================================= */
   CurrentInstance = NewInstance();
   
#if LOGICAL_DEPENDENCIES
   /* ==============================================
      Add this new instance as a dependent to
      any currently active basis - if the partial
      match was deleted, abort the instance creation
      ============================================== */
   if (AddLogicalDependencies((struct patternEntity *) CurrentInstance,CLIPS_FALSE)
        == CLIPS_FALSE)
     {
      rtn_struct(instance,CurrentInstance);
      CurrentInstance = NULL;
      return(NULL);
     }
#endif

   CurrentInstance->name = iname;
   CurrentInstance->cls = cls;
   BuildDefaultSlots(initMessage);
           
   /* ============================================================
      Put the instance in the instance hash table and put it on its
        class's instance list
      ============================================================ */
   CurrentInstance->hashTableIndex = hashTableIndex;
   if (iprv == NULL)
     {
      CurrentInstance->nxtHash = InstanceTable[hashTableIndex];
      if (InstanceTable[hashTableIndex] != NULL)
        InstanceTable[hashTableIndex]->prvHash = CurrentInstance;
      InstanceTable[hashTableIndex] = CurrentInstance;
     }
   else
     {
      CurrentInstance->nxtHash = iprv->nxtHash;
      if (iprv->nxtHash != NULL)
        iprv->nxtHash->prvHash = CurrentInstance;
      iprv->nxtHash = CurrentInstance;
      CurrentInstance->prvHash = iprv;
     }
   
   /* ======================================
      Put instance in global and class lists
      ====================================== */
   if (CurrentInstance->cls->instanceList == NULL)
     CurrentInstance->cls->instanceList = CurrentInstance;
   else
     CurrentInstance->cls->instanceListBottom->nxtClass = CurrentInstance;
   CurrentInstance->prvClass = CurrentInstance->cls->instanceListBottom;
   CurrentInstance->cls->instanceListBottom = CurrentInstance;
   
   if (InstanceList == NULL)
     InstanceList = CurrentInstance;
   else
     InstanceListBottom->nxtList = CurrentInstance;
   CurrentInstance->prvList = InstanceListBottom;
   InstanceListBottom = CurrentInstance;
   ChangesToInstances = CLIPS_TRUE;
   
   /* ==============================================================================
      Install the instance's name and slot-value symbols (prevent them from becoming
      ephemeral) - the class name and slot names are accounted for by the class
      ============================================================================== */
   InstallInstance(CurrentInstance,CLIPS_TRUE);

   ins = CurrentInstance;
   CurrentInstance = NULL;

#if INSTANCE_PATTERN_MATCHING
   if (ins->cls->reactive)
     ObjectNetworkAction(OBJECT_ASSERT,(VOID *) ins,-1);
#endif
   
   return(ins);
  }

/*****************************************************************************
  NAME         : InitSlotsCommand
  DESCRIPTION  : Calls CLIPS Kernel Expression Evaluator EvaluateExpression
                   for each expression-value of an instance expression
                   
                 Evaluates default slots only - slots that were specified
                 by overrides (sp->override == 1) are ignored)
  INPUTS       : 1) Instance address
  RETURNS      : Nothing useful
  SIDE EFFECTS : Each DATA_OBJECT slot in the instance's slot array is replaced
                   by the evaluation (by EvaluateExpression) of the expression
                   in the slot list.  The old expression-values
                   are deleted.
  NOTES        : CLIPS Syntax: (init-slots <instance>)
 *****************************************************************************/
globle VOID InitSlotsCommand(result)
  DATA_OBJECT *result;
  {
   SetpType(result,SYMBOL);
   SetpValue(result,CLIPSFalseSymbol);
   EvaluationError = CLIPS_FALSE;
   if (CheckCurrentMessage("init-slots",CLIPS_TRUE) == CLIPS_FALSE)
     return;
   EvaluateClassDefaults(GetActiveInstance());
   if (! EvaluationError)
     {
      SetpType(result,INSTANCE_ADDRESS);
      SetpValue(result,(VOID *) GetActiveInstance());
     }
  }
  
/******************************************************
  NAME         : QuashInstance
  DESCRIPTION  : Deletes an instance if it is not in
                   use, otherwise sticks it on the
                   garbage list
  INPUTS       : The instance
  RETURNS      : 1 if successful, 0 otherwise
  SIDE EFFECTS : Instance deleted or added to garbage
  NOTES        : Even though the instance is removed
                   from the class list, hash table and
                   instance list, its links remain
                   unchanged so that outside loops
                   can still determine where the next
                   node in the list is (assuming the
                   instance was garbage collected).
 ******************************************************/
globle BOOLEAN QuashInstance(ins)
  INSTANCE_TYPE *ins;
  {
   register int iflag;
   IGARBAGE *gptr;
   
#if INSTANCE_PATTERN_MATCHING
   if (JoinOperationInProgress && ins->cls->reactive)
     {
      PrintErrorID("INSMNGR",12,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Cannot delete instances of reactive classes while\n");
      PrintCLIPS(WERROR,"  pattern-matching is in process.\n");
      SetEvaluationError(CLIPS_TRUE);
      return(0);
     }
#endif
   if (ins->garbage == 1)
     return(0);
   if (ins->installed == 0)
     {
      PrintErrorID("INSMNGR",6,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Cannot delete instance ");
      PrintCLIPS(WERROR,ValueToString(ins->name));
      PrintCLIPS(WERROR," during initialization.\n");
      SetEvaluationError(CLIPS_TRUE);
      return(0);
     }
#if DEBUGGING_FUNCTIONS
   if (ins->cls->traceInstances)
     PrintInstanceWatch(UNMAKE_TRACE,ins);
#endif

#if LOGICAL_DEPENDENCIES
   RemoveEntityDependencies((struct patternEntity *) ins);
#endif

#if INSTANCE_PATTERN_MATCHING
   if (ins->cls->reactive)
     ObjectNetworkAction(OBJECT_RETRACT,(VOID *) ins,-1);
#endif

   if (ins->prvHash != NULL)
     ins->prvHash->nxtHash = ins->nxtHash;
   else
     InstanceTable[ins->hashTableIndex] = ins->nxtHash;
   if (ins->nxtHash != NULL)
     ins->nxtHash->prvHash = ins->prvHash;

   if (ins->prvClass != NULL)
     ins->prvClass->nxtClass = ins->nxtClass;
   else
     ins->cls->instanceList = ins->nxtClass;
   if (ins->nxtClass != NULL)
     ins->nxtClass->prvClass = ins->prvClass;
   else
     ins->cls->instanceListBottom = ins->prvClass;

   if (ins->prvList != NULL)
     ins->prvList->nxtList = ins->nxtList;
   else
     InstanceList = ins->nxtList;
   if (ins->nxtList != NULL)
     ins->nxtList->prvList = ins->prvList;
   else
     InstanceListBottom = ins->prvList;
     
   iflag = ins->installed;
   InstallInstance(ins,CLIPS_FALSE);
   
   /* ==============================================
      If the instance is the basis for an executing
      rule, don't bother deleting its slots yet, for
      they may still be needed by pattern variables
      ============================================== */
   if ((iflag == 1) 
#if INSTANCE_PATTERN_MATCHING
       && (ins->header.busyCount == 0)
#endif
     )
     RemoveInstanceData(ins);

   if ((ins->busy == 0) && (ins->depth > CurrentEvaluationDepth) &&
       (MaintainGarbageInstances == CLIPS_FALSE)
#if INSTANCE_PATTERN_MATCHING
        && (ins->header.busyCount == 0)
#endif
       )
     {
      DecrementSymbolCount(ins->name);
      rtn_struct(instance,ins);
     }
   else
     {
      gptr = get_struct(igarbage);
      ins->garbage = 1;
      gptr->ins = ins;
      gptr->nxt = InstanceGarbageList;
      InstanceGarbageList = gptr;
      EphemeralItemCount += 2;
      EphemeralItemSize += InstanceSizeHeuristic(ins) + sizeof(IGARBAGE);
     }
   ChangesToInstances = CLIPS_TRUE;
   return(1);
  }
  

#if INSTANCE_PATTERN_MATCHING

/****************************************************
  NAME         : InactiveInitializeInstance
  DESCRIPTION  : Initializes an instance of a class
                 Pattern-matching is automatically
                 delayed until the instance is
                 completely initialized
  INPUTS       : The address of the result value
  RETURNS      : Nothing useful
  SIDE EFFECTS : Instance intialized
  NOTES        : CLIPS Syntax:
                 (initialize-instance <instance-name>
                    <slot-override>*)
 ****************************************************/
globle VOID InactiveInitializeInstance(result)
  DATA_OBJECT *result;
  {
   int ov;
   
   ov = SetDelayObjectPatternMatching(CLIPS_TRUE);
   InitializeInstanceCommand(result);
   SetDelayObjectPatternMatching(ov);
  }
  
/**************************************************************
  NAME         : InactiveMakeInstance
  DESCRIPTION  : Creates and initializes an instance of a class
                 Pattern-matching is automatically
                 delayed until the instance is
                 completely initialized
  INPUTS       : The address of the result value
  RETURNS      : Nothing useful
  SIDE EFFECTS : Instance intialized
  NOTES        : CLIPS Syntax:
                 (make-instance <instance-name> of <class>
                    <slot-override>*)
 **************************************************************/
globle VOID InactiveMakeInstance(result)
  DATA_OBJECT *result;
  {
   int ov;
   
   ov = SetDelayObjectPatternMatching(CLIPS_TRUE);
   MakeInstanceCommand(result);
   SetDelayObjectPatternMatching(ov);
  }
  
#endif

/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/********************************************************
  NAME         : NewInstance
  DESCRIPTION  : Allocates and initializes a new instance
  INPUTS       : None
  RETURNS      : The address of the new instance
  SIDE EFFECTS : None
  NOTES        : None
 ********************************************************/
static INSTANCE_TYPE *NewInstance()
  {
   INSTANCE_TYPE *instance;
   
   instance = get_struct(instance);
#if INSTANCE_PATTERN_MATCHING || LOGICAL_DEPENDENCIES
   instance->header.theInfo = &InstanceInfo;
#if LOGICAL_DEPENDENCIES
   instance->header.dependents = NULL;
#endif
   instance->header.busyCount = 0;
   instance->header.timeTag = 0L;
#endif
#if INSTANCE_PATTERN_MATCHING
   instance->partialMatchList = NULL;
   instance->basisSlots = NULL;
#endif
   instance->busy = 0;
   instance->installed = 0;
   instance->garbage = 0;
   instance->initializeInProgress = 0;
   instance->depth = CurrentEvaluationDepth;
   instance->name = NULL;
   instance->hashTableIndex = 0;
   instance->cls = NULL;
   instance->slots = NULL;
   instance->slotAddresses = NULL;
   instance->prvClass = NULL;
   instance->nxtClass = NULL;
   instance->prvHash = NULL;
   instance->nxtHash = NULL;
   instance->prvList = NULL;
   instance->nxtList = NULL;
   return(instance);
  }

/*****************************************************************
  NAME         : InstanceLocationInfo
  DESCRIPTION  : Determines where a specified instance belongs
                   in the instance hash table
  INPUTS       : 1) The class of the new instance
                 2) The CLIPS symbol for the name of the instance
                 3) Caller's buffer for previous node address
                 4) Caller's buffer for hash value
  RETURNS      : The address of the found instance, NULL otherwise
  SIDE EFFECTS : None
  NOTES        : Instance names only have to be unique within
                 a module
 *****************************************************************/
static INSTANCE_TYPE *InstanceLocationInfo(cls,iname,prv,hashTableIndex)
  DEFCLASS *cls;
  SYMBOL_HN *iname;
  INSTANCE_TYPE **prv;
  unsigned *hashTableIndex;
  {
   INSTANCE_TYPE *ins;
   
   *hashTableIndex = HashInstance(iname);
   ins = InstanceTable[*hashTableIndex];
   
   /* ========================================
      Make sure all instances of the same name
      are grouped together regardless of what
      module their classes are in
      ======================================== */
   *prv = NULL;
   while ((ins != NULL) ? (ins->name != iname) : CLIPS_FALSE)
     {
      *prv = ins;
      ins = ins->nxtHash;
     }
   while ((ins != NULL) ? (ins->name == iname) : CLIPS_FALSE)
     {
      if (ins->cls->header.whichModule->theModule ==
          cls->header.whichModule->theModule)
        return(ins);
      *prv = ins;
      ins = ins->nxtHash;
     }
   return(NULL);
  }
  
/********************************************************
  NAME         : InstallInstance
  DESCRIPTION  : Prevent name and slot value symbols
                   from being ephemeral (all others
                   taken care of by class defn)
  INPUTS       : 1) The address of the instance
                 2) A flag indicating whether to
                    install or deinstall
  RETURNS      : Nothing useful
  SIDE EFFECTS : Symbol counts incremented or decremented
  NOTES        : Slot symbol installations are handled
                   by PutSlotValue
 ********************************************************/
static VOID InstallInstance(ins,set)
  INSTANCE_TYPE *ins;
  int set;
  {
   if (set == CLIPS_TRUE) 
     {
      if (ins->installed)
        return;
#if DEBUGGING_FUNCTIONS
      if (ins->cls->traceInstances)
        PrintInstanceWatch(MAKE_TRACE,ins);
#endif
      ins->installed = 1;
      ins->depth = CurrentEvaluationDepth;
      IncrementSymbolCount(ins->name);
      IncrementDefclassBusyCount((VOID *) ins->cls);
      GlobalNumberOfInstances++;
     }
   else
     {
      if (! ins->installed)
        return;
      ins->installed = 0;
      GlobalNumberOfInstances--;
      
      /* =======================================
         Class counts is decremented by
         RemoveInstanceData() when slot data is
         truly deleted - and name count is
         deleted by CleanupInstances() or
         QuashInstance() when instance is
         truly deleted
         ======================================= */
     }
  }
  
/****************************************************************
  NAME         : BuildDefaultSlots
  DESCRIPTION  : The current instance's address is
                   in the global variable CurrentInstance.
                   This builds the slots and the default values
                   from the direct class of the instance and its
                   inheritances.
  INPUTS       : Flag indicating whether init message will be
                 called for this instance or not
  RETURNS      : Nothing useful
  SIDE EFFECTS : Allocates the slot array for
                   the current instance
  NOTES        : The current instance's address is
                 stored in a global variable
 ****************************************************************/
static VOID BuildDefaultSlots(initMessage)
  BOOLEAN initMessage;
  {
   register int i,j;
   int scnt,lscnt;
   INSTANCE_SLOT *dst,**adst;
   SLOT_DESC **src;
   
   scnt = CurrentInstance->cls->instanceSlotCount;
   lscnt = CurrentInstance->cls->localInstanceSlotCount;
   if (scnt > 0)
     {
      CurrentInstance->slotAddresses = adst =
         (INSTANCE_SLOT **) gm2((int) (sizeof(INSTANCE_SLOT *) * scnt));
      if (lscnt != 0)
        CurrentInstance->slots = dst =
           (INSTANCE_SLOT *) gm2((int) (sizeof(INSTANCE_SLOT) * lscnt));
      src = CurrentInstance->cls->instanceTemplate;
      
      /* ==================================================
         A map of slot addresses is created - shared slots
         point at values in the class, and local slots
         point at values in the instance
         
         Also - slots are always given an initial value
         (since slots cannot be unbound). If there is
         already an instance of a class with a shared slot,
         that value is left alone
         ================================================== */
      for (i = 0 , j = 0 ; i < scnt ; i++)
        {
         if (src[i]->shared)
           {
            src[i]->sharedCount++;
            adst[i] = &(src[i]->sharedValue);
           }
         else
           {
            dst[j].desc = src[i];
            dst[j].value = NULL;
            adst[i] = &dst[j++];
           }
         if (adst[i]->value == NULL)
           {
            adst[i]->valueRequired = initMessage;
            if (adst[i]->desc->multiple)
              {
               adst[i]->type = MULTIFIELD;
               adst[i]->value = CreateMultifield2(0);
               MultifieldInstall((MULTIFIELD_PTR) adst[i]->value);
              }
            else
              {
               adst[i]->type = SYMBOL;
               adst[i]->value = AddSymbol("nil");
               AtomInstall((int) adst[i]->type,adst[i]->value);
              }
           }
         else
           adst[i]->valueRequired = CLIPS_FALSE;
         adst[i]->override = CLIPS_FALSE;
        }
     }
  }
  
/*******************************************************************
  NAME         : CoreInitializeInstance
  DESCRIPTION  : Performs the core work for initializing an instance
  INPUTS       : 1) The instance address
                 2) Slot override expressions
  RETURNS      : CLIPS_TRUE if all OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : EvaluationError set on errors - slots evaluated
  NOTES        : None
 *******************************************************************/
static int CoreInitializeInstance(ins,ovrexp)
  INSTANCE_TYPE *ins;
  EXPRESSION *ovrexp;
  {
   register int svaccess;
   DATA_OBJECT temp;
   
   if (ins->installed == 0)
     {
      PrintErrorID("INSMNGR",7,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Instance ");
      PrintCLIPS(WERROR,ValueToString(ins->name));
      PrintCLIPS(WERROR," is already being initialized.\n");
      SetEvaluationError(CLIPS_TRUE);
      return(CLIPS_FALSE);
     }  
   
   /* =======================================================
      Replace all default-slot values with any slot-overrides
      ======================================================= */
   ins->busy++;
   ins->installed = 0;
   if (InsertSlotOverrides(ins,ovrexp) == CLIPS_FALSE)
      {
       ins->installed = 1;
       ins->busy--;
       return(CLIPS_FALSE);
      }
      
   /* =================================================================
      Now that all the slot expressions are established - replace them
      with their evaluation
      
      If the slots are initialized properly - the initializeInProgress
      flag will be turned off.
      ================================================================= */
   ins->initializeInProgress = 1;
   svaccess = WithinInit;
   WithinInit = CLIPS_TRUE;

   if (MkInsMsgPass)
     DirectMessage(INIT_SYMBOL,ins,&temp,NULL);
   else
     EvaluateClassDefaults(ins);
   WithinInit = svaccess;
   ins->busy--;
   ins->installed = 1;
   if (EvaluationError)
     {
      PrintErrorID("INSMNGR",8,CLIPS_FALSE);
      PrintCLIPS(WERROR,"An error occurred during the initialization of instance ");
      PrintCLIPS(WERROR,ValueToString(ins->name));
      PrintCLIPS(WERROR,".\n");
      return(CLIPS_FALSE);
     }
   return((ins->initializeInProgress == 1) ? CLIPS_FALSE : CLIPS_TRUE);
  }           

/**********************************************************
  NAME         : InsertSlotOverrides
  DESCRIPTION  : Replaces value-expression for a slot
  INPUTS       : 1) The instance address
                 2) The address of the beginning of the
                    list of slot-expressions
  RETURNS      : CLIPS_TRUE if all okay, CLIPS_FALSE otherwise
  SIDE EFFECTS : Old slot expression deallocated
  NOTES        : Assumes symbols not yet installed
                 EVALUATES the slot-name expression but
                    simply copies the slot value-expression
 **********************************************************/
static int InsertSlotOverrides(ins,slot_exp)
  INSTANCE_TYPE *ins;
  EXPRESSION *slot_exp;
  {
   INSTANCE_SLOT *slot;
   int svaccess;
   DATA_OBJECT temp;
   
   EvaluationError = CLIPS_FALSE;
   svaccess = WithinInit;
   while (slot_exp != NULL)
     {
      if ((EvaluateExpression(slot_exp,&temp) == CLIPS_TRUE) ? CLIPS_TRUE :
          (GetType(temp) != SYMBOL))
        {
         PrintErrorID("INSMNGR",9,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Expected a valid slot name for slot-override.\n");
         SetEvaluationError(CLIPS_TRUE);
         return(CLIPS_FALSE);
        }
      slot = FindInstanceSlot(ins,(SYMBOL_HN *) GetValue(temp));
      if (slot == NULL)
        {
         PrintErrorID("INSMNGR",13,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Slot ");
         PrintCLIPS(WERROR,DOToString(temp));
         PrintCLIPS(WERROR," does not exist in instance ");
         PrintCLIPS(WERROR,ValueToString(ins->name));
         PrintCLIPS(WERROR,".\n");
         SetEvaluationError(CLIPS_TRUE);
         return(CLIPS_FALSE);
        }
      WithinInit = CLIPS_TRUE;
      if (MkInsMsgPass)
        { DirectMessage(slot->desc->overrideMessage,
                       ins,NULL,slot_exp->nextArg->argList); }
      else
        {
         if (EvaluateAndStoreInDataObject((int) slot->desc->multiple,
                               slot_exp->nextArg->argList,&temp))
             PutSlotValue(ins,slot,&temp,"function make-instance");
        }
      WithinInit = svaccess;
      if (EvaluationError)
        return(CLIPS_FALSE);
      slot->override = CLIPS_TRUE;
      slot_exp = slot_exp->nextArg->nextArg;
     }
   return(CLIPS_TRUE);
  }
  
/*****************************************************************************
  NAME         : EvaluateClassDefaults
  DESCRIPTION  : Evaluates default slots only - slots that were specified
                 by overrides (sp->override == 1) are ignored)
  INPUTS       : 1) Instance address
  RETURNS      : Nothing useful
  SIDE EFFECTS : Each DATA_OBJECT slot in the instance's slot array is replaced
                   by the evaluation (by EvaluateExpression) of the expression
                   in the slot list.  The old expression-values
                   are deleted.
  NOTES        : None
 *****************************************************************************/
static VOID EvaluateClassDefaults(ins)
  INSTANCE_TYPE *ins;
  {
   INSTANCE_SLOT *slot;
   DATA_OBJECT temp;
   register int i;

   if (ins->initializeInProgress == 0)
     {
      PrintErrorID("INSMNGR",15,CLIPS_FALSE);
      SetEvaluationError(CLIPS_TRUE);
      PrintCLIPS(WERROR,"init-slots not valid in this context.\n");
      return;
     }
   for (i = 0 ; i < ins->cls->instanceSlotCount ; i++)
     {
      slot = ins->slotAddresses[i];
      
      /* ===========================================================
         Slot-overrides are just a short-hand for put-slots, so they
         should be done with messages.  Defaults are from the class
         definition and can be placed directly.
         =========================================================== */
      if (slot->override)
        slot->override = CLIPS_FALSE;
      else if (slot->desc->dynamicDefault)
        {
         if (EvaluateAndStoreInDataObject((int) slot->desc->multiple,
                                          (EXPRESSION *) slot->desc->defaultValue,
                                          &temp))
           PutSlotValue(ins,slot,&temp,"function init-slots");
        }
      else if (((slot->desc->shared == 0) || (slot->desc->sharedCount == 1)) &&
               (slot->desc->noDefault == 0))
        DirectPutSlotValue(ins,slot,(DATA_OBJECT *) slot->desc->defaultValue);
      else if (slot->valueRequired)
        {
         PrintErrorID("INSMNGR",14,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Override required for slot ");
         PrintCLIPS(WERROR,ValueToString(slot->desc->slotName->name));
         PrintCLIPS(WERROR," in instance ");
         PrintCLIPS(WERROR,ValueToString(ins->name));
         PrintCLIPS(WERROR,".\n");
         SetEvaluationError(CLIPS_TRUE);
        }
      slot->valueRequired = CLIPS_FALSE;
      if (ins->garbage == 1)
        {
         PrintCLIPS(WERROR,ValueToString(ins->name));
         PrintCLIPS(WERROR," instance deleted by slot-override evaluation.\n");
         SetEvaluationError(CLIPS_TRUE);
        }
      if (EvaluationError)
        return;
     }
   ins->initializeInProgress = 0;
  }

#if DEBUGGING_FUNCTIONS

/***************************************************
  NAME         : PrintInstanceWatch
  DESCRIPTION  : Prints out a trace message for the
                 creation/deletion of an instance
  INPUTS       : 1) The trace string indicating if
                    this is a creation or deletion
                 2) The instance
  RETURNS      : Nothing usful
  SIDE EFFECTS : Watch message printed
  NOTES        : None
 ***************************************************/
static VOID PrintInstanceWatch(traceString,theInstance)
  char *traceString;
  INSTANCE_TYPE *theInstance;
  {
   PrintCLIPS(WTRACE,traceString);
   PrintCLIPS(WTRACE," instance ");
   PrintInstanceNameAndClass(WTRACE,theInstance,CLIPS_TRUE);
  }
  
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


