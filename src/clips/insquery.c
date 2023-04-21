   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                                                     */
   /*******************************************************/

/*************************************************************/
/* Purpose: Query Functions for Objects                      */
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

#if INSTANCE_SET_QUERIES

#include "argacces.h"
#include "classcom.h"
#include "classfun.h"
#include "clipsmem.h"
#include "exprnpsr.h"
#include "insfun.h"
#include "insmngr.h"
#include "insqypsr.h"
#include "prcdrfun.h"
#include "router.h"
#include "utility.h"

#define _INSQUERY_SOURCE_
#include "insquery.h"

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
typedef struct query_class
  {
   DEFCLASS *cls;
   struct defmodule *theModule;
   struct query_class *chain,*nxt;
  } QUERY_CLASS;

typedef struct query_soln
  {
   INSTANCE_TYPE **soln;
   struct query_soln *nxt;
  } QUERY_SOLN;
  
typedef struct query_core
  {
   INSTANCE_TYPE **solns;
   EXPRESSION *query,*action;
   QUERY_SOLN *soln_set,*soln_bottom;
   int soln_size,soln_cnt;
   DATA_OBJECT *result;
  } QUERY_CORE;
  
typedef struct query_stack
  {
   QUERY_CORE *core;
   struct query_stack *nxt;
  } QUERY_STACK;
  
/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER

static VOID PushQueryCore(void);
static VOID PopQueryCore(void);
static QUERY_CORE *FindQueryCore(int);
static QUERY_CLASS *DetermineQueryClasses(EXPRESSION *,char *,int *);
static QUERY_CLASS *FormChain(char *,DATA_OBJECT *);
static VOID DeleteQueryClasses(QUERY_CLASS *);
static int TestForFirstInChain(QUERY_CLASS *,int);
static int TestForFirstInstanceInClass(struct defmodule *,int,DEFCLASS *,QUERY_CLASS *,int);
static VOID TestEntireChain(QUERY_CLASS *,int);
static VOID TestEntireClass(struct defmodule *,int,DEFCLASS *,QUERY_CLASS *,int);
static VOID AddSolution(void);
static VOID PopQuerySoln(void);

#else

static VOID PushQueryCore();
static VOID PopQueryCore();
static QUERY_CORE *FindQueryCore();
static QUERY_CLASS *DetermineQueryClasses();
static QUERY_CLASS *FormChain();
static VOID DeleteQueryClasses();
static int TestForFirstInChain();
static int TestForFirstInstanceInClass();
static VOID TestEntireChain();
static VOID TestEntireClass();
static VOID AddSolution();
static VOID PopQuerySoln();

#endif
      
/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
globle SYMBOL_HN *QUERY_DELIMETER_SYMBOL = NULL;
   
/* =========================================
   *****************************************
      INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
static QUERY_CORE *QueryCore = NULL;
static QUERY_STACK *QueryCoreStack = NULL;
static int AbortQuery = CLIPS_FALSE;

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
/****************************************************
  NAME         : SetupQuery
  DESCRIPTION  : Initializes instance query CLIPS
                   functions and parsers
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Sets up kernel functions and parsers
  NOTES        : None
 ****************************************************/
globle VOID SetupQuery()
  {
#if ! RUN_TIME
   QUERY_DELIMETER_SYMBOL = (SYMBOL_HN *) AddSymbol(QUERY_DELIMETER_STRING);
   IncrementSymbolCount(QUERY_DELIMETER_SYMBOL);

   DefineFunction2("(query-instance)",'o',
                  PTIF GetQueryInstance,"GetQueryInstance",NULL);

   DefineFunction2("(query-instance-slot)",'u',
                  PTIF GetQueryInstanceSlot,"GetQueryInstanceSlot",NULL);

   DefineFunction2("any-instancep",'b',PTIF AnyInstances,"AnyInstances",NULL);
   AddFunctionParser("any-instancep",ParseQueryNoAction);

   DefineFunction2("find-instance",'m',
                  PTIF QueryFindInstance,"QueryFindInstance",NULL);
   AddFunctionParser("find-instance",ParseQueryNoAction);

   DefineFunction2("find-all-instances",'m',
                  PTIF QueryFindAllInstances,"QueryFindAllInstances",NULL);
   AddFunctionParser("find-all-instances",ParseQueryNoAction);

   DefineFunction2("do-for-instance",'u',
                  PTIF QueryDoForInstance,"QueryDoForInstance",NULL);
   AddFunctionParser("do-for-instance",ParseQueryAction);

   DefineFunction2("do-for-all-instances",'u',
                  PTIF QueryDoForAllInstances,"QueryDoForAllInstances",NULL);
   AddFunctionParser("do-for-all-instances",ParseQueryAction);

   DefineFunction2("delayed-do-for-all-instances",'u',
                  PTIF DelayedQueryDoForAllInstances,
                  "DelayedQueryDoForAllInstances",NULL);
   AddFunctionParser("delayed-do-for-all-instances",ParseQueryAction);
#endif
  }
  
/*************************************************************
  NAME         : GetQueryInstance
  DESCRIPTION  : Internal function for referring to instance
                    array on instance-queries
  INPUTS       : None
  RETURNS      : The name of the specified instance-set member
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax : ((query-instance) <index>)
 *************************************************************/
globle SYMBOL_HN *GetQueryInstance()
  {
   register QUERY_CORE *core;
   
   core = FindQueryCore(DOPToInteger(GetFirstArgument()));
   return(core->solns[DOPToInteger(GetFirstArgument()->nextArg)]->name);
  }
  
/***************************************************************************
  NAME         : GetQueryInstanceSlot
  DESCRIPTION  : Internal function for referring to slots of instances in
                    instance array on instance-queries
  INPUTS       : The caller's result buffer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Caller's result buffer set appropriately
  NOTES        : CLIPS Syntax : ((query-instance-slot) <index> <slot-name>)
 **************************************************************************/
globle VOID GetQueryInstanceSlot(result)
  DATA_OBJECT *result;
  {
   INSTANCE_TYPE *ins;
   INSTANCE_SLOT *sp;
   DATA_OBJECT temp;
   QUERY_CORE *core;
   
   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   
   core = FindQueryCore(DOPToInteger(GetFirstArgument()));
   ins = core->solns[DOPToInteger(GetFirstArgument()->nextArg)];
   EvaluateExpression(GetFirstArgument()->nextArg->nextArg,&temp);
   if (temp.type != SYMBOL)
     {
      ExpectedTypeError1("get",1,"symbol");
      SetEvaluationError(CLIPS_TRUE);
      return;
     }
   sp = FindInstanceSlot(ins,(SYMBOL_HN *) temp.value);
   if (sp == NULL)
     {
      SlotExistError(ValueToString(temp.value),"instance-set query");
      return;
     }
   result->type = sp->type;
   result->value = sp->value;
   if (sp->type == MULTIFIELD)
     {
      result->begin = 0;
      result->end = GetInstanceSlotLength(sp) - 1;
     }
  }
  
/* =============================================================================
   =============================================================================
   Following are the instance query functions :
   
     any-instancep         : Determines if any instances satisfy the query
     find-instance         : Finds first (set of) instance(s) which satisfies
                               the query and stores it in a multi-field
     find-all-instances    : Finds all (sets of) instances which satisfy the
                               the query and stores them in a multi-field
     do-for-instance       : Executes a given action for the first (set of)
                               instance(s) which satisfy the query
     do-for-all-instances  : Executes an action for all instances which satisfy
                               the query as they are found
     delayed-do-for-all-instances : Same as above - except that the list of instances
                               which satisfy the query is formed before any
                               actions are executed
                               
     Instance candidate search algorithm :
     
     All permutations of first restriction class instances with other
       restriction class instances (Rightmost are varied first)
     All permutations of first restriction class's subclasses' instances with
       other restriction class instances.
     And  so on...
     
     For any one class, instances are examined in the order they were defined
     
     Example :
     (defclass a (is-a standard-user))
     (defclass b (is-a standard-user))
     (defclass c (is-a standard-user))
     (defclass d (is-a a b))
     (make-instance a1 of a)
     (make-instance a2 of a)
     (make-instance b1 of b)
     (make-instance b2 of b)
     (make-instance c1 of c)
     (make-instance c2 of c)
     (make-instance d1 of d)
     (make-instance d2 of d)
     
     (any-instancep ((?a a b) (?b c)) <query>)
     
     The permutations (?a ?b) would be examined in the following order :
     
     (a1 c1),(a1 c2),(a2 c1),(a2 c2),(d1 c1),(d1 c2),(d2 c1),(d2 c2),
     (b1 c1),(b1 c2),(b2 c1),(b2 c2),(d1 c1),(d1 c2),(d2 c1),(d2 c2)
     
     Notice the duplication because d is a subclass of both and a and b.
   =============================================================================
   ============================================================================= */
   
/******************************************************************************
  NAME         : AnyInstances
  DESCRIPTION  : Determines if there any existing instances which satisfy
                   the query
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if the query is satisfied, CLIPS_FALSE otherwise
  SIDE EFFECTS : The query class-expressions are evaluated once,
                   and the query boolean-expression is evaluated
                   zero or more times (depending on instance restrictions
                   and how early the expression evaulates to CLIPS_TRUE - if at all).
  NOTES        : CLIPS Syntax : See ParseQueryNoAction()
 ******************************************************************************/
globle BOOLEAN AnyInstances()
  {
   QUERY_CLASS *qclasses;
   int rcnt;
   int TestResult;
   
   qclasses = DetermineQueryClasses(GetFirstArgument()->nextArg,
                                      "any-instancep",&rcnt);
   if (qclasses == NULL)
     return(CLIPS_FALSE);
   PushQueryCore();
   QueryCore = get_struct(query_core);
   QueryCore->solns = (INSTANCE_TYPE **) gm2((int) (sizeof(INSTANCE_TYPE *) * rcnt));
   QueryCore->query = GetFirstArgument();
   TestResult = TestForFirstInChain(qclasses,0);
   AbortQuery = CLIPS_FALSE;
   rm((VOID *) QueryCore->solns,(int) (sizeof(INSTANCE_TYPE *) * rcnt));
   rtn_struct(query_core,QueryCore);
   PopQueryCore();
   DeleteQueryClasses(qclasses);
   return(TestResult);
  }
  
/******************************************************************************
  NAME         : QueryFindInstance
  DESCRIPTION  : Finds the first set of instances which satisfy the query and
                   stores their names in the user's multi-field variable
  INPUTS       : Caller's result buffer
  RETURNS      : CLIPS_TRUE if the query is satisfied, CLIPS_FALSE otherwise
  SIDE EFFECTS : The query class-expressions are evaluated once,
                   and the query boolean-expression is evaluated
                   zero or more times (depending on instance restrictions
                   and how early the expression evaulates to CLIPS_TRUE - if at all).
  NOTES        : CLIPS Syntax : See ParseQueryNoAction()
 ******************************************************************************/
globle VOID QueryFindInstance(result)
  DATA_OBJECT *result;
  {
   QUERY_CLASS *qclasses;
   int rcnt,i;
   
   result->type = MULTIFIELD;
   result->begin = 0;
   result->end = -1;
   qclasses = DetermineQueryClasses(GetFirstArgument()->nextArg,
                                      "find-instance",&rcnt);
   if (qclasses == NULL)
     {
      result->value = (VOID *) CreateMultifield(0);
      return;
     }
   PushQueryCore();
   QueryCore = get_struct(query_core);
   QueryCore->solns = (INSTANCE_TYPE **) 
                      gm2((int) (sizeof(INSTANCE_TYPE *) * rcnt));
   QueryCore->query = GetFirstArgument();
   if (TestForFirstInChain(qclasses,0) == CLIPS_TRUE)
     {
      result->value = (VOID *) CreateMultifield(rcnt);
      result->end = rcnt-1;
      for (i = 1 ; i <= rcnt ; i++)
        {
         SetMFType(result->value,i,INSTANCE_NAME);
         SetMFValue(result->value,i,GetFullInstanceName(QueryCore->solns[i - 1]));
        }
     }
   else
      result->value = (VOID *) CreateMultifield(0);
   AbortQuery = CLIPS_FALSE;
   rm((VOID *) QueryCore->solns,(int) (sizeof(INSTANCE_TYPE *) * rcnt));
   rtn_struct(query_core,QueryCore);
   PopQueryCore();
   DeleteQueryClasses(qclasses);
  }
  
/******************************************************************************
  NAME         : QueryFindAllInstances
  DESCRIPTION  : Finds all sets of instances which satisfy the query and
                   stores their names in the user's multi-field variable
                   
                 The sets are stored sequentially :
                 
                   Number of sets = (Multi-field length) / (Set length)
                 
                 The first set is if the first (set length) atoms of the
                   multi-field variable, and so on.
  INPUTS       : Caller's result buffer
  RETURNS      : Nothing useful
  SIDE EFFECTS : The query class-expressions are evaluated once,
                   and the query boolean-expression is evaluated
                   once for every instance set.
  NOTES        : CLIPS Syntax : See ParseQueryNoAction()
 ******************************************************************************/
globle VOID QueryFindAllInstances(result)
  DATA_OBJECT *result;
  {
   QUERY_CLASS *qclasses;
   int rcnt;
   register int i,j;
   
   result->type = MULTIFIELD;
   result->begin = 0;
   result->end = -1;
   qclasses = DetermineQueryClasses(GetFirstArgument()->nextArg,
                                      "find-all-instances",&rcnt);
   if (qclasses == NULL)
     {
      result->value = (VOID *) CreateMultifield(0);
      return;
     }
   PushQueryCore();
   QueryCore = get_struct(query_core);
   QueryCore->solns = (INSTANCE_TYPE **) gm2((int) (sizeof(INSTANCE_TYPE *) * rcnt));
   QueryCore->query = GetFirstArgument();
   QueryCore->action = NULL;
   QueryCore->soln_set = NULL;
   QueryCore->soln_size = rcnt;
   QueryCore->soln_cnt = 0;
   TestEntireChain(qclasses,0);
   AbortQuery = CLIPS_FALSE;
   result->value = (VOID *) CreateMultifield(QueryCore->soln_cnt * rcnt);
   while (QueryCore->soln_set != NULL)
     {
      for (i = 0 , j = result->end + 2 ; i < rcnt ; i++ , j++)
        {
         SetMFType(result->value,j,INSTANCE_NAME);
         SetMFValue(result->value,j,GetFullInstanceName(QueryCore->soln_set->soln[i]));
        }
      result->end = j-2;
      PopQuerySoln();
     }
   rm((VOID *) QueryCore->solns,(int) (sizeof(INSTANCE_TYPE *) * rcnt));
   rtn_struct(query_core,QueryCore);
   PopQueryCore();
   DeleteQueryClasses(qclasses);
  }
  
/******************************************************************************
  NAME         : QueryDoForInstance
  DESCRIPTION  : Finds the first set of instances which satisfy the query and
                   executes a user-action with that set
  INPUTS       : None
  RETURNS      : Caller's result buffer
  SIDE EFFECTS : The query class-expressions are evaluated once,
                   and the query boolean-expression is evaluated
                   zero or more times (depending on instance restrictions
                   and how early the expression evaulates to CLIPS_TRUE - if at all).
                   Also the action expression is executed zero or once.
                 Caller's result buffer holds result of user-action
  NOTES        : CLIPS Syntax : See ParseQueryAction()
 ******************************************************************************/
globle VOID QueryDoForInstance(result)
  DATA_OBJECT *result;
  {
   QUERY_CLASS *qclasses;
   int rcnt;

   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   qclasses = DetermineQueryClasses(GetFirstArgument()->nextArg->nextArg,
                                      "do-for-instance",&rcnt);
   if (qclasses == NULL)
     return;
   PushQueryCore();
   QueryCore = get_struct(query_core);
   QueryCore->solns = (INSTANCE_TYPE **) gm2((int) (sizeof(INSTANCE_TYPE *) * rcnt));
   QueryCore->query = GetFirstArgument();
   QueryCore->action = GetFirstArgument()->nextArg;
   if (TestForFirstInChain(qclasses,0) == CLIPS_TRUE)
     EvaluateExpression(QueryCore->action,result);
   AbortQuery = CLIPS_FALSE;
   BreakFlag = CLIPS_FALSE;
   rm((VOID *) QueryCore->solns,(int) (sizeof(INSTANCE_TYPE *) * rcnt));
   rtn_struct(query_core,QueryCore);
   PopQueryCore();
   DeleteQueryClasses(qclasses);
  }
  
/******************************************************************************
  NAME         : QueryDoForAllInstances
  DESCRIPTION  : Finds all sets of instances which satisfy the query and
                   executes a user-function for each set as it is found
  INPUTS       : Caller's result buffer
  RETURNS      : Nothing useful
  SIDE EFFECTS : The query class-expressions are evaluated once,
                   and the query boolean-expression is evaluated
                   once for every instance set.  Also, the action is
                   executed for every instance set.
                 Caller's result buffer holds result of last action executed.
  NOTES        : CLIPS Syntax : See ParseQueryAction()
 ******************************************************************************/
globle VOID QueryDoForAllInstances(result)
  DATA_OBJECT *result;
  {
   QUERY_CLASS *qclasses;
   int rcnt;

   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   qclasses = DetermineQueryClasses(GetFirstArgument()->nextArg->nextArg,
                                      "do-for-all-instances",&rcnt);
   if (qclasses == NULL)
     return;
   PushQueryCore();
   QueryCore = get_struct(query_core);
   QueryCore->solns = (INSTANCE_TYPE **) gm2((int) (sizeof(INSTANCE_TYPE *) * rcnt));
   QueryCore->query = GetFirstArgument();
   QueryCore->action = GetFirstArgument()->nextArg;
   QueryCore->result = result;
   ValueInstall(QueryCore->result);
   TestEntireChain(qclasses,0);
   ValueDeinstall(QueryCore->result);
   PropagateReturnValue(QueryCore->result);
   AbortQuery = CLIPS_FALSE;
   BreakFlag = CLIPS_FALSE;
   rm((VOID *) QueryCore->solns,(int) (sizeof(INSTANCE_TYPE *) * rcnt));
   rtn_struct(query_core,QueryCore);
   PopQueryCore();
   DeleteQueryClasses(qclasses);
  }
  
/******************************************************************************
  NAME         : DelayedQueryDoForAllInstances
  DESCRIPTION  : Finds all sets of instances which satisfy the query and
                   and exceutes a user-action for each set
                   
                 This function differs from QueryDoForAllInstances() in
                   that it forms the complete list of query satisfactions
                   BEFORE executing any actions. 
  INPUTS       : Caller's result buffer
  RETURNS      : Nothing useful
  SIDE EFFECTS : The query class-expressions are evaluated once,
                   and the query boolean-expression is evaluated
                   once for every instance set.  The action is executed
                   for evry query satisfaction.
                 Caller's result buffer holds result of last action executed.
  NOTES        : CLIPS Syntax : See ParseQueryNoAction()
 ******************************************************************************/
globle VOID DelayedQueryDoForAllInstances(result)
  DATA_OBJECT *result;
  {
   QUERY_CLASS *qclasses;
   int rcnt;
   register int i;
   
   result->type = SYMBOL;
   result->value = CLIPSFalseSymbol;
   qclasses = DetermineQueryClasses(GetFirstArgument()->nextArg->nextArg,
                                      "delayed-do-for-all-instances",&rcnt);
   if (qclasses == NULL)
     return;
   PushQueryCore();
   QueryCore = get_struct(query_core);
   QueryCore->solns = (INSTANCE_TYPE **) gm2((int) (sizeof(INSTANCE_TYPE *) * rcnt));
   QueryCore->query = GetFirstArgument();
   QueryCore->action = NULL;
   QueryCore->soln_set = NULL;
   QueryCore->soln_size = rcnt;
   QueryCore->soln_cnt = 0;
   TestEntireChain(qclasses,0);
   AbortQuery = CLIPS_FALSE;
   QueryCore->action = GetFirstArgument()->nextArg;
   while (QueryCore->soln_set != NULL)
     {
      for (i = 0 ; i < rcnt ; i++)
        QueryCore->solns[i] = QueryCore->soln_set->soln[i];
      PopQuerySoln();
      CurrentEvaluationDepth++;
      EvaluateExpression(QueryCore->action,result);
      CurrentEvaluationDepth--;
      PeriodicCleanup(CLIPS_FALSE,CLIPS_TRUE);
      if (HaltExecution || BreakFlag || ReturnFlag)
        {
         while (QueryCore->soln_set != NULL)
           PopQuerySoln();
         break;
        }
     }
   BreakFlag = CLIPS_FALSE;
   rm((VOID *) QueryCore->solns,(int) (sizeof(INSTANCE_TYPE *) * rcnt));
   rtn_struct(query_core,QueryCore);
   PopQueryCore();
   DeleteQueryClasses(qclasses);
  }
  
/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/*******************************************************
  NAME         : PushQueryCore
  DESCRIPTION  : Pushes the current QueryCore onto stack
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Allocates new stack node and changes
                   QueryCoreStack
  NOTES        : None
 *******************************************************/
static VOID PushQueryCore()
  {
   QUERY_STACK *qptr;
   
   qptr = get_struct(query_stack);
   qptr->core = QueryCore;
   qptr->nxt = QueryCoreStack;
   QueryCoreStack = qptr;
  }
  
/******************************************************
  NAME         : PopQueryCore
  DESCRIPTION  : Pops top of QueryCore stack and
                   restores QueryCore to this core
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Stack node deallocated, QueryCoreStack
                   changed and QueryCore reset
  NOTES        : Assumes stack is not empty
 ******************************************************/
static VOID PopQueryCore()
  {
   QUERY_STACK *qptr;
   
   QueryCore = QueryCoreStack->core;
   qptr = QueryCoreStack;
   QueryCoreStack = QueryCoreStack->nxt;
   rtn_struct(query_stack,qptr);
  }
  
/***************************************************
  NAME         : FindQueryCore
  DESCRIPTION  : Looks up a QueryCore Stack Frame
                    Depth 0 is current frame
                    1 is next deepest, etc.
  INPUTS       : Depth
  RETURNS      : Address of query core stack frame
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
static QUERY_CORE *FindQueryCore(depth)
  int depth;
  {
   QUERY_STACK *qptr;
   
   if (depth == 0)
     return(QueryCore);
   qptr = QueryCoreStack;
   while (depth > 1)
     {
      qptr = qptr->nxt;
      depth--;
     }
   return(qptr->core);
  }
  
/**********************************************************
  NAME         : DetermineQueryClasses
  DESCRIPTION  : Builds a list of classes to be used in
                   instance queries - uses parse form.
  INPUTS       : 1) The parse class expression chain
                 2) The name of the function being executed
                 3) Caller's buffer for restriction count
                    (# of separate lists)
  RETURNS      : The query list, or NULL on errors
  SIDE EFFECTS : Memory allocated for list
                 Busy count incremented for all classes
  NOTES        : Each restriction is linked by nxt pointer,
                   multiple classes in a restriction are
                   linked by the chain pointer.
                 Rcnt caller's buffer is set to reflect the
                   total number of chains
                 Assumes classExp is not NULL and that each
                   restriction chain is terminated with
                   the QUERY_DELIMITER_SYMBOL "(QDS)"
 **********************************************************/
static QUERY_CLASS *DetermineQueryClasses(classExp,func,rcnt)
  EXPRESSION *classExp;
  char *func;
  int *rcnt;
  {
   QUERY_CLASS *clist = NULL,*cnxt = NULL,*cchain = NULL,*tmp;
   int new_list = CLIPS_FALSE;
   DATA_OBJECT temp;
   
   *rcnt = 0;
   while (classExp != NULL)
     {
      if (EvaluateExpression(classExp,&temp))
        {
         DeleteQueryClasses(clist);
         return(NULL);
        }
      if ((temp.type == SYMBOL) && (temp.value == (VOID *) QUERY_DELIMETER_SYMBOL))
        {
         new_list = CLIPS_TRUE;
         (*rcnt)++;
        }
      else if ((tmp = FormChain(func,&temp)) != NULL)
        {
         if (clist == NULL)
           clist = cnxt = cchain = tmp;
         else if (new_list == CLIPS_TRUE)
           {
            new_list = CLIPS_FALSE;
            cnxt->nxt = tmp;
            cnxt = cchain = tmp;
           }
         else
           cchain->chain = tmp;
         while (cchain->chain != NULL)
           cchain = cchain->chain;
        }
      else
        {
         SyntaxErrorMessage("instance-set query class restrictions");
         DeleteQueryClasses(clist);
         SetEvaluationError(CLIPS_TRUE);
         return(NULL);
        }
      classExp = classExp->nextArg;
     }
   return(clist);
  }
  
/*************************************************************
  NAME         : FormChain
  DESCRIPTION  : Builds a list of classes to be used in
                   instance queries - uses parse form.
  INPUTS       : 1) Name of calling function for error msgs
                 2) Data object - must be a symbol or a
                      multifield value containing all symbols
                 The symbols must be names of existing classes
  RETURNS      : The query chain, or NULL on errors
  SIDE EFFECTS : Memory allocated for chain
                 Busy count incremented for all classes
  NOTES        : None
 *************************************************************/
static QUERY_CLASS *FormChain(func,val)
  char *func;
  DATA_OBJECT *val;
  {
   DEFCLASS *cls;
   QUERY_CLASS *head,*bot,*tmp;
   register int i,end;
   char *className;
   struct defmodule *currentModule;
   
   currentModule = ((struct defmodule *) GetCurrentModule());
   if (val->type == DEFCLASS_PTR)
     {
      IncrementDefclassBusyCount((VOID *) val->value);
      head = get_struct(query_class);
      head->cls = (DEFCLASS *) val->value;
      if (DefclassInScope(head->cls,currentModule))
        head->theModule = currentModule;
      else
        head->theModule = head->cls->header.whichModule->theModule;
      head->chain = NULL;
      head->nxt = NULL;
      return(head);
     }
   if (val->type == SYMBOL)
     {
      /* ===============================================
         Allow instance-set query restrictions to have a
         module specifier as part of the class name,
         but search imported defclasses too if a
         module specifier is not given
         =============================================== */
      cls = LookupDefclassByMdlOrScope(DOPToString(val));
      if (cls == NULL)
        {
         ClassExistError(func,DOPToString(val));
         return(NULL);
        }
      IncrementDefclassBusyCount((VOID *) cls);
      head = get_struct(query_class);
      head->cls = cls;
      if (DefclassInScope(head->cls,currentModule))
        head->theModule = currentModule;
      else
        head->theModule = head->cls->header.whichModule->theModule;
      head->chain = NULL;
      head->nxt = NULL;
      return(head);
     }
   if (val->type == MULTIFIELD)
     {
      head = bot = NULL;
      end = GetpDOEnd(val);
      for (i = GetpDOBegin(val) ; i <= end ; i++)
        {
         if (GetMFType(val->value,i) == DEFCLASS_PTR)
           cls = (DEFCLASS *) GetMFValue(val->value,i);
         if (GetMFType(val->value,i) == SYMBOL)
           {
            className = ValueToString(GetMFValue(val->value,i));
            cls = LookupDefclassByMdlOrScope(className);
            if (cls == NULL)
              {
               ClassExistError(func,className);
               DeleteQueryClasses(head);
               return(NULL);
              }
           }
         else
           {
            DeleteQueryClasses(head);
            return(NULL);
           }
         IncrementDefclassBusyCount((VOID *) cls);
         tmp = get_struct(query_class);
         tmp->cls = cls;
         if (DefclassInScope(tmp->cls,currentModule))
           tmp->theModule = currentModule;
         else
           tmp->theModule = tmp->cls->header.whichModule->theModule;
         tmp->chain = NULL;
         tmp->nxt = NULL;
         if (head == NULL)
           head = tmp;
         else
           bot->chain = tmp;
         bot = tmp;
        }
      return(head);
     }
   return(NULL);
  }

/******************************************************
  NAME         : DeleteQueryClasses
  DESCRIPTION  : Deletes a query class-list
  INPUTS       : The query list address
  RETURNS      : Nothing useful
  SIDE EFFECTS : Nodes deallocated
                 Busy count decremented for all classes
  NOTES        : None
 ******************************************************/
static VOID DeleteQueryClasses(qlist)
  QUERY_CLASS *qlist;
  {
   QUERY_CLASS *tmp;
   
   while (qlist != NULL)
     {
      while (qlist->chain != NULL)
        {
         tmp = qlist->chain;
         qlist->chain = qlist->chain->chain;
         DecrementDefclassBusyCount((VOID *) tmp->cls);
         rtn_struct(query_class,tmp);
        }
      tmp = qlist;
      qlist = qlist->nxt;
      DecrementDefclassBusyCount((VOID *) tmp->cls);
      rtn_struct(query_class,tmp);
     }
  }
  
/************************************************************
  NAME         : TestForFirstInChain
  DESCRIPTION  : Processes all classes in a restriction chain
                   until success or done
  INPUTS       : 1) The current chain
                 2) The index of the chain restriction
                     (e.g. the 4th query-variable)
  RETURNS      : CLIPS_TRUE if query succeeds, CLIPS_FALSE otherwise
  SIDE EFFECTS : Sets current restriction class
                 Instance variable values set
  NOTES        : None
 ************************************************************/
static int TestForFirstInChain(qchain,indx)
  QUERY_CLASS *qchain;
  int indx;
  {
   QUERY_CLASS *qptr;
   int id;
   
   AbortQuery = CLIPS_TRUE;
   for (qptr = qchain ; qptr != NULL ; qptr = qptr->chain)
     {
      AbortQuery = CLIPS_FALSE;
      if ((id = GetTraversalID()) == -1)
        return(CLIPS_FALSE);
      if (TestForFirstInstanceInClass(qptr->theModule,id,qptr->cls,qchain,indx))
        {
         ReleaseTraversalID();
         return(CLIPS_TRUE);
        }
      ReleaseTraversalID();
      if ((HaltExecution == CLIPS_TRUE) || (AbortQuery == CLIPS_TRUE))
        return(CLIPS_FALSE);
     }
   return(CLIPS_FALSE);
  }

/*****************************************************************
  NAME         : TestForFirstInstanceInClass
  DESCRIPTION  : Processes all instances in a class and then
                   all subclasses of a class until success or done
  INPUTS       : 1) The module for which classes tested must be
                    in scope
                 2) Visitation traversal id
                 3) The class
                 4) The current class restriction chain
                 5) The index of the current restriction
  RETURNS      : CLIPS_TRUE if query succeeds, CLIPS_FALSE otherwise
  SIDE EFFECTS : Instance variable values set
  NOTES        : None
 *****************************************************************/
static int TestForFirstInstanceInClass(theModule,id,cls,qchain,indx)
  struct defmodule *theModule;
  int id;
  DEFCLASS *cls;
  QUERY_CLASS *qchain;
  int indx;
  {
   register unsigned i;
   INSTANCE_TYPE *ins;
   DATA_OBJECT temp;
   
   if (TestTraversalID(cls->traversalRecord,id))
     return(CLIPS_FALSE);
   SetTraversalID(cls->traversalRecord,id);
   if (DefclassInScope(cls,theModule) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   ins = cls->instanceList;
   while (ins != NULL)
     {
      QueryCore->solns[indx] = ins;
      if (qchain->nxt != NULL)
        {
         ins->busy++;
         if (TestForFirstInChain(qchain->nxt,indx+1) == CLIPS_TRUE)
           {
            ins->busy--;
            break;
           }
         ins->busy--;
         if ((HaltExecution == CLIPS_TRUE) || (AbortQuery == CLIPS_TRUE))
           break;
        }
      else
        {
         ins->busy++;
         CurrentEvaluationDepth++;
         EvaluateExpression(QueryCore->query,&temp);
         CurrentEvaluationDepth--;
         PeriodicCleanup(CLIPS_FALSE,CLIPS_TRUE);
         ins->busy--;
         if (HaltExecution == CLIPS_TRUE)
           break;
         if ((temp.type != SYMBOL) ? CLIPS_TRUE : 
             (temp.value != CLIPSFalseSymbol))
           break;
        }
      ins = ins->nxtClass;
      while ((ins != NULL) ? (ins->garbage == 1) : CLIPS_FALSE)
        ins = ins->nxtClass;
     }
   if (ins != NULL)
     return(((HaltExecution == CLIPS_TRUE) || (AbortQuery == CLIPS_TRUE)) 
             ? CLIPS_FALSE : CLIPS_TRUE);
   for (i = 0 ; i < cls->directSubclasses.classCount ; i++)
     {
      if (TestForFirstInstanceInClass(theModule,id,cls->directSubclasses.classArray[i],
                                      qchain,indx))
        return(CLIPS_TRUE);
      if ((HaltExecution == CLIPS_TRUE) || (AbortQuery == CLIPS_TRUE))
        return(CLIPS_FALSE);
     }
   return(CLIPS_FALSE);
  }
  
/************************************************************
  NAME         : TestEntireChain
  DESCRIPTION  : Processes all classes in a restriction chain
                   until done
  INPUTS       : 1) The current chain
                 2) The index of the chain restriction
                     (i.e. the 4th query-variable)
  RETURNS      : Nothing useful
  SIDE EFFECTS : Sets current restriction class
                 Query instance variables set
                 Solution sets stored in global list
  NOTES        : None
 ************************************************************/
static VOID TestEntireChain(qchain,indx)
  QUERY_CLASS *qchain;
  int indx;
  {
   QUERY_CLASS *qptr;
   int id;
   
   AbortQuery = CLIPS_TRUE;
   for (qptr = qchain ; qptr != NULL ; qptr = qptr->chain)
     {
      AbortQuery = CLIPS_FALSE;
      if ((id = GetTraversalID()) == -1)
        return;
      TestEntireClass(qptr->theModule,id,qptr->cls,qchain,indx);
      ReleaseTraversalID();
      if ((HaltExecution == CLIPS_TRUE) || (AbortQuery == CLIPS_TRUE))
        return;
     }
  }

/*****************************************************************
  NAME         : TestEntireClass
  DESCRIPTION  : Processes all instances in a class and then
                   all subclasses of a class until done
  INPUTS       : 1) The module for which classes tested must be
                    in scope
                 2) Visitation traversal id
                 3) The class
                 4) The current class restriction chain
                 5) The index of the current restriction
  RETURNS      : Nothing useful
  SIDE EFFECTS : Instance variable values set
                 Solution sets stored in global list
  NOTES        : None
 *****************************************************************/
static VOID TestEntireClass(theModule,id,cls,qchain,indx)
  struct defmodule *theModule;
  int id;
  DEFCLASS *cls;
  QUERY_CLASS *qchain;
  int indx;  
  {
   register unsigned i;
   INSTANCE_TYPE *ins;
   DATA_OBJECT temp;
   
   if (TestTraversalID(cls->traversalRecord,id))
     return;
   SetTraversalID(cls->traversalRecord,id);
   if (DefclassInScope(cls,theModule) == CLIPS_FALSE)
     return;
   ins = cls->instanceList;
   while (ins != NULL)
     {
      QueryCore->solns[indx] = ins;
      if (qchain->nxt != NULL)
        {
         ins->busy++;
         TestEntireChain(qchain->nxt,indx+1);
         ins->busy--;
         if ((HaltExecution == CLIPS_TRUE) || (AbortQuery == CLIPS_TRUE))
           break;
        }
      else
        {
         ins->busy++;
         CurrentEvaluationDepth++;
         EvaluateExpression(QueryCore->query,&temp);
         CurrentEvaluationDepth--;
         PeriodicCleanup(CLIPS_FALSE,CLIPS_TRUE);
         ins->busy--;
         if (HaltExecution == CLIPS_TRUE) 
           break;
         if ((temp.type != SYMBOL) ? CLIPS_TRUE : 
             (temp.value != CLIPSFalseSymbol))
           {
            if (QueryCore->action != NULL)
              {
               ins->busy++;
               CurrentEvaluationDepth++;
               ValueDeinstall(QueryCore->result);
               EvaluateExpression(QueryCore->action,QueryCore->result);
               ValueInstall(QueryCore->result);
               CurrentEvaluationDepth--;
               PeriodicCleanup(CLIPS_FALSE,CLIPS_TRUE);
               ins->busy--;
               if (BreakFlag || ReturnFlag)
                 {
                  AbortQuery = CLIPS_TRUE;
                  break;
                 }
               if (HaltExecution == CLIPS_TRUE)
                 break;
              }
            else
              AddSolution();
           }
        }
      ins = ins->nxtClass;
      while ((ins != NULL) ? (ins->garbage == 1) : CLIPS_FALSE)
        ins = ins->nxtClass;
     }
   if (ins != NULL)
     return;
   for (i = 0 ; i < cls->directSubclasses.classCount ; i++)
     {
      TestEntireClass(theModule,id,cls->directSubclasses.classArray[i],qchain,indx);
      if ((HaltExecution == CLIPS_TRUE) || (AbortQuery == CLIPS_TRUE))
        return;
     }
  }
  
/***************************************************************************
  NAME         : AddSolution
  DESCRIPTION  : Adds the current instance set to a global list of
                   solutions
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Global list and count updated
  NOTES        : Solutions are stored as sequential arrays of INSTANCE_TYPE *
 ***************************************************************************/
static VOID AddSolution()
  {
   QUERY_SOLN *new_soln;
   register int i;
   
   new_soln = (QUERY_SOLN *) gm2((int) sizeof(QUERY_SOLN));
   new_soln->soln = (INSTANCE_TYPE **) 
                    gm2((int) (sizeof(INSTANCE_TYPE *) * (QueryCore->soln_size)));
   for (i = 0 ; i < QueryCore->soln_size ; i++)
     new_soln->soln[i] = QueryCore->solns[i];
   new_soln->nxt = NULL;
   if (QueryCore->soln_set == NULL)
     QueryCore->soln_set = new_soln;
   else
     QueryCore->soln_bottom->nxt = new_soln;
   QueryCore->soln_bottom = new_soln;
   QueryCore->soln_cnt++;
  }
  
/***************************************************
  NAME         : PopQuerySoln
  DESCRIPTION  : Deallocates the topmost solution
                   set for an instance-set query
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Solution set deallocated
  NOTES        : Assumes QueryCore->soln_set != 0
 ***************************************************/
static VOID PopQuerySoln()
  {
   QueryCore->soln_bottom = QueryCore->soln_set;
   QueryCore->soln_set = QueryCore->soln_set->nxt;
   rm((VOID *) QueryCore->soln_bottom->soln,
      (int) (sizeof(INSTANCE_TYPE *) * QueryCore->soln_size));
   rm((VOID *) QueryCore->soln_bottom,(int) sizeof(QUERY_SOLN));
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


