   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                  DEFINSTANCES MODULE                */
   /*******************************************************/

/*************************************************************/
/* Purpose: CLIPS Kernel definstances interface commands     */
/*              and routines                                 */
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

#if DEFINSTANCES_CONSTRUCT

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
#include "bload.h"
#include "dfinsbin.h"
#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
#include "dfinscmp.h"
#endif

#include "argacces.h"
#include "classcom.h"
#include "classfun.h"
#include "clipsmem.h"
#include "cstrccom.h"
#include "cstrcpsr.h"
#include "constant.h"
#include "constrct.h"
#include "evaluatn.h"
#include "extnfunc.h"
#include "insfun.h"
#include "inspsr.h"
#include "modulpsr.h"
#include "router.h"
#include "scanner.h"
#include "symbol.h"
#include "utility.h"

#define _DEFINS_SOURCE_
#include "defins.h"

#if (! BLOAD_ONLY) && (! RUN_TIME)
extern struct token ObjectParseToken;
#endif

/* =========================================
   *****************************************
                   CONSTANTS
   =========================================
   ***************************************** */
#define ACTIVE_RLN "active"

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
#if (! BLOAD_ONLY) && (! RUN_TIME)
static int ParseDefinstances(char *);
static SYMBOL_HN *ParseDefinstancesName(char *,int *);
static VOID RemoveDefinstances(VOID *);
static VOID SaveDefinstances(char *);
static BOOLEAN RemoveAllDefinstances(void);
static VOID DefinstancesDeleteError(char *);

#if INSTANCE_PATTERN_MATCHING
static VOID CreateInitialDefinstances(void);
#endif
#endif

#if ! RUN_TIME
static VOID *AllocateModule(void);
static VOID  FreeModule(VOID *);
static BOOLEAN ClearDefinstancesReady(void);
static VOID CheckDefinstancesBusy(struct constructHeader *,VOID *);
#endif

static VOID ResetDefinstances(void);
static VOID ResetDefinstancesAction(struct constructHeader *,VOID *);

#else

#if (! BLOAD_ONLY) && (! RUN_TIME)
static int ParseDefinstances();
static SYMBOL_HN *ParseDefinstancesName();
static VOID RemoveDefinstances();
static VOID SaveDefinstances();
static BOOLEAN RemoveAllDefinstances();
static VOID DefinstancesDeleteError();

#if INSTANCE_PATTERN_MATCHING
static VOID CreateInitialDefinstances();
#endif
#endif

#if ! RUN_TIME
static VOID *AllocateModule();
static VOID  FreeModule();
static BOOLEAN ClearDefinstancesReady();
static VOID CheckDefinstancesBusy();
#endif

static VOID ResetDefinstances();
static VOID ResetDefinstancesAction();

#endif

/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
globle int DefinstancesModuleIndex;

/* =========================================
   *****************************************
      INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
static struct construct *DefinstancesConstruct;

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/***************************************************
  NAME         : SetupDefinstances
  DESCRIPTION  : Adds the definstance support routines
                   to the CLIPS Kernel
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Appropriate function lists modified
  NOTES        : None
 ***************************************************/
globle VOID SetupDefinstances()
  {
   DefinstancesModuleIndex = 
                RegisterModuleItem("definstances",
#if (! RUN_TIME)
                                    AllocateModule,FreeModule,
#else
                                    NULL,NULL,
#endif
#if BLOAD_AND_BSAVE || BLOAD || BLOAD_ONLY
                                    BloadDefinstancesModuleRef,
#else
                                    NULL,
#endif
#if CONSTRUCT_COMPILER && (! RUN_TIME)
                                    DefinstancesCModuleReference,
#else
                                    NULL,
#endif
                                    FindDefinstances);

   DefinstancesConstruct = 
      AddConstruct("definstances","definstances",
#if (! BLOAD_ONLY) && (! RUN_TIME)
                   ParseDefinstances,
#else
                   NULL,
#endif
                   FindDefinstances,
                   GetConstructNamePointer,GetConstructPPForm,
                   GetConstructModuleItem,GetNextDefinstances,SetNextConstruct,
                   IsDefinstancesDeletable,Undefinstances,
#if (! BLOAD_ONLY) && (! RUN_TIME)
                   RemoveDefinstances
#else
                   NULL
#endif
                   );

#if ! RUN_TIME
   AddClearReadyFunction("definstances",ClearDefinstancesReady,0);

#if ! BLOAD_ONLY
   DefineFunction2("undefinstances",'v',PTIF UndefinstancesCommand,"UndefinstancesCommand","11w");
   AddSaveFunction("definstances",SaveDefinstances,0);

#if INSTANCE_PATTERN_MATCHING
   AddClearFunction("definstances",CreateInitialDefinstances,-1000);
#endif

#endif

#if DEBUGGING_FUNCTIONS
   DefineFunction2("ppdefinstances",'v',PTIF PPDefinstancesCommand ,"PPDefinstancesCommand","11w");
   DefineFunction2("list-definstances",'v',PTIF ListDefinstancesCommand,"ListDefinstancesCommand","01");
#endif

   DefineFunction2("get-definstances-list",'m',PTIF GetDefinstancesListFunction,
                   "GetDefinstancesListFunction","01");
   DefineFunction2("definstances-module",'w',PTIF GetDefinstancesModuleCommand,
                   "GetDefinstancesModuleCommand","11w");

#endif
   AddResetFunction("definstances",(VOID (*)(VOID_ARG)) ResetDefinstances,0);
   
#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
   SetupDefinstancesBload();
#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
   SetupDefinstancesCompiler();
#endif
  }
  
/***********************************************************
  NAME         : GetNextDefinstances
  DESCRIPTION  : Finds first or next definstances
  INPUTS       : The address of the current definstances
  RETURNS      : The address of the next definstances
                   (NULL if none)
  SIDE EFFECTS : None
  NOTES        : If ptr == NULL, the first definstances
                    is returned.
 ***********************************************************/
globle VOID *GetNextDefinstances(ptr)
  VOID *ptr;
  {
   return((VOID *) GetNextConstructItem((struct constructHeader *) ptr,
                                        DefinstancesModuleIndex));
  }
  
/***************************************************
  NAME         : FindDefinstances
  DESCRIPTION  : Looks up a definstance construct
                   by name-string
  INPUTS       : The symbolic name
  RETURNS      : The definstance address, or NULL
                    if not found
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle VOID *FindDefinstances(name)
  char *name;
  {
   return(FindNamedConstruct(name,DefinstancesConstruct));
  }
  
/***************************************************
  NAME         : IsDefinstancesDeletable
  DESCRIPTION  : Determines if a definstances
                   can be deleted
  INPUTS       : Address of the definstances
  RETURNS      : CLIPS_TRUE if deletable, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle int IsDefinstancesDeletable(ptr)
  VOID *ptr;
  {
#if BLOAD_ONLY || RUN_TIME
   return(CLIPS_FALSE);
#else
#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded())
     return(CLIPS_FALSE);
#endif
   return((((DEFINSTANCES *) ptr)->busy == 0) ? CLIPS_TRUE : CLIPS_FALSE);
#endif
  }

/***********************************************************
  NAME         : UndefinstancesCommand
  DESCRIPTION  : Removes a definstance
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Definstance deallocated
  NOTES        : CLIPS Syntax : (undefinstances <name> | *)
 ***********************************************************/
globle VOID UndefinstancesCommand()
  {
   UndefconstructCommand("undefinstances",DefinstancesConstruct);
  }

/*****************************************************************
  NAME         : GetDefinstancesModuleCommand
  DESCRIPTION  : Determines to which module a definstances belongs
  INPUTS       : None
  RETURNS      : The symbolic name of the module
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax: (definstances-module <defins-name>)
 *****************************************************************/
globle SYMBOL_HN *GetDefinstancesModuleCommand()
  {
   return(GetConstructModuleCommand("definstances-module",DefinstancesConstruct));
  }
  
/***********************************************************
  NAME         : Undefinstances
  DESCRIPTION  : Removes a definstance
  INPUTS       : Address of definstances to remove
  RETURNS      : CLIPS_TRUE if successful,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : Definstance deallocated
  NOTES        : None
 ***********************************************************/
globle BOOLEAN Undefinstances(vptr)
  VOID *vptr;
  {
   DEFINSTANCES *dptr;
   
   dptr = (DEFINSTANCES *) vptr;
#if RUN_TIME || BLOAD_ONLY
   return(CLIPS_FALSE);
#else

#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded())
     return(CLIPS_FALSE);
#endif
   if (dptr == NULL)
     return(RemoveAllDefinstances());
   if (IsDefinstancesDeletable(vptr) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   RemoveConstructFromModule((struct constructHeader *) vptr);
   RemoveDefinstances((VOID *) dptr);
   return(CLIPS_TRUE);
#endif
  }
  
#if DEBUGGING_FUNCTIONS

/***************************************************************
  NAME         : PPDefinstancesCommand
  DESCRIPTION  : Prints out the pretty-print form of a definstance
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax : (ppdefinstances <name>)
 ***************************************************************/
globle VOID PPDefinstancesCommand()
  {
   PPConstructCommand("ppdefinstances",DefinstancesConstruct);
  }

/***************************************************
  NAME         : ListDefinstancesCommand
  DESCRIPTION  : Displays all definstances names
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Definstances name sprinted
  NOTES        : CLIPS Interface
 ***************************************************/
globle VOID ListDefinstancesCommand()
  {
   ListConstructCommand("list-definstances",DefinstancesConstruct);
  }

/***************************************************
  NAME         : ListDefinstances
  DESCRIPTION  : Displays all definstances names
  INPUTS       : 1) The logical name of the output
                 2) The module
  RETURNS      : Nothing useful
  SIDE EFFECTS : Definstances names printed
  NOTES        : C Interface
 ***************************************************/
globle VOID ListDefinstances(logicalName,theModule)
  char *logicalName;
  struct defmodule *theModule;
  {
   ListConstruct(DefinstancesConstruct,logicalName,theModule);
  }

#endif
    
/****************************************************************
  NAME         : GetDefinstancesListFunction
  DESCRIPTION  : Groups all definstances names into
                 a multifield list
  INPUTS       : A data object buffer to hold
                 the multifield result
  RETURNS      : Nothing useful
  SIDE EFFECTS : Multifield allocated and filled
  NOTES        : CLIPS Syntax: (get-definstances-list [<module>])
 ****************************************************************/
globle VOID GetDefinstancesListFunction(returnValue)
  DATA_OBJECT*returnValue;
  {
   GetConstructListFunction("get-definstances-list",returnValue,DefinstancesConstruct);
  }

/***************************************************************
  NAME         : GetDefinstancesList
  DESCRIPTION  : Groups all definstances names into
                 a multifield list
  INPUTS       : 1) A data object buffer to hold
                    the multifield result
                 2) The module from which to obtain definstances
  RETURNS      : Nothing useful
  SIDE EFFECTS : Multifield allocated and filled
  NOTES        : External C access
 ***************************************************************/
globle VOID GetDefinstancesList(returnValue,theModule)
  DATA_OBJECT *returnValue;
  struct defmodule *theModule;
  {
   GetConstructList(returnValue,DefinstancesConstruct,theModule);
  }
  
/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
#if (! BLOAD_ONLY) && (! RUN_TIME)
  
/*********************************************************************
  NAME         : ParseDefinstances
  DESCRIPTION  : Parses and allocates a definstances construct
  INPUTS       : The logical name of the input source
  RETURNS      : CLIPS_FALSE if no errors, CLIPS_TRUE otherwise
  SIDE EFFECTS : Definstances parsed and created
  NOTES        : CLIPS Syntax :
  
                 (definstances  <name> [active] [<comment>]
                    <instance-definition>+)
                    
                 <instance-definition> ::= 
                    (<instance-name> of <class-name> <slot-override>*)
                    
                 <slot-override> ::= (<slot-name> <value-expression>*)
 *********************************************************************/
static int ParseDefinstances(readSource)
  char *readSource;
  {
   SYMBOL_HN *dname;
   VOID *mkinsfcall;
   EXPRESSION *mkinstance,*mkbot = NULL;
   DEFINSTANCES *dobj;
   int active;
   
   SetPPBufferStatus(ON);
   FlushPPBuffer();          
   SetIndentDepth(3);    
   SavePPBuffer("(definstances ");

#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded())
     {
      CannotLoadWithBloadMessage("definstances");
      return(CLIPS_TRUE);
     }
#endif
   dname = ParseDefinstancesName(readSource,&active);
   if (dname == NULL)
     return(CLIPS_TRUE);

   dobj = get_struct(definstances);
   InitializeConstructHeader("definstances",(struct constructHeader *) dobj,dname);
   dobj->busy = 0;
   dobj->mkinstance = NULL;
#if INSTANCE_PATTERN_MATCHING
   mkinsfcall = (VOID *) FindFunction(active ? "active-make-instance" : "make-instance");
#else
   mkinsfcall = (VOID *) FindFunction("make-instance");
#endif
   while (GetType(ObjectParseToken) == LPAREN)
     {
      mkinstance = GenConstant(UNKNOWN,mkinsfcall);
      mkinstance = ParseInitializeInstance(mkinstance,readSource);
      if (mkinstance == NULL)
        {
         ReturnExpression(dobj->mkinstance);
         rtn_struct(definstances,dobj);
         return(CLIPS_TRUE);
        }
      if (ExpressionContainsVariables(mkinstance,CLIPS_FALSE) == CLIPS_TRUE)
        {
         LocalVariableErrorMessage("definstances");
         ReturnExpression(mkinstance);
         ReturnExpression(dobj->mkinstance);
         rtn_struct(definstances,dobj);
         return(CLIPS_TRUE);
        }
      if (mkbot == NULL)
        dobj->mkinstance = mkinstance;
      else
        GetNextArgument(mkbot) = mkinstance;
      mkbot = mkinstance;
      GetToken(readSource,&ObjectParseToken);
      PPBackup();
      PPCRAndIndent();
      SavePPBuffer(ObjectParseToken.print_rep);
     }
   if (GetType(ObjectParseToken) != RPAREN)
     {
      ReturnExpression(dobj->mkinstance);
      rtn_struct(definstances,dobj);
      SyntaxErrorMessage("definstances");
      return(CLIPS_TRUE);
     }
   else
     {
#if DEBUGGING_FUNCTIONS
      if (GetConserveMemory() == CLIPS_FALSE)
        {
         if (dobj->mkinstance != NULL)
           PPBackup();
         PPBackup();
         SavePPBuffer(")\n");
         SetDefinstancesPPForm((VOID *) dobj,CopyPPBuffer());
        }
#endif
      mkinstance = dobj->mkinstance;
      dobj->mkinstance = PackExpression(mkinstance);
      ReturnExpression(mkinstance);
      IncrementSymbolCount(GetDefinstancesNamePointer((VOID *) dobj));
      ExpressionInstall(dobj->mkinstance);
     }
   AddConstructToModule((struct constructHeader *) dobj);
   return(CLIPS_FALSE);
  }
  
/*************************************************************
  NAME         : ParseDefinstancesName
  DESCRIPTION  : Parses definstance name and optional comment
                 and optional "active" keyword
  INPUTS       : 1) The logical name of the input source
                 2) Buffer to hold flag indicating if
                    definstances should cause pattern-matching
                    to occur during slot-overrides
  RETURNS      : Address of CLIPS name symbol, or
                   NULL if there was an error
  SIDE EFFECTS : Token after name or comment is scanned
  NOTES        : Assumes "(definstances" has already
                   been scanned.
 *************************************************************/
static SYMBOL_HN *ParseDefinstancesName(readSource,active)
  char *readSource;
  int *active;
  {
   SYMBOL_HN *dname;

   *active = CLIPS_FALSE;
   dname = GetConstructNameAndComment(readSource,&ObjectParseToken,"definstances",
                                      FindDefinstances,Undefinstances,"@",
                                      CLIPS_TRUE,CLIPS_FALSE,CLIPS_TRUE);
   if (dname == NULL)
     return(NULL);

#if INSTANCE_PATTERN_MATCHING
   if ((GetType(ObjectParseToken) != SYMBOL) ? CLIPS_FALSE :
       (strcmp(ValueToString(GetValue(ObjectParseToken)),ACTIVE_RLN) == 0))
     {
      PPBackup();
      PPBackup();
      SavePPBuffer(" ");
      SavePPBuffer(ObjectParseToken.print_rep);
      PPCRAndIndent();
      GetToken(readSource,&ObjectParseToken);
      *active = CLIPS_TRUE;
     }
#endif
   if (GetType(ObjectParseToken) == STRING)
     {
      PPBackup();
      PPBackup();
      SavePPBuffer(" ");
      SavePPBuffer(ObjectParseToken.print_rep);
      PPCRAndIndent();          
      GetToken(readSource,&ObjectParseToken);
     }
   return(dname);
  }
  
/**************************************************************
  NAME         : RemoveDefinstances
  DESCRIPTION  : Deallocates and removes a definstance construct
  INPUTS       : The definstance address
  RETURNS      : Nothing useful
  SIDE EFFECTS : Existing definstance construct deleted
  NOTES        : Assumes busy count of definstance is 0
 **************************************************************/
static VOID RemoveDefinstances(vdptr)
  VOID *vdptr;
  {
   DEFINSTANCES *dptr = (DEFINSTANCES *) vdptr;
  
   DecrementSymbolCount(GetDefinstancesNamePointer((VOID *) dptr));
   ExpressionDeinstall(dptr->mkinstance);
   ReturnPackedExpression(dptr->mkinstance);
   SetDefinstancesPPForm((VOID *) dptr,NULL);
   rtn_struct(definstances,dptr);
  }

/***************************************************
  NAME         : SaveDefinstances
  DESCRIPTION  : Prints pretty print form of
                   definstances to specified output
  INPUTS       : The logical name of the output
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
static VOID SaveDefinstances(logName)
  char *logName;
  {
   SaveConstruct(logName,DefinstancesConstruct);
  }
  
/***************************************************
  NAME         : RemoveAllDefinstances
  DESCRIPTION  : Removes all definstances constructs
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if successful,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : All definstances deallocated
  NOTES        : None
 ***************************************************/
static BOOLEAN RemoveAllDefinstances()
  {
   DEFINSTANCES *dptr,*dhead;
   int success = CLIPS_TRUE;
   
#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded())
     return(CLIPS_FALSE);
#endif
  dhead = (DEFINSTANCES *) GetNextDefinstances(NULL);
  while (dhead != NULL)
    {
     dptr = dhead;
     dhead = (DEFINSTANCES *) GetNextDefinstances((VOID *) dhead);
     if (IsDefinstancesDeletable((VOID *) dptr))
       {
        RemoveConstructFromModule((struct constructHeader *) dptr);
        RemoveDefinstances((VOID *) dptr);
       }
     else
       {
        DefinstancesDeleteError(GetDefinstancesName((VOID *) dptr));
        success = CLIPS_FALSE;
       }
    }
   return(success);
  }
  
/***************************************************
  NAME         : DefinstancesDeleteError
  DESCRIPTION  : Prints an error message for
                 unsuccessful definstances
                 deletion attempts
  INPUTS       : The name of the definstances
  RETURNS      : Nothing useful
  SIDE EFFECTS : Error message printed
  NOTES        : None
 ***************************************************/
static VOID DefinstancesDeleteError(dname)
  char *dname;
  {
   CantDeleteItemErrorMessage("definstances",dname);
  }

#if INSTANCE_PATTERN_MATCHING

/********************************************************
  NAME         : CreateInitialDefinstances
  DESCRIPTION  : Makes the initial-object definstances
                 structure for creating an initial-object
                 which will match default object patterns
                 in defrules
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : initial-object definstances created
  NOTES        : None
 ********************************************************/
static VOID CreateInitialDefinstances()
  {
   EXPRESSION *tmp;
   DEFINSTANCES *theDefinstances;
   
   theDefinstances = get_struct(definstances);
   InitializeConstructHeader("definstances",(struct constructHeader *) theDefinstances,
                             INITIAL_OBJECT_SYMBOL);
   theDefinstances->busy = 0;
   tmp = GenConstant(FCALL,(VOID *) FindFunction("make-instance"));
   tmp->argList = GenConstant(INSTANCE_NAME,(VOID *) INITIAL_OBJECT_SYMBOL);
   tmp->argList->nextArg = 
       GenConstant(DEFCLASS_PTR,(VOID *) LookupDefclassInScope(INITIAL_OBJECT_CLASS_NAME));
   theDefinstances->mkinstance = PackExpression(tmp);
   ReturnExpression(tmp);
   IncrementSymbolCount(GetDefinstancesNamePointer((VOID *) theDefinstances));
   ExpressionInstall(theDefinstances->mkinstance);
   AddConstructToModule((struct constructHeader *) theDefinstances);
  }
  
#endif

#endif

#if ! RUN_TIME

/*****************************************************
  NAME         : AllocateModule
  DESCRIPTION  : Creates and initializes a
                 list of definstances for a new module
  INPUTS       : None
  RETURNS      : The new definstances module
  SIDE EFFECTS : Definstances module created
  NOTES        : None
 *****************************************************/
static VOID *AllocateModule()
  {
   return((VOID *) get_struct(definstancesModule));
  } 

/***************************************************
  NAME         : FreeModule
  DESCRIPTION  : Removes a definstances module and
                 all associated definstances
  INPUTS       : The definstances module
  RETURNS      : Nothing useful
  SIDE EFFECTS : Module and definstances deleted
  NOTES        : None
 ***************************************************/
static VOID FreeModule(theItem)
  VOID *theItem;
  {
#if (! BLOAD_ONLY)
   FreeConstructHeaderModule(theItem,DefinstancesConstruct);
#endif
   rtn_struct(definstancesModule,theItem);
  } 
  
/***************************************************
  NAME         : ClearDefinstancesReady
  DESCRIPTION  : Determines if it is safe to
                 remove all definstances
                 Assumes *all* constructs will be
                 deleted
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if all definstances can
                 be deleted, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : Used by (clear) and (bload)
 ***************************************************/
static BOOLEAN ClearDefinstancesReady()
  {
   int flagBuffer = CLIPS_TRUE;
   
   DoForAllConstructs(CheckDefinstancesBusy,DefinstancesModuleIndex,
                      CLIPS_FALSE,(VOID *) &flagBuffer);
   return(flagBuffer);
  }
  
/***************************************************
  NAME         : CheckDefinstancesBusy
  DESCRIPTION  : Determines if a definstances is
                 in use or not
  INPUTS       : 1) The definstances
                 2) A buffer to set to 0 if the
                    the definstances is busy
  RETURNS      : Nothing useful
  SIDE EFFECTS : Buffer set to 0 if definstances
                 busy
  NOTES        : The flag buffer is not modified
                 if definstances is not busy
                 (assumed to be initialized to 1)
 ***************************************************/
static VOID CheckDefinstancesBusy(theDefinstances,userBuffer)
  struct constructHeader *theDefinstances;
  VOID *userBuffer;
  {
   if (((DEFINSTANCES *) theDefinstances)->busy > 0)
     * (int *) userBuffer = CLIPS_FALSE;
  }

#endif

/***************************************************
  NAME         : ResetDefinstances
  DESCRIPTION  : Calls EvaluateExpression for each of
                   the make-instance calls in all
                   of the definstances constructs
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : All instances in the definstances
                   are evaluated (and created if
                   there are no errors)
                 Any previously existing instances
                 are deleted first.
  NOTES        : None
 ***************************************************/
static VOID ResetDefinstances()
  {
   DestroyAllInstances();
   DoForAllConstructs(ResetDefinstancesAction,DefinstancesModuleIndex,CLIPS_TRUE,NULL);
  }

/***************************************************
  NAME         : ResetDefinstancesAction
  DESCRIPTION  : Performs all the make-instance
                 calls in a definstances
  INPUTS       : 1) The definstances
                 2) User data buffer (ignored)
  RETURNS      : Nothing useful
  SIDE EFFECTS : Instances created
  NOTES        : None
 ***************************************************/
#if IBM_TBC
#pragma argsused
#endif
static VOID ResetDefinstancesAction(vDefinstances,userBuffer)
  struct constructHeader *vDefinstances;
  VOID *userBuffer;
  {
#if MAC_MPW
#pragma unused(userBuffer)
#endif
   DEFINSTANCES *theDefinstances = (DEFINSTANCES *) vDefinstances;
   EXPRESSION *exp;
   DATA_OBJECT temp;
   
   SaveCurrentModule();
   SetCurrentModule((VOID *) vDefinstances->whichModule->theModule);
   theDefinstances->busy++;
   for (exp = theDefinstances->mkinstance ; 
        exp != NULL ;
        exp = GetNextArgument(exp))
     {
      EvaluateExpression(exp,&temp);
      if (HaltExecution ||
          ((GetType(temp) == SYMBOL) &&
           (GetValue(temp) == CLIPSFalseSymbol)))
        {
         RestoreCurrentModule();
         theDefinstances->busy--;
         return;
        }
     }
   theDefinstances->busy--;
   RestoreCurrentModule();
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


