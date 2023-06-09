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
/* Purpose: CLIPS Generic Functions Interface Routines       */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
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

#if DEFGENERIC_CONSTRUCT

#if ANSI_COMPILER
#include <string.h>
#endif

#if DEFRULE_CONSTRUCT
#include "network.h"
#endif

#if BLOAD || BLOAD_AND_BSAVE
#include "bload.h"
#endif

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
#include "genrcbin.h"
#endif

#if CONSTRUCT_COMPILER
#include "genrccmp.h"
#endif

#if (! BLOAD_ONLY) && (! RUN_TIME)
#include "constrct.h"
#include "genrcpsr.h"
#endif

#if OBJECT_SYSTEM
#include "classcom.h"
#include "inscom.h"
#endif

#if DEBUGGING_FUNCTIONS
#include "watch.h"
#endif

#include "argacces.h"
#include "clipsmem.h"
#include "cstrcpsr.h"
#include "extnfunc.h"
#include "genrcexe.h"
#include "modulpsr.h"
#include "multifld.h"
#include "router.h"

#define _GENRCCOM_SOURCE_
#include "genrccom.h"

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
   
/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER

static VOID PrintGenericCall(char *,VOID *);
static BOOLEAN EvaluateGenericCall(VOID *,DATA_OBJECT *);
static VOID DecrementGenericBusyCount(VOID *);
static VOID IncrementGenericBusyCount(VOID *);

#if (! BLOAD_ONLY) && (! RUN_TIME)

static VOID SaveDefgenerics(char *);
static VOID SaveDefmethods(char *);
static VOID SaveDefmethodsForDefgeneric(struct constructHeader *,VOID *);
static VOID RemoveDefgenericMethod(DEFGENERIC *,int);

#endif

#if DEBUGGING_FUNCTIONS
static long ListMethodsForGeneric(char *,DEFGENERIC *);
static BOOLEAN DefgenericWatchAccess(int,int,EXPRESSION *);
static BOOLEAN DefgenericWatchPrint(char *,int,EXPRESSION *);
static BOOLEAN DefmethodWatchAccess(int,int,EXPRESSION *);
static BOOLEAN DefmethodWatchPrint(char *,int,EXPRESSION *);
static BOOLEAN DefmethodWatchSupport(char *,char *,int,
                                     VOID (*)(char *,VOID *,unsigned),
                                     VOID (*)(int,VOID *,unsigned),EXPRESSION *);
static VOID PrintMethodWatchFlag(char *,VOID *,unsigned);
#endif

#else

static VOID PrintGenericCall();
static BOOLEAN EvaluateGenericCall();
static VOID DecrementGenericBusyCount();
static VOID IncrementGenericBusyCount();

#if (! BLOAD_ONLY) && (! RUN_TIME)

static VOID SaveDefgenerics();
static VOID SaveDefmethods();
static VOID SaveDefmethodsForDefgeneric();
static VOID RemoveDefgenericMethod();

#endif

#if DEBUGGING_FUNCTIONS
static long ListMethodsForGeneric();
static BOOLEAN DefgenericWatchAccess();
static BOOLEAN DefgenericWatchPrint();
static BOOLEAN DefmethodWatchAccess();
static BOOLEAN DefmethodWatchPrint();
static BOOLEAN DefmethodWatchSupport();
static VOID PrintMethodWatchFlag();
#endif

#endif
      
/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
globle struct construct *DefgenericConstruct;
globle int DefgenericModuleIndex;

/* =========================================
   *****************************************
      INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */

static ENTITY_RECORD GenericEntityRecord =
                     { GCALL,0,0,1,
                       PrintGenericCall,PrintGenericCall,
                       NULL,EvaluateGenericCall,NULL,
                       DecrementGenericBusyCount,IncrementGenericBusyCount,
                       NULL,NULL,NULL,NULL };

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/***********************************************************
  NAME         : SetupGenericFunctions
  DESCRIPTION  : Initializes all generic function
                   data structures, constructs and functions
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Generic function CLIPS functions set up
  NOTES        : None
 ***********************************************************/
globle VOID SetupGenericFunctions()
  {
   InstallPrimitive(&GenericEntityRecord,GCALL);

   DefgenericModuleIndex = 
                RegisterModuleItem("defgeneric",
#if (! RUN_TIME)
                                    AllocateDefgenericModule,FreeDefgenericModule,
#else
                                    NULL,NULL,
#endif
#if BLOAD_AND_BSAVE || BLOAD || BLOAD_ONLY
                                    BloadDefgenericModuleReference,
#else
                                    NULL,
#endif
#if CONSTRUCT_COMPILER && (! RUN_TIME)
                                    DefgenericCModuleReference,
#else
                                    NULL,
#endif
                                    FindDefgeneric);

   DefgenericConstruct =  AddConstruct("defgeneric","defgenerics",
#if (! BLOAD_ONLY) && (! RUN_TIME)
                                       ParseDefgeneric,
#else
                                       NULL,
#endif
                                       FindDefgeneric,
                                       GetConstructNamePointer,GetConstructPPForm,
                                       GetConstructModuleItem,GetNextDefgeneric,
                                       SetNextConstruct,IsDefgenericDeletable,
                                       Undefgeneric,
#if (! BLOAD_ONLY) && (! RUN_TIME)
                                       RemoveDefgeneric
#else
                                       NULL
#endif
                                       );

#if ! RUN_TIME
   AddClearReadyFunction("defgeneric",ClearDefgenericsReady,0);

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE
   SetupGenericsBload();
#endif

#if CONSTRUCT_COMPILER
   SetupGenericsCompiler();
#endif

#if ! BLOAD_ONLY
#if DEFMODULE_CONSTRUCT
   AddPortConstructItem("defgeneric",SYMBOL);
#endif
   AddConstruct("defmethod","defmethods",ParseDefmethod,
                NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL);
   
  /* ================================================================
     Make sure defmethods are cleared last, for other constructs may
       be using them and need to be cleared first
       
     Need to be cleared in two stages so that mutually dependent
       constructs (like classes) can be cleared
     ================================================================ */
   AddSaveFunction("defgeneric",SaveDefgenerics,1000);
   AddSaveFunction("defmethod",SaveDefmethods,-1000);
   DefineFunction2("undefgeneric",'v',PTIF UndefgenericCommand,"UndefgenericCommand","11w");
   DefineFunction2("undefmethod",'v',PTIF UndefmethodCommand,"UndefmethodCommand","22*wg");
#endif

#if IMPERATIVE_METHODS
   DefineFunction2("call-next-method",'u',PTIF CallNextMethod,"CallNextMethod","00");
   FuncSeqOvlFlags("call-next-method",CLIPS_TRUE,CLIPS_FALSE);
   DefineFunction2("call-specific-method",'u',PTIF CallSpecificMethod,
                   "CallSpecificMethod","2**wi");
   FuncSeqOvlFlags("call-specific-method",CLIPS_TRUE,CLIPS_FALSE);
   DefineFunction2("override-next-method",'u',PTIF OverrideNextMethod,
                   "OverrideNextMethod",NULL);
   FuncSeqOvlFlags("override-next-method",CLIPS_TRUE,CLIPS_FALSE);
   DefineFunction2("next-methodp",'b',PTIF NextMethodP,"NextMethodP","00");
   FuncSeqOvlFlags("next-methodp",CLIPS_TRUE,CLIPS_FALSE);
#endif
   DefineFunction2("(gnrc-current-arg)",'u',PTIF GetGenericCurrentArgument,
                   "GetGenericCurrentArgument",NULL);

#if DEBUGGING_FUNCTIONS
   DefineFunction2("ppdefgeneric",'v',PTIF PPDefgenericCommand,"PPDefgenericCommand","11w");
   DefineFunction2("list-defgenerics",'v',PTIF ListDefgenericsCommand,"ListDefgenericsCommand","01");
   DefineFunction2("ppdefmethod",'v',PTIF PPDefmethodCommand,"PPDefmethodCommand","22*wi");
   DefineFunction2("list-defmethods",'v',PTIF ListDefmethodsCommand,"ListDefmethodsCommand","01w");
   DefineFunction2("preview-generic",'v',PTIF PreviewGeneric,"PreviewGeneric","1**w");
#endif
   
   DefineFunction2("get-defgeneric-list",'m',PTIF GetDefgenericListFunction,
                   "GetDefgenericListFunction","01");
   DefineFunction2("get-defmethod-list",'m',PTIF GetDefmethodListCommand,
                   "GetDefmethodListCommand","01w");
   DefineFunction2("get-method-restrictions",'m',PTIF GetMethodRestrictionsCommand,
                   "GetMethodRestrictionsCommand","22iw");
   DefineFunction2("defgeneric-module",'w',PTIF GetDefgenericModuleCommand,
                   "GetDefgenericModuleCommand","11w");

#if OBJECT_SYSTEM
   DefineFunction2("type",'u',PTIF ClassCommand,"ClassCommand","11");
#else
   DefineFunction2("type",'u',PTIF TypeCommand,"TypeCommand","11");
#endif

#endif

#if DEBUGGING_FUNCTIONS
   AddWatchItem("generic-functions",0,&WatchGenerics,34,
                DefgenericWatchAccess,DefgenericWatchPrint);
   AddWatchItem("methods",0,&WatchMethods,33,
                DefmethodWatchAccess,DefmethodWatchPrint);
#endif
  }
  
/***************************************************
  NAME         : FindDefgeneric
  DESCRIPTION  : Searches for a generic
  INPUTS       : The name of the generic
                 (possibly including a module name)
  RETURNS      : Pointer to the generic if
                 found, otherwise NULL
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle VOID *FindDefgeneric(genericModuleAndName)
  char *genericModuleAndName;
  {
   return(FindNamedConstruct(genericModuleAndName,DefgenericConstruct));
  }

/***************************************************
  NAME         : LookupDefgenericByMdlOrScope
  DESCRIPTION  : Finds a defgeneric anywhere (if
                 module is specified) or in current
                 or imported modules
  INPUTS       : The defgeneric name
  RETURNS      : The defgeneric (NULL if not found)
  SIDE EFFECTS : Error message printed on
                  ambiguous references
  NOTES        : None
 ***************************************************/
globle DEFGENERIC *LookupDefgenericByMdlOrScope(defgenericName)
  char *defgenericName;
  {
   return((DEFGENERIC *) LookupConstruct(DefgenericConstruct,defgenericName,CLIPS_TRUE));
  }
  
/***************************************************
  NAME         : LookupDefgenericInScope
  DESCRIPTION  : Finds a defgeneric in current or
                   imported modules (module
                   specifier is not allowed)
  INPUTS       : The defgeneric name
  RETURNS      : The defgeneric (NULL if not found)
  SIDE EFFECTS : Error message printed on
                  ambiguous references
  NOTES        : None
 ***************************************************/
globle DEFGENERIC *LookupDefgenericInScope(defgenericName)
  char *defgenericName;
  {
   return((DEFGENERIC *) LookupConstruct(DefgenericConstruct,defgenericName,CLIPS_FALSE));
  }
      
/***********************************************************
  NAME         : GetNextDefgeneric
  DESCRIPTION  : Finds first or next generic function
  INPUTS       : The address of the current generic function
  RETURNS      : The address of the next generic function
                   (NULL if none)
  SIDE EFFECTS : None
  NOTES        : If ptr == NULL, the first generic function
                    is returned.
 ***********************************************************/
globle VOID *GetNextDefgeneric(ptr)
  VOID *ptr;
  {
   return((VOID *) GetNextConstructItem((struct constructHeader *) ptr,DefgenericModuleIndex));
  }
  
/***********************************************************
  NAME         : GetNextDefmethod
  DESCRIPTION  : Find the next method for a generic function
  INPUTS       : 1) The generic function address
                 2) The index of the current method
  RETURNS      : The index of the next method
                    (0 if none)
  SIDE EFFECTS : None
  NOTES        : If index == 0, the index of the first
                   method is returned
 ***********************************************************/
globle unsigned GetNextDefmethod(ptr,index)
  VOID *ptr;
  unsigned index;
  {
   DEFGENERIC *gfunc;
   int mi;
   
   gfunc = (DEFGENERIC *) ptr;
   if (index == 0)
     {
      if (gfunc->methods != NULL)
        return(gfunc->methods[0].index);  
      return(0);
     }
   mi = FindMethodByIndex(gfunc,index);
   if ((mi+1) == gfunc->mcnt)
     return(0);
   return(gfunc->methods[mi+1].index);
  }
  
/***************************************************
  NAME         : IsDefgenericDeletable
  DESCRIPTION  : Determines if a generic function
                   can be deleted
  INPUTS       : Address of the generic function
  RETURNS      : CLIPS_TRUE if deletable, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle int IsDefgenericDeletable(ptr)
  VOID *ptr;
  {
#if BLOAD_ONLY || RUN_TIME
   return(CLIPS_FALSE);
#else
#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded())
     return(CLIPS_FALSE);
#endif
   return ((((DEFGENERIC *) ptr)->busy == 0) ? CLIPS_TRUE : CLIPS_FALSE);
#endif
  }

/***************************************************
  NAME         : IsDefmethodDeletable
  DESCRIPTION  : Determines if a generic function
                   method can be deleted
  INPUTS       : 1) Address of the generic function
                 2) Index of the method
  RETURNS      : CLIPS_TRUE if deletable, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle int IsDefmethodDeletable(ptr,index)
  VOID *ptr;
  unsigned index;
  {
#if BLOAD_ONLY || RUN_TIME
   return(CLIPS_FALSE);
#else
#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded())
     return(CLIPS_FALSE);
#endif
   if (((DEFGENERIC *) ptr)->methods[FindMethodByIndex((DEFGENERIC *) ptr,index)].system)
     return(CLIPS_FALSE);
   return((MethodsExecuting((DEFGENERIC *) ptr) == CLIPS_FALSE) ? CLIPS_TRUE : CLIPS_FALSE);
#endif
  }

/**********************************************************
  NAME         : UndefgenericCommand
  DESCRIPTION  : Deletes all methods for a generic function
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : methods deallocated
  NOTES        : CLIPS Syntax: (undefgeneric <name> | *)
 **********************************************************/
globle VOID UndefgenericCommand()
  {
   UndefconstructCommand("undefgeneric",DefgenericConstruct);
  }

/****************************************************************
  NAME         : GetDefgenericModuleCommand
  DESCRIPTION  : Determines to which module a defgeneric belongs
  INPUTS       : None
  RETURNS      : The symbolic name of the module
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax: (defgeneric-module <generic-name>)
 ****************************************************************/
globle SYMBOL_HN *GetDefgenericModuleCommand()
  {
   return(GetConstructModuleCommand("defgeneric-module",DefgenericConstruct));
  }
  
/**************************************************************
  NAME         : UndefmethodCommand
  DESCRIPTION  : Deletes one method for a generic function
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : methods deallocated
  NOTES        : CLIPS Syntax: (undefmethod <name> <index> | *)
 **************************************************************/
globle VOID UndefmethodCommand()
  {
   DATA_OBJECT temp;
   DEFGENERIC *gfunc;
   unsigned mi;
   
   if (ArgTypeCheck("undefmethod",1,SYMBOL,&temp) == CLIPS_FALSE)
     return;
   gfunc = LookupDefgenericByMdlOrScope(DOToString(temp));
   if ((gfunc == NULL) ? (strcmp(DOToString(temp),"*") != 0) : CLIPS_FALSE)
     { 
      PrintErrorID("GENRCCOM",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"No such generic function ");
      PrintCLIPS(WERROR,DOToString(temp));
      PrintCLIPS(WERROR," in function undefmethod.\n");
      return;
     }
   RtnUnknown(2,&temp);
   if (temp.type == SYMBOL)
     {
      if (strcmp(DOToString(temp),"*") != 0)
        {
         PrintErrorID("GENRCCOM",2,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Expected a valid method index in function undefmethod.\n");
         return;
        }
      mi = 0;
     }
   else if (temp.type == INTEGER)
     {
      mi = (unsigned) DOToInteger(temp);
      if (mi == 0)
        {
         PrintErrorID("GENRCCOM",2,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Expected a valid method index in function undefmethod.\n");
         return;
        }
     }
   else
     {
      PrintErrorID("GENRCCOM",2,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Expected a valid method index in function undefmethod.\n");
      return;
     }
   Undefmethod((VOID *) gfunc,mi);
  }

/**************************************************************
  NAME         : Undefgeneric
  DESCRIPTION  : Deletes all methods for a generic function
  INPUTS       : The generic-function address (NULL for all)
  RETURNS      : CLIPS_TRUE if generic successfully deleted,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : methods deallocated
  NOTES        : None
 **************************************************************/
globle BOOLEAN Undefgeneric(vptr)
  VOID *vptr;
  {
#if RUN_TIME || BLOAD_ONLY
   return(CLIPS_FALSE);
#else
   DEFGENERIC *gfunc;
   int success = CLIPS_TRUE;

   gfunc = (DEFGENERIC *) vptr;
   if (gfunc == NULL)
     {
      if (ClearDefmethods() == CLIPS_FALSE)
        success = CLIPS_FALSE;
      if (ClearDefgenerics() == CLIPS_FALSE)
        success = CLIPS_FALSE;
      return(success);
     }
   if (IsDefgenericDeletable(vptr) == CLIPS_FALSE)
     return(CLIPS_FALSE);
   RemoveConstructFromModule((struct constructHeader *) vptr);
   RemoveDefgeneric(gfunc);
   return(CLIPS_TRUE);
#endif
  }
  
/**************************************************************
  NAME         : Undefmethod
  DESCRIPTION  : Deletes one method for a generic function
  INPUTS       : 1) Address of generic function (can be NULL)
                 2) Method index (0 for all)
  RETURNS      : CLIPS_TRUE if method deleted successfully,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : methods deallocated
  NOTES        : None
 **************************************************************/
globle BOOLEAN Undefmethod(vptr,mi)
  VOID *vptr;
  unsigned mi;
  {
   DEFGENERIC *gfunc;
   
#if RUN_TIME || BLOAD_ONLY
   gfunc = (DEFGENERIC *) vptr;
   PrintErrorID("PRNTUTIL",4,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Unable to delete method ");
   if (gfunc != NULL)
     {
      PrintGenericName(WERROR,gfunc);
      PrintCLIPS(WERROR," #");
      PrintLongInteger(WERROR,(long) mi);
     }
   else
     PrintCLIPS(WERROR,"*");
   PrintCLIPS(WERROR,".\n");
   return(CLIPS_FALSE);
#else
   int nmi;

   gfunc = (DEFGENERIC *) vptr;
#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded() == CLIPS_TRUE)
     {   
      PrintErrorID("PRNTUTIL",4,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Unable to delete method ");
      if (gfunc != NULL)
        {
         PrintCLIPS(WERROR,GetDefgenericName((VOID *) gfunc));
         PrintCLIPS(WERROR," #");
         PrintLongInteger(WERROR,(long) mi);
        }
      else
        PrintCLIPS(WERROR,"*");
      PrintCLIPS(WERROR,".\n");
      return(CLIPS_FALSE);
     }
#endif
   if (gfunc == NULL)
     {
      if (mi != 0)
        {   
         PrintErrorID("GENRCCOM",3,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Incomplete method specification for deletion.\n");
         return(CLIPS_FALSE);
        }
      return(ClearDefmethods());
     }
   if (MethodsExecuting(gfunc))
     {
      MethodAlterError(gfunc);
      return(CLIPS_FALSE);
     }
   if (mi == 0)
     RemoveAllExplicitMethods(gfunc);
   else
     {
      nmi = CheckMethodExists("undefmethod",gfunc,(int) mi);
      if (nmi == -1)
        return(CLIPS_FALSE);
      RemoveDefgenericMethod(gfunc,nmi);
     }
   return(CLIPS_TRUE);
#endif
  }
  
#if DEBUGGING_FUNCTIONS

/*****************************************************
  NAME         : GetDefmethodDescription
  DESCRIPTION  : Prints a synopsis of method parameter
                   restrictions into caller's buffer
  INPUTS       : 1) Caller's buffer
                 2) Buffer size (not including space
                    for terminating '\0')
                 3) Address of generic function
                 4) Index of method
  RETURNS      : Nothing useful
  SIDE EFFECTS : Caller's buffer written
  NOTES        : Terminating '\n' not written
 *****************************************************/
globle VOID GetDefmethodDescription(buf,buflen,ptr,index)
  char *buf;
  int buflen;
  VOID *ptr;
  unsigned index;
  {
   DEFGENERIC *gfunc;
   int mi;
   
   gfunc = (DEFGENERIC *) ptr;
   mi = FindMethodByIndex(gfunc,index);
   PrintMethod(buf,buflen,&gfunc->methods[mi]);
  }
  
/*********************************************************
  NAME         : GetDefgenericWatch
  DESCRIPTION  : Determines if trace messages are
                 gnerated when executing generic function
  INPUTS       : A pointer to the generic
  RETURNS      : CLIPS_TRUE if a trace is active,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 *********************************************************/
globle BOOLEAN GetDefgenericWatch(theGeneric)
  VOID *theGeneric;
  {
   return(((DEFGENERIC *) theGeneric)->trace);
  }
  
/*********************************************************
  NAME         : SetDefgenericWatch
  DESCRIPTION  : Sets the trace to ON/OFF for the
                 generic function
  INPUTS       : 1) CLIPS_TRUE to set the trace on,
                    CLIPS_FALSE to set it off
                 2) A pointer to the generic
  RETURNS      : Nothing useful
  SIDE EFFECTS : Watch flag for the generic set
  NOTES        : None
 *********************************************************/
globle VOID SetDefgenericWatch(newState,theGeneric)
  int newState;
  VOID *theGeneric;
  {
   ((DEFGENERIC *) theGeneric)->trace = newState;
  }
  
/*********************************************************
  NAME         : GetDefmethodWatch
  DESCRIPTION  : Determines if trace messages for calls
                 to this method will be generated or not
  INPUTS       : 1) A pointer to the generic
                 2) The index of the method
  RETURNS      : CLIPS_TRUE if a trace is active,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 *********************************************************/
globle BOOLEAN GetDefmethodWatch(theGeneric,theIndex)
  VOID *theGeneric;
  unsigned theIndex;
  {
   DEFGENERIC *gfunc;
   int mi;
   
   gfunc = (DEFGENERIC *) theGeneric;
   mi = FindMethodByIndex(gfunc,theIndex);
   return(gfunc->methods[mi].trace);
  }
  
/*********************************************************
  NAME         : SetDefmethodWatch
  DESCRIPTION  : Sets the trace to ON/OFF for the
                 calling of the method
  INPUTS       : 1) CLIPS_TRUE to set the trace on,
                    CLIPS_FALSE to set it off
                 2) A pointer to the generic
                 3) The index of the method
  RETURNS      : Nothing useful
  SIDE EFFECTS : Watch flag for the method set
  NOTES        : None
 *********************************************************/
globle VOID SetDefmethodWatch(newState,theGeneric,theIndex)
  int newState;
  VOID *theGeneric;
  unsigned theIndex;
  {
   DEFGENERIC *gfunc;
   int mi;
   
   gfunc = (DEFGENERIC *) theGeneric;
   mi = FindMethodByIndex(gfunc,theIndex);
   gfunc->methods[mi].trace = newState;
  }


/********************************************************
  NAME         : PPDefgenericCommand
  DESCRIPTION  : Displays the pretty-print form of
                  a generic function header
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax: (ppdefgeneric <name>)
 ********************************************************/
globle VOID PPDefgenericCommand()
  {
   PPConstructCommand("ppdefgeneric",DefgenericConstruct);
  }
  
/**********************************************************
  NAME         : PPDefmethodCommand
  DESCRIPTION  : Displays the pretty-print form of
                  a method
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax: (ppdefmethod <name> <index>)
 **********************************************************/
globle VOID PPDefmethodCommand()
  {
   DATA_OBJECT temp;
   char *gname;
   DEFGENERIC *gfunc;
   int gi;

   if (ArgTypeCheck("ppdefmethod",1,SYMBOL,&temp) == CLIPS_FALSE)
     return;
   gname = DOToString(temp);
   if (ArgTypeCheck("ppdefmethod",2,INTEGER,&temp) == CLIPS_FALSE)
     return;
   gfunc = CheckGenericExists("ppdefmethod",gname);
   if (gfunc == NULL)
     return;
   gi = CheckMethodExists("ppdefmethod",gfunc,DOToInteger(temp));
   if (gi == -1)
     return;
   if (gfunc->methods[gi].ppForm != NULL)
     PrintInChunks(WDISPLAY,gfunc->methods[gi].ppForm);
  }
  
/******************************************************
  NAME         : ListDefmethodsCommand
  DESCRIPTION  : Lists a brief description of methods
                   for a particular generic function
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax: (list-defmethods <name>)
 ******************************************************/
globle VOID ListDefmethodsCommand()
  {
   DATA_OBJECT temp;
   DEFGENERIC *gfunc;
   
   if (RtnArgCount() == 0)
     ListDefmethods(WDISPLAY,NULL);
   else
     {
      if (ArgTypeCheck("list-defmethods",1,SYMBOL,&temp) == CLIPS_FALSE)
        return;
      gfunc = CheckGenericExists("list-defmethods",DOToString(temp));
      if (gfunc != NULL)
        ListDefmethods(WDISPLAY,(VOID *) gfunc);
     }
  }

/***************************************************************
  NAME         : GetDefmethodPPForm
  DESCRIPTION  : Getsa generic function method pretty print form
  INPUTS       : 1) Address of the generic function
                 2) Index of the method
  RETURNS      : Method ppform
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************************/
globle char *GetDefmethodPPForm(ptr,index)
  VOID *ptr;
  unsigned index;
  {
   DEFGENERIC *gfunc;
   int mi;
   
   gfunc = (DEFGENERIC *) ptr;
   mi = FindMethodByIndex(gfunc,index);
   return(gfunc->methods[mi].ppForm);
  }

/***************************************************
  NAME         : ListDefgenericsCommand
  DESCRIPTION  : Displays all defgeneric names
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Defgeneric names printed
  NOTES        : CLIPS Interface
 ***************************************************/
globle VOID ListDefgenericsCommand()
  {
   ListConstructCommand("list-defgenerics",DefgenericConstruct);
  }

/***************************************************
  NAME         : ListDefgenerics
  DESCRIPTION  : Displays all defgeneric names
  INPUTS       : 1) The logical name of the output
                 2) The module
  RETURNS      : Nothing useful
  SIDE EFFECTS : Defgeneric names printed
  NOTES        : C Interface
 ***************************************************/
globle VOID ListDefgenerics(logicalName,theModule)
  char *logicalName;
  struct defmodule *theModule;
  {
   ListConstruct(DefgenericConstruct,logicalName,theModule);
  }

/******************************************************
  NAME         : ListDefmethods
  DESCRIPTION  : Lists a brief description of methods
                   for a particular generic function
  INPUTS       : 1) The logical name of the output
                 2) Generic function to list methods for
                    (NULL means list all methods)
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : None
 ******************************************************/
globle VOID ListDefmethods(logicalName,vptr)
  char *logicalName;
  VOID *vptr;
  {
   DEFGENERIC *gfunc;
   long count;
   
   if (vptr != NULL)
     count = ListMethodsForGeneric(logicalName,(DEFGENERIC *) vptr);
   else
     {
      count = 0L;
      for (gfunc = (DEFGENERIC *) GetNextDefgeneric(NULL) ;
           gfunc != NULL ;
           gfunc = (DEFGENERIC *) GetNextDefgeneric((VOID *) gfunc))
        {
         count += ListMethodsForGeneric(logicalName,gfunc);
         if (GetNextDefgeneric((VOID *) gfunc) != NULL)
           PrintCLIPS(logicalName,"\n");
        }
     }
   PrintTally(logicalName,count,"method","methods");
  }

#endif

/***************************************************************
  NAME         : GetDefgenericListFunction
  DESCRIPTION  : Groups all defgeneric names into
                 a multifield list
  INPUTS       : A data object buffer to hold
                 the multifield result
  RETURNS      : Nothing useful
  SIDE EFFECTS : Multifield allocated and filled
  NOTES        : CLIPS Syntax: (get-defgeneric-list [<module>])
 ***************************************************************/
globle VOID GetDefgenericListFunction(returnValue)
  DATA_OBJECT*returnValue;
  {
   GetConstructListFunction("get-defgeneric-list",returnValue,DefgenericConstruct);
  }

/***************************************************************
  NAME         : GetDefgenericList
  DESCRIPTION  : Groups all defgeneric names into
                 a multifield list
  INPUTS       : 1) A data object buffer to hold
                    the multifield result
                 2) The module from which to obtain defgenerics
  RETURNS      : Nothing useful
  SIDE EFFECTS : Multifield allocated and filled
  NOTES        : External C access
 ***************************************************************/
globle VOID GetDefgenericList(returnValue,theModule)
  DATA_OBJECT *returnValue;
  struct defmodule *theModule;
  {
   GetConstructList(returnValue,DefgenericConstruct,theModule);
  }
  
/***********************************************************
  NAME         : GetDefmethodListCommand
  DESCRIPTION  : Groups indices of all methdos for a generic
                 function into a multifield variable
                 (NULL means get methods for all generics)
  INPUTS       : A data object buffer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Multifield set to list of method indices
  NOTES        : None
 ***********************************************************/
globle VOID GetDefmethodListCommand(returnValue)
  DATA_OBJECT_PTR returnValue;
  {
   DATA_OBJECT temp;
   DEFGENERIC *gfunc;
   
   if (RtnArgCount() == 0)
     GetDefmethodList(NULL,returnValue);
   else
     {
      if (ArgTypeCheck("get-defmethod-list",1,SYMBOL,&temp) == CLIPS_FALSE)
        {
         SetMultifieldErrorValue(returnValue);
         return;
        }
      gfunc = CheckGenericExists("get-defmethod-list",DOToString(temp));
      if (gfunc != NULL)
        GetDefmethodList((VOID *) gfunc,returnValue);
      else
        SetMultifieldErrorValue(returnValue);
     }
  }
  
/***********************************************************
  NAME         : GetDefmethodList
  DESCRIPTION  : Groups indices of all methdos for a generic
                 function into a multifield variable
                 (NULL means get methods for all generics)
  INPUTS       : 1) A pointer to a generic function
                 2) A data object buffer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Multifield set to list of method indices
  NOTES        : None
 ***********************************************************/
globle VOID GetDefmethodList(vgfunc,returnValue)
  VOID *vgfunc;
  DATA_OBJECT_PTR returnValue;
  {
   DEFGENERIC *gfunc,*svg,*svnxt;
   int i,j,count;
   MULTIFIELD_PTR theList;
   
   if (vgfunc != NULL)
     {
      gfunc = (DEFGENERIC *) vgfunc;
      svnxt = (DEFGENERIC *) GetNextDefgeneric(vgfunc);
      SetNextDefgeneric(vgfunc,NULL);
     }
   else
     {
      gfunc = (DEFGENERIC *) GetNextDefgeneric(NULL);
      svnxt = (gfunc != NULL) ? (DEFGENERIC *) GetNextDefgeneric((VOID *) gfunc) : NULL;
     }
   count = 0;
   for (svg = gfunc ;
        gfunc != NULL ;
        gfunc = (DEFGENERIC *) GetNextDefgeneric((VOID *) gfunc))
     count += (int) gfunc->mcnt;
   count *= 2;
   SetpType(returnValue,MULTIFIELD);
   SetpDOBegin(returnValue,1);
   SetpDOEnd(returnValue,count);
   theList = (MULTIFIELD_PTR) CreateMultifield(count);
   SetpValue(returnValue,theList);
   for (gfunc = svg , i = 1 ;
        gfunc != NULL ;
        gfunc = (DEFGENERIC *) GetNextDefgeneric((VOID *) gfunc))
     {
      for (j = 0 ; j < gfunc->mcnt ; j++)
        {
         SetMFType(theList,i,SYMBOL);
         SetMFValue(theList,i++,GetDefgenericNamePointer((VOID *) gfunc));
         SetMFType(theList,i,INTEGER);
         SetMFValue(theList,i++,AddLong((long) gfunc->methods[j].index));
        }
     }
   if (svg != NULL)
     SetNextDefgeneric((VOID *) svg,(VOID *) svnxt);
  }

/***********************************************************************************
  NAME         : GetMethodRestrictionsCommand
  DESCRIPTION  : Stores restrictions of a method in multifield
  INPUTS       : A data object buffer to hold a multifield
  RETURNS      : Nothing useful
  SIDE EFFECTS : Multifield created (length zero on errors)
  NOTES        : Syntax: (get-method-restrictions <generic-function> <method-index>)
 ***********************************************************************************/
globle VOID GetMethodRestrictionsCommand(result)
  DATA_OBJECT *result;
  {
   DATA_OBJECT temp;
   DEFGENERIC *gfunc;
   
   if (ArgTypeCheck("get-method-restrictions",1,SYMBOL,&temp) == CLIPS_FALSE)
     {
      SetMultifieldErrorValue(result);
      return;
     }
   gfunc = CheckGenericExists("get-method-restrictions",DOToString(temp));
   if (gfunc == NULL)
     {
      SetMultifieldErrorValue(result);
      return;
     }
   if (ArgTypeCheck("get-method-restrictions",2,INTEGER,&temp) == CLIPS_FALSE)
     {
      SetMultifieldErrorValue(result);
      return;
     }
   if (CheckMethodExists("get-method-restrictions",gfunc,DOToInteger(temp)) == -1)
     {
      SetMultifieldErrorValue(result);
      return;
     }
   GetMethodRestrictions((VOID *) gfunc,(unsigned) DOToInteger(temp),result);
  }
  
/***********************************************************************
  NAME         : GetMethodRestrictions
  DESCRIPTION  : Stores restrictions of a method in multifield
  INPUTS       : 1) Pointer to the generic function
                 2) The method index
                 3) A data object buffer to hold a multifield
  RETURNS      : Nothing useful
  SIDE EFFECTS : Multifield created (length zero on errors)
  NOTES        : The restrictions are stored in the multifield
                 in the following format:
                 
                 <min-number-of-arguments>
                 <max-number-of-arguments> (-1 if wildcard allowed)
                 <restriction-count>
                 <index of 1st restriction>
                       .
                       .
                 <index of nth restriction>
                 <restriction 1>
                     <query TRUE/FALSE>
                     <number-of-classes>
                     <class 1>
                        .
                        .
                     <class n>
                    .
                    .
                    .
                  <restriction n>
                  
                  Thus, for the method
                  (defmethod foo ((?a NUMBER SYMBOL) (?b (= 1 1)) $?c))
                  (get-method-restrictions foo 1) would yield
                  
                  (2 -1 3 7 11 13 FALSE 2 NUMBER SYMBOL TRUE 0 FALSE 0)
 ***********************************************************************/
globle VOID GetMethodRestrictions(vgfunc,mi,result)
  VOID *vgfunc;
  unsigned mi;
  DATA_OBJECT *result;
  {
   register int i,j;
   register DEFMETHOD *meth;
   register RESTRICTION *rptr;
   int count,roffset,rindex;
   MULTIFIELD_PTR theList;
   
   meth = ((DEFGENERIC *) vgfunc)->methods + FindMethodByIndex((DEFGENERIC *) vgfunc,mi);
   count = 3;
   for (i = 0 ; i < meth->restrictionCount ; i++)
     count += meth->restrictions[i].tcnt + 3;
   theList = (MULTIFIELD_PTR) CreateMultifield(count);
   SetpType(result,MULTIFIELD);
   SetpValue(result,theList);
   SetpDOBegin(result,1);
   SetpDOEnd(result,count);
   SetMFType(theList,1,INTEGER);
   SetMFValue(theList,1,AddLong((long) meth->minRestrictions));
   SetMFType(theList,2,INTEGER);
   SetMFValue(theList,2,AddLong((long) meth->maxRestrictions));
   SetMFType(theList,3,INTEGER);
   SetMFValue(theList,3,AddLong((long) meth->restrictionCount));
   roffset = 3 + meth->restrictionCount + 1;
   rindex = 4;
   for (i = 0 ; i < meth->restrictionCount ; i++)
     {
      rptr = meth->restrictions + i;
      SetMFType(theList,rindex,INTEGER);
      SetMFValue(theList,rindex++,AddLong((long) roffset));
      SetMFType(theList,roffset,SYMBOL);
      SetMFValue(theList,roffset++,(rptr->query != NULL) ? CLIPSTrueSymbol : CLIPSFalseSymbol);
      SetMFType(theList,roffset,INTEGER);
      SetMFValue(theList,roffset++,AddLong((long) rptr->tcnt));
      for (j = 0 ; j < rptr->tcnt ; j++)
        {
         SetMFType(theList,roffset,SYMBOL);
#if OBJECT_SYSTEM
         SetMFValue(theList,roffset++,AddSymbol(GetDefclassName(rptr->types[j])));
#else
         SetMFValue(theList,roffset++,AddSymbol(TypeName(ValueToInteger(rptr->types[j]))));
#endif
        }
     }
  }
  
/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

/***************************************************
  NAME         : PrintGenericCall
  DESCRIPTION  : PrintExpression() support function
                 for generic function calls
  INPUTS       : 1) The output logical name
                 2) The generic function
  RETURNS      : Nothing useful
  SIDE EFFECTS : Call expression printed
  NOTES        : None
 ***************************************************/
#if IBM_TBC && (! DEVELOPER)
#pragma argsused
#endif
static VOID PrintGenericCall(log,value)
  char *log;
  VOID *value;
  {
#if DEVELOPER
   PrintCLIPS(log,"(");
   PrintCLIPS(log,GetDefgenericName(value));
   if (GetFirstArgument() != NULL)
     {
      PrintCLIPS(log," ");
      PrintExpression(log,GetFirstArgument());
     }
   PrintCLIPS(log,")");
#else
#if MAC_MPW
#pragma unused(log)
#pragma unused(value)
#endif
#endif
  }

/*******************************************************
  NAME         : EvaluateGenericCall
  DESCRIPTION  : Primitive support function for
                 calling a generic function
  INPUTS       : 1) The generic function
                 2) A data object buffer to hold
                    the evaluation result
  RETURNS      : CLIPS_FALSE if the generic function
                 returns the symbol FALSE,
                 CLIPS_TRUE otherwise
  SIDE EFFECTS : Data obejct buffer set and any
                 side-effects of calling the generic
  NOTES        : None
 *******************************************************/
static BOOLEAN EvaluateGenericCall(value,result)
  VOID *value;
  DATA_OBJECT *result;
  {
   GenericDispatch((DEFGENERIC *) value,NULL,NULL,GetFirstArgument(),result);
   if ((GetpType(result) == SYMBOL) &&
       (GetpValue(result) == CLIPSFalseSymbol))
     return(CLIPS_FALSE);
   return(CLIPS_TRUE);
  }

/***************************************************
  NAME         : DecrementGenericBusyCount
  DESCRIPTION  : Lowers the busy count of a
                 generic function construct
  INPUTS       : The generic function
  RETURNS      : Nothing useful
  SIDE EFFECTS : Busy count decremented if a clear
                 is not in progress (see comment)
  NOTES        : None
 ***************************************************/
static VOID DecrementGenericBusyCount(value)
  VOID *value;
  {
   /* ==============================================
      The generics to which expressions in other
      constructs may refer may already have been
      deleted - thus, it is important not to modify
      the busy flag during a clear.
      ============================================== */
   if (! ClearInProgress)
     ((DEFGENERIC *) value)->busy--;
  }

/***************************************************
  NAME         : IncrementGenericBusyCount
  DESCRIPTION  : Raises the busy count of a
                 generic function construct
  INPUTS       : The generic function
  RETURNS      : Nothing useful
  SIDE EFFECTS : Busy count incremented
  NOTES        : None
 ***************************************************/
static VOID IncrementGenericBusyCount(value)
  VOID *value;
  {
   ((DEFGENERIC *) value)->busy++;
  }

#if (! BLOAD_ONLY) && (! RUN_TIME)

/**********************************************************************
  NAME         : SaveDefgenerics
  DESCRIPTION  : Outputs pretty-print forms of generic function headers
  INPUTS       : The logical name of the output
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : None
 **********************************************************************/
static VOID SaveDefgenerics(log)
  char *log;
  {
   SaveConstruct(log,DefgenericConstruct);
  }

/**********************************************************************
  NAME         : SaveDefmethods
  DESCRIPTION  : Outputs pretty-print forms of generic function methods
  INPUTS       : The logical name of the output
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : None
 **********************************************************************/
static VOID SaveDefmethods(log)
  char *log;
  {
   DoForAllConstructs(SaveDefmethodsForDefgeneric,DefgenericModuleIndex,
                      CLIPS_FALSE,(VOID *) log);
  }
  
/***************************************************
  NAME         : SaveDefmethodsForDefgeneric
  DESCRIPTION  : Save the pretty-print forms of
                 all methods for a generic function
                 to a file
  INPUTS       : 1) The defgeneric
                 2) The logical name of the output
  RETURNS      : Nothing useful
  SIDE EFFECTS : Methods written
  NOTES        : None
 ***************************************************/
static VOID SaveDefmethodsForDefgeneric(theDefgeneric,userBuffer)
  struct constructHeader *theDefgeneric;
  VOID *userBuffer;
  {
   DEFGENERIC *gfunc = (DEFGENERIC *) theDefgeneric;
   char *log = (char *) userBuffer;
   register unsigned i;
   
   for (i = 0 ; i < gfunc->mcnt ; i++)
     {
      if (gfunc->methods[i].ppForm != NULL)
        {
         PrintInChunks(log,gfunc->methods[i].ppForm);
         PrintCLIPS(log,"\n");
        }
     }
  }

/****************************************************
  NAME         : RemoveDefgenericMethod
  DESCRIPTION  : Removes a generic function method
                   from the array and removes the
                   generic too if its the last method
  INPUTS       : 1) The generic function
                 2) The array index of the method
  RETURNS      : Nothing useful
  SIDE EFFECTS : List adjusted
                 Nodes deallocated
  NOTES        : Assumes deletion is safe
 ****************************************************/
static VOID RemoveDefgenericMethod(gfunc,gi)
  DEFGENERIC *gfunc;
  int gi;
  {
   DEFMETHOD *narr;
   register int b,e;
   
   if (gfunc->methods[gi].system)
     {
      SetEvaluationError(CLIPS_TRUE);
      PrintErrorID("GENRCCOM",4,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Cannot remove implicit system function method for generic function ");
      PrintCLIPS(WERROR,GetDefgenericName((VOID *) gfunc));
      PrintCLIPS(WERROR,".\n");
      return;
     }
   DeleteMethodInfo(gfunc,&gfunc->methods[gi]);
   if (gfunc->mcnt == 1)
     {
      rm((VOID *) gfunc->methods,(int) sizeof(DEFMETHOD));
      gfunc->mcnt = 0;
      gfunc->methods = NULL;
     }
   else
     {
      gfunc->mcnt--;
      narr = (DEFMETHOD *) gm2((int) (sizeof(DEFMETHOD) * gfunc->mcnt));
      for (b = e = 0 ; b < gfunc->mcnt ; b++ , e++)
        {
         if (b == gi)
           e++;
         CopyMemory(DEFMETHOD,1,&narr[b],&gfunc->methods[e]);
        }
      rm((VOID *) gfunc->methods,(int) (sizeof(DEFMETHOD) * (gfunc->mcnt+1)));
      gfunc->methods = narr;
     }
  }

#endif
  
#if DEBUGGING_FUNCTIONS

/******************************************************
  NAME         : ListMethodsForGeneric
  DESCRIPTION  : Lists a brief description of methods
                   for a particular generic function
  INPUTS       : 1) The logical name of the output
                 2) Generic function to list methods for
  RETURNS      : The number of methods printed
  SIDE EFFECTS : None
  NOTES        : None
 ******************************************************/
static long ListMethodsForGeneric(logicalName,gfunc)
  char *logicalName;
  DEFGENERIC *gfunc;
  {
   int gi;
   char buf[256];

   for (gi = 0 ; gi < gfunc->mcnt ; gi++)
     {
      PrintCLIPS(logicalName,GetDefgenericName((VOID *) gfunc));
      PrintCLIPS(logicalName," #");
      PrintMethod(buf,255,&gfunc->methods[gi]);
      PrintCLIPS(logicalName,buf);
      PrintCLIPS(logicalName,"\n");
     }
   return((long) gfunc->mcnt);
  }

/******************************************************************
  NAME         : DefgenericWatchAccess
  DESCRIPTION  : Parses a list of generic names passed by
                 AddWatchItem() and sets the traces accordingly
  INPUTS       : 1) A code indicating which trace flag is to be set
                    Ignored
                 2) The value to which to set the trace flags
                 3) A list of expressions containing the names
                    of the generics for which to set traces
  RETURNS      : CLIPS_TRUE if all OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : Watch flags set in specified generics
  NOTES        : Accessory function for AddWatchItem()
 ******************************************************************/
#if IBM_TBC
#pragma argsused
#endif
static BOOLEAN DefgenericWatchAccess(code,newState,argExprs)
  int code,newState;
  EXPRESSION *argExprs;
  {
#if MAC_MPW
#pragma unused(code)
#endif
   return(ConstructSetWatchAccess(DefgenericConstruct,newState,argExprs,
                                    GetDefgenericWatch,SetDefgenericWatch));
  }
  
/***********************************************************************
  NAME         : DefgenericWatchPrint
  DESCRIPTION  : Parses a list of generic names passed by
                 AddWatchItem() and displays the traces accordingly
  INPUTS       : 1) The logical name of the output
                 2) A code indicating which trace flag is to be examined
                    Ignored
                 3) A list of expressions containing the names
                    of the generics for which to examine traces
  RETURNS      : CLIPS_TRUE if all OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : Watch flags displayed for specified generics
  NOTES        : Accessory function for AddWatchItem()
 ***********************************************************************/
#if IBM_TBC
#pragma argsused
#endif
static BOOLEAN DefgenericWatchPrint(log,code,argExprs)
  char *log;
  int code;
  EXPRESSION *argExprs;
  {
#if MAC_MPW
#pragma unused(code)
#endif
   return(ConstructPrintWatchAccess(DefgenericConstruct,log,argExprs,
                                    GetDefgenericWatch,SetDefgenericWatch));
  }
  
/******************************************************************
  NAME         : DefmethodWatchAccess
  DESCRIPTION  : Parses a list of methods passed by
                 AddWatchItem() and sets the traces accordingly
  INPUTS       : 1) A code indicating which trace flag is to be set
                    Ignored
                 2) The value to which to set the trace flags
                 3) A list of expressions containing the methods
                   for which to set traces
  RETURNS      : CLIPS_TRUE if all OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : Watch flags set in specified methods
  NOTES        : Accessory function for AddWatchItem()
 ******************************************************************/
#if IBM_TBC
#pragma argsused
#endif
static BOOLEAN DefmethodWatchAccess(code,newState,argExprs)
  int code,newState;
  EXPRESSION *argExprs;
  {
#if MAC_MPW
#pragma unused(code)
#endif
   return(DefmethodWatchSupport(newState ? "watch" : "unwatch",NULL,
                                newState,NULL,SetDefmethodWatch,argExprs));
  }
  
/***********************************************************************
  NAME         : DefmethodWatchPrint
  DESCRIPTION  : Parses a list of methods passed by
                 AddWatchItem() and displays the traces accordingly
  INPUTS       : 1) The logical name of the output
                 2) A code indicating which trace flag is to be examined
                    Ignored
                 3) A list of expressions containing the methods for
                    which to examine traces
  RETURNS      : CLIPS_TRUE if all OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : Watch flags displayed for specified methods
  NOTES        : Accessory function for AddWatchItem()
 ***********************************************************************/
#if IBM_TBC
#pragma argsused
#endif
static BOOLEAN DefmethodWatchPrint(log,code,argExprs)
  char *log;
  int code;
  EXPRESSION *argExprs;
  {
#if MAC_MPW
#pragma unused(code)
#endif
   return(DefmethodWatchSupport("list-watch-items",log,-1,
                                PrintMethodWatchFlag,NULL,argExprs));
  }

/*******************************************************
  NAME         : DefmethodWatchSupport
  DESCRIPTION  : Sets or displays methods specified
  INPUTS       : 1) The calling function name
                 2) The logical output name for displays
                    (can be NULL)
                 3) The new set state (can be -1)
                 4) The print function (can be NULL)
                 5) The trace function (can be NULL)
                 6) The methods expression list
  RETURNS      : CLIPS_TRUE if all OK,
                 CLIPS_FALSE otherwise
  SIDE EFFECTS : Method trace flags set or displayed
  NOTES        : None
 *******************************************************/
static BOOLEAN DefmethodWatchSupport(funcName,log,newState,printFunc,traceFunc,argExprs)
  char *funcName,*log;
  int newState;
#if ANSI_COMPILER
  VOID (*printFunc)(char *,VOID *,unsigned);
  VOID (*traceFunc)(int,VOID *,unsigned);
#else
  VOID (*printFunc)();
  VOID (*traceFunc)();
#endif
  EXPRESSION *argExprs;
  {
   VOID *theGeneric;
   unsigned theMethod;
   int argIndex = 2;
   DATA_OBJECT genericName,methodIndex;
   struct defmodule *theModule;
   
   /* ==============================
      If no methods are specified,
      show the trace for all methods
      in all generics
      ============================== */
   if (argExprs == NULL)
     {
      SaveCurrentModule();
      theModule = (struct defmodule *) GetNextDefmodule(NULL);
      while (theModule != NULL)
        {
         SetCurrentModule((VOID *) theModule);
         if (traceFunc == NULL)
           {
            PrintCLIPS(log,GetDefmoduleName((VOID *) theModule));
            PrintCLIPS(log,":\n");
           }
         theGeneric = GetNextDefgeneric(NULL);
         while (theGeneric != NULL)
            {
             theMethod = GetNextDefmethod(theGeneric,0);
             while (theMethod != 0)
               {
                if (traceFunc != NULL)
                  (*traceFunc)(newState,theGeneric,theMethod);
                else
                  {
                   PrintCLIPS(log,"   ");
                   (*printFunc)(log,theGeneric,theMethod);
                  }
                theMethod = GetNextDefmethod(theGeneric,theMethod);
               }
             theGeneric = GetNextDefgeneric(theGeneric);
            }
         theModule = (struct defmodule *) GetNextDefmodule((VOID *) theModule);
        }
      RestoreCurrentModule();
      return(CLIPS_TRUE);
     }
     
   /* =========================================
      Set the traces for every method specified
      ========================================= */
   while (argExprs != NULL)
     {
      if (EvaluateExpression(argExprs,&genericName))
        return(CLIPS_FALSE);
      if ((genericName.type != SYMBOL) ? CLIPS_TRUE :
          ((theGeneric = (VOID *) 
              LookupDefgenericByMdlOrScope(DOToString(genericName))) == NULL))
        {
         ExpectedTypeError1(funcName,argIndex,"generic function name");
         return(CLIPS_FALSE);
        }
      if (GetNextArgument(argExprs) == NULL)
        theMethod = 0;
      else
        {
         argExprs = GetNextArgument(argExprs);
         argIndex++;
         if (EvaluateExpression(argExprs,&methodIndex))
           return(CLIPS_FALSE);
         if ((methodIndex.type != INTEGER) ? CLIPS_FALSE :
             ((DOToInteger(methodIndex) <= 0) ? CLIPS_FALSE :
              (FindMethodByIndex((DEFGENERIC *) theGeneric,theMethod) != -1)))
           theMethod = (unsigned) DOToInteger(methodIndex);
         else
           {
            ExpectedTypeError1(funcName,argIndex,"method index");
            return(CLIPS_FALSE);
           }
        }
      if (theMethod == 0)
        {
         theMethod = GetNextDefmethod(theGeneric,0);
         while (theMethod != 0)
           {
            if (traceFunc != NULL)
              (*traceFunc)(newState,theGeneric,theMethod);
            else
              (*printFunc)(log,theGeneric,theMethod);
            theMethod = GetNextDefmethod(theGeneric,theMethod);
           }
        }
      else
        {
         if (traceFunc != NULL)
           (*traceFunc)(newState,theGeneric,theMethod);
         else
           (*printFunc)(log,theGeneric,theMethod);
        }
      argExprs = GetNextArgument(argExprs);
      argIndex++;
     }
   return(CLIPS_TRUE);
  }

/***************************************************
  NAME         : PrintMethodWatchFlag
  DESCRIPTION  : Displays trace value for method
  INPUTS       : 1) The logical name of the output
                 2) The generic function
                 3) The method index
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
static VOID PrintMethodWatchFlag(log,theGeneric,theMethod)
  char *log;
  VOID *theGeneric;
  unsigned theMethod;
  {
   char buf[60];

   PrintCLIPS(log,GetDefgenericName(theGeneric));
   PrintCLIPS(log," ");
   GetDefmethodDescription(buf,59,theGeneric,theMethod);
   PrintCLIPS(log,buf);
   PrintCLIPS(log,GetDefmethodWatch(theGeneric,theMethod) ? " = on\n" : " = off\n");
  }
    
#endif

#if ! OBJECT_SYSTEM

/***************************************************
  NAME         : TypeCommand
  DESCRIPTION  : Works like "class" in COOL
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : CLIPS Syntax: (type <primitive>)
 ***************************************************/
globle VOID TypeCommand(result)
  DATA_OBJECT *result;
  {
   EvaluateExpression(GetFirstArgument(),result);
   result->value = (VOID *) AddSymbol(TypeName(result->type));
   result->type = SYMBOL;
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
