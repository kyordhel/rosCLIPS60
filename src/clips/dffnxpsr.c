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
/* Purpose: CLIPS Deffunction Parsing Routines               */
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

#if DEFFUNCTION_CONSTRUCT && (! BLOAD_ONLY) && (! RUN_TIME)

#if BLOAD || BLOAD_AND_BSAVE
#include "bload.h"
#endif

#if DEFRULE_CONSTRUCT
#include "network.h"
#endif

#if DEFGENERIC_CONSTRUCT
#include "genrccom.h"
#endif

#include "clipsmem.h"
#include "constant.h"
#include "cstrcpsr.h"
#include "constrct.h"
#include "dffnxfun.h"
#include "expressn.h"
#include "exprnpsr.h"
#include "extnfunc.h"
#include "prccode.h"
#include "router.h"
#include "scanner.h"
#include "symbol.h"

#define _DFFNXPSR_SOURCE_
#include "dffnxpsr.h"

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

static BOOLEAN ValidDeffunctionName(char *);
static DEFFUNCTION *AddDeffunction(SYMBOL_HN *,EXPRESSION *,int,int,int,int);

#else /* ANSI_COMPILER */

static BOOLEAN ValidDeffunctionName();
static DEFFUNCTION *AddDeffunction();

#endif /* ANSI_COMPILER */
      
/* =========================================
   *****************************************
      INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
static struct token DFInputToken;

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */   
 
/***************************************************************************
  NAME         : ParseDeffunction
  DESCRIPTION  : Parses the deffunction construct
  INPUTS       : The input logical name
  RETURNS      : CLIPS_FALSE if successful parse, CLIPS_TRUE otherwise
  SIDE EFFECTS : Creates valid deffunction definition
  NOTES        : CLIPS Syntax :
                 (deffunction <name> [<comment>]
                    (<single-field-varible>* [<multifield-variable>])
                    <action>*)
 ***************************************************************************/
globle BOOLEAN ParseDeffunction(readSource)
  char *readSource;
  {
   SYMBOL_HN *deffunctionName;
   EXPRESSION *actions;
   EXPRESSION *parameterList;
   SYMBOL_HN *wildcard;
   int min,max,lvars,DeffunctionError = CLIPS_FALSE;

   DEFFUNCTION *dptr;

   SetPPBufferStatus(ON);

   FlushPPBuffer();
   SetIndentDepth(3);
   SavePPBuffer("(deffunction ");

#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded() == CLIPS_TRUE)
     {
      CannotLoadWithBloadMessage("deffunctions");
      return(CLIPS_TRUE);
     }
#endif

   /* =====================================================
      Parse the name and comment fields of the deffunction.
      ===================================================== */
   deffunctionName = GetConstructNameAndComment(readSource,&DFInputToken,"deffunction",
                                                FindDeffunction,NULL,
                                                "!",CLIPS_TRUE,CLIPS_TRUE,CLIPS_TRUE);
   if (deffunctionName == NULL)
     return(CLIPS_TRUE);
     
   if (ValidDeffunctionName(ValueToString(deffunctionName)) == CLIPS_FALSE)
     return(CLIPS_TRUE);

   /*==========================*/
   /* Parse the argument list. */
   /*==========================*/
   parameterList = ParseProcParameters(readSource,&DFInputToken,NULL,&wildcard,
                                       &min,&max,&DeffunctionError,NULL);
   if (DeffunctionError)
     return(CLIPS_TRUE);

   /*===================================================================*/
   /* Go ahead and add the deffunction so it can be recursively called. */
   /*===================================================================*/

   dptr = AddDeffunction(deffunctionName,NULL,min,max,0,CLIPS_TRUE);
   if (dptr == NULL)
     {
      ReturnExpression(parameterList);
      return(CLIPS_TRUE);
     }

   /*==================================================*/
   /* Parse the actions contained within the function. */
   /*==================================================*/

   PPCRAndIndent();

   ReturnContext = CLIPS_TRUE;
   actions = ParseProcActions("deffunction",readSource,
                              &DFInputToken,parameterList,wildcard,
                              NULL,NULL,&lvars,NULL);
   if (actions == NULL)
     {
      ReturnExpression(parameterList);
      if (dptr->busy == 0)
        {
         RemoveConstructFromModule((struct constructHeader *) dptr);
         RemoveDeffunction(dptr);
        }
      return(CLIPS_TRUE);
     }

   /*=============================*/
   /* Reformat the closing token. */
   /*=============================*/

   PPBackup();
   PPBackup();
   SavePPBuffer(DFInputToken.print_rep);
   SavePPBuffer("\n");

   /*======================*/
   /* Add the deffunction. */
   /*======================*/

   AddDeffunction(deffunctionName,actions,min,max,lvars,CLIPS_FALSE);

   ReturnExpression(parameterList);

   return(DeffunctionError);
  }

/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

/************************************************************
  NAME         : ValidDeffunctionName
  DESCRIPTION  : Determines if a new deffunction of the given
                 name can be defined in the current module
  INPUTS       : The new deffunction name
  RETURNS      : CLIPS_TRUE if OK, CLIPS_FALSE otherwise
  SIDE EFFECTS : Error message printed if not OK
  NOTES        : GetConstructNameAndComment() (called before
                 this function) ensures that the deffunction
                 name does not conflict with one from
                 another module
 ************************************************************/
static BOOLEAN ValidDeffunctionName(theDeffunctionName)
  char *theDeffunctionName;
  {
   struct constructHeader *theDeffunction;
#if DEFGENERIC_CONSTRUCT
   struct defmodule *theModule;
   struct constructHeader *theDefgeneric;
#endif
   
   /* ============================================
      A deffunction cannot be named the same as a
      construct type, e.g, defclass, defrule, etc.
      ============================================ */
   if (FindConstruct(theDeffunctionName) != NULL)
     {
      PrintErrorID("DFFNXPSR",1,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Deffunctions are not allowed to replace constructs.\n");
      return(CLIPS_FALSE);
     }

   /* ============================================
      A deffunction cannot be named the same as a
      pre-defined system function, e.g, watch,
      list-defrules, etc.
      ============================================ */
   if (FindFunction(theDeffunctionName) != NULL)
     {
      PrintErrorID("DFFNXPSR",2,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Deffunctions are not allowed to replace external functions.\n");
      return(CLIPS_FALSE);
     }

#if DEFGENERIC_CONSTRUCT
   /* ============================================
      A deffunction cannot be named the same as a
      generic function (either in this module or
      imported from another)
      ============================================ */
   theDefgeneric = 
     (struct constructHeader *) LookupDefgenericInScope(theDeffunctionName);
   if (theDefgeneric != NULL)
     {
      theModule = GetConstructModuleItem(theDefgeneric)->theModule;
      if (theModule != ((struct defmodule *) GetCurrentModule()))
        { 
         PrintErrorID("DFFNXPSR",5,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Defgeneric ");
         PrintCLIPS(WERROR,GetDefgenericName((VOID *) theDefgeneric));
         PrintCLIPS(WERROR," imported from module ");
         PrintCLIPS(WERROR,GetDefmoduleName((VOID *) theModule));
         PrintCLIPS(WERROR," conflicts with this deffunction.\n");
         return(CLIPS_FALSE);
        }
      else
        {
         PrintErrorID("DFFNXPSR",3,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Deffunctions are not allowed to replace generic functions.\n");
        }
      return(CLIPS_FALSE);
     }
#endif

   theDeffunction = (struct constructHeader *) FindDeffunction(theDeffunctionName);
   if (theDeffunction != NULL)
     {
      /* ===========================================
         And a deffunction in the current module can
         only be redefined if it is not executing.
         =========================================== */
      if (((DEFFUNCTION *) theDeffunction)->executing)
        {
         PrintErrorID("DFNXPSR",4,CLIPS_FALSE);
         PrintCLIPS(WERROR,"Deffunction ");
         PrintCLIPS(WERROR,GetDeffunctionName((VOID *) theDeffunction));
         PrintCLIPS(WERROR," may not be redefined while it is executing.\n");
         return(CLIPS_FALSE);
        }
     }
   return(CLIPS_TRUE);
  }


/****************************************************
  NAME         : AddDeffunction
  DESCRIPTION  : Adds a deffunction to the list of
                 deffunctions
  INPUTS       : 1) The symbolic name
                 2) The action expressions
                 3) The minimum number of arguments
                 4) The maximum number of arguments
                    (can be -1)
                 5) The number of local variables
                 6) A flag indicating if this is
                    a header call so that the
                    deffunction can be recursively
                    called
  RETURNS      : The new deffunction (NULL on errors)
  SIDE EFFECTS : Deffunction structures allocated
  NOTES        : Assumes deffunction is not executing
 ****************************************************/
static DEFFUNCTION *AddDeffunction(name,actions,min,max,lvars,headerp)
  SYMBOL_HN *name;
  EXPRESSION *actions;
  int min,max,lvars,headerp;
  {
   DEFFUNCTION *dfuncPtr;
   int oldbusy;
#if DEBUGGING_FUNCTIONS
   int DFHadWatch = CLIPS_FALSE;
#endif
   
   /*===============================================================*/
   /* If the deffunction doesn't exist, create a new structure to   */
   /* contain it and add it to the List of deffunctions. Otherwise, */
   /* use the existing structure and remove the pretty print form   */
   /* and interpretive code.                                        */
   /*===============================================================*/
   dfuncPtr = (DEFFUNCTION *) FindDeffunction(ValueToString(name));
   if (dfuncPtr == NULL)
     {
      dfuncPtr = get_struct(deffunctionStruct);
      InitializeConstructHeader("deffunction",(struct constructHeader *) dfuncPtr,name);
      IncrementSymbolCount(name);
      dfuncPtr->code = NULL;
      dfuncPtr->minNumberOfParameters = min;
      dfuncPtr->maxNumberOfParameters = max;
      dfuncPtr->numberOfLocalVars = lvars;
      dfuncPtr->busy = 0;
      dfuncPtr->executing = 0;
     }
   else
     {
#if DEBUGGING_FUNCTIONS
      DFHadWatch = GetDeffunctionWatch((VOID *) dfuncPtr);
#endif
      dfuncPtr->minNumberOfParameters = min;
      dfuncPtr->maxNumberOfParameters = max;
      dfuncPtr->numberOfLocalVars = lvars;
      oldbusy = dfuncPtr->busy;
      ExpressionDeinstall(dfuncPtr->code);
      dfuncPtr->busy = oldbusy;
      ReturnPackedExpression(dfuncPtr->code);
      dfuncPtr->code = NULL;
      SetDeffunctionPPForm((VOID *) dfuncPtr,NULL);
      
      /* =======================================
         Remove the deffunction from the list so
         that it can be added at the end
         ======================================= */
      RemoveConstructFromModule((struct constructHeader *) dfuncPtr);
     }

   AddConstructToModule((struct constructHeader *) dfuncPtr);

   /* ==================================
      Install the new interpretive code.
      ================================== */

   if (actions != NULL)
     {
      /* ===============================
         If a deffunction is recursive,
         do not increment its busy count
         based on self-references
         =============================== */
      oldbusy = dfuncPtr->busy;
      ExpressionInstall(actions);
      dfuncPtr->busy = oldbusy;
      dfuncPtr->code = actions;
     }

   /* ===============================================================
      Install the pretty print form if memory is not being conserved.
      =============================================================== */

#if DEBUGGING_FUNCTIONS
   SetDeffunctionWatch(DFHadWatch ? CLIPS_TRUE : WatchDeffunctions,(VOID *) dfuncPtr);
   if ((GetConserveMemory() == CLIPS_FALSE) && (headerp == CLIPS_FALSE))
     SetDeffunctionPPForm((VOID *) dfuncPtr,CopyPPBuffer());
#endif
   return(dfuncPtr);
  }

#endif /* DEFFUNCTION_CONSTRUCT && (! BLOAD_ONLY) && (! RUN_TIME) */

/***************************************************
  NAME         : 
  DESCRIPTION  : 
  INPUTS       : 
  RETURNS      : 
  SIDE EFFECTS : 
  NOTES        : 
 ***************************************************/
