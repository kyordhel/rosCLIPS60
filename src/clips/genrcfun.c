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
/* Purpose: CLIPS Generic Functions Internal Routines        */
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

#if BLOAD || BLOAD_AND_BSAVE
#include "bload.h"
#endif

#if OBJECT_SYSTEM
#include "classcom.h"
#include "classfun.h"
#endif

#include "argacces.h"
#include "clipsmem.h"
#include "constrct.h"
#include "cstrcpsr.h"
#include "genrccom.h"
#include "genrcexe.h"
#include "prccode.h"
#include "router.h"

#define _GENRCFUN_SOURCE_
#include "genrcfun.h"

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

#if DEBUGGING_FUNCTIONS
static VOID DisplayGenericCore(DEFGENERIC *);
#endif

#else

#if DEBUGGING_FUNCTIONS
static VOID DisplayGenericCore();
#endif

#endif
      

/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
globle DEFGENERIC *CurrentGeneric = NULL;
globle DEFMETHOD *CurrentMethod = NULL;
globle DATA_OBJECT *GenericCurrentArgument = NULL;

#if DEBUGGING_FUNCTIONS
globle int WatchGenerics = OFF;
globle int WatchMethods = OFF;
#endif

#if (! RUN_TIME) && (! BLOAD_ONLY)
globle int OldGenericBusySave;
#endif

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
  
#if ! RUN_TIME

/***************************************************
  NAME         : ClearDefgenericsReady
  DESCRIPTION  : Determines if it is safe to
                 remove all defgenerics
                 Assumes *all* constructs will be
                 deleted - only checks to see if
                 any methods are currently
                 executing
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if no methods are
                 executing, CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : Used by (clear) and (bload)
 ***************************************************/
globle BOOLEAN ClearDefgenericsReady()
  {
   return((CurrentGeneric != NULL) ? CLIPS_FALSE : CLIPS_TRUE);
  }

/*****************************************************
  NAME         : AllocateDefgenericModule
  DESCRIPTION  : Creates and initializes a
                 list of defgenerics for a new module
  INPUTS       : None
  RETURNS      : The new deffunction module
  SIDE EFFECTS : Deffunction module created
  NOTES        : None
 *****************************************************/
globle VOID *AllocateDefgenericModule()
  {
   return((VOID *) get_struct(defgenericModule));
  } 

/***************************************************
  NAME         : FreeDefgenericModule
  DESCRIPTION  : Removes a deffunction module and
                 all associated deffunctions
  INPUTS       : The deffunction module
  RETURNS      : Nothing useful
  SIDE EFFECTS : Module and deffunctions deleted
  NOTES        : None
 ***************************************************/
globle VOID FreeDefgenericModule(theItem)
  VOID *theItem;
  {
#if (! BLOAD_ONLY)
   FreeConstructHeaderModule((struct defmoduleItemHeader *) theItem,DefgenericConstruct);
#endif
   rtn_struct(defgenericModule,theItem);
  } 

#endif

#if (! BLOAD_ONLY) && (! RUN_TIME)

/************************************************************
  NAME         : ClearDefmethods
  DESCRIPTION  : Deletes all defmethods - generic headers
                   are left intact
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if all methods deleted, CLIPS_FALSE otherwise
  SIDE EFFECTS : Defmethods deleted
  NOTES        : Clearing generic functions is done in
                   two stages
                   
                 1) Delete all methods (to clear any
                    references to other constructs)
                 2) Delete all generic headers
                 
                 This allows other constructs which
                   mutually refer to generic functions
                   to be cleared
 ************************************************************/
globle int ClearDefmethods()
  {
   register DEFGENERIC *gfunc;
   int success = CLIPS_TRUE;
   
#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded() == CLIPS_TRUE) return(CLIPS_FALSE);
#endif

   gfunc = (DEFGENERIC *) GetNextDefgeneric(NULL);
   while (gfunc != NULL)
     {
      if (RemoveAllExplicitMethods(gfunc) == CLIPS_FALSE)
        success = CLIPS_FALSE;
      gfunc = (DEFGENERIC *) GetNextDefgeneric((VOID *) gfunc);
     }
   return(success);
  }

/*****************************************************************
  NAME         : RemoveAllExplicitMethods
  DESCRIPTION  : Deletes all explicit defmethods - generic headers
                   are left intact (as well as a method for an
                   overloaded system function)
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if all methods deleted, CLIPS_FALSE otherwise
  SIDE EFFECTS : Explicit defmethods deleted
  NOTES        : None
 *****************************************************************/
globle int RemoveAllExplicitMethods(gfunc)
  DEFGENERIC *gfunc;
  {
   register int i,j;
   unsigned systemMethodCount = 0;
   DEFMETHOD *narr;
   
   if (MethodsExecuting(gfunc) == CLIPS_FALSE)
     {
      for (i = 0 ; i < gfunc->mcnt ; i++)
        {
         if (gfunc->methods[i].system)
           systemMethodCount++;
         else
           DeleteMethodInfo(gfunc,&gfunc->methods[i]);
        }
      if (systemMethodCount != 0)
        {
         narr = (DEFMETHOD *) gm2((int) (systemMethodCount * sizeof(DEFMETHOD)));
         i = 0;
         j = 0;
         while (i < gfunc->mcnt)
           {
            if (gfunc->methods[i].system)
              CopyMemory(DEFMETHOD,1,&narr[j++],&gfunc->methods[i]);
            i++;
           }
         rm((VOID *) gfunc->methods,(int) (sizeof(DEFMETHOD) * gfunc->mcnt));
         gfunc->mcnt = systemMethodCount;
         gfunc->methods = narr;
        }
      else
        {
         if (gfunc->mcnt != 0)
           rm((VOID *) gfunc->methods,(int) (sizeof(DEFMETHOD) * gfunc->mcnt));
         gfunc->mcnt = 0;
         gfunc->methods = NULL;
        }
      return(CLIPS_TRUE);
     }
   return(CLIPS_FALSE);
  }
  
/**************************************************
  NAME         : RemoveDefgeneric
  DESCRIPTION  : Removes a generic function node
                   from the generic list along with
                   all its methods
  INPUTS       : The generic function
  RETURNS      : Nothing useful
  SIDE EFFECTS : List adjusted
                 Nodes deallocated
  NOTES        : Assumes generic is not in use!!!
 **************************************************/
globle VOID RemoveDefgeneric(vgfunc)
  VOID *vgfunc;
  {
   DEFGENERIC *gfunc = (DEFGENERIC *) vgfunc;
   register int i;
   
   for (i = 0 ; i < gfunc->mcnt ; i++)
     DeleteMethodInfo(gfunc,&gfunc->methods[i]);
   
   if (gfunc->mcnt != 0)
     rm((VOID *) gfunc->methods,(int) (sizeof(DEFMETHOD) * gfunc->mcnt));
   DecrementSymbolCount(GetDefgenericNamePointer((VOID *) gfunc));
   SetDefgenericPPForm((VOID *) gfunc,NULL);
   rtn_struct(defgeneric,gfunc);
  }
  
/****************************************************************
  NAME         : ClearDefgenerics
  DESCRIPTION  : Deletes all generic headers
  INPUTS       : None
  RETURNS      : CLIPS_TRUE if all methods deleted, CLIPS_FALSE otherwise
  SIDE EFFECTS : Generic headers deleted (and any implicit system
                  function methods)
  NOTES        : None
 ****************************************************************/
globle int ClearDefgenerics()
  {
   register DEFGENERIC *gfunc,*gtmp;
   int success = CLIPS_TRUE;
   
#if BLOAD || BLOAD_AND_BSAVE
   if (Bloaded() == CLIPS_TRUE) return(CLIPS_FALSE);
#endif

   gfunc = (DEFGENERIC *) GetNextDefgeneric(NULL);
   while (gfunc != NULL)
     {
      gtmp = gfunc;
      gfunc = (DEFGENERIC *) GetNextDefgeneric((VOID *) gfunc);
      if (RemoveAllExplicitMethods(gtmp) == CLIPS_FALSE)
        {
         CantDeleteItemErrorMessage("generic function",GetDefgenericName(gtmp));
         success = CLIPS_FALSE;
        }
      else
        {
         RemoveConstructFromModule((struct constructHeader *) gtmp);
         RemoveDefgeneric((VOID *) gtmp);
        }
     }
   return(success);
  }
  
/********************************************************
  NAME         : MethodAlterError
  DESCRIPTION  : Prints out an error message reflecting
                   that a generic function's methods
                   cannot be altered while any of them
                   are executing
  INPUTS       : The generic function
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : None
 ********************************************************/
globle VOID MethodAlterError(gfunc)
  DEFGENERIC *gfunc;
  {
   PrintErrorID("GENRCFUN",1,CLIPS_FALSE);
   PrintCLIPS(WERROR,"Defgeneric ");
   PrintCLIPS(WERROR,GetDefgenericName((VOID *) gfunc));
   PrintCLIPS(WERROR," cannot be modified while one of its methods is executing.\n");
  }
  
/***************************************************
  NAME         : DeleteMethodInfo
  DESCRIPTION  : Deallocates all the data associated
                  w/ a method but does not release
                  the method structure itself
  INPUTS       : 1) The generic function address
                 2) The method address
  RETURNS      : Nothing useful
  SIDE EFFECTS : Nodes deallocated
  NOTES        : None
 ***************************************************/
globle VOID DeleteMethodInfo(gfunc,meth)
  DEFGENERIC *gfunc;
  DEFMETHOD *meth;
  {
   register int j,k;
   register RESTRICTION *rptr;
   
   SaveBusyCount(gfunc);
   ExpressionDeinstall(meth->actions);
   ReturnPackedExpression(meth->actions);
   if (meth->ppForm != NULL)
     rm((VOID *) meth->ppForm,(int) (sizeof(char) * (strlen(meth->ppForm)+1)));
   for (j = 0 ; j < meth->restrictionCount ; j++)
     {
      rptr = &meth->restrictions[j];
      
      for (k = 0 ; k < rptr->tcnt ; k++)
#if OBJECT_SYSTEM
        DecrementDefclassBusyCount(rptr->types[k]);
#else
        DecrementIntegerCount((INTEGER_HN *) rptr->types[k]);
#endif

      if (rptr->types != NULL)
        rm((VOID *) rptr->types,(int) (sizeof(VOID *) * rptr->tcnt));
      ExpressionDeinstall(rptr->query);
      ReturnPackedExpression(rptr->query);
     }
   if (meth->restrictions != NULL)
     rm((VOID *) meth->restrictions,
        (int) (sizeof(RESTRICTION) * meth->restrictionCount));
   RestoreBusyCount(gfunc);
  }
  
/***************************************************
  NAME         : MethodsExecuting
  DESCRIPTION  : Determines if any of the methods of
                   a generic function are currently
                   executing
  INPUTS       : The generic function address
  RETURNS      : CLIPS_TRUE if any methods are executing,
                   CLIPS_FALSE otherwise
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle int MethodsExecuting(gfunc)
  DEFGENERIC *gfunc;
  {
   register unsigned i;
   
   for (i = 0 ; i < gfunc->mcnt ; i++)
     if (gfunc->methods[i].busy > 0)
       return(CLIPS_TRUE);
   return(CLIPS_FALSE);
  }
  
#if ! OBJECT_SYSTEM

/**************************************************************
  NAME         : SubsumeType
  DESCRIPTION  : Determines if the second type subsumes
                 the first type
                 (e.g. INTEGER is subsumed by NUMBER_TYPE_CODE)
  INPUTS       : Two type codes
  RETURNS      : CLIPS_TRUE if type 2 subsumes type 1, CLIPS_FALSE
                 otherwise
  SIDE EFFECTS : None
  NOTES        : Used only when COOL is not present
 **************************************************************/
globle BOOLEAN SubsumeType(t1,t2)
  int t1,t2;
  {
   if ((t2 == OBJECT_TYPE_CODE) || (t2 == PRIMITIVE_TYPE_CODE))
     return(CLIPS_TRUE);
   if ((t2 == NUMBER_TYPE_CODE) && ((t1 == INTEGER) || (t1 == FLOAT)))
     return(CLIPS_TRUE);
   if ((t2 == LEXEME_TYPE_CODE) && ((t1 == STRING) || (t1 == SYMBOL)))
     return(CLIPS_TRUE);
   if ((t2 == ADDRESS_TYPE_CODE) && ((t1 == EXTERNAL_ADDRESS) || 
       (t1 == FACT_ADDRESS) || (t1 == INSTANCE_ADDRESS)))
     return(CLIPS_TRUE);
   if ((t2 == LEXEME_TYPE_CODE) &&
       ((t1 == INSTANCE_NAME) || (t1 == INSTANCE_ADDRESS)))
     return(CLIPS_TRUE);
   return(CLIPS_FALSE);
  }

#endif

#endif

/*****************************************************
  NAME         : FindMethodByIndex
  DESCRIPTION  : Finds a generic function method of
                   specified index
  INPUTS       : 1) The generic function
                 2) The index
  RETURNS      : The position of the method in the
                   generic function's method array,
                   -1 if not found
  SIDE EFFECTS : None
  NOTES        : None
 *****************************************************/
globle int FindMethodByIndex(gfunc,index)
  DEFGENERIC *gfunc;
  unsigned index;
  {
   register unsigned i;
   
   for (i = 0 ; i < gfunc->mcnt ; i++)
     if (gfunc->methods[i].index == index)
       return(i);
   return(-1);
  }
  
#if DEBUGGING_FUNCTIONS

/*************************************************************
  NAME         : PreviewGeneric
  DESCRIPTION  : Allows the user to see a printout of all the
                   applicable methods for a particular generic
                   function call
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Any side-effects of evaluating the generic
                   function arguments
                 and evaluating query-functions to determine
                   the set of applicable methods
  NOTES        : CLIPS Syntax: (preview-generic <func> <args>)
 *************************************************************/
globle VOID PreviewGeneric()
  {
   DEFGENERIC *gfunc;
   DEFGENERIC *previousGeneric;
   int oldce;
   DATA_OBJECT temp;
   
   EvaluationError = CLIPS_FALSE;
   if (ArgTypeCheck("preview-generic",1,SYMBOL,&temp) == CLIPS_FALSE)
     return;
   gfunc = LookupDefgenericByMdlOrScope(DOToString(temp));
   if (gfunc == NULL)
     {
      PrintErrorID("GENRCFUN",3,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Unable to find generic function ");
      PrintCLIPS(WERROR,DOToString(temp));
      PrintCLIPS(WERROR," in function preview-generic.\n");
      return;
     }
   oldce = ExecutingConstruct();
   SetExecutingConstruct(CLIPS_TRUE);
   previousGeneric = CurrentGeneric;
   CurrentGeneric = gfunc;
   CurrentEvaluationDepth++;
   PushProcParameters(GetFirstArgument()->nextArg,
                          CountArguments(GetFirstArgument()->nextArg),
                          GetDefgenericName((VOID *) gfunc),"generic function",
                          UnboundMethodErr);
   if (EvaluationError) 
     {
      PopProcParameters();
      CurrentGeneric = previousGeneric;
      CurrentEvaluationDepth--;
      SetExecutingConstruct(oldce);
      return;
     }
   gfunc->busy++;
   DisplayGenericCore(gfunc);
   gfunc->busy--;
   PopProcParameters();
   CurrentGeneric = previousGeneric;
   CurrentEvaluationDepth--;
   SetExecutingConstruct(oldce);
  }

/******************************************************************
  NAME         : PrintMethod
  DESCRIPTION  : Lists a brief description of methods for a method
  INPUTS       : 1) Buffer for method info
                 2) Size of buffer (not including space for '\0')
                 3) The method address
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : A terminating newline is NOT included
 ******************************************************************/
globle VOID PrintMethod(buf,buflen,meth)
  char *buf;
  int buflen;
  DEFMETHOD *meth;
  {
   register int j,k;
   register RESTRICTION *rptr;
   char numbuf[15];
   
   buf[0] = '\0';
   if (meth->system)
     strncpy(buf,"SYS",buflen);
   sprintf(numbuf,"%-2d ",meth->index);
   strncat(buf,numbuf,buflen-3);
   for (j = 0 ; j < meth->restrictionCount ; j++)
     {
      rptr = &meth->restrictions[j];
      if ((j == meth->restrictionCount-1) && (meth->maxRestrictions == -1))
        {
         if ((rptr->tcnt == 0) && (rptr->query == NULL))
           {
            strncat(buf,"$?",buflen-strlen(buf));
            break;
           }
         strncat(buf,"($? ",buflen-strlen(buf));
        }
      else
        strncat(buf,"(",buflen-strlen(buf));
      for (k = 0 ; k < rptr->tcnt ; k++)
        {
#if OBJECT_SYSTEM
         strncat(buf,GetDefclassName(rptr->types[k]),buflen-strlen(buf));
#else
         strncat(buf,TypeName(ValueToInteger(rptr->types[k])),buflen-strlen(buf));
#endif
         if (k < (rptr->tcnt - 1))
           strncat(buf," ",buflen-strlen(buf));
        }
      if (rptr->query != NULL)
        {
         if (rptr->tcnt != 0)
           strncat(buf," ",buflen-strlen(buf));
         strncat(buf,"<qry>",buflen-strlen(buf));
        }
      strncat(buf,")",buflen-strlen(buf));
      if (j != (meth->restrictionCount-1))
        strncat(buf," ",buflen-strlen(buf));
     }
  }
    
#endif
 
/***************************************************
  NAME         : CheckGenericExists
  DESCRIPTION  : Finds the address of named
                  generic function and prints out
                  error message if not found
  INPUTS       : 1) Calling function
                 2) Name of generic function
  RETURNS      : Generic function address (NULL if
                   not found)
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle DEFGENERIC *CheckGenericExists(fname,gname)
  char *fname,*gname;
  {
   DEFGENERIC *gfunc;

   gfunc = LookupDefgenericByMdlOrScope(gname);
   if (gfunc == NULL)
     {
      PrintErrorID("GENRCFUN",3,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Unable to find generic function ");
      PrintCLIPS(WERROR,gname);
      PrintCLIPS(WERROR," in function ");
      PrintCLIPS(WERROR,fname);
      PrintCLIPS(WERROR,".\n");
      SetEvaluationError(CLIPS_TRUE);
     }
   return(gfunc); 
  }

/***************************************************
  NAME         : CheckMethodExists
  DESCRIPTION  : Finds the array index of the
                  specified method and prints out
                  error message if not found
  INPUTS       : 1) Calling function
                 2) Generic function address
                 3) Index of method
  RETURNS      : Method array index (-1 if not found)
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle int CheckMethodExists(fname,gfunc,mi)
  char *fname;
  DEFGENERIC *gfunc;
  int mi;
  {
   int fi;

   fi = FindMethodByIndex(gfunc,(unsigned) mi);
   if (fi == -1)
     {
      PrintErrorID("GENRCFUN",2,CLIPS_FALSE);
      PrintCLIPS(WERROR,"Unable to find method ");
      PrintCLIPS(WERROR,GetDefgenericName((VOID *) gfunc));
      PrintCLIPS(WERROR," #");
      PrintLongInteger(WERROR,(long) mi);
      PrintCLIPS(WERROR," in function ");
      PrintCLIPS(WERROR,fname);
      PrintCLIPS(WERROR,".\n");
      SetEvaluationError(CLIPS_TRUE);
     }
   return(fi); 
  }

#if ! OBJECT_SYSTEM

/*******************************************************
  NAME         : TypeName
  DESCRIPTION  : Given an integer type code, this
                 function returns the string name of
                 the type
  INPUTS       : The type code
  RETURNS      : The name-string of the type, or 
                 "<???UNKNOWN-TYPE???>" for unrecognized
                 types
  SIDE EFFECTS : EvaluationError set and error message
                 printed for unrecognized types
  NOTES        : Used only when COOL is not present
 *******************************************************/
globle char *TypeName(tcode)
  int tcode;
  {
   switch (tcode)
     {
      case INTEGER             : return(INTEGER_TYPE_NAME);
      case FLOAT               : return(FLOAT_TYPE_NAME);
      case SYMBOL              : return(SYMBOL_TYPE_NAME);
      case STRING              : return(STRING_TYPE_NAME);
      case MULTIFIELD          : return(MULTIFIELD_TYPE_NAME);
      case EXTERNAL_ADDRESS    : return(EXTERNAL_ADDRESS_TYPE_NAME);
      case FACT_ADDRESS        : return(FACT_ADDRESS_TYPE_NAME);
      case INSTANCE_ADDRESS    : return(INSTANCE_ADDRESS_TYPE_NAME);
      case INSTANCE_NAME       : return(INSTANCE_NAME_TYPE_NAME);
      case OBJECT_TYPE_CODE    : return(OBJECT_TYPE_NAME);
      case PRIMITIVE_TYPE_CODE : return(PRIMITIVE_TYPE_NAME);
      case NUMBER_TYPE_CODE    : return(NUMBER_TYPE_NAME);
      case LEXEME_TYPE_CODE    : return(LEXEME_TYPE_NAME);
      case ADDRESS_TYPE_CODE   : return(ADDRESS_TYPE_NAME);
      case INSTANCE_TYPE_CODE  : return(INSTANCE_TYPE_NAME);
      default                  : PrintErrorID("INSCOM",1,CLIPS_FALSE);
                                 PrintCLIPS(WERROR,"Undefined type in function type.\n");
                                 SetEvaluationError(CLIPS_TRUE);
                                 return("<???UNKNOWN-TYPE???>");
     }
  }

#endif

/******************************************************
  NAME         : PrintGenericName
  DESCRIPTION  : Prints the name of a gneric function
                 (including the module name if the
                  generic is not in the current module)
  INPUTS       : 1) The logical name of the output
                 2) The generic functions
  RETURNS      : Nothing useful
  SIDE EFFECTS : Generic name printed
  NOTES        : None
 ******************************************************/
globle VOID PrintGenericName(log,gfunc)
  char *log;
  DEFGENERIC *gfunc;
  {
   if (gfunc->header.whichModule->theModule != ((struct defmodule *) GetCurrentModule()))
     {
      PrintCLIPS(log,GetDefmoduleName((VOID *) 
                        gfunc->header.whichModule->theModule));
      PrintCLIPS(log,"::");
     }
   PrintCLIPS(log,ValueToString((VOID *) gfunc->header.name));
  }

/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
#if DEBUGGING_FUNCTIONS

/*********************************************************
  NAME         : DisplayGenericCore
  DESCRIPTION  : Prints out a description of a core
                   frame of applicable methods for
                   a particular call of a generic function
  INPUTS       : The generic function
  RETURNS      : Nothing useful
  SIDE EFFECTS : None
  NOTES        : None
 *********************************************************/
static VOID DisplayGenericCore(gfunc)
  DEFGENERIC *gfunc;
  {
   register int i;
   char buf[256];
   int rtn = CLIPS_FALSE;
   
   for (i = 0 ; i < gfunc->mcnt ; i++)
     {
      gfunc->methods[i].busy++;
      if (IsMethodApplicable(&gfunc->methods[i]))
        {
         rtn = CLIPS_TRUE;
         PrintCLIPS(WDISPLAY,GetDefgenericName((VOID *) gfunc));
         PrintCLIPS(WDISPLAY," #");
         PrintMethod(buf,255,&gfunc->methods[i]);
         PrintCLIPS(WDISPLAY,buf);
         PrintCLIPS(WDISPLAY,"\n");
#if ! IMPERATIVE_METHODS
         break;
#endif
        }
      gfunc->methods[i].busy--;
     }
   if (rtn == CLIPS_FALSE)
     {
      PrintCLIPS(WDISPLAY,"No applicable methods for ");
      PrintCLIPS(WDISPLAY,GetDefgenericName((VOID *) gfunc));
      PrintCLIPS(WDISPLAY,".\n");
     }
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
