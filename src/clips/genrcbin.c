   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                                                     */
   /*******************************************************/

/*************************************************************/
/* Purpose: Binary Load/Save Functions for Generic Functions */
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

#if DEFGENERIC_CONSTRUCT && (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE)

#include "constant.h"
#include "clipsmem.h"
#include "bload.h"
#include "bsave.h"

#include "cstrcbin.h"

#if OBJECT_SYSTEM
#include "objbin.h"
#endif

#include "genrccom.h"
#include "modulbin.h"

#define _GENRCBIN_SOURCE_
#include "genrcbin.h"

#include "router.h"

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
   
#define MethodPointer(i) (((i) == -1L) ? NULL : (DEFMETHOD *) &methodArray[i])
#define RestrictionPointer(i) (((i) == -1L) ? NULL : (RESTRICTION *) &restrictionArray[i])
#define TypePointer(i) (((i) == -1L) ? NULL : (VOID **) &typeArray[i])

typedef struct bsaveRestriction
  {
   long types,query;
   unsigned tcnt;
  } BSAVE_RESTRICTION;

typedef struct bsaveMethod
  {
   unsigned index;
   int restrictionCount,
       minRestrictions,maxRestrictions,
       localVarCount;
   int system;
   long restrictions,actions;
  } BSAVE_METHOD;

typedef struct bsaveGenericFunc
  {
   struct bsaveConstructHeader header;
   long methods;
   unsigned mcnt;
  } BSAVE_GENERIC;

typedef struct bsaveGenericModule
  {
   struct bsaveDefmoduleItemHeader header;
  } BSAVE_DEFGENERIC_MODULE;

/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER

#if BLOAD_AND_BSAVE

static VOID BsaveGenericsFind(void);
static VOID MarkDefgenericItems(struct constructHeader *,VOID *);
static VOID BsaveGenericsExpressions(FILE *);
static VOID BsaveMethodExpressions(struct constructHeader *,VOID *);
static VOID BsaveRestrictionExpressions(struct constructHeader *,VOID *);
static VOID BsaveGenerics(FILE *);
static VOID BsaveDefgenericHeader(struct constructHeader *,VOID *);
static VOID BsaveMethods(struct constructHeader *,VOID *);
static VOID BsaveMethodRestrictions(struct constructHeader *,VOID *);
static VOID BsaveRestrictionTypes(struct constructHeader *,VOID *);
static VOID BsaveStorageGenerics(FILE *);

#endif

static VOID BloadStorageGenerics(void);
static VOID BloadGenerics(void);
static VOID UpdateGenericModule(VOID *,long);
static VOID UpdateGeneric(VOID *,long);
static VOID UpdateMethod(VOID *,long);
static VOID UpdateRestriction(VOID *,long);
static VOID UpdateType(VOID *,long);
static VOID ClearBloadGenerics(void);

#else

#if BLOAD_AND_BSAVE

static VOID BsaveGenericsFind();
static VOID MarkDefgenericItems();
static VOID BsaveGenericsExpressions();
static VOID BsaveMethodExpressions();
static VOID BsaveRestrictionExpressions();
static VOID BsaveGenerics();
static VOID BsaveDefgenericHeader();
static VOID BsaveMethods();
static VOID BsaveMethodRestrictions();
static VOID BsaveRestrictionTypes();
static VOID BsaveStorageGenerics();

#endif

static VOID BloadStorageGenerics();
static VOID BloadGenerics();
static VOID UpdateGenericModule();
static VOID UpdateGeneric();
static VOID UpdateMethod();
static VOID UpdateRestriction();
static VOID UpdateType();
static VOID ClearBloadGenerics();

#endif
      
/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */

globle DEFGENERIC HUGE_ADDR *defgenericArray = NULL;

/* =========================================
   *****************************************
      INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */

static long ModuleCount = 0L,
            GenericCount = 0L,
            MethodCount = 0L,
            RestrictionCount = 0L,
            TypeCount = 0L;   

static DEFGENERIC_MODULE HUGE_ADDR *ModuleArray = NULL;
static DEFMETHOD HUGE_ADDR *methodArray = NULL;
static RESTRICTION HUGE_ADDR *restrictionArray = NULL;
static VOID * HUGE_ADDR *typeArray = NULL;

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/***********************************************************
  NAME         : SetupGenericsBload
  DESCRIPTION  : Initializes data structures and
                   routines for binary loads of
                   generic function constructs
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Routines defined and structures initialized
  NOTES        : None
 ***********************************************************/
globle VOID SetupGenericsBload()
  {
#if BLOAD_AND_BSAVE
   AddBinaryItem("generic functions",0,BsaveGenericsFind,BsaveGenericsExpressions,
                             BsaveStorageGenerics,BsaveGenerics,
                             BloadStorageGenerics,BloadGenerics,
                             ClearBloadGenerics);
#endif
#if BLOAD || BLOAD_ONLY
   AddBinaryItem("generic functions",0,NULL,NULL,NULL,NULL,
                             BloadStorageGenerics,BloadGenerics,
                             ClearBloadGenerics);
#endif
  }
   
/***************************************************
  NAME         : BloadDefgenericModuleReference
  DESCRIPTION  : Returns a pointer to the
                 appropriate defgeneric module
  INPUTS       : The index of the module
  RETURNS      : A pointer to the module
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle VOID *BloadDefgenericModuleReference(index)
  int index;
  {
   return ((VOID *) &ModuleArray[index]);
  }

/* =========================================
   *****************************************
          INTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */

#if BLOAD_AND_BSAVE

/***************************************************************************
  NAME         : BsaveGenericsFind
  DESCRIPTION  : For all generic functions and their
                   methods, this routine marks all 
                   the needed symbols and system functions.
                 Also, it also counts the number of
                   expression structures needed.
                 Also, counts total number of generics, methods,
                   restrictions and types.
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : ExpressionCount (a global from BSAVE.C) is incremented
                   for every expression needed
                 Symbols and system function are marked in their structures
  NOTES        : Also sets bsaveIndex for each generic function (assumes
                   generic functions will be bsaved in order of binary list)
 ***************************************************************************/
static VOID BsaveGenericsFind()
  {
   if (Bloaded())
     {
      SaveBloadCount(ModuleCount);
      SaveBloadCount(GenericCount);
      SaveBloadCount(MethodCount);
      SaveBloadCount(RestrictionCount);
      SaveBloadCount(TypeCount);
     }
   GenericCount = 0L;
   MethodCount = 0L;
   RestrictionCount = 0L;
   TypeCount = 0L;
   
   ModuleCount = DoForAllConstructs(MarkDefgenericItems,DefgenericModuleIndex,
                                    CLIPS_FALSE,NULL);
  }
  
/***************************************************
  NAME         : MarkDefgenericItems
  DESCRIPTION  : Marks the needed items for
                 a defgeneric (and methods) bsave
  INPUTS       : 1) The defgeneric
                 2) User data buffer (ignored)
  RETURNS      : Nothing useful
  SIDE EFFECTS : Needed items marked
  NOTES        : None
 ***************************************************/
#if IBM_TBC
#pragma argsused
#endif
static VOID MarkDefgenericItems(theDefgeneric,userBuffer)
  struct constructHeader *theDefgeneric;
  VOID *userBuffer;
  {
#if MAC_MPW
#pragma unused(userBuffer)
#endif
   DEFGENERIC *gfunc = (DEFGENERIC *) theDefgeneric;
   register unsigned i,j;
   DEFMETHOD *meth;
   RESTRICTION *rptr;
   
   MarkConstructHeaderNeededItems(&gfunc->header,GenericCount++);
   MethodCount += gfunc->mcnt;
   for (i = 0 ; i < gfunc->mcnt ; i++)
     {
      meth = &gfunc->methods[i];
      ExpressionCount += ExpressionSize(meth->actions);
      MarkNeededItems(meth->actions);
      RestrictionCount += meth->restrictionCount;
      for (j = 0 ; j < meth->restrictionCount ; j++)
        {
         rptr = &meth->restrictions[j];
         ExpressionCount += ExpressionSize(rptr->query);
         MarkNeededItems(rptr->query);
         TypeCount += rptr->tcnt;
        }
     }
  }

/***************************************************
  NAME         : BsaveGenericsExpressions
  DESCRIPTION  : Writes out all expressions needed
                   by generic functions
  INPUTS       : The file pointer of the binary file
  RETURNS      : Nothing useful
  SIDE EFFECTS : File updated
  NOTES        : None
 ***************************************************/
static VOID BsaveGenericsExpressions(fp)
  FILE *fp;
  {
   /* ================================================================
      Important to save all expressions for methods before any
      expressions for restrictions, since methods will be stored first
      ================================================================ */
   DoForAllConstructs(BsaveMethodExpressions,DefgenericModuleIndex,
                      CLIPS_FALSE,(VOID *) fp);
                      
   DoForAllConstructs(BsaveRestrictionExpressions,DefgenericModuleIndex,
                      CLIPS_FALSE,(VOID *) fp);
  }

/***************************************************
  NAME         : BsaveMethodExpressions
  DESCRIPTION  : Saves the needed expressions for
                 a defgeneric methods bsave
  INPUTS       : 1) The defgeneric
                 2) Output data file pointer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Method action expressions saved
  NOTES        : None
 ***************************************************/
static VOID BsaveMethodExpressions(theDefgeneric,userBuffer)
  struct constructHeader *theDefgeneric;
  VOID *userBuffer;
  {
   DEFGENERIC *gfunc = (DEFGENERIC *) theDefgeneric;
   register unsigned i;
   
   for (i = 0 ; i < gfunc->mcnt ; i++)
     BsaveExpression(gfunc->methods[i].actions,(FILE *) userBuffer);
  }

/***************************************************
  NAME         : BsaveRestrictionExpressions
  DESCRIPTION  : Saves the needed expressions for
                 a defgeneric method restriction
                 queries bsave
  INPUTS       : 1) The defgeneric
                 2) Output data file pointer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Method restriction query
                 expressions saved
  NOTES        : None
 ***************************************************/
static VOID BsaveRestrictionExpressions(theDefgeneric,userBuffer)
  struct constructHeader *theDefgeneric;
  VOID *userBuffer;
  {
   DEFGENERIC *gfunc = (DEFGENERIC *) theDefgeneric;
   register unsigned i,j;
   DEFMETHOD *meth;
   
   for (i = 0 ; i < gfunc->mcnt ; i++)
     {
      meth = &gfunc->methods[i];
      for (j = 0 ; j < meth->restrictionCount ; j++)
        BsaveExpression(meth->restrictions[j].query,(FILE *) userBuffer);
     }
  }

/***********************************************************
  NAME         : BsaveStorageGenerics
  DESCRIPTION  : Writes out number of each type of structure
                   required for generics
                 Space required for counts (unsigned long)
  INPUTS       : File pointer of binary file
  RETURNS      : Nothing useful
  SIDE EFFECTS : Binary file adjusted
  NOTES        : None
 ***********************************************************/
static VOID BsaveStorageGenerics(fp)
  FILE *fp;
  {
   unsigned long space;

   space = sizeof(long) * 5;
   GenWrite((VOID *) &space,(unsigned long) sizeof(long),fp);
   GenWrite((VOID *) &ModuleCount,(unsigned long) sizeof(long),fp);
   GenWrite((VOID *) &GenericCount,(unsigned long) sizeof(long),fp);
   GenWrite((VOID *) &MethodCount,(unsigned long) sizeof(long),fp);
   GenWrite((VOID *) &RestrictionCount,(unsigned long) sizeof(long),fp);
   GenWrite((VOID *) &TypeCount,(unsigned long) sizeof(long),fp);
  }
  
/****************************************************************************************
  NAME         : BsaveGenerics
  DESCRIPTION  : Writes out generic function in binary format
                 Space required (unsigned long)
                 All generic modules (sizeof(DEFGENERIC_MODULE) * Number of generic modules)
                 All generic headers (sizeof(DEFGENERIC) * Number of generics)
                 All methods (sizeof(DEFMETHOD) * Number of methods)
                 All method restrictions (sizeof(RESTRICTION) * Number of restrictions)
                 All restriction type arrays (sizeof(VOID *) * # of types)
  INPUTS       : File pointer of binary file
  RETURNS      : Nothing useful
  SIDE EFFECTS : Binary file adjusted
  NOTES        : None
 ****************************************************************************************/
static VOID BsaveGenerics(fp)
  FILE *fp;
  {
   struct defmodule *theModule;
   DEFGENERIC_MODULE *theModuleItem;
   unsigned long space;
   BSAVE_DEFGENERIC_MODULE dummy_generic_module;

   /* =====================================================================
      Space is: Sum over all structures(sizeof(structure) * structure-cnt))
      ===================================================================== */     
   space = ((unsigned long) ModuleCount * (unsigned long) sizeof(BSAVE_DEFGENERIC_MODULE)) +
           ((unsigned long) GenericCount * (unsigned long) sizeof(BSAVE_GENERIC)) +
           ((unsigned long) MethodCount * (unsigned long) sizeof(BSAVE_METHOD)) +
           ((unsigned long) RestrictionCount * (unsigned long) sizeof(BSAVE_RESTRICTION)) +
           ((unsigned long) TypeCount * (unsigned long) sizeof(unsigned long));
                   
   /* ================================================================
      Write out the total amount of space required:  modules,headers,
      methods, restrictions, types
      ================================================================ */         
   GenWrite((VOID *) &space,(unsigned long) sizeof(unsigned long),fp);
   
   /* ======================================
      Write out the generic function modules
      ====================================== */
   GenericCount = 0L;
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      theModuleItem = (DEFGENERIC_MODULE *) 
                      GetModuleItem(theModule,FindModuleItem("defgeneric")->moduleIndex);
      AssignBsaveDefmdlItemHdrVals(&dummy_generic_module.header,
                                           &theModuleItem->header);
      GenWrite((VOID *) &dummy_generic_module,
               (unsigned long) sizeof(BSAVE_DEFGENERIC_MODULE),fp);
      theModule = (struct defmodule *) GetNextDefmodule((VOID *) theModule);
     }
     
   
   /* ======================================
      Write out the generic function headers
      ====================================== */
   MethodCount = 0L;
   DoForAllConstructs(BsaveDefgenericHeader,DefgenericModuleIndex,
                      CLIPS_FALSE,(VOID *) fp);
     
   /* =====================
      Write out the methods
      ===================== */
   RestrictionCount = 0L;
   DoForAllConstructs(BsaveMethods,DefgenericModuleIndex,
                      CLIPS_FALSE,(VOID *) fp);
   
   /* =================================
      Write out the method restrictions
      ================================= */
   TypeCount = 0L;
   DoForAllConstructs(BsaveMethodRestrictions,DefgenericModuleIndex,
                      CLIPS_FALSE,(VOID *) fp);
     
   /* =============================================================
      Finally, write out the type lists for the method restrictions
      ============================================================= */
   DoForAllConstructs(BsaveRestrictionTypes,DefgenericModuleIndex,
                      CLIPS_FALSE,(VOID *) fp);

   if (Bloaded())
     {
      RestoreBloadCount(&ModuleCount);
      RestoreBloadCount(&GenericCount);
      RestoreBloadCount(&MethodCount);
      RestoreBloadCount(&RestrictionCount);
      RestoreBloadCount(&TypeCount);
     }
  }

/***************************************************
  NAME         : BsaveDefgenericHeader
  DESCRIPTION  : Bsaves a generic function header
  INPUTS       : 1) The defgeneric
                 2) Output data file pointer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Defgeneric header saved
  NOTES        : None
 ***************************************************/
static VOID BsaveDefgenericHeader(theDefgeneric,userBuffer)
  struct constructHeader *theDefgeneric;
  VOID *userBuffer;
  {
   DEFGENERIC *gfunc = (DEFGENERIC *) theDefgeneric;
   BSAVE_GENERIC dummy_generic;
   
   AssignBsaveConstructHeaderVals(&dummy_generic.header,&gfunc->header);
   dummy_generic.mcnt = gfunc->mcnt;
   if (gfunc->methods != NULL)
     {
      dummy_generic.methods = MethodCount;
      MethodCount += gfunc->mcnt;
     }
   else
     dummy_generic.methods = -1L;
   GenWrite((VOID *) &dummy_generic,(unsigned long) sizeof(BSAVE_GENERIC),(FILE *) userBuffer);
  }

/***************************************************
  NAME         : BsaveMethods
  DESCRIPTION  : Bsaves defgeneric methods
  INPUTS       : 1) The defgeneric
                 2) Output data file pointer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Defgeneric methods saved
  NOTES        : None
 ***************************************************/
static VOID BsaveMethods(theDefgeneric,userBuffer)
  struct constructHeader *theDefgeneric;
  VOID *userBuffer;
  {
   DEFGENERIC *gfunc = (DEFGENERIC *) theDefgeneric;
   DEFMETHOD *meth;
   BSAVE_METHOD dummy_method;
   register unsigned i;
   
   for (i = 0 ; i < gfunc->mcnt ; i++)
     {
      meth = &gfunc->methods[i];
      dummy_method.index = meth->index;
      dummy_method.restrictionCount = meth->restrictionCount;
      dummy_method.minRestrictions = meth->minRestrictions;
      dummy_method.maxRestrictions = meth->maxRestrictions;
      dummy_method.localVarCount = meth->localVarCount;
      dummy_method.system = meth->system;
      if (meth->restrictions != NULL)
        {
         dummy_method.restrictions = RestrictionCount;
         RestrictionCount += meth->restrictionCount;
        }
      else
        dummy_method.restrictions = -1L;
      if (meth->actions != NULL)
        {
         dummy_method.actions = ExpressionCount;
         ExpressionCount += ExpressionSize(meth->actions);
        }
      else
        dummy_method.actions = -1L;
      GenWrite((VOID *) &dummy_method,(unsigned long) sizeof(BSAVE_METHOD),(FILE *) userBuffer);
     }
  }
  
/******************************************************
  NAME         : BsaveMethodRestrictions
  DESCRIPTION  : Bsaves defgeneric methods' retrictions
  INPUTS       : 1) The defgeneric
                 2) Output data file pointer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Defgeneric methods' restrictions saved
  NOTES        : None
 ******************************************************/
static VOID BsaveMethodRestrictions(theDefgeneric,userBuffer)
  struct constructHeader *theDefgeneric;
  VOID *userBuffer;
  {
   DEFGENERIC *gfunc = (DEFGENERIC *) theDefgeneric;
   BSAVE_RESTRICTION dummy_restriction;
   RESTRICTION *rptr;
   register unsigned i,j;

   for (i = 0 ; i < gfunc->mcnt ; i++)
     {
      for (j = 0 ; j < gfunc->methods[i].restrictionCount ; j++)
        {
         rptr = &gfunc->methods[i].restrictions[j];
         dummy_restriction.tcnt = rptr->tcnt;
         if (rptr->types != NULL)
           {
            dummy_restriction.types = TypeCount;
            TypeCount += rptr->tcnt;
           }
         else
           dummy_restriction.types = -1L;
         if (rptr->query != NULL)
           {
            dummy_restriction.query = ExpressionCount;
            ExpressionCount += ExpressionSize(rptr->query);
           }
         else
           dummy_restriction.query = -1L;
         GenWrite((VOID *) &dummy_restriction,
                  (unsigned long) sizeof(BSAVE_RESTRICTION),(FILE *) userBuffer);
        }
     }
  }
  
/*************************************************************
  NAME         : BsaveRestrictionTypes
  DESCRIPTION  : Bsaves defgeneric methods' retrictions' types
  INPUTS       : 1) The defgeneric
                 2) Output data file pointer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Defgeneric methods' restrictions' types saved
  NOTES        : None
 *************************************************************/
static VOID BsaveRestrictionTypes(theDefgeneric,userBuffer)
  struct constructHeader *theDefgeneric;
  VOID *userBuffer;
  {
   DEFGENERIC *gfunc = (DEFGENERIC *) theDefgeneric;
   long dummy_type;
   RESTRICTION *rptr;
   register unsigned i,j,k;

   for (i = 0 ; i < gfunc->mcnt ; i++)
     {
      for (j = 0 ; j < gfunc->methods[i].restrictionCount ; j++)
        {
         rptr = &gfunc->methods[i].restrictions[j];
         for (k = 0 ; k < rptr->tcnt ; k++)
           {
#if OBJECT_SYSTEM
            dummy_type = DefclassIndex(rptr->types[k]);
#else
            dummy_type = (long) ((INTEGER_HN *) rptr->types[k])->contents;
#endif
            GenWrite(&dummy_type,(unsigned long) sizeof(long),(FILE *) userBuffer);
           }
        }
     }
  }
  
#endif

/***********************************************************************
  NAME         : BloadStorageGenerics
  DESCRIPTION  : This routine space required for generic function
                   structures and allocates space for them
  INPUTS       : Nothing
  RETURNS      : Nothing useful
  SIDE EFFECTS : Arrays allocated and set
  NOTES        : This routine makes no attempt to reset any pointers
                   within the structures
 ***********************************************************************/
static VOID BloadStorageGenerics()
  {
   unsigned long space;
   long counts[5];
   
   GenRead((VOID *) &space,(unsigned long) sizeof(unsigned long));
   if (space == 0L)
     return;
   GenRead((VOID *) counts,space);
   ModuleCount = counts[0];
   GenericCount = counts[1];
   MethodCount = counts[2];
   RestrictionCount = counts[3];
   TypeCount = counts[4];
   if (ModuleCount != 0L)
     {
      space = (unsigned long) (sizeof(DEFGENERIC_MODULE) * ModuleCount);
      ModuleArray = (DEFGENERIC_MODULE HUGE_ADDR *) genlongalloc(space);
     }
   else
     return;
   if (GenericCount != 0L)
     {
      space = (unsigned long) (sizeof(DEFGENERIC) * GenericCount);
      defgenericArray = (DEFGENERIC HUGE_ADDR *) genlongalloc(space);
     }
   else
     return;
   if (MethodCount != 0L)
     {
      space = (unsigned long) (sizeof(DEFMETHOD) * MethodCount);
      methodArray = (DEFMETHOD HUGE_ADDR *) genlongalloc(space);
     }
   else
     return;
   if (RestrictionCount != 0L)
     {
      space = (unsigned long) (sizeof(RESTRICTION) * RestrictionCount);
      restrictionArray = (RESTRICTION HUGE_ADDR *) genlongalloc(space);
     }
   else
     return;
   if (TypeCount != 0L)
     {
      space = (unsigned long) (sizeof(VOID *) * TypeCount);
      typeArray = (VOID * HUGE_ADDR *) genlongalloc(space);
     }
  }

/*********************************************************************
  NAME         : BloadGenerics
  DESCRIPTION  : This routine reads generic function information from
                 a binary file in four chunks:
                 Generic-header array
                 Method array
                 Method restrictions array
                 Restriction types array
                 
                 This routine moves through the generic function
                   binary arrays updating pointers
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Pointers reset from array indices
  NOTES        : Assumes all loading is finished
 ********************************************************************/
static VOID BloadGenerics()
  {
   unsigned long space;
   
   GenRead((VOID *) &space,(unsigned long) sizeof(unsigned long));
   if (ModuleCount == 0L)
     return;
   BloadandRefresh(ModuleCount,(unsigned) sizeof(BSAVE_DEFGENERIC_MODULE),UpdateGenericModule);
   if (GenericCount == 0L)
     return;
   BloadandRefresh(GenericCount,(unsigned) sizeof(BSAVE_GENERIC),UpdateGeneric);
   BloadandRefresh(MethodCount,(unsigned) sizeof(BSAVE_METHOD),UpdateMethod);
   BloadandRefresh(RestrictionCount,(unsigned) sizeof(BSAVE_RESTRICTION),UpdateRestriction);
   BloadandRefresh(TypeCount,(unsigned) sizeof(long),UpdateType);
  }

/*********************************************
  Bload update routines for generic structures
 *********************************************/
static VOID UpdateGenericModule(buf,obji)
  VOID *buf;
  long obji;
  {
   BSAVE_DEFGENERIC_MODULE *bdptr;
   
   bdptr = (BSAVE_DEFGENERIC_MODULE *) buf;
   UpdateDefmoduleItemHeader(&bdptr->header,&ModuleArray[obji].header,
                             (int) sizeof(DEFGENERIC),(VOID *) defgenericArray);
  }
  
static VOID UpdateGeneric(buf,obji)
  VOID *buf;
  long obji;
  {
   BSAVE_GENERIC *bgp;
   DEFGENERIC *gp;
   
   bgp = (BSAVE_GENERIC *) buf;
   gp = (DEFGENERIC *) &defgenericArray[obji];
   
   UpdateConstructHeader(&bgp->header,&gp->header,
                         (int) sizeof(DEFGENERIC_MODULE),(VOID *) ModuleArray,
                         (int) sizeof(DEFGENERIC),(VOID *) defgenericArray);
   defgenericArray[obji].busy = 0;
#if DEBUGGING_FUNCTIONS
   defgenericArray[obji].trace = WatchGenerics;
#endif
   defgenericArray[obji].methods = MethodPointer(bgp->methods);
   defgenericArray[obji].mcnt = bgp->mcnt;
   defgenericArray[obji].new_index = 0;
  }

static VOID UpdateMethod(buf,obji)
  VOID *buf;
  long obji;
  {
   BSAVE_METHOD *bmth;
   
   bmth = (BSAVE_METHOD *) buf;
   methodArray[obji].index = bmth->index;
   methodArray[obji].busy = 0;
#if DEBUGGING_FUNCTIONS
   methodArray[obji].trace = WatchMethods;
#endif
   methodArray[obji].restrictionCount = bmth->restrictionCount;
   methodArray[obji].minRestrictions = bmth->minRestrictions;
   methodArray[obji].maxRestrictions = bmth->maxRestrictions;
   methodArray[obji].localVarCount = bmth->localVarCount;
   methodArray[obji].system = bmth->system;
   methodArray[obji].restrictions = RestrictionPointer(bmth->restrictions);
   methodArray[obji].actions = ExpressionPointer(bmth->actions);
   methodArray[obji].ppForm = NULL;
  }

static VOID UpdateRestriction(buf,obji)
  VOID *buf;
  long obji;
  {
   BSAVE_RESTRICTION *brp;
   
   brp = (BSAVE_RESTRICTION *) buf;
   restrictionArray[obji].tcnt = brp->tcnt;
   restrictionArray[obji].types = TypePointer(brp->types);
   restrictionArray[obji].query = ExpressionPointer(brp->query);
  }

static VOID UpdateType(buf,obji)
  VOID *buf;
  long obji;
  {
#if OBJECT_SYSTEM
   typeArray[obji] = (VOID *) DefclassPointer(* (long *) buf);
#else
   if ((* (long *) buf) > (long) INSTANCE_TYPE_CODE)
     {
      PrintWarningID("GENRCBIN",1,CLIPS_FALSE);
      PrintCLIPS(WWARNING,"COOL not installed!  User-defined class\n");
      PrintCLIPS(WWARNING,"  in method restriction substituted with OBJECT.\n");
      typeArray[obji] = (VOID *) AddLong((long) OBJECT_TYPE_CODE);
     }
   else
     typeArray[obji] = (VOID *) AddLong(* (long *) buf);
   IncrementIntegerCount((INTEGER_HN *) typeArray[obji]);
#endif
  }

/***************************************************************
  NAME         : ClearBloadGenerics
  DESCRIPTION  : Release all binary-loaded generic function
                   structure arrays
                 Resets generic function list to NULL
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Memory cleared
  NOTES        : Generic function name symbol counts decremented
 ***************************************************************/
static VOID ClearBloadGenerics()
  {
   register long i;
   unsigned long space;
   
   space = (unsigned long) (sizeof(DEFGENERIC_MODULE) * ModuleCount);
   if (space == 0L)
     return;
   genlongfree((VOID *) ModuleArray,space);
   ModuleArray = NULL;
   ModuleCount = 0L;

   for (i = 0 ; i < GenericCount ; i++)
     UnmarkConstructHeader(&defgenericArray[i].header);
     
   space = (unsigned long) (sizeof(DEFGENERIC) * GenericCount);
   if (space == 0L)
     return;
   genlongfree((VOID *) defgenericArray,space);
   defgenericArray = NULL;
   GenericCount = 0L;
   
   space = (unsigned long) (sizeof(DEFMETHOD) * MethodCount);
   if (space == 0L)
     return;
   genlongfree((VOID *) methodArray,space);
   methodArray = NULL;
   MethodCount = 0L;
   
   space = (unsigned long) (sizeof(RESTRICTION) * RestrictionCount);
   if (space == 0L)
     return;
   genlongfree((VOID *) restrictionArray,space);
   restrictionArray = NULL;
   RestrictionCount = 0L;

#if ! OBJECT_SYSTEM
   for (i = 0 ; i < TypeCount ; i++)
     DecrementIntegerCount((INTEGER_HN *) typeArray[i]);
#endif
   space = (unsigned long) (sizeof(VOID *) * TypeCount);
   if (space == 0L)
     return;
   genlongfree((VOID *) typeArray,space);
   typeArray = NULL;
   TypeCount = 0L;
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
