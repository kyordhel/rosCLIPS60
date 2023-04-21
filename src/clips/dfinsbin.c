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
/* Purpose: Binary Load/Save Functions for Definstances      */
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

#if DEFINSTANCES_CONSTRUCT && (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE)

#include "bload.h"
#include "bsave.h"
#include "clipsmem.h"
#include "cstrcbin.h"
#include "defins.h"
#include "modulbin.h"

#define _DFINSBIN_SOURCE_
#include "dfinsbin.h"

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
typedef struct bsaveDefinstancesModule
  {
   struct bsaveDefmoduleItemHeader header;
  } BSAVE_DEFINSTANCES_MODULE;

typedef struct bsaveDefinstances
  {
   struct bsaveConstructHeader header;
   long mkinstance;
  } BSAVE_DEFINSTANCES;
     
/* =========================================
   *****************************************
      INTERNALLY VISIBLE FUNCTION HEADERS
   =========================================
   ***************************************** */
#if ANSI_COMPILER

#if BLOAD_AND_BSAVE
static VOID BsaveDefinstancesFind(void);
static VOID MarkDefinstancesItems(struct constructHeader *,VOID *);
static VOID BsaveDefinstancesExpressions(FILE *);
static VOID BsaveDefinstancesExpression(struct constructHeader *,VOID *);
static VOID BsaveStorageDefinstances(FILE *);
static VOID BsaveDefinstancesDriver(FILE *);
static VOID BsaveDefinstances(struct constructHeader *,VOID *);
#endif

static VOID BloadStorageDefinstances(void);
static VOID BloadDefinstances(void);
static VOID UpdateDefinstancesModule(VOID *,long);
static VOID UpdateDefinstances(VOID *,long);
static VOID ClearDefinstancesBload(void);

#else

#if BLOAD_AND_BSAVE
static VOID BsaveDefinstancesFind();
static VOID MarkDefinstancesItems();
static VOID BsaveDefinstancesExpressions();
static VOID BsaveDefinstancesExpression();
static VOID BsaveStorageDefinstances();
static VOID BsaveDefinstancesDriver();
static VOID BsaveDefinstances();
#endif

static VOID BloadStorageDefinstances();
static VOID BloadDefinstances();
static VOID UpdateDefinstancesModule();
static VOID UpdateDefinstances();
static VOID ClearDefinstancesBload();

#endif
      
/* =========================================
   *****************************************
      EXTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
globle DEFINSTANCES HUGE_ADDR *definstancesArray = NULL;

/* =========================================
   *****************************************
      INTERNALLY VISIBLE GLOBAL VARIABLES
   =========================================
   ***************************************** */
static long DefinstancesCount = 0L,
            ModuleCount = 0L;
static DEFINSTANCES_MODULE HUGE_ADDR *ModuleArray;

/* =========================================
   *****************************************
          EXTERNALLY VISIBLE FUNCTIONS
   =========================================
   ***************************************** */
   
/***********************************************************
  NAME         : SetupDefinstancesBload
  DESCRIPTION  : Initializes data structures and
                   routines for binary loads of definstances
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Routines defined and structures initialized
  NOTES        : None
 ***********************************************************/
globle VOID SetupDefinstancesBload()
  {
#if BLOAD_AND_BSAVE
   AddBinaryItem("definstances",0,BsaveDefinstancesFind,BsaveDefinstancesExpressions,
                             BsaveStorageDefinstances,BsaveDefinstancesDriver,
                             BloadStorageDefinstances,BloadDefinstances,
                             ClearDefinstancesBload);
#else
   AddBinaryItem("definstances",0,NULL,NULL,NULL,NULL,
                             BloadStorageDefinstances,BloadDefinstances,
                             ClearDefinstancesBload);
#endif
  }
   
/***************************************************
  NAME         : BloadDefinstancesModuleRef
  DESCRIPTION  : Returns a pointer to the
                 appropriate definstances module
  INPUTS       : The index of the module
  RETURNS      : A pointer to the module
  SIDE EFFECTS : None
  NOTES        : None
 ***************************************************/
globle VOID *BloadDefinstancesModuleRef(index)
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
  NAME         : BsaveDefinstancesFind
  DESCRIPTION  : For all definstances, this routine marks all 
                   the needed symbols.
                 Also, it also counts the number of
                   expression structures needed.
                 Also, counts total number of definstances.
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : ExpressionCount (a global from BSAVE.C) is incremented
                   for every expression needed
                 Symbols are marked in their structures
  NOTES        : Also sets bsaveIndex for each definstances (assumes
                   definstances will be bsaved in order of binary list)
 ***************************************************************************/
static VOID BsaveDefinstancesFind()
  {
   if (Bloaded())
     {
      SaveBloadCount(ModuleCount);
      SaveBloadCount(DefinstancesCount);
     }
   DefinstancesCount = 0L;
   
   ModuleCount = DoForAllConstructs(MarkDefinstancesItems,DefinstancesModuleIndex,
                                    CLIPS_FALSE,NULL);
  }
  

/***************************************************
  NAME         : MarkDefinstancesItems
  DESCRIPTION  : Marks the needed items for
                 a definstances bsave
  INPUTS       : 1) The definstances
                 2) User data buffer (ignored)
  RETURNS      : Nothing useful
  SIDE EFFECTS : Needed items marked
  NOTES        : None
 ***************************************************/
#if IBM_TBC
#pragma argsused
#endif
static VOID MarkDefinstancesItems(theDefinstances,userBuffer)
  struct constructHeader *theDefinstances;
  VOID *userBuffer;
  {
#if MAC_MPW
#pragma unused(userBuffer)
#endif
   MarkConstructHeaderNeededItems(theDefinstances,DefinstancesCount++);
   ExpressionCount += ExpressionSize(((DEFINSTANCES *) theDefinstances)->mkinstance);
   MarkNeededItems(((DEFINSTANCES *) theDefinstances)->mkinstance);
  }

/***************************************************
  NAME         : BsaveDefinstancesExpressions
  DESCRIPTION  : Writes out all expressions needed
                   by deffunctyions
  INPUTS       : The file pointer of the binary file
  RETURNS      : Nothing useful
  SIDE EFFECTS : File updated
  NOTES        : None
 ***************************************************/
static VOID BsaveDefinstancesExpressions(fp)
  FILE *fp;
  {
   DoForAllConstructs(BsaveDefinstancesExpression,DefinstancesModuleIndex,
                      CLIPS_FALSE,(VOID *) fp);
  }
  
/***************************************************
  NAME         : BsaveDefinstancesExpression
  DESCRIPTION  : Saves the needed expressions for
                 a definstances bsave
  INPUTS       : 1) The definstances
                 2) Output data file pointer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Expressions saved
  NOTES        : None
 ***************************************************/
static VOID BsaveDefinstancesExpression(theDefinstances,userBuffer)
  struct constructHeader *theDefinstances;
  VOID *userBuffer;
  {
   BsaveExpression(((DEFINSTANCES *) theDefinstances)->mkinstance,(FILE *) userBuffer);
  }

/***********************************************************
  NAME         : BsaveStorageDefinstances
  DESCRIPTION  : Writes out number of each type of structure
                   required for definstances
                 Space required for counts (unsigned long)
  INPUTS       : File pointer of binary file
  RETURNS      : Nothing useful
  SIDE EFFECTS : Binary file adjusted
  NOTES        : None
 ***********************************************************/
static VOID BsaveStorageDefinstances(fp)
  FILE *fp;
  {
   unsigned long space;
   
   space = sizeof(unsigned long) * 2;
   GenWrite((VOID *) &space,(unsigned long) sizeof(unsigned long),fp);
   GenWrite((VOID *) &ModuleCount,(unsigned long) sizeof(long),fp);
   GenWrite((VOID *) &DefinstancesCount,(unsigned long) sizeof(long),fp);
  }
  
/*************************************************************************************
  NAME         : BsaveDefinstancesDriver
  DESCRIPTION  : Writes out definstances in binary format
                 Space required (unsigned long)
                 All definstances (sizeof(DEFINSTANCES) * Number of definstances)
  INPUTS       : File pointer of binary file
  RETURNS      : Nothing useful
  SIDE EFFECTS : Binary file adjusted
  NOTES        : None
 *************************************************************************************/
static VOID BsaveDefinstancesDriver(fp)
  FILE *fp;
  {
   unsigned long space;
   struct defmodule *theModule;
   DEFINSTANCES_MODULE *theModuleItem;
   BSAVE_DEFINSTANCES_MODULE dummy_mitem;

   space = (unsigned long) ((sizeof(BSAVE_DEFINSTANCES_MODULE) * ModuleCount) +
                            (sizeof(BSAVE_DEFINSTANCES) * DefinstancesCount));
   GenWrite((VOID *) &space,(unsigned long) sizeof(unsigned long),fp);
   
   /* =================================
      Write out each definstances module
      ================================= */
   DefinstancesCount = 0L;
   theModule = (struct defmodule *) GetNextDefmodule(NULL);
   while (theModule != NULL)
     {
      theModuleItem = (DEFINSTANCES_MODULE *) 
                      GetModuleItem(theModule,FindModuleItem("definstances")->moduleIndex);
      AssignBsaveDefmdlItemHdrVals(&dummy_mitem.header,&theModuleItem->header);
      GenWrite((VOID *) &dummy_mitem,(unsigned long) sizeof(BSAVE_DEFINSTANCES_MODULE),fp);
      theModule = (struct defmodule *) GetNextDefmodule((VOID *) theModule);
     }
     
   /* ==========================
      Write out each definstances
      ========================== */
   DoForAllConstructs(BsaveDefinstances,DefinstancesModuleIndex,
                      CLIPS_FALSE,(VOID *) fp);
                      
   if (Bloaded())
     {
      RestoreBloadCount(&ModuleCount);
      RestoreBloadCount(&DefinstancesCount);
     }
  }

/***************************************************
  NAME         : BsaveDefinstances
  DESCRIPTION  : Bsaves a definstances
  INPUTS       : 1) The definstances
                 2) Output data file pointer
  RETURNS      : Nothing useful
  SIDE EFFECTS : Definstances saved
  NOTES        : None
 ***************************************************/
static VOID BsaveDefinstances(theDefinstances,userBuffer)
  struct constructHeader *theDefinstances;
  VOID *userBuffer;
  {
   DEFINSTANCES *dptr = (DEFINSTANCES *) theDefinstances;
   BSAVE_DEFINSTANCES dummy_df;
   
   AssignBsaveConstructHeaderVals(&dummy_df.header,&dptr->header);
  if (dptr->mkinstance != NULL)
     {
      dummy_df.mkinstance = ExpressionCount;
      ExpressionCount += ExpressionSize(dptr->mkinstance);
     }
   else
    dummy_df.mkinstance = -1L;
   GenWrite((VOID *) &dummy_df,(unsigned long) sizeof(BSAVE_DEFINSTANCES),(FILE *) userBuffer);
  }

#endif

/***********************************************************************
  NAME         : BloadStorageDefinstances
  DESCRIPTION  : This routine space required for definstances
                   structures and allocates space for them
  INPUTS       : Nothing
  RETURNS      : Nothing useful
  SIDE EFFECTS : Arrays allocated and set
  NOTES        : This routine makes no attempt to reset any pointers
                   within the structures
 ***********************************************************************/
static VOID BloadStorageDefinstances()
  {
   unsigned long space;

   GenRead((VOID *) &space,(unsigned long) sizeof(unsigned long));
   if (space == 0L)
     return;
   GenRead((VOID *) &ModuleCount,(unsigned long) sizeof(unsigned long));
   GenRead((VOID *) &DefinstancesCount,(unsigned long) sizeof(unsigned long));
   if (ModuleCount == 0L)
     {
      ModuleArray = NULL;
      definstancesArray = NULL;
      return;
     }
     
   space = (unsigned long) (ModuleCount * sizeof(DEFINSTANCES_MODULE));
   ModuleArray = (DEFINSTANCES_MODULE HUGE_ADDR *) genlongalloc(space);

   if (DefinstancesCount == 0L)
     {
      definstancesArray = NULL;
      return;
     }
     
   space = (unsigned long) (DefinstancesCount * sizeof(DEFINSTANCES));
   definstancesArray = (DEFINSTANCES HUGE_ADDR *) genlongalloc(space);
  }

/*********************************************************************
  NAME         : BloadDefinstances
  DESCRIPTION  : This routine reads definstances information from
                   a binary file
                 This routine moves through the definstances
                   binary array updating pointers
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Pointers reset from array indices
  NOTES        : Assumes all loading is finished
 ********************************************************************/
static VOID BloadDefinstances()
  {
   unsigned long space;
   
   GenRead((VOID *) &space,(unsigned long) sizeof(unsigned long));
   BloadandRefresh(ModuleCount,sizeof(BSAVE_DEFINSTANCES_MODULE),UpdateDefinstancesModule);
   BloadandRefresh(DefinstancesCount,sizeof(BSAVE_DEFINSTANCES),UpdateDefinstances);
  }

/*******************************************************
  NAME         : UpdateDefinstancesModule
  DESCRIPTION  : Updates definstances module with binary
                 load data - sets pointers from
                 offset information
  INPUTS       : 1) A pointer to the bloaded data
                 2) The index of the binary array
                    element to update
  RETURNS      : Nothing useful
  SIDE EFFECTS : Definstances moudle pointers updated
  NOTES        : None
 *******************************************************/
static VOID UpdateDefinstancesModule(buf,obji)
  VOID *buf;
  long obji;
  {
   BSAVE_DEFINSTANCES_MODULE *bdptr;
   
   bdptr = (BSAVE_DEFINSTANCES_MODULE *) buf;
   UpdateDefmoduleItemHeader(&bdptr->header,&ModuleArray[obji].header,
                             (int) sizeof(DEFINSTANCES),(VOID *) definstancesArray);
  }

/***************************************************
  NAME         : UpdateDefinstances
  DESCRIPTION  : Updates definstances with binary
                 load data - sets pointers from
                 offset information
  INPUTS       : 1) A pointer to the bloaded data
                 2) The index of the binary array
                    element to update
  RETURNS      : Nothing useful
  SIDE EFFECTS : Definstances pointers upadted
  NOTES        : None
 ***************************************************/
static VOID UpdateDefinstances(buf,obji)
  VOID *buf;
  long obji;
  {
   BSAVE_DEFINSTANCES *bdptr;
   DEFINSTANCES *dfiptr;
   
   bdptr = (BSAVE_DEFINSTANCES *) buf;
   dfiptr = (DEFINSTANCES *) &definstancesArray[obji];
   
   UpdateConstructHeader(&bdptr->header,&dfiptr->header,
                         (int) sizeof(DEFINSTANCES_MODULE),(VOID *) ModuleArray,
                         (int) sizeof(DEFINSTANCES),(VOID *) definstancesArray);
   dfiptr->mkinstance = ExpressionPointer(bdptr->mkinstance);
   dfiptr->busy = 0;
  }

/***************************************************************
  NAME         : ClearDefinstancesBload
  DESCRIPTION  : Release all binary-loaded definstances
                   structure arrays
                 Resets definstances list to NULL
  INPUTS       : None
  RETURNS      : Nothing useful
  SIDE EFFECTS : Memory cleared
  NOTES        : Definstances name symbol counts decremented
 ***************************************************************/
static VOID ClearDefinstancesBload()
  {
   register long i;
   unsigned long space;

   space = (unsigned long) (sizeof(DEFINSTANCES_MODULE) * ModuleCount);
   if (space == 0L)
     return;
   genlongfree((VOID *) ModuleArray,space);
   ModuleArray = NULL;
   ModuleCount = 0L;

   for (i = 0L ; i < DefinstancesCount ; i++)
     UnmarkConstructHeader(&definstancesArray[i].header);
   space = (unsigned long) (sizeof(DEFINSTANCES) * DefinstancesCount);
   if (space == 0L)
     return;
   genlongfree((VOID *) definstancesArray,space);
   definstancesArray = NULL;
   DefinstancesCount = 0L;
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
