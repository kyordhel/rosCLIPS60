   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*             LOGICAL DEPENDENCIES MODULE             */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _LGCLDPND_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_

#include "setup.h"

#if DEFRULE_CONSTRUCT && LOGICAL_DEPENDENCIES

#include "clipsmem.h"
#include "router.h"
#include "evaluatn.h"
#include "engine.h"
#include "reteutil.h"
#include "pattern.h"
#include "argacces.h"
#include "factmngr.h"

#if OBJECT_SYSTEM
#include "insfun.h"
#endif

#include "lgcldpnd.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static struct partialMatch    *FindLogicalBind(struct joinNode *,struct partialMatch *);
   static int                     FindEntityInPartialMatch(struct patternEntity *,struct partialMatch *);
   static struct dependency      *DetachAssociatedDependencies(struct dependency *,VOID *);
#if DEBUGGING_FUNCTIONS
   static VOID                   *GetFactOrInstanceArgument(DATA_OBJECT *,char *);
#endif
#else
   static struct partialMatch    *FindLogicalBind();
   static int                     FindEntityInPartialMatch();
   static struct dependency      *DetachAssociatedDependencies();
#if DEBUGGING_FUNCTIONS
   static VOID                   *GetFactOrInstanceArgument();
#endif
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static struct dependency     *DependencyList = NULL;

/***********************************************************************/
/* AddLogicalDependencies: Adds the logical dependency links between a */
/*   fact and the partial match which logically supports that fact. If */
/*   a fact is unconditionally asserted (i.e. the global variable      */
/*   TheLogicalJoin is NULL), then existing logical support for the    */
/*   fact is no longer needed and it is removed. If a fact is already  */
/*   unconditionally supported and the fact is conditionally asserted  */
/*   (i.e. the global variable TheLogicalJoin is not NULL), then the   */
/*   logical support is ignored. Otherwise, the partial match is       */
/*   linked to the fact and the fact is linked to the partial match.   */
/***********************************************************************/
globle BOOLEAN AddLogicalDependencies(theEntity,existingEntity) 
  struct patternEntity *theEntity;
  int existingEntity;
  {
   struct partialMatch *theBinds;
   struct dependency *newDependency;

   /*==============================================*/
   /* If the rule has no logical patterns, then no */
   /* dependencies have to be established.         */
   /*==============================================*/

   if (TheLogicalJoin == NULL)
     {
      if (existingEntity) RemoveEntityDependencies(theEntity);
      return(CLIPS_TRUE);
     }
   else if (existingEntity && (theEntity->dependents == NULL))
     { return(CLIPS_TRUE); }

   /*=====================================================*/
   /* Find the binds in the logical join associated with  */
   /* activation binds. If the binds cannot be found,     */
   /* then the binds must have been deleted by a previous */
   /* RHS action and the fact should not be asserted.     */
   /*=====================================================*/

   theBinds = FindLogicalBind(TheLogicalJoin,GlobalLHSBinds);
   if (theBinds == NULL) return(CLIPS_FALSE);

   /*==============================================================*/
   /* Add a dependency link between the partialMatch and the data  */
   /* entity. The dependency links are stored in the partial match */
   /* behind the data entities stored in the partial match and the */
   /* activation link, if any.                                     */
   /*==============================================================*/

   newDependency = get_struct(dependency);
   newDependency->dPtr = (VOID *) theEntity;
   newDependency->next = (struct dependency *) 
                         theBinds->binds[theBinds->bcount + theBinds->activationf].gm.theValue;
   theBinds->binds[theBinds->bcount + theBinds->activationf].gm.theValue = (VOID *) newDependency;
   
   /*================================================================*/
   /* Add a dependency link between the entity and the partialMatch. */
   /*================================================================*/
   
   newDependency = get_struct(dependency);
   newDependency->dPtr = (VOID *) theBinds;
   newDependency->next = (struct dependency *) theEntity->dependents;
   theEntity->dependents = (VOID *) newDependency;

   /*========================================================*/
   /* Return true to indicate that the fact can be asserted. */
   /*========================================================*/

   return(TRUE);
  }

/************************************************************************/
/* FindLogicalBind: Finds the partial match associated with the logical */
/*   CE which will provide logical support for a fact asserted from the */
/*   currently executing rule. The function is called when creating     */
/*   logical support links between the facts and supporting partial     */
/*   matches. It compares each partial match found at a specified join  */
/*   to the partial match associated with a rule activation until it    */
/*   finds the partial match that generated the rule activation.        */
/************************************************************************/
static struct partialMatch *FindLogicalBind(theJoin,theBinds) 
  struct joinNode *theJoin;
  struct partialMatch *theBinds;
  {
   struct partialMatch *compPtr;
   unsigned int max, i;
   int found;

   compPtr = theJoin->beta;
   while (compPtr != NULL)
     {
      max = compPtr->bcount;
      found = CLIPS_TRUE;

      for (i = 0; i < max; i++)
        {
         if (compPtr->binds[i].gm.theMatch != theBinds->binds[i].gm.theMatch) 
           {
            i = max;
            found = CLIPS_FALSE;
           }
        }

      if (found) return(compPtr);
      compPtr = compPtr->next;
     }

   return(NULL);
  }

/*********************************************************************/
/* RemoveEntityDependencies: Removes all logical support links from  */
/*   a pattern entity that point to partial matches or other pattern */
/*   entities. Also removes the associated links from the partial    */
/*   matches or pattern entities which point back to the pattern     */
/*   entities.                                                       */
/*********************************************************************/
globle VOID RemoveEntityDependencies(theEntity) 
  struct patternEntity *theEntity;
  {
   struct dependency *fdPtr, *nextPtr, *theList;
   struct partialMatch *theBinds;

   fdPtr = (struct dependency *) theEntity->dependents;

   while (fdPtr != NULL)
     {
      nextPtr = fdPtr->next;

      theBinds = (struct partialMatch *) fdPtr->dPtr;
      theList = (struct dependency *) 
                   theBinds->binds[theBinds->bcount + theBinds->activationf].gm.theValue;
      theList = DetachAssociatedDependencies(theList,(VOID *) theEntity);
      theBinds->binds[theBinds->bcount + theBinds->activationf].gm.theValue = (VOID *) theList;

      rtn_struct(dependency,fdPtr);
      fdPtr = nextPtr;
     }

   theEntity->dependents = NULL;
  }

/*******************************************************************/
/* DetachAssociatedDependencies: Removes all logical support links */
/*   which pointer to a pattern entity from a list of dependencies */
/*   (which may be associated with either a partial match or       */
/*   another pattern entity). Does not remove links which point in */
/*   the other direction.                                          */
/*******************************************************************/
static struct dependency *DetachAssociatedDependencies(theList,theEntity) 
  struct dependency *theList;
  VOID *theEntity;
  {
   struct dependency *fdPtr, *nextPtr, *lastPtr = NULL;

   fdPtr = theList;

   while (fdPtr != NULL)
     {
      if (fdPtr->dPtr == theEntity)
        {
         nextPtr = fdPtr->next;
         if (lastPtr == NULL) theList = nextPtr;
         else lastPtr->next = nextPtr;
         rtn_struct(dependency,fdPtr);
         fdPtr = nextPtr;
        }
      else
        {
         lastPtr = fdPtr;
         fdPtr = fdPtr->next;
        }
     }
     
   return(theList);
  }
  
/**************************************************************************/
/* RemovePMDependencies: Removes all logical support links from a partial */
/*   match that point to any facts. Also removes the associated links     */
/*   from the facts which point back to the partial match.                */
/**************************************************************************/
globle VOID RemovePMDependencies(theBinds) 
  struct partialMatch *theBinds;
  {
   struct dependency *fdPtr, *nextPtr, *theList;
   struct patternEntity *theEntity;

   fdPtr = (struct dependency *) theBinds->binds[theBinds->bcount + theBinds->activationf].gm.theValue;

   while (fdPtr != NULL)
     {
      nextPtr = fdPtr->next;

      theEntity = (struct patternEntity *) fdPtr->dPtr;

      theList = (struct dependency *) theEntity->dependents;
      theList = DetachAssociatedDependencies(theList,(VOID *) theBinds);
      theEntity->dependents = (VOID *) theList;
      
      rtn_struct(dependency,fdPtr);
      fdPtr = nextPtr;
     }

   theBinds->binds[theBinds->bcount + theBinds->activationf].gm.theValue = NULL;
  }

/*****************************************************************************/
/* AddToDependencyList: Removes the dependency links between a partial match */
/*   and the facts it logically supports. Also removes the associated links  */
/*   from the facts which point back to the partial match by calling         */
/*   DetachAssociatedEntityDependencies. If an entity has all of its logical */
/*   support removed as a result of this procedure, the dependency link from */
/*   the partial match is added to the DependencyList so that the entity     */
/*   will be retracted as a result of losing its logical support.            */
/*****************************************************************************/
globle VOID AddToDependencyList(theBinds)
  struct partialMatch *theBinds;
  {
   struct dependency *dlPtr, *tempPtr, *theList;
   struct patternEntity *theEntity;

   if (theBinds->dependentsf == CLIPS_FALSE) return;
   
   dlPtr = (struct dependency *) theBinds->binds[theBinds->bcount + theBinds->activationf].gm.theValue;

   while (dlPtr != NULL)
     {
      tempPtr = dlPtr->next;

      theEntity = (struct patternEntity *) dlPtr->dPtr;
      
      theList = (struct dependency *) theEntity->dependents;
      theList = DetachAssociatedDependencies(theList,(VOID *) theBinds);
      theEntity->dependents = (VOID *) theList;
      
      if (theEntity->dependents == NULL)
        {
         dlPtr->next = DependencyList;
         DependencyList = dlPtr;
        }
      else
        { rtn_struct(dependency,dlPtr); }

      dlPtr = tempPtr;
     }

   theBinds->binds[theBinds->bcount + theBinds->activationf].gm.theValue = NULL;
  }
  
/********************************************************************************/
/* ForceLogicalRetractions: Retracts the first fact found on the DependencyList */
/*   by calling RetractFact. This retraction will then trigger the retract of   */
/*   the remaining facts on the DependencyList since RetractFact will call      */
/*   GetNextLogicalRetraction. This function is called by AddFact after a new   */
/*   fact has been processed because the addition of a new fact may cause       */
/*   partial matches associated with a not conditional element to be removed.   */
/********************************************************************************/
globle VOID ForceLogicalRetractions()
  {
   struct dependency *tempPtr;
   struct patternEntity *theEntity;
   static int alreadyEntered = CLIPS_FALSE;

   if (alreadyEntered) return;
   alreadyEntered = CLIPS_TRUE;
   
   while (DependencyList != NULL)
     {
      theEntity = (struct patternEntity *) DependencyList->dPtr;

      tempPtr = DependencyList;
      DependencyList = DependencyList->next;
      rtn_struct(dependency,tempPtr);

      (*theEntity->theInfo->base.deleteFunction)(theEntity);
     }
     
   alreadyEntered = CLIPS_FALSE;
  }

/**********************************************************/
/* ListDependencies: Lists the partial matches from which */
/*   a specified pattern entity receives logical support. */
/**********************************************************/
globle VOID ListDependencies(theEntity)
  struct patternEntity *theEntity;
  {
   struct dependency *fdPtr;
   struct partialMatch *theBinds;

   fdPtr = (struct dependency *) theEntity->dependents;

   if (fdPtr == NULL) PrintCLIPS(WDISPLAY,"None\n");

   while (fdPtr != NULL)
     {
      theBinds = (struct partialMatch *) fdPtr->dPtr;
      PrintPartialMatch(WDISPLAY,theBinds);
      PrintCLIPS(WDISPLAY,"\n");

      fdPtr = fdPtr->next;
     }
  }

/************************************************************/
/* ListDependents: Lists all pattern entities which receive */
/*   logical support from a specified entity.               */
/************************************************************/
globle VOID ListDependents(theEntity)
  struct patternEntity *theEntity;
  {
   struct patternEntity *entityPtr = NULL;
   struct patternParser *theParser = NULL;
   struct dependency *fdPtr;
   struct partialMatch *theBinds;
   int found = CLIPS_FALSE;

   GetNextPatternEntity(&theParser,&entityPtr);

   while (entityPtr != NULL)
     {
      fdPtr = (struct dependency *) entityPtr->dependents;
      while (fdPtr != NULL)
        {
         theBinds = (struct partialMatch *) fdPtr->dPtr;
         if (FindEntityInPartialMatch(theEntity,theBinds) == CLIPS_TRUE)
           {
            if (found) PrintCLIPS(WDISPLAY,",");
            (*entityPtr->theInfo->base.shortPrintFunction)(WDISPLAY,entityPtr);
            found = CLIPS_TRUE;
            fdPtr = NULL;
           }
         else
          { fdPtr = fdPtr->next; }
        }
        
      GetNextPatternEntity(&theParser,&entityPtr);
     }

   if (! found) PrintCLIPS(WDISPLAY,"None\n");
   else PrintCLIPS(WDISPLAY,"\n");
  }
  
/******************************************************/
/* FindEntityInPartialMatch: Searches for a specified */
/*   entity index in a partial match.                 */
/******************************************************/
static int FindEntityInPartialMatch(theEntity,thePartialMatch)
  struct patternEntity *theEntity;
  struct partialMatch *thePartialMatch;
  {
   short int i;

   for (i = 0 ; i < thePartialMatch->bcount; i++)
     {
      if (thePartialMatch->binds[i].gm.theMatch->matchingItem == theEntity)
        { return(CLIPS_TRUE); }
     }

   return(CLIPS_FALSE);
  }


#if DEBUGGING_FUNCTIONS

/*************************************************************/
/* DependenciesCommand: Implements the dependencies command. */
/*************************************************************/
globle VOID DependenciesCommand()
  {
   DATA_OBJECT item;
   VOID *ptr;

   if (ArgCountCheck("dependencies",EXACTLY,1) == -1) return;

   ptr = GetFactOrInstanceArgument(&item,"dependencies");
   
   if (ptr == NULL) return;
   
#if DEFRULE_CONSTRUCT
   ListDependencies((struct patternEntity *) ptr);
#else
   PrintCLIPS(WDISPLAY,"None\n");
#endif
  }
        
/*********************************************************/
/* DependentsCommand: Implements the dependents command. */
/*********************************************************/
globle VOID DependentsCommand()
  {
   DATA_OBJECT item;
   VOID *ptr;

   if (ArgCountCheck("dependents",EXACTLY,1) == -1) return;

   ptr = GetFactOrInstanceArgument(&item,"dependents");
   
   if (ptr == NULL) return;
   
#if DEFRULE_CONSTRUCT
   ListDependents((struct patternEntity *) ptr);
#else
   PrintCLIPS(WDISPLAY,"None\n");
#endif
  }
        
/*********************************************************/
/* GetFactOrInstanceArgument: */
/*********************************************************/
static VOID *GetFactOrInstanceArgument(item,functionName)
  DATA_OBJECT *item;
  char *functionName;
  {
   VOID *ptr;

   RtnUnknown(1,item);

   if ((GetpType(item) == FACT_ADDRESS) ||
       (GetpType(item) == INSTANCE_ADDRESS))
     { return(GetpValue(item)); }
#if DEFTEMPLATE_CONSTRUCT
   else if (GetpType(item) == INTEGER)
     {
      if ((ptr = (VOID *) FindIndexedFact(DOPToLong(item))) == NULL)
        {
         char tempBuffer[20];
         sprintf(tempBuffer,"f-%ld",DOPToLong(item));
         CantFindItemErrorMessage("fact",tempBuffer);
        }
      return(ptr);
     }
#endif
#if OBJECT_SYSTEM
   else if ((GetpType(item) == INSTANCE_NAME) || (GetpType(item) == SYMBOL))
     {
      if ((ptr = (VOID *) FindInstanceBySymbol((SYMBOL_HN *) GetpValue(item))) == NULL)
        {
         CantFindItemErrorMessage("instance",ValueToString(GetpValue(item)));
        }
      return(ptr);
     }
#endif
   else
     { ExpectedTypeError2(functionName,1); }
     
   return(NULL);
  }

#endif

#endif

