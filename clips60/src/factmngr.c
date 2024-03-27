   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 FACT MANAGER MODULE                 */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/


#define _FACTMNGR_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT && DEFRULE_CONSTRUCT

#include "constant.h"
#include "symbol.h"
#include "clipsmem.h"
#include "exprnpsr.h"
#include "argacces.h"
#include "scanner.h"
#include "router.h"
#include "strngrtr.h"
#include "match.h"
#include "factbld.h"
#include "reteutil.h"
#include "retract.h"
#include "filecom.h"
#include "constrct.h"
#include "factrhs.h"
#include "factmch.h"
#include "watch.h"
#include "utility.h"
#include "factbin.h"
#include "factmngr.h"
#include "facthsh.h"
#include "default.h"

#include "engine.h"
#include "lgcldpnd.h"
#include "drive.h"
#include "ruledlt.h"

#include "tmpltbsc.h"
#include "tmpltdef.h"
#include "tmpltcom.h"
#include "tmpltfun.h"

/****************************************/
/* GLOBAL EXTERNAL FUNCTION DEFINITIONS */
/****************************************/

#if ANSI_COMPILER
   extern VOID                    InitFactCommands(void);
   extern VOID                    FactPatternsCompilerSetup(void);
#else
   extern VOID                    InitFactCommands();
   extern VOID                    FactPatternsCompilerSetup();
#endif

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static VOID                    ResetFacts(void);
   static int                     ClearFactsReady(void);
#else
   static VOID                    ResetFacts();
   static int                     ClearFactsReady();
#endif

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle int              ChangeToFactList = CLIPS_FALSE;   
   globle struct fact      DummyFact = { { &FactInfo }, NULL, NULL, -1L, 0, 1, 
                                                        NULL, NULL, { 1, 0, 0 } };

#if DEBUGGING_FUNCTIONS
   globle int              WatchFacts = OFF;
#endif


/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static struct fact            *GarbageFacts = NULL;
   static struct fact            *LastFact = NULL;
   static struct fact            *FactList = NULL;
   static long int                NextFactIndex = 0L;
   static long int                NumberOfFacts = 0;
   
/***********************************************/
/* PrintFactWithIdentifier:  Displays a single */
/*   fact preceded by its fact identifier.     */
/***********************************************/
globle VOID PrintFactWithIdentifier(logicalName,factPtr)
  char *logicalName;
  struct fact *factPtr;
  {
   char printSpace[20];

   sprintf(printSpace,"f-%-5ld ",factPtr->factIndex);
   PrintCLIPS(logicalName,printSpace);
   PrintFact(logicalName,factPtr);
  }
  
/****************************************************/
/* PrintFactIdentifier: Displays a fact identifier. */
/****************************************************/
globle VOID PrintFactIdentifier(logicalName,factPtr)
  char *logicalName;
  VOID *factPtr;
  {
   char printSpace[20];

   sprintf(printSpace,"f-%ld",((struct fact *) factPtr)->factIndex);
   PrintCLIPS(logicalName,printSpace);
  }
  
/****************************************************/
/* PrintFactInLongForm*/
/****************************************************/
globle VOID PrintFactInLongForm(logicalName,factPtr)
  char *logicalName;
  VOID *factPtr;
  {   
   if (AddressesToStrings) PrintCLIPS(logicalName,"\"");
   if (factPtr != (VOID *) &DummyFact)
     {
      PrintCLIPS(logicalName,"<Fact-");
      PrintLongInteger(logicalName,((struct fact *) factPtr)->factIndex);
      PrintCLIPS(logicalName,">");
     }
   else
     { PrintCLIPS(logicalName,"<Dummy Fact>"); }
     
   if (AddressesToStrings) PrintCLIPS(logicalName,"\"");
  }
  
/****************************************************/
/* Decrements the busy count of a fact              */
/****************************************************/
globle VOID DecrementFactBasisCount(factPtr)
  VOID *factPtr;
  { ((struct patternEntity *) factPtr)->busyCount--; }
  
/****************************************************/
/* Increments the busy count of a fact              */
/****************************************************/
globle VOID IncrementFactBasisCount(factPtr)
  VOID *factPtr;
  { ((struct patternEntity *) factPtr)->busyCount++; }
  
/***********************************************/
/* PrintFact: Displays the individual elements */
/*   of a fact enclosed within paretheses.     */
/***********************************************/
globle VOID PrintFact(logicalName,factPtr)
  char *logicalName;
  struct fact *factPtr;
  {
   struct field *sublist;
   struct multifield *segmentPtr;

   if (factPtr->whichDeftemplate->implied == CLIPS_FALSE)
     {
      PrintTemplateFact(logicalName,factPtr);
      return;
     }

   PrintCLIPS(logicalName,"(");
      
   PrintCLIPS(logicalName,factPtr->whichDeftemplate->header.name->contents);
   
   sublist = factPtr->theProposition.theFields;

   segmentPtr = (struct multifield *) sublist[0].value;
   if (segmentPtr->multifieldLength != 0)
     {
      PrintCLIPS(logicalName," ");
      PrintMultifield(logicalName,segmentPtr,0,segmentPtr->multifieldLength - 1,CLIPS_FALSE);
     }

   PrintCLIPS(logicalName,")");
  }
  
/****************************************************/
/* Filters a fact through the fact pattern network  */
/* Used in binary loads and run-time modules        */
/****************************************************/
globle VOID MatchFactFunction(vfactPtr)
  VOID *vfactPtr;
  {
   struct fact *factPtr;
   
   factPtr = (struct fact *) vfactPtr;
   FactPatternMatch(factPtr,factPtr->whichDeftemplate->patternNetwork,0,NULL,NULL);
  }

/******************************************************/
/* Retract: C access routine for the retract command. */
/******************************************************/
globle BOOLEAN Retract(theFact)
  VOID *theFact;
  {
   struct fact *temp_ptr;
   int rv = -1;
   struct fact *factPtr;

   if (JoinOperationInProgress)
     {
      PrintErrorID("FACTMNGR",1,CLIPS_TRUE);
      PrintCLIPS(WERROR,"Facts may not be retracted during pattern-matching\n");
      return(CLIPS_FALSE);
     }

   if (theFact == NULL)
     {
      RemoveAllFacts();
      return(CLIPS_TRUE);
     }

   factPtr = (struct fact *) theFact;
   
   /*===================================================*/
   /* Reset the evaluation error flag since expressions */
   /* will be evaluated as part of the retract.         */
   /*===================================================*/

   SetEvaluationError(CLIPS_FALSE);

   /*======================================================*/
   /* Check to see if the fact has already been retracted. */
   /*======================================================*/

   if (factPtr->garbage)
     {
      if (rv < 0) rv = CLIPS_FALSE;
      return(rv);
     }

   /*=========================================*/
   /* Show retraction if facts being watched. */
   /*=========================================*/

#if DEBUGGING_FUNCTIONS
   if (factPtr->whichDeftemplate->watch)
     {
      PrintCLIPS(WTRACE,"<== ");
      PrintFactWithIdentifier(WTRACE,factPtr);
      PrintCLIPS(WTRACE,"\n");
     }
#endif

   ChangeToFactList = CLIPS_TRUE;

   /*=====================================*/
   /* Delete the fact from the fact list. */
   /*=====================================*/

#if LOGICAL_DEPENDENCIES
   RemoveEntityDependencies((struct patternEntity *) factPtr);
#endif

   RemoveHashedFact(factPtr);

   if (factPtr == LastFact)
     { LastFact = factPtr->previousFact; }

   if (factPtr->previousFact == NULL)
     {
      /* Delete the head of the fact list. */
      FactList = FactList->nextFact;
      if (FactList != NULL)
        { FactList->previousFact = NULL; }
     }
   else
     {
      /* Delete a fact other than the head of the fact list. */
      factPtr->previousFact->nextFact = factPtr->nextFact;
      if (factPtr->nextFact != NULL)
        { factPtr->nextFact->previousFact = factPtr->previousFact; }
     }

   temp_ptr = GarbageFacts;
   GarbageFacts = factPtr;

   FactDeinstall(factPtr);
   EphemeralItemCount++;
   EphemeralItemSize += sizeof(struct fact) + (sizeof(struct field) * factPtr->theProposition.multifieldLength);

   factPtr->nextFact = temp_ptr;
   factPtr->garbage = CLIPS_TRUE;
   if (rv < 0) rv = CLIPS_TRUE;

   /*================================================*/
   /* Loop through the list of all the patterns that */
   /* matched the fact.                              */
   /*================================================*/

   JoinOperationInProgress = CLIPS_TRUE;
   NetworkRetract((struct patternMatch *) factPtr->list);
   JoinOperationInProgress = CLIPS_FALSE;

   if (ExecutingRule == NULL)
     { FlushGarbagePartialMatches(); }

#if LOGICAL_DEPENDENCIES
   ForceLogicalRetractions();
#endif

   return(rv);
  }

/*******************************************************************/
/* RemoveOldFacts:  Returns facts that have been retracted to the  */
/*   pool of available memory.  It is necessary to postpone        */
/*   returning the facts to memory because RHS actions retrieve    */
/*   their variable bindings directly from the fact data structure */
/*   and the facts may be in use in other data structures.         */
/*******************************************************************/
globle VOID RemoveOldFacts()
  {
   struct fact *factPtr, *nextPtr, *lastPtr = NULL;

   factPtr = GarbageFacts;
   while (factPtr != NULL)
     {
      nextPtr = factPtr->nextFact;
      if ((factPtr->factHeader.busyCount == 0) &&
          (((int) factPtr->depth) > CurrentEvaluationDepth)) 
        {
         EphemeralItemCount--;
         EphemeralItemSize -= sizeof(struct fact) + (sizeof(struct field) * factPtr->theProposition.multifieldLength);
         ReturnFact(factPtr);
         if (lastPtr == NULL) GarbageFacts = nextPtr;
         else lastPtr->nextFact = nextPtr;
        }
      else
        { lastPtr = factPtr; }

      factPtr = nextPtr;
     }
  }

/***************************************************/
/* Assert: C access routine for the assert command.*/
/***************************************************/
globle VOID *Assert(theFact)
  VOID *theFact;
  {
   int hashValue;
   int length, i;
   struct field *fieldPtr;
   struct fact *newFact;

   newFact = (struct fact *) theFact;

   if (JoinOperationInProgress)
     {
      ReturnFact(newFact);
      PrintErrorID("FACTMNGR",2,CLIPS_TRUE);
      PrintCLIPS(WERROR,"Facts may not be asserted during pattern-matching\n");
      return(NULL);
     }

   /*=============================================================*/
   /* Replace invalid data types in the fact with the symbol nil. */
   /*=============================================================*/

   length = newFact->theProposition.multifieldLength;
   fieldPtr = newFact->theProposition.theFields;

   for (i = 0; i < length; i++)
     {
      if (fieldPtr[i].type == RVOID)
        {
         fieldPtr[i].type = SYMBOL;
         fieldPtr[i].value = (VOID *) AddSymbol("nil");
        }
     }

   /*========================================================*/
   /* If fact assertions are being checked for duplications, */
   /* then search the fact list for a duplicate fact.        */
   /*========================================================*/

   hashValue = HandleFactDuplication(newFact);
   if (hashValue < 0) return(NULL);

#if LOGICAL_DEPENDENCIES
   if (AddLogicalDependencies((struct patternEntity *) newFact,CLIPS_FALSE) == CLIPS_FALSE)
     {
      ReturnFact(newFact);
      return(NULL);
     }
#endif

   AddHashedFact(newFact,hashValue);

   /*===================================================*/
   /* Add the fact to the fact list. Set the fact index */
   /* for the fact and install the symbols used by the  */
   /* fact in the symbol table.                         */
   /*===================================================*/

   newFact->nextFact = NULL;
   newFact->list = NULL;
   newFact->previousFact = LastFact;
   if (LastFact == NULL)
     { FactList = newFact; }
   else
     { LastFact->nextFact = newFact; }
   LastFact = newFact;

   newFact->factIndex = NextFactIndex++;
   newFact->factHeader.timeTag = CurrentEntityTimeTag++;
   FactInstall(newFact);
   
   /*===============================================*/
   /* Indicate the addition of the fact to the fact */
   /* list if facts are being watched.              */
   /*===============================================*/

#if DEBUGGING_FUNCTIONS
   if (newFact->whichDeftemplate->watch)
     {
      PrintCLIPS(WTRACE,"==> ");
      PrintFactWithIdentifier(WTRACE,newFact);
      PrintCLIPS(WTRACE,"\n");
     }
#endif

   ChangeToFactList = CLIPS_TRUE;
   
   /*===============================*/
   /* Check for errors in the fact. */
   /*===============================*/

   CheckTemplateFact(newFact);

   /*==============================================*/
   /* Filter the fact through the pattern network. */
   /*==============================================*/

   SetEvaluationError(CLIPS_FALSE);
   JoinOperationInProgress = CLIPS_TRUE;
   FactPatternMatch(newFact,newFact->whichDeftemplate->patternNetwork,0,NULL,NULL);
   JoinOperationInProgress = CLIPS_FALSE;

#if LOGICAL_DEPENDENCIES
   ForceLogicalRetractions();
#endif

   if (ExecutingRule == NULL) FlushGarbagePartialMatches();

   return((VOID *) newFact);
  }

/**********************************************************************/
/* RemoveAllFacts: Loops through the fact-list and removes each fact. */
/**********************************************************************/
globle VOID RemoveAllFacts()
  {
   while (FactList != NULL)
     { Retract((VOID *) FactList); }
  }

/*********************************************/
/* CreateFact: Creates a fact data structure */
/*   of the specified deftemplate.           */
/*********************************************/
globle struct fact *CreateFact(vTheDeftemplate)
  VOID *vTheDeftemplate;
  {
   struct deftemplate *theDeftemplate = (struct deftemplate *) vTheDeftemplate;
   struct fact *newFact;
   int i;
   
   if (theDeftemplate == NULL) return(NULL);
   
   if (theDeftemplate->implied == CLIPS_FALSE) 
     { 
      newFact = CreateFactBySize((int) theDeftemplate->numberOfSlots);
      for (i = 0;
           i < theDeftemplate->numberOfSlots;
           i++)
        { newFact->theProposition.theFields[i].type = RVOID; }
     }
   else 
     {
      newFact = CreateFactBySize(1);
      newFact->theProposition.theFields[0].type = MULTIFIELD;
      newFact->theProposition.theFields[0].value = CreateMultifield2(0);
     }
   
   newFact->whichDeftemplate = theDeftemplate;
   
   return(newFact);
  }
  
/********************************************/
/* GetFactSlot: Returns the slot value from */
/*   the specified slot of a fact.          */
/********************************************/
globle BOOLEAN GetFactSlot(vTheFact,slotName,theValue)
  VOID *vTheFact;
  char *slotName;
  DATA_OBJECT *theValue;
  {
   struct fact *theFact = (struct fact *) vTheFact;
   struct deftemplate *theDeftemplate;
   int whichSlot;
   
   theDeftemplate = theFact->whichDeftemplate;
   
   /*==============================================*/
   /* Handle retrieving the slot value from a fact */
   /* having an implied deftemplate. An implied    */
   /* facts has a single multifield slot.          */
   /*==============================================*/
   
   if (theDeftemplate->implied) 
     {
      if (slotName != NULL) return(CLIPS_FALSE);
      theValue->type = theFact->theProposition.theFields[0].type; 
      theValue->value = theFact->theProposition.theFields[0].value;
      SetpDOBegin(theValue,1);
      SetpDOEnd(theValue,((struct multifield *) theValue->value)->multifieldLength);
      return(CLIPS_TRUE); 
     } 
   
   /*===================================*/
   /* Make sure the slot name requested */
   /* corresponds to a valid slot name. */
   /*===================================*/
           
   if (FindSlot(theDeftemplate,AddSymbol(slotName),&whichSlot) == NULL) 
     { return(CLIPS_FALSE); }
   
   /*======================================================*/
   /* Return the slot value. If the slot value wasn't set, */
   /* then return FALSE to indicate that an appropriate    */
   /* slot value wasn't available.                         */
   /*======================================================*/
   
   theValue->type = theFact->theProposition.theFields[whichSlot-1].type; 
   theValue->value = theFact->theProposition.theFields[whichSlot-1].value;
   if (theValue->type == MULTIFIELD)
     {
      SetpDOBegin(theValue,1);
      SetpDOEnd(theValue,((struct multifield *) theValue->value)->multifieldLength);
     }
   
   if (theValue->type == RVOID) return(CLIPS_FALSE);
   
   return(CLIPS_TRUE);
  }
  
/***************************************/
/* PutFactSlot: Sets the slot value of */
/*   the specified slot of a fact.     */
/***************************************/
globle BOOLEAN PutFactSlot(vTheFact,slotName,theValue)
  VOID *vTheFact;
  char *slotName;
  DATA_OBJECT *theValue;
  {
   struct fact *theFact = (struct fact *) vTheFact;
   struct deftemplate *theDeftemplate;
   struct templateSlot *theSlot;
   int whichSlot;
   
   theDeftemplate = theFact->whichDeftemplate;
   
   /*============================================*/
   /* Handle setting the slot value of a fact    */
   /* having an implied deftemplate. An implied  */
   /* facts has a single multifield slot.        */
   /*============================================*/
   
   if (theDeftemplate->implied)
     {
      if ((slotName != NULL) || (theValue->type != MULTIFIELD))
        { return(CLIPS_FALSE); }
        
      if (theFact->theProposition.theFields[0].type == MULTIFIELD)
        { ReturnMultifield(theFact->theProposition.theFields[0].value); }
        
      theFact->theProposition.theFields[0].type = theValue->type; 
      theFact->theProposition.theFields[0].value = DOToMultifield(theValue);
      
      return(CLIPS_TRUE); 
     } 
   
   /*===================================*/
   /* Make sure the slot name requested */
   /* corresponds to a valid slot name. */
   /*===================================*/
           
   if ((theSlot = FindSlot(theDeftemplate,AddSymbol(slotName),&whichSlot)) == NULL) 
     { return(CLIPS_FALSE); }
     
   /*=============================================*/
   /* Make sure a single field value is not being */
   /* stored in a multifield slot or vice versa.  */
   /*=============================================*/
   
   if (((theSlot->multislot == 0) && (theValue->type == MULTIFIELD)) ||
       ((theSlot->multislot == 1) && (theValue->type != MULTIFIELD)))
     { return(CLIPS_FALSE); }
   
   /*=====================*/
   /* Set the slot value. */
   /*=====================*/
   
   if (theFact->theProposition.theFields[whichSlot-1].type == MULTIFIELD)
     { ReturnMultifield(theFact->theProposition.theFields[whichSlot-1].value); }
     
   theFact->theProposition.theFields[whichSlot-1].type = theValue->type; 
   
   if (theValue->type == MULTIFIELD)
     { theFact->theProposition.theFields[whichSlot-1].value = DOToMultifield(theValue); }
   else
     { theFact->theProposition.theFields[whichSlot-1].value = theValue->value; }
   
   return(CLIPS_TRUE);
  }
  
/********************************************************/
/* AssignFactSlotDefaults: Sets a facts' slot values to */
/*   its default value if the value of the slot has not */
/*   yet been set.                                      */
/********************************************************/
globle BOOLEAN AssignFactSlotDefaults(vTheFact)
  VOID *vTheFact;
  {
   struct fact *theFact = (struct fact *) vTheFact;
   struct deftemplate *theDeftemplate;
   struct templateSlot *slotPtr;  
   int i;
   DATA_OBJECT theResult;
   
   theDeftemplate = theFact->whichDeftemplate;
   if (theDeftemplate->implied) return(CLIPS_TRUE);
   
   for (i = 0, slotPtr = theDeftemplate->slotList; 
        i < theDeftemplate->numberOfSlots; 
        i++, slotPtr = slotPtr->next)
     {
      if (theFact->theProposition.theFields[i].type == RVOID) 
        {
         if (slotPtr->noDefault) return(CLIPS_FALSE);
         else if (slotPtr->defaultPresent)
           {
            if (slotPtr->multislot)
              {
               StoreInMultifield(&theResult,slotPtr->defaultList,CLIPS_TRUE);
               theFact->theProposition.theFields[i].value = DOToMultifield(&theResult);
              }
            else
              {
              theFact->theProposition.theFields[i].type = slotPtr->defaultList->type;
              theFact->theProposition.theFields[i].value = slotPtr->defaultList->value;
             }
           }
         else if (slotPtr->defaultDynamic)
           {
            EvaluateExpression(slotPtr->defaultList,&theResult);
            if (EvaluationError) return(CLIPS_FALSE);
            theFact->theProposition.theFields[i].type = theResult.type;
            if (theResult.type == MULTIFIELD)
              { theFact->theProposition.theFields[i].value = DOToMultifield(&theResult); }
            else
              { theFact->theProposition.theFields[i].value = theResult.value; }
           }
         else 
           {
            DeriveDefaultFromConstraints(slotPtr->constraints,&theResult,
                                         (int) slotPtr->multislot);
                                            
            theFact->theProposition.theFields[i].type = theResult.type;
            if (theResult.type == MULTIFIELD)
              { theFact->theProposition.theFields[i].value = DOToMultifield(&theResult); }
            else
              { theFact->theProposition.theFields[i].value = theResult.value; }
           }
        }
     }
   
   return(CLIPS_TRUE);
  }
  
/***********************************************************/
/* CreateFactBySize:  Allocates a fact structure / element array */
/*   structure combination for the storage of a fact.      */
/***********************************************************/
globle struct fact *CreateFactBySize(size)
  int size;
  {
   struct fact *theFact;
   int newSize;

   if (size <= 0) newSize = 1;
   else newSize = size;

   theFact = get_var_struct2(fact,sizeof(struct field) * (newSize - 1)); 
   
   theFact->depth = (unsigned) CurrentEvaluationDepth;
   theFact->garbage = CLIPS_FALSE;
   theFact->factIndex = 0L;
   theFact->factHeader.busyCount = 0;
   theFact->factHeader.theInfo = &FactInfo;
#if LOGICAL_DEPENDENCIES
   theFact->factHeader.dependents = NULL;
#endif
   theFact->whichDeftemplate = NULL;
   theFact->nextFact = NULL;
   theFact->previousFact = NULL;
   theFact->list = NULL;
   
   theFact->theProposition.multifieldLength = size;
   theFact->theProposition.depth = CurrentEvaluationDepth;
   theFact->theProposition.busyCount = 0;

   return(theFact);
  }

/*****************************************************************/
/* ReturnFact:                                             */
/*****************************************************************/
globle VOID ReturnFact(theFact)
  struct fact *theFact;
  {
   struct multifield *theSegment;
   int newSize, i;
   
   theSegment = &theFact->theProposition;
   
   for (i = 0; i < theSegment->multifieldLength; i++)
     {
      if (theSegment->theFields[i].type == MULTIFIELD)
        { ReturnMultifield(theSegment->theFields[i].value); }
     }
   
   if (theFact->theProposition.multifieldLength == 0) newSize = 1;
   else newSize = theFact->theProposition.multifieldLength;
   
   rtn_var_struct2(fact,sizeof(struct field) * (newSize - 1),theFact);
  }

/**************************************************************/
/* FactInstall:  Increments all occurrences in the hash table */
/*   of symbols found in the atoms of a fact.                 */
/**************************************************************/
globle VOID FactInstall(newFact)
  struct fact *newFact;
  {
   struct multifield *theSegment;
   int i;
   
   NumberOfFacts++;
   newFact->whichDeftemplate->busyCount++;
   theSegment = &newFact->theProposition;
      
   for (i = 0 ; i < theSegment->multifieldLength ; i++)
     { 
      AtomInstall(theSegment->theFields[i].type,theSegment->theFields[i].value);
     }
     
   newFact->factHeader.busyCount++; 
  }

/****************************************************************/
/* FactDeinstall:  Decrements all occurrences in the hash table */
/*   of symbols found in the atoms of a fact.                   */
/****************************************************************/
globle VOID FactDeinstall(newFact)
  struct fact *newFact;
  {
   struct multifield *theSegment;
   int i;
   
   NumberOfFacts--;
   theSegment = &newFact->theProposition;
   newFact->whichDeftemplate->busyCount--;
   
   for (i = 0 ; i < theSegment->multifieldLength ; i++)
     { 
      AtomDeinstall(theSegment->theFields[i].type,theSegment->theFields[i].value);
     }
     
   newFact->factHeader.busyCount--; 
  }

/**************************************************************/
/* IncrementFactCount:  Increments the counter for the number */
/*   of needed references to a particular fact.               */
/**************************************************************/
globle VOID IncrementFactCount(factPtr)
  VOID *factPtr;
  {
   ((struct fact *) factPtr)->factHeader.busyCount++;
  }

/**************************************************************/
/* DecrementFactCount:  Decrements the counter for the number */
/*   of needed references to a particular fact.               */
/**************************************************************/
globle VOID DecrementFactCount(factPtr)
  VOID *factPtr;
  {
   ((struct fact *) factPtr)->factHeader.busyCount--;
  }

/*****************************************************************/
/* SetFactID:                              */
/*****************************************************************/
globle VOID SetFactID(value)
  FACT_ID value;
  {
   NextFactIndex = value;
  }

/*****************************************************************/
/* GetNextFact:                    */
/*****************************************************************/
globle VOID *GetNextFact(factPtr)
  VOID *factPtr;
  {
   if (factPtr == NULL)
     { return((VOID *) FactList); }

   if (((struct fact *) factPtr)->garbage) return(NULL);

   return((VOID *) ((struct fact *) factPtr)->nextFact);
  }
  
/*****************************************************************/
/* GetNextFactInScope:                    */
/*****************************************************************/
globle VOID *GetNextFactInScope(vFactPtr)
  VOID *vFactPtr;
  {          
   static long lastModuleIndex = -1;
   struct fact *factPtr = (struct fact *) vFactPtr;
   
   if (factPtr == NULL)
     { 
      factPtr = FactList;
      if (lastModuleIndex != ModuleChangeIndex)
        {
         UpdateDeftemplateScope();
         lastModuleIndex = ModuleChangeIndex;
        }
     }
   else if (((struct fact *) factPtr)->garbage) 
     return(NULL);
   else
     { factPtr = factPtr->nextFact; }
     
   while (factPtr != NULL)
     {
      if (factPtr->whichDeftemplate->inScope) return((VOID *) factPtr);
        
      factPtr = factPtr->nextFact;
     }
     
   return(NULL);
  }

/***************************************************/
/* GetFactPPForm:                                  */
/***************************************************/
globle VOID GetFactPPForm(buffer,bufferLength,theFact)
  char *buffer;
  int bufferLength;
  VOID *theFact;
  {
   OpenStringDestination("FactPPForm",buffer,bufferLength);
   PrintFactWithIdentifier("FactPPForm",(struct fact *) theFact);
   CloseStringDestination("FactPPForm");
  }

/************************************************************/
/* FactIndex: C access routine for the fact-index function. */
/************************************************************/
globle long int FactIndex(factPtr)
  VOID *factPtr;
  {
   return(((struct fact *) factPtr)->factIndex);
  }

/********************************************************/
/* AssertString:                                              */
/********************************************************/
globle VOID *AssertString(str)
  char *str;
  {
   struct fact *factPtr;

   factPtr = StringToFact(str);

   if (factPtr == NULL) return(NULL);

   return((VOID *) Assert((VOID *) factPtr));
  }

/***************************************************/
/* GetFactListChanged:                              */
/***************************************************/
globle int GetFactListChanged()
  { return(ChangeToFactList); }

/***************************************************/
/* SetFactListChanged:                              */
/***************************************************/
globle VOID SetFactListChanged(value)
  int value;
  {
   ChangeToFactList = value;
  }


/***************************************************/
/* GetNumberOfFacts:                               */
/***************************************************/
globle long int GetNumberOfFacts()
  { return(NumberOfFacts); }

/**********************************************************/
/* InitializeFacts: Initializes reset function for facts. */
/**********************************************************/
globle VOID InitializeFacts()
  {
   InitializeFactHashTable();
   
   AddResetFunction("facts",ResetFacts,20);
   AddClearReadyFunction("facts",ClearFactsReady,0);
   
   AddCleanupFunction("facts",RemoveOldFacts,0);

   InitializeFactPatterns();

#if DEBUGGING_FUNCTIONS
   AddWatchItem("facts",0,&WatchFacts,80,DeftemplateWatchAccess,DeftemplateWatchPrint);
#endif
   InitFactCommands();
   
#if (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE) && (! RUN_TIME)
   FactBinarySetup();
#endif

#if CONSTRUCT_COMPILER && (! RUN_TIME)
   FactPatternsCompilerSetup();
#endif
  }

/*********************************************************************/
/* ResetFacts: */
/*********************************************************************/
static VOID ResetFacts()
  {
   /*====================================*/
   /* Initialize the fact index to zero. */
   /*====================================*/

   SetFactID(0L);

   /*======================================*/
   /* Remove all facts from the fact list. */
   /*======================================*/

   RemoveAllFacts();
  }
  
/*********************************************************************/
/* ClearFactsReady: */
/*********************************************************************/
static int ClearFactsReady()
  {
   /*====================================*/
   /* Initialize the fact index to zero. */
   /*====================================*/

   SetFactID(0L);

   /*======================================*/
   /* Remove all facts from the fact list. */
   /*======================================*/

   RemoveAllFacts();
   
   /*==============================================*/
   /* If for some reason there are any facts still */
   /* remaining, don't continue with the clear.    */
   /*==============================================*/
                    
   if (GetNextFact(NULL) != NULL) return(CLIPS_FALSE);
   
   return(CLIPS_TRUE);
  }

/***************************************************/
/* FindIndexedFact: Returns a pointer to a fact in */
/*   the fact list with the specified fact index.  */
/***************************************************/
globle struct fact *FindIndexedFact(factIndexSought)
  long int factIndexSought;
  {
   struct fact *ptr;

   ptr = (struct fact *) GetNextFact(NULL);
   while (ptr != NULL)
     {
      if (ptr->factIndex == factIndexSought)
        { return(ptr); }

      ptr = (struct fact *) GetNextFact(ptr);
     }

   return(NULL);
  }

#endif

