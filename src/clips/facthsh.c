   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 FACT HASHING MODULE                 */
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

#define _FACTHSH_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_

#include "setup.h"

#if DEFTEMPLATE_CONSTRUCT

#include "constant.h"
#include "clipsmem.h"
#include "router.h"

#if DEFRULE_CONSTRUCT
#include "lgcldpnd.h"
#endif

#include "facthsh.h"

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static int                     HashFact(struct fact *);
   static int                     HashMultifield(struct multifield *);
   static struct fact            *FactExists(struct fact *,int);
#else
   static int                     HashFact();
   static int                     HashMultifield();
   static struct fact            *FactExists();
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static struct factHashEntry  **FactHashTable;
   static BOOLEAN                 FactDuplication = CLIPS_FALSE;

/******************************************************/
/* HashFact: Returns the hash value for a given fact. */
/******************************************************/
static int HashFact(theFact)
  struct fact *theFact;
  {
   int count = 0;
   int hashValue = 0;
   
   count += HashSymbol(ValueToString(theFact->whichDeftemplate->header.name),
                       SIZE_FACT_HASH);
   
   count += (int) HashMultifield(&theFact->theProposition);
   
   hashValue = (int) (count % SIZE_FACT_HASH);
   if (hashValue < 0) hashValue = - hashValue;
   return(hashValue);
  }
     
/************************************************************/
/* HashMultifield: Returns the hash value for a multifield. */
/************************************************************/
static int HashMultifield(theSegment)
  struct multifield *theSegment;
  {
   int length, i = 0;
   unsigned int tvalue;
   unsigned int count = 0;
   struct field *fieldPtr;
   union
     {
      double fv;
      unsigned int liv;
     } fis;

   length = theSegment->multifieldLength;
   fieldPtr = theSegment->theFields;

    while (i < length)
     {
      switch(fieldPtr[i].type)
         {
          case MULTIFIELD:
            count += HashMultifield((struct multifield *) fieldPtr[i].value);
            break;
            
          case FLOAT:
            fis.fv = ValueToDouble(fieldPtr[i].value);
            count += (fis.liv * (i + 29));
            break;
            
          case INTEGER:
            count += (int) (((int) ValueToLong(fieldPtr[i].value)) * (i + 29));
            break;
            
          case FACT_ADDRESS:
          case EXTERNAL_ADDRESS:
#if OBJECT_SYSTEM
          case INSTANCE_ADDRESS:
#endif
            count += (int) (((int) fieldPtr[i].value) * (i + 29));
            break;
            
          case SYMBOL:
          case STRING:
#if OBJECT_SYSTEM
          case INSTANCE_NAME:
#endif
            tvalue = (unsigned) HashSymbol(ValueToString(fieldPtr[i].value),SIZE_FACT_HASH);
            count += (unsigned) (tvalue * (i + 29));
            break;
         }

      i++;
     }

   return(count);
  }

/**********************************************/
/* FactExists: Determines if a specified fact */
/*   already exists in the fact hash table.   */
/**********************************************/
static struct fact *FactExists(theFact,hashValue)
  struct fact *theFact;
  int hashValue;
  {
   struct factHashEntry *theFactHash;

   theFactHash = FactHashTable[hashValue];

   while (theFactHash != NULL)
     {
      if ((theFact->whichDeftemplate == theFactHash->theFact->whichDeftemplate) ?
          MultifieldsEqual(&theFact->theProposition,
                           &theFactHash->theFact->theProposition) : CLIPS_FALSE) 
        { return(theFactHash->theFact); }
      theFactHash = theFactHash->next;
     }

   return(NULL);
  }

/************************************************************/
/* AddHashedFact: Adds a fact entry to the fact hash table. */
/************************************************************/
globle VOID AddHashedFact(theFact,hashValue)
  struct fact *theFact;
  int hashValue;
  {
   struct factHashEntry *newhash, *temp;

   newhash = get_struct(factHashEntry);
   newhash->theFact = theFact;

   temp = FactHashTable[hashValue];
   FactHashTable[hashValue] = newhash;
   newhash->next = temp;
  }

/******************************************/
/* RemoveHashedFact: Removes a fact entry */
/*   from the fact hash table.            */
/******************************************/
globle BOOLEAN RemoveHashedFact(theFact)
  struct fact *theFact;
  {
   int hashValue;
   struct factHashEntry *hptr, *prev = NULL;

   hashValue = HashFact(theFact);

   hptr = FactHashTable[hashValue];
   while (hptr != NULL)
     {
      if (hptr->theFact == theFact)
        {
         if (prev == NULL)
           {
            FactHashTable[hashValue] = hptr->next;
            rtn_struct(factHashEntry,hptr);
            return(1);
           }
         else
           {
            prev->next = hptr->next;
            rtn_struct(factHashEntry,hptr);
            return(1);
           }
        }
      prev = hptr;
      hptr = hptr->next;
     }
     
   return(0);
  }

/*****************************************************/
/* HandleFactDuplication: Determines if a fact to be */
/*   added to the fact-list is a duplicate entry and */
/*   takes appropriate action based on the current   */
/*   setting of the fact-duplication flag.           */
/*****************************************************/
globle int HandleFactDuplication(theFact)
  VOID *theFact;
  {
   struct fact *tempPtr;
   int hashValue;

   hashValue = HashFact(theFact);

   if (FactDuplication) return(hashValue);
   
   tempPtr = FactExists(theFact,hashValue);
   if (tempPtr == NULL) return(hashValue);

   ReturnFact(theFact);
#if LOGICAL_DEPENDENCIES && DEFRULE_CONSTRUCT
   AddLogicalDependencies((struct patternEntity *) tempPtr,CLIPS_TRUE);
#endif
   return(-1);
  }

/********************************************/
/* GetFactDuplication: C access routine for */
/*   the get-fact-duplication command.      */
/********************************************/
globle BOOLEAN GetFactDuplication()
  { return(FactDuplication); }

/********************************************/
/* SetFactDuplication: C access routine for */
/*   the set-fact-duplication command.      */
/********************************************/
globle BOOLEAN SetFactDuplication(value)
  int value;
  {
   int ov;

   ov = FactDuplication;
   FactDuplication = value;
   return(ov);
  }

/**************************************************/
/* InitializeFactHashTable: Initializes the table */
/*   entries in the fact hash table to NULL.      */
/**************************************************/
globle VOID InitializeFactHashTable()
   {
    int i;

    FactHashTable = (struct factHashEntry **) 
                    gm2((int) sizeof (struct factHashEntry *) * SIZE_FACT_HASH);

    if (FactHashTable == NULL) ExitCLIPS(1);

    for (i = 0; i < SIZE_FACT_HASH; i++) FactHashTable[i] = NULL;
   }

#if DEVELOPER

/*****************************************************/
/* ShowFactHashTable: Displays the number of entries */
/*   in each slot of the fact hash table.            */
/*****************************************************/
globle VOID ShowFactHashTable()
   {
    int i, count;
    struct factHashEntry *theEntry;
    char buffer[20];


    for (i = 0; i < SIZE_FACT_HASH; i++) 
      {
       for (theEntry =  FactHashTable[i], count = 0;
            theEntry != NULL;
            theEntry = theEntry->next,count++);
            
       if (count != 0) 
         {
          sprintf(buffer,"%4d: %4d\n",i,count);
          PrintCLIPS(WDISPLAY,buffer);
         }
      }
   }
   
#endif

#endif

