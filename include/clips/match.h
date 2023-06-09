   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                  MATCH HEADER FILE                  */
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

#ifndef _H_match

#define _H_match

struct genericMatch;
struct patternMatch;
struct partialMatch;
struct alphaMatch;
struct multifieldMarker;

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif
#ifndef _H_network
#include "network.h"
#endif
#ifndef _H_pattern
#include "pattern.h"
#endif

/************************************************************/
/* PATTERNMATCH STRUCTURE:                                  */
/************************************************************/
struct patternMatch
  {
   struct patternMatch *next;
   struct partialMatch *theMatch;
   struct patternNodeHeader *matchingPattern;
  };
  
/**************************/
/* genericMatch structure */
/**************************/
struct genericMatch
  {
   union
     {
      VOID *theValue;
      struct alphaMatch *theMatch;
     } gm;
  };
  
/************************************************************/
/* PARTIALMATCH STRUCTURE:                                  */
/************************************************************/
struct partialMatch
  {
   unsigned int betaMemory  : 1;
   unsigned int busy        : 1;
   unsigned int activationf : 1;
   unsigned int dependentsf : 1;
   unsigned int notOriginf  : 1;
   unsigned int counterf    : 1;
   unsigned int bcount      : 9;
   struct partialMatch *next;
   struct genericMatch binds[1];
  }; 

/************************************************************/
/* ALPHAMATCH STRUCTURE:                                    */
/************************************************************/
struct alphaMatch
  {
   struct patternEntity *matchingItem;
   struct multifieldMarker *markers;
   struct alphaMatch *next;
  };
  
/************************************************************/
/* MULTIFIELDMARKER STRUCTURE: Used in the pattern matching */
/* process to mark the range of fields that the $? and      */
/* $?variables match because a single pattern restriction   */
/* may span zero or more fields..                           */
/************************************************************/
struct multifieldMarker
  {
   int whichField;
   union
     {
      VOID *whichSlot;
      short int whichSlotNumber;
     } where;
    int startPosition;                                   
    int endPosition;
    struct multifieldMarker *next;
   };

#define get_nth_pm_value(thePM,thePos) (thePM->binds[thePos].gm.theValue)
#define get_nth_pm_match(thePM,thePos) (thePM->binds[thePos].gm.theMatch)

#define set_nth_pm_value(thePM,thePos,theVal) (thePM->binds[thePos].gm.theValue = (VOID *) theVal)
#define set_nth_pm_match(thePM,thePos,theVal) (thePM->binds[thePos].gm.theMatch = theVal)

#endif






