   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*         CONFLICT RESOLUTION STRATEGY MODULE         */
   /*******************************************************/

/*************************************************************/
/* Purpose: Used to determine where a new activation is      */
/*   placed on the agenda based on the current conflict      */
/*   resolution strategy (depth, breadth, mea, lex,          */
/*   simplicity, or complexity). Also provides the           */
/*   set-strategy and get-strategy commands.                 */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _CRSTRTGY_SOURCE_

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>

#include "setup.h"

#if DEFRULE_CONSTRUCT

#include "constant.h"
#include "pattern.h"
#include "reteutil.h"
#include "argacces.h"
#include "agenda.h"
#include "crstrtgy.h"

#define GetMatchingItem(x,i) (x->basis->binds[i].gm.theMatch->matchingItem)
   
/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static ACTIVATION             *PlaceDepthActivation(ACTIVATION *,ACTIVATION *);
#if CONFLICT_RESOLUTION_STRATEGIES
   static ACTIVATION             *PlaceBreadthActivation(ACTIVATION *,ACTIVATION *);
   static ACTIVATION             *PlaceLEXActivation(ACTIVATION *,ACTIVATION *);
   static ACTIVATION             *PlaceMEAActivation(ACTIVATION *,ACTIVATION *);
   static ACTIVATION             *PlaceComplexityActivation(ACTIVATION *,ACTIVATION *);
   static ACTIVATION             *PlaceSimplicityActivation(ACTIVATION *,ACTIVATION *);
   static ACTIVATION             *PlaceRandomActivation(ACTIVATION *,ACTIVATION *);
   static struct partialMatch    *SortPartialMatch(struct partialMatch *);
   static int                     ComparePartialMatches(ACTIVATION *,ACTIVATION *);
   static char                   *GetStrategyName(int);
#endif
#else
   static ACTIVATION             *PlaceDepthActivation();
#if CONFLICT_RESOLUTION_STRATEGIES
   static ACTIVATION             *PlaceBreadthActivation();
   static ACTIVATION             *PlaceLEXActivation();
   static ACTIVATION             *PlaceMEAActivation();
   static ACTIVATION             *PlaceComplexityActivation();
   static ACTIVATION             *PlaceSimplicityActivation();
   static ACTIVATION             *PlaceRandomActivation();
   static struct partialMatch    *SortPartialMatch();
   static int                     ComparePartialMatches();
   static char                   *GetStrategyName();
#endif
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

#if CONFLICT_RESOLUTION_STRATEGIES
   static int                  Strategy = DEFAULT_STRATEGY;
#endif

/******************************************************************/
/* PlaceActivation: Coordinates placement of an activation on the */
/*   Agenda based on the current conflict resolution strategy.    */
/******************************************************************/
globle VOID PlaceActivation(whichAgenda,newActivation)
  ACTIVATION **whichAgenda;
  ACTIVATION *newActivation;
  {
   ACTIVATION *placeAfter = NULL;

   SetAgendaChanged(CLIPS_TRUE);

   /*=============================================*/
   /* Determine the location where the activation */
   /* should be placed in the agenda.             */
   /*==============================================*/

#if ! CONFLICT_RESOLUTION_STRATEGIES
   if (*whichAgenda != NULL) placeAfter = PlaceDepthActivation(*whichAgenda,newActivation);
#else
   if (*whichAgenda != NULL) switch (Strategy)
     {
      case DEPTH_STRATEGY:
        placeAfter = PlaceDepthActivation(*whichAgenda,newActivation);
        break;

      case BREADTH_STRATEGY:
        placeAfter = PlaceBreadthActivation(*whichAgenda,newActivation);
        break;

      case LEX_STRATEGY:
        placeAfter = PlaceLEXActivation(*whichAgenda,newActivation);
        break;

      case MEA_STRATEGY:
        placeAfter = PlaceMEAActivation(*whichAgenda,newActivation);
        break;

      case COMPLEXITY_STRATEGY:
        placeAfter = PlaceComplexityActivation(*whichAgenda,newActivation);
        break;

      case SIMPLICITY_STRATEGY:
        placeAfter = PlaceSimplicityActivation(*whichAgenda,newActivation);
        break;

      case RANDOM_STRATEGY:
        placeAfter = PlaceRandomActivation(*whichAgenda,newActivation);
        break;
     }
#endif

   /*==============================================================*/
   /* Place the activation at the appropriate place in the agenda. */
   /*==============================================================*/

   if (placeAfter == NULL)
     {
      newActivation->next = *whichAgenda;
      *whichAgenda = newActivation;
      if (newActivation->next != NULL) newActivation->next->prev = newActivation;
     }
   else
     {
      newActivation->next = placeAfter->next;
      newActivation->prev = placeAfter;
      placeAfter->next = newActivation;
      if (newActivation->next != NULL)
        { newActivation->next->prev = newActivation; }
     }
  }

/*******************************************************************/
/* PlaceDepthActivation: Determines the location in the agenda     */
/*    where a new activation should be placed for the depth        */
/*    strategy. Returns a pointer to the activation  after which   */
/*    the new activation should be placed (or NULL if the          */
/*    activation should be placed at the beginning of the agenda). */
/*******************************************************************/
static ACTIVATION *PlaceDepthActivation(actPtr,newActivation)
  ACTIVATION *actPtr;
  ACTIVATION *newActivation;
  {
   int salience;
   unsigned long timetag;
   ACTIVATION *lastAct;

   /*============================================*/
   /* Set up initial information for the search. */
   /*============================================*/

   salience = newActivation->salience;
   timetag = newActivation->timetag;
   lastAct = NULL;

   while (actPtr != NULL)
     {
      if (actPtr->salience > salience)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else if (actPtr->salience < salience)
        { return(lastAct); }
      else if (timetag < actPtr->timetag)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else
        { return(lastAct); }
     }

   /*===========================================*/
   /* Return the insertion point in the agenda. */
   /*===========================================*/
   
   return(lastAct);
  }

#if CONFLICT_RESOLUTION_STRATEGIES
/*******************************************************************/
/* PlaceBreadthActivation: Determines the location in the agenda   */
/*    where a new activation should be placed for the breadth      */
/*    strategy. Returns a pointer to the activation  after which   */
/*    the new activation should be placed (or NULL if the          */
/*    activation should be placed at the beginning of the agenda). */
/*******************************************************************/
static ACTIVATION *PlaceBreadthActivation(actPtr,newActivation)
  ACTIVATION *actPtr;
  ACTIVATION *newActivation;
  {
   int salience;
   unsigned long timetag;
   ACTIVATION *lastAct;

   /*============================================*/
   /* Set up initial information for the search. */
   /*============================================*/

   salience = newActivation->salience;
   timetag = newActivation->timetag;
   lastAct = NULL;

   while (actPtr != NULL)
     {
      if (actPtr->salience > salience)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else if (actPtr->salience < salience)
        { return(lastAct); }
      else if (timetag > actPtr->timetag)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else
       {  return(lastAct); }
     }

   /*===========================================*/
   /* Return the insertion point in the agenda. */
   /*===========================================*/
   
   return(lastAct);
  }

/*******************************************************************/
/* PlaceLEXActivation: Determines the location in the agenda       */
/*    where a new activation should be placed for the lex          */
/*    strategy. Returns a pointer to the activation  after which   */
/*    the new activation should be placed (or NULL if the          */
/*    activation should be placed at the beginning of the agenda). */
/*******************************************************************/
static ACTIVATION *PlaceLEXActivation(actPtr,newActivation)
  ACTIVATION *actPtr;
  ACTIVATION *newActivation;
  {
   int salience;
   unsigned long timetag;
   ACTIVATION *lastAct;
   int flag;

   /*===============================================*/
   /* Sort the fact identifiers for the activation. */
   /*===============================================*/

   if (newActivation->sortedBasis == NULL)
     { newActivation->sortedBasis = SortPartialMatch(newActivation->basis); }

   /*============================================*/
   /* Set up initial information for the search. */
   /*============================================*/

   timetag = newActivation->timetag;
   salience = newActivation->salience;
   lastAct = NULL;
   flag = CLIPS_FALSE;

   while (actPtr != NULL)
     {
      if (actPtr->salience > salience)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else if (actPtr->salience < salience)
        { return(lastAct); }
      else
        {
         flag = ComparePartialMatches(actPtr,newActivation);

         if (flag == LESS_THAN)
           {
            lastAct = actPtr;
            actPtr = actPtr->next;
           }
         else if (flag == GREATER_THAN)
           { return(lastAct); }
         else /* flag == EQUAL */
           {
            if (timetag > actPtr->timetag)
              {
               lastAct = actPtr;
               actPtr = actPtr->next;
              }
            else
             { return(lastAct); }
           }
        }
     }

   /*===========================================*/
   /* Return the insertion point in the agenda. */
   /*===========================================*/
   
   return(lastAct);
  }

/*******************************************************************/
/* PlaceMEAActivation: Determines the location in the agenda       */
/*    where a new activation should be placed for the mea          */
/*    strategy. Returns a pointer to the activation  after which   */
/*    the new activation should be placed (or NULL if the          */
/*    activation should be placed at the beginning of the agenda). */
/*******************************************************************/
static ACTIVATION *PlaceMEAActivation(actPtr,newActivation)
  ACTIVATION *actPtr;
  ACTIVATION *newActivation;
  {
   int salience;
   unsigned long timetag;
   ACTIVATION *lastAct;
   int flag;
   long int cWhoset, oWhoset;

   if (newActivation->sortedBasis == NULL)
     { newActivation->sortedBasis = SortPartialMatch(newActivation->basis); }

   /*============================================*/
   /* Set up initial information for the search. */
   /*============================================*/

   timetag = newActivation->timetag;
   salience = newActivation->salience;
   lastAct = NULL;
   flag = CLIPS_FALSE;

   while (actPtr != NULL)
     {
      if (actPtr->salience > salience)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else if (actPtr->salience < salience)
        { return(lastAct); }
      else
        {
         cWhoset = -1;
         oWhoset = -1;
         if (GetMatchingItem(newActivation,0) != NULL)
           { cWhoset = GetMatchingItem(newActivation,0)->timeTag; }
         if (GetMatchingItem(actPtr,0) != NULL)
           { oWhoset = GetMatchingItem(actPtr,0)->timeTag; }
         if (oWhoset < cWhoset)
           {
            if (cWhoset > 0) flag = GREATER_THAN;
            else flag = LESS_THAN;
           }
         else if (oWhoset > cWhoset)
           {
            if (oWhoset > 0) flag = LESS_THAN;
            else flag = GREATER_THAN;
           }
         else
           { flag = ComparePartialMatches(actPtr,newActivation); }

         if (flag == LESS_THAN)
           {
            lastAct = actPtr;
            actPtr = actPtr->next;
           }
         else if (flag == GREATER_THAN)
           { return(lastAct); }
         else /* flag == EQUAL */
           {
            if (timetag > actPtr->timetag)
              {
               lastAct = actPtr;
               actPtr = actPtr->next;
              }
            else
             { return(lastAct); }
           }
        }
     }

   /*===========================================*/
   /* Return the insertion point in the agenda. */
   /*===========================================*/
   
   return(lastAct);
  }

/*********************************************************************/
/* PlaceComplexityActivation: Determines the location in the agenda  */
/*    where a new activation should be placed for the complexity     */
/*    strategy. Returns a pointer to the activation  after which the */
/*    new activation should be placed (or NULL if the activation     */
/*    should be placed at the beginning of the agenda).              */
/*********************************************************************/
static ACTIVATION *PlaceComplexityActivation(actPtr,newActivation)
  ACTIVATION *actPtr;
  ACTIVATION *newActivation;
  {
   int salience, complexity;
   unsigned long timetag;
   ACTIVATION *lastAct;

   /*========================================*/
   /* Set up initial information for search. */
   /*========================================*/

   timetag = newActivation->timetag;
   salience = newActivation->salience;
   complexity = newActivation->theRule->complexity;
   lastAct = NULL;

   /*====================================================*/
   /* Search through the agenda for the insertion point. */
   /*====================================================*/

   while (actPtr != NULL)
     {
      if (actPtr->salience > salience)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else if (actPtr->salience < salience)
        { return(lastAct); }
      else if (complexity < actPtr->theRule->complexity)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else if (complexity > actPtr->theRule->complexity)
        { return(lastAct); }
      else if (timetag > actPtr->timetag)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else
        { return(lastAct); }
     }

   /*===========================================*/
   /* Return the insertion point in the agenda. */
   /*===========================================*/

   return(lastAct);
  }

/*********************************************************************/
/* PlaceSimplicityActivation: Determines the location in the agenda  */
/*    where a new activation should be placed for the simplicity     */
/*    strategy. Returns a pointer to the activation  after which the */
/*    new activation should be placed (or NULL if the activation     */
/*    should be placed at the beginning of the agenda).              */
/*********************************************************************/
static ACTIVATION *PlaceSimplicityActivation(actPtr,newActivation)
  ACTIVATION *actPtr;
  ACTIVATION *newActivation;
  {
   int salience, complexity;
   unsigned long timetag;
   ACTIVATION *lastAct;

   /*============================================*/
   /* Set up initial information for the search. */
   /*============================================*/

   timetag = newActivation->timetag;
   salience = newActivation->salience;
   complexity = newActivation->theRule->complexity;
   lastAct = NULL;

   while (actPtr != NULL)
     {
      if (actPtr->salience > salience)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else if (actPtr->salience < salience)
        { return(lastAct); }
      else if (complexity > actPtr->theRule->complexity)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else if (complexity < actPtr->theRule->complexity)
        { return(lastAct); }
      else if (timetag > actPtr->timetag)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else
       { return(lastAct); }
     }

   /*===========================================*/
   /* Return the insertion point in the agenda. */
   /*===========================================*/
   
   return(lastAct);
  }

/*******************************************************************/
/* PlaceRandomActivation: Determines the location in the agenda    */
/*    where a new activation should be placed for the random       */
/*    strategy. Returns a pointer to the activation  after which   */
/*    the new activation should be placed (or NULL if the          */
/*    activation should be placed at the beginning of the agenda). */
/*******************************************************************/
static ACTIVATION *PlaceRandomActivation(actPtr,newActivation)
  ACTIVATION *actPtr;
  ACTIVATION *newActivation;
  {
   int salience, randomID;
   unsigned long timetag;
   ACTIVATION *lastAct;

   /*============================================*/
   /* Set up initial information for the search. */
   /*============================================*/

   timetag = newActivation->timetag;
   salience = newActivation->salience;
   randomID = newActivation->randomID;
   lastAct = NULL;

   while (actPtr != NULL)
     {
      if (actPtr->salience > salience)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else if (actPtr->salience < salience)
        { return(lastAct); }
      else if (randomID > actPtr->randomID)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else if (randomID < actPtr->randomID)
       { return(lastAct); }
      else if (timetag > actPtr->timetag)
        {
         lastAct = actPtr;
         actPtr = actPtr->next;
        }
      else
       { return(lastAct); }
     }

   /*===========================================*/
   /* Return the insertion point in the agenda. */
   /*===========================================*/
   
   return(lastAct);
  }

/******************************************************************/
/* SortPartialMatch: Copies a partial match and then sorts the    */
/*   fact-indices in the copied partial match in ascending order. */
/******************************************************************/
static struct partialMatch *SortPartialMatch(binds)
  struct partialMatch *binds;
  {
   struct partialMatch *nbinds;
   struct alphaMatch *temp;
   int flag, j, k;

   /*=================*/
   /* Copy the array. */
   /*=================*/

   nbinds = CopyPartialMatch(binds,0,0);

   /*=================*/
   /* Sort the array. */
   /*=================*/

   flag = CLIPS_TRUE;
   k = binds->bcount - 1;
   while (flag == CLIPS_TRUE)
     {
      flag = CLIPS_FALSE;
      for (j = 0 ; j < k ; j++)
        {
         if ((nbinds->binds[j].gm.theMatch->matchingItem != NULL) &&
             (nbinds->binds[j + 1].gm.theMatch->matchingItem != NULL))
           {
            if (nbinds->binds[j].gm.theMatch->matchingItem->timeTag < 
                nbinds->binds[j + 1].gm.theMatch->matchingItem->timeTag)
              {
               temp = nbinds->binds[j].gm.theMatch;

               nbinds->binds[j].gm.theMatch = nbinds->binds[j+1].gm.theMatch;

               nbinds->binds[j+1].gm.theMatch = temp;

               flag = CLIPS_TRUE;
              }
           }
        }
      k--;
     }

   /*===================*/
   /* Return the array. */
   /*===================*/

   return(nbinds);
  }

/**************************************************************************/
/* ComparePartialMatches: Compares two activations using the lex conflict */
/*   resolution strategy to determine which activation should be placed   */
/*   first on the agenda. This lexicographic comparison function is used  */
/*   for both the lex and mea strategies.                                 */
/**************************************************************************/
static int ComparePartialMatches(actPtr,newActivation)
  ACTIVATION *actPtr, *newActivation;
  {
   int cCount, oCount, mCount, flag, i;
   int complexity;

   complexity = newActivation->theRule->complexity;
   if (actPtr->sortedBasis == NULL)
     { actPtr->sortedBasis = SortPartialMatch(actPtr->basis); }

   cCount = newActivation->sortedBasis->bcount;
   oCount = actPtr->sortedBasis->bcount;
   if (oCount > cCount) mCount = cCount;
   else mCount = oCount;

   flag = EQUAL;
   for (i = 0 ; (i < mCount) && (flag == EQUAL) ; i++)
     {
      if ((actPtr->sortedBasis->binds[i].gm.theMatch->matchingItem != NULL) &&
          (newActivation->sortedBasis->binds[i].gm.theMatch->matchingItem != NULL))
        {
         if (newActivation->sortedBasis->binds[i].gm.theMatch->matchingItem->timeTag <
             actPtr->sortedBasis->binds[i].gm.theMatch->matchingItem->timeTag)
           { flag = LESS_THAN; }
         else if (newActivation->sortedBasis->binds[i].gm.theMatch->matchingItem->timeTag >
                  actPtr->sortedBasis->binds[i].gm.theMatch->matchingItem->timeTag)
           { flag = GREATER_THAN; }
        }
      else if (newActivation->sortedBasis->binds[i].gm.theMatch->matchingItem != NULL)
        { flag = GREATER_THAN; }
      else if (actPtr->sortedBasis->binds[i].gm.theMatch->matchingItem != NULL)
        { flag = LESS_THAN; }
     }

   if (flag == EQUAL)
     {
      if (cCount < oCount) flag = LESS_THAN;
      else if (cCount > oCount) flag = GREATER_THAN;
     }

   if (flag == EQUAL)
     {
      if (complexity < actPtr->theRule->complexity) flag = LESS_THAN;
      else if (complexity > actPtr->theRule->complexity) flag = GREATER_THAN;
     }

   return(flag);
  }


/*****************************************************************/
/* SetStrategy: Sets the value of the variable Strategy and then */
/*   calls the ReorderAgenda function to update the Agenda.      */
/*****************************************************************/
globle int SetStrategy(value)
  int value;
  {
   int oldStrategy;

   oldStrategy = Strategy;
   Strategy = value;

   if (oldStrategy != Strategy) ReorderAgenda(NULL);

   return(oldStrategy);
  }

/************************************************************/
/* GetStrategy: Returns the value of the variable Strategy. */
/************************************************************/
globle int GetStrategy()
  {
   return(Strategy);
  }

/******************************************************************/
/* GetStrategyCommand: Implements the CLIPS get-strategy command. */
/******************************************************************/
globle SYMBOL_HN *GetStrategyCommand()
  {
   ArgCountCheck("get-strategy",EXACTLY,0);

   return((SYMBOL_HN *) AddSymbol(GetStrategyName(GetStrategy())));
  }

/******************************************************************/
/* SetStrategyCommand: Implements the CLIPS set-strategy command. */
/******************************************************************/
globle SYMBOL_HN *SetStrategyCommand()
  {
   DATA_OBJECT argPtr;
   char *argument;

   /*=====================================================*/
   /* Check for the correct number and type of arguments. */
   /*=====================================================*/
   
   if (ArgCountCheck("set-strategy",EXACTLY,1) == -1)
     { return((SYMBOL_HN *) AddSymbol(GetStrategyName(GetStrategy()))); }

   if (ArgTypeCheck("set-strategy",1,SYMBOL,&argPtr) == CLIPS_FALSE)
     { return((SYMBOL_HN *) AddSymbol(GetStrategyName(GetStrategy()))); }

   argument = DOToString(argPtr);

   /*=============================================*/
   /* Set the strategy to the specified strategy. */
   /*=============================================*/
   
   if (strcmp(argument,"depth") == 0)
     { SetStrategy(DEPTH_STRATEGY); }
   else if (strcmp(argument,"breadth") == 0)
     { SetStrategy(BREADTH_STRATEGY); }
   else if (strcmp(argument,"lex") == 0)
     { SetStrategy(LEX_STRATEGY); }
   else if (strcmp(argument,"mea") == 0)
     { SetStrategy(MEA_STRATEGY); }
   else if (strcmp(argument,"complexity") == 0)
     { SetStrategy(COMPLEXITY_STRATEGY); }
   else if (strcmp(argument,"simplicity") == 0)
     { SetStrategy(SIMPLICITY_STRATEGY); }
   else if (strcmp(argument,"random") == 0)
     { SetStrategy(RANDOM_STRATEGY); }
   else
     {
      ExpectedTypeError1("set-strategy",1,
      "symbol with value depth, breadth, lex, mea, complexity, simplicity, or random");
      return((SYMBOL_HN *) AddSymbol(GetStrategyName(GetStrategy())));
     }

   /*===============================================*/
   /* Return the new current value of the strategy. */
   /*===============================================*/
   
   return((SYMBOL_HN *) AddSymbol(argument));
  }
  
/**********************************************************/
/* GetStrategyName: Given the integer value corresponding */
/*   to a specified strategy, return a character string   */
/*   of the strategy's name.                              */
/**********************************************************/
static char *GetStrategyName(strategy)
  int strategy;
  {
   char *sname;

   switch (strategy)
     {
      case DEPTH_STRATEGY:
        sname = "depth";
        break;
      case BREADTH_STRATEGY:
        sname = "breadth";
        break;
      case LEX_STRATEGY:
        sname = "lex";
        break;
      case MEA_STRATEGY:
        sname = "mea";
        break;
      case COMPLEXITY_STRATEGY:
        sname = "complexity";
        break;
      case SIMPLICITY_STRATEGY:
        sname = "simplicity";
        break;
      case RANDOM_STRATEGY:
        sname = "random";
        break;
      default:
        sname = "unknown";
        break;
     }

   return(sname);
  }
#endif
  
#endif

