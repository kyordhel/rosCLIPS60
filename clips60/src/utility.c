   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                   UTILITY MODULE                    */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*      Provides a set of utility functions useful to other  */
/*      modules.                                             */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian Donnell                                        */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#define _UTILITY_SOURCE_

#include "setup.h"

#if ANSI_COMPILER
#include <ctype.h>
#include <stdlib.h>
#endif

#include <stdio.h>
#define _CLIPS_STDIO_
#include <string.h>
#include <math.h>

#include "clipsmem.h"
#include "evaluatn.h"
#include "multifld.h"
#include "utility.h"

#define MAX_EPHEMERAL_COUNT 1000L
#define MAX_EPHEMERAL_SIZE 10240L
#define COUNT_INCREMENT 1000L
#define SIZE_INCREMENT 10240L

struct cleanupFunction
  {
   char *name;
#if ANSI_COMPILER
   VOID (*ip)(void);
#else
   VOID (*ip)();
#endif
   int priority;
   struct cleanupFunction *next;
  };

/***************************************/
/* LOCAL INTERNAL FUNCTION DEFINITIONS */
/***************************************/

#if ANSI_COMPILER
   static BOOLEAN                 AddCPFunction(char *,VOID (*)(void),int,struct cleanupFunction **);
   static BOOLEAN                 RemoveCPFunction(char *,struct cleanupFunction **);
#else
   static BOOLEAN                 AddCPFunction();
   static BOOLEAN                 RemoveCPFunction();
#endif

/***************************************/
/* LOCAL INTERNAL VARIABLE DEFINITIONS */
/***************************************/

   static struct cleanupFunction   *ListOfCleanupFunctions = NULL;
   static struct cleanupFunction   *ListOfPeriodicFunctions = NULL;

/****************************************/
/* GLOBAL INTERNAL VARIABLE DEFINITIONS */
/****************************************/

   globle unsigned long          EphemeralItemCount = 0;
   globle unsigned long          EphemeralItemSize = 0;
   globle unsigned long          CurrentEphemeralCountMax = MAX_EPHEMERAL_COUNT;
   globle unsigned long          CurrentEphemeralSizeMax = MAX_EPHEMERAL_SIZE;

/*************************************************************/
/* PeriodicCleanup: Returns garbage created during execution */
/*   that has not been returned to the memory pool yet. The  */
/*   cleanup is normally deferred so that an executing rule  */
/*   can still access these data structures.                 */
/*                                                           */
/* Always calls a series of functions that should be called  */
/*   periodically.  Usually used by interfaces to update     */
/*   displays.                                               */
/*************************************************************/
globle VOID PeriodicCleanup(cleanupAllDepths,useHeuristics)
  BOOLEAN cleanupAllDepths;
  BOOLEAN useHeuristics;
  {
   int oldDepth = -1;
   struct cleanupFunction *cleanupPtr,*periodPtr;
   static int lastEvaluationDepth = -1;

   periodPtr = ListOfPeriodicFunctions;
   while (periodPtr != NULL)
     {
      (*periodPtr->ip)();
      periodPtr = periodPtr->next;
     }

   if (lastEvaluationDepth > CurrentEvaluationDepth)
     {
      lastEvaluationDepth = CurrentEvaluationDepth;
      CurrentEphemeralCountMax = MAX_EPHEMERAL_COUNT;
      CurrentEphemeralSizeMax = MAX_EPHEMERAL_SIZE;
     }

   if (useHeuristics &&
       (EphemeralItemCount < CurrentEphemeralCountMax) &&
       (EphemeralItemSize < CurrentEphemeralSizeMax))
     { return; }

   if (cleanupAllDepths)
     {
      oldDepth = CurrentEvaluationDepth;
      CurrentEvaluationDepth = -1;
     }

   FlushMultifields();

   cleanupPtr = ListOfCleanupFunctions;
   while (cleanupPtr != NULL)
     {
      (*cleanupPtr->ip)();
      cleanupPtr = cleanupPtr->next;
     }

   RemoveEphemeralAtoms();

   if (cleanupAllDepths) CurrentEvaluationDepth = oldDepth;

   if ((EphemeralItemCount + COUNT_INCREMENT) > CurrentEphemeralCountMax)
     { CurrentEphemeralCountMax = EphemeralItemCount + COUNT_INCREMENT; }

   if ((EphemeralItemSize + SIZE_INCREMENT) > CurrentEphemeralSizeMax)
     { CurrentEphemeralSizeMax = EphemeralItemSize + SIZE_INCREMENT; }

   lastEvaluationDepth = CurrentEvaluationDepth;
  }

/*************************/
/* AddCleanupFunction:   */
/*************************/
globle BOOLEAN AddCleanupFunction(name,func_ptr,priority)
  char *name;
  VOID (*func_ptr)(VOID_ARG);
  int priority;
  {
   return(AddCPFunction(name,func_ptr,priority,&ListOfCleanupFunctions));
  }

/*************************/
/* AddPeriodicFunction:   */
/*************************/
globle BOOLEAN AddPeriodicFunction(name,func_ptr,priority)
  char *name;
  VOID (*func_ptr)(VOID_ARG);
  int priority;
  {
   return(AddCPFunction(name,func_ptr,priority,&ListOfPeriodicFunctions));
  }

/*************************/
/* AddCPFunction:   */
/*************************/
static BOOLEAN AddCPFunction(name,func_ptr,priority,head)
  char *name;
  VOID (*func_ptr)(VOID_ARG);
  int priority;
  struct cleanupFunction **head;
  {
   struct cleanupFunction *newPtr, *currentPtr, *lastPtr = NULL;

   newPtr = get_struct(cleanupFunction);

   newPtr->name = name;
   newPtr->ip = func_ptr;
   newPtr->priority = priority;

   if (*head == NULL)
     {
      newPtr->next = NULL;
      *head = newPtr;
      return(1);
     }

   currentPtr = *head;
   while ((currentPtr != NULL) ? (priority < currentPtr->priority) : CLIPS_FALSE)
     {
      lastPtr = currentPtr;
      currentPtr = currentPtr->next;
     }

   if (lastPtr == NULL)
     {
      newPtr->next = *head;
      *head = newPtr;
     }
   else
     {
      newPtr->next = currentPtr;
      lastPtr->next = newPtr;
     }

   return(1);
  }

/****************************/
/* RemoveCleanupFunction:    */
/****************************/
globle BOOLEAN RemoveCleanupFunction(name)
  char *name;
  {
   return(RemoveCPFunction(name,&ListOfCleanupFunctions));
  }

/****************************/
/* RemovePeriodicFunction:  */
/****************************/
globle BOOLEAN RemovePeriodicFunction(name)
  char *name;
  {
   return(RemoveCPFunction(name,&ListOfPeriodicFunctions));
  }

/****************************/
/* RemoveCPFunction:    */
/****************************/
static BOOLEAN RemoveCPFunction(name,head)
  char *name;
  struct cleanupFunction **head;
  {
   struct cleanupFunction *currentPtr, *lastPtr;

   lastPtr = NULL;
   currentPtr = *head;

   while (currentPtr != NULL)
     {
      if (strcmp(name,currentPtr->name) == 0)
        {
         if (lastPtr == NULL)
           { *head = currentPtr->next; }
         else
           { lastPtr->next = currentPtr->next; }
         rtn_struct(cleanupFunction,currentPtr);
         return(CLIPS_TRUE);
        }
      lastPtr = currentPtr;
      currentPtr = currentPtr->next;
     }

   return(CLIPS_FALSE);
  }

/*****************************************************/
/* StringPrintForm: Generates printed representation */
/*   of a string. Replaces / with // and " with /".  */
/*****************************************************/
globle char *StringPrintForm(str)
  char *str;
  {
   int i = 0, pos = 0, max = 0;
   char *theString = NULL;
   VOID *thePtr;

   theString = ExpandStringWithChar('"',theString,&pos,&max,max+80);
   while (str[i] != EOS)
     {
      if ((str[i] == '"') || (str[i] == '\\'))
        {
         theString = ExpandStringWithChar('\\',theString,&pos,&max,max+80);
         theString = ExpandStringWithChar(str[i],theString,&pos,&max,max+80);
        }
      else
        { theString = ExpandStringWithChar(str[i],theString,&pos,&max,max+80); }
      i++;
     }

   theString = ExpandStringWithChar('"',theString,&pos,&max,max+80);

   thePtr = AddSymbol(theString);
   rm(theString,max);
   return(ValueToString(thePtr));
  }

/***********************************************************/
/* AppendStrings: Appends two strings together. The string */
/*   created is added to the SymbolTable, so it is not     */
/*   necessary to deallocate the string returned.          */
/***********************************************************/
globle char *AppendStrings(str1,str2)
  char *str1, *str2;
  {
   int pos = 0, max = 0;
   char *theString = NULL;
   VOID *thePtr;

   theString = AppendToString(str1,theString,&pos,&max);
   theString = AppendToString(str2,theString,&pos,&max);

   thePtr = AddSymbol(theString);
   rm(theString,max);
   return(ValueToString(thePtr));
  }

/**********************************************************/
/* AppendToString:                                        */
/**********************************************************/
globle char *AppendToString(appendStr,oldStr,oldPos,oldMax)
  char *appendStr, *oldStr;
  int *oldPos, *oldMax;
  {
   int length;

   length = strlen(appendStr);
   if (length + *oldPos + 1 > *oldMax)
     {
      oldStr = genrealloc(oldStr,(unsigned) *oldMax,(unsigned) length + *oldPos + 1);
      *oldMax = length + *oldPos + 1;
     }

   if (oldStr == NULL) { return(NULL); }

   strcpy(&oldStr[*oldPos],appendStr);
   *oldPos += length;

   return(oldStr);
  }

/**********************************************************/
/* AppendNToString:                                       */
/**********************************************************/
globle char *AppendNToString(appendStr,oldStr,length,oldPos,oldMax)
  char *appendStr, *oldStr;
  int length;
  int *oldPos, *oldMax;
  {
   int lengthWithEOS;

   if (appendStr[length-1] != '\0') lengthWithEOS = length + 1;
   else lengthWithEOS = length;

   if (lengthWithEOS + *oldPos > *oldMax)
     {
      oldStr = genrealloc(oldStr,(unsigned) *oldMax,(unsigned) *oldPos + lengthWithEOS);
      *oldMax = *oldPos + lengthWithEOS;
     }

   if (oldStr == NULL) { return(NULL); }

   strncpy(&oldStr[*oldPos],appendStr,length);
   *oldPos += (lengthWithEOS - 1);
   oldStr[*oldPos] = '\0';

   return(oldStr);
  }

/**********************************************************/
/* ExpandStringWithChar:                                 */
/**********************************************************/
globle char *ExpandStringWithChar(inchar,str,pos,max,newSize)
  int inchar;
  char *str;
  int *max, *pos, newSize;
  {
   if (*pos >= (*max - 1))
     {
      str = genrealloc(str,(unsigned) *max,(unsigned) newSize);
      *max = newSize;
     }

  if (inchar != '\b')
    {
     str[*pos] = (char) inchar;
     (*pos)++;
     str[*pos] = '\0';
    }
  else
    {
     if (*pos > 0) (*pos)--;
     str[*pos] = '\0';
    }

   return(str);
  }

/*****************************************************************/
/* AddFunctionToCallList: Adds a function to a list of functions */
/*   which are called to perform certain operations (e.g. clear, */
/*   reset, and bload functions).                                */
/*****************************************************************/
globle struct callFunctionItem *AddFunctionToCallList(name,priority,func,head)
  char *name;
  int priority;
#if ANSI_COMPILER
  VOID (*func)(void);
#else
  VOID (*func)();
#endif
  struct callFunctionItem *head;
  {
   struct callFunctionItem *newPtr, *currentPtr, *lastPtr = NULL;

   newPtr = get_struct(callFunctionItem);

   newPtr->name = name;
   newPtr->func = func;
   newPtr->priority = priority;

   if (head == NULL)
     {
      newPtr->next = NULL;
      return(newPtr);
     }

   currentPtr = head;
   while ((currentPtr != NULL) ? (priority < currentPtr->priority) : CLIPS_FALSE)
     {
      lastPtr = currentPtr;
      currentPtr = currentPtr->next;
     }

   if (lastPtr == NULL)
     {
      newPtr->next = head;
      head = newPtr;
     }
   else
     {
      newPtr->next = currentPtr;
      lastPtr->next = newPtr;
     }
     
   return(head);
  }
  
/*****************************************************************/
/* RemoveFunctionFromCallList: Removes a function from a list of */
/*   functions which are called to perform certain operations    */
/*   (e.g. clear, reset, and bload functions).                   */
/*****************************************************************/
globle struct callFunctionItem *RemoveFunctionFromCallList(name,head,found)
  char *name;
  struct callFunctionItem *head;
  int *found;
  {
   struct callFunctionItem *currentPtr, *lastPtr;

   *found = CLIPS_FALSE;
   lastPtr = NULL;
   currentPtr = head;

   while (currentPtr != NULL)
     {
      if (strcmp(name,currentPtr->name) == 0)
        {
         *found = CLIPS_TRUE;
         if (lastPtr == NULL)
           { head = currentPtr->next; }
         else
           { lastPtr->next = currentPtr->next; }
           
         rtn_struct(callFunctionItem,currentPtr);
         return(head);
        }
        
      lastPtr = currentPtr;
      currentPtr = currentPtr->next;
     }

   return(head);
  }
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               