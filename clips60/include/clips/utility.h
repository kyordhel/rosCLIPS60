   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 UTILITY HEADER FILE                 */
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

#ifndef _H_utility
#define _H_utility

#ifdef LOCALE
#undef LOCALE
#endif

struct callFunctionItem
  {
   char *name;
   VOID (*func)(VOID_ARG);
   int priority;
   struct callFunctionItem *next;
  };
  
#ifdef _UTILITY_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER  
   LOCALE VOID                           PeriodicCleanup(BOOLEAN,BOOLEAN);
   LOCALE BOOLEAN                        AddCleanupFunction(char *,VOID (*)(void),int);
   LOCALE BOOLEAN                        AddPeriodicFunction(char *,VOID (*)(void),int);
   LOCALE BOOLEAN                        RemoveCleanupFunction(char *);
   LOCALE BOOLEAN                        RemovePeriodicFunction(char *);
   LOCALE char                          *AppendStrings(char *,char *);
   LOCALE char                          *StringPrintForm(char *);
   LOCALE char                          *AppendToString(char *,char *,int *,int *);
   LOCALE char                          *AppendNToString(char *,char *,int,int *,int *);
   LOCALE char                          *ExpandStringWithChar(int,char *,int *,int *,int);
   LOCALE struct callFunctionItem       *AddFunctionToCallList(char *,int,VOID (*)(void),
                                                               struct callFunctionItem *);
   LOCALE struct callFunctionItem       *RemoveFunctionFromCallList(char *,
                                                             struct callFunctionItem *,
                                                             int *);
#else 
   LOCALE VOID                           PeriodicCleanup();
   LOCALE BOOLEAN                        AddCleanupFunction();
   LOCALE BOOLEAN                        AddPeriodicFunction();
   LOCALE BOOLEAN                        RemoveCleanupFunction();
   LOCALE BOOLEAN                        RemovePeriodicFunction();
   LOCALE char                          *AppendStrings();
   LOCALE char                          *StringPrintForm();
   LOCALE char                          *AppendToString();
   LOCALE char                          *AppendNToString();
   LOCALE char                          *ExpandStringWithChar();
   LOCALE struct callFunctionItem       *AddFunctionToCallList();
   LOCALE struct callFunctionItem       *RemoveFunctionFromCallList();
#endif 

#ifndef _UTILITY_SOURCE_
   extern unsigned long               EphemeralItemCount;
   extern unsigned long               EphemeralItemSize;
#endif

#endif




