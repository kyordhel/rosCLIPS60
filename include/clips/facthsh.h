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

#ifndef _H_facthsh

#define _H_facthsh

struct factHashEntry;

#ifndef _H_factmngr
#include "factmngr.h"
#endif

struct factHashEntry 
  {
   struct fact *theFact;
   struct factHashEntry *next;
  };

#define SIZE_FACT_HASH  1013

#ifdef LOCALE
#undef LOCALE
#endif
#ifdef _FACTHSH_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           AddHashedFact(struct fact *,int);
   LOCALE BOOLEAN                        RemoveHashedFact(struct fact *);
   LOCALE int                            HandleFactDuplication(VOID *);
   LOCALE BOOLEAN                        GetFactDuplication(void);
   LOCALE BOOLEAN                        SetFactDuplication(int);  
   LOCALE VOID                           InitializeFactHashTable(void);
   LOCALE VOID                           ShowFactHashTable(void);
#else 
   LOCALE VOID                           AddHashedFact();
   LOCALE BOOLEAN                        RemoveHashedFact();
   LOCALE int                            HandleFactDuplication();
   LOCALE BOOLEAN                        GetFactDuplication();
   LOCALE BOOLEAN                        SetFactDuplication();
   LOCALE VOID                           InitializeFactHashTable(); 
   LOCALE VOID                           ShowFactHashTable();
#endif


#endif





