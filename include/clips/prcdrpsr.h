   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00 05/12/93             */
   /*                                                     */
   /*       PROCEDURAL FUNCTIONS PARSER HEADER FILE       */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_prcdrpsr

#define _H_prcdrpsr

#ifndef _H_constrnt
#include "constrnt.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _PRCDRPSR_SOURCE
#define LOCALE
#else
#define LOCALE extern
#endif

struct BindInfo
  {
   struct symbolHashNode *name;
   CONSTRAINT_RECORD *constraints;
   struct BindInfo *next;
  };

#if ANSI_COMPILER
#if (! RUN_TIME)  
   LOCALE VOID                           ProceduralFunctionParsers(VOID);
   LOCALE struct BindInfo               *GetParsedBindNames(void);
   LOCALE VOID                           SetParsedBindNames(struct BindInfo *);
   LOCALE VOID                           ClearParsedBindNames(void);
   LOCALE BOOLEAN                        ParsedBindNamesEmpty(void);
#endif
#if (! BLOAD_ONLY) && (! RUN_TIME)  
   LOCALE int                            SearchParsedBindNames(struct symbolHashNode *);
   LOCALE int                            CountParsedBindNames(void);
   LOCALE VOID                           RemoveParsedBindName(struct symbolHashNode *);
   LOCALE struct constraintRecord       *FindBindConstraints(struct symbolHashNode *);
#endif
#else
#if (! RUN_TIME)  
   LOCALE VOID                           ProceduralFunctionParsers(); 
   LOCALE struct BindInfo               *GetParsedBindNames();
   LOCALE VOID                           SetParsedBindNames();
   LOCALE VOID                           ClearParsedBindNames();
   LOCALE BOOLEAN                        ParsedBindNamesEmpty();
#endif
#if (! BLOAD_ONLY) && (! RUN_TIME)  
   LOCALE int                            SearchParsedBindNames();
   LOCALE int                            CountParsedBindNames();
   LOCALE VOID                           RemoveParsedBindName();
   LOCALE struct constraintRecord       *FindBindConstraints();
#endif
#endif

#endif






