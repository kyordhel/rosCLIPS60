   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00 05/12/93             */
   /*                                                     */
   /*          PROCEDURAL FUNCTIONS HEADER FILE           */
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

#ifndef _H_prcdrfun

#define _H_prcdrfun

#ifndef _H_evaluatn
#include "evaluatn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _PRCDRFUN_SOURCE
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           ProceduralFunctionDefinitions(VOID);
   LOCALE VOID                           WhileFunction(DATA_OBJECT_PTR);
   LOCALE VOID                           LoopForCountFunction(DATA_OBJECT_PTR);
   LOCALE long                           GetLoopCount(void);
   LOCALE VOID                           IfFunction(DATA_OBJECT_PTR);
   LOCALE VOID                           BindFunction(DATA_OBJECT_PTR);
   LOCALE VOID                           PrognFunction(DATA_OBJECT_PTR);
   LOCALE VOID                           ReturnFunction(DATA_OBJECT_PTR);
   LOCALE VOID                           BreakFunction(void);
   LOCALE VOID                           SwitchFunction(DATA_OBJECT_PTR);
   LOCALE BOOLEAN                        GetBoundVariable(struct dataObject *,struct symbolHashNode *);
   LOCALE VOID                           FlushBindList(void);
#else
   LOCALE VOID                           ProceduralFunctionDefinitions();
   LOCALE VOID                           WhileFunction();
   LOCALE VOID                           LoopForCountFunction();
   LOCALE long                           GetLoopCount();
   LOCALE VOID                           IfFunction();
   LOCALE VOID                           BindFunction();
   LOCALE VOID                           PrognFunction();
   LOCALE VOID                           ReturnFunction();
   LOCALE VOID                           BreakFunction();
   LOCALE VOID                           SwitchFunction();
   LOCALE BOOLEAN                        GetBoundVariable();
   LOCALE VOID                           FlushBindList();
#endif

#ifndef _PRCDRFUN_SOURCE
   extern int                            ReturnFlag;
   extern int                            BreakFlag;
#endif

#endif






