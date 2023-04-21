   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00 05/12/93             */
   /*                                                     */
   /*          MISCELLANEOUS FUNCTIONS HEADER FILE        */
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

#ifndef _H_miscfun

#define _H_miscfun

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _MISCFUN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           MiscFunctionDefinitions(void);
   LOCALE VOID                           CreateFunction(DATA_OBJECT_PTR);
   LOCALE long int                       SetgenFunction(void);
   LOCALE VOID                          *GensymFunction(void);
   LOCALE VOID                          *GensymStarFunction(void);
   LOCALE long                           RandomFunction(void);
   LOCALE VOID                           SeedFunction(void);
   LOCALE long int                       LengthFunction(void);
   LOCALE VOID                           ConserveMemCommand(void);
   LOCALE long int                       ReleaseMemCommand(void);
   LOCALE long int                       MemUsedCommand(void);
   LOCALE long int                       MemRequestsCommand(void);
   LOCALE VOID                           OptionsCommand(void);
   LOCALE VOID                           ExpandFuncCall(DATA_OBJECT *);
   LOCALE VOID                           DummyExpandFuncMultifield(DATA_OBJECT *);
   LOCALE SYMBOL_HN                     *CauseEvaluationError(void);
   LOCALE BOOLEAN                        SetSORCommand(void);
   LOCALE SYMBOL_HN                     *GetFunctionRestrictions(void);
   LOCALE VOID                           AproposCommand(void);
#else
   LOCALE VOID                           MiscFunctionDefinitions();
   LOCALE VOID                           CreateFunction();
   LOCALE long int                       SetgenFunction();
   LOCALE VOID                          *GensymFunction();
   LOCALE VOID                          *GensymStarFunction();
   LOCALE long                           RandomFunction();
   LOCALE VOID                           SeedFunction();
   LOCALE long int                       LengthFunction();
   LOCALE VOID                           ConserveMemCommand();
   LOCALE long int                       ReleaseMemCommand();
   LOCALE long int                       MemUsedCommand();
   LOCALE long int                       MemRequestsCommand();
   LOCALE VOID                           OptionsCommand();
   LOCALE VOID                           ExpandFuncCall();
   LOCALE VOID                           DummyExpandFuncMultifield();
   LOCALE SYMBOL_HN                     *CauseEvaluationError();
   LOCALE BOOLEAN                        SetSORCommand();
   LOCALE SYMBOL_HN                     *GetFunctionRestrictions();
   LOCALE VOID                           AproposCommand();
#endif

#endif






