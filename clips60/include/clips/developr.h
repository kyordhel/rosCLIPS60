   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*                 DEVELOPER HEADER FILE               */
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

#ifndef _H_developr
#define _H_developr

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _DEVELOPR_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           DeveloperCommands(void);
   LOCALE VOID                           PrimitiveTablesInfo(void);
   
#if DEFRULE_CONSTRUCT && DEFTEMPLATE_CONSTRUCT
   LOCALE VOID                           ShowFactPatternNetwork(void);
#endif
#if INSTANCE_PATTERN_MATCHING
   LOCALE VOID                           PrintObjectPatternNetwork(void);
#endif

#else 
   LOCALE VOID                           DeveloperCommands();
   LOCALE VOID                           PrimitiveTablesInfo();
   
#if DEFRULE_CONSTRUCT &&DEFTEMPLATE_CONSTRUCT
   LOCALE VOID                           ShowFactPatternNetwork();
#endif
#if INSTANCE_PATTERN_MATCHING
   LOCALE VOID                           PrintObjectPatternNetwork();
#endif

#endif

#endif




