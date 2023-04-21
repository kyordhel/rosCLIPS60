   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*      DEFTEMPLATE CONSTRUCT COMPILER HEADER FILE     */
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

#ifndef _H_tmpltcmp

#define _H_tmpltcmp

#ifdef LOCALE
#undef LOCALE
#endif
  
#ifdef _TMPLTCMP_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           DeftemplateCompilerSetup(void);
   LOCALE VOID                           DeftemplateCModuleReference(FILE *,int,int,int);
   LOCALE VOID                           DeftemplateCConstructReference(FILE *,VOID *,int,int);
#else
   LOCALE VOID                           DeftemplateCompilerSetup();
   LOCALE VOID                           DeftemplateCModuleReference();
   LOCALE VOID                           DeftemplateCConstructReference();
#endif

#endif
