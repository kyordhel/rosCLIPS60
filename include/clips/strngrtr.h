   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            STRING I/O ROUTER HEADER FILE            */
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

#ifndef _H_strngrtr
#define _H_strngrtr

#ifndef _CLIPS_STDIO_
#define _CLIPS_STDIO_
#include <stdio.h>
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _STRNGRTR_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

/**************************/
/* I/O ROUTER DEFINITIONS */
/**************************/

#if ANSI_COMPILER
   LOCALE VOID                           InitializeStringRouter(void);
   LOCALE int                            OpenStringSource(char *,char *,int);
   LOCALE int                            OpenTextSource(char *,char *,int,int);
   LOCALE int                            CloseStringSource(char *);
   LOCALE int                            OpenStringDestination(char *,char *,int);
   LOCALE int                            CloseStringDestination(char *);
   LOCALE VOID                           InitializeStringRouter(void);
#else
   LOCALE VOID                           InitializeStringRouter();
   LOCALE int                            OpenStringSource();
   LOCALE int                            OpenTextSource();
   LOCALE int                            CloseStringSource();
   LOCALE int                            OpenStringDestination();
   LOCALE int                            CloseStringDestination();
   LOCALE VOID                           InitializeStringRouter();
#endif 

#endif






