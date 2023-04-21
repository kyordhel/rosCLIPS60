   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*               CLIPS Version 6.00  05/12/93          */
   /*                                                     */
   /*                                                     */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Contributing Programmer(s):                               */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_insfile
#define _H_insfile

#ifndef _H_expressn
#include "expressn.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _INSFILE_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

#if (! RUN_TIME)
LOCALE VOID SetupInstanceFileCommands(void);
#endif

LOCALE long SaveInstancesCommand(void);
LOCALE long LoadInstancesCommand(void);
LOCALE long RestoreInstancesCommand(void);
LOCALE long SaveInstances(char *,int,EXPRESSION *,BOOLEAN);

#if BSAVE_INSTANCES
LOCALE long BinarySaveInstancesCommand(void);
LOCALE long BinarySaveInstances(char *,int,EXPRESSION *,BOOLEAN);
#endif

#if BLOAD_INSTANCES
LOCALE long BinaryLoadInstancesCommand(void);
LOCALE long BinaryLoadInstances(char *);
#endif

LOCALE long LoadInstances(char *);
LOCALE long RestoreInstances(char *);

#else

#if (! RUN_TIME)
LOCALE VOID SetupInstanceFileCommands();
#endif

LOCALE long SaveInstancesCommand();
LOCALE long LoadInstancesCommand();
LOCALE long RestoreInstancesCommand();
LOCALE long SaveInstances();

#if BSAVE_INSTANCES
LOCALE long BinarySaveInstancesCommand();
LOCALE long BinarySaveInstances();
#endif

#if BLOAD_INSTANCES
LOCALE long BinaryLoadInstancesCommand();
LOCALE long BinaryLoadInstances();
#endif

LOCALE long LoadInstances();
LOCALE long RestoreInstances();

#endif

#ifndef _INSFILE_SOURCE_
#endif

#endif





