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

#ifndef _H_dffnxbin
#define _H_dffnxbin

#if DEFFUNCTION_CONSTRUCT && (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE)

#include "dffnxfun.h"

#define DeffunctionPointer(i) (((i) == -1L) ? NULL : (DEFFUNCTION *) &deffunctionArray[i])

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _DFFNXBIN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE VOID SetupDeffunctionsBload(void);
LOCALE VOID *BloadDeffunctionModuleReference(int);

#else

LOCALE VOID SetupDeffunctionsBload();
LOCALE VOID *BloadDeffunctionModuleReference();

#endif

#ifndef _DFFNXBIN_SOURCE_
extern DEFFUNCTION HUGE_ADDR *deffunctionArray;
#endif

#endif

#endif




