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

#ifndef _H_objbin
#define _H_objbin

#define DefclassPointer(i) (((i) == -1L) ? NULL : (DEFCLASS *) &defclassArray[i])
#define DefclassIndex(cls) (((cls) == NULL) ? -1 : ((struct constructHeader *) cls)->bsaveID)

#ifndef _H_object
#include "object.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _OBJBIN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE VOID SetupObjectsBload(void);
LOCALE VOID *BloadDefclassModuleReference(int);

#else

LOCALE VOID SetupObjectsBload();
LOCALE VOID *BloadDefclassModuleReference();

#endif

#ifndef _OBJBIN_SOURCE_
extern DEFCLASS HUGE_ADDR *defclassArray;
#endif

#endif





