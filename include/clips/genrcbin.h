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

#ifndef _H_genrcbin
#define _H_genrcbin

#include "genrcfun.h"

#define GenericPointer(i) (((i) == -1L) ? NULL : (DEFGENERIC *) &defgenericArray[i])

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _GENRCBIN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE VOID SetupGenericsBload(void);
LOCALE VOID *BloadDefgenericModuleReference(int);

#else

LOCALE VOID SetupGenericsBload();
LOCALE VOID *BloadDefgenericModuleReference();

#endif

#ifndef _GENRCBIN_SOURCE_
extern DEFGENERIC HUGE_ADDR *defgenericArray;
#endif

#endif





