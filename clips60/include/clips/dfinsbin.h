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

#ifndef _H_dfinsbin
#define _H_dfinsbin

#if DEFINSTANCES_CONSTRUCT && (BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE)

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _DFINSBIN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE VOID SetupDefinstancesBload(void);
LOCALE VOID *BloadDefinstancesModuleRef(int);

#else

LOCALE VOID SetupDefinstancesBload();
LOCALE VOID *BloadDefinstancesModuleRef();

#endif

#ifndef _DFINSBIN_SOURCE_
#endif

#endif

#endif




