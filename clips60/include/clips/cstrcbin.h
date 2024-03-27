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

#ifndef _H_cstrcbin
#define _H_cstrcbin

#if BLOAD || BLOAD_ONLY || BLOAD_AND_BSAVE

struct bsaveConstructHeader
  {
   long name;
   long whichModule;
   long next;
  };

#ifndef _H_constrct
#include "constrct.h"
#endif
 
#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CSTRCBIN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

#if BLOAD_AND_BSAVE
LOCALE VOID MarkConstructHeaderNeededItems(struct constructHeader *,long);
LOCALE VOID AssignBsaveConstructHeaderVals(struct bsaveConstructHeader *,
                                             struct constructHeader *);
#endif

LOCALE VOID UpdateConstructHeader(struct bsaveConstructHeader *,
                                  struct constructHeader *,int,VOID *,int,VOID *);
LOCALE VOID UnmarkConstructHeader(struct constructHeader *);

#else

#if BLOAD_AND_BSAVE
LOCALE VOID MarkConstructHeaderNeededItems();
LOCALE VOID AssignBsaveConstructHeaderVals();
#endif

LOCALE VOID UpdateConstructHeader();
LOCALE VOID UnmarkConstructHeader();

#endif

#ifndef _CSTRCBIN_SOURCE_
#endif

#endif

#endif




