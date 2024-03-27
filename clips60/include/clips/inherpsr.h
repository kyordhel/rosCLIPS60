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

#ifndef _H_inherpsr
#define _H_inherpsr

#if OBJECT_SYSTEM && (! BLOAD_ONLY) && (! RUN_TIME)

#ifndef _H_object
#include "object.h"
#endif

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _INHERPSR_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE PACKED_CLASS_LINKS *ParseSuperclasses(char *,SYMBOL_HN *);
LOCALE PACKED_CLASS_LINKS *FindPrecedenceList(DEFCLASS *,PACKED_CLASS_LINKS *);
LOCALE VOID PackClassLinks(PACKED_CLASS_LINKS *,CLASS_LINK *);

#else

LOCALE PACKED_CLASS_LINKS *ParseSuperclasses();
LOCALE PACKED_CLASS_LINKS *FindPrecedenceList();
LOCALE VOID PackClassLinks();

#endif

#ifndef _INHERPSR_SOURCE_
#endif

#endif

#endif





