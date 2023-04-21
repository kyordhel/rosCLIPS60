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

#ifndef _H_clsltpsr
#define _H_clsltpsr

#if OBJECT_SYSTEM && (! BLOAD_ONLY) && (! RUN_TIME)

#define MATCH_RLN            "pattern-match"
#define REACTIVE_RLN         "reactive"
#define NONREACTIVE_RLN      "non-reactive"

#ifndef _H_object
#include "object.h"
#endif

typedef struct tempSlotLink
  {
   SLOT_DESC *desc;
   struct tempSlotLink *nxt;
  } TEMP_SLOT_LINK;

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _CLSLTPSR_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER

LOCALE TEMP_SLOT_LINK *ParseSlot(char *,TEMP_SLOT_LINK *,PACKED_CLASS_LINKS *,int,int);
LOCALE VOID DeleteSlots(TEMP_SLOT_LINK *);

#else

LOCALE TEMP_SLOT_LINK *ParseSlot();
LOCALE VOID DeleteSlots();

#endif

#ifndef _CLSLTPSR_SOURCE_
#endif

#endif

#endif





