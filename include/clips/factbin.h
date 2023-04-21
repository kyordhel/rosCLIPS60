   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*            FACT BLOAD/BSAVE HEADER FILE             */
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

#ifndef _H_factbin

#define _H_factbin

#include "factbld.h"

#ifdef LOCALE
#undef LOCALE
#endif
  
#ifdef _FACTBIN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER
   LOCALE VOID                           FactBinarySetup(void);
#else
   LOCALE VOID                           FactBinarySetup();
#endif

#ifndef _FACTBIN_SOURCE_
   extern struct factPatternNode HUGE_ADDR  *FactPatternArray;
#endif

#define BsaveFactPatternIndex(patPtr) ((patPtr == NULL) ? -1L : ((struct factPatternNode *) patPtr)->bsaveID)
#define BloadFactPatternPointer(i) ((struct factPatternNode *) ((i == -1L) ? NULL : &FactPatternArray[i]))

#endif

