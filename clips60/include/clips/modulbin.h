   /*******************************************************/
   /*      "C" Language Integrated Production System      */
   /*                                                     */
   /*                  A Product Of The                   */
   /*             Software Technology Branch              */
   /*             NASA - Johnson Space Center             */
   /*                                                     */
   /*             CLIPS Version 6.00  05/12/93            */
   /*                                                     */
   /*           DEFMODULE BSAVE/BLOAD HEADER FILE         */
   /*******************************************************/

/*************************************************************/
/* Purpose:                                                  */
/*                                                           */
/* Principal Programmer(s):                                  */
/*      Gary D. Riley                                        */
/*                                                           */
/* Contributing Programmer(s):                               */
/*      Brian L. Donnell                                     */
/*                                                           */
/* Revision History:                                         */
/*                                                           */
/*************************************************************/

#ifndef _H_modulbin

#define _H_modulbin
  
#ifndef _H_moduldef
#include "moduldef.h"
#endif

struct bsaveDefmodule
  {
   long name;
   long importList;
   long exportList;
   long next;
   long bsaveID; 
  }; 

struct bsaveDefmoduleItemHeader
  {
   long theModule;
   long firstItem;
   long lastItem;
  };
  
struct bsavePortItem
  {
   long moduleName;
   long constructType;
   long constructName;
   long next; 
  };

#define ModulePointer(i) ((struct defmodule *) (&DefmoduleArray[i]))

#ifdef LOCALE
#undef LOCALE
#endif

#ifdef _MODULBIN_SOURCE_
#define LOCALE
#else
#define LOCALE extern
#endif

#if ANSI_COMPILER   
   LOCALE VOID                           DefmoduleBinarySetup(void);
   LOCALE VOID                           UpdateDefmoduleItemHeader
                                                 (struct bsaveDefmoduleItemHeader *,
                                                  struct defmoduleItemHeader *,int,VOID *);
   
#if BLOAD_AND_BSAVE
   LOCALE VOID                           AssignBsaveDefmdlItemHdrVals
                                                 (struct bsaveDefmoduleItemHeader *,
                                                  struct defmoduleItemHeader *);
#endif

#else
   LOCALE VOID                           DefmoduleBinarySetup();
   LOCALE VOID                           UpdateDefmoduleItemHeader();

#if BLOAD_AND_BSAVE
   LOCALE VOID                           AssignBsaveDefmdlItemHdrVals();
#endif

#endif

#ifndef _MODULBIN_SOURCE_
   extern struct defmodule HUGE_ADDR            *DefmoduleArray;
#endif

#endif







